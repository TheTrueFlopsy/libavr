
#include <avr/io.h>

#include "auto_watchdog.h"
//#include "watchdog.h"

#include "task_sched.h"
#include "tbouncer.h"
#include "std_tlv.h"

// ---<<< Test Procedures >>>---
// --- Test Case 1: Verify software reset and disable-on-startup ---
/*
	1: Power up the circuit.
		! LEDs should switch back and forth a few times, LED 1 (green) should then be on steady.
	2: Push button 0 (LED 1 toggle).
		! LED 1 should turn off.
	3: Push button 1 (software reset).
		! Behavior should be the same as at power-on.
	4: Push hardware reset button.
		! Behavior should be the same as at power-on.
*/

// --- Test Case 2: Verify behavior without disable-on-startup ---
/*
	1: Comment out the include of "auto_watchdog.h", uncomment include of "watchdog.h",
	   recompile and reupload the firmware. (Alternative: Define WATCHDOG_DO_NOT_AUTODISABLE
	   in the makefile.)
	2: Power up the circuit.
		! LEDs should switch back and forth a few times, LED 1 (green) should then be on steady.
	3: Push button 0 (LED 1 toggle).
		! LED 1 should turn off.
	4: Push button 1 (software reset).
		! LED 0 (red) should flash rapidly.
	5: Push hardware reset button.
		! Behavior should be the same as at power-on.
*/


// ---<<< Constant Definitions >>>---
#define FWID_L 0x03
#define FWID_H 0x01
#define FWVERSION 0x01

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PINR PIND
#define LED_PIN0 4
#define LED_PIN1 2

#define LED_INPUT_PINR PINB
#define LED_INPUT_PORT PORTB
#define LED_INPUT_PIN0 1
#define LED_INPUT_PIN1 0

#define LED_INPUT_RISING TBOUNCER_B_RISING

#define TBOUNCER_TASK_CAT 15
#define TTLV_TASK_CAT 14
#define TEST_TASK_CAT 0
#define MESSENGER_TASK_CAT 1

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define N_LEDS 2
#define MAX_LED_TOGGLE 11

#define INM_ADDR 0x01
#define MAX_MSG_LEN 8

static const uint8_t led_pins[N_LEDS] = {
	BV(LED_PIN0),
	BV(LED_PIN1)
};

static const uint8_t led_input_pins[N_LEDS] = {
	BV(LED_INPUT_PIN0),
	BV(LED_INPUT_PIN1)
};

static const sched_time led_toggle_delay = SCHED_TIME_MS(200);


// ---<<< Program State >>>---
static uint8_t led_states[N_LEDS] = { 1, 0 };

static sched_time led_toggle_timestamp = SCHED_TIME_ZERO;

static uint8_t led_toggle_counter = 0;

static union {
	uint8_t b[MAX_MSG_LEN];
	ttlv_msg_inm_reg r;
	ttlv_msg_inm_regpair rp;
} msg_data_out;

static union {
	uint8_t b[MAX_MSG_LEN];
	ttlv_msg_reg r;
} msg_data_in;


// ---<<< Helper Functions >>>---
static void update_led_state(uint8_t led_num, uint8_t led_flag) {
	uint8_t led_state = led_states[led_num];
	
	if (led_flag && !led_state) {  // Turn on LED.
		led_states[led_num] = 1;
		LED_PORT |= led_pins[led_num];
	}
	else if (!led_flag && led_state) {  // Turn off LED.
		led_states[led_num] = 0;
		LED_PORT &= ~led_pins[led_num];
	}
}

static uint8_t get_led_flags(void) {
	uint8_t led_flags = 0;
	
	for (uint8_t i = 0; i < N_LEDS; i++)
		if (led_states[i])
			led_flags |= BV(i);
	
	return led_flags;
}

static void set_led_flags(uint8_t led_flags) {
	for (uint8_t i = 0; i < N_LEDS; i++) {
		update_led_state(i, 1 & led_flags);
		led_flags >>= 1;
	}
}

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED states as bits (1 if enabled, otherwise 0).
		*value_p = get_led_flags();
		break;
	case TTLV_REG_DEBUG1:  // State of LED 0.
		*value_p = led_states[0];
		break;
	case TTLV_REG_DEBUG2:  // State of LED 1.
		*value_p = led_states[1];
		break;
	case TTLV_REG_FWID_L:
		*value_p = FWID_L;
		break;
	case TTLV_REG_FWID_H:
		*value_p = FWID_H;
		break;
	case TTLV_REG_FWVERSION:
		*value_p = FWVERSION;
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result ttlv_reg_write(ttlv_reg_index index, ttlv_reg_value value) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED states as bits (1 if enabled, otherwise 0).
		set_led_flags(value);
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

TTLV_STD_REG_TOGGLE(ttlv_reg_read, ttlv_reg_write, ttlv_reg_toggle)

TTLV_STD_REG_RW_EXCH(ttlv_reg_read, ttlv_reg_write, ttlv_reg_rw_exch)

TTLV_STD_REG_WR_EXCH(ttlv_reg_read, ttlv_reg_write, ttlv_reg_wr_exch)

TTLV_STD_REGPAIR_READ(ttlv_reg_read, ttlv_regpair_read)


// ---<<< Task Handlers >>>---
static void test_handler(sched_task *task) {
	// Detect and handle input events.
	if (LED_INPUT_RISING & led_input_pins[0])  // Button 0 pushed.
		update_led_state(1, !led_states[1]);  // Toggle LED 1.
	
	if (LED_INPUT_RISING & led_input_pins[1]) {  // Button 1 pushed.
		update_led_state(0, 1);  // Turn on both LEDs.
		update_led_state(1, 1);
		watchdog_reset_mcu();    // Trigger an MCU reset via the watchdog timer.
	}
	
	if (led_toggle_counter < MAX_LED_TOGGLE) {  // Still toggling the LEDs?
		// Check whether the LED toggle delay has expired.
		sched_time delay_elapsed = sched_time_sub(sched_ticks, led_toggle_timestamp);
		
		if (sched_time_gte(delay_elapsed, led_toggle_delay)) {
			update_led_state(0, !led_states[0]);   // Toggle LEDs.
			update_led_state(1, !led_states[1]);
			led_toggle_counter++;                  // Increment toggle counter.
			led_toggle_timestamp = sched_ticks;    // Remember start time of delay.
			task->delay = led_toggle_delay;        // Reset delay.
		}
		else
			task->delay = sched_time_sub(led_toggle_delay, delay_elapsed);  // Resume delay.
	}
	else
		task->st |= TASK_SLEEP_BIT;  // Set sleep flag.
}

static void message_handler(sched_task *task) {
	ttlv_result res = TTLV_RES_NONE;
	
	if (!TTLV_HAS_MESSAGE) {
		task->st |= TASK_SLEEP_BIT;  // Set sleep flag.
		return;
	}
	
	if (TTLV_CHECK_REG_READ) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r.index = msg_data_in.b[0];
		msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
		
		res = ttlv_reg_read(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_WRITE) {
		ttlv_recv(msg_data_in.b);
		
		res = ttlv_reg_write(msg_data_in.r.index, msg_data_in.r.value);
	}
	else if (TTLV_CHECK_REG_TOGGLE) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r.index = msg_data_in.r.index;
		msg_data_out.r.value = msg_data_in.r.value;
		msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
		
		res = ttlv_reg_toggle(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_RW_EXCH) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r.index = msg_data_in.r.index;
		msg_data_out.r.value = msg_data_in.r.value;
		msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
		
		res = ttlv_reg_rw_exch(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_WR_EXCH) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r.index = msg_data_in.r.index;
		msg_data_out.r.value = msg_data_in.r.value;
		msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
		
		res = ttlv_reg_wr_exch(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REGPAIR_READ) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.rp.index = msg_data_in.b[0];
		msg_data_out.rp.request_id = ttlv_recv_inm_header.h.msg_id;
		
		res = ttlv_regpair_read(msg_data_out.rp.index, &msg_data_out.rp.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_INM_REGPAIR_READ_RES, TTLV_MSG_L_INM_REGPAIR_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else {  // Unrecognized message.
		// NOTE: Probably not a good idea to always send error messages in
		//       response to unrecognized messages. Doing so can easily
		//       trigger self-sustaining message cascades.
		ttlv_finish_recv();
	}
	
	if (res != TTLV_RES_NONE)
		ttlv_xmit_inm_result(res);
}


// ---<<< Initialization Routines and Main Function >>>---
static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, TEST_TASK_CAT, 0),
		.delay = led_toggle_delay,
		.handler = test_handler
	};
	sched_add(&task);
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, MESSENGER_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = message_handler
	};
	sched_add(&task);
}

// Ensure that the pesky watchdog timer is disabled.
//WATCHDOG_DISABLE_ON_MCU_RESET

//WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS

/*void watchdog_disable_on_mcu_reset(void)
	__attribute__ ((naked))
	__attribute__ ((used))
	__attribute__ ((section (".init3")));
void watchdog_disable_on_mcu_reset(void) {
	MCUSR = 0;  // Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.)
	wdt_disable();  // Disable the watchdog timer.
}*/

// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// Ensure that the pesky watchdog timer is disabled.
	//watchdog_disable_on_mcu_reset();
	
	// Set LED pins as outputs. Turn on LED 0.
	LED_DDR |= BV(LED_PIN0) | BV(LED_PIN1);
	LED_PORT |= BV(LED_PIN0);
	
	// Enable pull-ups on input pins.
	LED_INPUT_PORT |= BV(LED_INPUT_PIN0) | BV(LED_INPUT_PIN1);
	
	sched_init();
	
	TBOUNCER_INIT(
		TASK_ST_MAKE(0, TBOUNCER_TASK_CAT, 0), SCHED_TIME_MS(4),
		BV(LED_INPUT_PIN0) | BV(LED_INPUT_PIN1), 0, 0,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(TEST_TASK_CAT));
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(MESSENGER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = INM_ADDR;  // Set INM source address.
	
	init_tasks();
	
	// Speed up the CPU clock.
	// CAUTION: MUST be done with interrupts disabled.
	//CLKPR = BV(CLKPCE);  // Begin clock prescaler update.
	//CLKPR = 0;  // No prescaling, full steam ahead.
	
	sched_run();
	
	return 0;
}
