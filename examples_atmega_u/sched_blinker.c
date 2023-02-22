
#include <avr/io.h>

#include "task_sched.h"
#include "tbouncer.h"
#include "std_tlv.h"

#define BLINK_DDR DDRD
#define BLINK_PORT PORTD
#define BLINK_PINR PIND
#define BLINK_PIN0 0  // D3 on the Leonardo
#define BLINK_PIN1 1  // D2 on the Leonardo

#define BLINK_INPUT_PINR PINB
#define BLINK_INPUT_PORT PORTB
#define BLINK_INPUT_PIN0 5  // D9 on the Leonardo
#define BLINK_INPUT_PIN1 4  // D8 on the Leonardo

#define BLINK_INPUT_RISING TBOUNCER_B_RISING

#define TBOUNCER_TASK_CAT 15
#define TTLV_TASK_CAT 14
#define BLINKER_TASK_CAT 0
#define MESSENGER_TASK_CAT 1

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define INM_ADDR 0x02
#define N_BLINKERS 2

#define BLINK_STATE_MSG_TYPE (TTLV_MSG_T_APPLICATION + 0x01)
#define BLINK_SET_STATE_MSG_TYPE (TTLV_MSG_T_APPLICATION + 0x02)

#define BLINK_RESPONSE_MSG_TYPE (TTLV_MSG_T_APPLICATION + 0x03)

static const uint8_t blink_pins[N_BLINKERS] = {
	BV(BLINK_PIN0),
	BV(BLINK_PIN1)
};

static const uint8_t blink_input_pins[N_BLINKERS] = {
	BV(BLINK_INPUT_PIN0),
	BV(BLINK_INPUT_PIN1)
};

static const sched_time blink_delays[N_BLINKERS] = {
	SCHED_TIME_MS(450),
	SCHED_TIME_MS(200)
};

static sched_time blink_timestamps[N_BLINKERS] = {
	SCHED_TIME_ZERO,
	SCHED_TIME_ZERO
};

enum {
	BLINK_STATE_START    = 0,
	BLINK_STATE_ENABLED  = 1,
	BLINK_STATE_STOP     = 2,
	BLINK_STATE_DISABLED = 3
};

static uint8_t blink_states[N_BLINKERS] = {
	BLINK_STATE_START,
	BLINK_STATE_START
};

static void update_blink_state(uint8_t blinker_num, uint8_t blink_flag) {
	uint8_t blink_state = blink_states[blinker_num];
	
	if (blink_flag) {  // Enable blinking.
		if (blink_state == BLINK_STATE_DISABLED || blink_state == BLINK_STATE_STOP) {
			blink_states[blinker_num] = BLINK_STATE_START;
			sched_task_tcww |= SCHED_CATFLAG(BLINKER_TASK_CAT);  // Notify blinker tasks.
		}
	}
	else {  // Disable blinking.
		if (blink_state == BLINK_STATE_ENABLED || blink_state == BLINK_STATE_START) {
			blink_states[blinker_num] = BLINK_STATE_STOP;
			sched_task_tcww |= SCHED_CATFLAG(BLINKER_TASK_CAT);  // Notify blinker tasks.
		}
	}
}

static void blink_handler(sched_task *task) {
	uint8_t num = TASK_ST_GET_NUM(task->st);
	uint8_t blink_state = blink_states[num];
	sched_time delay_elapsed;
	
	if (BLINK_INPUT_RISING & blink_input_pins[num]) {  // Detect input events.
		if (blink_state == BLINK_STATE_ENABLED || blink_state == BLINK_STATE_START)
			blink_state = BLINK_STATE_STOP;  // Disable blinking.
		else
			blink_state = BLINK_STATE_START;  // Enable blinking.
	}
	
	switch (blink_state) {  // Pretty blink state machine
	case BLINK_STATE_START:  // Enable blinking.
		blink_states[num] = BLINK_STATE_ENABLED;
		BLINK_PORT |= blink_pins[num];  // Turn on LED.
		blink_timestamps[num] = sched_ticks;  // Remember start time of delay.
		task->delay = blink_delays[num];  // Reset delay.
		
		// Send broadcast message about updated blink state.
		ttlv_xmit(TTLV_BROADCAST_ADR, BLINK_STATE_MSG_TYPE, N_BLINKERS, blink_states);
		
		break;
	case BLINK_STATE_ENABLED:  // Blinking enabled.
		// Check whether the LED toggle delay has expired.
		delay_elapsed = sched_time_sub(sched_ticks, blink_timestamps[num]);
		
		if (sched_time_gte(delay_elapsed, blink_delays[num])) {
			BLINK_PINR = blink_pins[num];  // Toggle LED by writing 1 to PIN register.
			blink_timestamps[num] = sched_ticks;  // Remember start time of delay.
			task->delay = blink_delays[num];  // Reset delay.
		}
		else
			task->delay = sched_time_sub(blink_delays[num], delay_elapsed);  // Resume delay.
		
		break;
	case BLINK_STATE_STOP:  // Disable blinking.
		blink_states[num] = BLINK_STATE_DISABLED;
		BLINK_PORT &= ~blink_pins[num];  // Turn off LED.
		
		// Send broadcast message about updated blink state.
		ttlv_xmit(TTLV_BROADCAST_ADR, BLINK_STATE_MSG_TYPE, N_BLINKERS, blink_states);
		
		// fallthrough
	case BLINK_STATE_DISABLED:  // Blinking disabled.
	default:
		task->st |= TASK_SLEEP_BIT;  // Set sleep flag.
		break;
	}
}

static void message_handler(sched_task *task) {
	if (TTLV_HAS_MESSAGE) {
		if (TTLV_CHECK_TL(BLINK_SET_STATE_MSG_TYPE, N_BLINKERS)) {  // Set blink state.
			uint8_t blink_flags[N_BLINKERS];
			
			ttlv_get_bytes(N_BLINKERS, blink_flags);
			ttlv_finish_recv();
			
			for (uint8_t i = 0; i < N_BLINKERS; i++)
				update_blink_state(i, blink_flags[i]);
			
			ttlv_xmit_response(BLINK_RESPONSE_MSG_TYPE, 2, TTLV_DATA_CPTR("OK"));
		}
		else {  // Assume that whoever it is wants to know about your blink state.
			ttlv_finish_recv();
			//ttlv_xmit_response(BLINK_RESPONSE_MSG_TYPE, 2, TTLV_DATA_CPTR("NO"));  // Or just say NO.
			ttlv_xmit_response(BLINK_STATE_MSG_TYPE, N_BLINKERS, blink_states);
		}
	}
	
	task->st |= TASK_SLEEP_BIT;  // Set sleep flag.
}

static void init_tasks(void) {
	sched_task task;
	
	for (uint8_t i = 0; i < N_BLINKERS; i++) {
		task = (sched_task) {
			.st = TASK_ST_MAKE(i, BLINKER_TASK_CAT, 0),
			.delay = SCHED_TIME_ZERO,
			.handler = blink_handler
		};
		sched_add(&task);
	}
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, MESSENGER_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = message_handler
	};
	sched_add(&task);
}

// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// Set blinky LED pins as outputs.
	BLINK_DDR |= BV(BLINK_PIN0) | BV(BLINK_PIN1);
	
	// Enable pull-ups on input pins.
	BLINK_INPUT_PORT |= BV(BLINK_INPUT_PIN0) | BV(BLINK_INPUT_PIN1);
	
	sched_init();
	
	TBOUNCER_INIT(
		TASK_ST_MAKE(0, TBOUNCER_TASK_CAT, 0), SCHED_TIME_MS(4),
		0, BV(BLINK_INPUT_PIN0) | BV(BLINK_INPUT_PIN1), 0, 0,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(BLINKER_TASK_CAT));
	
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
