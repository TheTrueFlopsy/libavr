
#include <avr/io.h>

#include "task_sched.h"
#include "tbouncer.h"
#include "std_tlv.h"


// ---<<< Constant Definitions >>>---
#define FWID_L 0x02
#define FWID_H 0x01
#define FWVERSION 0x01

#define BLINK_DDR DDRD
#define BLINK_PORT PORTD
#define BLINK_PINR PIND
#define BLINK_PIN0 4
#define BLINK_PIN1 2

#define BLINK_INPUT_PINR PINB
#define BLINK_INPUT_PORT PORTB
#define BLINK_INPUT_PIN0 1
#define BLINK_INPUT_PIN1 0

#define BLINK_INPUT_RISING TBOUNCER_B_RISING

#define TBOUNCER_TASK_CAT 15
#define TTLV_TASK_CAT 14
#define BLINKER_TASK_CAT 0
#define MESSENGER_TASK_CAT 1

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define INM_ADDR 0x01
#define N_BLINKERS 2

#define BLINK_MAX_MSG_LEN 8

enum {
	BLINK_MSG_T_GET_STATE     = 0x01 + TTLV_MSG_T_APPLICATION,
	BLINK_MSG_T_GET_STATE_RES = 0x02 + TTLV_MSG_T_APPLICATION
};

enum {
	BLINK_STATE_START    = 0,
	BLINK_STATE_ENABLED  = 1,
	BLINK_STATE_STOP     = 2,
	BLINK_STATE_DISABLED = 3
};

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


// ---<<< Program State >>>---
static sched_time blink_timestamps[N_BLINKERS] = {
	SCHED_TIME_ZERO,
	SCHED_TIME_ZERO
};

static uint8_t blink_states[N_BLINKERS] = {
	BLINK_STATE_START,
	BLINK_STATE_START
};

static union {
	uint8_t b[BLINK_MAX_MSG_LEN];
	ttlv_msg_inm_reg r;
	ttlv_msg_inm_regpair rp;
} msg_data_out;

static union {
	uint8_t b[BLINK_MAX_MSG_LEN];
	ttlv_msg_reg r;
} msg_data_in;


// ---<<< Helper Functions >>>---
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

static uint8_t get_blink_flags(void) {
	uint8_t blink_flags = 0;
	
	for (uint8_t i = 0; i < N_BLINKERS; i++)
		if (blink_states[i] == BLINK_STATE_ENABLED)
			blink_flags |= BV(i);
	
	return blink_flags;
}

static void set_blink_flags(uint8_t blink_flags) {
	for (uint8_t i = 0; i < N_BLINKERS; i++) {
		update_blink_state(i, 1 & blink_flags);
		blink_flags >>= 1;
	}
}

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if enabled, otherwise 0).
		*value_p = get_blink_flags();
		break;
	case TTLV_REG_DEBUG1:  // LED blinker state 0.
		*value_p = blink_states[0];
		break;
	case TTLV_REG_DEBUG2:  // LED blinker state 1.
		*value_p = blink_states[1];
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
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if enabled, otherwise 0).
		set_blink_flags(value);
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
		ttlv_xmit(TTLV_BROADCAST_ADR, BLINK_MSG_T_GET_STATE_RES, N_BLINKERS, blink_states);
		
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
		ttlv_xmit(TTLV_BROADCAST_ADR, BLINK_MSG_T_GET_STATE_RES, N_BLINKERS, blink_states);
		
		// fallthrough
	case BLINK_STATE_DISABLED:  // Blinking disabled.
	default:
		task->st |= TASK_SLEEP_BIT;  // Set sleep flag.
		break;
	}
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
	else if (TTLV_CHECK_TL(BLINK_MSG_T_GET_STATE, 0)) {  // Get blink states.
		ttlv_finish_recv();
		ttlv_xmit_response(BLINK_MSG_T_GET_STATE_RES, N_BLINKERS, blink_states);
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
		BV(BLINK_INPUT_PIN0) | BV(BLINK_INPUT_PIN1), 0, 0,
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
