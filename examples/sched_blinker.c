
#include <avr/io.h>
//#include <util/delay.h>

#include "task_sched.h"
#include "tbouncer.h"
#include "task_tlv.h"

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

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

static const uint8_t blink_pins[2] = {
	BV(BLINK_PIN0),
	BV(BLINK_PIN1)
};

static const uint8_t blink_input_pins[2] = {
	BV(BLINK_INPUT_PIN0),
	BV(BLINK_INPUT_PIN1)
};

static const sched_time blink_delays[2] = {
	SCHED_TIME_MS(450),
	SCHED_TIME_MS(200)
};

static uint8_t blink_states[2] = {
	1,
	1
};

//#define DEBUG_BLINK_DELAY 400.0

/*static void debug_blink(uint8_t n) {
	uint8_t i;
	for (i = 0; i < n; i++) {
		BLINK_PORT |= BV(PORTD3);
		_delay_ms(DEBUG_BLINK_DELAY);
		BLINK_PORT &= ~BV(PORTD3);
		_delay_ms(DEBUG_BLINK_DELAY);
	}
}*/

static void blink_handler(sched_task *task) {
	uint8_t num = TASK_ST_GET_NUM(task->st);
	//ttlv_state res;
	
	if (BLINK_INPUT_RISING & blink_input_pins[num]) { // Detect input event.
		blink_states[num] = !blink_states[num]; // Change blink state.
		
		ttlv_xmit(0, 0x11, 2, blink_states); // Send informational message about blink states.
		
		/*if (res >= TTLV_E_UNSPECIFIED) {// DEBUG:
			res -= TTLV_E_UNSPECIFIED;
			BLINK_PORT = 0;
			_delay_ms(200.0);
			BLINK_PORT |= (res << 2);
			_delay_ms(1000.0);
			BLINK_PORT = 0;
			_delay_ms(200.0);
			//debug_blink(res - (TTLV_E_UNSPECIFIED - 1));
		}*/
	}
	
	if (TTLV_HAS_MESSAGE) {
		//BLINK_PINR = BV(PIND3); // DEBUG: 
		
		if (ttlv_recv_header.h.type == 0x11 && ttlv_recv_header.h.length == 2) {
			ttlv_get_bytes(2, blink_states);
			ttlv_finish_recv();
			ttlv_xmit(0, 1, 2, (const uint8_t*)"OK");
		}
		else {
			ttlv_finish_recv();
			//ttlv_xmit(0, 1, 2, (const uint8_t*)"NO");
			ttlv_xmit(0, 0x11, 2, ttlv_recv_header.b);
		}
	}
	
	if (blink_states[num]) { // Enabled
		if (sched_time_is_zero(task->delay)) { // Timed out.
			BLINK_PINR = blink_pins[num]; // Toggle LED by writing 1 to PIN register.
			task->delay = blink_delays[num]; // Refresh task delay.
		}
	}
	else { // Disabled
		BLINK_PORT &= ~blink_pins[num]; // Turn off LED.
		task->st |= TASK_ST_SLP(1); // Set sleep flag.
	}
}

static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_CAT(BLINKER_TASK_CAT) | TASK_ST_NUM(0),
		.delay = SCHED_TIME_ZERO,
		.handler = blink_handler
	};
	sched_add(&task);
	
	task = (sched_task) {
		.st = TASK_ST_CAT(BLINKER_TASK_CAT) | TASK_ST_NUM(1),
		.delay = SCHED_TIME_ZERO,
		.handler = blink_handler
	};
	sched_add(&task);
}

int main(void) {
	// Set blinky LED pins as outputs.
	BLINK_DDR |= BV(BLINK_PIN0) | BV(BLINK_PIN1);
	
	BLINK_DDR |= BV(DDD3); // DEBUG: Debugging indicator LED.
	
	// Enable pull-ups on input pins.
	BLINK_INPUT_PORT |= BV(BLINK_INPUT_PIN0) | BV(BLINK_INPUT_PIN1);
	
	sched_init();
	
	tbouncer_init(
		TASK_ST_CAT(TBOUNCER_TASK_CAT) | TASK_ST_NUM(0), SCHED_TIME_MS(10),
		BV(BLINK_INPUT_PIN0) | BV(BLINK_INPUT_PIN1), 0, 0,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(BLINKER_TASK_CAT));
	
	ttlv_init(
		TASK_ST_CAT(TTLV_TASK_CAT) | TASK_ST_NUM(0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(BLINKER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = 0x20; // Set INM source address.
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
