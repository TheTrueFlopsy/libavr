
#include <avr/io.h>

// NOTE: When "auto_watchdog.h" is included, the watchdog timer is automatically
//       disabled following an MCU reset.
#include "auto_watchdog.h"
#include "task_sched.h"
#include "std_tlv.h"
#include "spihelper.h"
#include "mcp4x.h"


// ---<<< Constant Definitions >>>---
#define FWID_L 0x04
#define FWID_H 0x01
#define FWVERSION 0x01

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PINR PIND

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define INM_ADDR 0x01
#define N_LEDS 3

#define MCP4X_TEST_MAX_MSG_LEN 8

enum {
	MCP4X_TASK_CAT     =  0,
	MESSENGER_TASK_CAT =  1,
	TTLV_TASK_CAT      = 14
};

enum {
	TTLV_APP_REG_P0 = TTLV_REG_APPLICATION + 0,
	TTLV_APP_REG_P1 = TTLV_REG_APPLICATION + 1
};

enum {
	LED_PIN0 = PORTD4,
	LED_PIN1 = PORTD0,
	LED_PIN2 = PORTD1
};

static const uint8_t led_pins[N_LEDS] = {
	BV(LED_PIN0),
	BV(LED_PIN1),
	BV(LED_PIN2)
};


// ---<<< Program State >>>---
static uint8_t led_states[N_LEDS] = { 0, 0, 0 };

static uint8_t p0_wiper = 0x80;
static uint8_t p1_wiper = 0x80;

static uint8_t desired_p0_wiper = 0x80;
static uint8_t desired_p1_wiper = 0x80;

#ifndef MCP4X_SYNCHRONOUS
static uint8_t new_p0_wiper = 0x80;
static uint8_t new_p1_wiper = 0x80;
#endif

static union {
	uint8_t b[MCP4X_TEST_MAX_MSG_LEN];
	ttlv_msg_inm_reg r;
	ttlv_msg_inm_regpair rp;
} msg_data_out;

static union {
	uint8_t b[MCP4X_TEST_MAX_MSG_LEN];
	ttlv_msg_reg r;
} msg_data_in;


// ---<<< Helper Functions >>>---
static void update_led_state(uint8_t led_num, uint8_t led_flag) {
	led_states[led_num] = led_flag;
	
	if (led_flag)  // Enable LED.
		LED_PORT |= led_pins[led_num];
	else  // Disable LED.
		LED_PORT &= ~led_pins[led_num];
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

static uint8_t set_wiper(uint8_t pot_bits, uint8_t pos) {
	uint8_t res;
	
	SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
	
	// ISSUE: Do we need some sort of delay here?
	// NOTE: MCP4x datasheet says 40 ns. An ATmega clock cycle at 16 MHz is 62.5 ns.
	
	res = mcp4x_set_wiper(pot_bits, pos);
	
	// ISSUE: Or here?
	
#ifdef MCP4X_SYNCHRONOUS
	SPI_PORT |= BV(SPI_SS);  // mcp4x_set_wiper is synchronous. Drive slave select pin high.
#else
	if (!res)
		SPI_PORT |= BV(SPI_SS);  // Failed to start SPI operation. Drive slave select pin high.
#endif
	
	return res;
}

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if enabled, otherwise 0).
		*value_p = get_led_flags();
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
	case TTLV_APP_REG_P0:  // Potentiometer P0 wiper setting.
		*value_p = p0_wiper;
		break;
	case TTLV_APP_REG_P1:  // Potentiometer P1 wiper setting.
		*value_p = p1_wiper;
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result ttlv_reg_write(ttlv_reg_index index, ttlv_reg_value value) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if enabled, otherwise 0).
		set_led_flags(value);
		break;
	case TTLV_APP_REG_P0:  // Potentiometer P0 wiper setting.
		desired_p0_wiper = value;
		sched_task_tcww |= SCHED_CATFLAG(MCP4X_TASK_CAT);  // Awaken MCP4x control task.
		break;
	case TTLV_APP_REG_P1:  // Potentiometer P1 wiper setting.
		desired_p1_wiper = value;
		sched_task_tcww |= SCHED_CATFLAG(MCP4X_TASK_CAT);  // Awaken MCP4x control task.
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
static void mcp4x_handler(sched_task *task) {
	uint8_t res;
	
#ifndef MCP4X_SYNCHRONOUS
	if (SPI_IS_ACTIVE) {  // SPI interface is busy, unable to proceed.
		task->st |= TASK_ST_SLP(1);  // Set sleep flag.
		return;
	}
	else if (new_p0_wiper != p0_wiper) {  // P0 wiper update done.
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		p0_wiper = new_p0_wiper;
	}
	else if (new_p1_wiper != p1_wiper) {  // P1 wiper update done.
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		p1_wiper = new_p1_wiper;
	}
#endif
	
	if (desired_p0_wiper != p0_wiper) {
		res = set_wiper(BV(MCP4X_P0), desired_p0_wiper);
		
		if (res) {  // Success!
#ifdef MCP4X_SYNCHRONOUS
			p0_wiper = desired_p0_wiper;  // Done.
#else
			new_p0_wiper = desired_p0_wiper;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			desired_p0_wiper = p0_wiper;  // Cancel update attempt.
		}
		
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (desired_p1_wiper != p1_wiper) {
		res = set_wiper(BV(MCP4X_P1), desired_p1_wiper);
		
		if (res) {  // Success!
#ifdef MCP4X_SYNCHRONOUS
			p1_wiper = desired_p1_wiper;  // Done.
#else
			new_p1_wiper = desired_p1_wiper;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			desired_p1_wiper = p1_wiper;  // Cancel update attempt.
		}
		
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	task->st |= TASK_ST_SLP(1);  // No work to do. Set sleep flag.
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
		.st = TASK_ST_MAKE(0, MCP4X_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = mcp4x_handler
	};
	sched_add(&task);
	
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
	// Set LED pins as outputs.
	LED_DDR |= BV(LED_PIN0) | BV(LED_PIN1) | BV(LED_PIN2);
	
	sched_init();
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(MESSENGER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = INM_ADDR;  // Set INM source address.
	
	// Run SPI module in Master mode, at clock frequency F_CPU/64 ~= 250kHz (at F_CPU=16 MHz).
	// Make the Slave mode slave select pin (SPI_SS in port B) a Master mode slave select output.
#ifdef MCP4X_SYNCHRONOUS
	spihelper_mstr_init(BV(SPI_SS), 0, 0, BV(MSTR) | BV(SPR1));
#else
	spihelper_async_mstr_init(
		BV(SPI_SS), 0, 0, BV(MSTR) | BV(SPR1),
		SCHED_CATFLAG(MCP4X_TASK_CAT));
#endif
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
