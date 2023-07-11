
#include <stddef.h>
#include <stdint.h>
#include <avr/io.h>

// NOTE: When "auto_watchdog.h" is included, the watchdog timer is automatically
//       disabled following an MCU reset.
#include "auto_watchdog.h"
#include "bitops.h"
#include "task_sched.h"
#include "std_tlv.h"
#include "spihelper.h"
#include "nrf24x.h"


// ---<<< Constant Definitions >>>---
#define FWID_L 0x04
#define FWID_H 0x01
#define FWVERSION 0x01

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PINR PIND

#define NRF_DDR_CE DDRC
#define NRF_PORT_CE PORTC
#define NRF_PINR_CE PINC

#define NRF_DDR_IRQ DDRD
#define NRF_PORT_IRQ PORTD
#define NRF_PINR_IRQ PIND

// NOTE: On the Arduino Leonardo board, the hardware SS pin of the ATmegaU (PB0)
//       is wired to the RXLED indicator, so a different pin (e.g. PB4/IO8) must
//       be used to drive the SPI slave select output line.
//#define NRF24X_TEST_SS_PIN SPI_SS
#define NRF24X_TEST_SS_PIN PORTB4

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define INM_ADDR 0x02
#define N_LEDS 3

#define NRF24X_TEST_MAX_MSG_LEN 8

#define NOT_A_COMMAND 0b11110000
#define NOT_A_REGISTER 0

enum {
	NRF24X_TASK_CAT    =  0,
	MESSENGER_TASK_CAT =  1,
	TTLV_TASK_CAT      = 14
};

enum {
	TTLV_APP_MSG_T_OUT    = TTLV_MSG_T_APPLICATION + 0,
	TTLV_APP_MSG_T_IN     = TTLV_MSG_T_APPLICATION + 1,
	TTLV_APP_MSG_T_IN_RES = TTLV_MSG_T_APPLICATION + 2
};

enum {
	TTLV_APP_REG_STATUS     = TTLV_REG_APPLICATION + 0,
	TTLV_APP_REG_CMD_OUT_0  = TTLV_REG_APPLICATION + 1,
	TTLV_APP_REG_IOPINS     = TTLV_REG_APPLICATION + 2,  // CE and IRQ lines of the nRF24x chip
	TTLV_APP_REG_NRF24X     = TTLV_REG_APPLICATION + 0x20,
	TTLV_APP_REG_NRF24X_END = TTLV_APP_REG_NRF24X  + 0x20
};

enum {
	TTLV_APP_RES_BUSY     = TTLV_RES_APPLICATION + 0,
	TTLV_APP_RES_BFR_SIZE = TTLV_RES_APPLICATION + 1
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

enum {
	NRF_PIN_CE  = PORTC6,
	NRF_PIN_IRQ = PIND7
};


// ---<<< Program State >>>---
static uint8_t led_states[N_LEDS] = { 0, 0, 0 };

static uint8_t pending_cmd = NOT_A_COMMAND;

static uint8_t pending_reg_r = NOT_A_REGISTER;
static uint16_t pending_reg_r_request_id;
static uint8_t pending_reg_r_srcadr;

static uint8_t pending_reg_w = NOT_A_REGISTER;
static uint8_t pending_reg_w_value;

// ISSUE: Think about how to prevent wasteful buffer allocation and copying
//        just to avoid accessing volatile data as non-volatile (which apparently
//        causes undefined behavior, at least in principle). Would it work to
//        declare all library buffers non-volatile and then access them via
//        pointers to volatile when necessary?
static uint8_t pending_nrf_out_len = 0;
static uint8_t pending_nrf_out_bfr[NRF24X_BFR_SIZE];

static uint8_t pending_nrf_in_cmd = NOT_A_COMMAND;
static uint8_t pending_nrf_in_len;
static uint8_t pending_nrf_in_srcadr;

#ifndef NRF24X_SYNCHRONOUS
static uint8_t active_cmd = NOT_A_COMMAND;

static uint8_t active_reg_r = NOT_A_REGISTER;
static uint16_t active_reg_r_request_id;
static uint8_t active_reg_r_srcadr;
static uint8_t active_reg_r_value;

static uint8_t active_reg_w = NOT_A_REGISTER;

static uint8_t active_nrf_out_len = 0;

static uint8_t active_nrf_in_cmd = NOT_A_COMMAND;
static uint8_t active_nrf_in_len;
static uint8_t active_nrf_in_srcadr;
static uint8_t active_nrf_in_bfr[NRF24X_BFR_SIZE];
#endif

static union {
	uint8_t b[NRF24X_TEST_MAX_MSG_LEN];
	ttlv_msg_inm_reg r;
	ttlv_msg_inm_regpair rp;
} msg_data_out;

static union {
	uint8_t b[NRF24X_TEST_MAX_MSG_LEN];
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

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	// NOTE: This is sufficient for the R_REGISTER command, except the 5-byte address registers.
	if (index >= TTLV_APP_REG_NRF24X && index < TTLV_APP_REG_NRF24X_END) {
		if (pending_reg_r != NOT_A_REGISTER)
			return TTLV_APP_RES_BUSY;
		
		pending_reg_r = index;
		pending_reg_r_request_id = ttlv_recv_inm_header.h.msg_id;
		pending_reg_r_srcadr = ttlv_recv_inm_header.h.srcadr;
		
		sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
		return TTLV_RES_NONE;  // Suppress immediate INM response.
	}
	
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
	case TTLV_APP_REG_STATUS:  // nRF24x status register.
		*value_p = nrf24x_status;
		break;
	case TTLV_APP_REG_CMD_OUT_0:  // Pending nRF24x command.
		*value_p = pending_cmd;
		break;
	case TTLV_APP_REG_IOPINS:  // Get logic states of CE and IRQ pins.
		*value_p = GET_BITFIELD_AT(1, NRF_PIN_CE, NRF_PINR_CE) |
		           (GET_BITFIELD_AT(1, NRF_PIN_IRQ, NRF_PINR_IRQ) << 1);
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result ttlv_reg_write(ttlv_reg_index index, ttlv_reg_value value) {
	// NOTE: This is sufficient for the W_REGISTER command, except the 5-byte address registers.
	if (index >= TTLV_APP_REG_NRF24X && index < TTLV_APP_REG_NRF24X_END) {
		if (pending_reg_w != NOT_A_REGISTER)
			return TTLV_APP_RES_BUSY;
		
		pending_reg_w = index;
		pending_reg_w_value = value;
		
		sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
		return TTLV_RES_OK;  // Send immediate INM response, finish write operation later.
	}
	
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if enabled, otherwise 0).
		set_led_flags(value);
		break;
	case TTLV_APP_REG_CMD_OUT_0:  // Trigger nRF24x command without data bytes.
		// NOTE: This is sufficient for the FLUSH_TX, FLUSH_RX, REUSE_TX_PL and NOP commands.
		pending_cmd = value;
		sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
		break;
	case TTLV_APP_REG_IOPINS:  // Set logic state of CE pin.
		NRF_PORT_CE = SET_BITFIELD_AT(1, NRF_PIN_CE, NRF_PORT_CE, value);
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
static void nrf24x_handler(sched_task *task) {
	uint8_t res;
	
#ifndef NRF24X_SYNCHRONOUS
	if (SPI_IS_ACTIVE) {  // SPI interface is busy, unable to proceed.
		task->st |= TASK_ST_SLP(1);  // Set sleep flag.
		return;
	}
	else if (active_cmd != NOT_A_COMMAND) {  // nRF24x command transmitted.
		SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Drive slave select pin high.
		active_cmd = NOT_A_COMMAND;  // Done.
	}
	else if (active_reg_w != NOT_A_REGISTER) {  // nRF24x register write finished.
		SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Drive slave select pin high.
		active_reg_w = NOT_A_REGISTER;  // Done.
	}
	else if (active_reg_r != NOT_A_REGISTER) {  // nRF24x register read finished.
		SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Drive slave select pin high.
		nrf24x_in_finish(&active_reg_r_value);  // Fetch read register value.
		
		// Send INM response.
		msg_data_out.r.index = active_reg_r;
		msg_data_out.r.value = active_reg_r_value;
		msg_data_out.r.request_id = active_reg_r_request_id;
		
		ttlv_xmit(active_reg_r_srcadr,
			TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
		
		active_reg_r = NOT_A_REGISTER;  // Done.
	}
	else if (active_nrf_out_len) {
		SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Drive slave select pin high.
		active_nrf_out_len = 0;  // Done.
	}
	else if (active_nrf_in_cmd != NOT_A_COMMAND) {
		SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Drive slave select pin high.
		nrf24x_in_finish(active_nrf_in_bfr);  // Fetch input command data.
		// ISSUE: Verify that we got the right number of bytes?
		
		// Send INM response.
		ttlv_xmit(active_nrf_in_srcadr,
			TTLV_APP_MSG_T_IN_RES, active_nrf_in_len, active_nrf_in_bfr);
		
		active_nrf_in_cmd = NOT_A_COMMAND;  // Done.
	}
#endif
	
	if (pending_cmd != NOT_A_COMMAND) {
		SPI_PORT &= ~BV(NRF24X_TEST_SS_PIN); // Drive slave select pin low.
		res = nrf24x_out_0(pending_cmd);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Done. Drive slave select pin high.
#else
			active_cmd = pending_cmd;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Command canceled. Drive slave select pin high.
		}
		
		pending_cmd = NOT_A_COMMAND;  // Command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	// NOTE: Process pending register writes before reads to make REG_WR_EXCH work properly.
	if (pending_reg_w != NOT_A_REGISTER) {
		// IDEA: Add delays (1 musec?) between driving SS low/high and exchanging bytes.
		SPI_PORT &= ~BV(NRF24X_TEST_SS_PIN); // Drive slave select pin low.
		// IDEA: Try sending a register write as hard-coded bytes.
		res = nrf24x_out_1(NRF24X_W_REG_CMD(pending_reg_w), pending_reg_w_value);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Done. Drive slave select pin high.
#else
			active_reg_w = pending_reg_w;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Command canceled. Drive slave select pin high.
		}
		
		pending_reg_w = NOT_A_REGISTER;  // Register write finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_reg_r != NOT_A_REGISTER) {
#ifdef NRF24X_SYNCHRONOUS
		uint8_t active_reg_r_value;
#endif
		
		SPI_PORT &= ~BV(NRF24X_TEST_SS_PIN); // Drive slave select pin low.
		res = nrf24x_in_1(NRF24X_R_REG_CMD(pending_reg_r), &active_reg_r_value);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Done. Drive slave select pin high.
			
			// Send INM response.
			msg_data_out.r.index = pending_reg_r;
			msg_data_out.r.value = active_reg_r_value;
			msg_data_out.r.request_id = pending_reg_r_request_id;
			
			ttlv_xmit(pending_reg_r_srcadr,
				TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
#else
			active_reg_r = pending_reg_r;  // Wait for asynchronous SPI operation.
			active_reg_r_request_id = pending_reg_r_request_id;
			active_reg_r_srcadr = pending_reg_r_srcadr;
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Command canceled. Drive slave select pin high.
		}
		
		pending_reg_r = NOT_A_REGISTER;  // Register read finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_nrf_out_len) {
		SPI_PORT &= ~BV(NRF24X_TEST_SS_PIN); // Drive slave select pin low.
		res = nrf24x_out_n(pending_nrf_out_bfr[0], pending_nrf_out_len-1, pending_nrf_out_bfr+1);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Done. Drive slave select pin high.
#else
			active_nrf_out_len = pending_nrf_out_len;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Command canceled. Drive slave select pin high.
		}
		
		pending_nrf_out_len = 0;  // Output command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_nrf_in_cmd != NOT_A_COMMAND) {
#ifdef NRF24X_SYNCHRONOUS
		uint8_t active_nrf_in_bfr[NRF24X_BFR_SIZE];
#endif
		
		SPI_PORT &= ~BV(NRF24X_TEST_SS_PIN); // Drive slave select pin low.
		res = nrf24x_in_n(pending_nrf_in_cmd, pending_nrf_in_len, active_nrf_in_bfr);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Done. Drive slave select pin high.
			
			// Send INM response.
			ttlv_xmit(pending_nrf_in_srcadr,
				TTLV_APP_MSG_T_IN_RES, pending_nrf_in_len, active_nrf_in_bfr);
#else
			active_nrf_in_cmd = pending_nrf_in_cmd;  // Wait for asynchronous SPI operation.
			active_nrf_in_len = pending_nrf_in_len;
			active_nrf_in_srcadr = pending_nrf_in_srcadr;
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(NRF24X_TEST_SS_PIN);  // Command canceled. Drive slave select pin high.
		}
		
		pending_nrf_in_cmd = NOT_A_COMMAND;  // Input command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	task->st |= TASK_ST_SLP(1);  // No work to do. Set sleep flag.
}

// TODO: Add generic nRF24x output and and input commands.
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
		
		if (msg_data_in.r.index >= TTLV_APP_REG_NRF24X && msg_data_in.r.index < TTLV_APP_REG_NRF24X_END)
			res = TTLV_RES_REGISTER;  // Unsupported operation.
		else {
			msg_data_out.r.index = msg_data_in.r.index;
			msg_data_out.r.value = msg_data_in.r.value;
			msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
			
			res = ttlv_reg_toggle(msg_data_out.r.index, &msg_data_out.r.value);
			
			if (res == TTLV_RES_OK) {
				ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
				res = TTLV_RES_NONE;
			}
		}
	}
	else if (TTLV_CHECK_REG_RW_EXCH) {
		ttlv_recv(msg_data_in.b);
		
		if (msg_data_in.r.index >= TTLV_APP_REG_NRF24X && msg_data_in.r.index < TTLV_APP_REG_NRF24X_END)
			res = TTLV_RES_REGISTER;  // Unsupported operation.
		else {
			msg_data_out.r.index = msg_data_in.r.index;
			msg_data_out.r.value = msg_data_in.r.value;
			msg_data_out.r.request_id = ttlv_recv_inm_header.h.msg_id;
			
			res = ttlv_reg_rw_exch(msg_data_out.r.index, &msg_data_out.r.value);
			
			if (res == TTLV_RES_OK) {
				ttlv_xmit_response(TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
				res = TTLV_RES_NONE;
			}
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
	else if (TTLV_CHECK_TL_MAX(TTLV_APP_MSG_T_OUT, NRF24X_TEST_MAX_MSG_LEN)) {
		if (pending_nrf_out_len) {
			ttlv_finish_recv();
			res = TTLV_APP_RES_BUSY;
		}
		else {
			ttlv_recv(pending_nrf_out_bfr);
			pending_nrf_out_len = ttlv_recv_header.h.length;
			sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
			res = TTLV_RES_OK;
		}
	}
	else if (TTLV_CHECK_TL(TTLV_APP_MSG_T_IN, 2)) {
		if (pending_nrf_in_cmd != NOT_A_COMMAND) {
			ttlv_finish_recv();
			res = TTLV_APP_RES_BUSY;
		}
		else {
			ttlv_recv(msg_data_in.b);
			
			pending_nrf_in_len = msg_data_in.b[1];
			
			if (pending_nrf_in_len >= NRF24X_BFR_SIZE)
				res = TTLV_APP_RES_BFR_SIZE;
			else {
				pending_nrf_in_cmd = msg_data_in.b[0];
				pending_nrf_in_srcadr = ttlv_recv_inm_header.h.srcadr;
				sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
			}
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
		.st = TASK_ST_MAKE(0, NRF24X_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = nrf24x_handler
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
	
	// Set nRF24x CE control pin as output.
	NRF_DDR_CE |= BV(NRF_PIN_CE);
	
	sched_init();
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(MESSENGER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = INM_ADDR;  // Set INM source address.
	
	// Run SPI module in Master mode, at clock frequency F_CPU/64 ~= 250kHz (at F_CPU=16 MHz).
	// Make the Slave mode slave select pin (NRF24X_TEST_SS_PIN in port B) a Master mode slave
	// select output.
#ifdef NRF24X_SYNCHRONOUS
	spihelper_mstr_init(BV(NRF24X_TEST_SS_PIN), 0, 0, BV(MSTR) | BV(SPR1));
#else
	spihelper_async_mstr_init(
		BV(NRF24X_TEST_SS_PIN), 0, 0, BV(MSTR) | BV(SPR1),
		SCHED_CATFLAG(NRF24X_TASK_CAT));
#endif
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
