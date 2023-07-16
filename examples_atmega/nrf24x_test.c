
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

#define NRF_DDR DDRD
#define NRF_PORT PORTD
#define NRF_PINR PIND

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

#define INM_ADDR 0x01
#define N_LEDS 3

#define NRF24X_TEST_MAX_MSG_LEN 8

#define NOT_A_COMMAND 0b11110000
#define NOT_A_REGISTER 0

#define LED_CTRL_TASK_DELAY SCHED_TIME_MS(50)

enum {
	NRF24X_TASK_CAT    =  0,
	LED_CTRL_TASK_CAT  =  1,
	MESSENGER_TASK_CAT =  2,
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
	TTLV_APP_REG_LED_CTRL   = TTLV_REG_APPLICATION + 3,  // LED control mode
	TTLV_APP_REG_NRF24X     = TTLV_REG_APPLICATION + 0x20,
	TTLV_APP_REG_NRF24X_END = TTLV_APP_REG_NRF24X  + 0x20
};

enum {
	TTLV_APP_RES_BUSY     = TTLV_RES_APPLICATION + 0,
	TTLV_APP_RES_BFR_SIZE = TTLV_RES_APPLICATION + 1
};

enum {
	LED_PIN0 = PORTD4,
	LED_PIN1 = PORTD3,
	LED_PIN2 = PORTD2
};

enum {
	LED_STATE_OFF  = 0,
	LED_STATE_SLOW = 1,
	LED_STATE_FAST = 2,
	LED_STATE_ON   = 3
};

enum {
	LED_CTRL_MANUAL = 0,
	LED_CTRL_PTX    = 1,
	LED_CTRL_PRX    = 2
};

enum {
	LED_STEP_NONE    = 0,
	LED_STEP_STARTUP = 1,
	LED_STEP_INIT    = 2,
	LED_STEP_BEGIN   = 3
};

static const uint8_t led_pins[N_LEDS] = {
	BV(LED_PIN0),
	BV(LED_PIN1),
	BV(LED_PIN2)
};

enum {
	NRF_PIN_CE  = PORTD5,
	NRF_PIN_IRQ = PIND6,
	NRF_PIN_VCC = PORTD7
};

#define NRF_PINS_SIZE 3
#define NRF_PINS_OFFSET 5


// ---<<< Program State >>>---
static uint8_t led_states[N_LEDS] = { LED_STATE_OFF, LED_STATE_OFF, LED_STATE_OFF };
static uint8_t led_blink_counter = 0;
static uint8_t led_ctrl_mode = LED_CTRL_MANUAL;
static uint8_t led_ctrl_step = LED_STEP_NONE;

// TODO: Rationalize away the "cmd", "reg_r" and "reg_w" operations. They are
// special cases of the generic "in" and "out".
static uint8_t pending_cmd = NOT_A_COMMAND;

static uint8_t pending_reg_r = NOT_A_REGISTER;
static uint16_t pending_reg_r_request_id;
static uint8_t pending_reg_r_srcadr;      // For INM return values.
static uint8_t *pending_reg_r_value_ptr;  // For local return values.

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
static uint8_t pending_nrf_in_srcadr;    // For INM return values.
static uint8_t *pending_nrf_in_bfr_ptr;  // For local return values.

#ifndef NRF24X_SYNCHRONOUS
static uint8_t active_cmd = NOT_A_COMMAND;

static uint8_t active_reg_r = NOT_A_REGISTER;
static uint16_t active_reg_r_request_id;
static uint8_t active_reg_r_srcadr;
static uint8_t *active_reg_r_value_ptr;
static uint8_t active_reg_r_value;

static uint8_t active_reg_w = NOT_A_REGISTER;

static uint8_t active_nrf_out_len = 0;

static uint8_t active_nrf_in_cmd = NOT_A_COMMAND;
static uint8_t active_nrf_in_len;
static uint8_t active_nrf_in_srcadr;
static uint8_t *active_nrf_in_bfr_ptr;
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


// ---<<< Pre-Declaration of LED Task Sub-Handlers >>>---
// --- Manual Mode ---
static void led_handler_manual(sched_task *task);

/*
// --- PTX Mode ---
static void led_handler_ptx_startup(sched_task *task);

static void led_handler_ptx_init(sched_task *task);

static void led_handler_ptx_begin(sched_task *task);

static void led_handler_ptx_update(sched_task *task);

static void led_handler_ptx_xmit(sched_task *task);

static void led_handler_ptx_wait(sched_task *task);

static void led_handler_ptx_clear(sched_task *task);

static void led_handler_ptx_getlen(sched_task *task);

static void led_handler_ptx_getdata(sched_task *task);

// --- PRX Mode ---
static void led_handler_prx_startup(sched_task *task);

static void led_handler_prx_init(sched_task *task);

static void led_handler_prx_begin(sched_task *task);

static void led_handler_prx_wait(sched_task *task);

static void led_handler_prx_clear(sched_task *task);

static void led_handler_ptx_getlen(sched_task *task);

static void led_handler_ptx_getdata(sched_task *task);

static void led_handler_ptx_request(sched_task *task);
*/


// ---<<< Helper Functions >>>---
static ttlv_result update_led_state(uint8_t led_num, uint8_t led_state) {
	switch (led_state) {
	case LED_STATE_ON:  // LED fully on
		LED_PORT |= led_pins[led_num];
		break;
	case LED_STATE_SLOW:  // blink LED slowly
	case LED_STATE_FAST:  // blink LED quickly
		break;
	case LED_STATE_OFF: // LED fully off
		LED_PORT &= ~led_pins[led_num];
		break;
	default:
		return TTLV_RES_VALUE;  // Unrecognized LED state.
	}
	
	led_states[led_num] = led_state;
	return TTLV_RES_OK;
}

static uint8_t get_led_flags(void) {
	uint8_t led_flags = 0;
	
	for (uint8_t i = 0; i < N_LEDS; i++)
		if (led_states[i] == LED_STATE_ON)
			led_flags |= BV(i);
	
	return led_flags;
}

static void set_led_flags(uint8_t led_flags) {
	for (uint8_t i = 0; i < N_LEDS; i++) {
		update_led_state(i, (1 & led_flags) ? LED_STATE_ON : LED_STATE_OFF);
		led_flags >>= 1;
	}
}

static void blink_leds(void) {
	uint8_t toggle_slow = (GET_BITFIELD(3, led_blink_counter) == 0);
	uint8_t toggle_fast = (GET_BITFIELD(1, led_blink_counter) == 0);
	led_blink_counter++;
	
	for (uint8_t i = 0; i < N_LEDS; i++) {
		if ((toggle_slow && led_states[i] == LED_STATE_SLOW) ||
		    (toggle_fast && led_states[i] == LED_STATE_FAST))
		{
			LED_PINR |= led_pins[i];  // Toggle LED pin.
		}
	}
}

static ttlv_result set_led_ctrl_mode(uint8_t mode) {
	sched_task *led_task = sched_find(
		TASK_ST_NUM_CAT_MASK, TASK_ST_MAKE(0, LED_CTRL_TASK_CAT, 0), 0);
	
	if (led_task == NULL)
		return TTLV_RES_INTERNAL;
	
	// FIXME: How to handle transitions from PTX or PRX mode to another mode?
	
	switch (mode) {
	case LED_CTRL_MANUAL:  // manual LED control
		led_task->handler = led_handler_manual;
		break;
	case LED_CTRL_PTX:  // automatic LED control, PTX role
		//led_task->handler = led_handler_ptx_startup;
		//break;
		return TTLV_RES_NOT_IMPLEMENTED;
	case LED_CTRL_PRX:  // automatic LED control, PRX role
		//led_task->handler = led_handler_prx_startup;
		//break;
		return TTLV_RES_NOT_IMPLEMENTED;
	default:
		return TTLV_RES_VALUE;  // Unrecognized LED state.
	}
	
	led_ctrl_mode = mode;
	return TTLV_RES_OK;
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
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 if ON, otherwise 0)
		*value_p = get_led_flags();
		break;
	case TTLV_REG_DEBUG1:  // LED blinker state 0
		*value_p = led_states[0];
		break;
	case TTLV_REG_DEBUG2:  // LED blinker state 1
		*value_p = led_states[1];
		break;
	case TTLV_REG_DEBUG3:  // LED blinker state 2
		*value_p = led_states[2];
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
	case TTLV_APP_REG_STATUS:  // nRF24x status register
		*value_p = nrf24x_status;
		break;
	case TTLV_APP_REG_CMD_OUT_0:  // Get pending nRF24x command without data bytes.
		*value_p = pending_cmd;
		break;
	case TTLV_APP_REG_IOPINS:  // Get logic states of CE and IRQ pins.
		*value_p = GET_BITFIELD_AT(NRF_PINS_SIZE, NRF_PINS_OFFSET, NRF_PINR);
		break;
	case TTLV_APP_REG_LED_CTRL:
		*value_p = led_ctrl_mode;
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
	
	uint8_t tmp;
	
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 sets ON, 0 sets OFF)
		set_led_flags(value);
		break;
	case TTLV_REG_DEBUG1:  // LED blinker state 0
		return update_led_state(0, value);
	case TTLV_REG_DEBUG2:  // LED blinker state 1
		return update_led_state(1, value);
	case TTLV_REG_DEBUG3:  // LED blinker state 2
		return update_led_state(2, value);
	case TTLV_APP_REG_CMD_OUT_0:  // Enqueue nRF24x command without data bytes.
		// NOTE: This is sufficient for the FLUSH_TX, FLUSH_RX, REUSE_TX_PL and NOP commands.
		pending_cmd = value;
		sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
		break;
	case TTLV_APP_REG_IOPINS:  // Set logic state of CE and Vcc pins.
		tmp = NRF_PORT;
		tmp &= ~(BV(NRF_PIN_CE) | BV(NRF_PIN_VCC));
		tmp |= (BV(NRF_PIN_CE) | BV(NRF_PIN_VCC)) & (value << NRF_PINS_OFFSET);
		NRF_PORT = tmp;
		break;
	case TTLV_APP_REG_LED_CTRL:
		return set_led_ctrl_mode(value);
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
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		active_cmd = NOT_A_COMMAND;  // Done.
	}
	else if (active_reg_w != NOT_A_REGISTER) {  // nRF24x register write finished.
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		active_reg_w = NOT_A_REGISTER;  // Done.
	}
	else if (active_reg_r != NOT_A_REGISTER) {  // nRF24x register read finished.
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
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
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		active_nrf_out_len = 0;  // Done.
	}
	else if (active_nrf_in_cmd != NOT_A_COMMAND) {
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		nrf24x_in_finish(active_nrf_in_bfr);  // Fetch input command data.
		// ISSUE: Verify that we got the right number of bytes?
		
		// Send INM response.
		ttlv_xmit(active_nrf_in_srcadr,
			TTLV_APP_MSG_T_IN_RES, active_nrf_in_len, active_nrf_in_bfr);
		
		active_nrf_in_cmd = NOT_A_COMMAND;  // Done.
	}
#endif
	
	if (pending_cmd != NOT_A_COMMAND) {
		SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
		res = nrf24x_out_0(pending_cmd);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
#else
			active_cmd = pending_cmd;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(SPI_SS);  // Command canceled. Drive slave select pin high.
		}
		
		pending_cmd = NOT_A_COMMAND;  // Command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	// NOTE: Process pending register writes before reads to make REG_WR_EXCH work properly.
	if (pending_reg_w != NOT_A_REGISTER) {
		// IDEA: Add delays (1 musec?) between driving SS low/high and exchanging bytes.
		SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
		// IDEA: Try sending a register write as hard-coded bytes.
		res = nrf24x_out_1(NRF24X_W_REG_CMD(pending_reg_w), pending_reg_w_value);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
#else
			active_reg_w = pending_reg_w;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(SPI_SS);  // Command canceled. Drive slave select pin high.
		}
		
		pending_reg_w = NOT_A_REGISTER;  // Register write finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_reg_r != NOT_A_REGISTER) {
#ifdef NRF24X_SYNCHRONOUS
		uint8_t active_reg_r_value;
#endif
		
		SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
		res = nrf24x_in_1(NRF24X_R_REG_CMD(pending_reg_r), &active_reg_r_value);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
			
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
			SPI_PORT |= BV(SPI_SS);  // Command canceled. Drive slave select pin high.
		}
		
		pending_reg_r = NOT_A_REGISTER;  // Register read finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_nrf_out_len) {
		SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
		res = nrf24x_out_n(pending_nrf_out_bfr[0], pending_nrf_out_len-1, pending_nrf_out_bfr+1);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
#else
			active_nrf_out_len = pending_nrf_out_len;  // Wait for asynchronous SPI operation.
			task->st |= TASK_ST_SLP(1);  // Set sleep flag.
#endif
		}
		else {  // ERROR
			SPI_PORT |= BV(SPI_SS);  // Command canceled. Drive slave select pin high.
		}
		
		pending_nrf_out_len = 0;  // Output command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	if (pending_nrf_in_cmd != NOT_A_COMMAND) {
#ifdef NRF24X_SYNCHRONOUS
		uint8_t active_nrf_in_bfr[NRF24X_BFR_SIZE];
#endif
		
		SPI_PORT &= ~BV(SPI_SS); // Drive slave select pin low.
		res = nrf24x_in_n(pending_nrf_in_cmd, pending_nrf_in_len, active_nrf_in_bfr);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
			
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
			SPI_PORT |= BV(SPI_SS);  // Command canceled. Drive slave select pin high.
		}
		
		pending_nrf_in_cmd = NOT_A_COMMAND;  // Input command finished, initiated or canceled.
		return;  // Either wait for async operation or avoid dallying too long in the handler.
	}
	
	task->st |= TASK_ST_SLP(1);  // No work to do. Set sleep flag.
}

static void led_handler_manual(sched_task *task) {
	led_ctrl_step = LED_STEP_NONE;
	blink_leds();
	task->delay = LED_CTRL_TASK_DELAY;
}

/*
static void led_handler_ptx_startup(sched_task *task) {
	led_ctrl_step = LED_STEP_STARTUP;
	blink_leds();
	
	// IDEA: Switch a 5 V supply with a PNP, let the linear regulator convert the transistor
	// output to a steady 3.3 V. Any input above 4 V should be fine for the regulator.
	NRF_PORT |= BV(NRF_PIN_VCC);  // Enable power supply to nRF24x module.
	
	task->delay = SCHED_TIME_MS(100);  // nRF24x startup delay
	task->handler = led_handler_ptx_init;
}

static void led_handler_ptx_init(sched_task *task) {
	blink_leds();
	
	if (led_ctrl_step != LED_STEP_INIT) { // Beginning initialization.
		// TODO: Check whether there are enqueued nRF24x operations. In that case, wait.
		
		// TODO: Enqueue operations to set configuration registers in the nRF24x.
		
		led_ctrl_step = LED_STEP_INIT;
	}
	else {  // Waiting for initialization ops.
		// TODO: Check whether there are no enqueued nRF24x operations. In that case, continue.
		
		task->handler = led_handler_ptx_begin;
	}
	
	task->delay = LED_CTRL_TASK_DELAY;
}
*/

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
		.st = TASK_ST_MAKE(0, LED_CTRL_TASK_CAT, 0),
		.delay = LED_CTRL_TASK_DELAY,
		.handler = led_handler_manual
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
	NRF_DDR |= BV(NRF_PIN_CE);
	
	sched_init();
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(MESSENGER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = INM_ADDR;  // Set INM source address.
	
	// Run SPI module in Master mode, at clock frequency F_CPU/64 ~= 250kHz (at F_CPU=16 MHz).
	// Make the Slave mode slave select pin (SPI_SS in port B) a Master mode slave select output.
#ifdef NRF24X_SYNCHRONOUS
	spihelper_mstr_init(BV(SPI_SS), 0, 0, BV(MSTR) | BV(SPR1));
#else
	spihelper_async_mstr_init(
		BV(SPI_SS), 0, 0, BV(MSTR) | BV(SPR1),
		SCHED_CATFLAG(NRF24X_TASK_CAT));
#endif
	
	init_tasks();
	
	sched_run();
	
	return 0;
}
