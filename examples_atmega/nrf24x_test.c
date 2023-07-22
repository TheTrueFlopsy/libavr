
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

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
#define FWVERSION 0x02

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

#define NRF24X_TEST_MAX_MSG_LEN 8

#define NOT_A_COMMAND 0b11110000

#define N_LEDS 3
#define LED_BLINK_TASK_DELAY SCHED_TIME_MS(50)

#define LED_CTRL_TASK_DELAY SCHED_TIME_MS(10)
#define LED_CTRL_MAX_PAYLOAD (2*N_LEDS)
#define LED_CTRL_IDLE_DELAY SCHED_TIME_MS(2000)

enum {
	NRF24X_TASK_CAT    =  0,
	LED_BLINK_TASK_CAT =  1,
	LED_CTRL_TASK_CAT  =  2,
	MESSENGER_TASK_CAT =  3,
	TTLV_TASK_CAT      = 14
};

enum {
	TTLV_APP_MSG_T_OUT    = TTLV_MSG_T_APPLICATION + 0x00,
	TTLV_APP_MSG_T_IN     = TTLV_MSG_T_APPLICATION + 0x01,
	TTLV_APP_MSG_T_IN_RES = TTLV_MSG_T_APPLICATION + 0x02
};

enum {
	TTLV_APP_REG_STATUS     = TTLV_REG_APPLICATION + 0x00,
	TTLV_APP_REG_CMD_OUT_0  = TTLV_REG_APPLICATION + 0x01,
	TTLV_APP_REG_IOPINS     = TTLV_REG_APPLICATION + 0x02,  // CE and IRQ lines of the nRF24x chip
	TTLV_APP_REG_LED_CTRL   = TTLV_REG_APPLICATION + 0x03,  // LED control mode
	TTLV_APP_REG_LED_STEP   = TTLV_REG_APPLICATION + 0x04,  // LED control step
	TTLV_APP_REG_LED_FLAGS  = TTLV_REG_APPLICATION + 0x05,  // LED control flags
	TTLV_APP_REG_LED_RATE   = TTLV_REG_APPLICATION + 0x06,  // LED update request frequency
	TTLV_APP_REG_NRF24X     = TTLV_REG_APPLICATION + 0x20,
	TTLV_APP_REG_NRF24X_END = TTLV_APP_REG_NRF24X  + 0x20
};

enum {
	TTLV_APP_RES_BUSY     = TTLV_RES_APPLICATION + 0x00,
	TTLV_APP_RES_BFR_SIZE = TTLV_RES_APPLICATION + 0x01
};

enum {
	LED_PIN0 = PORTD4,
	LED_PIN1 = PORTD3,
	LED_PIN2 = PORTD2
};

#define N_LED_STATES 4

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
	LED_STEP_NONE         = 0,
	LED_STEP_STARTUP      = 1,
	LED_STEP_INIT_RETR    = 2,
	LED_STEP_INIT_DYNPD   = 3,
	LED_STEP_INIT_FEATURE = 4,
	LED_STEP_BEGIN        = 5,
	LED_STEP_UPDATE       = 6,
	LED_STEP_WAIT_POLL    = 7,
	LED_STEP_WAIT_POLLING = 8,
	LED_STEP_WAIT_FLUSH   = 9,
	LED_STEP_FETCH_LEN    = 10,
	LED_STEP_FETCH_DATA   = 11
};

enum {
	LED_FLAG_NONE   = 0,
	LED_FLAG_SYNCH  = BV(0),
	LED_FLAG_FREEZE = BV(1),
	LED_FLAG_INV    = BV(2),
	LED_FLAG_ROTP   = BV(3),
	LED_FLAG_ROTM   = BV(4)
};

enum {
	LED_UPDATE_NONE   = 0,
	LED_UPDATE_INVERT = 1,
	LED_UPDATE_ROTATE = 2
};

static const uint8_t led_pins[N_LEDS] = {
	BV(LED_PIN0),
	BV(LED_PIN1),
	BV(LED_PIN2)
};

enum {
	NRF_PIN_CE  = PORTD5, // output
	NRF_PIN_IRQ = PIND6,  // input, active-low
	NRF_PIN_VCC = PORTD7  // output, active-low (if implemented with a PNP transistor)
};

#define NRF_PINS_SIZE 3
#define NRF_PINS_OFFSET 5

enum {
	NRF_IN_OPT_NONE  = 0,
	NRF_IN_OPT_REG_R = BV(0)  // Send INM_REG_READ_RES response message.
};

enum {
	NRFQ_NONE        = 0,
	NRFQ_OUT_PENDING = BV(0),
	NRFQ_OUT_ACTIVE  = BV(1),
	NRFQ_OUT         = NRFQ_OUT_PENDING | NRFQ_OUT_ACTIVE,
	NRFQ_IN_PENDING  = BV(2),
	NRFQ_IN_ACTIVE   = BV(3),
	NRFQ_IN          = NRFQ_IN_PENDING | NRFQ_IN_ACTIVE,
	NRFQ_ANY         = NRFQ_OUT | NRFQ_IN
};


// ---<<< Program State >>>---
static uint8_t led_states[N_LEDS] = { LED_STATE_OFF, LED_STATE_OFF, LED_STATE_OFF };
static uint8_t led_blink_counter = 0;

static uint8_t led_ctrl_mode = LED_CTRL_MANUAL;
static uint8_t led_ctrl_step = LED_STEP_NONE;
static uint8_t led_ctrl_flags = LED_FLAG_NONE;
static uint8_t led_ctrl_reg_val;  // Register values fetched via SPI are stored here.
static uint8_t led_ctrl_payload_len;
static uint8_t led_ctrl_payload[LED_CTRL_MAX_PAYLOAD];
static uint8_t led_ctrl_rate = 0;
static uint8_t led_ctrl_counter = 0;

// ISSUE: Think about how to prevent wasteful buffer allocation and copying
//        just to avoid accessing volatile data as non-volatile (which apparently
//        causes undefined behavior, at least in principle). Would it work to
//        declare all library buffers non-volatile and then access them via
//        pointers to volatile when necessary?
static uint8_t pending_nrf_out_len = 0;
static uint8_t pending_nrf_out_bfr[NRF24X_BFR_SIZE];

static uint8_t pending_nrf_in_options = NRF_IN_OPT_NONE;
static uint8_t pending_nrf_in_cmd = NOT_A_COMMAND;
static uint8_t pending_nrf_in_len;
static uint16_t pending_nrf_in_request_id;
static uint8_t pending_nrf_in_srcadr;    // For INM return values.
static uint8_t *pending_nrf_in_bfr_ptr;  // For local return values.

#ifndef NRF24X_SYNCHRONOUS
static uint8_t active_nrf_out_len = 0;

static uint8_t active_nrf_in_options = NRF_IN_OPT_NONE;
static uint8_t active_nrf_in_cmd = NOT_A_COMMAND;
static uint8_t active_nrf_in_len;
static uint16_t active_nrf_in_request_id;
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
// --- Manual Mode / Common Steps ---
static void led_handler_manual(sched_task *task);

static void led_handler_reset(sched_task *task);

static void led_handler_startup(sched_task *task);

static void led_handler_init(sched_task *task);

// --- PTX Mode ---
static void led_handler_ptx_begin(sched_task *task);

static void led_handler_ptx_update(sched_task *task);

static void led_handler_ptx_wait(sched_task *task);

static void led_handler_ptx_fetch(sched_task *task);

// --- PRX Mode ---
static void led_handler_prx_begin(sched_task *task);

static void led_handler_prx_wait(sched_task *task);

static void led_handler_prx_fetch(sched_task *task);

static void led_handler_prx_request(sched_task *task);


// ---<<< Helper Functions >>>---
static ttlv_result set_led_state(uint8_t led_num, uint8_t led_state) {
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
		set_led_state(i, (1 & led_flags) ? LED_STATE_ON : LED_STATE_OFF);
		led_flags >>= 1;
	}
}

static void invert_led_states(void) {
	for (uint8_t i = 0; i < N_LEDS; i++)
		set_led_state(i, 0x03 & ~led_states[i]);
}

static void rotate_led_states(int8_t step) {
	if (step > 0) {
		for (int8_t s = 0; s < step; s++) {
			uint8_t tmp = led_states[N_LEDS-1];
			
			for (uint8_t i = 0; i < N_LEDS-1; i++)
				set_led_state(i+1, led_states[i]);
			
			set_led_state(0, tmp);
		}
	}
	else if (step < 0) {
		for (int8_t s = 0; s > step; s--) {
			uint8_t tmp = led_states[0];
			
			for (uint8_t i = N_LEDS-1; i > 0; i--)
				set_led_state(i-1, led_states[i]);
			
			set_led_state(N_LEDS-1, tmp);
		}
	}
}

static void update_all_leds(uint8_t update_type, uint8_t update_arg) {
	switch (update_type) {  // Check requested update type.
	case LED_UPDATE_INVERT:
		invert_led_states();
		break;
	case LED_UPDATE_ROTATE:
		rotate_led_states((int8_t)update_arg);
		break;
	default:
		break;  // Unrecognized, do nothing.
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
			LED_PINR = led_pins[i];  // Toggle LED pin.
		}
	}
}

static ttlv_result set_led_ctrl_mode(uint8_t mode) {
	if (mode != LED_CTRL_MANUAL && mode != LED_CTRL_PTX && mode != LED_CTRL_PRX)
		return TTLV_RES_VALUE;  // Unrecognized LED control mode.
	
	if (mode != led_ctrl_mode) {
		sched_task *led_ctrl_task = sched_find(
			TASK_ST_NUM_CAT_MASK, TASK_ST_MAKE(0, LED_CTRL_TASK_CAT, 0), 0);
		
		if (led_ctrl_task == NULL)
			return TTLV_RES_INTERNAL;
		
		// Turn off power to the nRF24x module and then reinitialize it.
		led_ctrl_task->delay = SCHED_TIME_ZERO;
		led_ctrl_task->handler = led_handler_reset;
		led_ctrl_mode = mode;
	}
	
	return TTLV_RES_OK;
}

static uint8_t check_nrf_queues(uint8_t queue_flags) {
	if ((NRFQ_OUT_PENDING & queue_flags) && pending_nrf_out_len)
		return 1;
	
#ifndef NRF24X_SYNCHRONOUS
	if ((NRFQ_OUT_ACTIVE & queue_flags) && active_nrf_out_len)
		return 1;
#endif
	
	if ((NRFQ_IN_PENDING & queue_flags) && pending_nrf_in_cmd != NOT_A_COMMAND)
		return 1;
	
#ifndef NRF24X_SYNCHRONOUS
	if ((NRFQ_IN_ACTIVE & queue_flags) && active_nrf_in_cmd != NOT_A_COMMAND)
		return 1;
#endif
	
	return 0;
}

// NOTE: Output data must be pre-loaded into pending_nrf_out_bfr.
static void enqueue_nrf_out(uint8_t len) {
	pending_nrf_out_len = len;
	sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
}

static void enqueue_nrf_w_reg(uint8_t reg, uint8_t val) {
	pending_nrf_out_bfr[0] = NRF24X_W_REG_CMD(reg);
	pending_nrf_out_bfr[1] = val;
	enqueue_nrf_out(2);
}

static uint8_t enqueue_nrf_in(
	uint8_t opt, uint8_t cmd, uint8_t len, uint16_t req_id, uint8_t srcadr, uint8_t *bfr_ptr)
{
	if (pending_nrf_in_cmd != NOT_A_COMMAND)
		return 0;
	
	pending_nrf_in_options = opt;
	pending_nrf_in_cmd = cmd;
	pending_nrf_in_len = len;
	pending_nrf_in_request_id = req_id;
	pending_nrf_in_srcadr = srcadr;
	pending_nrf_in_bfr_ptr = bfr_ptr;
	
	sched_task_tcww |= SCHED_CATFLAG(NRF24X_TASK_CAT);  // Awaken nRF24x control task.
	return 1;
}

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	// NOTE: This is sufficient for the R_REGISTER command, except the 5-byte address registers.
	if (index >= TTLV_APP_REG_NRF24X && index < TTLV_APP_REG_NRF24X_END) {
		if (enqueue_nrf_in(  // Attempt to enqueue nRF24x input operation.
			NRF_IN_OPT_REG_R, NRF24X_R_REG_CMD(index), 1,
			ttlv_recv_inm_header.h.msg_id, ttlv_recv_inm_header.h.srcadr, NULL))
		{
			return TTLV_RES_NONE;  // Suppress immediate INM response.
		}
		else  // Input operation already pending.
			return TTLV_APP_RES_BUSY;
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
	case TTLV_APP_REG_IOPINS:  // Get logic states of CE and IRQ pins.
		*value_p = GET_BITFIELD_AT(NRF_PINS_SIZE, NRF_PINS_OFFSET, NRF_PINR);
		break;
	case TTLV_APP_REG_LED_CTRL:
		*value_p = led_ctrl_mode;
		break;
	case TTLV_APP_REG_LED_STEP:
		*value_p = led_ctrl_step;
		break;
	case TTLV_APP_REG_LED_FLAGS:
		*value_p = led_ctrl_flags;
		break;
	case TTLV_APP_REG_LED_RATE:
		*value_p = led_ctrl_rate;
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result ttlv_reg_write(ttlv_reg_index index, ttlv_reg_value value) {
	// NOTE: This is sufficient for the W_REGISTER command, except the 5-byte address registers.
	if (index >= TTLV_APP_REG_NRF24X && index < TTLV_APP_REG_NRF24X_END) {
		if (pending_nrf_out_len)
			return TTLV_APP_RES_BUSY;
		
		enqueue_nrf_w_reg(index, value);
		return TTLV_RES_OK;  // Send immediate INM response, finish write operation later.
	}
	
	switch (index) {
		uint8_t tmp;
		
	case TTLV_REG_DEBUG0:  // LED blinker states as bits (1 sets ON, 0 sets OFF)
		set_led_flags(value);
		break;
	case TTLV_REG_DEBUG1:  // LED blinker state 0
		return set_led_state(0, value);
	case TTLV_REG_DEBUG2:  // LED blinker state 1
		return set_led_state(1, value);
	case TTLV_REG_DEBUG3:  // LED blinker state 2
		return set_led_state(2, value);
	case TTLV_APP_REG_CMD_OUT_0:  // Enqueue nRF24x command without data bytes.
		// NOTE: This is sufficient for the FLUSH_TX, FLUSH_RX, REUSE_TX_PL and NOP commands.
		if (pending_nrf_out_len)
			return TTLV_APP_RES_BUSY;
		
		pending_nrf_out_bfr[0] = value;
		enqueue_nrf_out(1);  // Enqueue nRF24x output operation.
		break;
	case TTLV_APP_REG_IOPINS:  // Set logic state of CE and Vcc pins.
		tmp = NRF_PORT;
		tmp &= ~(BV(NRF_PIN_CE) | BV(NRF_PIN_VCC));
		tmp |= (BV(NRF_PIN_CE) | BV(NRF_PIN_VCC)) & (value << NRF_PINS_OFFSET);
		NRF_PORT = tmp;
		break;
	case TTLV_APP_REG_LED_CTRL:
		return set_led_ctrl_mode(value);
	case TTLV_APP_REG_LED_FLAGS:
		led_ctrl_flags = value;
		break;
	case TTLV_APP_REG_LED_RATE:
		led_ctrl_rate = value;
		led_ctrl_counter = value;  // Reset the counter whenever the rate changes.
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
	else if (active_nrf_out_len) {
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		active_nrf_out_len = 0;  // Done.
	}
	else if (active_nrf_in_cmd != NOT_A_COMMAND) {
		uint8_t *nrf_in_bfr = (active_nrf_in_bfr_ptr) ? active_nrf_in_bfr_ptr : active_nrf_in_bfr;
		
		SPI_PORT |= BV(SPI_SS);  // Drive slave select pin high.
		nrf24x_in_finish(nrf_in_bfr);  // Fetch input command data.
		
		// Send INM response if requested.
		if (active_nrf_in_srcadr == TTLV_LOCAL_ADR)  // Do not send an INM response.
			;  // Do nothing.
		else if (NRF_IN_OPT_REG_R & active_nrf_in_options) {
			msg_data_out.r.index = TTLV_APP_REG_NRF24X + NRF24X_REG_ADDR(active_nrf_in_cmd);
			msg_data_out.r.value = nrf_in_bfr[0];
			msg_data_out.r.request_id = active_nrf_in_request_id;
			
			ttlv_xmit(active_nrf_in_srcadr,
				TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
		}
		else {
			ttlv_xmit(active_nrf_in_srcadr,
				TTLV_APP_MSG_T_IN_RES, active_nrf_in_len, nrf_in_bfr);
		}
		
		active_nrf_in_cmd = NOT_A_COMMAND;  // Done.
	}
#endif
	
	// NOTE: Process pending output operation before input operation to make REG_WR_EXCH work properly.
	if (pending_nrf_out_len) {
		SPI_PORT &= ~BV(SPI_SS);  // Drive slave select pin low.
		
		switch (pending_nrf_out_len) {
		case 1:
			res = nrf24x_out_0(pending_nrf_out_bfr[0]);
			break;
		case 2:
			res = nrf24x_out_1(pending_nrf_out_bfr[0], pending_nrf_out_bfr[1]);
			break;
		default:
			res = nrf24x_out_n(pending_nrf_out_bfr[0], pending_nrf_out_len-1, pending_nrf_out_bfr+1);
			break;
		}
		
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
		
		uint8_t *nrf_in_bfr = (pending_nrf_in_bfr_ptr) ? pending_nrf_in_bfr_ptr : active_nrf_in_bfr;
		
		SPI_PORT &= ~BV(SPI_SS);  // Drive slave select pin low.
		
		if (pending_nrf_in_len == 1)
			res = nrf24x_in_1(pending_nrf_in_cmd, nrf_in_bfr);
		else
			res = nrf24x_in_n(pending_nrf_in_cmd, pending_nrf_in_len, nrf_in_bfr);
		
		if (res) {  // Success!
#ifdef NRF24X_SYNCHRONOUS
			SPI_PORT |= BV(SPI_SS);  // Done. Drive slave select pin high.
			
			// Send INM response if requested.
			if (pending_nrf_in_srcadr == TTLV_LOCAL_ADR)  // Do not send an INM response.
				;  // Do nothing.
			else if (NRF_IN_OPT_REG_R & pending_nrf_in_options) {
				msg_data_out.r.index = TTLV_APP_REG_NRF24X + NRF24X_REG_ADDR(pending_nrf_in_cmd);
				msg_data_out.r.value = nrf_in_bfr[0];
				msg_data_out.r.request_id = pending_nrf_in_request_id;
				
				ttlv_xmit(pending_nrf_in_srcadr,
					TTLV_MSG_T_INM_REG_READ_RES, TTLV_MSG_L_INM_REG_READ_RES, msg_data_out.b);
			}
			else {
				ttlv_xmit(pending_nrf_in_srcadr,
					TTLV_APP_MSG_T_IN_RES, pending_nrf_in_len, nrf_in_bfr);
			}
#else
			active_nrf_in_options = pending_nrf_in_options;
			active_nrf_in_cmd = pending_nrf_in_cmd;  // Wait for asynchronous SPI operation.
			active_nrf_in_len = pending_nrf_in_len;
			active_nrf_in_request_id = pending_nrf_in_request_id;
			active_nrf_in_srcadr = pending_nrf_in_srcadr;
			active_nrf_in_bfr_ptr = pending_nrf_in_bfr_ptr;
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

static void led_blink_handler(sched_task *task) {
	blink_leds();
	task->delay = LED_BLINK_TASK_DELAY;
}

// --- Manual Mode / Common Steps---
static void led_handler_manual(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
}

static void led_handler_reset(sched_task *task) {
	led_ctrl_step = LED_STEP_NONE;
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (check_nrf_queues(NRFQ_ANY))  // Check whether there are any enqueued operations.
		return;  // In that case, wait.
	
	NRF_PORT &= ~BV(NRF_PIN_CE);  // Set the CE pin low.
	
	task->delay = SCHED_TIME_MS(500);  // Too short for power cycling?
	
	if (led_ctrl_mode == LED_CTRL_PTX || led_ctrl_mode == LED_CTRL_PRX)
		task->handler = led_handler_startup;
	else
		task->handler = led_handler_manual;
	
	NRF_PORT |= BV(NRF_PIN_VCC);  // Disable power supply to nRF24x module.
}

static void led_handler_startup(sched_task *task) {
	led_ctrl_step = LED_STEP_STARTUP;
	task->delay = SCHED_TIME_MS(200);  // nRF24x startup delay (with some margin)
	task->handler = led_handler_init;
	
	NRF_PORT &= ~BV(NRF_PIN_VCC);  // Enable power supply to nRF24x module.
}

static void led_handler_init(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
		return;  // In that case, wait.
	
	switch (led_ctrl_step) {
	case LED_STEP_INIT_RETR:
		// Write to the DYNPD register.
		led_ctrl_step = LED_STEP_INIT_DYNPD;
		enqueue_nrf_w_reg(NRF24X_DYNPD, BV(NRF24X_P0));
		break;
	case LED_STEP_INIT_DYNPD:
		// Write to the FEATURE register.
		// ISSUE: Set the FEATURE bits before setting DYNPD (and other config regs)?
		// ISSUE: Do that ACTIVATE thing, in case someone has a nRF24L01 without the '+'?
		//        Does the nRF24L01+ tolerate that?
		led_ctrl_step = LED_STEP_INIT_FEATURE;
		enqueue_nrf_w_reg(NRF24X_FEATURE, BV(NRF24X_EN_DPL) | BV(NRF24X_EN_ACK_PAY));
		break;
	case LED_STEP_INIT_FEATURE:  // Initialization done.
		switch (led_ctrl_mode) {
		case LED_CTRL_PTX:
			task->handler = led_handler_ptx_begin;
			break;
		case LED_CTRL_PRX:
			task->handler = led_handler_prx_begin;
			break;
		default:  // Something has gone wrong.
			task->handler = led_handler_reset;
			break;
		}
		break;
	default:  // Beginning initialization.
		// Write to the SETUP_RETR register.
		led_ctrl_step = LED_STEP_INIT_RETR;
		enqueue_nrf_w_reg(NRF24X_SETUP_RETR, NRF24X_SETUP_RETR_VAL(1, 3));
		break;
	}
}

// --- PTX Mode ---
static void led_handler_ptx_begin(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (led_ctrl_step == LED_STEP_BEGIN) {
		if (check_nrf_queues(NRFQ_OUT))  // Check whether there is an enqueued output operation.
			return;  // In that case, wait.
		
		srandom(SCHED_TIME_TO_TICKS(sched_ticks));  // Initialize PRNG.
		
		task->handler = led_handler_ptx_update;  // Chip activation done.
	}
	else {  // Activate the chip as PTX.
		if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
			return;  // In that case, wait.
		
		led_ctrl_step = LED_STEP_BEGIN;
		enqueue_nrf_w_reg(NRF24X_CONFIG, BV(NRF24X_EN_CRC) | BV(NRF24X_PWR_UP));
	}
}

static void led_handler_ptx_update(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
		return;  // In that case, wait.
	
	if (led_ctrl_step == LED_STEP_UPDATE) {  // LED state update initiated.
		if (check_nrf_queues(NRFQ_OUT_ACTIVE))  // Check whether there is an active output operation.
			return;  // In that case, wait. (Never set CE high before all SPI ops have finished.)
		
		// Drive the CE pin high for more than 10 μs to trigger transmission.
		_delay_us(50.0);  // Wait an extra little while just to be safe.
		NRF_PORT |= BV(NRF_PIN_CE);
		_delay_us(50.0);
		NRF_PORT &= ~BV(NRF_PIN_CE);
		
		task->handler = led_handler_ptx_wait;  // Wait for response.
	}
	else {  // Begin LED state update.
		led_ctrl_step = LED_STEP_UPDATE;
		
		// Update local LED states (arbitrarily chosen state change).
		uint32_t r = (uint32_t)random();
		uint8_t led_num = (uint8_t)(r >> 8) % N_LEDS;
		uint8_t led_state = (uint8_t)r % N_LED_STATES;
		
		if (LED_FLAG_FREEZE & led_ctrl_flags)  // The FREEZE flag preserves current LED states.
			led_state = led_states[led_num];
		else
			set_led_state(led_num, led_state);
		
		// Submit a LED state update message for transmission with the W_TX_PAYLOAD command.
		pending_nrf_out_bfr[0] = NRF24X_W_TX_PAYLOAD;
		
		if (LED_FLAG_SYNCH & led_ctrl_flags) {  // Full LED state synch requested.
			led_ctrl_flags &= ~LED_FLAG_SYNCH;
			
			for (uint8_t i = 0; i < N_LEDS; i++) {
				pending_nrf_out_bfr[1 + 2*i] = i;
				pending_nrf_out_bfr[2 + 2*i] = led_states[i];
			}
			
			enqueue_nrf_out(1 + 2*N_LEDS);
		}
		else {
			pending_nrf_out_bfr[1] = led_num;
			pending_nrf_out_bfr[2] = led_state;
			enqueue_nrf_out(3);
		}
	}
}

static void led_handler_ptx_wait(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (led_ctrl_step == LED_STEP_WAIT_POLLING) {  // STATUS polling in progress.
		// Check whether there is a pending output or enqueued input operation.
		if (check_nrf_queues(NRFQ_OUT_PENDING | NRFQ_IN))
			return;  // In that case, wait.
		
		uint8_t status = led_ctrl_reg_val;  // Get STATUS value.
		
		// ISSUE: Use a timeout as a fallback if we never see one of the expected interrupts?
		if (BV(NRF24X_TX_DS) & status) {  // Transmission successful!
			if (BV(NRF24X_RX_DR) & status) {  // Acknowledgement payload received.
				task->handler = led_handler_ptx_fetch;  // Fetch the received payload.
				enqueue_nrf_w_reg(NRF24X_STATUS, BV(NRF24X_TX_DS) | BV(NRF24X_RX_DR));  // Clear interrupt flags.
			}
			else {
				task->delay = LED_CTRL_IDLE_DELAY;  // Idle until next LED state update.
				task->handler = led_handler_ptx_update;
				enqueue_nrf_w_reg(NRF24X_STATUS, BV(NRF24X_TX_DS));  // Clear interrupt flag.
			}
		}
		else if (BV(NRF24X_MAX_RT) & status) {  // Transmission failed.
			led_ctrl_step = LED_STEP_WAIT_FLUSH;
			
			// Request a full LED state synch at the next update (since the PRX will be at least
			// two updates out of synch with the PTX by then).
			led_ctrl_flags |= LED_FLAG_SYNCH;
			
			enqueue_nrf_w_reg(NRF24X_STATUS, BV(NRF24X_MAX_RT));  // Clear interrupt flag.
		}
		else  // Transmission still ongoing?
			led_ctrl_step = LED_STEP_WAIT_POLL;
	}
	else if (led_ctrl_step == LED_STEP_WAIT_FLUSH) {
		if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
			return;  // In that case, wait.
		
		task->delay = LED_CTRL_IDLE_DELAY;  // Idle until next LED state update.
		task->handler = led_handler_ptx_update;  // Try again later.
		
		// Unacknowledged payloads remain in the TX FIFO, which must be flushed to remove the payload.
		pending_nrf_out_bfr[0] = NRF24X_FLUSH_TX;
		enqueue_nrf_out(1);
	}
	else if (BV(NRF_PIN_IRQ) & NRF_PINR)  // Don't bother with polling until the IRQ pin goes low.
		led_ctrl_step = LED_STEP_WAIT_POLL;  // Do nothing more at this time.
	else if (enqueue_nrf_in(  // Attempt to poll the STATUS register.
		NRF_IN_OPT_NONE, NRF24X_R_REG_CMD(NRF24X_STATUS), 1, 0, TTLV_LOCAL_ADR, &led_ctrl_reg_val))
	{
		led_ctrl_step = LED_STEP_WAIT_POLLING;
	}
}

static void led_handler_ptx_fetch(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	switch (led_ctrl_step) {
		uint8_t payload_len;
		
	case LED_STEP_FETCH_LEN:
		if (check_nrf_queues(NRFQ_IN))  // Check whether there is an enqueued input operation.
			return;  // In that case, wait.
		
		payload_len = led_ctrl_reg_val;  // Get payload length received from chip.
		
		if (payload_len > sizeof(led_ctrl_payload)) {  // Payload too large.
			if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
				return;  // In that case, wait.
			
			task->delay = LED_CTRL_IDLE_DELAY;  // Idle until next LED state update.
			task->handler = led_handler_ptx_update;
			
			pending_nrf_out_bfr[0] = NRF24X_FLUSH_RX;  // Flush the RX FIFO.
			enqueue_nrf_out(1);
		}
		else if (enqueue_nrf_in(  // Attempt to get the message payload.
			NRF_IN_OPT_NONE, NRF24X_R_RX_PAYLOAD, payload_len, 0, TTLV_LOCAL_ADR, led_ctrl_payload))
		{
			led_ctrl_step = LED_STEP_FETCH_DATA;
			led_ctrl_payload_len = payload_len;
		}
		break;
	case LED_STEP_FETCH_DATA:
		if (check_nrf_queues(NRFQ_IN))  // Check whether there is an enqueued input operation.
			return;  // In that case, wait.
		
		// Parse the received payload and update LED states accordingly.
		update_all_leds(led_ctrl_payload[0], led_ctrl_payload[1]);
		
		// Request a full LED state synch at the next update (since all states have changed).
		led_ctrl_flags |= LED_FLAG_SYNCH;
		
		task->delay = LED_CTRL_IDLE_DELAY;  // Idle until next LED state update.
		task->handler = led_handler_ptx_update;
		break;
	default:  // Beginning acknowledgement payload fetch.
		if (enqueue_nrf_in(  // Attempt to get the message payload length.
			NRF_IN_OPT_NONE, NRF24X_R_RX_PL_WID, 1, 0, TTLV_LOCAL_ADR, &led_ctrl_reg_val))
		{
			led_ctrl_step = LED_STEP_FETCH_LEN;
		}
		break;
	}
}

// --- PRX Mode ---
static void led_handler_prx_begin(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (check_nrf_queues(NRFQ_OUT))  // Check whether there is an enqueued output operation.
		return;  // In that case, wait.
	
	if (led_ctrl_step == LED_STEP_BEGIN) {
		// Set the CE pin high to enter the RX Mode state.
		_delay_us(50.0);  // Wait an extra little while just to be safe.
		NRF_PORT |= BV(NRF_PIN_CE);
		
		task->handler = led_handler_prx_wait;  // Chip activation done.
	}
	else {  // Activate the chip as PRX.
		led_ctrl_step = LED_STEP_BEGIN;
		enqueue_nrf_w_reg(NRF24X_CONFIG, BV(NRF24X_EN_CRC) | BV(NRF24X_PWR_UP) | BV(NRF24X_PRIM_RX));
	}
}

static void led_handler_prx_wait(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (led_ctrl_step == LED_STEP_WAIT_POLLING) {  // STATUS polling in progress.
		if (check_nrf_queues(NRFQ_IN))  // Check whether there is an enqueued input operation.
			return;  // In that case, wait.
		
		uint8_t status = led_ctrl_reg_val;  // Get STATUS value.
		
		if (BV(NRF24X_RX_DR) & status) {  // Packet received!
			if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
				return;  // In that case, wait.
			
			task->handler = led_handler_prx_fetch;  // Fetch the received payload.
			enqueue_nrf_w_reg(NRF24X_STATUS, BV(NRF24X_TX_DS) | BV(NRF24X_RX_DR));  // Clear interrupt flags.
		}
		else  // Keep waiting.
			led_ctrl_step = LED_STEP_WAIT_POLL;
	}
	else if (BV(NRF_PIN_IRQ) & NRF_PINR)  // Don't bother with polling until the IRQ pin goes low.
		led_ctrl_step = LED_STEP_WAIT_POLL;  // Do nothing more at this time.
	else if (enqueue_nrf_in(  // Attempt to poll the STATUS register.
		NRF_IN_OPT_NONE, NRF24X_R_REG_CMD(NRF24X_STATUS), 1, 0, TTLV_LOCAL_ADR, &led_ctrl_reg_val))
	{
		led_ctrl_step = LED_STEP_WAIT_POLLING;
	}
}

static void led_handler_prx_fetch(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	switch (led_ctrl_step) {
		uint8_t payload_len;
		
	case LED_STEP_FETCH_LEN:
		if (check_nrf_queues(NRFQ_IN))  // Check whether there is an enqueued input operation.
			return;  // In that case, wait.
		
		payload_len = led_ctrl_reg_val;  // Get payload length received from chip.
		
		if (payload_len > sizeof(led_ctrl_payload)) {  // Payload too large.
			if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
				return;  // In that case, wait.
			
			task->handler = led_handler_prx_wait;  // Resume waiting for packages.
			
			pending_nrf_out_bfr[0] = NRF24X_FLUSH_RX;  // Flush the RX FIFO.
			enqueue_nrf_out(1);
		}
		else if (enqueue_nrf_in(  // Get the message payload.
			NRF_IN_OPT_NONE, NRF24X_R_RX_PAYLOAD, payload_len, 0, TTLV_LOCAL_ADR, led_ctrl_payload))
		{
			led_ctrl_step = LED_STEP_FETCH_DATA;
			led_ctrl_payload_len = payload_len;
		}
		break;
	case LED_STEP_FETCH_DATA:
		if (check_nrf_queues(NRFQ_IN))  // Check whether there is an enqueued input operation.
			return;  // In that case, wait.
		
		// Parse the received payload and update LED states accordingly.
		payload_len = led_ctrl_payload_len;
		
		for (uint8_t i = 0; i < payload_len; i += 2) {
			uint8_t led_num = led_ctrl_payload[i];
			uint8_t led_state = led_ctrl_payload[i+1];
			
			if (led_num < N_LEDS)
				set_led_state(led_num, led_state);
		}
		
		task->handler = led_handler_prx_wait;  // Resume waiting for packages.
		
		// Consider submitting a LED state change request for transmission with the next
		// acknowledgement message.
		if (led_ctrl_counter > 0) {
			if (led_ctrl_counter == 1) {
				led_ctrl_counter = led_ctrl_rate;
				task->handler = led_handler_prx_request;  // Submit a state change request.
			}
			else
				led_ctrl_counter--;
		}
		break;
	default:  // Beginning payload fetch.
		if (enqueue_nrf_in(  // Get the message payload length.
			NRF_IN_OPT_NONE, NRF24X_R_RX_PL_WID, 1, 0, TTLV_LOCAL_ADR, &led_ctrl_reg_val))
		{
			led_ctrl_step = LED_STEP_FETCH_LEN;
		}
		break;
	}
}

static void led_handler_prx_request(sched_task *task) {
	task->delay = LED_CTRL_TASK_DELAY;
	
	if (check_nrf_queues(NRFQ_OUT_PENDING))  // Check whether there is a pending output operation.
		return;  // In that case, wait.
	
	pending_nrf_out_bfr[0] = NRF24X_W_ACK_PAYLOAD_CMD(0);  // Write acknowledgement payload.
	
	uint8_t flags = led_ctrl_flags;
	
	if (LED_FLAG_INV & flags) {  // Request state inversion.
		pending_nrf_out_bfr[1] = LED_UPDATE_INVERT;
		enqueue_nrf_out(2);
	}
	else if (LED_FLAG_ROTP & flags) {  // Request forward rotation.
		pending_nrf_out_bfr[1] = LED_UPDATE_ROTATE;
		pending_nrf_out_bfr[2] = 1;
		enqueue_nrf_out(3);
	}
	else {  // Request backward rotation.
		pending_nrf_out_bfr[1] = LED_UPDATE_ROTATE;
		pending_nrf_out_bfr[2] = (uint8_t)(-1);
		enqueue_nrf_out(3);
	}
	
	task->handler = led_handler_prx_wait;  // Resume waiting for packages.
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
			enqueue_nrf_out(ttlv_recv_header.h.length);  // Enqueue nRF24x output operation.
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
			
			if (msg_data_in.b[1] >= NRF24X_BFR_SIZE)
				res = TTLV_APP_RES_BFR_SIZE;
			else {
				enqueue_nrf_in(  // Enqueue nRF24x input operation.
					NRF_IN_OPT_NONE, msg_data_in.b[0], msg_data_in.b[1],
					0, ttlv_recv_inm_header.h.srcadr, NULL);
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
		.st = TASK_ST_MAKE(0, LED_BLINK_TASK_CAT, 0),
		.delay = LED_BLINK_TASK_DELAY,
		.handler = led_blink_handler
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
	
	// Set nRF24x CE and Vcc control pins as outputs.
	NRF_PORT |= BV(NRF_PIN_VCC);  // Vcc control pin is active-low.
	NRF_DDR |= BV(NRF_PIN_CE) | BV(NRF_PIN_VCC);
	
	sched_init();
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(MESSENGER_TASK_CAT));
	ttlv_xmit_inm_header.h.srcadr = INM_ADDR;  // Set INM source address.
	
	// Run SPI module in Master mode, at clock frequency F_CPU/64 ~= 250kHz (at F_CPU=16 MHz).
	// Make the Slave mode slave select pin a Master mode slave select output.
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
