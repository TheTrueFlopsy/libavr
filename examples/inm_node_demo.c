
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "task_sched.h"
#include "tbouncer.h"
#include "std_tlv.h"
//#include "memmon.h"

#include "remtcam_inm.h"


// ---<<< Constant Definitions >>>---
#define FWID_L 0x01
#define FWID_H 0x01
#define FWVERSION 0x03

#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PINR PIND

#define LED_INPUT_PINR PINB
#define LED_INPUT_PORT PORTB

#define LED_INPUT_RISING TBOUNCER_B_RISING

#define BAUD_RATE 38400
//#define BAUD_RATE 9600
#define USE_U2X 0

// TODO: This should perhaps not be hard-coded in the finished system.
//       Would a command-line macro redefinition be flexible enough?
#define INM_SRC_ADR 1

// TODO: This should perhaps not be hard-coded in the finished system.
#define INM_DST_ADR 12

//#define INM_DST_ADR_MEMMON 17
//#define MAX_MEMORY_MONITORS 4

#define REMTCAM_MAX_MSG_LEN 16

#define SENSOR_COUNT_OFFSET 5
#define SENSOR_INTERVAL_MUSECS 100

#define MICROPHONE_BIAS 512

enum {
	COMMAND_TASK_CAT     = 0,
	LED_TASK_CAT         = 1,
	SENSOR_TASK_CAT      = 2,
	TTLV_TASK_CAT        = 14,
	NOAWAKE_TASK_CAT     = 15
};

enum {
	ID_MICROPHONE  = 0, // fast sensor
	ID_LIGHT_METER = 1, // slow sensors
	// etc.
	//ID_SENSOR7 = 7,
	
	NO_SENSOR_ID = 0xff
};

enum {
	COUNT_MICROPHONE = ID_MICROPHONE << SENSOR_COUNT_OFFSET,
	COUNT_LIGHT_METER = ID_LIGHT_METER << SENSOR_COUNT_OFFSET,
	// etc.
};

enum {
	LED_PIN0      = PORTD4,
	LED_PIN1      = PORTD2,
	LED_PIN_DEBUG = PORTD3
};

enum {
	LED_INPUT_PIN0 = PINB1,
	LED_INPUT_PIN1 = PINB0
};

#define LED_PIN_MASK (BV(LED_PIN0) | BV(LED_PIN1) | BV(LED_PIN_DEBUG))


// ---<<< Program State >>>---
static uint8_t ctrl_flags = 0;
static uint8_t led_states = 0;

// ID of sensor being sampled by the ADC.
static remtcam_sensor_id curr_sensor = NO_SENSOR_ID;

// ADC sampling interval counter. Used to determine when to sample slow sensors.
static uint8_t sensor_counter = 0;

static uint16_t prev_microphone_state = 0;
static uint16_t microphone_state = 0;

static uint16_t light_meter_state = 0;

static union {
	uint8_t b[REMTCAM_MAX_MSG_LEN];
	//ttlv_msg_inm_result res;
	ttlv_msg_reg r;
	ttlv_msg_regpair rp;
	remtcam_var_header v;
	remtcam_sensor_event e;
	remtcam_sensor_sample s;
} msg_data_out;

static union {
	uint8_t b[REMTCAM_MAX_MSG_LEN];
	//ttlv_msg_inm_result res;
	ttlv_msg_reg r;
	remtcam_var_header v;
	remtcam_sensor_event e;
	remtcam_sensor_sample s;
} msg_data_in;

//static memmon_spec memory_monitors[MAX_MEMORY_MONITORS];


// ---<<< ISRs and Helper Functions >>>---

// IDEA: For the sound detector, do the accumulation of sample values in the ISR.
//       Run the sensor output routine about once for every N accumulated samples.
// IDEA: How about an unsigned (1, 2^M-1) running average (i.e. keep 2^M*Y in a wide
//       register, shift down by M to obtain Y, output Y, subtract Y, add X)?
// IDEA: Output mean square amplitude, let the receiver apply sqrt() if they want to?
ISR(ADC_vect) { // Conversion complete.
	sched_isr_tcww |= SCHED_CATFLAG(SENSOR_TASK_CAT); // Awaken the light meter task.
}

static ttlv_result ttlv_reg_read(ttlv_reg_index index, ttlv_reg_value *value_p) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED states.
		*value_p = led_states;
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
	case REMTCAM_REG_CTRL_FLAGS:
		*value_p = ctrl_flags;
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result ttlv_reg_write(ttlv_reg_index index, ttlv_reg_value value) {
	switch (index) {
	case TTLV_REG_DEBUG0:  // LED states.
		led_states = value;
		sched_task_tcww |= SCHED_CATFLAG(LED_TASK_CAT);  // Awaken the LED task.
		break;
	case REMTCAM_REG_CTRL_FLAGS:
		ctrl_flags = value;
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

static ttlv_result remtcam_get_var(
	remtcam_var_id id, remtcam_var_index index, uint8_t *value_len_p, void *value_p)
{
	switch (id) {
	case REMTCAM_VAR_DEBUG0:  // LED state.
		switch (index) {
		case 0:
			*((uint8_t*)value_p) = 1 & (led_states >> LED_PIN0);
			break;
		case 1:
			*((uint8_t*)value_p) = 1 & (led_states >> LED_PIN1);
			break;
		default:
			return REMTCAM_RES_INDEX;
		}
		*value_len_p = sizeof(uint8_t);
		break;
	case REMTCAM_SENSOR_VAR_T_LIGHT:
		if (index != 0)
			return REMTCAM_RES_INDEX;
		*((uint16_t*)value_p) = light_meter_state;
		*value_len_p = sizeof(uint16_t);
		break;
	case REMTCAM_SENSOR_VAR_T_SOUND:
		if (index != 0)
			return REMTCAM_RES_INDEX;
		*((uint16_t*)value_p) = prev_microphone_state;
		*value_len_p = sizeof(uint16_t);
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}

static ttlv_result remtcam_set_var(
	remtcam_var_id id, remtcam_var_index index, uint8_t value_len, const void *value_p)
{
	switch (id) {
	case REMTCAM_VAR_DEBUG0:  // LED state.
		if (value_len == 0)
			return REMTCAM_RES_VALUE;
		
		switch (index) {
		case 0:
			led_states &= ~BV(LED_PIN0);
			led_states |= (1 & *((uint8_t*)value_p)) << LED_PIN0;
			sched_task_tcww |= SCHED_CATFLAG(LED_TASK_CAT);  // Awaken the LED task.
			break;
		case 1:
			led_states &= ~BV(LED_PIN1);
			led_states |= (1 & *((uint8_t*)value_p)) << LED_PIN1;
			sched_task_tcww |= SCHED_CATFLAG(LED_TASK_CAT);  // Awaken the LED task.
			break;
		default:
			return REMTCAM_RES_INDEX;
		}
		break;
	default:
		return TTLV_RES_REGISTER;
	}
	
	return TTLV_RES_OK;
}


// ---<<< Task Handlers >>>---
static void command_handler(sched_task *task) {
	ttlv_result res = TTLV_RES_NONE;
	
	if (!TTLV_HAS_MESSAGE) {
		task->st |= TASK_ST_SLP(1); // Set sleep flag.
		return;
	}
	
	if (TTLV_CHECK_INM_RESULT) { // Received an INM result response.
		ttlv_finish_recv();
		LED_PINR = BV(LED_PIN_DEBUG); // Helpfully toggle the debug LED.
	}
	else if (TTLV_CHECK_REG_READ) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r.index = msg_data_in.b[0];
		res = ttlv_reg_read(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_REG_READ_RES, TTLV_MSG_L_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_WRITE) {
		ttlv_recv(msg_data_in.b);
		
		res = ttlv_reg_write(msg_data_in.r.index, msg_data_in.r.value);
	}
	else if (TTLV_CHECK_REG_TOGGLE) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r = msg_data_in.r;
		res = ttlv_reg_toggle(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_REG_READ_RES, TTLV_MSG_L_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_RW_EXCH) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r = msg_data_in.r;
		res = ttlv_reg_rw_exch(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_REG_READ_RES, TTLV_MSG_L_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REG_WR_EXCH) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.r = msg_data_in.r;
		res = ttlv_reg_wr_exch(msg_data_out.r.index, &msg_data_out.r.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_REG_READ_RES, TTLV_MSG_L_REG_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (TTLV_CHECK_REGPAIR_READ) {
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.rp.index = msg_data_in.b[0];
		res = ttlv_regpair_read(msg_data_out.rp.index, &msg_data_out.rp.value);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(TTLV_MSG_T_REGPAIR_READ_RES, TTLV_MSG_L_REGPAIR_READ_RES, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (REMTCAM_CHECK_GET_VAR) {
		uint8_t value_len;
		
		ttlv_recv(msg_data_in.b);
		
		msg_data_out.v = msg_data_in.v;
		res = remtcam_get_var(
			msg_data_out.v.id, msg_data_out.v.index, &value_len,
			msg_data_out.b + REMTCAM_MSG_L_GET_VAR_RES);
		
		if (res == TTLV_RES_OK) {
			ttlv_xmit_response(
				REMTCAM_MSG_T_GET_VAR_RES, REMTCAM_MSG_L_GET_VAR_RES + value_len, msg_data_out.b);
			res = TTLV_RES_NONE;
		}
	}
	else if (REMTCAM_CHECK_SET_VAR) {
		uint8_t value_len = ttlv_recv_header.h.length - REMTCAM_MSG_L_SET_VAR;
		
		ttlv_recv(msg_data_in.b);
		
		res = remtcam_set_var(
			msg_data_in.v.id, msg_data_in.v.index, value_len,
			msg_data_in.b + REMTCAM_MSG_L_SET_VAR);
	}
	else { // Unrecognized message.
		// NOTE: Probably not a good idea to always send error messages in
		//       response to unrecognized messages. Doing so can easily
		//       trigger self-sustaining message cascades.
		ttlv_finish_recv();
	}
	
	if (res != TTLV_RES_OK && res != TTLV_RES_NONE)
		ttlv_xmit_inm_result(res);
}

static void led_handler(sched_task *task) {
	uint8_t tmp_led_states;
	
	// Update LED 0.
	if (LED_INPUT_RISING & BV(LED_INPUT_PIN0)) { // Detect input event.
		led_states ^= BV(LED_PIN0); // Change LED state.
		tmp_led_states = 1 & (led_states >> LED_PIN0);
		
		// Send blink state update.
		msg_data_out.v.id = REMTCAM_VAR_DEBUG0;
		msg_data_out.v.index = 0;
		msg_data_out.b[REMTCAM_MSG_L_SET_VAR] = tmp_led_states;
		
		ttlv_xmit(INM_DST_ADR, REMTCAM_MSG_T_SET_VAR, REMTCAM_MSG_L_SET_VAR + 1,
			msg_data_out.b);
	}
	
	// Update LED 1.
	if (LED_INPUT_RISING & BV(LED_INPUT_PIN1)) { // Detect input event.
		led_states ^= BV(LED_PIN1); // Change LED state.
		tmp_led_states = 1 & (led_states >> LED_PIN1);
		
		// Send blink state update.
		msg_data_out.v.id = REMTCAM_VAR_DEBUG0;
		msg_data_out.v.index = 1;
		msg_data_out.b[REMTCAM_MSG_L_SET_VAR] = tmp_led_states;
		
		ttlv_xmit(INM_DST_ADR, REMTCAM_MSG_T_SET_VAR, REMTCAM_MSG_L_SET_VAR + 1,
			msg_data_out.b);
	}
	
	LED_PORT = led_states | (LED_PORT & ~LED_PIN_MASK);
	
	task->st |= TASK_ST_SLP(1); // Set sleep flag.
}

static remtcam_sensor_id select_sensor(uint8_t count, uint8_t flags) {
	remtcam_sensor_id next_sensor = NO_SENSOR_ID;
	
	switch (count) {
	case COUNT_LIGHT_METER:
		next_sensor = ID_LIGHT_METER;
		break;
	// TODO: etc.
	}
	
	// If not at the sampling slot of an enabled slow sensor...
	if (next_sensor == NO_SENSOR_ID || !(flags & BV(next_sensor))) {
		// ...then sample the fast sensor if it's enabled, otherwise be idle
		// during the next interval.
		if (flags & BV(ID_MICROPHONE))
			next_sensor = ID_MICROPHONE;
		else
			next_sensor = NO_SENSOR_ID;
	}
	
	return next_sensor;
}

static uint16_t update_microphone(uint16_t state, uint16_t adc_value) {
	int16_t tmp_value = (int16_t)(adc_value);    // [0,1023]
	tmp_value -= MICROPHONE_BIAS;                // [-512,511]
	tmp_value = abs(tmp_value);                  // [0,512]
	
	if (tmp_value >= MICROPHONE_BIAS)            // [0,511]
		tmp_value = MICROPHONE_BIAS - 1;
	
	return state + ((uint16_t)(tmp_value) >> 1); // [0,255]
}

// NOTE: This handler samples the "fast" microphone input repetitively,
//       occasionally switching the ADC muxer to the light sensor or some
//       other "slow" input. Skipping a few mic samples should not make much
//       difference.
// NOTE: It is stated in the datasheet that in free-running mode, the muxer
//       setting can be reliably updated soon after one conversion has finished
//       and another one automatically started (i.e. in the ADC ISR). There
//       will be a one-sampling-interval delay before the new muxer setting
//       is applied (i.e. after sample N-1 has been obtained, change the
//       muxer setting in the ISR to get sample N+1 from the new source
//       channel).
// NOTE: In free-running mode, it is NOT necessary to clear the
//       ADIF flag to keep the ADC running. (In any case, the hardware will
//       clear the flag automatically.)
// NOTE: "The ADSC bit will be read as one during a conversion,
//        independently of how the conversion was started."
static void sensor_handler(sched_task *task) {
	remtcam_sensor_id curr_sensor_tmp = curr_sensor;
	remtcam_sensor_id next_sensor;
	uint8_t count = sensor_counter;
	
	// Update the sensor sampling interval counter. Rollover is intended.
	count++;
	sensor_counter = count;
	
	if (curr_sensor_tmp != NO_SENSOR_ID) {
		// Fetch the ADC conversion result value.
		uint16_t adc_value = ADC;
		
		// Determine which sensor was sampled.
		// Process the result value and update the sensor state variable.
		switch (curr_sensor_tmp) {
		case ID_MICROPHONE:
			microphone_state = update_microphone(microphone_state, adc_value);
			break;
		case ID_LIGHT_METER:
			light_meter_state = adc_value;
			msg_data_out.s.sensor_type = REMTCAM_SENSOR_T_LIGHT;
			// Convert the 10-bit ADC result to an 8-bit value.
			msg_data_out.s.value = (remtcam_sensor_value)(adc_value >> 2);
			break;
		// TODO: etc.
		}
		
		// NOTE: This code produces ~40 Hz lap/message rate (at ~10 kHz ADC sampling rate),
		//       which should be fast enough to catch most clangs and bangs.
		if (count == COUNT_MICROPHONE) {
			msg_data_out.s.sensor_type = REMTCAM_SENSOR_T_SOUND;
			msg_data_out.s.value = (remtcam_sensor_value)(microphone_state >> 8);
			prev_microphone_state = microphone_state;
			microphone_state = 0;
		}
		
		// NOTE: Avoid spamming samples from the fast sensor (i.e. the microphone).
		// IDEA: Implement configurable 1-in-N message rate throttling (per sensor?).
		//       Remember and report the min and max readings for each message interval.
		if (curr_sensor_tmp != ID_MICROPHONE || count == COUNT_MICROPHONE) {
			// Send a sensor sample message to the control node.
			// IDEA: OR in some ID prefix here if necessary.
			msg_data_out.s.sensor_id = curr_sensor_tmp;
			ttlv_xmit(INM_DST_ADR, REMTCAM_MSG_T_SENSOR_SAMPLE, REMTCAM_MSG_L_SENSOR_SAMPLE,
				msg_data_out.b);
		}
	}
	
	// Determine which sensor to sample next (if any).
	next_sensor = select_sensor(count, ctrl_flags);
	
	if (next_sensor != NO_SENSOR_ID) {
		// ISSUE: Sampling speed/regularity optimize by doing this in the ISR?
		//        The big problem is how to prevent or deal with the next conversion
		//        finishing before the task handler has dealt with the previous one.
		// NOTE: This basically only matters for the fast sensor. How much does it matter
		//       when we only need a ballpark value for the mean square amplitude?
		
		// Use Vcc reference. Right adjust conversion result.
		// Get input from PC/ADC[next_sensor].
		ADMUX = BV(REFS0) | next_sensor;
		
		// Start a new ADC conversion.
		ADCSRA |= BV(ADSC);
		
		task->st |= TASK_ST_SLP(1); // Set the task sleep flag.
	}
	else {
		// IDEA: Power (and wear?) optimize by disabling the ADC?
		//       Or only when all sensors are disabled?
		
		// Delay the sensor handler by (approximately) one ADC sampling interval.
		task->delay = SCHED_TIME_MUSECS(SENSOR_INTERVAL_MUSECS);
	}
	
	curr_sensor = next_sensor;
}


// ---<<< Initialization Routines >>>---
static void init_adc(void) {
	// Configure ADC reference, input, frequency and mode.
	// Use Vcc reference. Right adjust conversion result. Get input from PC0/ADC0.
	ADMUX = BV(REFS0);
	// Enable the ADC and the ADC interrupt. Disable auto-triggering.
	// Prescaler divisor 128 (i.e. 125kHz@16MHz ADC clock, ~10kHz@16MHz sampling rate).
	ADCSRA = BV(ADEN) | BV(ADIE) | BV(ADPS2) | BV(ADPS1) | BV(ADPS0);
	// Don't bother the digital input buffers on the sensor pins with analog weirdness.
	// This is mostly a power conservation nicety.
	DIDR0 = BV(ADC0D) | BV(ADC1D);
}

static void init_tasks(void) {
	sched_task task;
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, COMMAND_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = command_handler
	};
	sched_add(&task);
	
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, LED_TASK_CAT, 0),
		.delay = SCHED_TIME_ZERO,
		.handler = led_handler
	};
	sched_add(&task);
	
	init_adc();
	task = (sched_task) {
		.st = TASK_ST_MAKE(0, SENSOR_TASK_CAT, 0),
		.delay = SCHED_TIME_MUSECS(SENSOR_INTERVAL_MUSECS),
		.handler = sensor_handler
	};
	sched_add(&task);
}

/*static void init_monitors(void) {
	memmon_spec spec;
	
	spec = (memmon_spec) { // Watch scheduler time deltas.
		.ptr = MEMMON_MAKE_PTR(&sched_delta),
		.size = sizeof(sched_delta),
		.delay_div = 21
	};
	memmon_add(&spec);
	
	spec = (memmon_spec) { // Watch light meter output.
		.ptr = MEMMON_MAKE_PTR(&light_meter_value),
		.size = sizeof(light_meter_value),
		.delay_div = 5
	};
	memmon_add(&spec);
	
	/ *spec = (memmon_spec) { // Watch free RAM.
		.ptr = MEMMON_MAKE_PTR(&memmon_free_ram),
		.size = sizeof(memmon_free_ram),
		.delay_div = 21
	};
	memmon_add(&spec);
	
	spec = (memmon_spec) { // Watch memmon message count.
		.ptr = MEMMON_MAKE_PTR(&memmon_msg_count),
		.size = sizeof(memmon_msg_count),
		.delay_div = 31
	};
	memmon_add(&spec);
	
	spec = (memmon_spec) { // Watch memmon message drop count.
		.ptr = MEMMON_MAKE_PTR(&memmon_drop_count),
		.size = sizeof(memmon_drop_count),
		.delay_div = 41
	};
	memmon_add(&spec);* /
}*/


// ---<<< Main Function >>>---
// NOTE: This is the entry point, tell compiler not to save/restore registers.
int main(void) __attribute__ ((OS_main));

int main(void) {
	// Set blinky LED pins as outputs.
	LED_DDR |= BV(LED_PIN0) | BV(LED_PIN1);
	
	LED_DDR |= BV(LED_PIN_DEBUG); // DEBUG: Debugging indicator LED.
	
	// Enable pull-ups on input pins.
	LED_INPUT_PORT |= BV(LED_INPUT_PIN0) | BV(LED_INPUT_PIN1);
	
	sched_init();
	
	TBOUNCER_INIT(
		TASK_ST_MAKE(0, NOAWAKE_TASK_CAT, 0), SCHED_TIME_MS(10),
		BV(LED_INPUT_PIN0) | BV(LED_INPUT_PIN1), 0, 0,
		0, TASK_ST_CAT_MASK, TASK_ST_CAT(LED_TASK_CAT));
	
	ttlv_init(
		TASK_ST_MAKE(0, TTLV_TASK_CAT, 0),
		BAUD_TO_UBRR(BAUD_RATE, USE_U2X), TTLV_PARITY_NONE, USE_U2X,
		TTLV_MODE_INM, 0, SCHED_CATFLAG(COMMAND_TASK_CAT));
	
	/*memmon_init(
		TASK_ST_MAKE(1, NOAWAKE_TASK_CAT, 0), MAX_MEMORY_MONITORS, memory_monitors,
		SCHED_TIME_MS(100), TTLV_MSG_T_MEMMON_DATA, INM_DST_ADR_MEMMON);*/
	
	ttlv_xmit_inm_header.h.srcadr = INM_SRC_ADR; // Set INM source address.
	
	init_tasks();
	
	//init_monitors();
	
	sched_run();
	
	return 0;
}
