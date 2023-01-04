
#include <stddef.h>
#include <string.h>

#include "tbouncer.h"

#ifdef LIBAVR_TEST_BUILD
static uint8_t PINA;
static uint8_t PINB;
static uint8_t PINC;
static uint8_t PIND;
static uint8_t PINF;
static uint16_t sched_task_tcww;

#define sched_invoke_all(ST_MASK, ST_VAL)
#define sched_add(TASK)
#define sched_remove(ST_MASK, ST_VAL, START_I)

#else
#include <avr/io.h>
#endif


// ---<<< Data Types >>>---
#if TBOUNCER_TICK_COUNT_BITS + TBOUNCER_NEQ_COUNT_BITS > 8
#error "Too many debouncer counter bits (max total is 8)."
#endif

#if TBOUNCER_TICKS_FOR_UPDATE > TBOUNCER_TICKS_FOR_UPDATE_MAX
#error "Ticks-for-update value above maximum allowed by counter field size."
#endif

#if TBOUNCER_NEQ_FOR_UPDATE > TBOUNCER_NEQ_FOR_UPDATE_MAX
#error "Inequal-for-update value above maximum allowed by counter field size."
#endif

typedef struct tbouncer_pin_data {
	uint8_t tick_count : TBOUNCER_TICK_COUNT_BITS;
	uint8_t neq_count : TBOUNCER_NEQ_COUNT_BITS;
} tbouncer_pin_data;


// ---<<< Data >>>---
// NOTE: Port C sucks on the ATmegaU (only two pins), so we pretend that Port F is Port C
//       and that Port C is Port A. (Confusing, I know, but /they started it/ with this weird
//       port layout.)

#ifndef TBOUNCER_DISABLE_PORT_A
#ifdef LIBAVR_ATMEGA_U
#define TBOUNCER_PINA PINC
#else
#define TBOUNCER_PINA PINA
#endif
uint8_t tbouncer_a_mask;  // Change notification pin mask.
uint8_t tbouncer_a;       // Debounced pin values.
uint8_t tbouncer_a_prev;  // Previous debounced pin values.
uint8_t tbouncer_a_diff;  // Pin change flags.

static tbouncer_pin_data tbouncer_a_pin_data[8];  // Debouncing counters.
#endif

#ifndef TBOUNCER_DISABLE_PORT_B
#define TBOUNCER_PINB PINB
uint8_t tbouncer_b_mask;
uint8_t tbouncer_b;
uint8_t tbouncer_b_prev;
uint8_t tbouncer_b_diff;

static tbouncer_pin_data tbouncer_b_pin_data[8];
#endif

#ifndef TBOUNCER_DISABLE_PORT_C
#ifdef LIBAVR_ATMEGA_U
#define TBOUNCER_PINC PINF
#else
#define TBOUNCER_PINC PINC
#endif
uint8_t tbouncer_c_mask;
uint8_t tbouncer_c;
uint8_t tbouncer_c_prev;
uint8_t tbouncer_c_diff;

static tbouncer_pin_data tbouncer_c_pin_data[8];
#endif

#ifndef TBOUNCER_DISABLE_PORT_D
#define TBOUNCER_PIND PIND
uint8_t tbouncer_d_mask;
uint8_t tbouncer_d;
uint8_t tbouncer_d_prev;
uint8_t tbouncer_d_diff;

static tbouncer_pin_data tbouncer_d_pin_data[8];
#endif

sched_catflags tbouncer_task_cats;
uint8_t tbouncer_invoke_mask;
uint8_t tbouncer_invoke_st;

static uint8_t tbouncer_poll_num_cat;
static sched_time tbouncer_poll_delay;


// ---<<< Functions >>>---
// NOTE: The combined M/N counting technique provides quick reactions
//       (via a short polling interval), dodges switch bounce (via M)
//       and filters out most spikes (via N).
static uint8_t tbouncer_pin_update(tbouncer_pin_data *data_p, uint8_t curr_neq_in) {
	//tbouncer_pin_data data = *data_p;
	uint8_t tick_count = data_p->tick_count;
	uint8_t out_change = 0;
	
	// NOTE: Require M ticks between output changes.
	if (tick_count >= TBOUNCER_TICKS_FOR_UPDATE - 1) {
		if (curr_neq_in) {
			uint8_t neq_count = data_p->neq_count;
			
			// NOTE: Require N consecutive samples to be different from current output
      //       to change current output.
			if (neq_count >= TBOUNCER_NEQ_FOR_UPDATE - 1) {
				*data_p = (tbouncer_pin_data) { .tick_count=0, .neq_count=0 };
				out_change = 1;
			}
			else
				data_p->neq_count = neq_count + 1;
		}
		else
			data_p->neq_count = 0;
	}
	else
		data_p->tick_count = tick_count + 1;
	
	return out_change;
}

static void tbouncer_handler(sched_task *task) {
	uint8_t diff = 0;
	uint8_t curr_neq_in, i, bit_i;
	
	task->delay = tbouncer_poll_delay; // Refresh polling delay.
	
#ifndef TBOUNCER_DISABLE_PORT_A
	tbouncer_a_prev = tbouncer_a;
	curr_neq_in = tbouncer_a ^ TBOUNCER_PINA;
	
	for (i = 0, bit_i = 1; i < 8; i++, bit_i <<= 1) {
		if ((tbouncer_a_mask & bit_i) && tbouncer_pin_update(tbouncer_a_pin_data + i, 1 & curr_neq_in))
			tbouncer_a ^= bit_i;
		curr_neq_in >>= 1;
	}
	
	tbouncer_a_diff = tbouncer_a ^ tbouncer_a_prev;
	diff |= tbouncer_a_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_B
	tbouncer_b_prev = tbouncer_b;
	curr_neq_in = tbouncer_b ^ TBOUNCER_PINB;
	
	for (i = 0, bit_i = 1; i < 8; i++, bit_i <<= 1) {
		if ((tbouncer_b_mask & bit_i) && tbouncer_pin_update(tbouncer_b_pin_data + i, 1 & curr_neq_in))
			tbouncer_b ^= bit_i;
		curr_neq_in >>= 1;
	}
	
	tbouncer_b_diff = tbouncer_b ^ tbouncer_b_prev;
	diff |= tbouncer_b_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_C
	tbouncer_c_prev = tbouncer_c;
	curr_neq_in = tbouncer_c ^ TBOUNCER_PINC;
	
	for (i = 0, bit_i = 1; i < 8; i++, bit_i <<= 1) {
		if ((tbouncer_c_mask & bit_i) && tbouncer_pin_update(tbouncer_c_pin_data + i, 1 & curr_neq_in))
			tbouncer_c ^= bit_i;
		curr_neq_in >>= 1;
	}
	
	tbouncer_c_diff = tbouncer_c ^ tbouncer_c_prev;
	diff |= tbouncer_c_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_D
	tbouncer_d_prev = tbouncer_d;
	curr_neq_in = tbouncer_d ^ TBOUNCER_PIND;
	
	for (i = 0, bit_i = 1; i < 8; i++, bit_i <<= 1) {
		if ((tbouncer_d_mask & bit_i) && tbouncer_pin_update(tbouncer_d_pin_data + i, 1 & curr_neq_in))
			tbouncer_d ^= bit_i;
		curr_neq_in >>= 1;
	}
	
	tbouncer_d_diff = tbouncer_d ^ tbouncer_d_prev;
	diff |= tbouncer_d_diff;
#endif
	
	if (diff) {
		sched_task_tcww |= tbouncer_task_cats;
		if (tbouncer_invoke_mask != 0)
			sched_invoke_all(tbouncer_invoke_mask, tbouncer_invoke_st);
	}
}

void tbouncer_init(
	uint8_t task_num_cat, sched_time poll_delay,
	uint8_t a_mask, uint8_t b_mask, uint8_t c_mask, uint8_t d_mask,
	sched_catflags awaken_cats, uint8_t invoke_mask, uint8_t invoke_st)
{
	sched_task task;
	
	// Initialize debouncer state.
	tbouncer_task_cats = awaken_cats;
	tbouncer_invoke_mask = invoke_mask;
	tbouncer_invoke_st = invoke_st;
	
#ifndef TBOUNCER_DISABLE_PORT_A
	tbouncer_a_mask = a_mask;
	tbouncer_a = a_mask & TBOUNCER_PINA;
	tbouncer_a_prev = tbouncer_a;
	tbouncer_a_diff = 0;
	memset(tbouncer_a_pin_data, 0, sizeof(tbouncer_a_pin_data));
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_B
	tbouncer_b_mask = b_mask;
	tbouncer_b = b_mask & TBOUNCER_PINB;
	tbouncer_b_prev = tbouncer_b;
	tbouncer_b_diff = 0;
	memset(tbouncer_b_pin_data, 0, sizeof(tbouncer_b_pin_data));
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_C
	tbouncer_c_mask = c_mask;
	tbouncer_c = c_mask & TBOUNCER_PINC;
	tbouncer_c_prev = tbouncer_c;
	tbouncer_c_diff = 0;
	memset(tbouncer_c_pin_data, 0, sizeof(tbouncer_c_pin_data));
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_D
	tbouncer_d_mask = d_mask;
	tbouncer_d = d_mask & TBOUNCER_PIND;
	tbouncer_d_prev = tbouncer_d;
	tbouncer_d_diff = 0;
	memset(tbouncer_d_pin_data, 0, sizeof(tbouncer_d_pin_data));
#endif
	
	task_num_cat &= TASK_ST_NUM_CAT_MASK;
	tbouncer_poll_num_cat = task_num_cat;
	tbouncer_poll_delay = poll_delay;
	
	// Create pin polling task.
	task = (sched_task) {
		.st = task_num_cat,
		.delay = poll_delay,
		.handler = tbouncer_handler
	};
	sched_add(&task);
}

void tbouncer_shutdown(void) {
	sched_remove(TASK_ST_NUM_CAT_MASK, tbouncer_poll_num_cat, 0);
}

#ifdef LIBAVR_TEST_BUILD
void tbouncer_update(uint8_t pina, uint8_t pinb, uint8_t pinc, uint8_t pind, uint8_t pinf) {
	sched_task task;
	
	PINA = pina;
	PINB = pinb;
	PINC = pinc;
	PIND = pind;
	PINF = pinf;
	
	tbouncer_handler(&task);
}
#endif
