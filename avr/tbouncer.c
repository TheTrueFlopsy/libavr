
#include <stddef.h>
#include <avr/io.h>

#include "tbouncer.h"

#ifndef TBOUNCER_DISABLE_PORT_A
uint8_t tbouncer_a_mask; // Change notification pin mask.
uint8_t tbouncer_a; // Debounced pin values.
uint8_t tbouncer_a_prev; // Previous debounced pin values.
uint8_t tbouncer_a_diff; // Pin change flags.
#endif

#ifndef TBOUNCER_DISABLE_PORT_B
uint8_t tbouncer_b_mask; // Change notification pin mask.
uint8_t tbouncer_b; // Debounced pin values.
uint8_t tbouncer_b_prev; // Previous debounced pin values.
uint8_t tbouncer_b_diff; // Pin change flags.
#endif

#ifndef TBOUNCER_DISABLE_PORT_C
uint8_t tbouncer_c_mask; // Change notification pin mask.
uint8_t tbouncer_c; // Debounced pin values.
uint8_t tbouncer_c_prev; // Previous debounced pin values.
uint8_t tbouncer_c_diff; // Pin change flags.
#endif

#ifndef TBOUNCER_DISABLE_PORT_D
uint8_t tbouncer_d_mask; // Change notification pin mask.
uint8_t tbouncer_d; // Debounced pin values.
uint8_t tbouncer_d_prev; // Previous debounced pin values.
uint8_t tbouncer_d_diff; // Pin change flags.
#endif

sched_catflags tbouncer_task_cats;
uint8_t tbouncer_invoke_mask;
uint8_t tbouncer_invoke_st;

static uint8_t tbouncer_poll_num_cat;
static sched_time tbouncer_poll_delay;


// ISSUE: This does not do any spike filtering.
// ISSUE: This does not do any edge detection, so button presses
//        that are shorter than the polling delay may be missed.
// IDEA: Use a shorter polling delay and require N consecutive
//       samples to be different from current output to change
//       current output.
// IDEA: Require M sample intervals between output changes.
// NOTE: The combined M/N counting technique provides quick reactions
//       (via a short polling interval), dodges switch bounce (via M)
//       and filters out most spikes (via N).
static void tbouncer_handler(sched_task *task) {
	uint8_t diff = 0;
	
	task->delay = tbouncer_poll_delay; // Refresh polling delay.
	
#ifndef TBOUNCER_DISABLE_PORT_A
	tbouncer_a_prev = tbouncer_a;
	tbouncer_a = tbouncer_a_mask & PINA;
	tbouncer_a_diff = tbouncer_a ^ tbouncer_a_prev;
	diff |= tbouncer_a_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_B
	tbouncer_b_prev = tbouncer_b;
	tbouncer_b = tbouncer_b_mask & PINB;
	tbouncer_b_diff = tbouncer_b ^ tbouncer_b_prev;
	diff |= tbouncer_b_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_C
	tbouncer_c_prev = tbouncer_c;
	tbouncer_c = tbouncer_c_mask & PINC;
	tbouncer_c_diff = tbouncer_c ^ tbouncer_c_prev;
	diff |= tbouncer_c_diff;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_D
	tbouncer_d_prev = tbouncer_d;
	tbouncer_d = tbouncer_d_mask & PIND;
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
	tbouncer_a = a_mask & PINA;
	tbouncer_a_prev = tbouncer_a;
	tbouncer_a_diff = 0;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_B
	tbouncer_b_mask = b_mask;
	tbouncer_b = b_mask & PINB;
	tbouncer_b_prev = tbouncer_b;
	tbouncer_b_diff = 0;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_C
	tbouncer_c_mask = c_mask;
	tbouncer_c = c_mask & PINC;
	tbouncer_c_prev = tbouncer_c;
	tbouncer_c_diff = 0;
#endif
	
#ifndef TBOUNCER_DISABLE_PORT_D
	tbouncer_d_mask = d_mask;
	tbouncer_d = d_mask & PIND;
	tbouncer_d_prev = tbouncer_d;
	tbouncer_d_diff = 0;
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
