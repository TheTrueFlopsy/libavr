
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#ifdef SCHED_RATE_LIMITING
#include <util/delay_basic.h>
#endif

#include "task_sched.h"

volatile sched_catflags sched_isr_tcww; // ISR-Task Category Wakeup Word (I-TCWW).

sched_catflags sched_task_tcww; // Task-Task Category Wakeup Word (T-TCWW).

sched_time sched_ticks; // Scheduler iteration timestamp.

sched_time sched_delta; // Scheduler iteration time delta.

uint8_t sched_list_size; // Number of scheduled tasks in task_list.

volatile uint16_t sched_tick_count_h; // High bytes of the timer tick count.

static sched_task task_list[SCHED_MAX_TASKS];

#ifdef SCHED_RATE_LIMITING
static sched_time sched_delay; // Scheduler rate limiting delay.
#endif

#ifndef SCHED_NO_ISR
ISR(TIMER2_OVF_vect) { // The tick counter has overflowed.
	sched_tick_count_h++; // Add 1 to the high bytes of the tick count.
}
#endif

uint8_t sched_time_is_zero(sched_time t) { // Tests whether T == 0.
	return t.h == 0 && t.l == 0;
}

uint8_t sched_time_gt(sched_time a, sched_time b) { // Tests whether A > B.
	return (a.h == b.h) ? (a.l > b.l) : (a.h > b.h);
}

// TODO: Verify that this produces the correct difference even when
//       the subtraction overflows.
sched_time sched_time_sub(sched_time a, sched_time b) { // Subtracts B from A.
	a.h -= b.h;
	if (b.l > a.l) // Must borrow.
		a.h--;       // Borrow from 'a.h'.
	a.l -= b.l;
	return a;
}

void sched_init(void) {
	sched_isr_tcww = 0;
	sched_task_tcww = 0;
	sched_ticks = SCHED_TIME_ZERO;
	sched_delta = SCHED_TIME_ZERO;
	sched_list_size = 0;
	
	sched_tick_count_h = 0;
	
#ifdef SCHED_RATE_LIMITING
	sched_delay = SCHED_TIME_ZERO;
#endif
	
	// Configure the scheduler's tick counter.
	TCNT2 = 0; // Clear the count.
	TIMSK2 = BV(TOIE2); // Enable overflow interrupt.
	TCCR2A = 0; // No waveform generation, normal mode.
	TCCR2B = 0; // Normal mode, counter stopped.
}

uint8_t sched_query(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	uint8_t i;
	st_val &= st_mask;
	
	for (i = start_i; i < sched_list_size; i++) {
		uint8_t masked_st = st_mask & task_list[i].st;
		if (masked_st == st_val)
			return i;
	}
	
	return SCHED_MAX_TASKS;
}

uint8_t sched_invoke(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	uint8_t i = sched_query(st_mask, st_val, start_i);
	
	if (i < SCHED_MAX_TASKS) {
		sched_task *task = task_list + i;
		task->st &= ~TASK_ST_SLP_MASK;
		task->handler(task);
	}
	
	return i;
}

void sched_invoke_all(uint8_t st_mask, uint8_t st_val) {
	uint8_t i = 0xff;
	
	do {
		i = sched_invoke(st_mask, st_val, i+1);
	} while (i < SCHED_MAX_TASKS);
}

sched_task *sched_get_at(uint8_t i) {
	return (i < sched_list_size) ? task_list + i : NULL;
}

uint8_t sched_add(const sched_task *task) {
	uint8_t i = sched_list_size;
	if (i >= SCHED_MAX_TASKS)
		return SCHED_MAX_TASKS;
	
	sched_list_size++;
	task_list[i] = *task;
	return i;
}

uint8_t sched_remove(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	uint8_t i = sched_query(st_mask, st_val, start_i);
	
	if (i < SCHED_MAX_TASKS) {
		sched_task *task = task_list + i;
		task->st = TASK_ST_GARBAGE;
	}
	
	return i;
}

void sched_run(void) {
	uint16_t tcww;
	sched_time delta;
	uint8_t n_ready, n_garbage;
	uint8_t i, k;
	sched_task *task_k1_p, *task_k0_p;
	sched_task task_i;
	
	sei(); // Ensure that interrupts are enabled.
	
	// Start the tick counter with prescaler divisor 32.
	// TODO: Make the divisor compile-time configurable.
	TCCR2B |= BV(CS21) | BV(CS20);
	
	// Main scheduler loop.
	while (1) {
		// ISSUE: If current tasks finish very quickly, then this will
		//        be executed very often and prevent timely ISR execution.
		//        The minimal number of instructions executed in the rest
		//        of the scheduler loop might be large enough to prevent
		//        this problem from becoming critical. However, having
		//        a way to limit the scheduler iteration rate can be
		//        useful anyway.
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// Get and clear the ISR-Task Category Wakeup Word.
			tcww = sched_isr_tcww;
			sched_isr_tcww = 0;
			
			// Get TCNT2 as seen /before/ checking for overflow.
			delta.l = TCNT2;
			
			// Get 'sched_tick_count_h'.
			delta.h = sched_tick_count_h;
			
			// NOTE: Timer counts are NOT stopped by disabling interrupts, so TCNT2
			//       may actually overflow /inside/ the atomic block, and in that case,
			//       there obviously won't be any update of 'sched_tick_count_h' until after
			//       it has been read into 'delta.h', and then it will be /too late/.
			if (BV(TOV2) & TIFR2) { // A timer overflow has happened sometime /between/ cli() and now.
				// Clear the overflow interrupt flag, because we'll update 'sched_tick_count_h' here.
				TIFR2 = BV(TOV2); // Clear-by-setting semantics.
				
				// Get TCNT2 as seen /after/ the overflow, hopefully before the next one.
				delta.l = TCNT2;
				
				// Increment and write back 'sched_tick_count_h'.
				delta.h++;
				sched_tick_count_h = delta.h;
			}
			// else: Keep the TCNT2 value we stored /before/ checking for and /not seeing/ an overflow.
		}
		
		// Get and clear the Task-Task Category Wakeup Word.
		tcww |= sched_task_tcww;
		sched_task_tcww = 0;
		
		// Update scheduler timing fields.
		// CAUTION: It is imperative that 'sched_ticks' never holds a value that
		//          is slightly /ahead/ of 'delta', because that makes the subtraction
		//          below produce an incorrect and enormous result.
		sched_delta = sched_time_sub(delta, sched_ticks);
		sched_ticks = delta;
		delta = sched_delta;
		
		// Refresh the task list.
		n_garbage = 0;
		n_ready = 0;
		
		for (i = 0; i < sched_list_size; i++) {
			task_k1_p = task_list + i;
			task_i = *task_k1_p;
			
			// If bit number 'task_i.st.cat' in 'tcww' is set,
			//  clear the sleep flag in 'task_i.st'.
			if (tcww & SCHED_CATFLAG(TASK_ST_GET_CAT(task_i.st))) {
				task_i.st &= ~TASK_ST_SLP_MASK;
				task_k1_p->st = task_i.st;
			}
			
			if (task_i.st == TASK_ST_GARBAGE)
				n_garbage++; // Count the number of garbage tasks.
			else if (!(TASK_ST_SLP_MASK & task_i.st))
				n_ready++; // Count the number of ready tasks.
			
			// Perform an insertion sort on the task list.
			// Sort by the unsigned integer value of the status field, in ascending order.
			for (k = i-1; k < SCHED_MAX_TASKS; k--) {
				task_k0_p = task_list + k;
				
				if (task_i.st < task_k0_p->st) { // 'task_i' goes before 'task_k'.
					*task_k1_p = *task_k0_p; // Move 'task_k' one step endward.
					task_k1_p = task_k0_p; // The task list slot that 'task_k' occupied is now available.
				}
				else // 'task_i' goes after 'task_k'.
					break;
			}
			// NOTE: If the iteration ends because k == (0 - 1), then 'task_i' belongs at task_list[0].
			
			if (k+1 != i) // If 'task_i' isn't already in the right place...
				*task_k1_p = task_i; // ...put it at task_list[k+1].
		}
		
		sched_list_size -= n_garbage; // Remove any garbage tasks at the end of the list.
		
		// For each of the first 'n_ready' tasks on the task list
		// (i.e. the ones that were ready at the start of this
		// scheduler iteration):
		for (i = 0; i < n_ready; i++) {
			task_k1_p = task_list + i;
			task_i = *task_k1_p;
			
			if (TASK_ST_SLP_MASK & task_i.st) // Task is sleeping.
				continue; // Let sleeping tasks lie.
			
			if (sched_time_gt(task_i.delay, delta)) { // Task is delayed.
				task_k1_p->delay = sched_time_sub(task_i.delay, delta); // Update the task execution delay.
				
				if (!(tcww & SCHED_CATFLAG(TASK_ST_GET_CAT(task_i.st)))) // Task is not notified.
					continue; // Wait a little longer
			}
			
			// Time to run this task.
			// Clear the task execution delay.
			task_k1_p->delay = SCHED_TIME_ZERO;
			
			// Call the task handler.
			task_i.handler(task_k1_p);
		}
		
#ifdef SCHED_RATE_LIMITING
		// If the time delta (i.e. the execution time of the /previous/
		// scheduler iteration) is less than some specified minimum value,
		// then artificially lengthen /this/ iteration by the difference
		// between the minimum and the measured delta.
		//
		// Remember the artificial delay and subtract it from the time
		// delta when considering the need for artifical delay at the
		// end of the next iteration.
		if (sched_time_gt(sched_delay, delta))
			delta = SCHED_TIME_ZERO;
		else
			delta = sched_time_sub(delta, sched_delay);
		
		if (sched_time_gt(SCHED_MIN_DELTA, delta)) {
			sched_delay = sched_time_sub(SCHED_MIN_DELTA, delta);
			
			// Impose scheduler iteration rate limiting delay.
			delta.h = ((uint16_t)sched_delay.l << (SCHED_CLOCK_PRESCALE_LOG - 2));
			delta.h += sched_delay.h << (SCHED_CLOCK_PRESCALE_LOG + 6); // 8 - 2 = 6.
			_delay_loop_2(delta.h); // Four cycles per step.
		}
		else // No delay.
			sched_delay = SCHED_TIME_ZERO;
#endif
	}
}
