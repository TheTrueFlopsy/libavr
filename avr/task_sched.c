
#include <stddef.h>

#ifndef LIBAVR_TEST_BUILD

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>

#ifdef SCHED_RATE_LIMITING
#include <util/delay_basic.h>
#endif

#endif

#include "task_sched.h"


#ifndef LIBAVR_TEST_BUILD

#ifdef LIBAVR_ATTINY

// NOTE: No Timer2 on the ATtiny.
#define SCHED_USE_TIMER0

// NOTE: This thing with inconsistent names in the I/O headers is so dumb.
#ifndef TIMER0_OVF_vect
#define TIMER0_OVF_vect TIM0_OVF_vect
#endif

#endif

#ifdef LIBAVR_ATMEGA_U

// NOTE: No Timer2 on the ATmegaU either.
#define SCHED_USE_TIMER0

#endif

#ifdef SCHED_USE_TIMER0

#define SCHED_CLOCK_PRESCALE_1    (BV(CS00))
#define SCHED_CLOCK_PRESCALE_8    (BV(CS01))
#define SCHED_CLOCK_PRESCALE_64   (BV(CS01) | BV(CS00))
#define SCHED_CLOCK_PRESCALE_256  (BV(CS02))
#define SCHED_CLOCK_PRESCALE_1024 (BV(CS02) | BV(CS00))

#ifndef SCHED_CLOCK_PRESCALE_LOG
#error "SCHED_CLOCK_PRESCALE_LOG must be defined."
#endif

#if SCHED_CLOCK_PRESCALE_LOG == 0
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_1
#elif SCHED_CLOCK_PRESCALE_LOG == 3
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_8
#elif SCHED_CLOCK_PRESCALE_LOG == 6
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_64
#elif SCHED_CLOCK_PRESCALE_LOG == 8
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_256
#elif SCHED_CLOCK_PRESCALE_LOG == 10
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_1024
#else
#error "SCHED_CLOCK_PRESCALE_LOG has an illegal value."
#endif

#define SCHED_TIMER_VECT TIMER0_OVF_vect

#define SCHED_TCNT TCNT0
#define SCHED_TIMSK TIMSK0
#define SCHED_TCCRA TCCR0A
#define SCHED_TCCRB TCCR0B
#define SCHED_TIFR TIFR0

#define SCHED_TIMSK_INIT_VAL BV(TOIE0)
#define SCHED_TCCRA_INIT_VAL 0
#define SCHED_TCCRB_INIT_VAL 0
#define SCHED_TIFR_BIT_MASK BV(TOV0)

#else

#define SCHED_CLOCK_PRESCALE_1    (BV(CS20))
#define SCHED_CLOCK_PRESCALE_8    (BV(CS21))
#define SCHED_CLOCK_PRESCALE_32   (BV(CS21) | BV(CS20))
#define SCHED_CLOCK_PRESCALE_64   (BV(CS22))
#define SCHED_CLOCK_PRESCALE_128  (BV(CS22) | BV(CS20))
#define SCHED_CLOCK_PRESCALE_256  (BV(CS22) | BV(CS21))
#define SCHED_CLOCK_PRESCALE_1024 (BV(CS22) | BV(CS21) | BV(CS20))

#ifndef SCHED_CLOCK_PRESCALE_LOG
#error "SCHED_CLOCK_PRESCALE_LOG must be defined."
#endif

#if SCHED_CLOCK_PRESCALE_LOG == 0
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_1
#elif SCHED_CLOCK_PRESCALE_LOG == 3
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_8
#elif SCHED_CLOCK_PRESCALE_LOG == 5
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_32
#elif SCHED_CLOCK_PRESCALE_LOG == 6
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_64
#elif SCHED_CLOCK_PRESCALE_LOG == 7
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_128
#elif SCHED_CLOCK_PRESCALE_LOG == 8
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_256
#elif SCHED_CLOCK_PRESCALE_LOG == 10
#define SCHED_CLOCK_PRESCALE_BITS SCHED_CLOCK_PRESCALE_1024
#else
#error "SCHED_CLOCK_PRESCALE_LOG has an illegal value."
#endif

#define SCHED_TIMER_VECT TIMER2_OVF_vect

#define SCHED_TCNT TCNT2
#define SCHED_TIMSK TIMSK2
#define SCHED_TCCRA TCCR2A
#define SCHED_TCCRB TCCR2B
#define SCHED_TIFR TIFR2

#define SCHED_TIMSK_INIT_VAL BV(TOIE2)
#define SCHED_TCCRA_INIT_VAL 0
#define SCHED_TCCRB_INIT_VAL 0
#define SCHED_TIFR_BIT_MASK BV(TOV2)

#endif

volatile sched_catflags sched_isr_tcww; // ISR-Task Category Wakeup Word (I-TCWW).

sched_catflags sched_task_tcww; // Task-Task Category Wakeup Word (T-TCWW).

sched_time sched_ticks; // Scheduler iteration timestamp.

sched_time sched_delta; // Scheduler iteration time delta.

uint8_t sched_list_size; // Number of scheduled tasks in task_list.

volatile uint16_t sched_tick_count_h; // High bytes of the timer tick count.

#if SCHED_MAX_TASKS < 1 || SCHED_MAX_TASKS > 254
#error "SCHED_MAX_TASKS has an illegal value."
#endif

static sched_task task_list[SCHED_MAX_TASKS];

#ifdef SCHED_RATE_LIMITING
static sched_time sched_delay;  // Scheduler rate limiting delay.
#endif

#ifndef SCHED_NO_ISR
// IDEA: Replace this with an assembly implementation that doesn't do any
// unnecessary pushing and popping?
ISR(SCHED_TIMER_VECT) {  // The tick counter has overflowed.
	sched_tick_count_h++;  // Add 1 to the high bytes of the tick count.
}
#endif

#endif

uint8_t sched_time_is_zero(sched_time t) {  // Tests whether T == 0.
	return t.h == 0 && t.l == 0;
}

uint8_t sched_time_gt(sched_time a, sched_time b) {  // Tests whether A > B.
	return (a.h == b.h) ? (a.l > b.l) : (a.h > b.h);
}

uint8_t sched_time_lt(sched_time a, sched_time b) {  // Tests whether A < B.
	return sched_time_gt(b, a);
}

uint8_t sched_time_gte(sched_time a, sched_time b) {  // Tests whether A >= B.
	return !sched_time_gt(b, a);
}

uint8_t sched_time_lte(sched_time a, sched_time b) {  // Tests whether A <= B.
	return !sched_time_gt(a, b);
}

sched_time sched_time_add(sched_time a, sched_time b) {  // Adds B to A.
	a.h += ((b.l > (UINT8_MAX - a.l)) ? b.h + 1 : b.h);  // Carry to 'a.h' if necessary.
	a.l += b.l;
	return a;
}

// NOTE: This function performs an operation equivalent to subtraction (with
//       wraparound) of 24-bit unsigned smalltick counts and will always produce
//       a result (in smallticks) that is equal to the minimum number of times
//       that one needs to increment 'b' (with wraparound at b == 2^24) to obtain 'a'.
//       If (a >= b), then this is obvious. If (a < b), then the result is equal
//       to the one that would've been obtained if 'b' had been subtracted from
//       (2^24 + a), which is obviously equal to the number of times to
//       increment 'b' to obtain (2^24 + a). But since the first 24 bits
//       of (2^24 + a) are equal to 'a', this is also the (minimum) number of
//       times to increment 'b' to obtain 'a', taking the wraparound from (2^24 - 1)
//       to 0 into account.
sched_time sched_time_sub(sched_time a, sched_time b) {  // Subtracts B from A.
	a.h -= ((b.l > a.l) ? b.h + 1 : b.h);  // Borrow from 'a.h' if necessary.
	a.l -= b.l;
	return a;
}

#ifndef LIBAVR_TEST_BUILD

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
	
	// Configure the scheduler's tick counter timer.
	SCHED_TCNT = 0; // Clear the count.
	SCHED_TIMSK = SCHED_TIMSK_INIT_VAL; // Enable overflow interrupt.
	SCHED_TCCRA = SCHED_TCCRA_INIT_VAL; // No waveform generation, normal mode.
	SCHED_TCCRB = SCHED_TCCRB_INIT_VAL; // Normal mode, counter stopped.
}

sched_task *sched_ptr_query(
	uint8_t st_mask, uint8_t st_val, uint8_t start_i, uint8_t *task_i_p)
{
	if (st_val == TASK_ST_GARBAGE && st_mask == 0xff)
		return NULL;
	
	st_val &= st_mask;
	
	sched_task *task_p = task_list + start_i;
	
	for (uint8_t i = start_i; i < sched_list_size; i++, task_p++) {
		uint8_t masked_st = st_mask & task_p->st;
		
		if (masked_st == st_val) {
			*task_i_p = i;
			return task_p;
		}
	}
	
	return NULL;
}

uint8_t sched_query(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	sched_task *task_p = sched_ptr_query(st_mask, st_val, start_i, &start_i);
	return (task_p) ? start_i : SCHED_MAX_TASKS;
}

sched_task *sched_find(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	return sched_ptr_query(st_mask, st_val, start_i, &start_i);
}

uint8_t sched_invoke(uint8_t st_mask, uint8_t st_val, uint8_t start_i) {
	sched_task *task_p = sched_ptr_query(st_mask, st_val, start_i, &start_i);
	
	if (!task_p)
		return SCHED_MAX_TASKS;
	else if (task_p->st != TASK_ST_GARBAGE) {  // Detect and refuse to invoke garbage tasks.
		task_p->st &= ~TASK_SLEEP_BIT;  // You are being invoked, wakey wakey.
		task_p->delay = SCHED_TIME_ZERO;  // Delay always zero at handler invocation.
		task_p->handler(task_p);
	}
	
	// NOTE: We return the task index (instead of SCHED_MAX_TASKS) even if
	//       the task we found was garbage and not invoked, to avoid breaking
	//       sched_invoke_all().
	return start_i;
}

void sched_invoke_all(uint8_t st_mask, uint8_t st_val) {
	uint8_t i = UINT8_MAX;
	
	do {
		i = sched_invoke(st_mask, st_val, i+1);
	} while (i < SCHED_MAX_TASKS);
}

uint8_t sched_wake(uint8_t st_mask, uint8_t st_val, uint8_t start_i, uint8_t notify) {
	sched_task *task_p = sched_ptr_query(st_mask, st_val, start_i, &start_i);
	
	if (!task_p)
		return SCHED_MAX_TASKS;
	else if (task_p->st != TASK_ST_GARBAGE) {  // Detect and refuse to wake garbage tasks.
		task_p->st &= ~TASK_SLEEP_BIT;  // Wakey wakey.
		if (notify)
			task_p->delay = SCHED_TIME_ZERO;  // Cancel delay, execute task ASAP.
	}
	
	// NOTE: We return the task index (instead of SCHED_MAX_TASKS) even if
	//       the task we found was garbage and not awakened, to avoid breaking
	//       sched_wake_all().
	return start_i;
}

void sched_wake_all(uint8_t st_mask, uint8_t st_val, uint8_t notify) {
	uint8_t i = UINT8_MAX;
	
	do {
		i = sched_wake(st_mask, st_val, i+1, notify);
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
	sched_task *task_p = sched_ptr_query(st_mask, st_val, start_i, &start_i);
	
	if (!task_p)
		return SCHED_MAX_TASKS;
	
	task_p->st = TASK_ST_GARBAGE;
	return start_i;
}

void sched_run(void) {
	sched_catflags tcww;
	sched_time delta;
	uint8_t n_ready, n_garbage;
	sched_task *task_p;
	
	sei(); // Ensure that interrupts are enabled.
	
	// Start the tick counter with the appropriate prescaler divisor.
	SCHED_TCCRB |= SCHED_CLOCK_PRESCALE_BITS;
	
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
			
			// Get TCNT as seen /before/ checking for overflow.
			delta.l = SCHED_TCNT;
			
			// Get 'sched_tick_count_h'.
			delta.h = sched_tick_count_h;
			
			// NOTE: Timer counts are NOT stopped by disabling interrupts, so TCNT
			//       may actually overflow /inside/ the atomic block, and in that case,
			//       there obviously won't be any update of 'sched_tick_count_h' until after
			//       it has been read into 'delta.h', and then it will be /too late/.
			if (SCHED_TIFR_BIT_MASK & SCHED_TIFR) {  // A timer overflow has happened sometime /between/ cli() and now.
				// Clear the overflow interrupt flag, because we'll update 'sched_tick_count_h' here.
				SCHED_TIFR = SCHED_TIFR_BIT_MASK;  // Clear-by-setting semantics.
				
				// Get TCNT as seen /after/ the overflow, hopefully before the next one.
				delta.l = SCHED_TCNT;
				
				// Increment and write back 'sched_tick_count_h'.
				delta.h++;
				sched_tick_count_h = delta.h;
			}
			// else: Keep the TCNT value we stored /before/ checking for and /not seeing/ an overflow.
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
		task_p = task_list;
		
		for (uint8_t i = 0; i < sched_list_size; i++, task_p++) {
			sched_task *task_k1_p = task_p;  // Current task record pointer.
			sched_task task_i = *task_k1_p;  // Current task record.
			
			// Check task status.
			if (task_i.st == TASK_ST_GARBAGE)  // Task marked as garbage.
				n_garbage++;  // Count the number of garbage tasks.
			else if (!TASK_SLEEP_BIT_SET(task_i.st))  // Task is not sleeping.
				n_ready++;  // Count the number of ready (possibly delayed) tasks.
			else if (tcww & TASK_ST_GET_CATFLAG(task_i.st)) {  // Task notified.
				task_i.st &= ~TASK_SLEEP_BIT;  // Awaken notified task.
				task_k1_p->st = task_i.st;
				n_ready++;  // Task is now ready.
			}
			
			// Perform an insertion sort on the task list.
			// Sort by the unsigned integer value of the TCSB field, in ascending order.
			sched_task *task_k0_p = task_p - 1;  // Preceding task record pointer.
			uint8_t k;
			
			for (k = i-1; k < SCHED_MAX_TASKS; k--, task_k0_p--) {
				if (task_i.st < task_k0_p->st) {  // 'task_i' goes before 'task_k'.
					*task_k1_p = *task_k0_p;  // Move 'task_k' one step endward.
					task_k1_p = task_k0_p;  // The task list slot that 'task_k' occupied is now available.
				}
				else  // 'task_i' goes after 'task_k'.
					break;  // Stop.
				
				// Continue with the preceding task record.
			}
			// NOTE: If the iteration ends because k == (0 - 1), then 'task_i' belongs at task_list[0].
			
			// NOTE: Cast to avoid 16-bit comparison.
			if ((uint8_t)(k+1) != i)  // If 'task_i' isn't already in the right place...
				*task_k1_p = task_i;  // ...put it at task_list[k+1].
			
			// Continue with the next task record.
		}
		
		sched_list_size -= n_garbage;  // Remove any garbage tasks at the end of the list.
		
		// For each of the first 'n_ready' tasks on the task list (i.e. the ones that were
		// ready at the start of this scheduler iteration, they may since have been put
		// to sleep or marked as garbage (e.g. by another task's handler)):
		task_p = task_list;
		
		for (uint8_t i = 0; i < n_ready; i++, task_p++) {
			sched_task task_i = *task_p;
			
			// NOTE: This check also finds tasks that were marked as garbage after being found ready.
			if (TASK_SLEEP_BIT_SET(task_i.st))  // This task was put to sleep after being found ready.
				continue;  // Let sleeping tasks lie.
			
			if (sched_time_gt(task_i.delay, delta) && !(tcww & TASK_ST_GET_CATFLAG(task_i.st))) {
				// Task is delayed and not notified (being notified would override the delay).
				task_p->delay = sched_time_sub(task_i.delay, delta);  // Update the remaining delay.
				continue;  // Wait a little longer
			}
			
			// Time to run this task.
			// ISSUE: The compiler is being really dumb with pointer->struct assignments, doing
			//        some silly two-steps-forward-one-step-back exercise, and restoring index
			//        registers only to immediately clobber them. You'd think writing three zeros
			//        to memory would be easy.
			task_p->delay = SCHED_TIME_ZERO;  // Clear the task execution delay.
			task_i.handler(task_p);  // Call the task handler.
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
#if SCHED_CLOCK_PRESCALE_LOG < 2
			delta.h = ((uint16_t)sched_delay.l >> (2 - SCHED_CLOCK_PRESCALE_LOG));
#else
			delta.h = ((uint16_t)sched_delay.l << (SCHED_CLOCK_PRESCALE_LOG - 2));
#endif
			delta.h += sched_delay.h << (SCHED_CLOCK_PRESCALE_LOG + 6);  // 8 - 2 = 6.
			_delay_loop_2(delta.h);  // Four (i.e. 1 << 2) cycles per step.
		}
		else  // No scheduler delay.
			sched_delay = SCHED_TIME_ZERO;
#endif
	}
}

#endif
