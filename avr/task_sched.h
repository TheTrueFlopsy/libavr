#ifndef AVR_TASK_SCHED_H
#define AVR_TASK_SCHED_H

#include <stdint.h>

#include "bitops.h"

/**
	File: task_sched.h
	A task scheduler for AVR microcontrollers.
	
	NOTE: The scheduler uses the MCU's Timer2 or Timer0
	to implement the scheduling clock, so programs that use
	the scheduler must be carefully written if they also use
	that timer for other purposes.
	
	NOTE: If the macro *SCHED_USE_TIMER0* is defined, then the scheduler
	will use Timer0 instead of Timer2. Since the ATtiny has no Timer2,
	this macro is defined internally when *LIBAVR_ATTINY* is defined.
	The same applies to the ATmegaU and *LIBAVR_ATMEGA_U*.
	
	NOTE: If the macro *SCHED_NO_ISR* is defined, then the scheduler
	will be compiled without an *TIMER[02]_OVF_vect* ISR, which means
	that external code needs to provide that ISR and ensure that it
	updates the scheduling clock. The documentation of the variable
	<sched_tick_count_h> contains more information about this.
*/

// IDEA: Add a feature that lets a task "hijack" the MCU. Run
// for as long as it pleases, use Timer2 and the task list memory
// for other things, etc., and then hand control back to the scheduler.

// IDEA: Add a way to pause and restart the scheduler in a controlled fashion,
// for low-power modes, etc. Even handle low-power modes in the scheduler
// itself?

// IDEA: Should there be a convenient way for a task to find out why its
//       handler is being invoked (e.g. notified, delay elapsed, running,
//       invoked via sched_invoke())?
//       What would the best way to implement this be?


/// Section: Configuration Macros

/**
	Macro: SCHED_CLOCK_PRESCALE_LOG
	The two-logarithm of the CPU clock prescale value of the timer that the scheduler
	uses to keep track of elapsed time. Either Timer2 or Timer0 is used, depending on
	whether *SCHED_USE_TIMER0* is defined. Configuration macro.
	
	Default value: 6 if *SCHED_USE_TIMER0*, *LIBAVR_ATTINY* or *LIBAVR_ATMEGA_U*
	is defined, otherwise 5.
	
	CAUTION: When the scheduler uses Timer2, this macro MUST be defined to be 0, 3, 5,
	6, 7, 8 or 10. When Timer0 is used, this macro MUST be defined to be 0, 3, 6, 8
	or 10. It MUST also be true that:
	> 1 <= (1000000 * (1 << SCHED_CLOCK_PRESCALE_LOG)) / F_CPU <= 255.
	
	CAUTION: This macro SHOULD have a value such that
	> (1000000 * (1 << SCHED_CLOCK_PRESCALE_LOG)) / F_CPU
	is an integer (without integer division). If it isn't, the scheduler's timekeeping,
	including values produced by the time-related macros in this header, will be very
	inexact. (See <SCHED_TICK_MUSECS>.)
*/
#ifndef SCHED_CLOCK_PRESCALE_LOG
#if defined(SCHED_USE_TIMER0) || defined(LIBAVR_ATTINY) || defined(LIBAVR_ATMEGA_U)
#define SCHED_CLOCK_PRESCALE_LOG 6
#else
#define SCHED_CLOCK_PRESCALE_LOG 5
#endif
#endif

/**
	Macro: SCHED_MAX_TASKS
	The maximum number of tasks that can be scheduled simultaneously. The limit
	applies to the total number of tasks on the task list, including sleeping
	and garbage tasks. Configuration macro.
	
	Default value: 8 if *LIBAVR_ATTINY* is defined, otherwise 16.
	
	CAUTION: This value MUST be in the range 1-254.
	
	CAUTION: Increasing this value will increase the amount of statically
	allocated memory used by the scheduler.
*/
#ifndef SCHED_MAX_TASKS
#ifdef LIBAVR_ATTINY
#define SCHED_MAX_TASKS 8
#else
#define SCHED_MAX_TASKS 16
#endif
#endif


/// Section: Timekeeping

/**
	Macro: CYCLES_TO_MUSECS
	Converts a given number of CPU clock cycles to the duration in microseconds
	of that many clock cycles at the current clock frequency (given by the *F_CPU*
	macro). Function-like macro.
	
	CAUTION: This macro does 32-bit integer arithmetic that is implemented
	in software on the MCU (i.e. it will be slow and potentially bloat
	the compiled program) unless the argument is a compile-time constant.
	
	Parameters:
		N - Number of clock cycles.
	
	Returns:
		Duration in microseconds of *N* clock cycles.
*/
#define CYCLES_TO_MUSECS(N) ((1000000L * (N)) / F_CPU)

/**
	Macro: MUSECS_TO_CYCLES
	Converts a duration in microseconds to the number of CPU clock cycles
	performed during that time at the current clock frequency (given by the *F_CPU*
	macro). Function-like macro.
	
	CAUTION: This macro does 32-bit integer arithmetic that is implemented
	in software on the MCU (i.e. it will be slow and potentially bloat
	the compiled program) unless the argument is a compile-time constant.
	
	Parameters:
		T - Time in microseconds.
	
	Returns:
		Number of clock cycles performed by the MCU in *T* microseconds.
*/
#define MUSECS_TO_CYCLES(T) ((F_CPU * (T)) / 1000000L)

/**
	Macro: SCHED_CLOCK_PRESCALE
	The CPU clock prescale value of the timer that the scheduler uses to keep track
	of elapsed time. It is usually best to set this via <SCHED_CLOCK_PRESCALE_LOG>.
	Constant macro.
*/
#define SCHED_CLOCK_PRESCALE (1 << SCHED_CLOCK_PRESCALE_LOG)

/**
	Macro: SCHED_TICK_MUSECS
	The duration in microseconds of a scheduler tick. Constant macro.
	Depends on <SCHED_CLOCK_PRESCALE_LOG> and *F_CPU*. Equal to
	> (1000000 * (1 << SCHED_CLOCK_PRESCALE_LOG)) / F_CPU  // Integer division
*/
#define SCHED_TICK_MUSECS ((uint8_t)CYCLES_TO_MUSECS(SCHED_CLOCK_PRESCALE))

#define MUSECS_TO_SCHED_TICKS(T) ((T) / SCHED_TICK_MUSECS)
#define SCHED_TICKS_TO_MUSECS(T) (SCHED_TICK_MUSECS * (T))
#define MUSECS_TO_SCHED_SMALLTICKS(T) (MUSECS_TO_SCHED_TICKS(T) % 0x100)
#define MUSECS_TO_SCHED_BIGTICKS(T) (MUSECS_TO_SCHED_TICKS(T) / 0x100)
#define SCHED_BIGTICKS_TO_MUSECS(T) SCHED_TICKS_TO_MUSECS(0x100 * (T))

// NOTE: The SCHED_MIN_DELTA_* macros are used by the scheduler only when
//       SCHED_RATE_LIMITING is defined.
#define SCHED_MIN_DELTA_CYCLES 0

#if SCHED_MIN_DELTA_CYCLES > 0
#define SCHED_MIN_DELTA_MUSECS CYCLES_TO_MUSECS(SCHED_MIN_DELTA_CYCLES)
#else
#define SCHED_MIN_DELTA_MUSECS 1000
#endif

#define SCHED_MIN_DELTA_TICKS MUSECS_TO_SCHED_TICKS(SCHED_MIN_DELTA_MUSECS)
#define SCHED_MIN_DELTA_L ((uint8_t)MUSECS_TO_SCHED_SMALLTICKS(SCHED_MIN_DELTA_MUSECS))
#define SCHED_MIN_DELTA_H ((uint16_t)MUSECS_TO_SCHED_BIGTICKS(SCHED_MIN_DELTA_MUSECS))

/**
	Macro: SCHED_TIME_LH
	Constructs a <sched_time> value representing the given duration in smallticks
	and bigticks. Function-like macro.
	
	Parameters:
		L - Unsigned 8-bit smalltick count.
		H - Unsigned 16-bit bigtick count.
	
	Returns:
		A <sched_time> with the smalltick counter set to *L* and the bigtick counter
		set to *H*.
*/
#define SCHED_TIME_LH(L, H) ((sched_time) { .l=(uint8_t)(L), .h=(uint16_t)(H) })

/**
	Macro: SCHED_TIME_TO_TICKS
	Evaluates to the total number of smallticks stored in a given <sched_time> value.
	Converts bigticks to smallticks to obtain the total number. Function-like macro.
	
	Parameters:
		T - A <sched_time>.
	
	Returns:
		The number of smallticks (including converted bigticks) stored in *T*.
		The type of the result will be *uint32_t*.
*/
#define SCHED_TIME_TO_TICKS(T) ((uint32_t)(0x100L*(T).h + (T).l))

/**
	Macro: SCHED_TIME_MUSECS
	Constructs a <sched_time> value representing the given duration in microseconds.
	Function-like macro.
	
	Parameters:
		T - A duration in microseconds.
	
	Returns:
		A <sched_time> representing *T* microseconds.
*/
#define SCHED_TIME_MUSECS(T) SCHED_TIME_LH( \
	MUSECS_TO_SCHED_SMALLTICKS(T), MUSECS_TO_SCHED_BIGTICKS(T))

/**
	Macro: SCHED_TIME_TO_MUSECS
	Evaluates to the duration in microseconds represented by a given <sched_time> value.
	Function-like macro.
	
	Parameters:
		T - A <sched_time> representing a duration.
	
	Returns:
		The duration represented by *T*, in microseconds.
		The type of the result will be *uint32_t*.
*/
#define SCHED_TIME_TO_MUSECS(T) SCHED_TICKS_TO_MUSECS(SCHED_TIME_TO_TICKS(T))

/**
	Macro: SCHED_TIME_MS
	Constructs a <sched_time> value representing the given duration in milliseconds.
	Function-like macro.
	
	CAUTION: This macro does 32-bit integer arithmetic that is implemented
	in software on the MCU (i.e. it will be slow and potentially bloat
	the compiled program) unless the argument is a compile-time constant.
	
	Parameters:
		T - A duration in milliseconds.
	
	Returns:
		A <sched_time> representing *T* milliseconds.
*/
#define SCHED_TIME_MS(T) SCHED_TIME_MUSECS(1000L*(T))

/**
	Macro: SCHED_TIME_TO_MS
	Evaluates to the duration in milliseconds represented by a given <sched_time> value.
	Function-like macro.
	
	Parameters:
		T - A <sched_time> representing a duration.
	
	Returns:
		The duration represented by *T*, in milliseconds (rounded down).
		The type of the result will be *uint32_t*.
*/
#define SCHED_TIME_TO_MS(T) ((uint32_t)(SCHED_TIME_TO_MUSECS(T)/1000L))

/**
	Macro: SCHED_TIME_ZERO
	A <sched_time> value representing a zero duration (no time at all).
	Constant macro.
*/
#define SCHED_TIME_ZERO SCHED_TIME_LH(0, 0)

/**
	Macro: SCHED_TIME_MAX
	A <sched_time> value representing the longest representable duration.
	Constant macro.
*/
#define SCHED_TIME_MAX SCHED_TIME_LH(UINT8_MAX, UINT16_MAX)

/**
	Macro: SCHED_TIME_MAX_TICKS
	The largest number of smallticks storable in a <sched_time>, including
	those stored as bigticks. The type of this macro is *uint32_t*.
	Constant macro.
*/
#define SCHED_TIME_MAX_TICKS SCHED_TIME_TO_TICKS(SCHED_TIME_MAX)

/**
	Macro: SCHED_TIME_MAX_MUSECS
	The longest duration representable by a <sched_time>, in microseconds.
	The type of this macro is *uint32_t*. Constant macro.
*/
#define SCHED_TIME_MAX_MUSECS SCHED_TIME_TO_MUSECS(SCHED_TIME_MAX)

/**
	Macro: SCHED_TIME_MAX_MS
	The longest duration representable by a <sched_time>, in milliseconds
	(rounded down). The type of this macro is *uint32_t*. Constant macro.
*/
#define SCHED_TIME_MAX_MS SCHED_TIME_TO_MS(SCHED_TIME_MAX)

#define SCHED_MIN_DELTA SCHED_TIME_LH(SCHED_MIN_DELTA_L, SCHED_MIN_DELTA_H)

/**
	Struct: sched_time
	Represents either a duration or a scheduler timestamp. Time is measured
	in scheduler ticks, and the length of a scheduler tick is the reciprocal
	of the scheduling clock frequency, which depends on the CPU clock
	frequency and the scheduler's timer prescaler setting.
	See <SCHED_CLOCK_PRESCALE_LOG> and <SCHED_TICK_MUSECS>.
	
	Each *sched_time* contains two scheduler tick counts, the "smalltick" count
	in the 8-bit field *l* and the "bigtick" count in the 16-bit field *h*.
	Together, the two fields represent a duration of
	> SCHED_TICK_MUSECS*(256*h + l)
	microseconds. The maximum representable duration (<SCHED_TIME_MAX>) is
	> SCHED_TICK_MUSECS*16777215
	microseconds. Since <SCHED_TICK_MUSECS> must be at least 1, durations up to
	16 seconds are always representable.
	
	The scheduler stores the timestamp of the current scheduler iteration in the
	global variable <sched_ticks>. A scheduler timestamp encodes the duration of
	time elapsed since the latest rollover of the scheduling clock's tick counter,
	at the time when the timestamp was recorded. This means that a timestamp with
	a larger total tick count (obtained via <SCHED_TIME_TO_TICKS>) than another
	may still represent an earlier point in time.
	
	However, provided that the total duration of scheduler iterations performed
	between iteration *X* and subsequent iteration *Y* does not exceed the maximum
	representable duration (<SCHED_TIME_MAX>), the difference (as calculated
	by <sched_time_sub>) between a timestamp obtained (from <sched_ticks>)
	during iteration *Y* and one obtained during iteration *X* will always be
	equal to the duration of time (as measured by the scheduling clock) between
	the start of iteration *X* and the start of iteration *Y*.
*/
typedef struct sched_time {
	/**
		Field: l
		The smalltick count field. The duration of one smalltick (which is also
		the resolution of the scheduling clock) is <SCHED_TICK_MUSECS> microseconds.
	*/
	uint8_t l;
	
	/**
		Field: h
		The bigtick count field. One bigtick is equivalent to 256 smallticks.
	*/
	uint16_t h;
	
} sched_time;


/// Section: Task Control and Status Bytes

#define TASK_ST_NUM_OFFS 0
#define TASK_ST_NUM_BITS 3
#define TASK_ST_N_NUMS BITSTATES(TASK_ST_NUM_BITS)
#define TASK_ST_MAX_NUM BITMAX(TASK_ST_NUM_BITS)

/**
	Macro: TASK_ST_NUM_MASK
	Bit mask for the task instance number in a TCSB (see <sched_task>).
	Constant macro. Useful for task list queries (see <sched_query>).
*/
#define TASK_ST_NUM_MASK BITMASK_AT(TASK_ST_NUM_BITS, TASK_ST_NUM_OFFS)

/**
	Macro: TASK_ST_NUM
	Constructs a TCSB value (see <sched_task>) with the given instance number set
	(the category number and sleep bit are zero). Function-like macro.
	
	Parameters:
		NUM - Task instance number in the range 0-7.
	
	Returns:
		A TCSB value with (only) the instance number *NUM* set.
*/
#define TASK_ST_NUM(NUM) MAKE_BITFIELD_AT(TASK_ST_NUM_BITS, TASK_ST_NUM_OFFS, NUM)

/**
	Macro: TASK_ST_GET_NUM
	Extracts the instance number from a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the instance number from.
	
	Returns:
		The instance number from *ST*, right-shifted into the three least significant bits.
*/
#define TASK_ST_GET_NUM(ST) GET_BITFIELD_AT(TASK_ST_NUM_BITS, TASK_ST_NUM_OFFS, ST)

/**
	Macro: TASK_ST_SET_NUM
	Replaces the instance number in a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to replace the instance number in.
		NUM - Task instance number in the range 0-7.
	
	Returns:
		A TCSB value with the category number and sleep bit equal to the ones in *ST* and
		the instance number set to *NUM*.
*/
#define TASK_ST_SET_NUM(ST, NUM) SET_BITFIELD_AT(TASK_ST_NUM_BITS, TASK_ST_NUM_OFFS, ST, NUM)

#define TASK_ST_CAT_OFFS (TASK_ST_NUM_OFFS + TASK_ST_NUM_BITS)
#define TASK_ST_CAT_BITS 4
#define TASK_ST_N_CATS BITSTATES(TASK_ST_CAT_BITS)
#define TASK_ST_MAX_CAT BITMAX(TASK_ST_CAT_BITS)

/**
	Macro: TASK_ST_CAT_MASK
	Bit mask for the task category number in a TCSB (see <sched_task>).
	Constant macro. Useful for task list queries (see <sched_query>).
*/
#define TASK_ST_CAT_MASK BITMASK_AT(TASK_ST_CAT_BITS, TASK_ST_CAT_OFFS)

/**
	Macro: TASK_ST_CAT
	Constructs a TCSB value (see <sched_task>) with the given category number set
	(the instance number and sleep bit are zero). Function-like macro.
	
	Parameters:
		CAT - Task category number in the range 0-15.
	
	Returns:
		A TCSB value with (only) the category number *CAT* set.
*/
#define TASK_ST_CAT(CAT) MAKE_BITFIELD_AT(TASK_ST_CAT_BITS, TASK_ST_CAT_OFFS, CAT)

/**
	Macro: TASK_ST_GET_CAT
	Extracts the category number from a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the category number from.
	
	Returns:
		The category number from *ST*, right-shifted into the four least significant bits.
*/
#define TASK_ST_GET_CAT(ST) GET_BITFIELD_AT(TASK_ST_CAT_BITS, TASK_ST_CAT_OFFS, ST)

/**
	Macro: TASK_ST_SET_CAT
	Replaces the category number in a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to replace the category number in.
		CAT - Task category number in the range 0-15.
	
	Returns:
		A TCSB value with the instance number and sleep bit equal to the ones in *ST* and
		the category number set to *CAT*.
*/
#define TASK_ST_SET_CAT(ST, CAT) SET_BITFIELD_AT(TASK_ST_CAT_BITS, TASK_ST_CAT_OFFS, ST, CAT)

/**
	Macro: TASK_ST_NUM_CAT_MASK
	Bit mask for the task instance and category numbers in a TCSB (see <sched_task>).
	Constant macro. Useful for task list queries (see <sched_query>).
	
	> // Get pointer to task instance 2 in category 5.
	> sched_task *task_p = SCHED_FIND(
	>   TASK_ST_NUM_CAT_MASK, TASK_ST_NUM(2) | TASK_ST_CAT(5), 0);
	>
	> // Invoke task instance 4 in category 11, more tersely.
	> sched_invoke(TASK_ST_NUM_CAT_MASK, TASK_ST_MAKE(4, 11, 0), 0);
*/
#define TASK_ST_NUM_CAT_MASK (TASK_ST_NUM_MASK | TASK_ST_CAT_MASK)

#define TASK_ST_SLP_OFFS (TASK_ST_CAT_OFFS + TASK_ST_CAT_BITS)
#define TASK_ST_SLP_BITS 1
#define TASK_ST_SLP_MASK BITMASK_AT(TASK_ST_SLP_BITS, TASK_ST_SLP_OFFS)
#define TASK_ST_SLP(SLP) MAKE_BITFIELD_AT(TASK_ST_SLP_BITS, TASK_ST_SLP_OFFS, SLP)

/**
	Macro: TASK_ST_GET_SLP
	Extracts the sleep bit from a TCSB value (see <sched_task>). Function-like macro.
	
	NOTE: When only the truth value of the result matters, <TASK_SLEEP_BIT_SET> can
	be a faster alternative.
	
	Parameters:
		ST - The TCSB value to get the sleep bit from.
	
	Returns:
		The sleep bit from *ST*, right-shifted into the least significant bit.
*/
#define TASK_ST_GET_SLP(ST) GET_BITFIELD_AT(TASK_ST_SLP_BITS, TASK_ST_SLP_OFFS, ST)

#define TASK_ST_SET_SLP(ST, SLP) SET_BITFIELD_AT(TASK_ST_SLP_BITS, TASK_ST_SLP_OFFS, ST, SLP)

/**
	Macro: TASK_SLEEP_BIT
	Integer value with (only) the TCSB sleep bit (see <sched_task>) set.
	Constant macro. Can be used to put a task to sleep or awaken it:
	
	> task->st |= TASK_SLEEP_BIT;   // Sleep now.
	> task->st &= ~TASK_SLEEP_BIT;  // Awaken!
*/
#define TASK_SLEEP_BIT TASK_ST_SLP_MASK

/**
	Macro: TASK_SLEEP_BIT_SET
	True if and only if the sleep bit is set in the specified TCSB value
	(see <sched_task>). Function-like macro.
	
	NOTE: This macro can be faster than <TASK_ST_GET_SLP>, especially if
	the argument is not a compile-time constant.
	
	Parameters:
		ST - The TCSB value to check the sleep bit in.
	
	Returns:
		An integer value that is true (i.e. non-zero) if and only if the
		sleep bit in *ST* is set to one (1). This value is NOT necessarily
		either 0 or 1.
*/
#define TASK_SLEEP_BIT_SET(ST) (TASK_SLEEP_BIT & (ST))

/**
	Macro: TASK_ST_MAKE
	Convenience macro for constructing a complete TSCB value (see <sched_task>).
	Function-like macro.
	
	Parameters:
		N - Task instance number in the range 0-7.
		C - Task category number in the range 0-15.
		S - Task sleep bit, 0 or 1.
	
	Returns:
		A TCSB value with the instance number set to *N*, the category number set
		to *C* and the sleep bit set to *S*.
*/
#define TASK_ST_MAKE(N, C, S) (TASK_ST_NUM(N) | TASK_ST_CAT(C) | TASK_ST_SLP(S))

/**
	Macro: TASK_ST_GARBAGE
	This is a special TCSB value (see <sched_task>) that marks
	a task as "garbage" when assigned. At the start of each scheduler
	iteration, the scheduler will remove all tasks marked as garbage from
	the task list. Constant macro.
	
	> task->st = TASK_ST_GARBAGE;  // This is garbage, throw it out.
*/
#define TASK_ST_GARBAGE 0xff


/// Section: Task Records

/**
	Ref: sched_catflags
	The type of fields containing sets of bit flags that correspond to task
	categories.
*/
typedef uint16_t sched_catflags;

/**
	Macro: SCHED_CATFLAG
	Maps task category numbers to the corresponding bit flags. Function-like macro.
	
	Parameters:
		N - A task category number in the range 0-15.
	
	Returns:
		A <sched_catflags> value with (only) bit number *N* set to one (1).
*/
#define SCHED_CATFLAG(N) ((sched_catflags)BV(N))

/**
	Macro: TASK_ST_GET_CATFLAG
	Gets the category number from a TCSB value (see <sched_task>) as a <sched_catflags>
	value. Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the category bit flag for.
	
	Returns:
		A <sched_catflags> value with (only) the bit corresponding to the category number
		in *ST* set to one (1).
*/
#define TASK_ST_GET_CATFLAG(ST) SCHED_CATFLAG(TASK_ST_GET_CAT(ST))

struct sched_task;

/**
	Routine: sched_task_handler
	The type of pointers to functions that implement a task handler procedure.
	
	Parameters:
		task - Pointer to the <sched_task> struct of the task instance.
*/
typedef void (*sched_task_handler)(struct sched_task *task);

/**
	Struct: sched_task
	Represents an instance of a task that can be scheduled for execution.
*/
typedef struct sched_task {
	// IDEA: Use bit fields in the task control and status byte?
	// IDEA: Move the category number into the least significant bits,
	//       to avoid bit shifting when extracting it?
	/**
		Field: st
		Task control and status byte (TCSB) of the task instance. The control
		and status byte has the following format:
		
		>        7      6      5      4      3      2      1      0
		> |  SLP | CAT3 | CAT2 | CAT1 | CAT0 | NUM2 | NUM1 | NUM0 |
		
		Bit fields:
			NUM2:0 - Task instance number, in the range 0-7.
			CAT3:0 - Task category number, in the range 0-15.
			SLP - Sleep bit. If this is set to one, the task will not
				be executed until a wakeup event occurs.
		
		NOTE: TCSB values can be conveniently constructed with the macro
			<TASK_ST_MAKE>.
		
		CAUTION: A task where the TCSB is set to all ones will be treated
		as garbage by the scheduler, which means that it will be removed
		from the task list at the start of a scheduler iteration. For
		this reason, assigning instance number 7 to a task in category 15
		is generally something to be avoided (since putting such a task to
		sleep is equivalent to marking it as garbage). Also note that any
		task marked as garbage will effectively be in category 15, which
		can be confusing if that category is also used for non-garbage
		tasks. (See <TASK_ST_GARBAGE>.)
	*/
	uint8_t st;
	
	/**
		Field: delay
		Execution delay of the task instance. The scheduler will subtract
		elapsed time deltas (i.e. the durations of scheduler iterations)
		from this field, and will execute the task when the value of the
		field reaches zero. If the current time delta is greater than the
		value of the field, the field will be set to zero.
		
		NOTE: When the scheduler invokes a task's *handler* for any reason,
		including the task being notified or invoked via <sched_invoke>,
		any currently elapsing delay is canceled. The *delay* field always
		starts out zeroed when the task handler is invoked.
		
		NOTE: If a task should execute periodically, its task handler
		should re-assign the desired period to the *delay* field upon
		task execution. The scheduler itself will simply leave zeroed
		*delay* fields unchanged and execute the corresponding tasks
		once per scheduler iteration.
	*/
	sched_time delay;
	
	/**
		Field: handler
		Pointer to the task handler procedure. The task handler will be invoked
		whenever the scheduler executes the task. The handler may also be invoked
		via the API functions <sched_invoke> and <sched_invoke_all>.
		
		NOTE: It is perfectly safe to update this field while the scheduler is
		running (except in ISRs). For example, *handler* field updates can be used
		to implement a state machine in a task.
	*/
	sched_task_handler handler;
	
} sched_task;


/// Section: API Variables

#ifndef LIBAVR_TEST_BUILD

/**
	Variable: sched_isr_tcww
	This is the ISR-Task Category Wakeup Word (I-TCWW), a variable that
	contains bit flags representing a set of task categories that should
	be notified at the start of the next scheduler iteration.
	
	The I-TCWW is a volatile variable and should be used by ISRs that need
	to notify categories of tasks.
*/
extern volatile sched_catflags sched_isr_tcww;

/**
	Variable: sched_task_tcww
	This is the Task-Task Category Wakeup Word (T-TCWW), a variable that
	contains bit flags representing a set of task categories that should
	be notified at the start of the next scheduler iteration.
	
	The T-TCWW is a non-volatile variable and should be used by task
	handler procedures that need to notify categories of tasks.
*/
extern sched_catflags sched_task_tcww;

/**
	Variable: sched_ticks
	The scheduler iteration timestamp. When task handlers run, this variable
	will contain a timestamp obtained at the beginning of the current scheduler
	iteration.
*/
extern sched_time sched_ticks;

/**
	Variable: sched_delta
	The scheduler iteration time delta. When task handlers run, this variable
	will contain the difference of the current timestamp and the timestamp of
	the previous scheduler iteration.
*/
extern sched_time sched_delta;

/**
	Variable: sched_list_size
	Current size of the task list. When task handlers run, this will contain
	the number of tasks currently on the task list, including sleeping and
	garbage tasks.
*/
extern uint8_t sched_list_size;

/**
	Variable: sched_tick_count_h
	High bytes ("bigticks") of the scheduling clock's tick count.
	
	NOTE: To get correct scheduling when *SCHED_NO_ISR* is defined, this
	variable MUST be incremented in an externally defined *TIMER2_OVF_vect*
	(*TIMER0_OVF_vect* when *SCHED_USE_TIMER0* is defined) ISR.
*/
extern volatile uint16_t sched_tick_count_h;

#endif


/// Section: API Functions - Helpers

/**
	Function: sched_time_is_zero
	Tests whether a <sched_time> contains a zero value. The test is equivalent
	to the expression *t.l == 0 && t.h == 0*.
*/
uint8_t sched_time_is_zero(sched_time t);

/**
	Function: sched_time_gt
	Tests whether <sched_time> *a* contains a greater integer value than *b*.
	The test is equivalent to the expression *a.h+a.l > b.h+b.l*.
	
	NOTE: This operation is only meaningful for durations. It CANNOT be used
	to determine whether one scheduler timestamp represents a later moment
	in time than another.
*/
uint8_t sched_time_gt(sched_time a, sched_time b);

/**
	Function: sched_time_lt
	Tests whether <sched_time> *a* contains a lesser integer value than *b*.
	The test is equivalent to the expression *a.h+a.l < b.h+b.l*.
	
	NOTE: This operation is only meaningful for durations. It CANNOT be used
	to determine whether one scheduler timestamp represents an earlier moment
	in time than another.
*/
uint8_t sched_time_lt(sched_time a, sched_time b);

/**
	Function: sched_time_gte
	Tests whether <sched_time> *a* contains a greater-or-equal integer value
	than *b*. The test is equivalent to the expression *a.h+a.l >= b.h+b.l*.
	
	NOTE: This operation is only meaningful for durations. It CANNOT be used
	to determine whether one scheduler timestamp represents a later moment
	in time than another.
*/
uint8_t sched_time_gte(sched_time a, sched_time b);

/**
	Function: sched_time_lte
	Tests whether <sched_time> *a* contains a lesser-or-equal integer value
	than *b*. The test is equivalent to the expression *a.h+a.l <= b.h+b.l*.
	
	NOTE: This operation is only meaningful for durations. It CANNOT be used
	to determine whether one scheduler timestamp represents an earlier moment
	in time than another.
*/
uint8_t sched_time_lte(sched_time a, sched_time b);

/**
	Function: sched_time_add
	Adds <sched_time> *b* to <sched_time> *a*. The addition is performed
	in a way that is equivalent to addition of 24-bit unsigned smalltick
	counts (with wraparound in case the sum is greater than <SCHED_TIME_MAX>).
	
	This operation produces a meaningful result when one duration is added to
	another such that their sum is no longer than <SCHED_TIME_MAX>. In that
	case, the result represents a nonnegative duration.
	
	This operation also produces a meaningful result when a duration is
	added to a timestamp. In that case, the result is the timestamp
	of the point in time that many seconds later than the one the original
	timestamp corresponds to.
*/
sched_time sched_time_add(sched_time a, sched_time b);

/**
	Function: sched_time_sub
	Subtracts <sched_time> *b* from <sched_time> *a*. The subtraction is
	performed in a way that is equivalent to subtraction of 24-bit unsigned
	smalltick counts (with wraparound in case *a* is less than *b*).
	
	This operation produces a meaningful result when one duration is
	subtracted from another that is no shorter and when one timestamp is
	subtracted from another that was obtained no earlier in time, and not more
	than <SCHED_TIME_MAX> seconds later. In those cases, the result represents
	a nonnegative duration.
	
	This operation also produces a meaningful result when a duration is
	subtracted from a timestamp. In that case, the result is the timestamp
	of the point in time that many seconds earlier than the one the original
	timestamp corresponds to.
*/
sched_time sched_time_sub(sched_time a, sched_time b);


/// Section: API Functions - Scheduler Operations

#ifndef LIBAVR_TEST_BUILD

/**
	Macro: SCHED_FIND
	Deprecated, kept for backward compatibility. Use the function <sched_find>
	instead of this macro.
*/
#define SCHED_FIND(M, V, I) sched_find(M, V, I)

/**
	Function: sched_init
	Initializes the task scheduler.
	
	CAUTION: This function MUST be called before <sched_run> is called or any
	tasks are added to the task list with <sched_add>.
*/
void sched_init(void);

/**
	Function: sched_ptr_query
	Queries the task list. Returns a pointer to the first encountered task
	on the list for which the TCSB bits selected (i.e. set to one) in *st_mask*
	are equal to the corresponding bits in *st_val*. The search starts
	at index *start_i* in the task list. If no matching task is found, then
	a null pointer is returned. If a matching task is found, then the task list
	index of that task is stored in the location pointed to by *task_i_p*.
	
	NOTE: If all bits in *st_mask* are set (i.e. the value is 255) and *st_val*
	      is <TASK_ST_GARBAGE>, then a null pointer will always be returned.
	      This special case can be used when an always-empty query is useful,
	      e.g. when some API requires task query arguments to use as a filter
	      for synchronous task invocation, but that feature isn't being used.
	
	NOTE: The matched task may be one that is marked as garbage.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
		task_i_p - Pointer to location where the task list index of the matched task
			should be stored. Note that this location is NOT updated if no matching
			task is found.
	
	Returns:
		A pointer to the matched task, or a null pointer if there are no matching
		tasks.
*/
sched_task *sched_ptr_query(
	uint8_t st_mask, uint8_t st_val, uint8_t start_i, uint8_t *task_i_p);

/**
	Function: sched_query
	Queries the task list. Returns the task list index of the first encountered
	task on the list for which the TCSB bits selected (i.e. set to one) in
	*st_mask* are equal to the corresponding bits in *st_val*. The search starts
	at index *start_i* in the task list. If no matching task is found, then
	<SCHED_MAX_TASKS> is returned.
	
	NOTE: This function delegates to <sched_ptr_query>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
	
	Returns:
		The task list index of the matched task, or <SCHED_MAX_TASKS> if there are
		no matching tasks.
*/
uint8_t sched_query(uint8_t st_mask, uint8_t st_val, uint8_t start_i);

/**
	Function: sched_find
	Queries the task list. Returns a pointer to the first encountered task
	on the list for which the TCSB bits selected (i.e. set to one) in *st_mask*
	are equal to the corresponding bits in *st_val*. The search starts at index
	*start_i* in the task list. If no matching task is found, then a null pointer
	is returned.
	
	NOTE: This function delegates to <sched_ptr_query>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
	
	Returns:
		A pointer to the matched task, or a null pointer if there are no matching
		tasks.
*/
sched_task *sched_find(uint8_t st_mask, uint8_t st_val, uint8_t start_i);

/**
	Function: sched_invoke
	Invokes a matching task on the task list. Invokes and returns the task list
	index of the first encountered task on the list for which the TCSB bits
	selected (i.e. set to one) in *st_mask* are equal to the corresponding bits
	in *st_val*. The search starts at index *start_i* in the task list. If no
	matching task is found, then <SCHED_MAX_TASKS> is returned.
	
	NOTE: Before handler invocation, sleeping tasks are awakened (i.e. have the
	      sleep bit in their TCSB cleared) and elapsing execution delays canceled.
	
	NOTE: A matched task marked as garbage is NOT invoked or awakened,
	      but the index where the task was found is still returned.
	
	NOTE: The task handler procedure is invoked synchronously.
	
	NOTE: This function delegates to <sched_ptr_query>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
	
	Returns:
		The task list index of the matched task, or <SCHED_MAX_TASKS> if there
		are no matching tasks.
*/
uint8_t sched_invoke(uint8_t st_mask, uint8_t st_val, uint8_t start_i);

/**
	Function: sched_invoke_all
	Invokes all matching tasks on the task list. Invokes every non-garbage task
	on the list for which the TCSB bits selected (i.e. set to one) in *st_mask* are
	equal to the corresponding bits in *st_val*.
	
	NOTE: This function delegates to <sched_invoke>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
*/
void sched_invoke_all(uint8_t st_mask, uint8_t st_val);

/**
	Function: sched_wake
	Awakens and optionally notifies a matching task on the task list. Awakens and
	returns the task list index of the first encountered task on the list for
	which the TCSB bits selected (i.e. set to one) in *st_mask* are equal to the
	corresponding bits in *st_val*. The search starts at index *start_i* in the
	task list. If no matching task is found, then <SCHED_MAX_TASKS> is returned.
	
	NOTE: A matched task marked as garbage is NOT awakened, but the index where
	      the task was found is still returned.
	
	NOTE: This function delegates to <sched_ptr_query>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
		notify - If this is true, a matched task will be notified (i.e. have its
			execution delay set to zero) instead of just awakened.
	
	Returns:
		The task list index of the matched task, or <SCHED_MAX_TASKS> if there
		are no matching tasks.
*/
uint8_t sched_wake(uint8_t st_mask, uint8_t st_val, uint8_t start_i, uint8_t notify);

/**
	Function: sched_wake_all
	Awakens and optionally notifies all matching tasks on the task list. Awakens every
	non-garbage task on the list for which the TCSB bits selected (i.e. set to one)
	in *st_mask* are equal to the corresponding bits in *st_val*.
	
	NOTE: This function delegates to <sched_wake>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		notify - If this is true, matched tasks will be notified (i.e. have their
			execution delay set to zero) instead of just awakened.
*/
void sched_wake_all(uint8_t st_mask, uint8_t st_val, uint8_t notify);

/**
	Function: sched_get_at
	Accesses the task list by index.
	
	Parameters:
		i - Task list index.
	
	Returns:
		A pointer to the scheduler task at index *i*, or a null pointer if *i*
		is not a valid index.
*/
sched_task *sched_get_at(uint8_t i);

/**
	Function: sched_add
	Adds the specified task to the task list.
	
	Parameters:
		task - Pointer to the task to add to the task list. The specified
			<sched_task> structure will be copied into internal storage.
	
	Returns:
		The task list index of the added task, or <SCHED_MAX_TASKS> if the task
		couldn't be added.
*/
uint8_t sched_add(const sched_task *task);

/**
	Function: sched_remove
	Marks a matching task on the task list for removal. Marks as garbage and
	returns the index of the first encountered task on the list for which
	the TCSB bits selected (i.e. set to one) in *st_mask* are equal to the
	corresponding bits in *st_val*. The search starts at index *start_i* in the
	task list. If no matching task is found, then <SCHED_MAX_TASKS> is returned.
	
	NOTE: This function does not check whether the matching task was already
	      marked as garbage.
	
	NOTE: This function delegates to <sched_ptr_query>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
	
	Returns:
		The task list index of the task marked for removal, or <SCHED_MAX_TASKS>
		if there were no matching tasks.
*/
uint8_t sched_remove(uint8_t st_mask, uint8_t st_val, uint8_t start_i);

/**
	Function: sched_remove_all
	Marks all matching tasks on the task list for removal. Marks as garbage
	every task on the list for which the TCSB bits selected (i.e. set to one)
	in *st_mask* are equal to the corresponding bits in *st_val*.
	
	NOTE: This function delegates to <sched_remove>, and the notes for that
	      function also apply to this one.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
*/
void sched_remove_all(uint8_t st_mask, uint8_t st_val);

/**
	Function: sched_run
	Enters the main loop of the task scheduler. This is a non-returning routine,
	typically invoked when the application has completed initialization and is
	ready to begin task execution.
*/
void sched_run(void) __attribute__ ((noreturn));

#endif
#endif
