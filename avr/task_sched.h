#ifndef AVR_TASK_SCHED_H
#define AVR_TASK_SCHED_H

#include <stdint.h>

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
	The same applies to the ATmega(16|32)U4 and *LIBAVR_ATMEGA_U*.
	
	NOTE: If the macro *SCHED_NO_ISR* is defined, then the scheduler
	will be compiled without an *TIMER[02]_OVF_vect* ISR, which means
	that external code needs to provide that ISR and ensure that it
	updates the scheduling clock. The documentation of the variable
	<sched_tick_count_h> contains more information about this.
*/

// IDEA: Add a feature that lets a task "hijack" the MCU. Run
// for as long as it pleases, use Timer2 and the task list memory
// for other things, etc., and then hand control back to the scheduler.

// IDEA: Should there be a convenient way for a task to find out why its
//       handler is being invoked (e.g. notified, delay elapsed, running,
//       invoked via sched_invoke())?
//       What would the best way to implement this be?

// TODO: Document all these macros. At least the ones that are likely to
//       be used in applications. Sometimes it's better to just look
//       at the code.

/**
	Macro: BV
	Utility macro that maps a bit number to an integer value with (only)
	the corresponding bit set to 1. Function-like macro.
	
	Parameters:
		N - A bit number. Should be in the range 0-7 for 8-bit values
			and 0-15 for 16-bit values. Bit 0 is the least significant
			and bit 7/15 the most significant.
*/
#define BV(N) (1 << (N))

/**
	Macro: CYCLES_TO_MUSECS
	Converts a given number of MCU clock cycles to the duration in microseconds
	of that many clock cycles at the current clock frequency (given by the *F_CPU*
	macro). Function-like macro.
	
	Parameters:
		N - Number of clock cycles.
*/
#define CYCLES_TO_MUSECS(N) ((1000000L * (N)) / F_CPU)

/**
	Macro: MUSECS_TO_CYCLES
	Converts a duration in microseconds to the number of MCU clock cycles
	performed during that time at the current clock frequency (given by the *F_CPU*
	macro). Function-like macro.
	
	Parameters:
		T - Time in microseconds.
*/
#define MUSECS_TO_CYCLES(T) ((F_CPU * (T)) / 1000000L)

/**
	Macro: SCHED_CLOCK_PRESCALE_LOG
	The two-logarithm of the MCU clock prescale value of the timer that the scheduler
	uses to keep track of elapsed time. Either Timer2 or Timer0 is used, depending on
	whether *SCHED_USE_TIMER0* is defined. Configuration macro.
	
	Default value: 6 if *SCHED_USE_TIMER0*, *LIBAVR_ATTINY* or *LIBAVR_ATMEGA_U* is defined,
	otherwise 5.
	
	CAUTION: When the scheduler uses Timer2, this macro MUST be defined to be 0, 3, 5,
	6, 7, 8 or 10. When Timer0 is used, this macro MUST be defined to be 0, 3, 6, 8 or 10.
	It MUST also be true (with integer division) that:
	
	> 1 <= (1000000 * 2^SCHED_CLOCK_PRESCALE_LOG) / F_CPU <= 255.
*/
#ifndef SCHED_CLOCK_PRESCALE_LOG
#if defined(SCHED_USE_TIMER0) || defined(LIBAVR_ATTINY) || defined(LIBAVR_ATMEGA_U)
#define SCHED_CLOCK_PRESCALE_LOG 6
#else
#define SCHED_CLOCK_PRESCALE_LOG 5
#endif
#endif

/**
	Macro: SCHED_CLOCK_PRESCALE
	The MCU clock prescale value of the timer that the scheduler uses to keep track
	of elapsed time. It is usually best to set this via *SCHED_CLOCK_PRESCALE_LOG*.
	Constant macro.
*/
#define SCHED_CLOCK_PRESCALE (1 << SCHED_CLOCK_PRESCALE_LOG)

/**
	Macro: SCHED_TICK_MUSECS
	The duration in microseconds of a scheduler timer tick. Constant macro.
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
	Macro: SCHED_MAX_TASKS
	The maximum number of tasks that can be scheduled simultaneously. The limit
	applies to the total number of tasks on the task list, including sleeping
	and garbage tasks. Configuration macro.
	
	Default value: 8 if *LIBAVR_ATTINY* is defined, otherwise 16.
	
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

#define TASK_ST_N(B) (1 << (B))
#define TASK_ST_MAX(B) (TASK_ST_N(B) - 1)
#define TASK_ST_MASK(O, B) (TASK_ST_MAX(B) << (O))
#define TASK_ST_INIT(O, M, V) ((M) & ((V) << (O)))
#define TASK_ST_GET(O, M, ST) (((M) & (ST)) >> (O))
#define TASK_ST_SET(O, M, ST, V) ((~(M) & (ST)) | TASK_ST_INIT(O, M, V))

#define TASK_ST_NUM_OFFS 0
#define TASK_ST_NUM_BITS 3
#define TASK_ST_N_NUMS TASK_ST_N(TASK_ST_NUM_BITS)
#define TASK_ST_MAX_NUM TASK_ST_MAX(TASK_ST_NUM_BITS)
#define TASK_ST_NUM_MASK TASK_ST_MASK(TASK_ST_NUM_OFFS, TASK_ST_NUM_BITS)

/**
	Macro: TASK_ST_NUM
	Constructs a TCSB value (see <sched_task>) with the given instance number set
	(the category number and sleep bit are zero). Function-like macro.
	
	Parameters:
		NUM - Task instance number in the range 0-7.
*/
#define TASK_ST_NUM(NUM) TASK_ST_INIT(TASK_ST_NUM_OFFS, TASK_ST_NUM_MASK, NUM)

/**
	Macro: TASK_ST_GET_NUM
	Extracts the instance number from a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the instance number from.
*/
#define TASK_ST_GET_NUM(ST) TASK_ST_GET(TASK_ST_NUM_OFFS, TASK_ST_NUM_MASK, ST)

/**
	Macro: TASK_ST_SET_NUM
	Replaces the instance number in a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to replace the instance number in.
		NUM - Task instance number in the range 0-7.
*/
#define TASK_ST_SET_NUM(ST, NUM) TASK_ST_SET(TASK_ST_NUM_OFFS, TASK_ST_NUM_MASK, ST, NUM)

#define TASK_ST_CAT_OFFS (TASK_ST_NUM_OFFS + TASK_ST_NUM_BITS)
#define TASK_ST_CAT_BITS 4
#define TASK_ST_N_CATS TASK_ST_N(TASK_ST_CAT_BITS)
#define TASK_ST_MAX_CAT TASK_ST_MAX(TASK_ST_CAT_BITS)
#define TASK_ST_CAT_MASK TASK_ST_MASK(TASK_ST_CAT_OFFS, TASK_ST_CAT_BITS)

/**
	Macro: TASK_ST_CAT
	Constructs a TCSB value (see <sched_task>) with the given category number set
	(the instance number and sleep bit are zero). Function-like macro.
	
	Parameters:
		CAT - Task category number in the range 0-15.
*/
#define TASK_ST_CAT(CAT) TASK_ST_INIT(TASK_ST_CAT_OFFS, TASK_ST_CAT_MASK, CAT)

/**
	Macro: TASK_ST_GET_CAT
	Extracts the category number from a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the category number from.
*/
#define TASK_ST_GET_CAT(ST) TASK_ST_GET(TASK_ST_CAT_OFFS, TASK_ST_CAT_MASK, ST)

/**
	Macro: TASK_ST_SET_CAT
	Replaces the category number in a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to replace the category number in.
		CAT - Task category number in the range 0-15.
*/
#define TASK_ST_SET_CAT(ST, CAT) TASK_ST_SET(TASK_ST_CAT_OFFS, TASK_ST_CAT_MASK, ST, CAT)

#define TASK_ST_NUM_CAT_MASK (TASK_ST_NUM_MASK | TASK_ST_CAT_MASK)

#define TASK_ST_SLP_OFFS (TASK_ST_CAT_OFFS + TASK_ST_CAT_BITS)
#define TASK_ST_SLP_BITS 1
#define TASK_ST_SLP_MASK TASK_ST_MASK(TASK_ST_SLP_OFFS, TASK_ST_SLP_BITS)
#define TASK_ST_SLP(SLP) TASK_ST_INIT(TASK_ST_SLP_OFFS, TASK_ST_SLP_MASK, SLP)

/**
	Macro: TASK_ST_GET_SLP
	Extracts the sleep bit from a TCSB value (see <sched_task>). Function-like macro.
	
	Parameters:
		ST - The TCSB value to get the sleep bit from.
*/
#define TASK_ST_GET_SLP(ST) TASK_ST_GET(TASK_ST_SLP_OFFS, TASK_ST_SLP_MASK, ST)

#define TASK_ST_SET_SLP(ST, SLP) TASK_ST_SET(TASK_ST_SLP_OFFS, TASK_ST_SLP_MASK, ST, SLP)

/**
	Macro: TASK_SLEEP_BIT
	Integer value with the TCSB sleep bit (see <sched_task>) set. Constant macro.
	Can be used to put a task to sleep or awaken it:
	
	> task->st |= TASK_SLEEP_BIT;   // Sleep now.
	> task->st &= ~TASK_SLEEP_BIT;  // Awaken!
*/
#define TASK_SLEEP_BIT TASK_ST_SLP(1)

/**
	Macro: TASK_ST_MAKE
	Convenience macro for constructing a complete TSCB value (see <sched_task>).
	Function-like macro.
	
	Parameters:
		N - Task instance number in the range 0-7.
		C - Task category number in the range 0-15.
		S - Task sleep bit, 0 or 1.
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

// FIXME: Explain what the "maximum allowed length" is, and why.
/**
	Struct: sched_time
	Represents either a duration or a scheduler timestamp. Time is measured
	in scheduler ticks, and the length of a scheduler tick is the reciprocal
	of the scheduling clock frequency, which depends on the CPU clock
	frequency and the scheduler's clock prescaler setting (the default is
	to divide the CPU clock frequency by 32).
	
	Each *sched_time* represents a duration of
	> SCHED_TICK_MUSECS*(256*h + l)
	microseconds.
	
	A scheduler timestamp is the duration of time elapsed since the latest
	rollover of the scheduling clock's tick counter, at the time when the
	timestamp was created. This means that timestamps with larger integer
	values may represent earlier points in time. However, provided that
	the duration of a scheduler iteration does not exceed the maximum
	allowed length, the difference between two timestamps obtained during
	successive iterations will always be equal to the duration of time
	between the moments in time (as measured by the scheduling clock) when
	those timestamps were created.
*/
typedef struct sched_time {
	/**
		Field: l
		Low "smalltick" byte of the time value. One smalltick is *SCHED_TICK_MUSECS*
		microseconds.
	*/
	uint8_t l;
	
	/**
		Field: h
		High "bigtick" bytes of the time value. One bigtick is 256 smallticks.
	*/
	uint16_t h;
	
} sched_time;

/**
	Macro: SCHED_TIME_LH
	Constructs a <sched_time> value representing the given duration in smallticks
	and bigticks. Function-like macro.
	
	Parameters:
		L - Unsigned 8-bit smalltick count.
		H - Unsigned 16-bit bigtick count.
*/
#define SCHED_TIME_LH(L, H) ((sched_time) { .l=(uint8_t)(L), .h=(uint16_t)(H) })

/**
	Macro: SCHED_TIME_MUSECS
	Constructs a <sched_time> value representing the given duration in microseconds.
	Function-like macro.
	
	Parameters:
		T - A duration in microseconds.
*/
#define SCHED_TIME_MUSECS(T) SCHED_TIME_LH( \
	MUSECS_TO_SCHED_SMALLTICKS(T), MUSECS_TO_SCHED_BIGTICKS(T))

/**
	Macro: SCHED_TIME_MS
	Constructs a <sched_time> value representing the given duration in milliseconds.
	Function-like macro.
	
	Parameters:
		T - A duration in milliseconds.
*/
#define SCHED_TIME_MS(T) SCHED_TIME_MUSECS(1000L*(T))

/**
	Macro: SCHED_TIME_ZERO
	A <sched_time> value representing a zero duration (no time at all).
	Constant macro.
*/
#define SCHED_TIME_ZERO SCHED_TIME_LH(0, 0)

#define SCHED_MIN_DELTA SCHED_TIME_LH(SCHED_MIN_DELTA_L, SCHED_MIN_DELTA_H)


/// Section: Task Records

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
	/**
		Field: st
		Task control and status byte (TCSB) of the task instance. The control
		and status byte has the following format:
		
		>        7      6      5      4      3      2      1      0
		> |  SLP | CAT3 | CAT2 | CAT1 | CAT0 | NUM2 | NUM1 | NUM0 |
		
		Bit fields:
			NUM0:2 - Task instance number, in the range 0-7.
			CAT0:3 - Task category number, in the range 0-15.
			SLP - Sleep bit. If this is set to one, the task will not
				be executed until a wakeup event occurs.
		
		NOTE: TCSB values can be conveniently constructed with the macro
			<TASK_ST_MAKE>.
		
		CAUTION: A task where the TCSB is set to all ones will be treated
		as garbage by the scheduler, which means that it will be removed
		from the task list at the start of a scheduler iteration. For
		this reason, assigning instance number 7 to a task in category 15
		is generally something to be avoided.
	*/
	uint8_t st;
	
	/**
		Field: delay
		Execution delay of the task instance. The scheduler will subtract
		elapsed time deltas from this field, and will execute the task
		when the value of the field is zero. If the time delta is greater
		than the value of the field, the field will be set to zero.
		
		NOTE: When a task's *handler* is invoked for any reason, including
		the task being notified or invoked via <sched_invoke>, any currently
		elapsing delay is canceled. The *delay* field always starts out
		zeroed when the handler is invoked.
		
		NOTE: If a task should execute periodically, then that task is
		responsible for re-assigning the period to the *delay* field upon
		task execution. The scheduler itself will simply leave zeroed
		*delay* fields unchanged and execute the corresponding tasks
		once per scheduler iteration.
	*/
	sched_time delay;
	
	/**
		Field: handler
		Pointer to the task handler procedure. The task handler will be invoked
		when the scheduler executes the task, and may also be invoked via
		the functions <sched_invoke> and <sched_invoke_all>.
	*/
	sched_task_handler handler;
	
} sched_task;


/// Section: Global Variables

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
*/
#define SCHED_CATFLAG(N) ((sched_catflags)1 << (N))

#ifndef LIBAVR_TEST_BUILD

/**
	Variable: sched_isr_tcww
	This is the ISR-Task Category Wakeup Word (I-TCWW), a variable that
	contains bit flags representing a set of task categories that should
	be awakened at the start of the next scheduler iteration.
	
	The I-TCWW is a volatile variable and should be used by ISRs that need
	to awaken categories of tasks.
*/
extern volatile sched_catflags sched_isr_tcww;

/**
	Variable: sched_task_tcww
	This is the Task-Task Category Wakeup Word (T-TCWW), a variable that
	contains bit flags representing a set of task categories that should
	be awakened at the start of the next scheduler iteration.
	
	The T-TCWW is a non-volatile variable and should be used by task
	handler procedures that need to awaken categories of tasks.
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


/// Section: Helper Functions

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

// FIXME: How much later is "too much later"?
/**
	Function: sched_time_sub
	Subtracts one <sched_time> from another. The subtraction is performed in
	a way that is equivalent to two's complement subtraction of 24-bit integers.
	
	NOTE: This operation produces a meaningful result only when one duration is
	subtracted from another that is no shorter and when one timestamp is
	subtracted from another that was obtained no earlier in time, and not too
	much later. In those cases, the result represents a nonnegative duration.
*/
sched_time sched_time_sub(sched_time a, sched_time b);


/// Section: Scheduler Operations

/**
	Macro: SCHED_FIND
	Gets a pointer to the first task that matches a query. Returns a pointer to
	the first encountered task on the list where the TCSB bits selected
	(i.e. set to one) in *M* are equal to the corresponding bits in *V*.
	The search starts at index *I* in the task list. If no matching task is
	found, then a null pointer is returned. Function-like macro.
	
	Parameters:
		M - Bit mask to apply to the task control and status byte.
		V - Bit pattern to compare with the task control and status byte.
		I - Starting index of the forward search of the task list.
	
	Returns:
		A pointer to the matched task, or a null pointer if there are no matching
		tasks.
*/
#define SCHED_FIND(M, V, I) sched_get_at(sched_query(M, V, I))

/**
	Function: sched_init
	Initializes the task scheduler.
	
	CAUTION: This function MUST be called before <sched_run> is called or any
	tasks are added to the task list with <sched_add>.
*/
void sched_init(void);

/**
	Function: sched_query
	Queries the task list. Returns the task list index of the first encountered
	task on the list where the TCSB bits selected (i.e. set to one) in
	*st_mask* are equal to the corresponding bits in *st_val*. The search starts
	at index *start_i* in the task list. If no matching task is found, then
	<SCHED_MAX_TASKS> is returned.
	
	NOTE: If all bits in *st_mask* are set (i.e. the value is 255) and *st_val*
	      is <TASK_ST_GARBAGE>, then <SCHED_MAX_TASKS> will always be returned.
	      This special case can be used when an always-empty query is useful,
	      e.g. when some API requires task query arguments to use as a filter
	      for synchronous task invocation, but that feature isn't being used.
	
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
	Function: sched_invoke
	Invokes a matching task on the task list. Invokes and returns the task list
	index of the first encountered task on the list where the TCSB bits
	selected (i.e. set to one) in *st_mask* are equal to the corresponding bits
	in *st_val*. The search starts at index *start_i* in the task list. If no
	matching task is found, then <SCHED_MAX_TASKS> is returned.
	
	NOTE: Before handler invocation, sleeping tasks are awakened (i.e. have the
	      sleep bit in their TCSB cleared) and elapsing delays canceled.
	
	NOTE: The task handler procedure is invoked synchronously.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
		start_i - Starting index of the forward search of the task list.
	
	Returns:
		The task list index of the invoked task, or <SCHED_MAX_TASKS> if there
		are no matching tasks.
*/
uint8_t sched_invoke(uint8_t st_mask, uint8_t st_val, uint8_t start_i);

/**
	Function: sched_invoke_all
	Invokes all matching tasks on the task list. Invokes every task on the list
	where the TCSB bits selected (i.e. set to one) in *st_mask* are equal
	to the corresponding bits in *st_val*.
	
	NOTE: Before handler invocation, sleeping tasks are awakened (i.e. have the
	      sleep bit in their TCSB cleared) and elapsing delays canceled.
	
	NOTE: The task handler procedures are invoked synchronously.
	
	Parameters:
		st_mask - Bit mask to apply to the task control and status byte.
		st_val - Bit pattern to compare with the task control and status byte.
*/
void sched_invoke_all(uint8_t st_mask, uint8_t st_val);

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
	returns the index of the first encountered task on the list where the TCSB
	bits selected (i.e. set to one) in *st_mask* are equal to the
	corresponding bits in *st_val*. The search starts at index *start_i* in the
	task list. If no matching task is found, then <SCHED_MAX_TASKS> is returned.
	
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
	Function: sched_run
	Enters the main loop of the task scheduler. This is a non-returning routine,
	typically invoked when the application has completed initialization and is
	ready to begin task execution.
*/
void sched_run(void) __attribute__ ((noreturn));

#endif
#endif
