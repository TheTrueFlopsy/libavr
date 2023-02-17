#ifndef AVR_TBOUNCER_H
#define AVR_TBOUNCER_H

#include <stdint.h>

#include "task_sched.h"

/**
	File: tbouncer.h
	Input debouncing module that uses the <task_sched.h> task scheduler.
	
	NOTE: To disable debouncing on an I/O port, define the macro
	*TBOUNCER_DISABLE_PORT_{x}*, where {x} is "A", "B", "C" or "D". Doing
	this saves both some clock cycles and about 12 bytes of RAM per
	disabled port.
	
	CAUTION (ATmega): The ATmega has no port A, so if neither *LIBAVR_ATTINY*
	nor *LIBAVR_ATMEGA_U* is defined, that port is automatically disabled.
	
	CAUTION (ATtiny): The ATtiny has no port C or D, so if *LIBAVR_ATTINY* is
	defined, those ports are automatically disabled.
	
	CAUTION (ATmegaU) : Port C is very small on the ATmegaU (only two pins),
	so if *LIBAVR_ATMEGA_U* is defined, the port called "C" in this module is
	actually hardware port F, while the port called "A" is hardware port C.
	(Confusing, I know, but *they started it* with their weird port layout.)
*/


/// Section: Configuration Macros

/**
	Macro: TBOUNCER_TICK_COUNT_BITS
	The number of bits in the tick count field of an I/O pin's debouncing
	state variable. Configuration macro.
	
	This must be in the range 1-7, and the sum of *TBOUNCER_TICK_COUNT_BITS*
	and <TBOUNCER_NEQ_COUNT_BITS> must be less than or equal to 8.
	
	Default value: 5
*/
#ifndef TBOUNCER_TICK_COUNT_BITS
#define TBOUNCER_TICK_COUNT_BITS 5
#endif

/**
	Macro: TBOUNCER_NEQ_COUNT_BITS
	The number of bits in the tick count field of an I/O pin's debouncing
	state variable. Configuration macro.
	
	This must be in the range 1-7, and the sum of *TBOUNCER_NEQ_COUNT_BITS*
	and <TBOUNCER_TICK_COUNT_BITS> must be less than or equal to 8.
	
	Default value: 3
*/
#ifndef TBOUNCER_NEQ_COUNT_BITS
#define TBOUNCER_NEQ_COUNT_BITS 3
#endif

/**
	Macro: TBOUNCER_TICKS_FOR_UPDATE
	The number of debouncer ticks that must pass after a pin value update before
	the debouncer starts counting consecutive pin input values different from
	the current debounced value. If the debounced value was last updated at tick
	*T*, then the debouncer will start counting different input values at tick
	*T + TBOUNCER_TICKS_FOR_UPDATE*. Configuration macro.
	
	The value of this macro must be in the range 1-(*TBOUNCER_TICKS_FOR_UPDATE_MAX*).
	
	Default value: 5
*/
#ifndef TBOUNCER_TICKS_FOR_UPDATE
#define TBOUNCER_TICKS_FOR_UPDATE 5
#endif

/**
	Macro: TBOUNCER_NEQ_FOR_UPDATE
	The number of consecutive pin input values (as obtained at each debouncer
	tick) different from the current debounced value that are required to update
	the debounced value. If the first different input value was obtained at tick
	*T*, then the debouncer will update the debounced pin value at tick
	*T + TBOUNCER_NEQ_FOR_UPDATE*. Configuration macro.
	
	The value of this macro must be in the range 1-(*TBOUNCER_NEQ_FOR_UPDATE_MAX*).
	
	Default value: 2
*/
#ifndef TBOUNCER_NEQ_FOR_UPDATE
#define TBOUNCER_NEQ_FOR_UPDATE 2
#endif


/// Section: API

/**
	Macro: TBOUNCER_TICKS_FOR_UPDATE_MAX
	Maximum allowed value of <TBOUNCER_TICKS_FOR_UPDATE>. Constant macro.
	Equal to *2^TBOUNCER_TICK_COUNT_BITS*.
*/
#define TBOUNCER_TICKS_FOR_UPDATE_MAX (1 << TBOUNCER_TICK_COUNT_BITS)

/**
	Macro: TBOUNCER_NEQ_FOR_UPDATE_MAX
	Maximum allowed value of <TBOUNCER_NEQ_FOR_UPDATE>. Constant macro.
	Equal to *2^TBOUNCER_NEQ_COUNT_BITS*.
*/
#define TBOUNCER_NEQ_FOR_UPDATE_MAX (1 << TBOUNCER_NEQ_COUNT_BITS)

/**
	Macro: TBOUNCER_ALL
	This value can be given to the pin change notification mask arguments of
	<tbouncer_init> to indicate that all the pins of the corresponding I/O port
	should trigger pin change notifications. Constant macro.
*/
#define TBOUNCER_ALL 0xff

/**
	Macro: TBOUNCER_INIT
	Wrapper macro for <tbouncer_init>. Omits the pin change notification bit mask
	arguments for any ports that are not available on the target device.
	Function-like macro.
	
	+ If *LIBAVR_ATTINY* is defined, ports A and B are available.
	+ If *LIBAVR_ATMEGA_U* is defined, ports A, B, C and D are available.
	  Note, however, that port "C" is actually port F, while port "A" is port C.
	+ Otherwise, ports B, C and D are available.
	
	Parameters:
		TNC - A TCSB value (see <sched_task>) containing the task instance and
			category numbers that should be used by the debouncer task.
			(The sleep bit is ignored.)
		PD - Scheduler delay of the debouncer task. Determines how often
			the I/O ports are sampled and the debounced pin values updated.
		[ABCD]M - Device-dependent pin change notification bit mask arguments.
			See <tbouncer_init> and the explanation above.
		AC - Target task categories for pin change notifications. Will be used to
			initialize <tbouncer_task_cats>.
		IM - Bit mask of pin change notification query. Will be used to initialize
			<tbouncer_invoke_mask>.
		IS - Bit pattern of pin change notification query. Will be used to initialize
			<tbouncer_invoke_st>.
*/
#if defined(LIBAVR_ATTINY)

#ifndef TBOUNCER_DISABLE_PORT_C
#define TBOUNCER_DISABLE_PORT_C
#endif

#ifndef TBOUNCER_DISABLE_PORT_D
#define TBOUNCER_DISABLE_PORT_D
#endif

#define TBOUNCER_INIT(TNC, PD, AM, BM, AC, IM, IS) \
	tbouncer_init((TNC), (PD), (AM), (BM), 0, 0, (AC), (IM), (IS))

#elif defined(LIBAVR_ATMEGA_U)

#define TBOUNCER_INIT(TNC, PD, AM, BM, CM, DM, AC, IM, IS) \
	tbouncer_init((TNC), (PD), (AM), (BM), (CM), (DM), (AC), (IM), (IS))

#else

#ifndef TBOUNCER_DISABLE_PORT_A
#define TBOUNCER_DISABLE_PORT_A
#endif

#define TBOUNCER_INIT(TNC, PD, BM, CM, DM, AC, IM, IS) \
	tbouncer_init((TNC), (PD), 0, (BM), (CM), (DM), (AC), (IM), (IS))

#endif

#ifndef TBOUNCER_DISABLE_PORT_A
/**
	Variable: tbouncer_a_mask
	Pin change notification bit mask for I/O port A. Notifications will be
	sent for a given pin only if the corresponding bit in this mask is set
	to one (1).
*/
extern uint8_t tbouncer_a_mask;

/**
	Variable: tbouncer_a
	Debounced pin values for I/O port A.
*/
extern uint8_t tbouncer_a;

/**
	Variable: tbouncer_a_prev
	Previous debounced pin values for I/O port A.
*/
extern uint8_t tbouncer_a_prev;

/**
	Variable: tbouncer_a_diff
	Pin change flags for I/O port A. Equal to <tbouncer_a> XOR <tbouncer_a_prev>.
*/
extern uint8_t tbouncer_a_diff;

/**
	Macro: TBOUNCER_A_RISING
	Rising edge flags for I/O port A. Expression macro.
	Evaluates to <tbouncer_a> AND NOT <tbouncer_a_prev>.
*/
#define TBOUNCER_A_RISING (tbouncer_a & ~tbouncer_a_prev)

/**
	Macro: TBOUNCER_A_FALLING
	Falling edge flags for I/O port A. Expression macro.
	Evaluates to NOT <tbouncer_a> AND <tbouncer_a_prev>.
*/
#define TBOUNCER_A_FALLING (~tbouncer_a & tbouncer_a_prev)
#endif

#ifndef TBOUNCER_DISABLE_PORT_B
/**
	Variable: tbouncer_b_mask
	Pin change notification bit mask for I/O port B. Notifications will be
	sent for a given pin only if the corresponding bit in this mask is set
	to one (1).
*/
extern uint8_t tbouncer_b_mask;

/**
	Variable: tbouncer_b
	Debounced pin values for I/O port B.
*/
extern uint8_t tbouncer_b;

/**
	Variable: tbouncer_b_prev
	Previous debounced pin values for I/O port B.
*/
extern uint8_t tbouncer_b_prev;

/**
	Variable: tbouncer_b_diff
	Pin change flags for I/O port B. Equal to <tbouncer_b> XOR <tbouncer_b_prev>.
*/
extern uint8_t tbouncer_b_diff;

/**
	Macro: TBOUNCER_B_RISING
	Rising edge flags for I/O port B. Expression macro.
	Evaluates to <tbouncer_b> AND NOT <tbouncer_b_prev>.
*/
#define TBOUNCER_B_RISING (tbouncer_b & ~tbouncer_b_prev)

/**
	Macro: TBOUNCER_B_FALLING
	Falling edge flags for I/O port B. Expression macro.
	Evaluates to NOT <tbouncer_b> AND <tbouncer_b_prev>.
*/
#define TBOUNCER_B_FALLING (~tbouncer_b & tbouncer_b_prev)
#endif

#ifndef TBOUNCER_DISABLE_PORT_C
/**
	Variable: tbouncer_c_mask
	Pin change notification bit mask for I/O port C. Notifications will be
	sent for a given pin only if the corresponding bit in this mask is set
	to one (1).
*/
extern uint8_t tbouncer_c_mask;

/**
	Variable: tbouncer_c
	Debounced pin values for I/O port C.
*/
extern uint8_t tbouncer_c;

/**
	Variable: tbouncer_c_prev
	Previous debounced pin values for I/O port C.
*/
extern uint8_t tbouncer_c_prev;

/**
	Variable: tbouncer_c_diff
	Pin change flags for I/O port C. Equal to <tbouncer_c> XOR <tbouncer_c_prev>.
*/
extern uint8_t tbouncer_c_diff;

/**
	Macro: TBOUNCER_C_RISING
	Rising edge flags for I/O port C. Expression macro.
	Evaluates to <tbouncer_c> AND NOT <tbouncer_c_prev>.
*/
#define TBOUNCER_C_RISING (tbouncer_c & ~tbouncer_c_prev)

/**
	Macro: TBOUNCER_C_FALLING
	Falling edge flags for I/O port C. Expression macro.
	Evaluates to NOT <tbouncer_c> AND <tbouncer_c_prev>.
*/
#define TBOUNCER_C_FALLING (~tbouncer_c & tbouncer_c_prev)
#endif

#ifndef TBOUNCER_DISABLE_PORT_D
/**
	Variable: tbouncer_d_mask
	Pin change notification bit mask for I/O port D. Notifications will be
	sent for a given pin only if the corresponding bit in this mask is set
	to one (1).
*/
extern uint8_t tbouncer_d_mask;

/**
	Variable: tbouncer_d
	Debounced pin values for I/O port D.
*/
extern uint8_t tbouncer_d;

/**
	Variable: tbouncer_d_prev
	Previous debounced pin values for I/O port D.
*/
extern uint8_t tbouncer_d_prev;

/**
	Variable: tbouncer_d_diff
	Pin change flags for I/O port D. Equal to <tbouncer_d> XOR <tbouncer_d_prev>.
*/
extern uint8_t tbouncer_d_diff;

/**
	Macro: TBOUNCER_D_RISING
	Rising edge flags for I/O port D. Expression macro.
	Evaluates to <tbouncer_d> AND NOT <tbouncer_d_prev>.
*/
#define TBOUNCER_D_RISING (tbouncer_d & ~tbouncer_d_prev)

/**
	Macro: TBOUNCER_D_FALLING
	Falling edge flags for I/O port D. Expression macro.
	Evaluates to NOT <tbouncer_d> AND <tbouncer_d_prev>.
*/
#define TBOUNCER_D_FALLING (~tbouncer_d & tbouncer_d_prev)
#endif

/**
	Variable: tbouncer_task_cats
	Contains a set of task category bit flags. Tasks in the indicated categories will
	be awakened by the debouncer task when there is a change in the debounced value
	of any I/O pin that is selected in one of the pin change notification bit masks
	(i.e. <tbouncer_b_mask>, <tbouncer_c_mask> and <tbouncer_d_mask>).
	
	CAUTION: When the tasks awakened via this variable are executed, the debounced pin
	values may have changed again. If it's important not to miss any pin changes, use
	the <tbouncer_invoke_mask> and <tbouncer_invoke_st> variables to invoke tasks
	synchronously.
*/
extern sched_catflags tbouncer_task_cats;

/**
	Variable: tbouncer_invoke_mask
	Contains a TCSB bit mask. Together with <tbouncer_invoke_st>, this variable
	defines a TCSB query. When there is a change in the debounced value of any
	I/O pin that is selected in one of the pin change notification bit masks
	(i.e. <tbouncer_b_mask>, <tbouncer_c_mask> and <tbouncer_d_mask>), the
	debouncer task will use the function <sched_invoke_all> to invoke all tasks
	that match this TCSB query.
*/
extern uint8_t tbouncer_invoke_mask;

/**
	Variable: tbouncer_invoke_st
	Contains a TCSB bit pattern. Refer to the documentation of <tbouncer_invoke_mask>
	for an explanation.
*/
extern uint8_t tbouncer_invoke_st;

/**
	Function: tbouncer_init
	Initializes the debouncer module. Creates the debouncer task.
	
	Parameters:
		task_num_cat - A TCSB value (see <sched_task>) containing the task instance
			and category numbers that should be used by the debouncer task.
			(The sleep bit is ignored.)
		poll_delay - Scheduler delay of the debouncer task. Determines how
			often the I/O ports are sampled and the debounced pin values updated.
		a_mask - Pin change notification bit mask for I/O port A. Will be used
			to initialize <tbouncer_a_mask>.
		b_mask - Pin change notification bit mask for I/O port B. Will be used
			to initialize <tbouncer_b_mask>.
		c_mask - Pin change notification bit mask for I/O port C. Will be used
			to initialize <tbouncer_c_mask>.
		d_mask - Pin change notification bit mask for I/O port D. Will be used
			to initialize <tbouncer_d_mask>.
		awaken_cats - Target task categories for pin change notifications.
			Will be used to initialize <tbouncer_task_cats>.
		invoke_mask - Bit mask of pin change notification query. Will be used
			to initialize <tbouncer_invoke_mask>.
		invoke_st - Bit pattern of pin change notification query. Will be used
			to initialize <tbouncer_invoke_st>.
*/
void tbouncer_init(
	uint8_t task_num_cat, sched_time poll_delay,
	uint8_t a_mask, uint8_t b_mask, uint8_t c_mask, uint8_t d_mask,
	sched_catflags awaken_cats, uint8_t invoke_mask, uint8_t invoke_st);

/**
	Function: tbouncer_shutdown
	Shuts down the debouncer module. Stops the debouncer task.
*/
void tbouncer_shutdown(void);

#ifdef LIBAVR_TEST_BUILD
void tbouncer_update(uint8_t pina, uint8_t pinb, uint8_t pinc, uint8_t pind, uint8_t pinf);
#endif

#endif
