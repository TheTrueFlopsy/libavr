#ifndef AVR_TBOUNCER_H
#define AVR_TBOUNCER_H

#include <stdint.h>

#include "task_sched.h"

/**
	File: tbouncer.h
	Input debouncing module that uses the <task_sched.h> task scheduler.
	
	NOTE: To disable debouncing on an ATmega I/O port, define the macro
	TBOUNCER_DISABLE_PORT_{x}, where {x} is "B", "C" or "D".
*/

/**
	Macro: TBOUNCER_ALL
	This value can be given to the pin change notification mask arguments of
	<tbouncer_init> to indicate that all the pins of the corresponding I/O port
	should trigger pin change notifications.
*/
#define TBOUNCER_ALL 0xff

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
	Rising edge flags for I/O port B. Evaluates to <tbouncer_b> AND NOT <tbouncer_b_prev>.
*/
#define TBOUNCER_B_RISING (tbouncer_b & ~tbouncer_b_prev)

/**
	Macro: TBOUNCER_B_FALLING
	Falling edge flags for I/O port B. Evaluates to NOT <tbouncer_b> AND <tbouncer_b_prev>.
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
	Rising edge flags for I/O port C. Evaluates to <tbouncer_c> AND NOT <tbouncer_c_prev>.
*/
#define TBOUNCER_C_RISING (tbouncer_c & ~tbouncer_c_prev)

/**
	Macro: TBOUNCER_C_FALLING
	Falling edge flags for I/O port C. Evaluates to NOT <tbouncer_c> AND <tbouncer_c_prev>.
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
	Rising edge flags for I/O port D. Evaluates to <tbouncer_d> AND NOT <tbouncer_d_prev>.
*/
#define TBOUNCER_D_RISING (tbouncer_d & ~tbouncer_d_prev)

/**
	Macro: TBOUNCER_D_FALLING
	Falling edge flags for I/O port D. Evaluates to NOT <tbouncer_d> AND <tbouncer_d_prev>.
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
		task_num_cat - A TCSB value containing the task instance and category
			numbers that should be used by the debouncer task. (The SLP bit
			of the argument is ignored.)
		poll_delay - Scheduler delay of the debouncer task. Determines how
			often the I/O ports are sampled and the debounced pin values updated.
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
	uint8_t b_mask, uint8_t c_mask, uint8_t d_mask,
	sched_catflags awaken_cats, uint8_t invoke_mask, uint8_t invoke_st);

/**
	Function: tbouncer_shutdown
	Shuts down the debouncer module. Stops the debouncer task.
*/
void tbouncer_shutdown(void);

#endif
