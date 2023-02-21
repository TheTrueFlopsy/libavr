#ifndef AVR_MEMMON_H
#define AVR_MEMMON_H

#include <stddef.h>
#include <stdint.h>

#include "task_sched.h"
#include "task_tlv.h"

/**
	File: memmon.h
	A memory monitor module that uses the <task_sched.h> scheduler
	to periodically execute a task that sends messages about the content
	of specfied memory locations via the <task_tlv.h> module. The set of
	monitored memory locations can be updated while the task scheduler
	is running.
*/


/// Section: Data Types

/**
	Variable: memmon_ptr
	The type of pointers to memory locations in the memory monitor API.
*/
typedef uint8_t *memmon_ptr;

/**
	Variable: memmon_cptr
	The type of pointers to constant memory locations in the memory monitor API.
*/
typedef const uint8_t *memmon_cptr;

/**
	Macro: MEMMON_MAKE_PTR
	Converts a pointer to an arbitrary type into a pointer that is compatible
	with the memory monitor API (a <memmon_ptr>), without triggering compiler
	warnings. Function-like macro.
	
	Parameters:
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A <memmon_ptr> containing the same address as *P*.
*/
#define MEMMON_MAKE_PTR(P) ((memmon_ptr)TTLV_DATA_PTR(P))

/**
	Macro: MEMMON_MAKE_CPTR
	Converts a pointer to an arbitrary type into a pointer-to-constant that is
	compatible with the memory monitor API (a <memmon_cptr>), without triggering
	compiler warnings. Function-like macro.
	
	Parameters:
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A <memmon_cptr> containing the same address as *P*.
*/
#define MEMMON_MAKE_CPTR(P) ((memmon_cptr)TTLV_DATA_CPTR(P))

/**
	Struct: memmon_spec
	Specifies a memory monitor.
*/
typedef struct memmon_spec {
	/**
		Field: ptr
		Pointer to the memory location to monitor.
	*/
	memmon_cptr ptr;
	
	/**
		Field: size
		Size in bytes of the memory location to monitor.
	*/
	uint8_t size;
	
	/**
		Field: delay_div
		Notification rate divisor to apply to the monitor. The monitored
		memory location will be read and the obtained value sent in a TLV
		notification message once for every *delay_div* executions of the
		memory monitor task.
	*/
	uint8_t delay_div;
	
	/**
		Field: delay_count
		Used internally by the memory monitor task to keep track of the
		number of task executions since the lastest notification was sent.
	*/
	uint8_t delay_count;
} memmon_spec;

/**
	Struct: memmon_header
	Specifies the byte format of a memory monitor notification header.
*/
typedef struct __attribute__ ((__packed__)) memmon_header {
	/**
		Field: mon_i
		Index number of the memory monitor.
	*/
	uint8_t mon_i;
	
	/**
		Field: ptr
		Address of the monitored memory location.
	*/
	memmon_cptr ptr;
} memmon_header;


/// Section: Global Variables

/**
	Macro: MEMMON_MAX_SIZE
	Maximum size in bytes of a monitored memory location. Constant macro.
*/
#define MEMMON_MAX_SIZE (TTLV_MAX_LEN_INM - sizeof(memmon_header))

/**
	Variable: memmon_max_monitors
	The maximum number of simultaneously registered memory monitors.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern uint8_t memmon_max_monitors;

/**
	Variable: memmon_n_monitors
	The number of registered memory monitors.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern uint8_t memmon_n_monitors;

/**
	Variable: memmon_msg_count
	Number of notification attempts made by the memory monitor task.
	
	NOTE: This will roll over to zero if the number of notification
	attempts exceeds *UINT16_MAX*.
*/
extern uint16_t memmon_msg_count;

/**
	Variable: memmon_drop_count
	Number of failed notification attempts made by the memory monitor task.
	
	NOTE: This will roll over to zero if the number of failed notification
	attempts exceeds *UINT16_MAX*.
*/
extern uint16_t memmon_drop_count;

/**
	Variable: memmon_free_ram
	Available unused RAM in bytes. Updated at the start of each execution
	of the memory monitor task. Uses <memmon_get_free_ram> to obtain the
	amount of unused RAM.
*/
extern ptrdiff_t memmon_free_ram;


/// Section: API Functions

// TODO: Add a feature that makes the memmon module controllable
//       via TLV/INM messages. Basically, it should be possible
//       to invoke the "add" and "remove" operations via messages.

/**
	Function: memmon_get_free_ram
	Obtains the amount of available unused RAM.
	
	NOTE: This function can be used even if the memory monitor module
	hasn't been initialized with <memmon_init>.
	
	Returns:
		Available free RAM in bytes.
*/
ptrdiff_t memmon_get_free_ram(void);

/**
	Function: memmon_init
	Initializes the memory monitor module. Creates the memory monitor task.
	
	CAUTION: This function MUST be called before any memory monitors are
	registered.
	
	Parameters:
		task_num_cat - A TCSB value (see <sched_task>) containing the task instance
			and category numbers that should be used by the memory monitor task.
			(The sleep bit is ignored.)
		max_mon - Maximum number of simultaneously registered memory monitors.
		mon_array - Pointer to memory where <memmon_spec> structs representing
			registered memory monitors should be stored. The specified memory
			location MUST be large enough to store at least *max_mon* such structs
			and remain valid until <memmon_shutdown> is called.
		delay - Scheduler delay of the memory monitor task.
		tlv_typ - TLV message type identifier to use for memory monitor
			notification messages (<TTLV_MSG_T_MEMMON_DATA> is recommended).
		inm_dstadr - INM destination address to use for memory monitor
			notification messages.
*/
void memmon_init(
	uint8_t task_num_cat, uint8_t max_mon, memmon_spec *mon_array,
	sched_time delay, uint8_t tlv_typ, uint8_t inm_dstadr);

/**
	Function: memmon_add
	Registers a memory monitor.
	
	Parameters:
		mon - Pointer to a <memmon_spec> specifying the monitor to
			register.
	
	Returns:
		The index number of the registered memory monitor, or
		<memmon_max_monitors> if the monitor couldn't be registered.
*/
uint8_t memmon_add(const memmon_spec *mon);

/**
	Function: memmon_remove
	Unregisters a memory monitor with a specified index number.
	
	Parameters:
		mon_i - Index number of the monitor to unregister.
	
	Returns:
		A true value if and only if *mon_i* was the index number of a registered
		monitor that was removed by this call.
*/
uint8_t memmon_remove(uint8_t mon_i);

/**
	Function: memmon_remove_ptr
	Unregisters a memory monitor monitoring a specified location.
	
	Parameters:
		mon_ptr - Address of the memory location monitored by the
			monitor to unregister.
	
	Returns:
		A true value if and only if a matching registered memory monitor
		was found and removed by this call.
*/
uint8_t memmon_remove_ptr(memmon_cptr mon_ptr);

/**
	Function: memmon_shutdown
	Shuts down the memory monitor module. Stops the memory monitor task.
*/
void memmon_shutdown(void);

#endif
