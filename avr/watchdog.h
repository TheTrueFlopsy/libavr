#ifndef AVR_WATCHDOG_H
#define AVR_WATCHDOG_H

/**
	File: watchdog.h
	Watchdog timer control facilities. See also <auto_watchdog.h>.
*/

// TODO: Test the watchdog module, including the "auto_watchdog" header.

//#define WATCHDOG_STRINGIZE_INNER(A) #A
//#define WATCHDOG_STRINGIZE(A) WATCHDOG_STRINGIZE_INNER(A)

#include <stdint.h>
#include <avr/io.h>
#include <avr/wdt.h>

/*static inline __attribute__ ((always_inline)) uint8_t watchdog_disable_on_mcu_reset(void) {
	uint8_t mcusr_copy = MCUSR;  // Save MCU reset flags for inspection.
	MCUSR = 0;  // Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.)
	wdt_disable();  // Disable the watchdog timer.
	return mcusr_copy;
}*/

// ISSUE: Replace the raw hex I/O address 0x34 with a symbolic reference to MCUSR?

/**
	Macro: WATCHDOG_DISABLE_ON_MCU_RESET
	Defines a watchdog-disabling routine that is executed automatically at MCU reset.
	The routine is executed before *main* is called. The routine clears the MCUSR register to ensure
	that no reset flag that implicitly enables the watchdog timer remains set. After MCUSR has been
	cleared, the routine disables the watchdog timer and then firmware initialization continues.
	Function definition macro.
	
	NOTE: This macro can be implicitly expanded in a code file by including the header <auto_watchdog.h>
	(with both *WATCHDOG_AUTODISABLE_SAVE_FLAGS* and *WATCHDOG_DO_NOT_AUTODISABLE* undefined).
*/
#define WATCHDOG_DISABLE_ON_MCU_RESET \
	void watchdog_disable_on_mcu_reset(void) __attribute__ ((naked, used, section (".init3"))); \
	void watchdog_disable_on_mcu_reset(void) { \
		__asm__ ( \
			"out 0x34, r1" "\n\t"); /* Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.) */ \
		wdt_disable();  /* Disable the watchdog timer. */ \
	}

//uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((section (".noinit"), address (RAMEND)));
//uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((noinit, address (RAMEND)));

/**
	Macro: WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS
	Defines a watchdog-disabling routine that is executed automatically at MCU reset.
	The routine is executed before *main* is called. The routine first saves the contents
	of the MCUSR register in <watchdog_saved_mcu_reset_flags>, then clears the register to ensure
	that no reset flag that implicitly enables the watchdog timer remains set. After MCUSR has been
	cleared, the routine disables the watchdog timer and then firmware initialization continues.
	Function definition macro.
	
	NOTE: This macro can be implicitly expanded in a code file by defining
	the macro *WATCHDOG_AUTODISABLE_SAVE_FLAGS* and then including the header <auto_watchdog.h>.
*/
#define WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS \
	void watchdog_disable_on_mcu_reset_save_flags(void) __attribute__ ((naked, used, section (".init3"))); \
	void watchdog_disable_on_mcu_reset_save_flags(void) { \
		__asm__ ( \
			"in r0, 0x34" "\n\t" /* Get MCU reset flags from MCUSR. */ \
			"push r0" "\n\t" /* Save MCU reset flags on stack. */ \
			"out 0x34, r1" "\n\t"); /* Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.) */ \
		wdt_disable();  /* Disable the watchdog timer. */ \
	}

/**
	Variable: watchdog_saved_mcu_reset_flags
	When the watchdog is disabled via <WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS>, the contents
	of the MCUSR register are stored here before the register is cleared.
*/
extern uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((address (RAMEND)));

/**
	Function: watchdog_reset_mcu
	Uses the watchdog timer to trigger a reset of the MCU. There is a delay of about 16 ms
	before the reset happens, the function will disable interrupts and then busy-loop
	to wait out this delay.
	
	CAUTION: The watchdog timer remains enabled after the MCU reset. To prevent repeated resets,
	the watchdog timer MUST be disabled (or reset or reconfigured) as soon as possible when
	the firmware is restarted after a call to this function.
*/
void watchdog_reset_mcu(void) __attribute__ ((noreturn));

#endif
