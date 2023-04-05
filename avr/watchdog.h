#ifndef AVR_WATCHDOG_H
#define AVR_WATCHDOG_H

// TODO: Test the watchdog module, including the "auto_watchdog" header.
// TODO: Document the watchdog module, including the "auto_watchdog" header.

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

#define WATCHDOG_DISABLE_ON_MCU_RESET \
	void watchdog_disable_on_mcu_reset(void) __attribute__ ((naked, used, section (".init3"))); \
	void watchdog_disable_on_mcu_reset(void) { \
		__asm__ ("out 0x34, r1"); /* Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.) */ \
		wdt_disable();  /* Disable the watchdog timer. */ \
	}

//uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((section (".noinit"), address (RAMEND)));
//uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((noinit, address (RAMEND)));

#define WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS \
	void watchdog_disable_on_mcu_reset_save_flags(void) __attribute__ ((naked, used, section (".init3"))); \
	void watchdog_disable_on_mcu_reset_save_flags(void) { \
		__asm__ ( \
			"in r0, 0x34" "\n\t" /* Get MCU reset flags from MCUSR. */ \
			"push r0" "\n\t" /* Save MCU reset flags on stack. */ \
			"out 0x34, r1" "\n\t"); /* Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.) */ \
		wdt_disable();  /* Disable the watchdog timer. */ \
	}

extern volatile uint8_t watchdog_saved_mcu_reset_flags __attribute__ ((address (RAMEND)));

void watchdog_reset_mcu(void) __attribute__ ((noreturn));

#endif
