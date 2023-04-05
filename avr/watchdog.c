
#include <avr/interrupt.h>

#include "watchdog.h"

#define BV(N) (1 << (N))

/*void watchdog_disable_on_mcu_reset(void) {
	__asm__ ("out 0x34, r1");  // Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.)
	wdt_disable();  // Disable the watchdog timer.
}

void watchdog_disable_on_mcu_reset_save_flags(void) {
	__asm__ (
		"in r0, 0x34" "\n\t"  // Get MCU reset flags from MCUSR.
		"push r0" "\n\t"  // Save MCU reset flags on stack.
		"out 0x34, r1" "\n\t");  // Clear MCU reset flags. (Necessary to ensure that the WDT is disabled.)
	wdt_disable();  // Disable the watchdog timer.
}*/

void watchdog_reset_mcu(void) {
	cli();  // Disable interrupts.
	WDTCSR &= ~BV(WDIE);  // Disable watchdog timer interrupt mode.
	wdt_enable(WDTO_15MS);  // Enable watchdog timer with minimal timeout.
	
	while (1)
		;  // Busy loop until the watchdog times out.
}
