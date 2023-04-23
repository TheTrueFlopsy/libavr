
#include <avr/interrupt.h>

#include "watchdog.h"

#define BV(N) (1 << (N))

void watchdog_reset_mcu(void) {
	cli();  // Disable interrupts.
	WDTCSR &= ~BV(WDIE);  // Disable watchdog timer interrupt mode.
	wdt_enable(WDTO_15MS);  // Enable watchdog timer with minimal timeout.
	
	while (1)
		;  // Busy loop until the watchdog times out.
}
