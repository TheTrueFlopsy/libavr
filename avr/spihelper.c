
#include <avr/io.h>

#include "spihelper.h"

#define BV(N) (1 << (N))

#define SPI_PORTB PORTB
#define SPI_DDRB DDRB

#ifdef LIBAVR_ATMEGA_U
// NOTE: Port C sucks on the ATmegaU (only two pins), so we pretend that port F is port C.
#define SPI_PORTC PORTF
#define SPI_DDRC DDRF
#else
#define SPI_PORTC PORTC
#define SPI_DDRC DDRC
#endif

#define SPI_PORTD PORTD
#define SPI_DDRD DDRD

void spihelper_mstr_init(uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl) {
	if (ctrl & BV(MSTR)) {
		// Set pull-up on SS pin to avoid accidental triggering of slave-on-demand feature.
		SPI_PORT |= BV(SPI_SS);
	}
	
	// Set high/pull-up state on slave select pins (may or may not include the Slave mode SS pin).
	SPI_PORTB |= ss_b;
	SPI_PORTC |= ss_c;
	SPI_PORTD |= ss_d;
	
	// Make slave select pins outputs.
	SPI_DDRB |= ss_b;
	SPI_DDRC |= ss_c;
	SPI_DDRD |= ss_d;
	
	ctrl |= BV(MSTR);
	spihelper_init(ctrl);
}

void spihelper_init(uint8_t ctrl) {
	if (ctrl & BV(MSTR)) { // Initialize SPI module in Master mode.
		// Make the MISO pin an input (this is an automatic override, but just in case).
		SPI_DDR &= ~BV(SPI_MISO);
		// Make the MOSI and SCK pins outputs.
		SPI_DDR |= BV(SPI_MOSI) | BV(SPI_SCK);
	}
	else { // Initialize SPI module in Slave mode.
		// Make the SS, MOSI and SCK pins inputs (these are automatic overrides, but just in case).
		SPI_DDR &= ~(BV(SPI_SS) | BV(SPI_MOSI) | BV(SPI_SCK));
		// Make the MISO pin an output.
		SPI_DDR |= BV(SPI_MISO);
	}
	
	// Initialize the SPCR register.
	ctrl &= ~BV(SPE); // Don't enable the SPI module just yet.
	SPCR = ctrl;      // Configure the SPI module.
	SPCR |= BV(SPE);  // Enable the SPI module.
}

void spihelper_shutdown(void) {
	SPCR &= ~BV(SPE); // Disable the SPI module.
}
