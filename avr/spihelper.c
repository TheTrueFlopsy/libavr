
#include <avr/io.h>

#include "spihelper.h"

#define BV(N) (1 << (N))

void spihelper_mstr_init(uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl) {
	if (ctrl & BV(MSTR)) {
		// Set pull-up on SS pin to avoid accidental triggering of slave-on-demand feature.
		SPI_PORT |= BV(SPI_SS);
	}
	
	// Set high/pull-up state on slave select pins (may or may not include the Slave mode SS pin).
	PORTB |= ss_b;
	PORTC |= ss_c;
	PORTD |= ss_d;
	
	// Make slave select pins outputs.
	DDRB |= ss_b;
	DDRC |= ss_c;
	DDRD |= ss_d;
	
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
