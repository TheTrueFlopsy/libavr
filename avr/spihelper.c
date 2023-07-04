
#include <avr/io.h>

#ifndef SPI_NO_ASYNC_API
#include <avr/interrupt.h>
#endif

#include "bitops.h"
#include "spihelper.h"

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


#ifndef SPI_NO_ASYNC_API

#ifndef SPI_DUMMY_BYTE
#define SPI_DUMMY_BYTE 0
#endif

volatile sched_catflags spi_task_cats = 0;
volatile spi_state spi_request_state = SPI_DISABLED;

static volatile uint8_t n_to_transmit;
static volatile const uint8_t *volatile transmit_ptr;

static volatile uint8_t n_to_receive;
static volatile uint8_t receive_offset;
static volatile uint8_t *volatile receive_ptr;

ISR(SPI_STC_vect) {
	uint8_t n_in = n_to_receive;
	
	if (n_in > 0) {  // We should store some received data bytes...
		uint8_t in_offset = receive_offset;
		
		if (in_offset > 0)  // ...but not yet.
			receive_offset = in_offset - 1;  // Decrement counter for receive offset.
		else {  // ...right here and now.
			*(receive_ptr++) = SPDR;
			n_to_receive = --n_in;  // Decrement counter for bytes to receive.
		}
	}
	
	uint8_t n_out = n_to_transmit;
	
	if (n_in == 0 && n_out == 0) {  // SPI transaction ended.
		spi_request_state = SPI_READY;  // No longer busy.
		sched_isr_tcww |= spi_task_cats;  // Notify tasks.
		SPCR &= ~BV(SPIE);  // Disable SPI interrupt.
	}
	else if (n_out > 0) {  // We have more data bytes to transmit.
		n_to_transmit = n_out - 1;  // Decrement counter for bytes to transmit.
		SPDR = *(transmit_ptr++);  // Initiate transmission of data byte.
	}
	else  // We have no data bytes to transmit, but do have bytes to receive (n_in > 0).
		SPDR = SPI_DUMMY_BYTE;  // Initiate transmission of dummy byte.
}

void spihelper_async_mstr_init(
	uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl, sched_catflags task_cats)
{
	spi_task_cats = task_cats;
	spihelper_mstr_init(ss_b, ss_c, ss_d, ctrl);
}

#endif

void spihelper_mstr_init(uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl) {
	if (ctrl & BV(MSTR)) {
		// Make the SS pin an input
		SPI_DDR &= ~BV(SPI_SS);
		// Enable pull-up on SS pin to avoid accidental triggering of slave-on-demand feature.
		SPI_PORT |= BV(SPI_SS);
	}
	
	// IDEA: Implement an open-drain slave select mode, where the SS pins are toggled between
	// output-low and input-hiZ (with external pull-ups) instead of output-low and output-high.
	
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
	
#ifndef SPI_NO_ASYNC_API
	ctrl &= ~BV(SPIE);  // I'm sorry, Dave, but I decide when to enable the interrupt.
	spi_request_state = (ctrl & BV(MSTR)) ? SPI_READY : SPI_SLAVE;
#endif
	
	// Initialize the SPCR register.
	ctrl &= ~BV(SPE); // Don't enable the SPI module just yet.
	SPCR = ctrl;      // Configure the SPI module.
	SPCR |= BV(SPE);  // Enable the SPI module.
}

uint8_t spihelper_exchange_bytes(uint8_t data_out) {
	SPDR = data_out;
	
	while (!(SPSR & BV(SPIF)))
		/* do nothing */;
	
	return SPDR;
}

#ifndef SPI_NO_ASYNC_API

spi_state spihelper_request(
	uint8_t n_out, volatile const uint8_t *bfr_out,
	uint8_t n_in, uint8_t offset_in, volatile uint8_t *bfr_in)
{
	if (n_out == 0 && n_in == 0)
		return SPI_E_LENGTH;
	
	spi_state state = spi_request_state;
	
	if (state >= SPI_E_UNSPECIFIED)
		return state;
	else if (state != SPI_READY)
		return SPI_E_STATE;
	
	spi_request_state = SPI_ACTIVE;  // Initiate SPI transaction.
	
	// Prepare transmit and receive buffers.
	if (n_out > 0) {
		// Transmission of the first data byte starts immediately, so the ISR
		// doesn't need to deal with it.
		n_to_transmit = n_out - 1;
		transmit_ptr = bfr_out + 1;
	}
	else
		n_to_transmit = 0;
	
	n_to_receive = n_in;
	receive_offset = offset_in;
	receive_ptr = bfr_in;
	
	SPCR |= BV(SPIE);  // Enable SPI interrupt.
	
	// Initiate transmission of first data byte, or a dummy byte if there
	// is no data to transmit.
	SPDR = (n_out > 0) ? *bfr_out : SPI_DUMMY_BYTE;
	
	return SPI_ACTIVE;
}

#endif

void spihelper_shutdown(void) {
	SPCR &= ~(BV(SPE) | BV(SPIE)); // Disable the SPI module.
	
#ifndef SPI_NO_ASYNC_API
	spi_request_state = SPI_DISABLED;
#endif
}
