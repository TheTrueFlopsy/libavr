#ifndef AVR_SPIHELPER_H
#define AVR_SPIHELPER_H

#include <stdint.h>

/**
	File: spihelper.h
	Helper functions and macros for communication via the ATmega's SPI hardware.
	
	CAUTION (ATmegaU) : Port C is very small on the ATmegaU (only two pins),
	so if *LIBAVR_ATMEGA_U* is defined, the port called "C" in this module is
	actually hardware port F.
*/

#define SPI_PINR PINB
#define SPI_DDR  DDRB
#define SPI_PORT PORTB

#ifdef LIBAVR_ATMEGA_U

#define SPI_SS   PINB0
#define SPI_MOSI PINB2
#define SPI_MISO PINB3
#define SPI_SCK  PINB1

#else

#define SPI_SS   PINB2
#define SPI_MOSI PINB3
#define SPI_MISO PINB4
#define SPI_SCK  PINB5

#endif

/**
	Function: spihelper_mstr_init
	Configures the SPI module for Master mode.
	
	I/O pins that are marked as Slave Select output pins via *ss_x* parameters
	will be configured as outputs and driven high by this function. The output
	high / pull-up enable state will be set before the data direction is set
	to output.
	
	NOTE: This function invokes <spihelper_init> internally.
	
	Parameters:
		ss_b - Bit flags specifying the pins in I/O port B that should be readied
			for use as SPI Slave Select output pins.
		ss_c - Bit flags specifying the pins in I/O port C that should be readied
			for use as SPI Slave Select output pins.
		ss_d - Bit flags specifying the pins in I/O port D that should be readied
			for use as SPI Slave Select output pins.
		ctrl - Value to assign to the SPI Control Register (SPCR). When this
			function returns, the SPI Enable (SPE) bit in the SPCR will always be
			set to one, regardless of its value in the *ctrl* parameter.
			The Master/Slave Select (MSTR) bit will likewise always be set to one.
*/
void spihelper_mstr_init(uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl);

/**
	Function: spihelper_init
	Configures the SPI module for either Master or Slave mode, depending on the
	value of the Master/Slave Select (MSTR) bit in the *ctrl* argument.
	
	Parameters:
		ctrl - Value to assign to the SPI Control Register (SPCR). When this
			function returns, the SPI Enable (SPE) bit in the SPCR will always be
			set to one, regardless of its value in the *ctrl* parameter.
			If the MSTR bit in this parameter is set to one, the SPI module will
			be configured for Master mode, otherwise, it will be configured for
			Slave mode.
*/
void spihelper_init(uint8_t ctrl);

/**
	Function: spihelper_shutdown
	Disables the SPI module.
*/
void spihelper_shutdown(void);

#endif
