#ifndef AVR_SPIHELPER_H
#define AVR_SPIHELPER_H

#include <stdint.h>

#ifndef SPI_NO_ASYNC_API
#include "task_sched.h"
#endif

/**
	File: spihelper.h
	Helper module for SPI communication. Provides an asynchronous,
	interrupt-driven request-response API. This API is master mode only,
	for the time being.
	
	NOTE: To disable the asynchronous request-response API (see <spihelper_request>),
	define the macro *SPI_NO_ASYNC_API*. Doing this means that this module won't
	define or depend on the *SPI_STC_vect* ISR, and will use less memory.
	
	CAUTION (ATmegaU): Port C is very small on the ATmegaU (only two pins),
	so if *LIBAVR_ATMEGA_U* is defined, the port called "C" in this module is
	actually hardware port F.
*/

/**
	Macro: SPI_PINR
	Reference to the I/O input register (PINx) for the pins used by the SPI peripheral.
	Constant macro.
*/
#define SPI_PINR PINB

/**
	Macro: SPI_DDR
	Reference to the I/O direction register (DDRx) for the pins used by the SPI peripheral.
	Constant macro.
*/
#define SPI_DDR  DDRB

/**
	Macro: SPI_PORT
	Reference to the I/O output register (PORTx) for the pins used by the SPI peripheral.
	Constant macro.
*/
#define SPI_PORT PORTB

#ifdef LIBAVR_ATMEGA_U

#define SPI_SS   PINB0
#define SPI_MOSI PINB2
#define SPI_MISO PINB3
#define SPI_SCK  PINB1

#else

/**
	Macro: SPI_SS
	Register bit number of the SS pin of the SPI peripheral. Constant macro.
*/
#define SPI_SS   PINB2

/**
	Macro: SPI_MOSI
	Register bit number of the MOSI pin of the SPI peripheral. Constant macro.
*/
#define SPI_MOSI PINB3

/**
	Macro: SPI_MISO
	Register bit number of the MISO pin of the SPI peripheral. Constant macro.
*/
#define SPI_MISO PINB4

/**
	Macro: SPI_SCK
	Register bit number of the SCK pin of the SPI peripheral. Constant macro.
*/
#define SPI_SCK  PINB5

#endif

/**
	Macro: SPI_SS_PULLUP
	Alias of the bit number of the MSTR bit in the SPCR register. May be used to
	clarify its use to enable the pull-up on the SS pin when set in the *ctrl*
	argument of <spihelper_mstr_init> or <spihelper_async_mstr_init>. Constant macro.
*/
#define SPI_SS_PULLUP MSTR


#ifndef SPI_NO_ASYNC_API

/**
	Enum: SPI Module State/Result Codes
	
	NOTE: The error codes are those with names starting with "SPI_E_". These all have
	numeric values greater than or equal to *SPI_E_UNSPECIFIED*.
	
	SPI_DISABLED      - module is disabled
	SPI_SLAVE         - module is configured as a slave
	SPI_READY         - module is idle as master and ready to begin a new operation
	SPI_ACTIVE        - module is currently performing an operation
	SPI_E_UNSPECIFIED - unspecified error
	SPI_E_LENGTH      - SPI length error (bytes to transmit and receive both zero)
	SPI_E_STATE       - attempted operation not allowed in current module state
*/
enum {
	SPI_DISABLED      = 0,
	SPI_SLAVE         = 1,
	SPI_READY         = 2,
	SPI_ACTIVE        = 3,
	SPI_E_UNSPECIFIED = 4,
	SPI_E_LENGTH      = 5,
	SPI_E_STATE       = 6
};

/**
	Ref: spi_state
	The type of state and result codes for the SPI module.
*/
typedef uint8_t spi_state;

/**
	Variable: spi_task_cats
	Tasks in the categories indicated by this <sched_catflags> value will be
	notified by the SPI module's ISR when a request operation finishes (either
	successfully or with an error).
*/
extern volatile sched_catflags spi_task_cats;

/**
	Variable: spi_request_state
	Contains a state code representing the current state of the SPI module.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern volatile spi_state spi_request_state;

/**
	Macro: SPI_IS_READY
	Evaluates to a true value if and only if the SPI module is initialized and
	ready to begin a new operation. Expression macro.
*/
#define SPI_IS_READY (spi_request_state == SPI_READY)

/**
	Macro: SPI_IS_ACTIVE
	Evaluates to a true value if and only if the SPI module is busy with an
	ongoing operation. Expression macro.
*/
#define SPI_IS_ACTIVE (spi_request_state == SPI_ACTIVE)

/**
	Function: spihelper_async_mstr_init
	Configures the SPI module for Master mode and sets task categories for
	notifications.
	
	NOTE: This function invokes <spihelper_mstr_init> internally. See that
	function for full details.
	
	Parameters:
		ss_b - Bit flags specifying the pins in I/O port B that should be readied
			for use as SPI Slave Select output pins.
		ss_c - Bit flags specifying the pins in I/O port C that should be readied
			for use as SPI Slave Select output pins.
		ss_d - Bit flags specifying the pins in I/O port D that should be readied
			for use as SPI Slave Select output pins.
		ctrl - Value to assign to the SPI Control Register (SPCR). If the MSTR bit
			in this parameter is set to one, then the Slave mode Slave Select pin
			will be made an input and have its pull-up enabled, to avoid accidental
			triggering of the slave-on-demand feature of the SPI peripheral.
		task_cats - A set of bit flags specifying task categories whose members
			should be notified when this module has finished an SPI transaction.
			Used to initialize <spi_task_cats>.
*/
void spihelper_async_mstr_init(
	uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl, sched_catflags task_cats);

#endif

/**
	Function: spihelper_mstr_init
	Configures the SPI module for Master mode.
	
	I/O pins that are marked as Slave Select output pins via *ss_x* parameters
	will be configured as outputs and driven high by this function. The output
	high / pull-up enable state will be set before the data direction is set
	to output.
	
	NOTE: When this function returns, the SPI Enable (SPE) bit in the SPI Control
	Register (SPCR) will always be set to one, regardless of its value in the
	*ctrl* parameter. The Master/Slave Select (MSTR) bit will likewise always be
	set to one.
	
	NOTE: This function invokes <spihelper_init> internally.
	
	Parameters:
		ss_b - Bit flags specifying the pins in I/O port B that should be readied
			for use as SPI Slave Select output pins.
		ss_c - Bit flags specifying the pins in I/O port C that should be readied
			for use as SPI Slave Select output pins.
		ss_d - Bit flags specifying the pins in I/O port D that should be readied
			for use as SPI Slave Select output pins.
		ctrl - Value to assign to the SPI Control Register (SPCR). If the MSTR bit
			in this parameter is set to one, then the Slave mode Slave Select pin
			will be made an input and have its pull-up enabled, to avoid accidental
			triggering of the slave-on-demand feature of the SPI peripheral.
*/
void spihelper_mstr_init(uint8_t ss_b, uint8_t ss_c, uint8_t ss_d, uint8_t ctrl);

/**
	Function: spihelper_init
	Configures the SPI module for either Master or Slave mode, depending on the
	value of the Master/Slave Select (MSTR) bit in the *ctrl* argument.
	
	NOTE: When this function returns, the SPI Enable (SPE) bit in the SPI Control
	Register (SPCR) will always be set to one, regardless of its value in the *ctrl*
	parameter.
	
	Parameters:
		ctrl - Value to assign to the SPI Control Register (SPCR). If the MSTR bit
			in this parameter is set to one, the SPI module will be configured for
			Master mode, otherwise, it will be configured for Slave mode.
*/
void spihelper_init(uint8_t ctrl);

/**
	Function: spihelper_exchange_bytes
	Synchronously exchanges a single pair of bytes via the SPI peripheral.
	
	NOTE: This function does NOT manipulate any Slave Select pins. Application
	developers MUST ensure that appropriate slave selection has been performed
	before this function is called.
	
	CAUTION: This function does NOT check the current state of the SPI module
	and MUST NOT be called when the module is uninitialized or busy with an
	asynchronous operation.
	
	Parameters:
		data_out - Data byte to transmit.
	
	Returns:
		The received data byte.
*/
uint8_t spihelper_exchange_bytes(uint8_t data_out);

#ifndef SPI_NO_ASYNC_API

/**
	Function: spihelper_request
	Initiates an SPI request operation consisting of a transmit phase followed
	by a receive phase. Either the transmit or the receive phase can be
	omitted, but not both. This function only works if the SPI module has been
	configured for Master mode.
	
	Completion of an SPI request initiated by this function can be
	detected by polling <spi_request_state> (e.g. via the <SPI_IS_ACTIVE>
	macro) and by specifying task categories to be notified via <spi_task_cats>.
	
	NOTE: This function does NOT manipulate any Slave Select pins. Application
	developers MUST ensure that appropriate slave selection has been performed
	before this function is called.
	
	NOTE: The *bfr_out* and *bfr_in* arguments MAY point to the same buffer.
	This function ensures that any byte to transmit from *bfr_out[i]*
	is transmitted before any received byte is stored at *bfr_in[i]*.
	
	NOTE: The default dummy byte transmitted by this function is zero.
	A different dummy value can be specified by defining the macro *SPI_DUMMY_BYTE*
	as the desired dummy value. (Make this a compile-time constant, please.)
	
	Parameters:
		n_out - Number of data bytes to transmit. If this argument is zero, the
			transmit phase will be omitted. If *offset_in+n_in* is greater than
			*n_out*, the SPI module will begin transmitting dummy bytes after
			transmitting *n_out* bytes from *bfr_out*.
		bfr_out - Pointer to the data bytes to transmit. If *n_out* is zero,
			this MAY be a null pointer.
		n_in - Number of data bytes to receive. If this argument is zero, the
			receive phase will be omitted.
		offset_in - Number of received bytes to discard before starting the
			receive phase. The total number of bytes exchanged to perform the SPI
			request operation will be *max(n_out, offset_in+n_in)*, unless *n_in*
			is zero. If *n_in* is zero, this argument is ignored.
		bfr_in - Pointer to memory to copy received data bytes into. If *n_in*
			is zero, this MAY be a null pointer.
	
	Returns:
		<SPI_ACTIVE> if and only if the request operation was successfully started,
		otherwise an error code.
*/
spi_state spihelper_request(
	uint8_t n_out, volatile const uint8_t *bfr_out,
	uint8_t n_in, uint8_t offset_in, volatile uint8_t *bfr_in);

#endif

/**
	Function: spihelper_shutdown
	Disables the SPI module.
*/
void spihelper_shutdown(void);

#endif
