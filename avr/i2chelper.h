#ifndef AVR_I2CHELPER_H
#define AVR_I2CHELPER_H

#include <stdint.h>

#include "task_sched.h"

/**
	File: i2chelper.h
	Helper module for I2C (aka TWI) communication. Provides an asynchronous,
	interrupt-driven request-response API. This API is master mode only,
	for the time being.
*/

#ifdef LIBAVR_ATMEGA_U

#define I2C_SCL_DDR  DDRD
#define I2C_SCL_PORT PORTD
#define I2C_SCL_PIN  PIND

#define I2C_SCL_DDR_PIN  BV(DDD0)
#define I2C_SCL_PORT_PIN BV(PORTD0)
#define I2C_SCL_PIN_PIN  BV(PIND0)

#define I2C_SDA_DDR  DDRD
#define I2C_SDA_PORT PORTD
#define I2C_SDA_PIN  PIND

#define I2C_SDA_DDR_PIN  BV(DDD1)
#define I2C_SDA_PORT_PIN BV(PORTD1)
#define I2C_SDA_PIN_PIN  BV(PIND1)

#else

#define I2C_SCL_DDR  DDRC
#define I2C_SCL_PORT PORTC
#define I2C_SCL_PIN  PINC

#define I2C_SCL_DDR_PIN  BV(DDC5)
#define I2C_SCL_PORT_PIN BV(PORTC5)
#define I2C_SCL_PIN_PIN  BV(PINC5)

#define I2C_SDA_DDR  DDRC
#define I2C_SDA_PORT PORTC
#define I2C_SDA_PIN  PINC

#define I2C_SDA_DDR_PIN  BV(DDC4)
#define I2C_SDA_PORT_PIN BV(PORTC4)
#define I2C_SDA_PIN_PIN  BV(PINC4)

#endif


/**
	Enum: I2C Module State/Result Codes
	
	NOTE: The error codes are those with names starting with "I2C_E_". These all have
	numeric values greater than or equal to *I2C_E_UNSPECIFIED*.
	
	I2C_DISABLED      - module is disabled
	I2C_READY         - module is idle and ready to begin a new operation
	I2C_ACTIVE        - module is currently performing an operation
	I2C_E_UNSPECIFIED - unspecified error
	I2C_E_LENGTH      - I2C length error (bytes to transmit and receive both zero)
	I2C_E_STATE       - attempted operation not allowed in current module state
	I2C_E_NOT_ACK     - received unexpected negative acknowledgement
	I2C_E_ARB_LOST    - lost I2C master arbitration
	I2C_E_FSM         - hardware generated unexpected status code
*/
enum {
	I2C_DISABLED      = 0,
	I2C_READY         = 1,
	I2C_ACTIVE        = 2,
	I2C_E_UNSPECIFIED = 3,
	I2C_E_LENGTH      = 4,
	I2C_E_STATE       = 5,
	I2C_E_NOT_ACK     = 6,
	I2C_E_ARB_LOST    = 7,
	I2C_E_FSM         = 8
};

/**
	Ref: i2c_slave_addr
	The type of I2C slave addresses.
*/
typedef uint8_t i2c_slave_addr;

/**
	Ref: i2c_state
	The type of state and result codes for the I2C module.
*/
typedef uint8_t i2c_state;

//extern volatile uint8_t last_twisc; // DEBUG: 

/**
	Variable: i2c_task_cats
	Tasks in the categories indicated by this <sched_catflags> value will be
	notified by the I2C module's ISR when a request operation finishes (either
	successfully or with an error).
*/
extern volatile sched_catflags i2c_task_cats;

/**
	Variable: i2c_request_state
	Contains a state code representing the current state of the I2C module.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern volatile i2c_state i2c_request_state;

/**
	Macro: I2C_IS_READY
	Evaluates to a true value if and only if the I2C module is initialized and
	ready to begin a new operation. Expression macro.
*/
#define I2C_IS_READY (i2c_request_state == I2C_READY)

/**
	Macro: I2C_IS_ACTIVE
	Evaluates to a true value if and only if the I2C module is busy with an
	ongoing operation. Expression macro.
*/
#define I2C_IS_ACTIVE (i2c_request_state == I2C_ACTIVE)

/**
	Function: i2chelper_mstr_init
	Configures the I2C module for master mode (the only one currently implemented).
	
	Parameters:
		twbr - A clock divisor value that is used to control the I2C bit rate.
		task_cats - A set of bit flags specifying task categories whose members
			should be notified when this module has finished an I2C transaction.
			Used to initialize <i2c_task_cats>.
*/
void i2chelper_mstr_init(uint8_t twbr, sched_catflags task_cats);

/**
	Function: i2chelper_request
	Initiates an I2C request operation consisting of a transmit phase followed
	by a receive phase. Either the transmit or the receive phase can be
	omitted, but not both.
	
	Completion of an I2C request initiated by this function can be
	detected by polling <i2c_request_state> (e.g. via the <I2C_IS_ACTIVE>
	macro) and by specifying task categories to be notified via <i2c_task_cats>.
	
	NOTE: The *bfr_out* and *bfr_in* arguments MAY point to the same buffer.
	This function ensures that all the bytes to transmit from *bfr_out* are
	transmitted before any received bytes are stored in *bfr_in*.
	
	Parameters:
		addr - The 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		n_out - Number of data bytes to transmit. If this argument is zero, the
			transmit phase will be omitted and the I2C module will transmit
			an SLA+R address byte immediately after transmitting a START condition.
		bfr_out - Pointer to the data bytes to transmit. If *n_out* is zero,
			this MAY be a null pointer.
		n_in - Number of data bytes to receive. If this argument is zero, the
			receive phase will be omitted and the I2C module will transmit
			a STOP condition immediately after receiving (positive or negative)
			acknowledgement of the last transmitted data byte.
		bfr_in - Pointer to memory to copy received data bytes into. If *n_in*
			is zero, this MAY be a null pointer.
	
	Returns:
		<I2C_ACTIVE> if and only if the request operation was successfully started,
		otherwise an error code.
*/
i2c_state i2chelper_request(
	i2c_slave_addr addr, uint8_t n_out, volatile const uint8_t *bfr_out,
	uint8_t n_in, volatile uint8_t *bfr_in);

/**
	Function: i2chelper_shutdown
	Shuts down the I2C module.
	
	CAUTION: Do not call this function while an I2C operation is ongoing.
*/
void i2chelper_shutdown(void);

#endif
