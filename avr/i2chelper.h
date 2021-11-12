#ifndef AVR_I2CHELPER_H
#define AVR_I2CHELPER_H

/**
	File: i2chelper.h
	Asynchronous, interrupt-driven I2C (aka TWI) helper module.
*/

#include <stdint.h>

#include "task_sched.h"

/**
	Enum: I2C Module State/Result Codes
	
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
	The type of state codes for the I2C module.
*/
typedef uint8_t i2c_state;

//extern volatile uint8_t last_twisc; // DEBUG: 

/**
	Variable: i2c_task_cats
	Contains a set of task category bit flags. Tasks in the indicated categories will
	be awakened by the i2chelper ISR when an I2C operation finishes (either successfully
	or with an error).
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
	Evaluates to a true value iff the I2C module is initialized and
	ready to begin a new operation.
*/
#define I2C_IS_READY (i2c_request_state == I2C_READY)

/**
	Macro: I2C_IS_ACTIVE
	Evaluates to a true value iff the I2C module is busy with an
	ongoing operation.
*/
#define I2C_IS_ACTIVE (i2c_request_state == I2C_ACTIVE)

/**
	Function: i2chelper_mstr_init
	Configures the I2C module for Master mode.
	
	Parameters:
		twbr - A clock divisor value that is used to control the I2C bit rate.
		task_cats - A set of bit flags specifying task categories whose
			members should be awakened in response to certain events in the
			I2C module. See <i2c_task_cats> for details.
*/
void i2chelper_mstr_init(uint8_t twbr, sched_catflags task_cats);

/**
	Function: i2chelper_request
	Initiates an I2C request operation consisting of a transmit phase followed
	by a receive phase. Either the transmit or the receive phase can be
	omitted, but not both.
	
	Completion of an I2C request initiated by this function can be
	detected either by polling <i2c_request_state> (e.g. via the <I2C_IS_ACTIVE>
	macro) or by specifying task categories to be awakened via <i2c_task_cats>.
	
	Parameters:
		addr - 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		n_out - Number of bytes to transmit. If this argument is zero, the
			transmit phase will be omitted and the I2C module will transmit
			a SLA+R address byte immediately after transmitting a START condition.
		bfr_out - Pointer to the data bytes to transmit. If *n_out* is zero,
			this MAY be a null pointer.
		n_in - Number of bytes to receive. If this argument is zero, the
			receive phase will be omitted and the I2C module will transmit
			a STOP condition immediately after receiving (positive or negative)
			acknowledgement of the last transmitted data byte.
		bfr_in - Pointer to memory to copy received data bytes into. If *n_in*
			is zero, this MAY be a null pointer.
	
	Returns:
		<I2C_ACTIVE> iff the request operation was successfully started,
		otherwise an error code.
*/
i2c_state i2chelper_request(
	i2c_slave_addr addr, uint8_t n_out, volatile const uint8_t *bfr_out,
	uint8_t n_in, volatile uint8_t *bfr_in);

// ISSUE: Is there a delay between the ATmega detecting an interrupt
// condition and interruption of the currently executing code? Does
// this mean that a jump to an ISR may happen after the corresponding
// interrupt enable bit has been cleared, if the interrupt condition
// is detected a few clock cycles before the execution of
// the instruction that clears the enable bit? At which stage of the
// interrupt triggering process is the enable bit checked? I haven't
// been able to find any definitive statement about this in the ATmega
// datasheet.
/**
	Function: i2chelper_shutdown
	Shuts down the I2C module.
	
	CAUTION: Do not call this function while an I2C operation is ongoing.
*/
void i2chelper_shutdown(void);

#endif
