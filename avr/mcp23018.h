#ifndef AVR_MCP23018_H
#define AVR_MCP23018_H

/**
	File: mcp23018.h
	Helper module for controlling an MCP23018 I/O expander via I2C.
	Depends on the <i2chelper.h> module.
*/

#include <stdint.h>

#include "i2chelper.h"

#ifdef MCP23018_BANK_MODE
// --- MCP23018 register pair addresses (bits 0:3) ---
// NOTE: The IODIRx registers are set to all ones (i.e. all inputs) on startup/reset.
//       All other registers are zeroed on startup/reset.
enum {
	MCP23018_IODIR   = 0x00, // [RW] I/O direction, 0: output, 1: input
	MCP23018_IPOL    = 0x01, // [RW] input polarity, 0: same, 1: inverted
	MCP23018_GPINTEN = 0x02, // [RW] interrupt trigger enable, 0: disabled, 1: enabled
	MCP23018_DEFVAL  = 0x03, // [RW] reference value for level-triggered interrupts
	MCP23018_INTCON  = 0x04, // [RW] interrupt mode select, 0: edge-triggered, 1: level-triggered
	MCP23018_IOCON   = 0x05, // [RW] configuration register
	MCP23018_GPPU    = 0x06, // [RW] I/O pull-up enable, 0: disabled, 1: enabled
	MCP23018_INTF    = 0x07, // [R] interrupt flag register, 0: no interrupt, 1: interrupt pending
	MCP23018_INTCAP  = 0x08, // [R] interrupt capture register
	MCP23018_GPIO    = 0x09, // [RW] I/O port register
	MCP23018_OLAT    = 0x0a  // [RW] output latch register
};

#define MCP23018_PAIR_MASK 0b00001111

// --- MCP23018 bank addresses (bit 4) ---
enum {
	MCP23018_BANK_A = 0x00,
	MCP23018_BANK_B = 0x10
};

#define MCP23018_BANK_MASK 0b00010000

#else
// --- MCP23018 register pair addresses (bits 1:4) ---
enum {
	MCP23018_IODIR   = 0x00,
	MCP23018_IPOL    = 0x02,
	MCP23018_GPINTEN = 0x04,
	MCP23018_DEFVAL  = 0x06,
	MCP23018_INTCON  = 0x08,
	MCP23018_IOCON   = 0x0a,
	MCP23018_GPPU    = 0x0c,
	MCP23018_INTF    = 0x0e,
	MCP23018_INTCAP  = 0x10,
	MCP23018_GPIO    = 0x12,
	MCP23018_OLAT    = 0x14
};

#define MCP23018_PAIR_MASK 0b00011110

// --- MCP23018 bank addresses (bit 0) ---
enum {
	MCP23018_BANK_A = 0x00,
	MCP23018_BANK_B = 0x01
};

#define MCP23018_BANK_MASK 0b00000001

#endif

/**
	Macro: MCP23018_MAX_PAIR
	The largest valid register pair address.
*/
#define MCP23018_MAX_PAIR MCP23018_OLAT

/**
	Macro: MCP23018_MAX_BANK
	The largest valid register bank address.
*/
#define MCP23018_MAX_BANK MCP23018_BANK_B

/**
	Macro: MCP23018_REG_MASK
	Bit mask for the valid bits of an MCP23018 register address.
*/
#define MCP23018_REG_MASK (MCP23018_BANK_MASK | MCP23018_PAIR_MASK)

/**
	Enum: MCP23018 IOCON Bit Numbers
	An MCP23018 can be reconfigured by setting these bits in the IOCON register.
	
	MCP23018_INTCC  - interrupt clearing control, 0: clear on GPIO read, 1: clear on INTCAP read
	MCP23018_INTPOL - interrupt output polarity, 0: active-low, 1: active-high
	MCP23018_ODR    - open-drain interrupt output enable, 0: active driver, 1: open-drain
	MCP23018_SEQOP  - sequential operation disable, 0: address auto-incremented, 1: address retained
	MCP23018_MIRROR - mirrored interrupt output enable, 0: outputs separate, 1: outputs mirrored
	MCP23018_BANK   - register bank mode enable, 0: registers paired, 1: registers in banks
*/
enum {
	MCP23018_INTCC  = 0,
	MCP23018_INTPOL = 1,
	MCP23018_ODR    = 2,
	MCP23018_SEQOP  = 5,
	MCP23018_MIRROR = 6,
	MCP23018_BANK   = 7
};

/**
	Enum: MCP23018 Register Addresses
	These registers are used to configure and perform I/O with an MCP23018 via I2C.
	
	NOTE: The IODIRx registers are set to all ones (i.e. all inputs) on startup/reset.
	All other registers are zeroed on startup/reset.
	
	NOTE: Entries where the register name ends with an 'x' describe two registers,
	one for the 'A' port and one for the 'B' port, named correspondingly.
	
	MCP23018_IODIRx   - [RW] I/O direction, 0: output, 1: input
	MCP23018_IPOLx    - [RW] input polarity, 0: same, 1: inverted
	MCP23018_GPINTENx - [RW] interrupt trigger enable, 0: disabled, 1: enabled
	MCP23018_DEFVALx  - [RW] reference value for level-triggered interrupts
	MCP23018_INTCONx  - [RW] interrupt mode select, 0: edge-triggered, 1: level-triggered
	MCP23018_IOCON    - [RW] configuration register
	MCP23018_GPPUx    - [RW] I/O pull-up enable, 0: disabled, 1: enabled
	MCP23018_INTFx    - [R] interrupt flag register, 0: no interrupt, 1: interrupt pending
	MCP23018_INTCAPx  - [R] interrupt capture register
	MCP23018_GPIOx    - [RW] I/O port register
	MCP23018_OLATx    - [RW] output latch register
*/
enum {
	MCP23018_IODIRA   = MCP23018_IODIR | MCP23018_BANK_A,
	MCP23018_IODIRB   = MCP23018_IODIR | MCP23018_BANK_B,
	MCP23018_IPOLA    = MCP23018_IPOL | MCP23018_BANK_A,
	MCP23018_IPOLB    = MCP23018_IPOL | MCP23018_BANK_B,
	MCP23018_GPINTENA = MCP23018_GPINTEN | MCP23018_BANK_A,
	MCP23018_GPINTENB = MCP23018_GPINTEN | MCP23018_BANK_B,
	MCP23018_DEFVALA  = MCP23018_DEFVAL | MCP23018_BANK_A,
	MCP23018_DEFVALB  = MCP23018_DEFVAL | MCP23018_BANK_B,
	MCP23018_INTCONA  = MCP23018_INTCON | MCP23018_BANK_A,
	MCP23018_INTCONB  = MCP23018_INTCON | MCP23018_BANK_B,
	MCP23018_IOCONA   = MCP23018_IOCON | MCP23018_BANK_A,
	MCP23018_IOCONB   = MCP23018_IOCON | MCP23018_BANK_B,
	MCP23018_GPPUA    = MCP23018_GPPU | MCP23018_BANK_A,
	MCP23018_GPPUB    = MCP23018_GPPU | MCP23018_BANK_B,
	MCP23018_INTFA    = MCP23018_INTF | MCP23018_BANK_A,
	MCP23018_INTFB    = MCP23018_INTF | MCP23018_BANK_B,
	MCP23018_INTCAPA  = MCP23018_INTCAP | MCP23018_BANK_A,
	MCP23018_INTCAPB  = MCP23018_INTCAP | MCP23018_BANK_B,
	MCP23018_GPIOA    = MCP23018_GPIO | MCP23018_BANK_A,
	MCP23018_GPIOB    = MCP23018_GPIO | MCP23018_BANK_B,
	MCP23018_OLATA    = MCP23018_OLAT | MCP23018_BANK_A,
	MCP23018_OLATB    = MCP23018_OLAT | MCP23018_BANK_B,
	MCP23018_NOT_A_REG = 0xff
};

/**
	Ref: mcp23018_reg
	The type of MCP23018 register addresses.
	
	NOTE: Bits that are set to zero in <MCP23018_REG_MASK> are zero
	in all valid register addresses.
*/
typedef uint8_t mcp23018_reg;

/**
	Function: mcp23018_is_valid_reg
	Register address validator.
	
	Parameters:
		r - A register address.
	
	Returns:
		A true value iff the argument is a valid MCP23018 register address in the
		selected addressing mode (i.e. paired or banked).
*/
uint8_t mcp23018_is_valid_reg(mcp23018_reg r);

/**
	Function: mcp23018_begin_read
	Starts an asynchronous register read operation. This function delegates to
	<i2chelper_request> and completion of the register read can be detected as
	described in the documentation of that function.
	
	Parameters:
		addr - 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		r - Address of the register to read.
	
	Returns:
		<I2C_ACTIVE> iff the register read operation was successfully started,
		otherwise an error code.
*/
i2c_state mcp23018_begin_read(i2c_slave_addr addr, mcp23018_reg r);

/**
	Function: mcp23018_poll_read
	Tests whether there is an ongoing I2C operation. If there is no ongoing
	operation, this function will output the most recently read MCP23018
	register value.
	
	CAUTION: This function does not check whether any MCP23018 register read
	operation has actually been started.
	
	Parameters:
		v - Pointer to memory where the most recently read register value should
			be stored. This output parameter is updated only when *mcp23018_poll_read*
			returns <I2C_READY>.
	
	Returns:
		An <i2c_state> value representing the current state of the I2C module.
*/
i2c_state mcp23018_poll_read(uint8_t *v);

/**
	Function: mcp23018_read
	Performs a synchronous register read operation.
	
	Parameters:
		addr - 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		r - Address of the register to read.
		v - Pointer to memory where the result of the read register operation should
			be stored. This output parameter is updated only when *mcp23018_read*
			returns <I2C_READY>.
	
	Returns:
		<I2C_READY> iff the register read operation was successfully performed,
		otherwise an error code.
*/
i2c_state mcp23018_read(i2c_slave_addr addr, mcp23018_reg r, uint8_t *v);

/**
	Function: mcp23018_begin_write
	Starts an asynchronous register write operation. This function delegates to
	<i2chelper_request> and completion of the register write can be detected as
	described in the documentation of that function.
	
	Parameters:
		addr - 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		r - Address of the register to write to.
		v - Value to write to the target register.
	
	Returns:
		<I2C_ACTIVE> iff the register read operation was successfully started,
		otherwise an error code.
*/
i2c_state mcp23018_begin_write(i2c_slave_addr addr, mcp23018_reg r, uint8_t v);

/**
	Function: mcp23018_write
	Performs a synchronous register write operation.
	
	Parameters:
		addr - 7-bit I2C slave address of the target device. The address MUST
			be stored in the 7 least significant bits of the argument and
			MUST NOT include an R/W control bit.
		r - Address of the register to write to.
		v - Value to write to the target register.
	
	Returns:
		<I2C_READY> iff the register write operation was successfully performed,
		otherwise an error code.
*/
i2c_state mcp23018_write(i2c_slave_addr addr, mcp23018_reg r, uint8_t v);

#endif
