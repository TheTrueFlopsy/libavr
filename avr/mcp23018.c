
// IDEA: Set up a stderr that uses the TLV module.
//#include <assert.h>

#include <stddef.h>
#include <util/delay.h>

#include "mcp23018.h"

#define MCP23018_READ_BFR_SIZE 1
#define MCP23018_WRITE_BFR_SIZE 2

#define MCP23018_READ_DELAY 1.0
#define MCP23018_WRITE_DELAY 1.0

static volatile uint8_t mcp23018_read_bfr[MCP23018_READ_BFR_SIZE];
static volatile uint8_t mcp23018_write_bfr[MCP23018_WRITE_BFR_SIZE];

uint8_t mcp23018_is_valid_reg(mcp23018_reg r) {
	return
		(r & MCP23018_PAIR_MASK) <= MCP23018_MAX_PAIR &&
	  (r & MCP23018_BANK_MASK) <= MCP23018_MAX_BANK &&
	  (r & ~MCP23018_REG_MASK) == 0;
}

i2c_state mcp23018_begin_read(i2c_slave_addr addr, mcp23018_reg r) {
#ifndef NDEBUG
	if (!mcp23018_is_valid_reg(r))
		return I2C_E_UNSPECIFIED;
#endif
	
	mcp23018_write_bfr[0] = r;
	return i2chelper_request(addr, 1, mcp23018_write_bfr, 1, mcp23018_read_bfr);
}

i2c_state mcp23018_poll_read(uint8_t *v) {
	i2c_state	res = i2c_request_state;
	if (res == I2C_READY)
		*v = mcp23018_read_bfr[0];
	return res;
}

i2c_state mcp23018_read(i2c_slave_addr addr, mcp23018_reg r, uint8_t *v) {
	i2c_state res = mcp23018_begin_read(addr, r);
	if (res != I2C_ACTIVE)
		return res;
	
	while (1) {
		res = mcp23018_poll_read(v);
		if (res != I2C_ACTIVE)
			break;
		
		_delay_ms(MCP23018_READ_DELAY);
	}
	
	return res;
}

i2c_state mcp23018_begin_write(i2c_slave_addr addr, mcp23018_reg r, uint8_t v) {
#ifndef NDEBUG
	if (!mcp23018_is_valid_reg(r))
		return I2C_E_UNSPECIFIED;
#endif
	
	mcp23018_write_bfr[0] = r;
	mcp23018_write_bfr[1] = v;
	return i2chelper_request(addr, 2, mcp23018_write_bfr, 0, NULL);
}

i2c_state mcp23018_write(i2c_slave_addr addr, mcp23018_reg r, uint8_t v) {
	i2c_state res = mcp23018_begin_write(addr, r, v);
	if (res != I2C_ACTIVE)
		return res;
	
	while (1) {
		res = i2c_request_state;
		if (res != I2C_ACTIVE)
			break;
		
		_delay_ms(MCP23018_WRITE_DELAY);
	}
	
	return res;
}
