
#include <avr/io.h>

#include "mcp4x.h"

#define BV(N) (1 << (N))

#define MCP4X_C0 4
#define MCP4X_C1 5

// TODO: Implement interrupt-driven SPI communication.

// NOTE: The MCP4X wants its data MSB first, but that is the default
//       on the ATmega, too.

static void send_byte_busy_loop(uint8_t data) {
	SPDR = data;
	while (!(SPSR & BV(SPIF)))
		/* do nothing */;
}

void mcp4x_set_wiper(uint8_t pot, uint8_t pos) {
	uint8_t cmd = ((BV(MCP4X_P0) | BV(MCP4X_P1)) & pot) | BV(MCP4X_C0);
	send_byte_busy_loop(cmd);
	send_byte_busy_loop(pos);
}

void mcp4x_shutdown(uint8_t pot) {
	uint8_t cmd = ((BV(MCP4X_P0) | BV(MCP4X_P1)) & pot) | BV(MCP4X_C1);
	send_byte_busy_loop(cmd);
	send_byte_busy_loop(0);
}
