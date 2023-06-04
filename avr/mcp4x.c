
#include <stddef.h>

#include "mcp4x.h"

#ifndef BV
#define BV(N) (1 << (N))
#endif

#define MCP4X_C0 4
#define MCP4X_C1 5

// NOTE: The MCP4X wants its data MSB first, but that is the default
//       on the ATmega, too.

uint8_t mcp4x_set_wiper(uint8_t pot_bits, uint8_t pos) {
	uint8_t cmd = ((BV(MCP4X_P0) | BV(MCP4X_P1)) & pot_bits) | BV(MCP4X_C0);

#if defined(MCP4X_SYNCHRONOUS) || defined(SPI_NO_ASYNC_API)
	spihelper_exchange_bytes(cmd);
	spihelper_exchange_bytes(pos);
	return 1;
#else
	uint8_t cmd_buf[2] = { cmd, pos };
	spi_state res = spihelper_request(2, cmd_buf, 0, 0, NULL);
	return res == SPI_ACTIVE;
#endif
}

uint8_t mcp4x_shutdown(uint8_t pot_bits) {
	uint8_t cmd = ((BV(MCP4X_P0) | BV(MCP4X_P1)) & pot_bits) | BV(MCP4X_C1);
	
#if defined(MCP4X_SYNCHRONOUS) || defined(SPI_NO_ASYNC_API)
	spihelper_exchange_bytes(cmd);
	spihelper_exchange_bytes(0);
	return 1;
#else
	uint8_t cmd_buf[2] = { cmd, 0 };
	spi_state res = spihelper_request(2, cmd_buf, 0, 0, NULL);
	return res == SPI_ACTIVE;
#endif
}
