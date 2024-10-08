#ifndef AVR_MCP4X_H
#define AVR_MCP4X_H

#include <stdint.h>

/**
	File: mcp4x.h
	Control functions for the MCP4x series of digitally controlled potentiometers.
	Relies on the <spihelper.h> module.
	
	NOTE: These functions do NOT manipulate any SPI Slave Select pins. Application
	developers must ensure that the commands these functions transmit via the
	SPI peripheral reach the intended destination device.
	
	NOTE: By default, these functions will use the asynchronous API of the SPI
	helper module (i.e. <spihelper_request>), unless that API has been disabled.
	To use the synchronous API (i.e. <spihelper_exchange_bytes>) regardless of
	whether the asynchronous API is available, define the macro *MCP4X_SYNCHRONOUS*.
*/

#if defined(SPI_NO_ASYNC_API) && !defined(MCP4X_SYNCHRONOUS)
#define MCP4X_SYNCHRONOUS
#endif

/**
	Macro: MCP4X_P0
	Numeric identifier of the P0 potentiometer in an MCP4x device.
	Constant macro.
*/
#define MCP4X_P0 0

/**
	Macro: MCP4X_P1
	Numeric identifier of the P1 potentiometer in an MCP4x device.
	Constant macro.
*/
#define MCP4X_P1 1

/**
	Function: mcp4x_set_wiper
	Updates potentiometer wiper settings in the selected MCP4x device.
	
	Parameters:
		pot_bits - Identifier of the target potentiometer(s) in the destination device.
			Note that this is a bit mask, where the bit at position *n* is set in order
			to target potentiometer *n* (as identified by an *MCP4X_P[n]* constant).
		pos - The potentiometer wiper setting.
	
	Returns:
		A true value if and only if the operation was successfully initiated
		(in asynchronous mode) or completed (in synchronous mode).
*/
uint8_t mcp4x_set_wiper(uint8_t pot_bits, uint8_t pos);

/**
	Function: mcp4x_shutdown
	Disables potentiometers in the selected MCP4x device.
	
	Parameters:
		pot_bits - Identifier of the target potentiometer(s) in the destination device.
			Note that this is a bit mask, where the bit at position *n* is set in order
			to target potentiometer *n* (as identified by an *MCP4X_P[n]* constant).
	
	Returns:
		A true value if and only if the operation was successfully initiated
		(in asynchronous mode) or completed (in synchronous mode).
*/
uint8_t mcp4x_shutdown(uint8_t pot_bits);

#endif
