#ifndef AVR_MCP4X_H
#define AVR_MCP4X_H

#include <stdint.h>

#include "spihelper.h"

/**
	File: mcp4x.h
	Control functions for the MCP4x series of digitally controlled potentiometers.
	Relies on the <spihelper.h> SPI setup facility.
	
	NOTE: These functions do not control any SPI slave select pin. External
	      code must ensure that the commands these functions transmit via the
	      ATmega SPI module reach the intended destination device.
*/

/**
	Macro: MCP4X_P0
	Numeric identifier of the P0 potentiometer in an MCP4x device.
*/
#define MCP4X_P0 0

/**
	Macro: MCP4X_P1
	Numeric identifier of the P1 potentiometer in an MCP4x device.
*/
#define MCP4X_P1 1

/**
	Function: mcp4x_set_wiper
	Updates the potentiometer wiper setting of the selected MCP4x device.
	
	Parameters:
		pot - Identifier of the target potentiometer in the destination device.
		pos - The potentiometer wiper setting.
*/
void mcp4x_set_wiper(uint8_t pot, uint8_t pos);

/**
	Function: mcp4x_shutdown
	Disables a potentiometer in the selected MCP4x device.
	
	Parameters:
		pot - Identifier of the target potentiometer in the destination device.
*/
void mcp4x_shutdown(uint8_t pot);

#endif
