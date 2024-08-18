#ifndef AVR_NRF24X_H
#define AVR_NRF24X_H

#include <stdint.h>

#include "bitops.h"

/**
	File: nrf24x.h
	Constant definitions and helper functions for the SPI command interface of
	the nRF24x series of 2.4 GHz digital transceivers. Relies on the <spihelper.h>
	module.
	
	NOTE: The functions provided by this module do NOT manipulate any SPI Slave Select
	pins. Application developers must ensure that the commands these functions transmit
	via the SPI peripheral reach the intended destination device.
	
	NOTE: This module does not deal with the CE and IRQ pins of the nRF24x IC.
	Those pins must be managed by application code.
	
	NOTE: By default, this module will use the asynchronous API of the SPI
	helper module (i.e. <spihelper_request>), unless that API has been disabled.
	To use the synchronous API (i.e. <spihelper_exchange_bytes>) regardless of
	whether the asynchronous API is available, define the macro *NRF24X_SYNCHRONOUS*.
	
	NOTE: By default, this module uses an internal command buffer when operating in
	asynchronous mode (i.e. when using the asynchronous API of <spihelper.h>).
	This buffer can be disabled and left undeclared (saving *NRF24X_BFR_SIZE* bytes
	of RAM) by defining the macro *NRF24X_NO_CMD_BFR*. However, doing this will
	also disable most of the helper functions.
*/

#if defined(SPI_NO_ASYNC_API) && !defined(NRF24X_SYNCHRONOUS)
#define NRF24X_SYNCHRONOUS
#endif

#ifndef NRF24X_SYNCHRONOUS
#include "spihelper.h"
#endif

/// Section: Configuration Macros

/**
	Macro: NRF24X_BFR_SIZE
	Size in bytes of the buffer for asynchronous SPI operations. Configuration macro.
	
	Default value: 33
*/
#ifndef NRF24X_BFR_SIZE
#define NRF24X_BFR_SIZE 33
#endif


/// Section: SPI Interface Constants and Helper Macros

// TODO: Verify that the NRF24X_W_TX_PAYLOAD_NOACK code is correct.
/**
	Enum: nRF24x Command Words
	See the IC product specification for information about the protocol
	of the SPI interface.
	
	NRF24X_R_REGISTER         - R_REGISTER command word (register address zero)
	NRF24X_W_REGISTER         - W_REGISTER command word (register address zero)
	NRF24X_R_RX_PAYLOAD       - R_RX_PAYLOAD command word
	NRF24X_W_TX_PAYLOAD       - W_TX_PAYLOAD command word
	NRF24X_FLUSH_TX           - FLUSH_TX command word
	NRF24X_FLUSH_RX           - FLUSH_RX command word
	NRF24X_REUSE_TX_PL        - REUSE_TX_PL command word
	NRF24X_ACTIVATE           - ACTIVATE command word (not used by nRF24L01+)
	NRF24X_ACTIVATE_2         - ACTIVATE command argument (not used by nRF24L01+)
	NRF24X_R_RX_PL_WID        - R_RX_PL_WID command word
	NRF24X_W_ACK_PAYLOAD      - W_ACK_PAYLOAD command word (data pipe number zero)
	NRF24X_W_TX_PAYLOAD_NOACK - W_TX_PAYLOAD_NOACK command word
	NRF24X_NOP                - NOP command word
*/
enum {
	NRF24X_R_REGISTER         = 0b00000000,  // 0x00 (plus 5-bit register address)
	NRF24X_W_REGISTER         = 0b00100000,  // 0x20 (plus 5-bit register address)
	NRF24X_R_RX_PAYLOAD       = 0b01100001,  // 0x61
	NRF24X_W_TX_PAYLOAD       = 0b10100000,  // 0xa0
	NRF24X_FLUSH_TX           = 0b11100001,  // 0xe1
	NRF24X_FLUSH_RX           = 0b11100010,  // 0xe2
	NRF24X_REUSE_TX_PL        = 0b11100011,  // 0xe3
	NRF24X_ACTIVATE           = 0b01010000,  // 0x50
	NRF24X_ACTIVATE_2         = 0b01110011,  // 0x73
	NRF24X_R_RX_PL_WID        = 0b01100000,  // 0x60
	NRF24X_W_ACK_PAYLOAD      = 0b10101000,  // 0xa8 (plus 3-bit data pipe number)
	NRF24X_W_TX_PAYLOAD_NOACK = 0b10110000,  // 0xb0
	NRF24X_NOP                = 0b11111111   // 0xff
};

#define NRF24X_REG_ADDR_SIZE 5

#define NRF24X_REG_ADDR(A) GET_BITFIELD(NRF24X_REG_ADDR_SIZE, A)
#define NRF24X_IS_R_REG_CMD(C) \
	(CLEAR_BITFIELD_AT(NRF24X_REG_ADDR_SIZE, 0, C) == NRF24X_R_REGISTER)
#define NRF24X_IS_W_REG_CMD(C) \
	(CLEAR_BITFIELD_AT(NRF24X_REG_ADDR_SIZE, 0, C) == NRF24X_W_REGISTER)

/**
	Macro: NRF24X_R_REG_CMD
	Constructs an R_REGISTER command word with a specified nRF24x register address.
	Function-like macro.
	
	Parameters:
		A - 5-bit register address
	
	Returns:
		An R_REGISTER command word containing the specified register address.
*/
#define NRF24X_R_REG_CMD(A) ((uint8_t)(NRF24X_R_REGISTER | NRF24X_REG_ADDR(A)))

/**
	Macro: NRF24X_W_REG_CMD
	Constructs a W_REGISTER command word with a specified nRF24x register address.
	Function-like macro.
	
	Parameters:
		A - 5-bit register address
	
	Returns:
		A W_REGISTER command word containing the specified register address.
*/
#define NRF24X_W_REG_CMD(A) ((uint8_t)(NRF24X_W_REGISTER | NRF24X_REG_ADDR(A)))

#define NRF24X_PIPE_NUM_SIZE 3

#define NRF24X_PIPE_NUM(P) GET_BITFIELD(NRF24X_PIPE_NUM_SIZE, P)
#define NRF24X_IS_W_ACK_PAYLOAD_CMD(C) \
	(CLEAR_BITFIELD_AT(NRF24X_PIPE_NUM_SIZE, 0, C) == NRF24X_W_ACK_PAYLOAD)

/**
	Macro: NRF24X_W_ACK_PAYLOAD_CMD
	Constructs a W_ACK_PAYLOAD command word with a specified nRF24x data pipe number.
	Function-like macro.
	
	Parameters:
		P - 3-bit data pipe number
	
	Returns:
		A W_ACK_PAYLOAD command word containing the specified data pipe number.
*/
#define NRF24X_W_ACK_PAYLOAD_CMD(P) ((uint8_t)(NRF24X_W_ACK_PAYLOAD | NRF24X_PIPE_NUM(P)))

/**
	Enum: nRF24x Register Addresses
	See the IC product specification for information about register semantics.
	
	NRF24X_CONFIG      - CONFIG register address
	NRF24X_EN_AA       - EN_AA register address
	NRF24X_EN_RXADDR   - EN_RXADDR register address
	NRF24X_SETUP_AW    - SETUP_AW register address
	NRF24X_SETUP_RETR  - SETUP_RETR register address
	NRF24X_RF_CH       - RF_CH register address
	NRF24X_RF_SETUP    - RF_SETUP register address
	NRF24X_STATUS      - STATUS register address
	NRF24X_OBSERVE_TX  - OBSERVE_TX register address
	NRF24X_CD          - CD register address (old name used for nRF24L01)
	NRF24X_RPD         - RPD register address (new name used for nRF24L01+)
	NRF24X_RX_ADDR_P0  - RX_ADDR_P0 register address
	NRF24X_RX_ADDR_P1  - RX_ADDR_P1 register address
	NRF24X_RX_ADDR_P2  - RX_ADDR_P2 register address
	NRF24X_RX_ADDR_P3  - RX_ADDR_P3 register address
	NRF24X_RX_ADDR_P4  - RX_ADDR_P4 register address
	NRF24X_RX_ADDR_P5  - RX_ADDR_P5 register address
	NRF24X_TX_ADDR     - TX_ADDR register address
	NRF24X_RX_PW_P0    - RX_PW_P0 register address
	NRF24X_RX_PW_P1    - RX_PW_P1 register address
	NRF24X_RX_PW_P2    - RX_PW_P2 register address
	NRF24X_RX_PW_P3    - RX_PW_P3 register address
	NRF24X_RX_PW_P4    - RX_PW_P4 register address
	NRF24X_RX_PW_P5    - RX_PW_P5 register address
	NRF24X_FIFO_STATUS - FIFO_STATUS register address
	NRF24X_DYNPD       - DYNPD register address
	NRF24X_FEATURE     - FEATURE register address
*/
enum {
	NRF24X_CONFIG      = 0x00,
	NRF24X_EN_AA       = 0x01,
	NRF24X_EN_RXADDR   = 0x02,
	NRF24X_SETUP_AW    = 0x03,
	NRF24X_SETUP_RETR  = 0x04,
	NRF24X_RF_CH       = 0x05,
	NRF24X_RF_SETUP    = 0x06,
	NRF24X_STATUS      = 0x07,
	NRF24X_OBSERVE_TX  = 0x08,
	NRF24X_CD          = 0x09,
	NRF24X_RPD         = 0x09,
	NRF24X_RX_ADDR_P0  = 0x0a,  // 5 bytes
	NRF24X_RX_ADDR_P1  = 0x0b,  // 5 bytes
	NRF24X_RX_ADDR_P2  = 0x0c,
	NRF24X_RX_ADDR_P3  = 0x0d,
	NRF24X_RX_ADDR_P4  = 0x0e,
	NRF24X_RX_ADDR_P5  = 0x0f,
	NRF24X_TX_ADDR     = 0x10,  // 5 bytes
	NRF24X_RX_PW_P0    = 0x11,
	NRF24X_RX_PW_P1    = 0x12,
	NRF24X_RX_PW_P2    = 0x13,
	NRF24X_RX_PW_P3    = 0x14,
	NRF24X_RX_PW_P4    = 0x15,
	NRF24X_RX_PW_P5    = 0x16,
	NRF24X_FIFO_STATUS = 0x17,
	NRF24X_DYNPD       = 0x1c,
	NRF24X_FEATURE     = 0x1d
};

/**
	Enum: nRF24x Data Pipe Numbers
	See the IC product specification for information about data pipes.
	
	NRF24X_P0 - data pipe number 0
	NRF24X_P1 - data pipe number 1
	NRF24X_P2 - data pipe number 2
	NRF24X_P3 - data pipe number 3
	NRF24X_P4 - data pipe number 4
	NRF24X_P5 - data pipe number 5
*/
enum {
	NRF24X_P0 = 0,
	NRF24X_P1 = 1,
	NRF24X_P2 = 2,
	NRF24X_P3 = 3,
	NRF24X_P4 = 4,
	NRF24X_P5 = 5
};

/**
	Enum: nRF24x CONFIG Register Bits
	See the IC product specification for information about the CONFIG register.
	
	NRF24X_MASK_RX_DR  - MASK_RX_DR bit number
	NRF24X_MASK_TX_DS  - MASK_TX_DS bit number
	NRF24X_MASK_MAX_RT - MASK_MAX_RT bit number
	NRF24X_EN_CRC      - EN_CRC bit number
	NRF24X_CRCO        - CRCO bit number
	NRF24X_PWR_UP      - PWR_UP bit number
	NRF24X_PRIM_RX     - PRIM_RX bit number
*/
enum {
	NRF24X_MASK_RX_DR  = 6,
	NRF24X_MASK_TX_DS  = 5,
	NRF24X_MASK_MAX_RT = 4,
	NRF24X_EN_CRC      = 3,
	NRF24X_CRCO        = 2,
	NRF24X_PWR_UP      = 1,
	NRF24X_PRIM_RX     = 0
};

/**
	Enum: nRF24x SETUP_AW Register Values
	See the IC product specification for information about the SETUP_AW register.
	
	NRF24X_AW_3 - AW register value specifying 3-byte RX/TX addresses
	NRF24X_AW_4 - AW register value specifying 4-byte RX/TX addresses
	NRF24X_AW_5 - AW register value specifying 5-byte RX/TX addresses
*/
enum {
	NRF24X_AW_3  = 0x01,
	NRF24X_AW_4  = 0x02,
	NRF24X_AW_5  = 0x03
};

#define NRF24X_ARD_SIZE 4
#define NRF24X_ARD_OFFSET 4
#define NRF24X_ARC_SIZE 4

/**
	Macro: NRF24X_SETUP_RETR_VAL
	Constructs a SETUP_RETR register value from ARD and ARC bitfield values.
	Function-like macro.
	
	Parameters:
		ARD - 4-bit ARD bitfield value
		ARC - 4-bit ARC bitfield value
	
	Returns:
		A SETUP_RETR register value containing the specified ARD and ARC values.
*/
#define NRF24X_SETUP_RETR_VAL(ARD, ARC) \
	(MAKE_BITFIELD_AT(NRF24X_ARD_SIZE, NRF24X_ARD_OFFSET, ARD) | \
	 GET_BITFIELD(NRF24X_ARC_SIZE, ARC))

#define NRF24X_GET_ARD(SR) GET_BITFIELD_AT(NRF24X_ARD_SIZE, NRF24X_ARD_OFFSET, SR)

#define NRF24X_GET_ARC(SR) GET_BITFIELD(NRF24X_ARC_SIZE, SR)

#define NRF24X_RF_CH_SIZE 7

/**
	Macro: NRF24X_RF_CH_VAL
	Constructs an RF_CH register value from an RF_CH bitfield value.
	Function-like macro.
	
	Parameters:
		CH - 7-bit RF_CH bitfield value
	
	Returns:
		An RF_CH register value containing the specified bitfield value.
*/
#define NRF24X_RF_CH_VAL(CH) GET_BITFIELD(NRF24X_RF_CH_SIZE, CH)

/**
	Enum: nRF24x RF_SETUP Register Bits
	See the IC product specification for information about the RF_SETUP register.
	
	NRF24X_PLL_LOCK  - PLL_LOCK bit number
	NRF24X_RF_DR     - RF_DR bit number
	NRF24X_RF_PWR1   - RF_PWR bit number 1
	NRF24X_RF_PWR0   - RF_PWR bit number 0
	NRF24X_LNA_HCURR - LNA_HCURR bit number
*/
enum {
	NRF24X_PLL_LOCK  = 4,
	NRF24X_RF_DR     = 3,
	NRF24X_RF_PWR1   = 2,
	NRF24X_RF_PWR0   = 1,
	NRF24X_LNA_HCURR = 0
};

/**
	Enum: nRF24x STATUS Register Bits
	See the IC product specification for information about the STATUS register.
	
	NRF24X_RX_DR      - RX_DR bit number
	NRF24X_TX_DS      - TX_DS bit number
	NRF24X_MAX_RT     - MAX_RT bit number
	NRF24X_RX_P_NO2   - RX_P_NO bit number 2
	NRF24X_RX_P_NO1   - RX_P_NO bit number 1
	NRF24X_RX_P_NO0   - RX_P_NO bit number 0
	NRF24X_ST_TX_FULL - ST_TX_FULL bit number
*/
enum {
	NRF24X_RX_DR      = 6,
	NRF24X_TX_DS      = 5,
	NRF24X_MAX_RT     = 4,
	NRF24X_RX_P_NO2   = 3,
	NRF24X_RX_P_NO1   = 2,
	NRF24X_RX_P_NO0   = 1,
	NRF24X_ST_TX_FULL = 0
};

#define NRF24X_RX_P_NO_OFFSET 1

/**
	Macro: NRF24X_GET_RX_P_NO
	Extracts the RX_P_NO bitfield value from a STATUS register value.
	Function-like macro.
	
	Parameters:
		ST - STATUS register value
	
	Returns:
		The 3-bit RX_P_NO bitfield value in the specified register value.
*/
#define NRF24X_GET_RX_P_NO(ST) \
	GET_BITFIELD_AT(NRF24X_PIPE_NUM_SIZE, NRF24X_RX_P_NO_OFFSET, ST)

#define NRF24X_PLOS_CNT_SIZE 4
#define NRF24X_PLOS_CNT_OFFSET 4

/**
	Macro: NRF24X_GET_PLOS_CNT
	Extracts the PLOS_CNT bitfield value from an OBSERVE_TX register value.
	Function-like macro.
	
	Parameters:
		OTX - OBSERVE_TX register value
	
	Returns:
		The 4-bit PLOS_CNT bitfield value in the specified register value.
*/
#define NRF24X_GET_PLOS_CNT(OTX) \
	GET_BITFIELD_AT(NRF24X_PLOS_CNT_SIZE, NRF24X_PLOS_CNT_OFFSET, OTX)

#define NRF24X_ARC_CNT_SIZE 4

/**
	Macro: NRF24X_GET_ARC_CNT
	Extracts the ARC_CNT bitfield value from an OBSERVE_TX register value.
	Function-like macro.
	
	Parameters:
		OTX - OBSERVE_TX register value
	
	Returns:
		The 4-bit ARC_CNT bitfield value in the specified register value.
*/
#define NRF24X_GET_ARC_CNT(OTX) GET_BITFIELD(NRF24X_ARC_CNT_SIZE, OTX)

/**
	Enum: nRF24x CD/RPD Register Bits
	See the IC product specification for information about the CD/RPD register.
	
	NRF24X_CD0  - CD bit number 0
	NRF24X_RPD0 - RPD bit number 0
*/
enum {
	NRF24X_CD0  = 0,
	NRF24X_RPD0 = 0
};

#define NRF24X_RX_PW_SIZE 6

#define NRF24X_RX_PW_VAL(PW) GET_BITFIELD(NRF24X_RX_PW_SIZE, PW)

/**
	Enum: nRF24x FIFO_STATUS Register Bits
	See the IC product specification for information about the FIFO_STATUS register.
	
	NRF24X_TX_REUSE - TX_REUSE bit number
	NRF24X_TX_FULL  - TX_FULL bit number
	NRF24X_TX_EMPTY - TX_EMPTY bit number
	NRF24X_RX_FULL  - RX_FULL bit number
	NRF24X_RX_EMPTY - RX_EMPTY bit number
*/
enum {
	NRF24X_TX_REUSE = 6,
	NRF24X_TX_FULL  = 5,
	NRF24X_TX_EMPTY = 4,
	NRF24X_RX_FULL  = 1,
	NRF24X_RX_EMPTY = 0
};

/**
	Enum: nRF24x FEATURE Register Bits
	See the IC product specification for information about the FEATURE register.
	
	NRF24X_EN_DPL     - EN_DPL bit number
	NRF24X_EN_ACK_PAY - EN_ACK_PAY bit number
	NRF24X_EN_DYN_ACK - EN_DYN_ACK bit number
*/
enum {
	NRF24X_EN_DPL     = 2,
	NRF24X_EN_ACK_PAY = 1,
	NRF24X_EN_DYN_ACK = 0
};


/// Section: Command API Variables

/**
	Variable: nrf24x_status
	Storage location for STATUS values. The nRF24x IC returns the value of its
	STATUS register when it receives a command word via SPI. Some of the API
	macros and functions in this module will store the returned STATUS value
	here when they receive it.
*/
extern volatile uint8_t nrf24x_status;

#ifndef NRF24X_SYNCHRONOUS

/**
	Variable: nrf24x_command
	When the <NRF24X_CMD_IN> API macro is used, the command word of the input
	command to send MUST be stored here before the macro is invoked. That is
	generally the only reason to touch this variable application code will have.
	
	NOTE: Not defined in synchronous mode.
*/
extern volatile uint8_t nrf24x_command;

#ifndef NRF24X_NO_CMD_BFR
extern uint8_t nrf24x_pending_n_in;
#endif


/// Section: Command API Macros and Functions

/**
	Macro: NRF24X_CMD_OUT
	Initiates an output command to the nRF24x via an asynchronous SPI request.
	Function-like macro.
	
	NOTE: Not defined in synchronous mode.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>.
	
	Parameters:
		N - Number of output bytes to transmit, including the command word.
		B - Pointer to the output bytes to transmit, including the command word.
	
	Returns:
		<SPI_ACTIVE> if and only if the SPI request was successfully started,
		otherwise an error code.
*/
#define NRF24X_CMD_OUT(N, B) spihelper_request((N), (B), 1, 0, &nrf24x_status)

/**
	Macro: NRF24X_CMD_IN
	Initiates an input command to the nRF24x via an asynchronous SPI request.
	Function-like macro.
	
	NOTE: Not defined in synchronous mode.
	
	CAUTION: The command word of the input command to send to the nRF24x MUST
	be stored in <nrf24x_command> before this macro is invoked.
	
	Parameters:
		N - Number of input bytes to receive.
		O - Number of received bytes to discard before getting the input bytes.
			Note that the sequence of received bytes includes the STATUS register value
			that the nRF24x returns when the command word is transmitted.
		B - Pointer to memory to copy the received input bytes into.
	
	Returns:
		<SPI_ACTIVE> if and only if the SPI request was successfully started,
		otherwise an error code.
*/
#define NRF24X_CMD_IN(N, O, B) spihelper_request(1, &nrf24x_command, (N), (O), (B))

#endif

/**
	Function: nrf24x_out_0
	Sends a 0-byte output command (one that consists only of a command word)
	to the nRF24x.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>.
	
	Parameters:
		cmd - Command word of the output command.
	
	Returns:
		A true value if and only if the SPI request was successfully performed
		(or started, in asynchronous mode).
*/
uint8_t nrf24x_out_0(uint8_t cmd);

#if !defined(NRF24X_NO_CMD_BFR) || defined(NRF24X_SYNCHRONOUS)

/**
	Function: nrf24x_out_1
	Sends a 1-byte output command (one that consists of a command word and
	a single output data byte) to the nRF24x.
	
	NOTE: Not defined when *NRF24X_NO_CMD_BFR* is defined in asynchronous mode.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>
	when called in synchronous mode.
	
	Parameters:
		cmd - Command word of the output command.
		data - Data byte of the output command.
	
	Returns:
		A true value if and only if the SPI request was successfully performed
		(or started, in asynchronous mode).
*/
uint8_t nrf24x_out_1(uint8_t cmd, uint8_t data);

/**
	Function: nrf24x_out_n
	Sends an N-byte output command (one that consists of a command word and
	a specified number of output data bytes) to the nRF24x.
	
	NOTE: Not defined when *NRF24X_NO_CMD_BFR* is defined in asynchronous mode.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>
	when called in synchronous mode.
	
	Parameters:
		cmd - Command word of the output command.
		n_out - Number of data bytes to transmit.
		bfr_out - Pointer to the data bytes to transmit.
	
	Returns:
		A true value if and only if the SPI request was successfully performed
		(or started, in asynchronous mode).
*/
uint8_t nrf24x_out_n(uint8_t cmd, uint8_t n_out, uint8_t *bfr_out);

/**
	Function: nrf24x_in_1
	Sends a 1-byte input command (one that receives a single input data byte
	in response) to the nRF24x.
	
	NOTE: Not defined when *NRF24X_NO_CMD_BFR* is defined in asynchronous mode.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>
	when called in synchronous mode.
	
	Parameters:
		cmd - Command word of the input command.
		data_p - Pointer to location to copy the received data byte into.
			Note that this argument is ignored in asynchronous mode, where the input
			buffer pointer is given as an argument to <nrf24x_in_finish>.
	
	Returns:
		A true value if and only if the SPI request was successfully performed
		(or started, in asynchronous mode).
*/
uint8_t nrf24x_in_1(uint8_t cmd, uint8_t *data_p);

/**
	Function: nrf24x_in_n
	Sends an N-byte input command (one that receives a specified number
	of input data bytes in response) to the nRF24x.
	
	NOTE: Not defined when *NRF24X_NO_CMD_BFR* is defined in asynchronous mode.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>
	when called in synchronous mode.
	
	Parameters:
		cmd - Command word of the input command.
		n_in - Number of data bytes to receive.
		bfr_in - Pointer to memory to copy received data bytes into.
			Note that this argument is ignored in asynchronous mode, where the input
			buffer pointer is given as an argument to <nrf24x_in_finish>.
	
	Returns:
		A true value if and only if the SPI request was successfully performed
		(or started, in asynchronous mode).
*/
uint8_t nrf24x_in_n(uint8_t cmd, uint8_t n_in, uint8_t *bfr_in);

#endif

#if !defined(NRF24X_NO_CMD_BFR) && !defined(NRF24X_SYNCHRONOUS)
/**
	Macro: NRF24X_NONE_PENDING
	Special value returned by <nrf24x_in_finish> when no asynchronous command is pending.
	
	CAUTION: This is equal to 255/0xff, so that value MUST NOT be used as an actual data
	byte count. There will generally not be any need for byte counts that large, anyway.
	
	Constant macro.
*/
#define NRF24X_NONE_PENDING 0xff

// NOTE: Last-minute addition. Not sure it's a good idea. Leaving it undocumented.
uint8_t nrf24x_try_finish(uint8_t *n_in, uint8_t *bfr_in);

/**
	Function: nrf24x_in_finish
	Used to finish an asynchronous input command sent to the nRF24x and fetch
	the received input data.
	
	NOTE: Not defined if the module is in synchronous mode or *NRF24X_NO_CMD_BFR* is defined.
	
	NOTE: Writes the STATUS register value returned by the nRF24x to <nrf24x_status>.
	
	Parameters:
		bfr_in - Pointer to memory to copy received data bytes into.
	
	Returns:
		The number of data bytes received, or <NRF24X_NONE_PENDING> if no asynchronous
		command is pending.
*/
uint8_t nrf24x_in_finish(uint8_t *bfr_in);
#endif

#endif
