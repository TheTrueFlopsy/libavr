#ifndef AVR_NRF24X_H
#define AVR_NRF24X_H

#include <stdint.h>

#include "bitops.h"

/**
	File: nrf24x.h
	Constant definitions and helper functions for the SPI command interface of
	the nRF24x series of 2.4 GHz digital transceivers. Relies on the <spihelper.h>
	module.
	
	NOTE: These functions do NOT manipulate any SPI Slave Select pins. Application
	developers must ensure that the commands these functions transmit via the
	SPI peripheral reach the intended destination device.
	
	NOTE: This module does not deal with the CE and IRQ pins of the nRF24x IC.
	Those pins must be managed by application code.
	
	NOTE: By default, these functions will use the asynchronous API of the SPI
	helper module (i.e. <spihelper_request>), unless that API has been disabled.
	To use the synchronous API (i.e. <spihelper_exchange_bytes>) regardless of
	whether the asynchronous API is available, define the macro *NRF24X_SYNCHRONOUS*.
*/

#if defined(SPI_NO_ASYNC_API) && !defined(NRF24X_SYNCHRONOUS)
#define NRF24X_SYNCHRONOUS
#endif

#ifndef NRF24X_SYNCHRONOUS
#include "spihelper.h"
#endif

/**
	Macro: NRF24X_BFR_SIZE
	Size in bytes of the buffer for asynchronous SPI operations. Configuration macro.
	
	Default value: 33
*/
#ifndef NRF24X_BFR_SIZE
#define NRF24X_BFR_SIZE 33
#endif

#define NRF24X_NONE_PENDING 0xff

// TODO: Verify that the NRF24X_W_TX_PAYLOAD_NOACK code is correct.
enum {  // Command bytes
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
#define NRF24X_R_REG_CMD(A) ((uint8_t)(NRF24X_R_REGISTER | NRF24X_REG_ADDR(A)))
#define NRF24X_W_REG_CMD(A) ((uint8_t)(NRF24X_W_REGISTER | NRF24X_REG_ADDR(A)))

#define NRF24X_PIPE_NUM_SIZE 3

#define NRF24X_PIPE_NUM(P) GET_BITFIELD(NRF24X_PIPE_NUM_SIZE, P)
#define NRF24X_W_ACK_PAYLOAD_CMD(P) ((uint8_t)(NRF24X_W_ACK_PAYLOAD | NRF24X_PIPE_NUM(P)))

enum {  // Register addresses
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

enum {  // Data pipe numbers
	NRF24X_P0 = 0,
	NRF24X_P1 = 1,
	NRF24X_P2 = 2,
	NRF24X_P3 = 3,
	NRF24X_P4 = 4,
	NRF24X_P5 = 5
};

enum {  // CONFIG register bits
	NRF24X_MASK_RX_DR  = 6,
	NRF24X_MASK_TX_DS  = 5,
	NRF24X_MASK_MAX_RT = 4,
	NRF24X_EN_CRC      = 3,
	NRF24X_CRCO        = 2,
	NRF24X_PWR_UP      = 1,
	NRF24X_PRIM_RX     = 0
};

enum {  // SETUP_AW register values
	NRF24X_AW_3  = 0x01,
	NRF24X_AW_4  = 0x02,
	NRF24X_AW_5  = 0x03
};

#define NRF24X_ARD_SIZE 4
#define NRF24X_ARD_OFFSET 4
#define NRF24X_ARC_SIZE 4

#define NRF24X_SETUP_RETR_VAL(ARD, ARC) \
	(MAKE_BITFIELD_AT(NRF24X_ARD_SIZE, NRF24X_ARD_OFFSET, ARD) | \
	 GET_BITFIELD(NRF24X_ARC_SIZE, ARC))

#define NRF24X_GET_ARD(SR) GET_BITFIELD_AT(NRF24X_ARD_SIZE, NRF24X_ARD_OFFSET, SR)

#define NRF24X_GET_ARC(SR) GET_BITFIELD(NRF24X_ARC_SIZE, SR)

#define NRF24X_RF_CH_SIZE 7

#define NRF24X_RF_CH_VAL(CH) GET_BITFIELD(NRF24X_RF_CH_SIZE, CH)

enum {  // RF_SETUP register bits
	NRF24X_PLL_LOCK  = 4,
	NRF24X_RF_DR     = 3,
	NRF24X_RF_PWR1   = 2,
	NRF24X_RF_PWR0   = 1,
	NRF24X_LNA_HCURR = 0
};

enum {  // STATUS register bits
	NRF24X_RX_DR      = 6,
	NRF24X_TX_DS      = 5,
	NRF24X_MAX_RT     = 4,
	NRF24X_RX_P_NO2   = 3,
	NRF24X_RX_P_NO1   = 2,
	NRF24X_RX_P_NO0   = 1,
	NRF24X_ST_TX_FULL = 0
};

#define NRF24X_RX_P_NO_OFFSET 1

#define NRF24X_GET_RX_P_NO(ST) \
	GET_BITFIELD_AT(NRF24X_PIPE_NUM_SIZE, NRF24X_RX_P_NO_OFFSET, ST)

#define NRF24X_PLOS_CNT_SIZE 4
#define NRF24X_PLOS_CNT_OFFSET 4

#define NRF24X_GET_PLOS_CNT(OTX) \
	GET_BITFIELD_AT(NRF24X_PLOS_CNT_SIZE, NRF24X_PLOS_CNT_OFFSET, OTX)

#define NRF24X_ARC_CNT_SIZE 4

#define NRF24X_GET_ARC_CNT(OTX) GET_BITFIELD(NRF24X_ARC_CNT_SIZE, OTX)

enum {  // CD register bits
	NRF24X_CD0 = 0
};

#define NRF24X_RX_PW_SIZE 6

#define NRF24X_RX_PW_VAL(PW) GET_BITFIELD(NRF24X_RX_PW_SIZE, PW)

enum {  // FIFO_STATUS register bits
	NRF24X_TX_REUSE = 6,
	NRF24X_TX_FULL  = 5,
	NRF24X_TX_EMPTY = 4,
	NRF24X_RX_FULL  = 1,
	NRF24X_RX_EMPTY = 0
};

enum {  // FEATURE register bits
	NRF24X_EN_DPL     = 2,
	NRF24X_EN_ACK_PAY = 1,
	NRF24X_EN_DYN_ACK = 0
};

extern volatile uint8_t nrf24x_status;

#ifndef NRF24X_SYNCHRONOUS

extern volatile uint8_t nrf24x_command;

#ifndef NRF24X_NO_CMD_BFR
extern uint8_t nrf24x_pending_n_in;
#endif

#define NRF24X_CMD_OUT(N, B) spihelper_request((N), (B), 1, 0, &nrf24x_status)

#define NRF24X_CMD_IN(N, O, B) spihelper_request(1, &nrf24x_command, (N), (O), (B))

#endif

uint8_t nrf24x_out_0(uint8_t cmd);

#if !defined(NRF24X_NO_CMD_BFR) || defined(NRF24X_SYNCHRONOUS)

uint8_t nrf24x_out_1(uint8_t cmd, uint8_t data);

uint8_t nrf24x_out_n(uint8_t cmd, uint8_t n_out, uint8_t *bfr_out);

uint8_t nrf24x_in_1(uint8_t cmd, uint8_t *data_p);

uint8_t nrf24x_in_n(uint8_t cmd, uint8_t n_in, uint8_t *bfr_in);

#endif

#if !defined(NRF24X_NO_CMD_BFR) && !defined(NRF24X_SYNCHRONOUS)
uint8_t nrf24x_in_finish(uint8_t *bfr_in);
#endif

#endif
