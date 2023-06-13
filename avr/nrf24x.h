#ifndef AVR_NRF24X_H
#define AVR_NRF24X_H

#include <stdint.h>

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
enum {  // Command bytes.
	NRF24X_R_REGISTER         = 0x00000000,  // 0x00 (plus 5-bit register address)
	NRF24X_W_REGISTER         = 0x00100000,  // 0x20 (plus 5-bit register address)
	NRF24X_R_RX_PAYLOAD       = 0x01100001,  // 0x61
	NRF24X_W_TX_PAYLOAD       = 0x10100000,  // 0xa0
	NRF24X_FLUSH_TX           = 0x11100001,  // 0xe1
	NRF24X_FLUSH_RX           = 0x11100010,  // 0xe2
	NRF24X_REUSE_TX_PL        = 0x11100011,  // 0xe3
	NRF24X_ACTIVATE           = 0x01010000,  // 0x50
	NRF24X_ACTIVATE_2         = 0x01110011,  // 0x73
	NRF24X_R_RX_PL_WID        = 0x01100000,  // 0x60
	NRF24X_W_ACK_PAYLOAD      = 0x10101000,  // 0xa8 (plus 3-bit data pipe number)
	NRF24X_W_TX_PAYLOAD_NOACK = 0x10110000,  // 0xb0
	NRF24X_NOP                = 0x11111111,  // 0xff
};

#define NRF24X_REG_MASK 0b00011111
#define NRF24X_REG_ADDR(A) (NRF24X_REG_MASK & (A))
#define NRF24X_R_REG_CMD(A) ((uint8_t)(NRF24X_R_REGISTER | NRF24X_REG_ADDR(A)))
#define NRF24X_W_REG_CMD(A) ((uint8_t)(NRF24X_W_REGISTER | NRF24X_REG_ADDR(A)))

#define NRF24X_PIPE_MASK 0b00000111
#define NRF24X_PIPE_NUM(P) (NRF24X_PIPE_MASK & (P))
#define NRF24X_W_ACK_PAYLOAD_CMD(P) ((uint8_t)(NRF24X_W_ACK_PAYLOAD | NRF24X_PIPE_NUM(P)))

enum {  // Register addresses.
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
	NRF24X_RX_ADDR_P2  = 0x0c,  // 5 bytes
	NRF24X_RX_ADDR_P3  = 0x0d,  // 5 bytes
	NRF24X_RX_ADDR_P4  = 0x0e,  // 5 bytes
	NRF24X_RX_ADDR_P5  = 0x0f,  // 5 bytes
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
uint8_t nrf24x_in_finish(void);
#endif

#endif
