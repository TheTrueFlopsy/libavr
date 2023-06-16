
#include "spihelper.h"
#include "nrf24x.h"

volatile uint8_t nrf24x_status;

#ifndef NRF24X_SYNCHRONOUS

volatile uint8_t nrf24x_command;

#ifndef NRF24X_NO_CMD_BFR

#if (!defined(NRF24X_BFR_SIZE) || NRF24X_BFR_SIZE < 2 || NRF24X_BFR_SIZE > 33)
#error "NRF24X_BFR_SIZE must be defined and between 2 and 33 (inclusive)."
#endif

static volatile uint8_t cmd_bfr[NRF24X_BFR_SIZE];

uint8_t nrf24x_pending_n_in = NRF24X_NONE_PENDING;
//static uint8_t *pending_bfr_in;

#endif
#endif

uint8_t nrf24x_out_0(uint8_t cmd) {
#ifdef NRF24X_SYNCHRONOUS
	nrf24x_status = spihelper_exchange_bytes(cmd);
	return 1;
#else
	nrf24x_command = cmd;
	spi_state res = NRF24X_CMD_OUT(1, &nrf24x_command);
	return res == SPI_ACTIVE;
#endif
}

#if !defined(NRF24X_NO_CMD_BFR) || defined(NRF24X_SYNCHRONOUS)

uint8_t nrf24x_out_1(uint8_t cmd, uint8_t data) {
#ifdef NRF24X_SYNCHRONOUS
	nrf24x_status = spihelper_exchange_bytes(cmd);
	spihelper_exchange_bytes(data);
	return 1;
#else
	cmd_bfr[0] = cmd;
	cmd_bfr[1] = data;
	spi_state res = NRF24X_CMD_OUT(2, cmd_bfr);
	return res == SPI_ACTIVE;
#endif
}

uint8_t nrf24x_out_n(uint8_t cmd, uint8_t n_out, uint8_t *bfr_out) {
#ifdef NRF24X_SYNCHRONOUS
	nrf24x_status = spihelper_exchange_bytes(cmd);
	
	for (uint8_t i = 0; i < n_out; i++)
		spihelper_exchange_bytes(*bfr_out++);
	
	return 1;
#else
	cmd_bfr[0] = cmd;
	
	// ISSUE: Would it be OK to simply use memcpy() here?
	volatile uint8_t *cmd_bfr_p = cmd_bfr + 1;
	
	for (uint8_t i = 0; i < n_out; i++)
		*cmd_bfr_p++ = *bfr_out++;
	
	spi_state res = NRF24X_CMD_OUT(n_out + 1, cmd_bfr);
	return res == SPI_ACTIVE;
#endif
}

uint8_t nrf24x_in_1(uint8_t cmd, uint8_t *data_p) {
#ifdef NRF24X_SYNCHRONOUS
	nrf24x_status = spihelper_exchange_bytes(cmd);
	*data_p = spihelper_exchange_bytes(0);
	return 1;
#else
	nrf24x_command = cmd;
	nrf24x_pending_n_in = 1;
	//pending_bfr_in = data_p;
	spi_state res = NRF24X_CMD_IN(2, 0, cmd_bfr);
	return res == SPI_ACTIVE;
#endif
}

uint8_t nrf24x_in_n(uint8_t cmd, uint8_t n_in, uint8_t *bfr_in) {
#ifdef NRF24X_SYNCHRONOUS
	nrf24x_status = spihelper_exchange_bytes(cmd);
	
	for (uint8_t i = 0; i < n_in; i++)
		*bfr_in++ = spihelper_exchange_bytes(0);
	
	return 1;
#else
	nrf24x_command = cmd;
	nrf24x_pending_n_in = n_in;
	//pending_bfr_in = bfr_in;
	spi_state res = NRF24X_CMD_IN(n_in + 1, 0, cmd_bfr);
	return res == SPI_ACTIVE;
#endif
}

#endif

#if !defined(NRF24X_NO_CMD_BFR) && !defined(NRF24X_SYNCHRONOUS)
uint8_t nrf24x_in_finish(uint8_t *bfr_in) {
	// ISSUE: Add a check to ensure that no asynchronous SPI operation is ongoing?
	
	uint8_t n_in = nrf24x_pending_n_in;
	if (n_in == NRF24X_NONE_PENDING)
		return NRF24X_NONE_PENDING;
	
	// ISSUE: Would it be OK to simply use memcpy() here?
	volatile uint8_t *cmd_bfr_p = cmd_bfr;
	//uint8_t *bfr_in = pending_bfr_in;
	
	nrf24x_status = *cmd_bfr_p++;
	
	for (uint8_t i = 0; i < n_in; i++)
		*bfr_in++ = *cmd_bfr_p++;
	
	nrf24x_pending_n_in = NRF24X_NONE_PENDING;
	return n_in;
}
#endif
