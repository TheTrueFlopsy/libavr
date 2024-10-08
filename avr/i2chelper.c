
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2chelper.h"

#ifndef I2C_TWSR_INIT
// TWPS[1..0]=0 - prescaler divisor 4^0 == 1
#define I2C_TWSR_INIT 0
#endif

#define TWISC_BITMASK 0xf8

#define TWI_INT_EN_IE (BV(TWINT) | BV(TWEN) | BV(TWIE))
#define TWI_INT_STO_EN (BV(TWINT) | BV(TWSTO) | BV(TWEN))

enum {
	TWISC_START_T  = 0x08, // START transmitted.
	TWISC_RSTART_T = 0x10, // Repeated START transmitted.
	TWISC_SLAW_TA  = 0x18, // SLA+W transmitted, ACK received.
	TWISC_SLAW_TNA = 0x20, // SLA+W transmitted, NOT ACK received.
	TWISC_DATA_TA  = 0x28, // Data byte transmitted, ACK received.
	TWISC_DATA_TNA = 0x30, // Data byte transmitted, NOT ACK received.
	TWISC_ARB_LOST = 0x38, // Master arbitration lost.
	TWISC_SLAR_TA  = 0x40, // SLA+R transmitted, ACK received.
	TWISC_SLAR_TNA = 0x48, // SLA+R transmitted, NOT ACK received.
	TWISC_DATA_RA  = 0x50, // Data byte received, ACK sent.
	TWISC_DATA_RNA = 0x58  // Data byte received, NOT ACK sent.
};

volatile sched_catflags i2c_task_cats = 0;
volatile i2c_state i2c_request_state = I2C_DISABLED;

static volatile uint8_t slaw;
static volatile uint8_t slar;

static volatile uint8_t n_to_transmit;
static volatile const uint8_t *volatile transmit_ptr;

static volatile uint8_t n_to_receive;
static volatile uint8_t *volatile receive_ptr;

//volatile uint8_t last_twisc = 0xff; // DEBUG: 

ISR(TWI_vect) {
	// Get the value of the status code in the TWI status register.
	uint8_t twisc = TWISC_BITMASK & TWSR;
	uint8_t n_bytes;
	i2c_state next_state = I2C_ACTIVE;
	
	switch (twisc) {
	case TWISC_START_T:  // Started I2C transaction.
		// Send address+W if we have bytes to transmit, otherwise address+R.
		TWDR = (n_to_transmit > 0) ? slaw : slar;
		TWCR = TWI_INT_EN_IE;
		break;
	
	case TWISC_RSTART_T:  // Restarted to begin receive phase.
		TWDR = slar;  // Send address+R.
		TWCR = TWI_INT_EN_IE;
		break;
	
	case TWISC_SLAW_TA:  // Address+W was acknowledged.
		TWDR = *(transmit_ptr++);
		TWCR = TWI_INT_EN_IE;
		break;
	
	case TWISC_DATA_TA:  // Transmitted data byte was acknowledged.
		n_bytes = n_to_transmit - 1;
		
	transmit_update:
		if (n_bytes == 0) {  // Transmission done.
			if (n_to_receive == 0) {
				TWCR = TWI_INT_STO_EN;  // Transaction done.
				next_state = I2C_READY;
			}
			else
				TWCR = TWI_INT_EN_IE | BV(TWSTA);  // Restart to begin receive phase.
		}
		else {  // Continue transmission.
			TWDR = *(transmit_ptr++);
			TWCR = TWI_INT_EN_IE;
			n_to_transmit = n_bytes;
		}
		
		break;
	
	case TWISC_DATA_TNA:  // Transmitted data byte was NOT acknowledged.
		n_bytes = n_to_transmit - 1;
		if (n_bytes == 0)  // Transmission done.
			goto transmit_update;  // Behave as for TWISC_DATA_TA.
		TWCR = BV(TWINT);
		next_state = I2C_E_NOT_ACK;
		break;
	
	case TWISC_SLAR_TA:  // Address+R was acknowledged.
		// Begin receiving. Send NOT ACK if a single byte, otherwise ACK.
		TWCR = (n_to_receive == 1) ? TWI_INT_EN_IE : (TWI_INT_EN_IE | BV(TWEA));
		break;
	
	case TWISC_DATA_RA:   // Received data byte, acknowledged it...
	case TWISC_DATA_RNA:  // ...or not.
		*(receive_ptr++) = TWDR;
		n_bytes = n_to_receive - 1;
		
		if (n_bytes == 0) {  // Reception done.
			TWCR = TWI_INT_STO_EN;  // Transaction done.
			next_state = I2C_READY;
		}
		else {
			// Continue receiving. Send NOT ACK if last byte, otherwise ACK.
			TWCR = (n_bytes == 1) ? TWI_INT_EN_IE : (TWI_INT_EN_IE | BV(TWEA));
			n_to_receive = n_bytes;
		}
		
		break;
	
	case TWISC_SLAW_TNA:  // Address+W or...
	case TWISC_SLAR_TNA:  // ...address+R was NOT acknowledged.
		TWCR = BV(TWINT);
		next_state = I2C_E_NOT_ACK;
		break;
	
	case TWISC_ARB_LOST:  // Lost Master arbitration.
		TWCR = BV(TWINT);
		next_state = I2C_E_ARB_LOST;
		break;
	
	default:  // What is happening?!?
		TWCR = BV(TWINT);
		next_state = I2C_E_FSM;  // It is a mystery.
		//last_twisc = twisc; // DEBUG: 
		break;
	}
	
	if (next_state != I2C_ACTIVE) {  // I2C transaction ended, one way or another.
		i2c_request_state = next_state;  // Report transaction exit status.
		sched_isr_tcww |= i2c_task_cats;  // Notify tasks.
	}
}

void i2chelper_mstr_init(uint8_t twbr, sched_catflags task_cats) {
	// Set clock rate (via prescaler and TWBR).
	TWSR = I2C_TWSR_INIT;
	TWBR = twbr;
	i2c_task_cats = task_cats;
	i2c_request_state = I2C_READY;
}

i2c_state i2chelper_request(
	i2c_slave_addr addr, uint8_t n_out, volatile const uint8_t *bfr_out,
	uint8_t n_in, volatile uint8_t *bfr_in)
{
	if (n_out == 0 && n_in == 0)
		return I2C_E_LENGTH;
	
	i2c_state state = i2c_request_state;
	
	if (state >= I2C_E_UNSPECIFIED)
		return state;
	else if (state != I2C_READY)
		return I2C_E_STATE;
	
	i2c_request_state = I2C_ACTIVE;  // Initiate I2C transaction.
	
	// Prepare address+direction field for transmission.
	addr <<= 1;
	slaw = addr;
	slar = addr | 1;
	
	// Prepare transmit and receive buffers.
	n_to_transmit = n_out;
	transmit_ptr = bfr_out;
	
	n_to_receive = n_in;
	receive_ptr = bfr_in;
	
	// Enable the TWI hardware module. Clear the TWINT flag. Send START condition.
	TWCR = TWI_INT_EN_IE | BV(TWSTA);
	
	return I2C_ACTIVE;
}

void i2chelper_shutdown(void) {
	// Disable the TWI hardware module.
	TWCR = BV(TWINT);
	i2c_request_state = I2C_DISABLED;
	
	/*n_to_transmit = 0;
	transmit_ptr = NULL;
	
	n_to_receive = 0;
	receive_ptr = NULL;*/
}
