
#include <stddef.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "task_tlv.h"


// ---<<< Constant Definitions >>>---
#ifdef LIBAVR_ATMEGA_U

#define TTLV_RX_VECT USART1_RX_vect
#define TTLV_UDRE_VECT USART1_UDRE_vect

#define TTLV_U2X U2X1
#define TTLV_UPE UPE1
#define TTLV_DOR DOR1
#define TTLV_FE FE1

#define TTLV_TXEN TXEN1
#define TTLV_RXEN RXEN1
#define TTLV_UDRIE UDRIE1
#define TTLV_RXCIE RXCIE1

#define TTLV_UCSZ0 UCSZ10
#define TTLV_UCSZ1 UCSZ11
#define TTLV_UPM0 UPM10

#define TTLV_UBRR UBRR1
#define TTLV_UCSRA UCSR1A
#define TTLV_UCSRB UCSR1B
#define TTLV_UCSRC UCSR1C
#define TTLV_UDR UDR1

#else

#define TTLV_RX_VECT USART_RX_vect
#define TTLV_UDRE_VECT USART_UDRE_vect

#define TTLV_U2X U2X0
#define TTLV_UPE UPE0
#define TTLV_DOR DOR0
#define TTLV_FE FE0

#define TTLV_TXEN TXEN0
#define TTLV_RXEN RXEN0
#define TTLV_UDRIE UDRIE0
#define TTLV_RXCIE RXCIE0

#define TTLV_UCSZ0 UCSZ00
#define TTLV_UCSZ1 UCSZ01
#define TTLV_UPM0 UPM00

#define TTLV_UBRR UBRR0
#define TTLV_UCSRA UCSR0A
#define TTLV_UCSRB UCSR0B
#define TTLV_UCSRC UCSR0C
#define TTLV_UDR UDR0

#endif

#define USART_ERROR_BITS (BV(TTLV_UPE) | BV(TTLV_DOR) | BV(TTLV_FE))

enum {
	BFR_FLAG_DATA      = BV(0),        // Bytes have been successfully received or transmitted.
	BFR_FLAG_DONE      = BV(1),        // Transmit ISR disabled itself due to an empty buffer.
	BFR_FLAG_E_PARITY  = BV(TTLV_UPE), // Received byte with parity error.
	BFR_FLAG_E_OVERRUN = BV(TTLV_DOR), // Received bytes were dropped due to a full USART buffer.
	BFR_FLAG_E_FRAME   = BV(TTLV_FE),  // Received invalid byte frame.
	BFR_FLAG_E_DROP    = BV(5)         // Received bytes were dropped due to a full TTLV buffer.
};

#define BFR_ERROR_FLAGS \
	(BFR_FLAG_E_PARITY | BFR_FLAG_E_OVERRUN | BFR_FLAG_E_FRAME | BFR_FLAG_E_DROP)


// ---<<< API Variables >>>---
ttlv_mode ttlv_mode_flags;

// Awaken tasks in these categories when there is a change in the state
// of any transmit operation.
sched_catflags ttlv_xmit_task_cats;

// Awaken tasks in these categories when there is a change in the state
// of any receive operation.
sched_catflags ttlv_recv_task_cats;

ttlv_state ttlv_xmit_state = TTLV_DISABLED;
ttlv_inm_header ttlv_xmit_inm_header;
ttlv_header ttlv_xmit_header;

ttlv_state ttlv_recv_state = TTLV_DISABLED;
ttlv_inm_header ttlv_recv_inm_header;
ttlv_header ttlv_recv_header;


// ---<<< Static Variables >>>---
static volatile sched_catflags ttlv_man_cat_bv;

// This is a circular array queue. The transmit ISR gets bytes from
// the head of the committed range of the queue and the TTLV transmitter
// commits bytes for transmission at the tail of the range. The buffer
// space outside the committed range is used for buffering
// of uncommitted bytes.
static volatile uint8_t xmit_bfr_flags;
static volatile uint8_t xmit_bfr_start;
static volatile uint8_t xmit_bfr_n;
static volatile uint8_t xmit_bfr[TTLV_XMIT_BFR_SIZE];

// This is a circular array queue. The receive ISR puts bytes at the
// tail of the queue and the TTLV receiver commits received bytes at
// the head of the queue.
static volatile uint8_t recv_bfr_flags;
static volatile uint8_t recv_bfr_start;
static volatile uint8_t recv_bfr_n;
static volatile uint8_t recv_bfr[TTLV_RECV_BFR_SIZE];

static uint8_t ttlv_man_num_cat;

static uint8_t n_header_bytes;

#define MAX_TLV_LENGTH (0xff - n_header_bytes)

// End of committed buffer range, start of uncommitted range.
static uint8_t xmit_bfr_end;
// Total length (headers + value) of message being transmitted.
static uint8_t xmit_msg_length;
// Number of uncommitted message bytes in the transmit buffer.
static uint8_t xmit_n_uncommitted;
// Number of message bytes committed for transmission.
static uint8_t xmit_n_committed;

// Total length (headers + value) of message being received.
static uint8_t recv_msg_length;
// Number of message bytes received.
static uint8_t recv_n_committed;


// ---<<< Static Helper Functions >>>---
static uint8_t uint8_min(uint8_t a, uint8_t b) {
	return (a < b) ? a : b;
}

static uint8_t buffer_index(uint8_t i, uint8_t size) {
	return (i >= size) ? i - size : i;
}

static uint8_t buffer_read(
	uint8_t *to, volatile const uint8_t *bfr_from,
	uint8_t bfr_size, uint8_t bfr_i, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		to[i] = bfr_from[bfr_i];
		if (++bfr_i == bfr_size)
			bfr_i = 0;
	}
	
	return bfr_i;
}

static uint8_t buffer_write(
	volatile uint8_t *bfr_to, const uint8_t *from,
	uint8_t bfr_size, uint8_t bfr_i, uint8_t n)
{
	for (uint8_t i = 0; i < n; i++) {
		bfr_to[bfr_i] = from[i];
		if (++bfr_i == bfr_size)
			bfr_i = 0;
	}
	
	return bfr_i;
}


// ---<<< ISRs >>>---
ISR(TTLV_RX_VECT) { // USART has received and buffered a data byte.
	uint8_t csra = TTLV_UCSRA; // Read USART status flags BEFORE the data byte.
	uint8_t data = TTLV_UDR; // Always read TTLV_UDR to clear the RxC interrupt.
	
	// IDEA: Add 9th bit message synchronization check.
	
	csra &= USART_ERROR_BITS; // Only the error bits are of interest.
	
	if (csra) // An USART receive error has occurred.
		recv_bfr_flags |= csra; // Inform the TTLV receiver about USART errors.
	else {
		uint8_t n = recv_bfr_n;
		
		if (n < TTLV_RECV_BFR_SIZE) {
			uint8_t i = buffer_index(recv_bfr_start + n, TTLV_RECV_BFR_SIZE);
			
			recv_bfr[i] = data;
			recv_bfr_n = n + 1;
			
			recv_bfr_flags |= BFR_FLAG_DATA; // Inform the TTLV receiver about received byte.
		}
		else
			recv_bfr_flags |= BFR_FLAG_E_DROP; // Inform the TTLV receiver about dropped byte.
	}
	
	sched_isr_tcww |= ttlv_man_cat_bv; // Notify the TTLV receiver about this event.
}

ISR(TTLV_UDRE_VECT) { // USART is ready to receive a data byte for transmission.
	uint8_t n = xmit_bfr_n;
	
	if (n > 0) {
		uint8_t start = xmit_bfr_start;
		uint8_t data = xmit_bfr[start];
		
		xmit_bfr_start = buffer_index(start + 1, TTLV_XMIT_BFR_SIZE);
		xmit_bfr_n = n - 1;
		
		TTLV_UDR = data; // Transmit data byte.
		
		xmit_bfr_flags |= BFR_FLAG_DATA; // Inform the TTLV transmitter about transmitted byte.
	}
	else { // No more bytes to send.
		TTLV_UCSRB &= ~BV(TTLV_UDRIE); // Disable the UDRE interrupt (or it would just be re-triggered).
		
		xmit_bfr_flags |= BFR_FLAG_DONE; // Inform the TTLV transmitter about being done.
	}
	
	sched_isr_tcww |= ttlv_man_cat_bv; // Notify the TTLV transmitter about this event.
}


// ---<<< Static Routines >>>---
static void run_recv_state_machine(void) {
	uint8_t bfr_start = recv_bfr_start;
	uint8_t bfr_n = recv_bfr_n;
	uint8_t bfr_i;
	
	if (ttlv_recv_state == TTLV_READY) {
		if (TTLV_MODE_INM & ttlv_mode_flags) {
			if (bfr_n >= sizeof(ttlv_inm_header)) {
				// Copy header bytes into the header field.
				bfr_i = bfr_start;
				bfr_i = buffer_read(
					ttlv_recv_inm_header.b, recv_bfr,
					TTLV_RECV_BFR_SIZE, bfr_i, sizeof(ttlv_inm_header));
				
				bfr_start = bfr_i;
				recv_n_committed = sizeof(ttlv_inm_header);
				
				ATOMIC_BLOCK(ATOMIC_FORCEON) {
					recv_bfr_start = bfr_start;
					bfr_n = recv_bfr_n;
					bfr_n -= sizeof(ttlv_inm_header);
					recv_bfr_n = bfr_n;
				}
				
				// Inspect the INM header.
				// TODO: Detect duplicate message IDs.
				// ISSUE: Should routed/incorrect destination addresses be detected and
				//        handled here, or is it more convenient to do it in application
				//        code or an add-on routing task? I'm leaning toward the latter.
				
				// Update module state.
				ttlv_recv_state = TTLV_INM_HEADER;
				
				// Notify tasks about the received INM header.
				sched_task_tcww |= ttlv_recv_task_cats;
			}
		}
		else if (bfr_n >= sizeof(ttlv_header)) {
			recv_n_committed = 0;
			ttlv_recv_state = TTLV_INM_HEADER; // Pretend to have received an INM header.
		}
	}
	
	if (ttlv_recv_state == TTLV_INM_HEADER && bfr_n >= sizeof(ttlv_header)) {
		// Copy header bytes into the header field.
		bfr_i = bfr_start;
		bfr_i = buffer_read(
			ttlv_recv_header.b, recv_bfr,
			TTLV_RECV_BFR_SIZE, bfr_i, sizeof(ttlv_header));
		
		recv_n_committed += sizeof(ttlv_header);
		
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			recv_bfr_start = bfr_i;
			bfr_n = recv_bfr_n;
			bfr_n -= sizeof(ttlv_header);
			recv_bfr_n = bfr_n;
		}
		
		// Inspect the TLV header.
		if (ttlv_recv_header.h.length > MAX_TLV_LENGTH)
			ttlv_recv_state = TTLV_E_LENGTH;
		else {
			recv_msg_length = n_header_bytes + ttlv_recv_header.h.length;
			
			// Update module state.
			ttlv_recv_state = TTLV_HEADER;
		}
		
		// Notify tasks about the received TLV header.
		sched_task_tcww |= ttlv_recv_task_cats;
	}
	
	// Check whether we have a complete or partial value sequence.
	// If we do, update module state and notify tasks about
	// available value bytes.
	if (ttlv_recv_state == TTLV_HEADER) {
		if (bfr_n >= (recv_msg_length - recv_n_committed)) { // Complete message.
			ttlv_recv_state = TTLV_VALUE_DONE;
			
			// Notify tasks about the received complete message.
			sched_task_tcww |= ttlv_recv_task_cats;
		}
		else if ((TTLV_MODE_STREAM & ttlv_mode_flags) && bfr_n > 0) { // Some value bytes.
			// Notify tasks about the received value bytes.
			sched_task_tcww |= ttlv_recv_task_cats;
		}
	}
}

static void ttlv_handler(sched_task *task) {
	uint8_t xmit_flags;
	uint8_t recv_flags;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON) { // Check what the ISRs are reporting.
		xmit_flags = xmit_bfr_flags;
		xmit_bfr_flags = 0;
		recv_flags = recv_bfr_flags;
		recv_bfr_flags = 0;
	}
	
	if (BFR_FLAG_DATA & xmit_flags) { // More data transmitted.
		uint8_t bfr_n = xmit_bfr_n;
		uint8_t bfr_n_free = TTLV_XMIT_BFR_SIZE - bfr_n;
		
		if (ttlv_xmit_state == TTLV_VALUE || bfr_n_free > n_header_bytes) {
			// Notify tasks about available transmit buffer space.
			sched_task_tcww |= ttlv_xmit_task_cats;
		}
	}
	
	if (BFR_FLAG_DONE & xmit_flags) {
		// The transmit ISR has nothing more to do and has disabled itself.
		
		// TODO: Disable the USART transmitter?
		
		if (ttlv_xmit_state == TTLV_ACTIVE) // All pending messages transmitted.
			ttlv_xmit_state = TTLV_READY;
	}
	
	if (BFR_ERROR_FLAGS & recv_flags) { // A receive error has occurred.
		// Update module state to reflect the specific error.
		if (BFR_FLAG_E_FRAME & recv_flags)
			ttlv_recv_state = TTLV_E_FRAME;
		else if (BFR_FLAG_E_PARITY & recv_flags)
			ttlv_recv_state = TTLV_E_PARITY;
		else if (BFR_FLAG_E_OVERRUN & recv_flags)
			ttlv_recv_state = TTLV_E_OVERRUN;
		else if (BFR_FLAG_E_DROP & recv_flags)
			ttlv_recv_state = TTLV_E_DROP;
		else
			ttlv_recv_state = TTLV_E_UNSPECIFIED;
		
		// TODO: Disable the RX interrupt and the USART receiver?
		
		// Notify tasks about the error.
		sched_task_tcww |= ttlv_recv_task_cats;
	}
	
	if (BFR_FLAG_DATA & recv_flags) { // More data received.
		run_recv_state_machine();
	}
	
	task->st |= TASK_SLEEP_BIT; // Go back to sleep.
}


// ---<<< API Functions >>>---
void ttlv_init(
	uint8_t task_num_cat, uint16_t ubrr, uint8_t parity, uint8_t u2x,
	ttlv_mode mode, sched_catflags xmit_task_cats, sched_catflags recv_task_cats)
{
	sched_task task;
	
	// Configure the USART.
	TTLV_UBRR = ubrr; // Set baud rate. BAUD = Fosc / ((u2x ? 8 : 16)*(UBRR + 1)).
	TTLV_UCSRA = 0; // Clear TTLV_UCSRA.
	TTLV_UCSRA |= (0x1 & u2x) << TTLV_U2X; // Set double-speed mode.
	TTLV_UCSRB = 0; // Clear TTLV_UCSRB.
	TTLV_UCSRC = 0; // Clear TTLV_UCSRC.
	TTLV_UCSRC |= (0x3 & parity) << TTLV_UPM0; // Set parity checking mode.
	TTLV_UCSRC |= BV(TTLV_UCSZ0) | BV(TTLV_UCSZ1); // Enable 8-bit serial data bytes.
	
	ttlv_mode_flags = mode;
	ttlv_xmit_task_cats = xmit_task_cats;
	ttlv_recv_task_cats = recv_task_cats;
	
	ttlv_xmit_state = TTLV_READY;
	ttlv_recv_state = TTLV_READY;
	
	xmit_bfr_flags = 0;
	xmit_bfr_start = 0;
	xmit_bfr_n = 0;
	
	recv_bfr_flags = 0;
	recv_bfr_start = 0;
	recv_bfr_n = 0;
	
	task_num_cat &= TASK_ST_NUM_CAT_MASK;
	ttlv_man_num_cat = task_num_cat;
	ttlv_man_cat_bv = TASK_ST_GET_CATFLAG(task_num_cat);
	
	n_header_bytes = (TTLV_MODE_INM & mode)
		? TTLV_HEADER_BYTES_INM
		: TTLV_HEADER_BYTES_TLV;
	
	xmit_bfr_end = 0;
	xmit_msg_length = 0;
	xmit_n_uncommitted = n_header_bytes; // Reserve buffer space for headers.
	xmit_n_committed = 0;
	
	recv_msg_length = 0;
	recv_n_committed = 0;
	
	// Create the TTLV receiver/transmitter task.
	task = (sched_task) {
		.st = task_num_cat | TASK_SLEEP_BIT,
		.delay = SCHED_TIME_ZERO,
		.handler = ttlv_handler
	};
	sched_add(&task);
	
	// Enable the USART and the RX interrupt.
	TTLV_UCSRB |= BV(TTLV_RXEN) | BV(TTLV_TXEN);
	TTLV_UCSRB |= BV(TTLV_RXCIE);
}

uint8_t ttlv_put_byte(uint8_t data) {
	return ttlv_put_bytes(1, &data);
}

uint8_t ttlv_put_bytes(uint8_t n, const uint8_t *data_p) {
	uint8_t bfr_n;
	uint8_t bfr_n_free;
	uint8_t n_max;
	uint8_t bfr_i;
	
	if (TTLV_IS_ERROR(ttlv_xmit_state))
		return 0; // Transmitter has encountered an error and ceased to function.
	
	bfr_n = xmit_bfr_n; // Volatile read.
	bfr_n_free = TTLV_XMIT_BFR_SIZE - bfr_n;
	
	if (n_header_bytes > bfr_n_free) // Not enough space for headers.
		return 0; // No space for TLV value bytes.
	
	bfr_n_free -= xmit_n_uncommitted;
	n_max = uint8_min(n, bfr_n_free);
	
	// ISSUE: Is this check for too many bytes worth doing?
	//if (ttlv_xmit_state == TTLV_VALUE)
	//	n_max = uint8_min(n_max, xmit_msg_length - (xmit_n_committed + xmit_n_uncommitted));
	
	bfr_i = buffer_index(xmit_bfr_end + xmit_n_uncommitted, TTLV_XMIT_BFR_SIZE);
	bfr_i = buffer_write(xmit_bfr, data_p, TTLV_XMIT_BFR_SIZE, bfr_i, n_max);
	
	xmit_n_uncommitted += n_max;
	
	if (ttlv_xmit_state == TTLV_VALUE) {
		// If in stream mode and transmitting value bytes, commit additional bytes.
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// Don't clobber updates made by the ISR.
			xmit_bfr_n += xmit_n_uncommitted;
			
			TTLV_UCSRB |= BV(TTLV_UDRIE); // Enable the UDRE interrupt.
		}
		
		// Update buffer counters.
		xmit_bfr_end = bfr_i;
		xmit_n_committed += xmit_n_uncommitted;
		xmit_n_uncommitted = 0;
		
		if (xmit_n_committed == xmit_msg_length) { // Complete message committed.
			xmit_msg_length = 0;
			xmit_n_committed = 0;
			xmit_n_uncommitted = n_header_bytes; // Reserve buffer space for headers.
			
			ttlv_xmit_state = TTLV_ACTIVE; // Ready to buffer another message for transmission.
		}
	}
	
	return n_max;
}

uint8_t ttlv_unput_bytes(uint8_t n) {
	uint8_t n_max;
	
	if (xmit_n_uncommitted <= n_header_bytes)
		return 0;
	
	n_max = uint8_min(n, xmit_n_uncommitted - n_header_bytes);
	
	xmit_n_uncommitted -= n_max;
	
	return n_max;
}

uint8_t ttlv_try_put_bytes(uint8_t n, const uint8_t *data_p) {
	if (ttlv_put_bytes(n, data_p) != n) {
		ttlv_unput_bytes(TTLV_XMIT_BFR_SIZE);
		return 0;
	}
	
	return 1;
}

ttlv_state ttlv_begin_xmit(void) {
	uint8_t bfr_n;
	uint8_t bfr_n_free;
	uint8_t bfr_i;
	
	// Check module state.
	if (TTLV_IS_ERROR(ttlv_xmit_state))
		return ttlv_xmit_state; // Transmitter has encountered an error and ceased to function.
	
	if (ttlv_xmit_state == TTLV_VALUE) // Currently streaming TLV value.
		return TTLV_E_STATE; // Cannot begin new transmission.
	
	// Check buffer state.
	if (ttlv_xmit_header.h.length > MAX_TLV_LENGTH)
		return TTLV_E_LENGTH;
	
	xmit_msg_length = n_header_bytes + ttlv_xmit_header.h.length;
	
	if (!(TTLV_MODE_STREAM & ttlv_mode_flags) && xmit_n_uncommitted < xmit_msg_length)
		return TTLV_E_BUFFER; // Not enough buffered bytes.
	
	if (xmit_n_uncommitted > xmit_msg_length)
		return TTLV_E_BUFFER; // Too many buffered bytes.
	
	bfr_n = xmit_bfr_n; // Volatile read.
	bfr_n_free = TTLV_XMIT_BFR_SIZE - bfr_n;
	
	if (n_header_bytes > bfr_n_free) // Not enough space for headers.
		return TTLV_E_BUFFER; // No space for new message.
	
	// Copy header fields into the transmit buffer.
	bfr_i = xmit_bfr_end;
	
	if (TTLV_MODE_INM & ttlv_mode_flags) {
		bfr_i = buffer_write( // Copy INM header into transmit buffer.
			xmit_bfr, ttlv_xmit_inm_header.b,
			TTLV_XMIT_BFR_SIZE, bfr_i, sizeof(ttlv_inm_header));
		
		// Auto-increment the ID field of the transmit INM header field.
		ttlv_xmit_inm_header.h.msg_id++;
	}
	
	bfr_i = buffer_write( // Copy TLV header into transmit buffer.
		xmit_bfr, ttlv_xmit_header.b,
		TTLV_XMIT_BFR_SIZE, bfr_i, sizeof(ttlv_header));
	
	// Commit message bytes for transmission.
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		// Don't clobber updates made by the ISR.
		xmit_bfr_n += xmit_n_uncommitted;
		
		// ISSUE: Do we need to keep turning the USART transmitter on and off, or does it
		//        suffice to enable/disable the UDRE interrupt?
		
		TTLV_UCSRB |= BV(TTLV_UDRIE); // Enable the UDRE interrupt.
	}
	
	// Update buffer counters.
	xmit_bfr_end = buffer_index(xmit_bfr_end + xmit_n_uncommitted, TTLV_XMIT_BFR_SIZE);
	xmit_n_committed = xmit_n_uncommitted;
	
	// Update transmitter state.
	if (xmit_n_committed < xmit_msg_length) { // Additional value bytes expected.
		xmit_n_uncommitted = 0;
		
		ttlv_xmit_state = TTLV_VALUE;
	}
	else { // Complete message committed.
		xmit_msg_length = 0;
		xmit_n_committed = 0;
		xmit_n_uncommitted = n_header_bytes; // Reserve buffer space for headers.
		
		ttlv_xmit_state = TTLV_ACTIVE; // Ready to buffer another message for transmission.
	}
	
	return ttlv_xmit_state;
}

ttlv_state ttlv_try_begin_xmit(void) {
	uint8_t res = ttlv_begin_xmit();
	
	if (TTLV_IS_ERROR(res))
		ttlv_unput_bytes(TTLV_XMIT_BFR_SIZE);
	
	return res;
}

ttlv_state ttlv_xmit(uint8_t dstadr, uint8_t type, uint8_t length, const uint8_t *data_p) {
	if (ttlv_xmit_state != TTLV_READY && ttlv_xmit_state != TTLV_ACTIVE)
		return TTLV_E_STATE; // Cannot begin new transmission.
	
	ttlv_xmit_inm_header.h.dstadr = dstadr;
	ttlv_xmit_header.h.type = type;
	ttlv_xmit_header.h.length = length;
	
	if (length > 0)
		if (!ttlv_try_put_bytes(length, data_p))
			return TTLV_E_BUFFER;
	
	return ttlv_try_begin_xmit();
}

uint8_t ttlv_get_bytes(uint8_t n, uint8_t *data_p) {
	uint8_t bfr_n;
	uint8_t n_max;
	uint8_t bfr_i;
	
	if (ttlv_recv_state != TTLV_HEADER && ttlv_recv_state != TTLV_VALUE_DONE)
		return 0; // No data for you!
	
	bfr_n = recv_bfr_n; // Volatile read.
	
	n_max = uint8_min(n, TTLV_RECV_BFR_SIZE - bfr_n);
	n_max = uint8_min(n_max, recv_msg_length - recv_n_committed);
	
	if (n_max > 0) {
		bfr_i = recv_bfr_start; // Volatile but not updated by ISR.
		bfr_i = buffer_read(data_p, recv_bfr, TTLV_RECV_BFR_SIZE, bfr_i, n_max);
		
		recv_n_committed += n_max;
		
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// We need to provide consistent values for these two fields to the ISR.
			recv_bfr_start = bfr_i;
			recv_bfr_n -= n_max;
		}
	}
	
	return n_max;
}

ttlv_state ttlv_finish_recv(void) {
	uint8_t n_uncommitted;
	uint8_t bfr_start;
	
	// Check receiver state.
	if (ttlv_recv_state != TTLV_VALUE_DONE)
		return TTLV_E_STATE;
	
	n_uncommitted = recv_msg_length - recv_n_committed;
	
	// Update buffer counters.
	recv_msg_length = 0;
	recv_n_committed = 0;
	
	if (n_uncommitted > 0) { // Unread value bytes left in receive buffer.
		bfr_start = recv_bfr_start; // Volatile but not updated by ISR.
		bfr_start = buffer_index(bfr_start + n_uncommitted, TTLV_RECV_BFR_SIZE);
		
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			// We need to provide consistent values for these two fields to the ISR.
			recv_bfr_start = bfr_start;
			recv_bfr_n -= n_uncommitted;
		}
	}
	
	// Update receiver state.
	ttlv_recv_state = TTLV_READY;
	
	// Run the receiver state machine. Is there already another message
	// received or being received? In that case, copy header bytes, etc...
	run_recv_state_machine();
	
	return ttlv_recv_state;
}

ttlv_state ttlv_recv(uint8_t *data_p) {
	if (ttlv_recv_state != TTLV_VALUE_DONE)
		return TTLV_E_STATE; // No data for you!
	
	if (ttlv_recv_header.h.length > 0) {
		uint8_t n_bytes = ttlv_get_bytes(ttlv_recv_header.h.length, data_p);
		if (n_bytes != ttlv_recv_header.h.length)
			return TTLV_E_BUFFER;
	}
	
	return ttlv_finish_recv();
}

void ttlv_shutdown(void) {
	// Disable the USART and the RX and UDRE interrupts.
	TTLV_UCSRB &= ~(BV(TTLV_RXCIE) | BV(TTLV_UDRIE));
	TTLV_UCSRB &= ~(BV(TTLV_RXEN) | BV(TTLV_TXEN));
	
	// Remove the receiver/transmitter task.
	sched_remove(TASK_ST_NUM_CAT_MASK, ttlv_man_num_cat, 0);
	
	ttlv_xmit_state = TTLV_DISABLED;
	ttlv_recv_state = TTLV_DISABLED;
}
