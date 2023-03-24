#ifndef AVR_TASK_TLV_H
#define AVR_TASK_TLV_H

#include <stdint.h>

#include "task_sched.h"

/**
	File: task_tlv.h
	Task-based TLV (TTLV) communication module for ATmega microcontrollers. Also supports
	the INM (Inter-Node Messaging) protocol. Uses the <task_sched.h> task scheduler to
	perform communication asynchronously.
*/

// IDEA: Add stdio support? What kind of interface and semantics? Use the STDIN/STDOUT/
//       STDERR registers? A subscribe/push solution seems like a fundamentally better idea.
//       But pushing STDIN is hard? Implement the push solution as "write to register R at
//       node N", where R may or may not be the STDIN register? What about the read-write
//       exchange registers (STDIO, etc.)? Implement both push and pull for output streams?
//       How should the buffering work? Implement "read register R at node N" fetching for
//       input streams? This could get complicated, and I'm not sure about the use cases.
//       Perhaps streaming is not a job for INM?

// TODO: Document some recommended standard ways to use this module.
// * Get awakened - check TTLV_HAS_HEADER/TTLV_HAS_MESSAGE - inspect headers
//   (e.g. with TTLV_CHECK_TL) - call ttlv_recv/ttlv_finish_recv - process
//   message - resume sleep.
// * For transmission, simply calling ttlv_xmit should suffice, unless you
//   need reliability and might be stuffing your buffer.


/// Section: Configuration Macros

/**
	Macro: TTLV_XMIT_BFR_SIZE
	Size in bytes of the TTLV transmit buffer. Configuration macro.
	
	Default value: 32
*/
#define TTLV_XMIT_BFR_SIZE 32

/**
	Macro: TTLV_RECV_BFR_SIZE
	Size in bytes of the TTLV receive buffer. Configuration macro.
	
	Default value: 32
*/
#define TTLV_RECV_BFR_SIZE 32


/// Section: Protocol Constants

/**
	Macro: TTLV_MAX_MESSAGE_ID
	Largest valid TTLV message ID. Constant macro.
*/
#define TTLV_MAX_MESSAGE_ID 0xffff

/**
	Macro: TTLV_MAX_MESSAGE_ADR
	Largest valid INM address. Constant macro.
*/
#define TTLV_MAX_MESSAGE_ADR 0xff

/**
	Macro: TTLV_LOCAL_ADR
	Special INM address representing the local node. Constant macro.
*/
#define TTLV_LOCAL_ADR 0x00

/**
	Macro: TTLV_BROADCAST_ADR
	Special INM address used for broadcast messages. Constant macro.
*/
#define TTLV_BROADCAST_ADR TTLV_MAX_MESSAGE_ADR

/**
	Macro: TTLV_MIN_TYPE_NUM
	Smallest valid TTLV message type number for standard messages. Constant macro.
*/
#define TTLV_MIN_TYPE_NUM 0x00

/**
	Macro: TTLV_MAX_TYPE_NUM
	Largest valid TTLV message type number for standard messages. Constant macro.
*/
#define TTLV_MAX_TYPE_NUM 0x7f

/**
	Macro: TTLV_MAX_MESSAGE_LEN
	Maximum TTLV message length for standard messages. Constant macro.
	
	CAUTION: The actual maximum message length this module can handle is less than
	this general upper limit. See <TTLV_MAX_LEN_TLV> and <TTLV_MAX_LEN_INM>.
*/
#define TTLV_MAX_MESSAGE_LEN 0xff

/**
	Macro: TTLV_PROTOCOL_TYPE_NUM
	TTLV message type number reserved for protocol messages. Constant macro.
*/
#define TTLV_PROTOCOL_TYPE_NUM TTLV_MAX_TYPE_NUM

/**
	Macro: TTLV_MIN_LARGE_TYPE_NUM
	Smallest valid TTLV message type number for large messages. Constant macro.
*/
#define TTLV_MIN_LARGE_TYPE_NUM 0x80

/**
	Macro: TTLV_MAX_LARGE_TYPE_NUM
	Largest valid TTLV message type number for large messages. Constant macro.
*/
#define TTLV_MAX_LARGE_TYPE_NUM 0xff

/**
	Macro: TTLV_MAX_LARGE_MESSAGE_LEN
	Maximum TTLV message length for large messages. Constant macro.
*/
#define TTLV_MAX_LARGE_MESSAGE_LEN 0xffffffffL


/// Section: Data Types

/**
	Enum: TTLV Module State/Result Codes
	
	NOTE: The error codes are those starting with "TTLV_E_". These all have
	numeric values greater than or equal to *TTLV_E_UNSPECIFIED*.
	
	TTLV_SUCCESS       - operation or function finished successfully
	TTLV_DISABLED      - module is disabled
	TTLV_READY         - module is idle and ready to transmit or receive
	TTLV_ACTIVE        - module is currently transmitting or receiving data
	TTLV_INM_HEADER    - module has an incoming INM header buffered, but not a TLV header
	TTLV_HEADER        - module has an incoming TLV header buffered, but not a complete message
	TTLV_VALUE         - module is currently transmitting a TLV value in streaming mode
	TTLV_VALUE_DONE    - module has a complete incoming message buffered
	TTLV_E_UNSPECIFIED - unspecified error
	TTLV_E_PARITY      - parity error in USART receiver
	TTLV_E_OVERRUN     - data overrun error in USART receiver
	TTLV_E_FRAME       - frame error in USART receiver
	TTLV_E_DROP        - data overrun error in TTLV receive buffer
	TTLV_E_LENGTH      - TLV length error (message too large)
	TTLV_E_STATE       - attempted operation not allowed in current module state
	TTLV_E_BUFFER      - data buffering error (too few or too many buffered bytes)
*/
enum {
	TTLV_DISABLED      = 0,
	TTLV_READY         = 1,
	TTLV_ACTIVE        = 2,
	TTLV_INM_HEADER    = 3,
	TTLV_HEADER        = 4,
	TTLV_VALUE         = 5,
	TTLV_VALUE_DONE    = 6,
	TTLV_E_UNSPECIFIED = 7,
	TTLV_E_PARITY      = 8,
	TTLV_E_OVERRUN     = 9,
	TTLV_E_FRAME       = 10,
	TTLV_E_DROP        = 11,
	TTLV_E_LENGTH      = 12,
	TTLV_E_STATE       = 13,
	TTLV_E_BUFFER      = 14
};

/**
	Enum: TTLV Mode Flags
	
	TTLV_MODE_INM    - Specifies that INM message headers should be used.
	TTLV_MODE_STREAM - Specifies that stream I/O should be enabled.
*/
enum {
	TTLV_MODE_INM    = BV(0),
	TTLV_MODE_STREAM = BV(1)
};

/**
	Enum: TTLV Byte Parity Codes
	
	TTLV_PARITY_NONE - no byte parity
	TTLV_PARITY_EVEN - even byte parity
	TTLV_PARITY_ODD  - odd byte parity
*/
enum {
	TTLV_PARITY_NONE = 0,
	TTLV_PARITY_EVEN = 2,
	TTLV_PARITY_ODD  = 3
};

/**
	Ref: ttlv_mode
	The type of mode flags for the TTLV module.
*/
typedef uint8_t ttlv_mode;

/**
	Ref: ttlv_state
	The type of state codes for the TTLV module.
*/
typedef uint8_t ttlv_state;

/**
	Struct: ttlv_s_inm_header
	Specifies the byte format of an INM message header.
*/
typedef struct __attribute__ ((__packed__)) ttlv_s_inm_header {
	/**
		Field: msg_id
		Identification number of the INM message. These are generated independently
		by each source node.
	*/
	uint16_t msg_id;
	
	/**
		Field: dstadr
		INM address of the destination node.
	*/
	uint8_t dstadr;
	
	/**
		Field: srcadr
		INM address of the source node.
	*/
	uint8_t srcadr;
	
} ttlv_s_inm_header;

/**
	Struct: ttlv_inm_header
	Provides byte access to the <ttlv_s_inm_header> struct *h* via the byte array *b*.
*/
typedef union ttlv_inm_header {
	ttlv_s_inm_header h;
	uint8_t b[sizeof(ttlv_s_inm_header)];
} ttlv_inm_header;

/**
	Struct: ttlv_s_header
	Specifies the byte format of a TLV header.
*/
typedef struct __attribute__ ((__packed__)) ttlv_s_header {
	/**
		Field: type
		TLV type identifier field. The type identifiers are application-defined.
	*/
	uint8_t type;
	
	/**
		Field: length
		TLV length field. Specifies the size in bytes of the TLV value field that follows.
	*/
	uint8_t length;
	
} ttlv_s_header;

/**
	Struct: ttlv_header
	Provides byte access to the <ttlv_s_header> struct *h* via the byte array *b*.
*/
typedef union ttlv_header {
	ttlv_s_header h;
	uint8_t b[sizeof(ttlv_s_header)];
} ttlv_header;

/**
	Macro: TTLV_HEADER_BYTES_TLV
	Size in bytes of a TLV header. Constant macro.
*/
#define TTLV_HEADER_BYTES_TLV sizeof(ttlv_header)

/**
	Macro: TTLV_HEADER_BYTES_INM
	Combined size in bytes of an INM header and a TLV header.
*/
#define TTLV_HEADER_BYTES_INM (sizeof(ttlv_inm_header) + sizeof(ttlv_header))

// TODO: Get rid of these limitations if they become a problem.
/**
	Macro: TTLV_MAX_LEN_TLV
	Maximum TLV length of a plain TLV message handled by the TTLV module.
*/
#define TTLV_MAX_LEN_TLV (0xff - TTLV_HEADER_BYTES_TLV)

/**
	Macro: TTLV_MAX_LEN_INM
	Maximum TLV length of an INM message handled by the TTLV module.
*/
#define TTLV_MAX_LEN_INM (0xff - TTLV_HEADER_BYTES_INM)


/// Section: API Variables

/**
	Variable: ttlv_mode_flags
	Contains a set of mode flags that represent the current operation mode
	of the TTLV module.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern ttlv_mode ttlv_mode_flags;

/**
	Variable: ttlv_xmit_task_cats
	Contains a set of task category bit flags. Tasks in the indicated categories will
	be awakened in response to certain events in the TTLV transmitter.
	
	The following transmitter events will trigger awakening of tasks:
		+ Additional transmit buffer space becomes available while a message is being
		  transmitted in streaming mode.
*/
extern sched_catflags ttlv_xmit_task_cats;

/**
	Variable: ttlv_recv_task_cats
	Contains a set of task category bit flags. Tasks in the indicated categories will
	be awakened in response to certain events in the TTLV receiver.
	
	The following receiver events will trigger awakening of tasks:
		+ A complete INM header is received in INM mode.
		+ A complete TLV header is received.
		+ A complete message is received.
		+ Additional data bytes become available while a message is being received in
		  streaming mode.
		+ A communication error is detected by the TTLV receiver.
*/
extern sched_catflags ttlv_recv_task_cats;

/**
	Variable: ttlv_xmit_state
	Contains a state code representing the current state of the TTLV transmitter.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern ttlv_state ttlv_xmit_state;

/**
	Variable: ttlv_xmit_inm_header
	Contains the INM header to transmit.
	
	When the TTLV module is used in INM mode, the *srcadr* field of this struct
	MUST be set to an appropriate value by external code before any function that
	initiates transmission of a message is called. Once set, the same value may be
	used for several messages.
	
	NOTE: In INM mode, the *msg_id* field of this struct is incremented by
	the TTLV transmitter each time an INM header has been committed for
	transmission.
*/
extern ttlv_inm_header ttlv_xmit_inm_header;

/**
	Variable: ttlv_xmit_header
	Contains the TLV header to transmit.
*/
extern ttlv_header ttlv_xmit_header;

/**
	Variable: ttlv_recv_state
	Contains a state code representing the current state of the TTLV receiver.
	
	CAUTION: Do not update this variable in external code, unless you know what
	you're doing.
*/
extern ttlv_state ttlv_recv_state;

/**
	Variable: ttlv_recv_inm_header
	Contains the most recently received INM header.
	
	CAUTION: Do not update this struct in external code, unless you know what
	you're doing.
*/
extern ttlv_inm_header ttlv_recv_inm_header;

/**
	Variable: ttlv_recv_header
	Contains the most recently received TLV header.
	
	CAUTION: Do not update this struct in external code, unless you know what
	you're doing.
*/
extern ttlv_header ttlv_recv_header;


/// Section: Helper Macros

/**
	Macro: BAUD_TO_UBRR
	Converts a baud rate to an UBRR register value that can be passed
	to <ttlv_init>. Function-like macro.
	
	CAUTION: This macro depends on the *F_CPU* macro deing defined
	as a long integer expression representing the master clock
	frequency of the target ATmega controller.
	
	Parameters:
		B - A positive integer baud rate. *B* MUST be less than or
			equal to *F_CPU* / 16 (*F_CPU* / 8 if *X* is true).
		X - A Boolean flag indicating whether the double-speed mode
			of the ATmega USART will be used.
	
	Returns:
		A nonnegative integer UBRR clock divisor value that may be passed
		to <ttlv_init>.
*/
#define BAUD_TO_UBRR(B, X) ((uint16_t)((F_CPU)/(((int32_t)(B)*8)*(2-!!(X)))-1))

/**
	Macro: TTLV_HAS_MESSAGE
	Evaluates to a true value if and only if at least one complete received
	message is currently pending. Expression macro.
*/
#define TTLV_HAS_MESSAGE (ttlv_recv_state == TTLV_VALUE_DONE)

/**
	Macro: TTLV_HAS_HEADER
	Evaluates to a true value if and only if at least one received message header
	(TLV header only in TLV mode, INM+TLV header in INM mode) is currently
	pending. Expression macro.
*/
#define TTLV_HAS_HEADER (ttlv_recv_state == TTLV_HEADER || TTLV_HAS_MESSAGE)

/**
	Macro: TTLV_IS_ERROR
	Evaluates to a true value if and only if the argument is a TTLV error code.
	Function-like macro.
	
	CAUTION: This macro may evaluate true if the argument is not a valid TTLV
	result code.
	
	Parameters:
		S - A <ttlv_state> result code.
	
	Returns:
		True if and only if *S* is a TTLV error code.
*/
#define TTLV_IS_ERROR(S) ((S) >= TTLV_E_UNSPECIFIED)

/**
	Macro: TTLV_CHECK_TL
	Evaluates to a true value and only if the specified TLV message type
	identifier and length match the ones currently stored in <ttlv_recv_header>.
	The type identifier is tested for equality, while the message length is
	tested for being less than or equal to the one in <ttlv_recv_header>.
	Function-like macro.
	
	Parameters:
		T - A TLV message type identifier.
		L - A TLV message length.
	
	Returns:
		True if and only if the arguments match the current <ttlv_recv_header>.
*/
#define TTLV_CHECK_TL(T, L) \
	(ttlv_recv_header.h.type == (T) && ttlv_recv_header.h.length >= (L))

/**
	Macro: TTLV_PTR
	Converts a pointer to an arbitrary type into a pointer to another
	arbitrary type, without triggering compiler warnings.
	Function-like macro.
	
	Parameters:
		T - A type name. The macro will evaluate to a pointer to an instance
			of this type.
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A pointer to *T*, containing the same address as *P*.
*/
#define TTLV_PTR(T, P) ((T*)(void*)(P))

/**
	Macro: TTLV_CPTR
	Converts a pointer to an arbitrary type into a pointer to a constant
	instance of another arbitrary type, without triggering compiler warnings.
	Function-like macro.
	
	Parameters:
		T - A type name. The macro will evaluate to a pointer to a constant
			instance of his type.
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A pointer to *const T*, containing the same address as *P*.
*/
#define TTLV_CPTR(T, P) ((const T*)(const void*)(P))

/**
	Macro: TTLV_DATA_PTR
	Converts a pointer to an arbitrary type into a pointer to *uint8_t*,
	without triggering compiler warnings. Function-like macro.
	
	Parameters:
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A pointer to *uint8_t*, containing the same address as *P*.
*/
#define TTLV_DATA_PTR(P) TTLV_PTR(uint8_t, (P))

/**
	Macro: TTLV_DATA_CPTR
	Converts a pointer to an arbitrary type into a pointer to
	*const uint8_t*, without triggering compiler warnings.
	Function-like macro.
	
	Parameters:
		P - A pointer. The macro will evaluate to a pointer containing the
			same address as this pointer.
	
	Returns:
		A pointer to *const uint8_t*, containing the same address as *P*.
*/
#define TTLV_DATA_CPTR(P) TTLV_CPTR(uint8_t, (P))


/// Section: API Functions

/**
	Function: ttlv_init
	Initializes the TTLV communication module.
	
	CAUTION: This function MUST be called before any other API function
	of the TTLV module is called.
	
	Parameters:
		task_num_cat - A TCSB value containing the task instance and category
			numbers that should be used by the TTLV receiver/transmitter task.
			(The SLP bit of the argument is ignored.)
		ubrr - A nonnegative clock divisor value that determines the baud rate
			of the ATmega USART. The <BAUD_TO_UBRR> macro may be used to convert
			a baud rate to a corresponding UBRR value. Note that this conversion
			is approximate, i.e. the baud rate produced by the resulting UBRR
			value may be slightly different from the one given to the macro.
		parity - A value that specifies the type of byte parity checking to be
			used by the ATmega USART. This should be one of the *TTLV_PARITY_x*
			enum constants.
		u2x - A value that specifies whether double-speed mode should be used
			by the ATmega USART. This should be either 0 (false) or 1 (true).
			The double-speed reception mode doubles the baud rate for a given
			UBRR value.
		mode - A set of bit flags specifying the mode the TTLV module should
			operate in,
		xmit_task_cats - A set of bit flags specifying task categories whose
			members should be awakened in response to certain events in the
			TTLV transmitter. See <ttlv_xmit_task_cats> for details.
		recv_task_cats - A set of bit flags specifying task categories whose
			members should be awakened in response to certain events in the
			TTLV receiver. See <ttlv_recv_task_cats> for details.
*/
void ttlv_init(
	uint8_t task_num_cat, uint16_t ubrr, uint8_t parity, uint8_t u2x,
	ttlv_mode mode, sched_catflags xmit_task_cats, sched_catflags recv_task_cats);

/**
	Function: ttlv_put_byte
	Appends a data byte to the sequence of outgoing bytes in the transmit buffer.
	If a message is being transmitted in streaming mode, the appended byte will
	immediately be committed for transmission.
	
	Parameters:
		data - A data byte to append to the outgoing data.
	
	Returns:
		The number of bytes appended (1 if successful, 0 if there was a problem).
*/
uint8_t ttlv_put_byte(uint8_t data);

/**
	Function: ttlv_put_bytes
	Appends data bytes to the sequence of outgoing bytes in the transmit buffer.
	If a message is being transmitted in streaming mode, the appended bytes will
	immediately be committed for transmission.
	
	Parameters:
		n - Number of data bytes to append to the outgoing data.
		data_p - Pointer to data bytes to append to the outgoing data.
	
	Returns:
		The number of bytes appended. If an error occurred or there was insufficient
		buffer space, this number will be less than *n*.
*/
uint8_t ttlv_put_bytes(uint8_t n, const uint8_t *data_p);

/**
	Function: ttlv_unput_bytes
	Removes uncommitted data bytes from the end of the sequence of outgoing bytes
	in the transmit buffer.
	
	NOTE: To remove all uncommitted bytes, give the argument <TTLV_XMIT_BFR_SIZE>.
	
	Parameters:
		n - Maximum number of data bytes to remove from the outgoing data.
	
	Returns:
		The number of bytes removed.
*/
uint8_t ttlv_unput_bytes(uint8_t n);

/**
	Function: ttlv_try_put_bytes
	Attempts to append all the specified data bytes to the sequence of outgoing
	bytes in the transmit buffer. If a message is being transmitted in streaming
	mode, the appended bytes will immediately be committed for transmission.
	If this function cannot append all the specified bytes, it will remove ALL
	uncommitted data bytes from the transmit buffer.
	
	NOTE: This is intended to work as a fire-and-forget output function that will
	either successfully deliver data bytes for transmission or leave the transmit
	buffer cleared and ready to begin transmission of another, unrelated message.
	
	Parameters:
		n - Number of data bytes to append to the outgoing data.
		data_p - Pointer to data bytes to append to the outgoing data.
	
	Returns:
		A true value iff all the specified bytes were appended to the outgoing data.
*/
uint8_t ttlv_try_put_bytes(uint8_t n, const uint8_t *data_p);

/**
	Function: ttlv_begin_xmit
	Initiates message transmission. Message headers will be copied into the transmit
	buffer from the <ttlv_xmit_inm_header> and <ttlv_xmit_header> structs. Any
	uncommitted data bytes in the transmit buffer will be committed for transmission.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_begin_xmit(void);

/**
	Function: ttlv_try_begin_xmit
	Initiates message transmission or clears the transmit buffer. Message headers
	will be copied into the transmit buffer from the <ttlv_xmit_inm_header> and
	<ttlv_xmit_header> structs. Any uncommitted data bytes in the transmit buffer
	will either be committed for transmission or, if the attempt to initiate
	transmission fails, removed from the buffer.
	
	NOTE: This is intended to work as a fire-and-forget output function that will
	either successfully initiate message transmission or leave the transmit buffer
	cleared and ready to begin transmission of another, unrelated message.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_try_begin_xmit(void);

/**
	Function: ttlv_xmit
	Initiates transmission of a specified message or clears the transmit buffer.
	The header field arguments will be used to update <ttlv_xmit_inm_header> and
	<ttlv_xmit_header>, which will then be copied into the transmit buffer. The
	specified data bytes will be copied into the transmit buffer and then either
	committed for transmission or, if the attempt to initiate transmission fails,
	removed from the buffer.
	
	NOTE: This is intended to work as a fire-and-forget output function that will
	either successfully initiate transmission of a complete message or leave the
	transmit buffer cleared and ready to begin transmission of another, unrelated
	message.
	
	Parameters:
		dstadr - INM destination address of the message to transmit. If not in INM
			mode, the <ttlv_xmit_inm_header> field will still be updated.
		type - TLV type identifier of the message to transmit.
		length - TLV length of the message to transmit.
		data_p - Pointer to the data bytes (TLV value) of the message to transmit.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_xmit(uint8_t dstadr, uint8_t type, uint8_t length, const uint8_t *data_p);

/**
	Function: ttlv_get_bytes
	Retrieves data bytes from the start of the sequence of incoming bytes in
	the receive buffer. Each retrieved byte will be removed from the buffer.
	
	Parameters:
		n - Maximum number of data bytes to retrieve.
		data_p - Pointer to memory to copy the retrieved data bytes into.
	
	Returns:
		The number of bytes retrieved. If an error occurred or there was an
		insufficient number of bytes available in the buffer, this number will be
		less than *n*.
*/
uint8_t ttlv_get_bytes(uint8_t n, uint8_t *data_p);

/**
	Function: ttlv_finish_recv
	Finishes message reception. If any of the data bytes of the message that was
	being received remain in the receive buffer, they will be removed from the
	buffer. When this function returns, the TTLV receiver will be ready to begin
	processing of the next incoming message.
	
	CAUTION: Once a complete message has been received, processing of incoming
	messages will be paused until this function or <ttlv_recv> is called by
	external code. This means that to prevent receive buffer overruns, programs
	that use the TTLV module MUST respond to each incoming message by calling
	one of those functions within a message reception interval, even if
	the received message itself is not of interest.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to finish reception failed.
*/
ttlv_state ttlv_finish_recv(void);

/**
	Function: ttlv_recv
	Retrieves the data bytes of a received message and finishes message reception.
	When this function returns, the TTLV receiver will be ready to begin processing
	of the next incoming message.
	
	CAUTION: Once a complete message has been received, processing of incoming
	messages will be paused until this function or <ttlv_finish_recv> is called by
	external code. This means that to prevent receive buffer overruns, programs
	that use the TTLV module MUST respond to each incoming message by calling
	one of those functions within a message reception interval, even if
	the received message itself is not of interest.
	
	Parameters:
		data_p - Pointer to memory to copy the data bytes of the received message
			into. The specified block of memory MUST be large enough to store all the
			data bytes of the received message (as indicated by the *length* field
			of the <ttlv_recv_header> struct).
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to retrieve data bytes and finish reception failed.
*/
ttlv_state ttlv_recv(uint8_t *data_p);

/**
	Function: ttlv_shutdown
	Shuts down the TTLV communication module. Erases all current transmit
	and receive operations, aborting them first if they are still pending.
*/
void ttlv_shutdown(void);

#endif
