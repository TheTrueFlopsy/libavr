#ifndef AVR_STD_TLV_H
#define AVR_STD_TLV_H

#include <stdint.h>

#include "task_tlv.h"

/**
	File: std_tlv.h
	Common message types, result codes and payload data types for the Task-based
	TLV/INM (TTLV) communication module (<task_tlv.h>). Also declares a standard
	set of logical registers for TLV/INM nodes, as well as helper macros that
	simplify implementation of the logical register interface. Finally, there are
	some convenience functions for transmission of INM response messages.
	
	CAUTION: The current logical register specification has bothersome and
	unnecessary limitations (e.g. all registers being one byte large) and is
	therefore being considered for compatibility-breaking replacement in a future
	release of libavr.
*/


/// Section: Message Types

/**
	Enum: TTLV Message Type Identifiers
	
	NOTE: Application-specific message type identifiers SHOULD be greater than
	or equal to *TTLV_MSG_T_APPLICATION*.
	
	TTLV_MSG_T_DEFAULT              - Default message type.
	TTLV_MSG_T_RESULT               - Generic TLV operation response (result code).
	TTLV_MSG_T_INM_RESULT           - Generic INM operation response (result code and request ID).
	TTLV_MSG_T_REG_READ             - Logical register read request.
	TTLV_MSG_T_REG_READ_RES         - Logical TLV register read response.
	TTLV_MSG_T_INM_REG_READ_RES     - Logical INM register read response (with request ID).
	TTLV_MSG_T_REG_WRITE            - Logical register write (*R* = *V*) request.
	TTLV_MSG_T_REG_TOGGLE           - Logical register toggle (*R* ^= *V*; ret *R*) request.
	TTLV_MSG_T_REG_RW_EXCH          - Logical register read-write (*X* = *R*; *R* = *V*; ret *X*) request.
	TTLV_MSG_T_REG_WR_EXCH          - Logical register write-read (*R* = *V*; ret *R*) request.
	TTLV_MSG_T_REGPAIR_READ         - Logical register pair read (ret *R*) request.
	TTLV_MSG_T_REGPAIR_READ_RES     - Logical TLV register pair read response.
	TTLV_MSG_T_INM_REGPAIR_READ_RES - Logical INM register pair read response (with request ID).
	TTLV_MSG_T_REGPAIR_WRITE        - Logical register pair write request.
	TTLV_MSG_T_REGPAIR_TOGGLE       - Logical register pair toggle request.
	TTLV_MSG_T_REGPAIR_RW_EXCH      - Logical register pair read-write request.
	TTLV_MSG_T_REGPAIR_WR_EXCH      - Logical register pair write-read request.
	TTLV_MSG_T_MEMMON_DATA          - Memory monitor notification. Variable-length message.
	TTLV_MSG_T_MEMMON_CTRL          - Memory monitor control request. Not implemented.
	TTLV_MSG_T_APPLICATION          - Start of application-specific identifier range.
*/
enum {
	TTLV_MSG_T_DEFAULT              = 0x00,
	TTLV_MSG_T_RESULT               = 0x01,
	TTLV_MSG_T_INM_RESULT           = 0x02,
	TTLV_MSG_T_REG_READ             = 0x03,
	TTLV_MSG_T_REG_READ_RES         = 0x04,
	TTLV_MSG_T_INM_REG_READ_RES     = 0x05,
	TTLV_MSG_T_REG_WRITE            = 0x06,
	TTLV_MSG_T_REG_TOGGLE           = 0x07,
	TTLV_MSG_T_REG_RW_EXCH          = 0x08,
	TTLV_MSG_T_REG_WR_EXCH          = 0x09,
	TTLV_MSG_T_REGPAIR_READ         = 0x0a,
	TTLV_MSG_T_REGPAIR_READ_RES     = 0x0b,
	TTLV_MSG_T_INM_REGPAIR_READ_RES = 0x0c,
	TTLV_MSG_T_REGPAIR_WRITE        = 0x0d,
	TTLV_MSG_T_REGPAIR_TOGGLE       = 0x0e,
	TTLV_MSG_T_REGPAIR_RW_EXCH      = 0x0f,
	TTLV_MSG_T_REGPAIR_WR_EXCH      = 0x10,
	TTLV_MSG_T_MEMMON_DATA          = 0x11,
	TTLV_MSG_T_MEMMON_CTRL          = 0x12,
	TTLV_MSG_T_APPLICATION          = 0x40
};


/// Section: Result Codes

/**
	Enum: TTLV Operation Result Codes
	
	NOTE: Application-specific result codes SHOULD be greater than
	or equal to *TTLV_RES_APPLICATION* and not equal to *TTLV_RES_NONE*.
	
	TTLV_RES_OK              - Request successfully handled.
	TTLV_RES_TYPE            - Error: Unrecognized message type.
	TTLV_RES_LENGTH          - Error: Invalid message length.
	TTLV_RES_REGISTER        - Error: Message specified unrecognized logical register.
	TTLV_RES_NOT_IMPLEMENTED - Error: Message recognized but cannot be handled.
	TTLV_RES_APPLICATION     - Start of application-specific result code range.
	TTLV_RES_NONE            - Invalid result code.
*/
enum {
	TTLV_RES_OK              = 0x00,
	TTLV_RES_TYPE            = 0x01,
	TTLV_RES_LENGTH          = 0x02,
	TTLV_RES_REGISTER        = 0x03,
	TTLV_RES_NOT_IMPLEMENTED = 0x3f,
	TTLV_RES_APPLICATION     = 0x40,
	TTLV_RES_NONE            = 0xff
};

/**
	Ref: ttlv_result
	The type of TTLV result codes.
*/
typedef uint8_t ttlv_result;


/**
	Struct: ttlv_msg_inm_result
	Specifies the format of a generic INM response message.
*/
typedef struct __attribute__ ((__packed__)) ttlv_msg_inm_result {
	/**
		Field: result_code
		Identifies the result of the requested operation.
	*/
	ttlv_result result_code;
	
	/**
		Field: request_id
		Identifies the request message.
	*/
	uint16_t request_id;
	
} ttlv_msg_inm_result;


/// Section: Logical Registers

enum {
	TTLV_REG_NULL = 0x00,
	TTLV_REG_APP_STATUS = 0x01, // Zero means normal status.
	TTLV_REG_DEBUG0 = 0x02, // For general debugging use.
	TTLV_REG_DEBUG1 = 0x03, // For general debugging use.
	TTLV_REG_DEBUG2 = 0x04, // For general debugging use.
	TTLV_REG_DEBUG3 = 0x05, // For general debugging use.
	TTLV_REG_DEBUG4 = 0x06, // For general debugging use.
	TTLV_REG_DEBUG5 = 0x07, // For general debugging use.
	TTLV_REG_DEBUG6 = 0x08, // For general debugging use.
	TTLV_REG_DEBUG7 = 0x09, // For general debugging use.
	TTLV_REG_DEBUG8 = 0x0a, // For general debugging use.
	TTLV_REG_DEBUG9 = 0x0b, // For general debugging use.
	TTLV_REG_INDEX = 0x0c, // Low index byte. Write before accessing array register.
	TTLV_REG_INDEX_L = TTLV_REG_INDEX, // Low index byte. Write before accessing array register.
	TTLV_REG_INDEX_H = 0x0d, // High index byte. Write before accessing (large) array register.
	TTLV_REGPAIR_INDEX = TTLV_REG_INDEX_L, // Index register pair.
	TTLV_REG_STDIN = 0x0e, // Input stream register.
	TTLV_REG_STDOUT = 0x0f, // Output stream register.
	TTLV_REG_STDERR = 0x10, // Error/debug output stream register.
	TTLV_REG_STDIO = 0x11, // I/O stream register (for exchange operations).
	TTLV_REG_STDIO_L = TTLV_REG_STDIO, // I/O stream register (for exchange operations).
	TTLV_REG_STDIO_H = 0x12, // I/O stream register (for exchange operations).
	TTLV_REGPAIR_STDIO = TTLV_REG_STDIO_L, // I/O stream register pair (for exchange operations).
	TTLV_REG_STDIOERR = 0x13, // Error/debug I/O stream register (for exchange operations).
	TTLV_REG_STDIOERR_L = TTLV_REG_STDIOERR, // Error/debug I/O stream register (for exchange operations).
	TTLV_REG_STDIOERR_H = 0x14, // Error/debug I/O stream register (for exchange operations).
	TTLV_REGPAIR_STDIOERR = TTLV_REG_STDIOERR_L,
	TTLV_REG_FWID = 0x15, // Firmware ID, low byte.
	TTLV_REG_FWID_L = TTLV_REG_FWID, // Firmware ID, low byte.
	TTLV_REG_FWID_H = 0x16, // Firmware ID, high byte.
	TTLV_REGPAIR_FWID = TTLV_REG_FWID_L, // Firmware ID register pair.
	TTLV_REG_FWVERSION = 0x17, // Firmware version.
	TTLV_REG_APPLICATION = 0x40
};

/**
	Ref: ttlv_reg_index
	The type of TTLV logical register identifiers.
	
	NOTE: These identifiers do NOT necessarily correspond to ATmega RAM
	or I/O addresses. Their meaning is application-specific.
*/
typedef uint8_t ttlv_reg_index;

/**
	Ref: ttlv_reg_value
	The type of TTLV logical register values.
*/
typedef uint8_t ttlv_reg_value;

/**
	Ref: ttlv_regpair_value
	The type of TTLV logical register pair values.
*/
typedef uint16_t ttlv_regpair_value;

/**
	Struct: ttlv_msg_reg
	Specifies the format of an identifier-value pair for a
	logical register. Such pairs are the payloads of register
	write requests and register read responses.
*/
typedef struct __attribute__ ((__packed__)) ttlv_msg_reg {
	/**
		Field: index
		Register identifier.
	*/
	ttlv_reg_index index;  // ISSUE: Isn't this redundant for read responses?
	
	/**
		Field: value
		Register value.
	*/
	ttlv_reg_value value;
	
} ttlv_msg_reg;

/**
	Struct: ttlv_msg_inm_reg
	Specifies the format of an identifier-value pair for a logical register,
	with an INM message ID appended. Such triples are the payloads of register read
	responses in INM applications.
*/
typedef struct __attribute__ ((__packed__)) ttlv_msg_inm_reg {
	/**
		Field: index
		Register identifier.
	*/
	ttlv_reg_index index;  // ISSUE: Isn't this redundant, especially with request ID in the response?
	
	/**
		Field: value
		Register value.
	*/
	ttlv_reg_value value;
	
	/**
		Field: request_id
		Identifies the request message.
	*/
	uint16_t request_id;
	
} ttlv_msg_inm_reg;

/**
	Struct: ttlv_msg_regpair
	Specifies the format of an identifier-value pair for a
	logical register pair. Such pairs are the payloads of register
	pair write requests and register pair read responses.
*/
typedef struct __attribute__ ((__packed__)) ttlv_msg_regpair {
	/**
		Field: index
		Register identifier.
	*/
	ttlv_reg_index index;  // ISSUE: Isn't this redundant for read responses?
	
	/**
		Field: value
		Register pair value. The value of the lower register (i.e.
		the one with the lesser register identifier) is in the least
		significant byte and the value of the higher register is in
		the most significant byte.
	*/
	ttlv_regpair_value value;
	
} ttlv_msg_regpair;

/**
	Struct: ttlv_msg_inm_regpair
	Specifies the format of an identifier-value pair for a logical register pair,
	with an INM message ID appended. Such triples are the payloads of register pair read
	responses in INM applications.
*/
typedef struct __attribute__ ((__packed__)) ttlv_msg_inm_regpair {
	/**
		Field: index
		Register identifier.
	*/
	ttlv_reg_index index;  // ISSUE: Isn't this redundant, especially with request ID in the response?
	
	/**
		Field: value
		Register pair value. The value of the lower register (i.e.
		the one with the lesser register identifier) is in the least
		significant byte and the value of the higher register is in
		the most significant byte.
	*/
	ttlv_regpair_value value;
	
	/**
		Field: request_id
		Identifies the request message.
	*/
	uint16_t request_id;
	
} ttlv_msg_inm_regpair;


/// Section: Message Lengths and Type Checking

/**
	Enum: Standard TTLV Message Lengths
	
	TTLV_MSG_L_RESULT               - TLV length of a <TTLV_MSG_T_RESULT> message.
	TTLV_MSG_L_INM_RESULT           - TLV length of a <TTLV_MSG_T_INM_RESULT> message.
	TTLV_MSG_L_REG_READ             - TLV length of a <TTLV_MSG_T_REG_READ> message.
	TTLV_MSG_L_REG_READ_RES         - TLV length of a <TTLV_MSG_T_REG_READ_RES> message.
	TTLV_MSG_L_INM_REG_READ_RES     - TLV length of a <TTLV_MSG_T_INM_REG_READ_RES> message.
	TTLV_MSG_L_REG_WRITE            - TLV length of a <TTLV_MSG_T_REG_WRITE> message.
	TTLV_MSG_L_REG_TOGGLE           - TLV length of a <TTLV_MSG_T_REG_TOGGLE> message.
	TTLV_MSG_L_REG_RW_EXCH          - TLV length of a <TTLV_MSG_T_REG_RW_EXCH> message.
	TTLV_MSG_L_REG_WR_EXCH          - TLV length of a <TTLV_MSG_T_REG_WR_EXCH> message.
	TTLV_MSG_L_REGPAIR_READ         - TLV length of a <TTLV_MSG_T_REGPAIR_READ> message.
	TTLV_MSG_L_REGPAIR_READ_RES     - TLV length of a <TTLV_MSG_T_REGPAIR_READ_RES> message.
	TTLV_MSG_L_INM_REGPAIR_READ_RES - TLV length of a <TTLV_MSG_T_INM_REGPAIR_READ_RES> message.
	TTLV_MSG_L_REGPAIR_WRITE        - TLV length of a <TTLV_MSG_T_REGPAIR_WRITE> message.
	TTLV_MSG_L_REGPAIR_TOGGLE       - TLV length of a <TTLV_MSG_T_REGPAIR_TOGGLE> message.
	TTLV_MSG_L_REGPAIR_RW_EXCH      - TLV length of a <TTLV_MSG_T_REGPAIR_RW_EXCH> message.
	TTLV_MSG_L_REGPAIR_WR_EXCH      - TLV length of a <TTLV_MSG_T_REGPAIR_WR_EXCH> message.
*/
enum {
	TTLV_MSG_L_RESULT               = sizeof(ttlv_result),
	TTLV_MSG_L_INM_RESULT           = sizeof(ttlv_msg_inm_result),
	TTLV_MSG_L_REG_READ             = sizeof(ttlv_reg_index),
	TTLV_MSG_L_REG_READ_RES         = sizeof(ttlv_msg_reg),
	TTLV_MSG_L_INM_REG_READ_RES     = sizeof(ttlv_msg_inm_reg),
	TTLV_MSG_L_REG_WRITE            = sizeof(ttlv_msg_reg),
	TTLV_MSG_L_REG_TOGGLE           = sizeof(ttlv_msg_reg),
	TTLV_MSG_L_REG_RW_EXCH          = sizeof(ttlv_msg_reg),
	TTLV_MSG_L_REG_WR_EXCH          = sizeof(ttlv_msg_reg),
	TTLV_MSG_L_REGPAIR_READ         = sizeof(ttlv_reg_index),
	TTLV_MSG_L_REGPAIR_READ_RES     = sizeof(ttlv_msg_regpair),
	TTLV_MSG_L_INM_REGPAIR_READ_RES = sizeof(ttlv_msg_inm_regpair),
	TTLV_MSG_L_REGPAIR_WRITE        = sizeof(ttlv_msg_regpair),
	TTLV_MSG_L_REGPAIR_TOGGLE       = sizeof(ttlv_msg_regpair),
	TTLV_MSG_L_REGPAIR_RW_EXCH      = sizeof(ttlv_msg_regpair),
	TTLV_MSG_L_REGPAIR_WR_EXCH      = sizeof(ttlv_msg_regpair)
};

/**
	Macro: TTLV_CHECK_RESULT
	True if and only if the current <ttlv_recv_header> matches a *RESULT*
	message with a <ttlv_result> payload. Expression macro.
*/
#define TTLV_CHECK_RESULT TTLV_CHECK_TL(TTLV_MSG_T_RESULT, TTLV_MSG_L_RESULT)

/**
	Macro: TTLV_CHECK_INM_RESULT
	True if and only if the current <ttlv_recv_header> matches an *INM_RESULT*
	message with a <ttlv_msg_inm_result> payload. Expression macro.
*/
#define TTLV_CHECK_INM_RESULT TTLV_CHECK_TL(TTLV_MSG_T_INM_RESULT, TTLV_MSG_L_INM_RESULT)

/**
	Macro: TTLV_CHECK_REG_READ
	True if and only if the current <ttlv_recv_header> matches a *REG_READ*
	message with a <ttlv_reg_index> payload. Expression macro.
*/
#define TTLV_CHECK_REG_READ TTLV_CHECK_TL(TTLV_MSG_T_REG_READ, TTLV_MSG_L_REG_READ)

/**
	Macro: TTLV_CHECK_REG_WRITE
	True if and only if the current <ttlv_recv_header> matches a *REG_WRITE*
	message with a <ttlv_msg_reg> payload. Expression macro.
*/
#define TTLV_CHECK_REG_WRITE TTLV_CHECK_TL(TTLV_MSG_T_REG_WRITE, TTLV_MSG_L_REG_WRITE)

/**
	Macro: TTLV_CHECK_REG_TOGGLE
	True if and only if the current <ttlv_recv_header> matches a *REG_TOGGLE*
	message with a <ttlv_msg_reg> payload. Expression macro.
*/
#define TTLV_CHECK_REG_TOGGLE TTLV_CHECK_TL(TTLV_MSG_T_REG_TOGGLE, TTLV_MSG_L_REG_TOGGLE)

/**
	Macro: TTLV_CHECK_REG_RW_EXCH
	True if and only if the current <ttlv_recv_header> matches a *REG_RW_EXCH*
	message with a <ttlv_msg_reg> payload. Expression macro.
*/
#define TTLV_CHECK_REG_RW_EXCH TTLV_CHECK_TL(TTLV_MSG_T_REG_RW_EXCH, TTLV_MSG_L_REG_RW_EXCH)

/**
	Macro: TTLV_CHECK_REG_WR_EXCH
	True if and only if the current <ttlv_recv_header> matches a *REG_WR_EXCH*
	message with a <ttlv_msg_reg> payload. Expression macro.
*/
#define TTLV_CHECK_REG_WR_EXCH TTLV_CHECK_TL(TTLV_MSG_T_REG_WR_EXCH, TTLV_MSG_L_REG_WR_EXCH)

/**
	Macro: TTLV_CHECK_REGPAIR_READ
	True if and only if the current <ttlv_recv_header> matches a *REGPAIR_READ*
	message with a <ttlv_reg_index> payload. Expression macro.
*/
#define TTLV_CHECK_REGPAIR_READ TTLV_CHECK_TL(TTLV_MSG_T_REGPAIR_READ, TTLV_MSG_L_REGPAIR_READ)

/**
	Macro: TTLV_CHECK_REGPAIR_WRITE
	True if and only if the current <ttlv_recv_header> matches a *REGPAIR_WRITE*
	message with a <ttlv_msg_regpair> payload. Expression macro.
*/
#define TTLV_CHECK_REGPAIR_WRITE TTLV_CHECK_TL(TTLV_MSG_T_REGPAIR_WRITE, TTLV_MSG_L_REGPAIR_WRITE)

/**
	Macro: TTLV_CHECK_REGPAIR_TOGGLE
	True if and only if the current <ttlv_recv_header> matches a *REGPAIR_TOGGLE*
	message with a <ttlv_msg_regpair> payload. Expression macro.
*/
#define TTLV_CHECK_REGPAIR_TOGGLE TTLV_CHECK_TL(TTLV_MSG_T_REGPAIR_TOGGLE, TTLV_MSG_L_REGPAIR_TOGGLE)

/**
	Macro: TTLV_CHECK_REGPAIR_RW_EXCH
	True if and only if the current <ttlv_recv_header> matches a *REGPAIR_RW_EXCH*
	message with a <ttlv_msg_regpair> payload. Expression macro.
*/
#define TTLV_CHECK_REGPAIR_RW_EXCH TTLV_CHECK_TL(TTLV_MSG_T_REGPAIR_RW_EXCH, TTLV_MSG_L_REGPAIR_RW_EXCH)

/**
	Macro: TTLV_CHECK_REGPAIR_WR_EXCH
	True if and only if the current <ttlv_recv_header> matches a *REGPAIR_WR_EXCH*
	message with a <ttlv_msg_regpair> payload. Expression macro.
*/
#define TTLV_CHECK_REGPAIR_WR_EXCH TTLV_CHECK_TL(TTLV_MSG_T_REGPAIR_WR_EXCH, TTLV_MSG_L_REGPAIR_WR_EXCH)


/// Section: Function Definition Macros

// NOTE: These macros let a TLV application provide full logical register support
//       by creating application-specific implementations of the READ_REG and
//       WRITE_REG operations. The other register (and register pair) operations
//       can then be defined declaratively in terms of READ_REG and WRITE_REG.

/**
	Function: ttlv_reg_setter
	The type of pointers to functions that set the value of a specified logical
	register.
	
	Parameters:
		index - Identifier of the target logical register.
		value - The new register value.
	
	Returns:
		A <ttlv_result> indicating the outcome of the attempted register operation.
*/
typedef ttlv_result (*ttlv_reg_setter)(ttlv_reg_index index, ttlv_reg_value value);

/**
	Function: ttlv_reg_manipulator
	The type of pointers to functions that perform some update operation
	on a specified logical register.
	
	Parameters:
		index - Identifier of the target logical register.
		value_p - Pointer to a register value. Used both to receive the input value
			and to return the output value.
	
	Returns:
		A <ttlv_result> indicating the outcome of the attempted register operation.
*/
typedef ttlv_result (*ttlv_reg_manipulator)(ttlv_reg_index index, ttlv_reg_value *value_p);

/**
	Function: ttlv_regpair_setter
	The type of pointers to functions that set the value of a specified logical
	register pair.
	
	Parameters:
		index - Identifier of the lower half of the target logical register pair.
		value - The new register pair value.
	
	Returns:
		A <ttlv_result> indicating the outcome of the attempted register operation.
*/
typedef ttlv_result (*ttlv_regpair_setter)(ttlv_reg_index index, ttlv_regpair_value value);

/**
	Function: ttlv_regpair_manipulator
	The type of pointers to functions that perform some update operation
	on a specified logical register pair.
	
	Parameters:
		index - Identifier of the lower half of the target logical register pair.
		value_p - Pointer to a register pair value. Used both to receive the input value
			and to return the output value.
	
	Returns:
		A <ttlv_result> indicating the outcome of the attempted register operation.
*/
typedef ttlv_result (*ttlv_regpair_manipulator)(ttlv_reg_index index, ttlv_regpair_value *value_p);

/**
	Macro: TTLV_STD_REG_TOGGLE
	Defines a static function that uses specified logical register read and write
	functions to implement the register toggle operation (*R* ^= *V*; ret *R*).
	Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_reg_manipulator> that implements the read operation.
		WF - Name of a <ttlv_reg_setter> that implements the write operation.
		DF - Name of the emitted register toggle function.
	
	Returns:
		Emits a definition of a static <ttlv_reg_manipulator> function that
		implements the register toggle operation.
*/
#define TTLV_STD_REG_TOGGLE(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_reg_value *value_p) { \
		ttlv_reg_value tmp_value; \
		ttlv_result res = RF(index, &tmp_value); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		*value_p ^= tmp_value; \
		\
		return WF(index, *value_p); \
	}

/**
	Macro: TTLV_STD_REG_RW_EXCH
	Defines a static function that uses specified logical register read and write
	functions to implement the register read-write operation (*X* = *R*; *R* = *V*;
	ret *X*). Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_reg_manipulator> that implements the read operation.
		WF - Name of a <ttlv_reg_setter> that implements the write operation.
		DF - Name of the emitted register read-write function.
	
	Returns:
		Emits a definition of a <ttlv_reg_manipulator> function that implements
		the register read-write operation.
*/
#define TTLV_STD_REG_RW_EXCH(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_reg_value *value_p) { \
		ttlv_reg_value tmp_value; \
		ttlv_result res = RF(index, &tmp_value); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		res = WF(index, *value_p); \
		\
		*value_p = tmp_value; \
		return res; \
	}

/**
	Macro: TTLV_STD_REG_WR_EXCH
	Defines a static function that uses specified logical register read and write
	functions to implement the register write-read operation (*R* = *V*; ret *R*).
	Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_reg_manipulator> that implements the read operation.
		WF - Name of a <ttlv_reg_setter> that implements the write operation.
		DF - Name of the emitted register write-read function.
	
	Returns:
		Emits a definition of a static <ttlv_reg_manipulator> function that
		implements the register write-read operation.
*/
#define TTLV_STD_REG_WR_EXCH(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_reg_value *value_p) { \
		ttlv_result res = WF(index, *value_p); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		return RF(index, value_p); \
	}

/**
	Macro: TTLV_STD_REGPAIR_READ
	Defines a static function that uses a specified logical register read function
	to implement the register pair read operation (ret *R*).
	Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_reg_manipulator> that implements the read operation.
		DF - Name of the emitted register pair read function.
	
	Returns:
		Emits a definition of a static <ttlv_regpair_manipulator> function that
		implements the register pair read operation.
*/
#define TTLV_STD_REGPAIR_READ(RF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_regpair_value *value_p) { \
		ttlv_reg_value *value_byte_p = (ttlv_reg_value*)value_p; \
		\
		ttlv_result res = RF(index, value_byte_p); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		return RF(index+1, value_byte_p+1); \
	}

/**
	Macro: TTLV_STD_REGPAIR_WRITE
	Defines a static function that uses a specified logical register write function
	to implement the register pair write operation (*R* = *V*).
	Function definition macro.
	
	Parameters:
		WF - Name of a <ttlv_reg_setter> that implements the write operation.
		DF - Name of the emitted register pair write function.
	
	Returns:
		Emits a definition of a static <ttlv_regpair_setter> function that
		implements the register pair write operation.
*/
#define TTLV_STD_REGPAIR_WRITE(WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_regpair_value value) { \
		ttlv_reg_value *value_byte_p = (ttlv_reg_value*)&value; \
		\
		ttlv_result res = WF(index, *value_byte_p); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		return WF(index+1, *(value_byte_p+1)); \
	}

/**
	Macro: TTLV_STD_REGPAIR_TOGGLE
	Defines a static function that uses specified logical register pair read and
	write functions to implement the register pair toggle operation (*R* ^= *V*;
	ret *R*). Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_regpair_manipulator> that implements the read operation.
		WF - Name of a <ttlv_regpair_setter> that implements the write operation.
		DF - Name of the emitted register pair toggle function.
	
	Returns:
		Emits a definition of a static <ttlv_regpair_manipulator> function that
		implements the register pair toggle operation.
*/
#define TTLV_STD_REGPAIR_TOGGLE(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_regpair_value *value_p) { \
		ttlv_regpair_value tmp_value; \
		ttlv_result res = RF(index, &tmp_value); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		*value_p ^= tmp_value; \
		\
		return WF(index, *value_p); \
	}

/**
	Macro: TTLV_STD_REGPAIR_RW_EXCH
	Defines a static function that uses specified logical register pair read and
	write functions to implement the register pair read-write operation
	(*X* = *R*; *R* = *V*; ret *X*). Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_regpair_manipulator> that implements the read operation.
		WF - Name of a <ttlv_regpair_setter> that implements the write operation.
		DF - Name of the emitted register pair read-write function.
	
	Returns:
		Emits a definition of a static <ttlv_regpair_manipulator> function that
		implements the register pair read-write operation.
*/
#define TTLV_STD_REGPAIR_RW_EXCH(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_regpair_value *value_p) { \
		ttlv_regpair_value tmp_value; \
		ttlv_result res = RF(index, &tmp_value); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		res = WF(index, *value_p); \
		\
		*value_p = tmp_value; \
		return res; \
	}

/**
	Macro: TTLV_STD_REGPAIR_WR_EXCH
	Defines a static function that uses specified logical register pair read and
	write functions to implement the register pair write-read operation (*R* = *V*;
	ret *R*). Function definition macro.
	
	Parameters:
		RF - Name of a <ttlv_regpair_manipulator> that implements the read operation.
		WF - Name of a <ttlv_regpair_setter> that implements the write operation.
		DF - Name of the emitted register pair write-read function.
	
	Returns:
		Emits a definition of a static <ttlv_regpair_manipulator> function that
		implements the register pair write-read operation.
*/
#define TTLV_STD_REGPAIR_WR_EXCH(RF, WF, DF) \
	static ttlv_result DF(ttlv_reg_index index, ttlv_regpair_value *value_p) { \
		ttlv_result res = WF(index, *value_p); \
		if (res != TTLV_RES_OK) \
			return res; \
		\
		return RF(index, value_p); \
	}


/// Section: API Functions

/**
	Function: ttlv_xmit_response
	Convenience function that uses <ttlv_xmit> to transmit a message. The INM
	destination address will be copied from the *srcadr* field of
	<ttlv_recv_inm_header> (assumed to contain the address of the source
	of the most recently received request).
	
	Parameters:
		type - TLV type identifier of the message to transmit.
		length - TLV length of the message to transmit.
		data_p - Pointer to the data bytes (TLV value) of the message to transmit.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_xmit_response(uint8_t type, uint8_t length, const uint8_t *data_p);

/**
	Function: ttlv_xmit_result
	Convenience function that uses <ttlv_xmit> to transmit a <TTLV_MSG_T_RESULT>
	message. The INM destination address will be copied from the *srcadr*
	field of <ttlv_recv_inm_header> (assumed to contain the address of the source
	of the most recently received request).
	
	Parameters:
		res - The result code to send in the <TTLV_MSG_T_RESULT> message.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_xmit_result(ttlv_result res);

/**
	Function: ttlv_xmit_inm_result
	Convenience function that uses <ttlv_xmit> to transmit a <TTLV_MSG_T_INM_RESULT>
	message. The INM destination address and request ID in the transmitted message
	will be copied from the *srcadr* and *msg_id* fields of <ttlv_recv_inm_header>
	(assumed to contain the source address and message ID of the most recently
	received request).
	
	Parameters:
		res - The result code to send in the <TTLV_MSG_T_INM_RESULT> message.
	
	Returns:
		An integer result code, which will be an error code if and only if
		the attempt to initiate transmission failed.
*/
ttlv_state ttlv_xmit_inm_result(ttlv_result res);

#endif
