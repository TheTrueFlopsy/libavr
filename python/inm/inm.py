
import datetime
import enum
import inspect
import socket
import selectors

# IDEA: Make this import conditional? It's only needed for the SerialMessageChannel.
import serial

## File: inm.py
## The *inm* module provides a complete INM messaging API. This API is primarily
## designed for flexibility, not ease of use (or performance). When implementing
## an INM client, consider using an <InmHelper> to simplify the exchange of messages.
##
## CAUTION: This module is NOT inherently thread-safe. In a multi-threaded
## application, access to thread-shared objects from this module (including the
## <default_msg_factory>) MUST be synchronized in application code.


## Section: General Utilities

## Variable: ZERO_DURATION
## A *datetime.timedelta* representing a zero duration.
ZERO_DURATION = datetime.timedelta(0)

## Variable: get_timestamp
## Produces a *datetime.datetime* representing the current local time.
## Alias of the standard library method *datetime.datetime.now*.
get_timestamp = datetime.datetime.now

## Function: get_timedelta
## Produces a *datetime.timedelta* with a specified duration.
##
## Parameters:
##   secs - An integer or float representing a number of seconds.
##   musecs - An integer or float representing a number of microseconds.
##
## Returns:
##   A *datetime.timedelta* representing a duration of *(secs + musecs/1000000)*
##   seconds.
def get_timedelta(secs=0, musecs=0):
	return datetime.timedelta(seconds=secs, microseconds=musecs)

## Function: bytes_to_hex
## Produces a hexadecimal string representation of a bytes object.
##
## Parameters:
##   bs - A bytes object.
##   sep - Group separator. Either *None* or a one-character string.
##   bytes_per_sep - Group size.
##   big_endian - If true, the order of byte representations in the returned
##     string will be reversed relative to the order of bytes in *bs* (i.e.
##     the representation of *bs[0]* will be at the end of the string, etc.).
##
## Returns:
##   A hexadecimal string representation of *bs*. No prefix is included.
def bytes_to_hex(bs, sep=None, bytes_per_sep=1, big_endian=False):
	if big_endian:
		bs = bytes(reversed(bs))
	
	if sep is not None:
		return bs.hex(sep, bytes_per_sep)
	else:
		return bs.hex()

## Function: bytes_to_bin
## Produces a binary string representation of a bytes object.
##
## Parameters:
##   bs - A bytes object.
##   sep - Group separator. Either *None* or a one-character string.
##   bytes_per_sep - Group size.
##   big_endian - If true, the order of byte representations in the returned
##     string will be reversed relative to the order of bytes in *bs* (i.e.
##     the representation of *bs[0]* will be at the end of the string, etc.).
##
## Returns:
##   A binary string representation of *bs*. No prefix is included.
def bytes_to_bin(bs, sep=None, bytes_per_sep=1, big_endian=False):
	if big_endian:
		bs = bytes(reversed(bs))
	
	if sep is not None:
		groups = []
		g1_len = len(bs) % bytes_per_sep
		
		if g1_len > 0:  # Input length not divisible by group size.
			groups.append(bs[0:g1_len])  # Put incomplete group at the start, like *bytes.hex()* does.
		
		for i in range(g1_len, len(bs), bytes_per_sep):
			groups.append(bs[i:i+bytes_per_sep])
		
		return sep.join(''.join(f'{b:08b}' for b in g) for g in groups)
	else:
		return ''.join(f'{b:08b}' for b in bs)

## Function: map_enum
## Searches an enum class for a member with a specified integer value.
##
## Parameters:
##   enum_class - An enum class.
##   i - An integer.
##   defval - Default return value.
##
## Returns:
##   If a member with integer value *i* is found in *enum_class*, then
##   that member is returned. Otherwise, *defval* is returned.
def map_enum(enum_class, i, defval=None):
	for e in enum_class:
		if e.value == i:
			return e
	return defval

## Function: format_msg_info
## Provides a standard way to represent INM communication events as strings. This can
## be useful for logging and debugging.
##
## Parameters:
##   obj - An object describing some event of interest (e.g. a <Message>).
##   event - An optional string identifying the type of event (e.g. 'RECV').
##   header - An optional <MessageHeader>.
##   link_adr - An optional INM link address (e.g. an (ip_adr, udp_port) pair).
##   event_colw - Minimum width of the *event* column.
##   timestamp - If this parameter is true, a representation of the current time
##     will be included in the returned string. If this parameter is a nonempty string,
##     it will be used as the timestamp format string.
##
## Returns:
##   A string representation of the specified event, in the following format:
## > '{timestamp} {event} [{header.srcadr}.{header.msg_id} => {header.dstadr}]{link_adr}: {obj}'
def format_msg_info(obj, event=None, header=None, link_adr=None, event_colw=0, timestamp=False):
	if isinstance(link_adr, tuple) and len(link_adr) > 1 and isinstance(link_adr[0], MessageChannel):
		link_adr = link_adr[1]  # Extract channel-specific link address.
	
	timestamp_fmt = '%Y-%m-%dT%H:%M:%S.%f'
	if timestamp and isinstance(timestamp, str):
		timestamp_fmt = timestamp
	
	time_field = '' if not timestamp else f'{get_timestamp():{timestamp_fmt}} '
	event_field = '' if event is None else f'{event:>{event_colw:d}} '
	header_field = '' if header is None else f'[{header.srcadr}.{header.msg_id} => {header.dstadr}]'
	link_adr_field = '' if link_adr is None else link_adr
	
	return f'{time_field}{event_field}{header_field}{link_adr_field}: {str(obj)}'


## Section: Enum Types

## Enum: StandardTypes
## Standard INM message types. Message type identifiers are transmitted as 8-bit
## unsigned integers and MUST be specified as Python integers in the range 0-255
## (inclusive).
##
## NOTE: Application-specific message type identifiers SHOULD be greater than
## or equal to *APPLICATION* (i.e. 0x40/64).
##
## DEFAULT              - Default message type.
## RESULT               - Generic TLV operation response (<StandardResults> code).
## INM_RESULT           - Generic INM operation response (<StandardResults> code and request ID).
## REG_READ             - Logical register read request.
## REG_READ_RES         - Logical TLV register read response.
## INM_REG_READ_RES     - Logical INM register read response (with request ID).
## REG_WRITE            - Logical register write (*R* = *V*) request.
## REG_TOGGLE           - Logical register toggle (*R* ^= *V*; ret *R*) request.
## REG_RW_EXCH          - Logical register read-write (*X* = *R*; *R* = *V*; ret *X*) request.
## REG_WR_EXCH          - Logical register write-read (*R* = *V*; ret *R*) request.
## REGPAIR_READ         - Logical register pair read (ret *R*) request.
## REGPAIR_READ_RES     - Logical TLV register pair read response.
## INM_REGPAIR_READ_RES - Logical INM register pair read response (with request ID).
## REGPAIR_WRITE        - Logical register pair write request.
## REGPAIR_TOGGLE       - Logical register pair toggle request.
## REGPAIR_RW_EXCH      - Logical register pair read-write request.
## REGPAIR_WR_EXCH      - Logical register pair write-read request.
## MEMMON_DATA          - Memory monitor notification. Variable-length message.
## MEMMON_CTRL          - Memory monitor control request. Not implemented.
## APPLICATION          - Start of application-specific identifier range.
@enum.unique
class StandardTypes(enum.IntEnum):
	DEFAULT              = 0x00
	RESULT               = 0x01
	INM_RESULT           = 0x02
	REG_READ             = 0x03
	REG_READ_RES         = 0x04
	INM_REG_READ_RES     = 0x05
	REG_WRITE            = 0x06
	REG_TOGGLE           = 0x07
	REG_RW_EXCH          = 0x08
	REG_WR_EXCH          = 0x09
	REGPAIR_READ         = 0x0a
	REGPAIR_READ_RES     = 0x0b
	INM_REGPAIR_READ_RES = 0x0c
	REGPAIR_WRITE        = 0x0d
	REGPAIR_TOGGLE       = 0x0e
	REGPAIR_RW_EXCH      = 0x0f
	REGPAIR_WR_EXCH      = 0x10
	MEMMON_DATA          = 0x11
	MEMMON_CTRL          = 0x12
	APPLICATION          = 0x40

## Enum: StandardResults
## Standard INM result codes. Result codes are transmitted as 8-bit unsigned
## integers and MUST be specified as Python integers in the range 0-255
## (inclusive).
##
## NOTE: Application-specific result codes SHOULD be greater than or equal to
## *APPLICATION* (i.e. 0x40/64) and not equal to *NONE* (i.e. 0xff/255).
##
## OK              - Request successfully handled.
## TYPE            - Error: Unrecognized message type.
## LENGTH          - Error: Invalid message length.
## REGISTER        - Error: Message specified unrecognized logical register.
## NOT_IMPLEMENTED - Error: Message recognized but cannot be handled.
## APPLICATION     - Start of application-specific result code range.
## NONE            - Invalid result code.
@enum.unique
class StandardResults(enum.IntEnum):
	OK              = 0x00
	TYPE            = 0x01
	LENGTH          = 0x02
	REGISTER        = 0x03
	NOT_IMPLEMENTED = 0x3f
	APPLICATION     = 0x40
	NONE            = 0xff

## Enum: StandardRegisters
## Standard INM register identifiers. Identifiers are transmitted as 8-bit
## unsigned integers and MUST be specified as Python integers in the range 0-255
## (inclusive).
##
## NOTE: Application-specific register identifiers SHOULD be greater than or
## equal to *APPLICATION* (i.e. 0x40/64).
##
## NULL          - Null register. Yeah. (What was the idea here again?)
## APP_STATUS    - Application status. Zero means normal status, other values are
##                 application-specific.
## DEBUG[0-9]    - Registers for general temporary / debugging use.
## INDEX         - Low index byte. Used to specify an index in array registers.
## INDEX_L       - Alias of *INDEX*.
## INDEX_H       - High index byte. Used together with *INDEX_L* to specify an index
##                 in large array registers.
## PAIR_INDEX    - Alias of *INDEX_L*. Intended for register pair access.
## STDIN         - Standard input stream register.
## STDOUT        - Standard output stream register.
## STDERR        - Error/debug output stream register.
## STDIO         - Standard I/O stream register. Intended for exchange operations.
## STDIO_L       - Alias of *STDIO*.
## STDIO_H       - Upper half of 16-bit standard I/O stream register pair.
## PAIR_STDIO    - Alias of *STDIO_L*. Intended for register pair exchange operations.
## STDIOERR      - Error/debug I/O stream register. Intended for exchange operations.
## STDIOERR_L    - Alias of *STDIOERR*.
## STDIOERR_H    - Upper half of 16-bit error/debug I/O stream register pair.
## PAIR_STDIOERR - Alias of *STDIOERR_L*. Intended for register pair exchange operations.
## FWID          - Firmware ID, low byte.
## FWID_L        - Alias of *FWID*.
## FWID_H        - Firmware ID, high byte.
## PAIR_FWID     - Alias of *FWID_L*. Intended for register pair access.
## FWVERSION     - Firmware version.
## APPLICATION   - Start of application-specific register identifier range.
class StandardRegisters(enum.IntEnum):
	NULL          = 0x00
	APP_STATUS    = 0x01 # Zero means normal status.
	DEBUG0        = 0x02 # For general debugging use.
	DEBUG1        = 0x03 # For general debugging use.
	DEBUG2        = 0x04 # For general debugging use.
	DEBUG3        = 0x05 # For general debugging use.
	DEBUG4        = 0x06 # For general debugging use.
	DEBUG5        = 0x07 # For general debugging use.
	DEBUG6        = 0x08 # For general debugging use.
	DEBUG7        = 0x09 # For general debugging use.
	DEBUG8        = 0x0a # For general debugging use.
	DEBUG9        = 0x0b # For general debugging use.
	INDEX         = 0x0c # Low index byte. Write before accessing array register.
	INDEX_L       = INDEX # Low index byte. Write before accessing array register.
	INDEX_H       = 0x0d # High index byte. Write before accessing (large) array register.
	PAIR_INDEX    = INDEX_L # Index register pair.
	STDIN         = 0x0e # Input stream register.
	STDOUT        = 0x0f # Output stream register.
	STDERR        = 0x10 # Error/debug output stream register.
	STDIO         = 0x11 # I/O stream register (for exchange operations).
	STDIO_L       = STDIO # I/O stream register (for exchange operations).
	STDIO_H       = 0x12 # I/O stream register (for exchange operations).
	PAIR_STDIO    = STDIO_L # I/O stream register pair (for exchange operations).
	STDIOERR      = 0x13 # Error/debug I/O stream register (for exchange operations).
	STDIOERR_L    = STDIOERR # Error/debug I/O stream register (for exchange operations).
	STDIOERR_H    = 0x14 # Error/debug I/O stream register (for exchange operations).
	PAIR_STDIOERR = STDIOERR_L # Error/debug I/O stream register pair (for exchange operations).
	FWID          = 0x15 # Firmware ID, low byte.
	FWID_L        = FWID # Firmware ID, low byte.
	FWID_H        = 0x16 # Firmware ID, high byte.
	PAIR_FWID     = FWID_L # Firmware ID register pair.
	FWVERSION     = 0x17 # Firmware version.
	APPLICATION   = 0x40

## Enum: ResultCode
## Result codes for the INM module.
##
## SUCCESS          - Operation successfully completed.
## RECV_TIMEOUT     - Receive operation timed out.
## RECV_FAILURE     - Receive operation failed in underlying API.
## UNROUTABLE       - Message could not be routed toward destination.
## LINK_FAILURE     - Error in underlying API.
## CHANNEL_FAILURE  - Error in underlying message channel.
## INVALID_HEADER   - Invalid message header received or submitted for sending.
## INVALID_MESSAGE  - Invalid message received or submitted for sending.
## INVALID_ARGUMENT - Invalid argument in module API function call.
## INVALID_STATE    - Module API function call made in wrong object state.
@enum.unique
class ResultCode(enum.IntEnum):
	SUCCESS          = 0
	RECV_TIMEOUT     = 1
	RECV_FAILURE     = 2
	UNROUTABLE       = 3
	LINK_FAILURE     = 4
	CHANNEL_FAILURE  = 5
	INVALID_HEADER   = 6
	INVALID_MESSAGE  = 7
	INVALID_ARGUMENT = 8
	INVALID_STATE    = 9

## Enum: ValueConversions
## Type conversion specifiers for INM message values.
##
## NoConv - No conversion / unspecified conversion
## Hex    - Hexadecimal string (unprefixed)
## Bin    - Binary string (unprefixed)
## Int    - Integer
## Str    - Decoded text string
## Tuple  - Tuple of byte values (as integers)
## List   - List of byte values (as integers)
## Bytes  - Bytes object
@enum.unique
class ValueConversions(enum.Enum):
	NoConv = enum.auto()
	Hex    = enum.auto()
	Bin    = enum.auto()
	Int    = enum.auto()
	Str    = enum.auto()
	Tuple  = enum.auto()
	List   = enum.auto()
	Bytes  = enum.auto()

## Enum: Strictness
## Multipart formatting strictness specifiers for INM message values.
##
## Anything   - Any value whatsoever is accepted.
## AllWanted  - All requested fields must be present.
## OnlyWanted - All requested fields must be present, and only those.
## Exact      - All requested fields must be present, and only those,
##              and no unparsed bytes are allowed to remain.
@enum.unique
class Strictness(enum.IntEnum):
	Anything   = 0
	AllWanted  = 1
	OnlyWanted = 2
	Exact      = 3


## Section: Messages and Message Processing

## Class: Message
## Instances of subclasses of this abstract class represent INM messages.
## This class provides methods that inspect and format the content of
## a message in various ways.
class Message:
	## Variable: MESSAGE_TYP_SIZE
	## Size in bytes of the message type identifier field in an INM message.
	MESSAGE_TYP_SIZE = 1
	
	@classmethod
	def _pretty_typ(cls, f_typ):
		if isinstance(f_typ, enum.Enum):
			f_typ = f_typ.name
		elif isinstance(f_typ, int):
			f_typ = f'{f_typ:#04x}'
		
		return f_typ
	
	@classmethod
	def _pretty_val(cls, f_val):
		if isinstance(f_val, enum.Enum):
			f_val = f_val.name
		elif isinstance(f_val, tuple):
			f_val = f'({", ".join(str(cls._pretty_val(v)) for v in f_val)})'
		
		return f_val
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   typ - Message type identifier, e.g. one of the <StandardTypes>.
	##   val - Message value/payload. MUST be a bytes object.
	def __init__(self, typ, val):
		if not (self.MIN_TYPE_NUM <= typ <= self.MAX_TYPE_NUM):
			raise ValueError()
		
		if len(val) > self.MAX_MESSAGE_LEN:
			raise ValueError()
		
		## Property: typ
		## Message type identifier.
		self.typ = typ
		
		## Property: val
		## Message value/payload.
		self.val = val
	
	## Method: __len__
	## Message (value) length.
	##
	## NOTE: The return value does NOT include the size of the message type and
	## length fields that are part of the standard binary on-wire representation
	## of an INM message.
	##
	## Returns:
	##   The INM message length, determined by the *len()* of the <val> attribute.
	def __len__(self):
		return len(self.val)
	
	## Method: __str__
	## String conversion.
	##
	## Returns:
	##   A string representation of the *Message*, in the format "Message(typ, len, val)".
	def __str__(self):
		length = len(self.val)
		f_typ, f_val = self.format_str()
		
		f_typ = self._pretty_typ(f_typ)
		f_val = self._pretty_val(f_val)
		
		return f'Message({f_typ}, {length}, {f_val})'
	
	## Method: __repr__
	## String representation.
	##
	## Returns:
	##   A string representation of the *Message*, in the format "<Message(typ, len, val)>".
	def __repr__(self):
		return f'<{str(self)}>'
	
	## Method: check_tl
	## Message type and length check predicate.
	##
	## Parameters:
	##   typ - Required message type identifier.
	##   length - Required minimum message length.
	##
	## Returns:
	##   True if and only if the <typ> attribute is equal to the *typ* argument
	##   and *len(self)* is greater than or equal to the *length* argument.
	def check_tl(self, typ, length):
		return self.typ == typ and len(self) >= length
	
	## Method: format_str
	## Applies string formatting to the INM message type and value.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##
	## Returns:
	##   f_typ - Formatted message type.
	##   f_val - String formatted message value.
	def format_str(self, formatter=None):
		return self.format_type(formatter), self.format_val_str(formatter)
	
	## Method: format
	## Applies formatting to the INM message type and value.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##   conv - Message value conversion specifier (e.g. from <ValueConversions>), or *None*
	##     to apply the default conversion.
	##
	## Returns:
	##   f_typ - Formatted message type.
	##   f_val - Formatted message value.
	def format(self, formatter=None, conv=None):
		return self.format_type(formatter), self.format_val(formatter, conv)
	
	## Method: format_type
	## Applies formatting to the INM message type.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##
	## Returns:
	##   Formatted message type.
	def format_type(self, formatter=None):
		if formatter is None:
			formatter = default_msg_factory
		return map_enum(formatter.type_enum_class, self.typ, self.typ)
	
	## Method: format_val_str
	## Applies string formatting to the INM message value.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##
	## Returns:
	##   String formatted message value.
	def format_val_str(self, formatter=None):
		if formatter is None:
			formatter = default_msg_factory
		f_typ = self.format_type(formatter)
		return formatter.format_val_str(self.val, f_typ)
	
	## Method: format_val
	## Applies formatting to the INM message value.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##   conv - Message value conversion specifier (e.g. from <ValueConversions>), or *None*
	##     to apply the default conversion.
	##
	## Returns:
	##   Formatted message value.
	def format_val(self, formatter=None, conv=None):
		if formatter is None:
			formatter = default_msg_factory
		f_typ = self.format_type(formatter)
		return formatter.format_val(self.val, conv, f_typ)
	
	## Method: format_mval
	## Applies multipart formatting to the INM message value.
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##   conv - Field value conversion specifiers (e.g. from <ValueConversions>).
	##     May be a scalar (to apply the same conversion to all fields), tuple or list.
	##     If this is *None*, default conversions are applied.
	##   size - Multipart value field sizes. May be a scalar (if all fields have the same size),
	##     tuple or list. If this is *None*, the message type is used to infer field sizes, if
	##     possible.
	##   n_vals - Multipart value field count. If this is *None*, the field count is inferred
	##     from the *conv* and *size* arguments (or their default values).
	##   strict - Multipart formatting <Strictness> specifier, or *None* to apply the default
	##     strictness level.
	##
	## Returns:
	##   The formatted multipart message value (list of formatted fields), if formatting was
	##   successful. Otherwise *None*.
	def format_mval(self, formatter=None, conv=None, size=None, n_vals=None, strict=None):
		if formatter is None:
			formatter = default_msg_factory
		f_typ = self.format_type(formatter)
		return formatter.format_mval(self.val, conv, size, f_typ, n_vals, strict)
	
	## Method: format_mval_0
	## Applies multipart formatting to the INM message value, but returns only the first field.
	## (I.e. return the field at index 0 in the list of formatted fields).
	##
	## Parameters:
	##   formatter - The message formatter to use, or *None* to use the default formatter.
	##   conv - Field value conversion specifiers (e.g. from <ValueConversions>).
	##     May be a scalar (to apply the same conversion to all fields), tuple or list.
	##     If this is *None*, default conversions are applied.
	##   size - Multipart value field sizes. May be a scalar (if all fields have the same size),
	##     tuple or list. If this is *None*, the message type is used to infer field sizes, if
	##     possible.
	##   n_vals - Multipart value field count. If this is *None*, the field count is inferred
	##     from the *conv* and *size* arguments (or their default values).
	##   strict - Multipart formatting <Strictness> specifier, or *None* to apply the default
	##     strictness level.
	##
	## Returns:
	##   The first field of the formatted multipart message value, if at least one field
	##   was successfully formatted. Otherwise *None*.
	def format_mval_0(self, formatter=None, conv=None, size=None, n_vals=None, strict=None):
		mval = self.format_mval(formatter, conv, size, n_vals, strict)
		return mval[0] if mval is not None and len(mval) >= 1 else None
	
	## Method: to_bytes
	## Converts the *Message* to a bytes object containing the message in standard binary
	## on-wire format.
	##
	## Parameters:
	##   formatter - Formatter to use for message type and length conversion, or *None*
	##     to use the default formatter.
	##
	## Returns:
	##   A bytes object containing the INM message represented by this *Message*
	##   in standard binary on-wire format.
	def to_bytes(self, formatter=None):
		if formatter is None:
			formatter = default_msg_factory
		
		typ_b = formatter.make_val(self.typ, int_size=1)
		len_b = formatter.make_val(len(self.val), int_size=self.MESSAGE_LEN_SIZE)
		val_b = bytes(self.val)
		return typ_b + len_b + val_b

## Class: StandardMessage
## Concrete <Message> subclass for standard-size INM messages.
## The maximum size of a standard message value/payload is 255 bytes.
class StandardMessage(Message):
	## Variable: MESSAGE_LEN_SIZE
	## Size in bytes of the message length field in a standard INM message.
	MESSAGE_LEN_SIZE = 1
	
	## Variable: MIN_MESSAGE_SIZE
	## Minimum size in bytes of a standard INM message.
	MIN_MESSAGE_SIZE = Message.MESSAGE_TYP_SIZE + MESSAGE_LEN_SIZE
	
	## Variable: MIN_TYPE_NUM
	## Smallest valid message type identifier for standard INM messages.
	MIN_TYPE_NUM = 0x00
	
	## Variable: MAX_TYPE_NUM
	## Largest valid message type identifier for standard INM messages.
	MAX_TYPE_NUM = 0x7f
	
	## Variable: MAX_MESSAGE_LEN
	## Maximum value of the message length field in a standard INM message,
	## and thereby also the maximum size in bytes of a standard message value
	## field.
	MAX_MESSAGE_LEN = 0xff
	
	## Variable: PROTOCOL_TYPE_NUM
	## Type identifier reserved for INM protocol messages.
	PROTOCOL_TYPE_NUM = MAX_TYPE_NUM

## Class: LargeMessage
## Concrete <Message> subclass for large-size INM messages.
## The maximum size of a large message value/payload is 4294967295 bytes.
class LargeMessage(Message):
	## Variable: MESSAGE_LEN_SIZE
	## Size in bytes of the message length field in a large INM message.
	MESSAGE_LEN_SIZE = 4
	
	## Variable: MIN_MESSAGE_SIZE
	## Minimum size in bytes of a large INM message.
	MIN_MESSAGE_SIZE = Message.MESSAGE_TYP_SIZE + MESSAGE_LEN_SIZE
	
	## Variable: MIN_TYPE_NUM
	## Smallest valid message type identifier for large INM messages.
	MIN_TYPE_NUM = 0x80
	
	## Variable: MAX_TYPE_NUM
	## Largest valid message type identifier for large INM messages.
	MAX_TYPE_NUM = 0xff
	
	## Variable: MAX_MESSAGE_LEN
	## Maximum value of the message length field in a large INM message,
	## and thereby also the maximum size in bytes of a large message value
	## field.
	MAX_MESSAGE_LEN = 0xffffffff


## Class: MessageFactory
## A configurable factory class for INM messages. Provides various methods
## that construct and parse <Message> objects and INM message payloads.
class MessageFactory:
	
	## Variable: MAKE_VAL_ATTR_NAME
	## Name of attribute to use for custom conversion of objects into INM message
	## values/payloads. If present, such an attribute MUST be callable, return
	## a bytes object and accept the following positional arguments:
	##
	##   formatter - A formatter object that provides the *MessageFactory* API.
	##   int_size - The requested size in bytes of a message value or field produced
	##     from an integer. May be *None*.
	##   tlv_type - An INM message type specifier. May be *None*.
	MAKE_VAL_ATTR_NAME = 'to_inm_val'
	
	## Variable: DEFAULT_RES_ENUM_MSG_TYPES
	## Default message types for result code enum conversion.
	DEFAULT_RES_ENUM_MSG_TYPES = (StandardTypes.RESULT, StandardTypes.INM_RESULT)
	
	## Variable: DEFAULT_REG_ENUM_MSG_TYPES
	## Default message types for register identifier enum conversion.
	DEFAULT_REG_ENUM_MSG_TYPES = (
		StandardTypes.REG_READ,
		StandardTypes.REG_READ_RES,
		StandardTypes.INM_REG_READ_RES,
		StandardTypes.REG_WRITE,
		StandardTypes.REG_TOGGLE,
		StandardTypes.REG_RW_EXCH,
		StandardTypes.REG_WR_EXCH,
		StandardTypes.REGPAIR_READ,
		StandardTypes.REGPAIR_READ_RES,
		StandardTypes.INM_REGPAIR_READ_RES,
		StandardTypes.REGPAIR_WRITE,
		StandardTypes.REGPAIR_TOGGLE,
		StandardTypes.REGPAIR_RW_EXCH,
		StandardTypes.REGPAIR_WR_EXCH)
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   default_type_mappings - Default type mappings will be added to the message type
	##     lookup tables if and only if this is true.
	def __init__(self, default_type_mappings=True):
		## Property: make_val_hex_str
		## If true, strings will be interpreted as sequences of hexadecimal digits
		## (instead of text) when making message values. Default: *False*
		self.make_val_hex_str = False
		
		## Property: str_encoding
		## Name of text encoding to use when making and formatting message values.
		## Default: 'ascii'
		self.str_encoding = 'ascii'
		
		## Property: int_size
		## Default size in bytes of integer fields in message values.
		## Default: 1
		self.int_size = 1
		
		## Property: int_byteorder
		## Name of default byte order of integer fields in message values.
		## Default: 'little'
		self.int_byteorder = 'little'
		
		## Property: byte_sep
		## Byte group separator for message values formatted as hex or binary strings.
		## Default: *None*
		self.byte_sep = None
		
		## Property: bytes_per_sep
		## Byte group size for message values formatted as hex or binary strings.
		## Default: 2
		self.bytes_per_sep = 2
		
		## Property: byte_str_big_endian
		## Byte order selector for message values formatted as hex or binary strings.
		## Default: *False*
		self.byte_str_big_endian = False
		
		## Property: default_val_conv
		## Default conversion for formatted message values.
		## Default: <ValueConversions.Bytes>
		self.default_val_conv = ValueConversions.Bytes
		
		## Property: str_val_conv
		## Conversion for string-formatted message values.
		## Default: <ValueConversions.Hex>
		self.str_val_conv = ValueConversions.Hex
		
		## Property: enum_format_val
		## If true, fields known to contain result codes or message type or register
		## identifiers will be converted to the corresponding enum members (from
		## <type_enum_class> or <res_enum_class>) when formatting message values.
		## This setting overrides any other conversion for those fields.
		## Default: *True*
		self.enum_format_val = True
		
		## Property: type_enum_class
		## Enum class to use for conversion of message type identifiers.
		## Default: <StandardTypes>
		self.type_enum_class = StandardTypes
		
		## Property: res_enum_class
		## Enum class to use for conversion of INM result codes.
		## Default: <StandardResults>
		self.res_enum_class = StandardResults
		
		## Property: res_enum_msg_types
		## Message types to perform result code enum conversion on.
		## SHOULD be a tuple of message type identifiers.
		## Default: <DEFAULT_RES_ENUM_MSG_TYPES>
		self.res_enum_msg_types = self.DEFAULT_RES_ENUM_MSG_TYPES
		
		## Property: reg_enum_class
		## Enum class to use for conversion of logical register identifiers.
		## Default: <StandardRegisters>
		self.reg_enum_class = StandardRegisters
		
		## Property: reg_enum_msg_types
		## Message types to perform register identifier enum conversion on.
		## SHOULD be a tuple of message type identifiers.
		## Default: <DEFAULT_REG_ENUM_MSG_TYPES>
		self.reg_enum_msg_types = self.DEFAULT_REG_ENUM_MSG_TYPES
		
		## Property: default_strictness
		## Default strictness level for multipart message value formatting.
		## Default: <Strictness.Exact>
		self.default_strictness = Strictness.Exact
		
		# NOTE: Centralized mapping of INM message types to value field structure
		#       and data types. Implemented with lazy evaluation, via the 'make'
		#       and 'format' methods. Basically lookup tables that provide
		#       default values for the 'int_size', 'conv' and 'size' arguments.
		self._typ_to_make_val_int_size = {}
		self._typ_to_format_val_conv = {}
		self._typ_to_format_val_size = {}
		
		if default_type_mappings:
			self._populate_type_mapping_tables()
	
	def _populate_type_mapping_tables(self):
		self.set_make_val_int_size(StandardTypes.RESULT, (1,))
		self.set_make_val_int_size(StandardTypes.INM_RESULT, (1, 2))
		self.set_make_val_int_size(StandardTypes.REG_READ, (1,))
		self.set_make_val_int_size(StandardTypes.REG_READ_RES, (1, 1))
		self.set_make_val_int_size(StandardTypes.INM_REG_READ_RES, (1, 1, 2))
		self.set_make_val_int_size(StandardTypes.REG_WRITE, (1, 1))
		self.set_make_val_int_size(StandardTypes.REG_TOGGLE, (1, 1))
		self.set_make_val_int_size(StandardTypes.REG_RW_EXCH, (1, 1))
		self.set_make_val_int_size(StandardTypes.REG_WR_EXCH, (1, 1))
		self.set_make_val_int_size(StandardTypes.REGPAIR_READ, (1,))
		self.set_make_val_int_size(StandardTypes.REGPAIR_READ_RES, (1, 2))
		self.set_make_val_int_size(StandardTypes.INM_REGPAIR_READ_RES, (1, 2, 2))
		self.set_make_val_int_size(StandardTypes.REGPAIR_WRITE, (1, 2))
		self.set_make_val_int_size(StandardTypes.REGPAIR_TOGGLE, (1, 2))
		self.set_make_val_int_size(StandardTypes.REGPAIR_RW_EXCH, (1, 2))
		self.set_make_val_int_size(StandardTypes.REGPAIR_WR_EXCH, (1, 2))
		self.set_make_val_int_size(StandardTypes.MEMMON_DATA, (1, 2, 1))
		
		self.set_format_val_size(StandardTypes.RESULT, (1,))
		self.set_format_val_size(StandardTypes.INM_RESULT, (1, 2))
		self.set_format_val_size(StandardTypes.REG_READ, (1,))
		self.set_format_val_size(StandardTypes.REG_READ_RES, (1, 1))
		self.set_format_val_size(StandardTypes.INM_REG_READ_RES, (1, 1, 2))
		self.set_format_val_size(StandardTypes.REG_WRITE, (1, 1))
		self.set_format_val_size(StandardTypes.REG_TOGGLE, (1, 1))
		self.set_format_val_size(StandardTypes.REG_RW_EXCH, (1, 1))
		self.set_format_val_size(StandardTypes.REG_WR_EXCH, (1, 1))
		self.set_format_val_size(StandardTypes.REGPAIR_READ, (1,))
		self.set_format_val_size(StandardTypes.REGPAIR_READ_RES, (1, 2))
		self.set_format_val_size(StandardTypes.INM_REGPAIR_READ_RES, (1, 2, 2))
		self.set_format_val_size(StandardTypes.REGPAIR_WRITE, (1, 2))
		self.set_format_val_size(StandardTypes.REGPAIR_TOGGLE, (1, 2))
		self.set_format_val_size(StandardTypes.REGPAIR_RW_EXCH, (1, 2))
		self.set_format_val_size(StandardTypes.REGPAIR_WR_EXCH, (1, 2))
		self.set_format_val_size(StandardTypes.MEMMON_DATA, (1, 2, 1))
	
	## Method: clone
	## Copies the *MessageFactory*.
	##
	## Returns:
	##   A new *MessageFactory* with the same formatting configuration as this
	##   *MessageFactory*. The new object contains no reference to any mutable
	##   state of this object (i.e. subsequent changes to this object will not
	##   affect the copy, or vice versa).
	def clone(self):
		copy = MessageFactory(False)
		
		copy.make_val_hex_str = self.make_val_hex_str
		copy.str_encoding = self.str_encoding
		copy.int_size = self.int_size
		copy.int_byteorder = self.int_byteorder
		copy.byte_sep = self.byte_sep
		copy.bytes_per_sep = self.bytes_per_sep
		copy.byte_str_big_endian = self.byte_str_big_endian
		copy.default_val_conv = self.default_val_conv
		copy.str_val_conv = self.str_val_conv
		copy.enum_format_val = self.enum_format_val
		copy.type_enum_class = self.type_enum_class
		copy.res_enum_class = self.res_enum_class
		copy.res_enum_msg_types = self.res_enum_msg_types
		copy.reg_enum_class = self.reg_enum_class
		copy.reg_enum_msg_types = self.reg_enum_msg_types
		copy.default_strictness = self.default_strictness
		
		copy._typ_to_make_val_int_size.update(self._typ_to_make_val_int_size)
		copy._typ_to_format_val_conv.update(self._typ_to_format_val_conv)
		copy._typ_to_format_val_size.update(self._typ_to_format_val_size)
	
	## Method: get_make_val_int_size
	## Gets the value construction integer field sizes associated with a specified
	## message type.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   default - Value to return if *tlv_type* is not found in the lookup table.
	##
	## Returns:
	##   The value construction integer field sizes associated with *tlv_type*, if
	##   that type is found in the lookup table. Otherwise *default*.
	def get_make_val_int_size(self, tlv_type, default=None):
		if not isinstance(tlv_type, int):
			return default
		
		return self._typ_to_make_val_int_size.get(int(tlv_type), default)
	
	## Method: set_make_val_int_size
	## Sets the value construction integer field sizes associated with a specified
	## message type. Can also be used to delete entries from the value construction
	## field size table.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   int_size - Value construction field sizes for *tlv_type*. SHOULD be a
	##     positive integer, a list or tuple of positive integers, or *None*.
	##     If the argument is *None*, any existing entry for *tlv_type* is
	##     deleted from the lookup table.
	##
	## Returns:
	##   True if and only if the table entry was successfully updated or deleted.
	def set_make_val_int_size(self, tlv_type, int_size):
		if not isinstance(tlv_type, int):
			return False
		
		tlv_type = int(tlv_type)
		
		if int_size is None:
			if tlv_type in self._typ_to_make_val_int_size:
				del self._typ_to_make_val_int_size[tlv_type]
			else:
				return False
		else:
			if isinstance(int_size, list):
				int_size = tuple(int_size)
			elif not isinstance(int_size, tuple):
				int_size = (int_size,)
			
			if len(int_size) == 0:  # No empty tuples.
				return False
			
			self._typ_to_make_val_int_size[tlv_type] = int_size
		
		return True
	
	## Method: get_format_val_conv
	## Gets the value formatting conversion specifier associated with a specified
	## message type.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   default - Value to return if *tlv_type* is not found in the lookup table.
	##
	## Returns:
	##   The value formatting conversion specifier associated with *tlv_type*, if
	##   that type is found in the lookup table. Otherwise *default*.
	def get_format_val_conv(self, tlv_type, default=None):
		if not isinstance(tlv_type, int):
			return default
		
		return self._typ_to_format_val_conv.get(int(tlv_type), default)
	
	## Method: set_format_val_conv
	## Sets the value formatting conversion specifier associated with a specified
	## message type. Can also be used to delete entries from the value formatting
	## conversion table.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   conv - Value formatting conversion specifier for *tlv_type*. SHOULD be a
	##     member of <ValueConversions>, an enum class, a list or tuple of any
	##     of the aforementioned types, or *None*. If the argument is *None*,
	##     any existing entry for *tlv_type* is deleted from the lookup table.
	##
	## Returns:
	##   True if and only if the table entry was successfully updated or deleted.
	def set_format_val_conv(self, tlv_type, conv):
		if not isinstance(tlv_type, int):
			return False
		
		tlv_type = int(tlv_type)
		
		if conv is None:
			if tlv_type in self._typ_to_format_val_conv:
				del self._typ_to_format_val_conv[tlv_type]
			else:
				return False
		else:
			if isinstance(conv, list):
				conv = tuple(conv)
			elif not isinstance(conv, tuple):
				conv = (conv,)
			
			if len(conv) == 0:  # No empty tuples.
				return False
			
			self._typ_to_format_val_conv[tlv_type] = conv
		
		return True
	
	## Method: get_format_val_size
	## Gets the value formatting field sizes associated with a specified message type.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   default - Value to return if *tlv_type* is not found in the lookup table.
	##
	## Returns:
	##   The value formatting field sizes associated with *tlv_type*, if
	##   that type is found in the lookup table. Otherwise *default*.
	def get_format_val_size(self, tlv_type, default=None):
		if not isinstance(tlv_type, int):
			return default
		
		return self._typ_to_format_val_size.get(int(tlv_type), default)
	
	## Method: set_format_val_size
	## Sets the value formatting field sizes associated with a specified message type.
	## Can also be used to delete entries from the value formatting field size table.
	##
	## Parameters:
	##   tlv_type - An INM message type identifier.
	##   size - Value formatting field sizes for *tlv_type*. SHOULD be a
	##     positive integer, a list or tuple of positive integers, or *None*.
	##     If the argument is *None*, any existing entry for *tlv_type* is
	##     deleted from the lookup table.
	##
	## Returns:
	##   True if and only if the table entry was successfully updated or deleted.
	def set_format_val_size(self, tlv_type, size):
		if not isinstance(tlv_type, int):
			return False
		
		tlv_type = int(tlv_type)
		
		if size is None:
			if tlv_type in self._typ_to_format_val_size:
				del self._typ_to_format_val_size[tlv_type]
			else:
				return False
		else:
			if isinstance(size, list):
				size = tuple(size)
			elif not isinstance(size, tuple):
				size = (size,)
			
			if len(size) == 0:  # No empty tuples.
				return False
			
			self._typ_to_format_val_size[tlv_type] = size
		
		return True
	
	## Method: make_val
	## Creates a bytes object containing an INM message value. Objects representable
	## as message values include bytes objects, strings, integers, enum members and
	## tuples and lists of integers in the unsigned byte range (i.e. 0-255).
	##
	## Parameters:
	##   val - The object to convert to a message value.
	##   int_size - The size in bytes of a message value produced from an integer.
	##     If this is *None*, a default size will be used.
	##   tlv_type - An optional INM message type specifier. Used to determine
	##     the default integer value size.
	##
	## Returns:
	##   A bytes object containing a converted representation of *val*.
	##   (Or *None*, if *val* couldn't be converted.)
	def make_val(self, val, int_size=None, tlv_type=None):
		if int_size is None:
			int_size = self.get_make_val_int_size(tlv_type, self.int_size)
		
		if isinstance(int_size, tuple) or isinstance(int_size, list):
			int_size = int_size[0]  # Only a scalar is wanted.
		
		b_val = None
		
		if isinstance(val, bytes):
			b_val = val
		elif isinstance(val, str):
			if self.make_val_hex_str:
				b_val = bytes.fromhex(val)
			else:
				b_val = bytes(val, self.str_encoding)
		elif isinstance(val, int):
			# NOTE: Integers must be nonnegative and fit in (int_size) bytes.
			if val >= 0 and (val >> 8*int_size) == 0:
				b_val = val.to_bytes(int_size, self.int_byteorder)
		elif isinstance(val, tuple) or isinstance(val, list):
			b_val = bytes(val)
		elif isinstance(val, enum.Enum):
			if val.value >= 0 and (val.value >> 8*int_size) == 0:
				b_val = val.value.to_bytes(int_size, self.int_byteorder)
		elif hasattr(val, self.MAKE_VAL_ATTR_NAME):  # TODO: Test and document this feature.
			to_inm_val = getattr(val, self.MAKE_VAL_ATTR_NAME)
			b_val = to_inm_val(self, int_size, tlv_type)
		elif val is None:
			b_val = b''
		
		return b_val
	
	## Method: make_mval
	## Creates a bytes object containing a multipart INM message value. Objects
	## representable as fields in multipart values include bytes objects, strings,
	## integers, enum members and tuples and lists of integers in the unsigned
	## byte range (i.e. 0-255).
	##
	## Parameters:
	##   mval - The objects to convert to a multipart message value. MUST be a sequence.
	##   int_size - The sizes in bytes of message value fields produced from an
	##     integer. May be a scalar (if all integer fields have the same size),
	##     tuple or list. If this is *None*, default sizes will be used.
	##   tlv_type - An optional INM message type specifier. Used to determine
	##     the default integer field sizes.
	##
	## Returns:
	##   A bytes object containing a converted representation of *mval*.
	##   (Or *None*, if some object in *mval* couldn't be converted.)
	def make_mval(self, mval, int_size=None, tlv_type=None):
		if int_size is None:
			int_size = self.get_make_val_int_size(tlv_type, self.int_size)
		
		if isinstance(int_size, int):
			old_int_size = int_size
			int_size = (old_int_size for _ in range(len(mval)))
		
		b_mval = b''
		
		# ISSUE: If len(int_size) < len(mval), then only len(int_size) entries will be
		#        generated. Is this behavior undesirable (potentially confusing)?
		for val, val_int_size in zip(mval, int_size):
			b_val = self.make_val(val, val_int_size, tlv_type)
			if b_val is None:
				return None
			b_mval += b_val
		
		return b_mval
	
	## Method: format_val_str
	## Applies string formatting to an INM message value.
	##
	## Parameters:
	##   val - A bytes object containing the message value to format.
	##   tlv_type - An optional INM message type specifier. Used to determine
	##     appropriate enum mappings.
	##
	## Returns:
	##   The result of applying string formatting (determined by <str_val_conv>)
	##   to *val*.
	def format_val_str(self, val, tlv_type=None):
		return self.format_val(val, self.str_val_conv, tlv_type)
	
	## Method: format_val
	## Applies formatting to an INM message value.
	##
	## Parameters:
	##   val - A bytes object containing the message value to format.
	##   conv - Message value conversion specifier (e.g. from <ValueConversions>),
	##     or *None* to apply the default conversion.
	##   tlv_type - An optional INM message type specifier. Used to determine
	##     the default value conversion and enum mappings.
	##
	## Returns:
	##   The result of applying message value formatting to *val*.
	def format_val(self, val, conv=None, tlv_type=None):
		if conv is None:
			conv = self.get_format_val_conv(tlv_type, self.default_val_conv)
		
		if isinstance(conv, tuple) or isinstance(conv, list):
			conv = conv[0]  # Only a scalar is wanted.
		
		if self.enum_format_val and tlv_type is not None and len(val) >= 1:
			e_val = None
			
			if tlv_type in self.res_enum_msg_types:
				e_val = map_enum(self.res_enum_class, val[0], val[0])
			elif tlv_type in self.reg_enum_msg_types:
				e_val = map_enum(self.reg_enum_class, val[0], val[0])
			
			if e_val is None:
				pass
			elif len(val) == 1:
				return e_val
			else:
				# NOTE: Omit tlv_type to prevent re-application of standard enum mapping.
				i_val = self.format_val(val[1:], conv)
				return (e_val, i_val)
		
		f_val = None
		
		# ISSUE: Remove the legacy magic strings?
		if conv == ValueConversions.Hex or conv == 'hex':
			f_val = bytes_to_hex(val, self.byte_sep, self.bytes_per_sep, self.byte_str_big_endian)
		elif conv == ValueConversions.Bin or conv == 'bin':
			f_val = bytes_to_bin(val, self.byte_sep, self.bytes_per_sep, self.byte_str_big_endian)
		elif conv == ValueConversions.Int or conv == 'int':
			f_val = int.from_bytes(val, self.int_byteorder)
		elif conv == ValueConversions.Str or conv == 'str':
			f_val = str(val, self.str_encoding)
		elif conv == ValueConversions.Tuple or conv == 'tuple':
			f_val = tuple(val)
		elif conv == ValueConversions.List or conv == 'list':
			f_val = list(val)
		elif conv == ValueConversions.Bytes or conv == 'bytes':
			f_val = val
		elif inspect.isclass(conv) and issubclass(conv, enum.Enum):
			# ISSUE: Does this make the special cases (StandardTypes.RESULT, etc.) above redundant?
			f_val = int.from_bytes(val, self.int_byteorder)
			f_val = map_enum(conv, f_val, f_val)
		elif callable(conv):  # TODO: Test and document this feature.
			f_val = conv(self, val, tlv_type)  # Pass the factory to make it available as a helper.
		else:
			pass  # Unrecognized conversion type, return None.
		
		return f_val
	
	## Method: format_mval
	## Applies multipart formatting to an INM message value.
	##
	## Parameters:
	##   mval - A bytes object containing the multipart message value to format.
	##   conv - Field value conversion specifiers (e.g. from <ValueConversions>).
	##     May be a scalar (to apply the same conversion to all fields), tuple or list.
	##     If this is *None*, default conversions are applied.
	##   size - Multipart value field sizes. May be a scalar (if all fields have the same size),
	##     tuple or list. If this is *None*, the message type is used to infer field sizes, if
	##     possible.
	##   tlv_type - An optional INM message type specifier. Used to determine the default
	##     value conversions, field sizes and enum mappings.
	##   n_vals - Multipart value field count. If this is *None*, the field count is inferred
	##     from the *conv* and *size* arguments (or their default values).
	##   strict - Multipart formatting <Strictness> specifier, or *None* to apply the default
	##     strictness level.
	##
	## Returns:
	##   The formatted multipart message value (list of formatted fields), if formatting was
	##   successful. Otherwise *None*.
	def format_mval(self, mval, conv=None, size=None, tlv_type=None, n_vals=None, strict=None):
		if conv is None:
			conv = self.get_format_val_conv(tlv_type, self.default_val_conv)
		
		if size is None:
			size = self.get_format_val_size(tlv_type, self.int_size)
		
		if n_vals is not None:
			pass
		elif isinstance(conv, tuple) or isinstance(conv, list):
			n_vals = len(conv)
		elif isinstance(size, tuple) or isinstance(size, list):
			n_vals = len(size)
		else:
			n_vals = 1
		
		if strict is None:
			strict = self.default_strictness
		
		if not (isinstance(conv, tuple) or isinstance(conv, list)):
			old_conv = conv
			conv = (old_conv for _ in range(n_vals))
		
		if not (isinstance(size, tuple) or isinstance(size, list)):
			old_size = size
			size = (old_size for _ in range(n_vals))
		
		f_mval = []
		val_offset = 0
		
		# ISSUE: If len(conv) < len(size), then only len(conv) entries will be
		#        generated. Is this behavior undesirable (potentially confusing)?
		for val_conv, val_size in zip(conv, size):
			val_end_offset = val_offset + val_size
			
			if val_end_offset > len(mval):  # No more values present.
				break
			
			val = mval[val_offset:val_end_offset]
			
			if val_offset == 0:  # First field is subject to standard enum mapping.
				# ISSUE: Is standard enum mapping desirable in multipart formatting?
				f_val = self.format_val(val, val_conv, tlv_type)
			else:
				# NOTE: Omit tlv_type to prevent re-application of standard enum mapping.
				f_val = self.format_val(val, val_conv)
			
			if f_val is None:  # Conversion failed.
				#print(f'conv={val_conv} size={val_size}') # DEBUG: 
				break
			
			f_mval.append(f_val)
			val_offset += val_size
		
		if strict >= Strictness.AllWanted and len(f_mval) != n_vals:
			if len(f_mval) < n_vals or strict >= Strictness.OnlyWanted:
				return None  # Wrong number of fields parsed.
		
		if strict >= Strictness.Exact and val_offset != len(mval):
			return None  # Unparsed bytes at end of input sequence.
		
		return f_mval
	
	# ISSUE: Check the type identifier range in the make*_msg* methods?
	#        Are large messages even a good idea?
	## Method: make_msg
	## Constructs an INM <Message> with the specified type and value.
	##
	## Parameters:
	##   typ - An INM message type specifier.
	##   val - The object to convert to a message value.
	##   int_size - The size in bytes of a message value produced from an integer.
	##     If this is *None*, a default size will be used.
	##
	## Returns:
	##   A <StandardMessage> with type *typ* and a value constructed by applying
	##   <make_val> to *val*. (Or *None*, if a value couldn't be constructed.)
	def make_msg(self, typ, val, int_size=None):
		b_val = self.make_val(val, int_size, typ)
		if b_val is None:
			return None
		
		msg = StandardMessage(typ, b_val)
		return msg
	
	## Method: make_msg_mval
	## Constructs an INM <Message> with the specified type and multipart value.
	##
	## Parameters:
	##   typ - An INM message type specifier.
	##   mval - The objects to convert to a multipart message value. MUST be a sequence.
	##   int_size - The sizes in bytes of message value fields produced from an
	##     integer. May be a scalar (if all integer fields have the same size),
	##     tuple or list. If this is *None*, default sizes will be used.
	##
	## Returns:
	##   A <StandardMessage> with type *typ* and a value constructed by applying
	##   <make_mval> to *mval*. (Or *None*, if a value couldn't be constructed.)
	def make_msg_mval(self, typ, mval, int_size=None):
		b_mval = self.make_mval(mval, int_size, typ)
		if b_mval is None:
			return None
		
		msg = StandardMessage(typ, b_mval)
		return msg
	
	## Method: make_large_msg
	## Constructs a large INM <Message> with the specified type and value.
	##
	## Parameters:
	##   typ - An INM message type specifier.
	##   val - The object to convert to a message value.
	##   int_size - The size in bytes of a message value produced from an integer.
	##     If this is *None*, a default size will be used.
	##
	## Returns:
	##   A <LargeMessage> with type *typ* and a value constructed by applying
	##   <make_val> to *val*. (Or *None*, if a value couldn't be constructed.)
	def make_large_msg(self, typ, val, int_size=None):
		b_val = self.make_val(val, int_size, typ)
		if b_val is None:
			return None
		
		msg = LargeMessage(typ, b_val)
		return msg
	
	## Method: make_large_msg_mval
	## Constructs a large INM <Message> with the specified type and multipart value.
	##
	## Parameters:
	##   typ - An INM message type specifier.
	##   mval - The objects to convert to a multipart message value. MUST be a sequence.
	##   int_size - The sizes in bytes of message value fields produced from an
	##     integer. May be a scalar (if all integer fields have the same size),
	##     tuple or list. If this is *None*, default sizes will be used.
	##
	## Returns:
	##   A <LargeMessage> with type *typ* and a value constructed by applying
	##   <make_mval> to *mval*. (Or *None*, if a value couldn't be constructed.)
	def make_large_msg_mval(self, typ, mval, int_size=None):
		b_mval = self.make_mval(mval, int_size, typ)
		if b_mval is None:
			return None
		
		msg = LargeMessage(typ, b_mval)
		return msg
	
	## Method: msg_to_bytes
	## Converts a <Message> and <MessageHeader> to a bytes object containing the
	## message and header in standard binary on-wire format.
	##
	## Parameters:
	##   msg - An instance of <Message>.
	##   header - An optional instance of <MessageHeader>.
	##
	## Returns:
	##   A bytes object containing the INM message represented by *msg* in standard
	##   binary on-wire format, preceded by *header* (also in standard binary on-wire
	##   format), if that argument was provided.
	def msg_to_bytes(self, msg, header=None):
		msg_b = msg.to_bytes(self)
		
		if header is not None:
			header_b = header.to_bytes(self)
			msg_b = header_b + msg_b
		
		return msg_b


## Class: MessageHeader
## Represents the header of an INM message.
class MessageHeader:
	## Variable: MESSAGE_ID_SIZE
	## Size in bytes of the message identifier field in an INM message header.
	MESSAGE_ID_SIZE = 2
	
	## Variable: DSTADR_SIZE
	## Size in bytes of the destination address field in an INM message header.
	DSTADR_SIZE = 1
	
	## Variable: SRCADR_SIZE
	## Size in bytes of the source address field in an INM message header.
	SRCADR_SIZE = 1
	
	## Variable: HEADER_SIZE
	## Size in bytes of a complete INM message header.
	HEADER_SIZE = MESSAGE_ID_SIZE + DSTADR_SIZE + SRCADR_SIZE
	
	# ISSUE: Are 16-bit per-originating-channel message IDs enough to avoid ambiguity?
	#        Can we generally assume that any response will be received on the channel
	#        that originated the corresponding request, and that all requests to a given
	#        node from a given node will be sent via the same originating channel?
	## Variable: MAX_MESSAGE_ID
	## Largest valid INM message identifier.
	MAX_MESSAGE_ID = 0xffff
	
	## Variable: MAX_MESSAGE_ADR
	## Largest valid INM node address.
	MAX_MESSAGE_ADR = 0xff
	
	## Variable: LOCAL_ADR
	## Reserved INM node address for the local node.
	LOCAL_ADR = 0x00
	
	## Variable: BROADCAST_ADR
	## Reserved INM node address for broadcast messages.
	BROADCAST_ADR = MAX_MESSAGE_ADR
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   msg_id - INM message identifier.
	##   dstadr - Destionation INM address.
	##   srcadr - Source INM address.
	def __init__(self, msg_id, dstadr, srcadr):
		# ISSUE: Is this the right place for these range checks? Better in send()?
		if not (0 <= msg_id <= self.MAX_MESSAGE_ID):
			raise ValueError()
		
		if not (0 <= dstadr <= self.MAX_MESSAGE_ADR):
			raise ValueError()
		
		if not (0 <= srcadr <= self.MAX_MESSAGE_ADR):
			raise ValueError()
		
		## Property: msg_id
		## INM message identifier.
		self.msg_id = msg_id
		
		## Property: dstadr
		## Destination INM node address.
		self.dstadr = dstadr
		
		## Property: srcadr
		## Source INM node address.
		self.srcadr = srcadr
	
	## Method: __str__
	## String conversion.
	##
	## Returns:
	##   A string representation of the *MessageHeader*, in the format
	##   "MessageHeader(msg_id, dstadr, srcadr)".
	def __str__(self):
		return f'MessageHeader({self.msg_id}, {self.dstadr}, {self.srcadr})'
	
	## Method: __repr__
	## String representation.
	##
	## Returns:
	##   A string representation of the *MessageHeader*, in the format
	##   "<MessageHeader(msg_id, dstadr, srcadr)>".
	def __repr__(self):
		return f'<{str(self)}>'
	
	## Method: to_bytes
	## Converts the *MessageHeader* to a bytes object containing the header in standard
	## binary on-wire format.
	##
	## Parameters:
	##   formatter - Formatter to use for integer conversion, or *None* to use
	##     the default formatter.
	##
	## Returns:
	##   A bytes object containing the INM message header represented by this
	##   *MessageHeader* in standard binary on-wire format.
	def to_bytes(self, formatter=None):
		if formatter is None:
			formatter = default_msg_factory
		
		msg_id_b = formatter.make_val(self.msg_id, int_size=2)
		dstadr_b = formatter.make_val(self.dstadr, int_size=1)
		srcadr_b = formatter.make_val(self.srcadr, int_size=1)
		return msg_id_b + dstadr_b + srcadr_b


## Section: Default Objects

## Variable: default_msg_factory
## A default instance of <MessageFactory>. Used by <Message>, <MessageHeader> and
## <MessageChannel> if no other message formatter is provided. This is a module-level
## attribute.
default_msg_factory = MessageFactory()


## Section: Message Channels

## Class: MessageChannelError
## Exception class for <MessageChannels>.
class MessageChannelError(Exception):
	pass

## Class: MessageChannel
## Abstract parent class for objects that send and receive INM messages.
##
## A *MessageChannel* can be used as a context manager, opening itself when
## the context is entered and then closing itself upon exit from the context
## (see <open> and <close>).
class MessageChannel:
	
	## Variable: make_default_selector
	## If this is true, a default selector object (from the standard library module
	## *selectors*) will be created for each instance of *MessageChannel* that isn't
	## provided with a custom selector at initialization.
	make_default_selector = True
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   srcadr - INM address of the local node. Will be placed in the source address
	##     header field of sent messages for which no other source address is provided.
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   msg_factory - The <MessageFactory> that the *MessageChannel* should use to create
	##     INM <Message> objects. If this parameter is *None*, a default <MessageFactory>
	##     will be used.
	##   selector - A selector object compatible with the ones provided by the standard
	##     library module *selectors*. The selector is used to implement <readable> and
	##     may also be used internally. If this parameter is *None*, a default selector
	##     may be created (see <make_default_selector>). If this parameter is *False*
	##     (specifically, not some other false value), a default selector will NOT be
	##     created.
	##   ch_num - Channel number of the *MessageChannel*. SHOULD be either *None* or
	##     a nonnegative integer.
	def __init__(self, srcadr, timeout=None, msg_factory=None, selector=None, ch_num=None):
		if msg_factory is None:
			msg_factory = default_msg_factory
		
		if selector is False:
			selector = None
		elif selector is None and MessageChannel.make_default_selector:
			selector = selectors.DefaultSelector()
		
		## Property: srcadr
		## Default INM source address of messages sent via this *MessageChannel*.
		self.srcadr = srcadr
		
		## Property: timeout
		## Receive timeout. MUST be an instance of the standard library class
		## *datetime.timedelta*. If this attribute is *None*, receive operations will
		## never time out.
		##
		## NOTE: The method <set_timeout> SHOULD be used to change this attribute.
		self.timeout = timeout
		
		## Property: msg_factory
		## The <MessageFactory> that the *MessageChannel* uses to create INM <Message>
		## objects.
		self.msg_factory = msg_factory
		
		## Property: selector
		## A selector object compatible with the ones provided by the standard
		## library module *selectors*. May be *None*. SHOULD be treated as
		## a read-only attribute.
		self.selector = selector
		
		## Property: ch_num
		## Channel number of this *MessageChannel*. May be *None*. SHOULD be treated
		## as a read-only attribute.
		self.ch_num = ch_num  # ISSUE: Should this be type/range checked?
		
		self._is_open = False
		self._prev_msg_id = None
		self._next_msg_id = 0
	
	## Method: __str__
	## String conversion.
	##
	## Returns:
	##   A string representation of the *MessageChannel*, in the format
	##   "class_name(srcadr)".
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr})'
	
	## Method: __repr__
	## String representation.
	##
	## Returns:
	##   A string representation of the *MessageChannel*, in the format
	##   "<class_name(srcadr)>".
	def __repr__(self):
		return f'<{str(self)}>'
	
	def __enter__(self):
		res = self.open()
		if res not in (ResultCode.SUCCESS, ResultCode.INVALID_STATE):
			raise MessageChannelError(str(res))
		return self
	
	def __exit__(self, exc_type, exc_value, traceback):
		self.close()
		return False
	
	def _register_selectable(self, selectable, ch_num=None):
		if self.selector is None:
			return None
		
		try:
			selector_key = self.selector.register(selectable, selectors.EVENT_READ, ch_num)
		except Exception:
			return None
		
		return selector_key
	
	def _unregister_selectable(self, selectable):
		if self.selector is None:
			return
		
		try:
			self.selector.unregister(selectable)
		except Exception:
			pass  # TODO: Report this error.
	
	def _readable_selectables(self, timeout_secs=0.0):
		if self.selector is None:
			return None
		
		selected = self.selector.select(timeout_secs)
		return (k for k, e in selected if e & selectors.EVENT_READ)
	
	def _selectable_is_readable(self, timeout_secs=0.0):
		if self.selector is None:
			return False
		
		selected = self.selector.select(timeout_secs)
		return any(e & selectors.EVENT_READ for k, e in selected)
	
	## Method: peek_prev_msg_id
	## Peeks at the most recently auto-generated INM message ID.
	##
	## CAUTION: This is not thread-safe. If this *MessageChannel* is thread-shared,
	## other threads may go ahead and cause auto-generation of additional message IDs
	## at any time, unless measures are taken (outside the *inm* module) to prevent
	## or handle such occurrences.
	##
	## Returns:
	##   The INM message ID most recently auto-generated by this *MessageChannel*.
	def peek_prev_msg_id(self):
		return self._prev_msg_id
	
	## Method: peek_next_msg_id
	## Peeks at the next INM message ID to be auto-generated.
	##
	## CAUTION: This is not thread-safe. If this *MessageChannel* is thread-shared,
	## other threads may go ahead and cause auto-generation of additional message IDs
	## at any time, unless measures are taken (outside the *inm* module) to prevent
	## or handle such occurrences.
	##
	## Returns:
	##   The next INM message ID to be auto-generated by this *MessageChannel*.
	def peek_next_msg_id(self):
		return self._next_msg_id
	
	## Method: get_next_msg_id
	## Auto-generates an INM message ID.
	##
	## CAUTION: This is not thread-safe. If this *MessageChannel* is thread-shared,
	## other threads may interfere with the auto-generation, causing duplicate message
	## IDs, unless measures are taken (outside the *inm* module) to prevent such
	## occurrences.
	##
	## Returns:
	##   An auto-generated INM message ID.
	def get_next_msg_id(self):
		msg_id = self._next_msg_id
		self._next_msg_id = (msg_id + 1) % (MessageHeader.MAX_MESSAGE_ID + 1)
		return msg_id
	
	## Method: is_open
	## Checks whether the *MessageChannel* is open.
	##
	## Returns:
	##   True if and only if this *MessageChannel* is currently open.
	def is_open(self):
		return self._is_open
	
	## Method: open
	## Opens the *MessageChannel*. Subclasses may need to override the base class
	## implementation.
	##
	## Returns:
	##   <ResultCode.SUCCESS> if the channel was successfully opened, otherwise another
	##   <ResultCode> indicating what went wrong.
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		self._is_open = True
		return ResultCode.SUCCESS
	
	## Method: close
	## Closes the *MessageChannel*. Subclasses may need to override the base class
	## implementation.
	def close(self):
		self._is_open = False
	
	## Method: readable
	## Checks whether the *MessageChannel* is readable.
	##
	## NOTE: The base class implementation of this method needs a <selector> to work.
	##
	## CAUTION: Even if this method returns true, a subsequent call to <recv> may
	## not return a message without blocking.
	##
	## Returns:
	##   True if and only if this *MessageChannel* is currently open and readable.
	def readable(self):
		if not self.is_open():
			return False
		
		return self._selectable_is_readable()
	
	## Method: get_selector_keys
	## Gets the selector keys of encapsulated selectable objects from this *MessageChannel*.
	##
	## NOTE: The base class implementation always returns an empty tuple.
	##
	## Returns:
	##   A tuple of selector keys compatible with the ones produced by the
	##   *register* method of the standard library class *selectors.BaseSelector*.
	def get_selector_keys(self):
		return ()
	
	## Method: set_timeout
	## Updates the <timeout> attribute. Subclasses may need to override the base class
	## implementation.
	##
	## Parameters:
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	def set_timeout(self, timeout):
		self.timeout = timeout
	
	## Method: send
	## Sends an INM message. This is an abstract method.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   msg - The INM <Message> to send.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     internally.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the configured <srcadr> of this *MessageChannel* will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the behavior is implementation-defined (e.g. some subclasses may not use link
	##     addresses at all).
	##
	## Returns:
	##   A <ResultCode> indicating the outcome of the attempted send operation.
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		raise NotImplementedError()
	
	## Method: recv
	## Attempts to receive an INM message. This is an abstract method.
	##
	## Returns:
	##   res - A <ResultCode> indicating the outcome of the attempted receive operation.
	##   header - The <MessageHeader> of the received message, or *None* in case of failure.
	##   msg - The received <Message>, or *None* in case of failure.
	##   link_adr - The link address of the received message, or *None* in case of failure.
	def recv(self):
		raise NotImplementedError()


## Class: RoutingMessageChannel
## A <MessageChannel> with message routing functionality. Maintains collections of other
## <MessageChannels> that it uses to receive incoming messages and then retransmit them
## toward their final destinations. Uses a static routing table to determine on which
## channel to retransmit each incoming message.
class RoutingMessageChannel(MessageChannel):
	
	## Variable: TIMEOUT_DIVISOR
	## If a receive timeout is set on a *RoutingMessageChannel*, the timeout set on each
	## of the <recv_channels> will be the primary timeout divided by *TIMEOUT_DIVISOR*
	## times the number of receive channels.
	TIMEOUT_DIVISOR = 10.0
	
	## Variable: DEFAULT_CH_TIMEOUT
	## If no receive timeout is set on a *RoutingMessageChannel*, then the timeout
	## set on each of the <recv_channels> will be *DEFAULT_CH_TIMEOUT*.
	##
	## NOTE: Setting a timeout on the receive channels even when there is no timeout
	## on the RoutingMessageChannel itself prevents <recv> from blocking forever
	## on one receive channel while another receive channel has data available.
	DEFAULT_CH_TIMEOUT = get_timedelta(musecs=10000)
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   srcadr - INM address of the local node. Will be placed in the source address
	##     header field of sent messages for which no other source address is provided.
	##   rtab - A routing table. Used to initialize the <rtab> attribute, see that entry
	##     for more information.
	##   recv_channels - A receive channel table. Used to initialize the <recv_channels>
	##     attribute, see that entry for more information.
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   msg_factory - The <MessageFactory> that the *RoutingMessageChannel* should use
	##     to create INM <Message> objects. If this parameter is *None*, a default
	##     <MessageFactory> will be used.
	##   selector - A selector object compatible with the ones provided by the standard
	##     library module *selectors*. This parameter is passed on to
	##     <MessageChannel.__init__>, see that entry for for more information.
	##   ch_num - Channel number of the *RoutingMessageChannel*. SHOULD be either *None*
	##     or a nonnegative integer.
	def __init__(self, srcadr, rtab, recv_channels, timeout=None, msg_factory=None, selector=None, ch_num=None):
		super().__init__(srcadr, timeout, msg_factory, selector, ch_num)
		
		## Property: rtab
		## The routing table. MUST be a dictionary that maps INM destination node
		## addresses to lists of link address pairs. Each link address pair MUST consist
		## of a <MessageChannel> for outgoing messages to the destination node and
		## a channel-specific link address to use when sending messages on the outgoing
		## message channel.
		self.rtab = rtab
		
		## Property: recv_channels
		## The receive channel table. MUST be a dictionary that maps channel numbers
		## to <MessageChannels> for incoming messages.
		self.recv_channels = recv_channels
		
		## Property: cc_to
		## List of INM node addresses of CC message destinations. SHOULD be updated
		## via <add_cc_adr> and <remove_cc_adr>.
		##
		## Any CC destination node on this list will be sent a copy of each non-broadcast,
		## non-CC message sent by this *RoutingMessageChannel* (sent messages include
		## both routed messages and messages sent from the local node), unless the CC
		## destination is the source of the message.
		self.cc_to = []
		
		## Property: close_recv_channels
		## If this is true, all channels in <recv_channels> are closed when this
		## *RoutingMessageChannel* is closed. Default: *True*
		self.close_recv_channels = True
		
		## Property: relay_messages
		## If this is true, routing is applied even to messages where the destination
		## address equals the source address of the local node.
		## Default: *False*
		self.relay_messages = False
		
		self._selector_key_cache = None  # Cache of readable selector keys, used by recv().
	
	## Method: add_cc_adr
	## Adds the INM address of a CC message destination node to <cc_to>.
	##
	## Parameters:
	##   cc_adr - The INM address of a CC destination.
	##
	## Returns:
	##   True if and only if *cc_adr* was successfully added to <cc_to>.
	def add_cc_adr(self, cc_adr):
		if cc_adr in self.cc_to:
			return False
		
		self.cc_to.append(cc_adr)
		return True
	
	## Method: remove_cc_adr
	## Removes the INM address of a CC message destination node from <cc_to>.
	##
	## Parameters:
	##   cc_adr - The INM address of a CC destination.
	##
	## Returns:
	##   True if and only if *cc_adr* was successfully removed from <cc_to>.
	def remove_cc_adr(self, cc_adr):
		if cc_adr not in self.cc_to:
			return False
		
		self.cc_to.remove(cc_adr)
		return True
	
	## Method: open
	## Overrides <MessageChannel.open>.
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		
		self.set_timeout(self.timeout) # NOTE: Update timeouts in child channels.
		
		for ch in self.recv_channels.values():
			res = ch.open()
			if res not in (ResultCode.SUCCESS, ResultCode.INVALID_STATE):
				return res
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	## Method: close
	## Overrides <MessageChannel.close>.
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self.close_recv_channels:
			for ch in self.recv_channels.values():
				ch.close()
		
		self.rtab = None
		self.recv_channels = None
		
		self._selector_key_cache = None
	
	# NOTE: The base class implementation works for all currently defined
	#       MessageChannel subclasses, and is more efficient.
	#def readable(self):
	#	return any(ch.readable() for ch in self.recv_channels.values())
	
	## Method: get_selector_keys
	## Returns all selector keys obtained by calling the *get_selector_keys* methods
	## of the channels in <recv_channels>. Overrides <MessageChannel.get_selector_keys>.
	def get_selector_keys(self):
		return (k for ch in self.recv_channels.values() for k in ch.get_selector_keys())
	
	## Method: set_timeout
	## Overrides <MessageChannel.set_timeout>.
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if self.timeout is None:
			# NOTE: Set a receive channel timeout even when there is no timeout for
			#       the RoutingMessageChannel itself. This prevents recv() from
			#       blocking forever on one receive channel while another receive
			#       channel has data available.
			ch_timeout = self.DEFAULT_CH_TIMEOUT
		else:
			n_ch = max(1, len(self.recv_channels))
			ch_timeout = self.timeout / (self.TIMEOUT_DIVISOR * n_ch)
		
		for ch in self.recv_channels.values():
			# ISSUE: Why am I setting ZERO_DURATION instead of ch_timeout here?
			# I think I changed how this works, making it less loop-de-loopy,
			# a while ago. Receive channel timeouts might not be a thing anymore.
			#ch.set_timeout(ch_timeout)
			ch.set_timeout(ZERO_DURATION)
	
	## Method: send
	## Implements <MessageChannel.send>. Adds a few more some optional parameters.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   msg - The INM <Message> to send.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     internally.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the value of <MessageChannel.srcadr> will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the *dstadr* argument will be used to look up a link address in <rtab>.
	##   in_link_adr - Optional link address via which routed message *msg* was received.
	##   envelope_dstadr - Optional INM address that will be sent in the on-wire
	##     destination address header field, but NOT used to look up the destination node
	##     in <rtab> (the *dstadr* argument will be used for that).
	##     This parameter is used when sending CC messages, since the original
	##     destination address might be of interest to the CC target.
	##   send_cc - If this is false, no CC message copies of *msg* will be sent.
	##
	## Returns:
	##   A <ResultCode> indicating the outcome of the attempted send operation.
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None, in_link_adr=None,
	               envelope_dstadr=None, send_cc=True):
		if msg_id is None:
			msg_id = self.get_next_msg_id()
		
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is None:
			link_adr = self.rtab.get(dstadr)
		elif link_adr is not None and not isinstance(link_adr, list):
			link_adr = (link_adr,)
		
		if link_adr is None:
			return ResultCode.UNROUTABLE
		
		if envelope_dstadr is None:
			envelope_dstadr = dstadr
		
		res = ResultCode.UNROUTABLE
		
		for link_adr_pair in link_adr:
			if link_adr_pair == in_link_adr:
				continue # NOTE: Never bounce a message back on the link it arrived on.
			
			ch, ch_link_adr = link_adr_pair
			#print(f'{dstadr} {ch} {ch_link_adr}') # DEBUG: 
			
			ch_res = ch.send(envelope_dstadr, msg, msg_id, srcadr, ch_link_adr)
			
			if ch_res != ResultCode.SUCCESS:
				res = ch_res
			elif res == ResultCode.UNROUTABLE:
				res = ResultCode.SUCCESS
		
		if send_cc and dstadr != MessageHeader.BROADCAST_ADR:  # Don't CC broadcast messages.
			for cc_adr in (adr for adr in self.cc_to if adr != dstadr):  # Don't CC to original destination.
				# ISSUE: Do something with the CC result?
				cc_res = self.send(cc_adr, msg, msg_id, srcadr, envelope_dstadr=dstadr, send_cc=False)
		
		# ISSUE: Is updating self._prev_msg_id here meaningful for a routing channel?
		self._prev_msg_id = msg_id
		
		return res
	
	## Method: recv
	## Implements <MessageChannel.recv>.
	def recv(self):
		timeout = self.timeout
		timeout_remaining = timeout
		if timeout_remaining is not None:
			timeout_remaining_secs = timeout_remaining.total_seconds()
		else:
			timeout_remaining_secs = None
		
		selector_keys = self._selector_key_cache  # Attempt to get selector key from cache.
		selector_key = None if selector_keys is None else next(selector_keys, None)
		
		res, header, msg, link_adr = None, None, None, None
		
		if timeout is not None:
			t0 = get_timestamp()
		
		while res is None:  # Not done yet.
			if selector_key is None:  # Attempt to get more readable selector keys.
				selector_keys = self._readable_selectables(timeout_remaining_secs)
				selector_key = next(selector_keys, None)
			
			while selector_key:  # We have a readable selector key.
				ch_num = selector_key.data  # Get channel number from readable selector key.
				ch = self.recv_channels[ch_num]  # Map channel number to receive channel.
				
				# NOTE: _readable_selectables() reports low-level internal channel readability,
				#       so ch.recv() may not have a complete message to return.
				ch_res, header, msg, ch_link_adr = ch.recv()  # Attempt to receive a message.
				
				if ch_res == ResultCode.SUCCESS:
					res, link_adr = ResultCode.SUCCESS, (ch, ch_link_adr)
					break  # Done (success).
				elif ch_res != ResultCode.RECV_TIMEOUT:
					res, link_adr = ResultCode.CHANNEL_FAILURE, (ch, ch_link_adr)
					break  # Done (failure).
				
				selector_key = next(selector_keys, None)  # Try the next selector key.
			
			if res is None and timeout is not None:  # Not done, timeout elapsing.
				t1 = get_timestamp()  # Update receive timeout.
				dt = t1 - t0
				timeout_remaining -= dt
				timeout_remaining_secs = timeout_remaining.total_seconds()
				t0 = t1
				
				if timeout_remaining <= ZERO_DURATION:  # Timed out.
					res = ResultCode.RECV_TIMEOUT
		
		if selector_key:  # Is selector_keys worth caching?
			self._selector_key_cache = selector_keys  # This one might have more life in it.
		else:
			self._selector_key_cache = None  # Start over with new keys at the next invocation.
		
		return res, header, msg, link_adr
	
	## Method: route
	## Attempts to receive and then route an INM message.
	##
	## Returns:
	##   delivery_flag - True if and only if the local node is an INM destination of
	##     the received message (as the unicast destination or as receiver of a broadcast
	##     message).
	##   res - A <ResultCode> indicating the outcome of the attempted route operation.
	##   header - The <MessageHeader> of the received message, or *None* in case of failure.
	##   msg - The received <Message>, or *None* in case of failure.
	##   link_adr - The link address of the received message, or *None* in case of failure.
	def route(self):
		delivery_flag = False
		res, header, msg, link_adr = self.recv()  # Receive a message.
		
		if res == ResultCode.SUCCESS:  # Did we receive anything?
			forward_flag = False
			
			if header.dstadr == self.srcadr:  # Message for us?
				delivery_flag = True
				forward_flag = self.relay_messages  # Forward message iff relay enabled.
			else:  # Message not specifically for us, but broadcast messages are for everyone.
				delivery_flag = (header.dstadr == MessageHeader.BROADCAST_ADR)
				forward_flag = True
			
			if forward_flag:  # Pass the message on?
				res = self.send(header.dstadr, msg, header.msg_id, header.srcadr, in_link_adr=link_adr)
		
		return delivery_flag, res, header, msg, link_adr


## Class: BinaryMessageChannel
## Abstract parent class for <MessageChannels> that send and receive INM messages
## encoded in the standard binary on-wire format.
class BinaryMessageChannel(MessageChannel):
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   srcadr - INM address of the local node. Will be placed in the source address
	##     header field of sent messages for which no other source address is provided.
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   msg_factory - The <MessageFactory> that the *BinaryMessageChannel* should use
	##     to create INM <Message> objects. If this parameter is *None*, a default
	##     <MessageFactory> will be used.
	##   selector - A selector object compatible with the ones provided by the standard
	##     library module *selectors*. This parameter is passed on to
	##     <MessageChannel.__init__>, see that entry for for more information.
	##   ch_num - Channel number of the *BinaryMessageChannel*. SHOULD be either *None*
	##     or a nonnegative integer.
	##   send_bfr_size - Initial capacity in bytes of the send buffer.
	##   recv_bfr_size - Initial capacity in bytes of the receive buffer.
	def __init__(self, srcadr, timeout=None, msg_factory=None, selector=None, ch_num=None,
	             send_bfr_size=0, recv_bfr_size=0):
		
		super().__init__(srcadr, timeout, msg_factory, selector, ch_num)
		
		# ISSUE: Does the send_bfr_size parameter help? Is previously allocated capacity
		# retained when a bytearray is cleared?
		self._send_buffer = bytearray(send_bfr_size)
		self._recv_buffer = bytearray(recv_bfr_size)
	
	# CAUTION: This is not any sort of thread safe.
	def _prepare_send_buffer(self, dstadr, srcadr, msg, msg_id=None):
		if msg_id is None:
			msg_id = self.get_next_msg_id()
		
		msg_id_b = self.msg_factory.make_val(msg_id, int_size=2)
		dstadr_b = self.msg_factory.make_val(dstadr, int_size=1)
		srcadr_b = self.msg_factory.make_val(srcadr, int_size=1)
		msg_typ_b = self.msg_factory.make_val(msg.typ, int_size=1)
		msg_len_b = self.msg_factory.make_val(len(msg.val), int_size=msg.MESSAGE_LEN_SIZE)
		
		self._send_buffer.clear()
		self._send_buffer.extend(msg_id_b)
		self._send_buffer.extend(dstadr_b)
		self._send_buffer.extend(srcadr_b)
		self._send_buffer.extend(msg_typ_b)
		self._send_buffer.extend(msg_len_b)
		self._send_buffer.extend(msg.val)
		
		# NOTE: Done for the benefit of calling code, which might want to call peek_prev_msg_id().
		self._prev_msg_id = msg_id
	
	def _parse_recv_header(self, n_bytes):
		if n_bytes < MessageHeader.HEADER_SIZE:
			return ResultCode.INVALID_HEADER, None, None, None, None
		
		msg_size = n_bytes - MessageHeader.HEADER_SIZE
		
		if msg_size < StandardMessage.MIN_MESSAGE_SIZE:
			return ResultCode.INVALID_MESSAGE, None, None, None, None
		
		#print(n_bytes) # DEBUG: 
		#print(self.msg_factory.format_val(self._recv_buffer[:n_bytes], ValueConversions.Hex)) # DEBUG: 
		
		i = 0
		msg_id = self.msg_factory.format_val(
			self._recv_buffer[i:i+MessageHeader.MESSAGE_ID_SIZE], ValueConversions.Int)
		i += MessageHeader.MESSAGE_ID_SIZE
		dstadr = self._recv_buffer[i]
		i += 1
		srcadr = self._recv_buffer[i]
		i += 1
		msg_typ = self._recv_buffer[i]
		i += 1
		
		# TODO: Check message IDs for duplicates.
		
		if msg_typ >= LargeMessage.MIN_TYPE_NUM:
			msg_len = self.msg_factory.format_val(
				self._recv_buffer[i:i+LargeMessage.MESSAGE_LEN_SIZE], ValueConversions.Int)
			i += LargeMessage.MESSAGE_LEN_SIZE
		else:
			msg_len = self._recv_buffer[i]
			i += 1
		
		header = MessageHeader(msg_id, dstadr, srcadr)
		return ResultCode.SUCCESS, header, msg_typ, msg_len, i
	
	def _parse_recv_value(self, n_bytes, msg_typ, msg_len, start_i):
		msg_val = bytes(self._recv_buffer[start_i:n_bytes])
		
		if msg_typ >= LargeMessage.MIN_TYPE_NUM:
			msg = LargeMessage(msg_typ, msg_val)
		else:
			msg = StandardMessage(msg_typ, msg_val)
		
		if len(msg.val) != msg_len:
			#print(f'{len(msg.val)} {msg_len}') # DEBUG: 
			#print(str(msg)) # DEBUG: 
			return ResultCode.INVALID_MESSAGE, msg
		
		return ResultCode.SUCCESS, msg
	
	def _parse_recv_buffer(self, n_bytes):
		res, header, msg_typ, msg_len, i = self._parse_recv_header(n_bytes)
		if res != ResultCode.SUCCESS:
			return res, dstadr, srcadr, None
		
		res, msg = self._parse_recv_value(n_bytes, msg_typ, msg_len, i)
		return res, header, msg


## Class: InetMessageChannel
## A <BinaryMessageChannel> that sends and receives messages via a UDP socket.
class InetMessageChannel(BinaryMessageChannel):
	## Variable: DEFAULT_IP_ADR
	## Default IP socket address.
	DEFAULT_IP_ADR = '127.0.0.1'
	
	## Variable: DEFAULT_UDP_PORT
	## Default port number of UDP socket.
	DEFAULT_UDP_PORT = 2357
	
	## Variable: DEFAULT_TCP_PORT
	## Default port number of TCP socket.
	DEFAULT_TCP_PORT = 2357
	
	## Variable: MAX_DATAGRAM_SIZE
	## Maximum size of sent UDP datagrams.
	MAX_DATAGRAM_SIZE = 0xffff
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   srcadr - INM address of the local node. Will be placed in the source address
	##     header field of sent messages for which no other source address is provided.
	##   ip_adr - IP address of sockets created by the *InetMessageChannel*. Specify
	##     the address as a hostname or dotted-decimal string. If this parameter is
	##     *None*, the value of <DEFAULT_IP_ADR> will be used.
	##   udp_port - Port number of UDP socket created by the *InetMessageChannel*.
	##     If this parameter is *None* or zero, the value of <DEFAULT_UDP_PORT> will
	##     be used.
	##   tcp_port - Port number of TCP socket created by the *InetMessageChannel*.
	##     If this parameter is *None* or zero, the value of <DEFAULT_TCP_PORT> will
	##     be used.
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   msg_factory - The <MessageFactory> that the *InetMessageChannel* should use
	##     to create INM <Message> objects. If this parameter is *None*, a default
	##     <MessageFactory> will be used.
	##   selector - A selector object compatible with the ones provided by the standard
	##     library module *selectors*. This parameter is passed on to
	##     <MessageChannel.__init__>, see that entry for for more information.
	##   ch_num - Channel number of the *InetMessageChannel*. SHOULD be either *None*
	##     or a nonnegative integer.
	def __init__(self, srcadr, ip_adr=None, udp_port=None, tcp_port=None,
		timeout=None, msg_factory=None, selector=None, ch_num=None):
		
		if ip_adr is None:
			ip_adr = self.DEFAULT_IP_ADR
		if udp_port is None or udp_port == 0:
			udp_port = self.DEFAULT_UDP_PORT
		if tcp_port is None or tcp_port == 0:
			tcp_port = self.DEFAULT_TCP_PORT
		
		super().__init__(srcadr, timeout, msg_factory, selector, ch_num, 0, self.MAX_DATAGRAM_SIZE)
		
		## Property: ip_adr
		## IP address of sockets used by this *InetMessageChannel*, as a hostname or dotted-decimal
		## string. This SHOULD be treated as a read-only attribute.
		self.ip_adr = ip_adr
		
		## Property: udp_port
		## Port number of UDP socket used by this *InetMessageChannel*. This SHOULD be treated as
		## a read-only attribute.
		self.udp_port = udp_port
		
		## Property: tcp_port
		## Port number of TCP socket used by this *InetMessageChannel*. This SHOULD be treated as
		## a read-only attribute.
		##
		## NOTE: No TCP socket is actually created or used by the current version of this class.
		self.tcp_port = tcp_port
		
		self._udp_socket = None
		self._udp_selector_key = None
		self._tcp_socket = None
		self._tcp_selector_key = None
	
	## Method: __str__
	## String conversion. Overrides <MessageChannel.__str__>.
	##
	## Returns:
	##   A string representation of this *InetMessageChannel*, in the format
	##   "class_name(srcadr, ip_adr, udp_port)".
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr}, {self.ip_adr}, {self.udp_port})'
	
	## Method: open
	## Overrides <MessageChannel.open>.
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		
		udp_adr = (self.ip_adr, self.udp_port)
		
		udp_socket = socket.socket(type=socket.SOCK_DGRAM)
		
		if self.timeout is not None:
			timeout_seconds = self.timeout.total_seconds()
			udp_socket.settimeout(timeout_seconds)
		
		try:
			udp_socket.bind(udp_adr)
		except Exception:
			return ResultCode.LINK_FAILURE
		
		# TODO: Implement TCP transport for large messages.
		
		self._udp_socket = udp_socket
		self._udp_selector_key = self._register_selectable(self._udp_socket, self.ch_num)
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	## Method: close
	## Overrides <MessageChannel.close>.
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self._udp_socket is not None:
			self._unregister_selectable(self._udp_socket)
			
			self._udp_socket.close()
			self._udp_socket = None
			self._udp_selector_key = None
		
		# TODO: Shut down and close the TCP socket, if it exists.
	
	## Method: get_selector_keys
	## Returns the selector key of the UDP socket.
	## Overrides <MessageChannel.get_selector_keys>. 
	def get_selector_keys(self):
		if self._udp_selector_key is None:
			return ()
		return (self._udp_selector_key,)
	
	## Method: set_timeout
	## Overrides <MessageChannel.set_timeout>.
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if not self.is_open():
			return
		
		if self.timeout is not None:
			timeout_seconds = self.timeout.total_seconds()
		else:
			timeout_seconds = None
		
		self._udp_socket.settimeout(timeout_seconds)
	
	## Method: send
	## Implements <MessageChannel.send>.
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is None:
			return ResultCode.INVALID_ARGUMENT
		
		# TODO: Implement TCP transport for large messages.
		
		self._prepare_send_buffer(dstadr, srcadr, msg, msg_id)
		
		n_bytes = self._udp_socket.sendto(self._send_buffer, link_adr)
		
		if n_bytes != len(self._send_buffer):
			return ResultCode.LINK_FAILURE
		
		return ResultCode.SUCCESS
	
	## Method: recv
	## Implements <MessageChannel.recv>.
	def recv(self):
		try:
			# ISSUE: Does this always provide either nothing or a complete datagram?
			# NOTE: "man udp" seems to indicate that this is the case:
			#         "All receive operations return only one packet."
			#       Nothing is mentioned or implied about partial packets being
			#       received. That makes sense, since it would just be a PITA for
			#       no good reason.
			n_bytes, link_adr = self._udp_socket.recvfrom_into(
				self._recv_buffer, len(self._recv_buffer))
		except socket.timeout:
			return ResultCode.RECV_TIMEOUT, None, None, None
		# NOTE: The Python socket module docs state the following:
		#   "In non-blocking mode, operations fail (with an error
		#    that is unfortunately system-dependent) if they cannot
		#    be completed immediately"
		except OSError:
			return ResultCode.RECV_FAILURE, None, None, None
		
		# TODO: Implement TCP transport for large messages.
		
		res, header, msg = self._parse_recv_buffer(n_bytes)
		
		return res, header, msg, link_adr


## Class: SerialMessageChannel
## A <BinaryMessageChannel> that sends and receives messages via a serial port.
class SerialMessageChannel(BinaryMessageChannel):
	## Variable: DEFAULT_BAUDRATE
	## Default baud rate to set on the opened serial port.
	DEFAULT_BAUDRATE = 38400
	#DEFAULT_BAUDRATE = 9600
	
	## Variable: N_HEADER_BYTES
	## Total size of the header, type and length fields of an INM message.
	N_HEADER_BYTES = MessageHeader.HEADER_SIZE + StandardMessage.MIN_MESSAGE_SIZE
	
	## Variable: MAX_MESSAGE_LEN
	## Maximum supported INM message (value) length.
	MAX_MESSAGE_LEN = 0xff - N_HEADER_BYTES
	
	## Variable: TIMEOUT_DIVISOR
	## If a receive timeout is set on a *SerialMessageChannel*, the timeout set for each
	## attempt to read data from the underlying serial port API will be the primary
	## timeout divided by *TIMEOUT_DIVISOR*. This is done because several reads may be
	## needed to obtain a complete message, so if the primary timeout was used for each
	## read, there could be significant overshoot.
	TIMEOUT_DIVISOR = 10.0
	
	## Method: __init__
	## Instance initializer.
	##
	## Parameters:
	##   srcadr - INM address of the local node. Will be placed in the source address
	##     header field of sent messages for which no other source address is provided.
	##   port - Filesystem path of the serial port device to open.
	##   baudrate - Baud rate to open the serial port at. If this parameter is *None*,
	##     the value of <DEFAULT_BAUDRATE> will be used.
	##   timeout - Receive timeout. MUST be an instance of the standard library class
	##     *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   msg_factory - The <MessageFactory> that the *SerialMessageChannel* should use
	##     to create INM <Message> objects. If this parameter is *None*, a default
	##     <MessageFactory> will be used.
	##   selector - A selector object compatible with the ones provided by the standard
	##     library module *selectors*. This parameter is passed on to
	##     <MessageChannel.__init__>, see that entry for for more information.
	##   ch_num - Channel number of the *SerialMessageChannel*. SHOULD be either *None*
	##     or a nonnegative integer.
	def __init__(self, srcadr, port, baudrate=None, timeout=None, msg_factory=None, selector=None, ch_num=None):
		if baudrate is None:
			baudrate = self.DEFAULT_BAUDRATE
		
		super().__init__(srcadr, timeout, msg_factory, selector, ch_num, 0, 0)
		
		## Property: port
		## Filesystem path of the used serial port device. This SHOULD be treated as
		## a read-only attribute.
		self.port = port
		
		## Property: baudrate
		## Baud rate to open the used serial port at. This SHOULD be treated as
		## a read-only attribute.
		self.baudrate = baudrate
		
		self._serial_dev = None
		self._serial_selector_key = None
		
		self._recv_header = None
		self._recv_msg_typ = None
		self._recv_msg_len = None
	
	## Method: __str__
	## String conversion. Overrides <MessageChannel.__str__>.
	##
	## Returns:
	##   A string representation of this *SerialMessageChannel*, in the format
	##   "class_name(srcadr, port)".
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr}, {self.port})'
	
	## Method: open
	## Overrides <MessageChannel.open>.
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		
		if self.timeout is not None:
			timeout_seconds = self.timeout.total_seconds() / self.TIMEOUT_DIVISOR
		else:
			timeout_seconds = None
		
		serial_dev = serial.Serial(baudrate=self.baudrate, timeout=timeout_seconds)
		
		try:
			serial_dev.port = self.port
			serial_dev.open()
		except Exception:
			return ResultCode.LINK_FAILURE
		
		self._serial_dev = serial_dev
		self._serial_selector_key = self._register_selectable(self._serial_dev, self.ch_num)
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	## Method: close
	## Overrides <MessageChannel.close>.
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self._serial_dev is not None:
			self._unregister_selectable(self._serial_dev)
			
			self._serial_dev.close()
			self._serial_dev = None
			self._serial_selector_key = None
	
	## Method: get_selector_keys
	## Returns the selector key of the serial port device.
	## Overrides <MessageChannel.get_selector_keys>. 
	def get_selector_keys(self):
		if self._serial_selector_key is None:
			return ()
		return (self._serial_selector_key,)
	
	## Method: set_timeout
	## Overrides <MessageChannel.set_timeout>.
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if not self.is_open():
			return
		
		if timeout is not None:
			timeout_seconds = timeout.total_seconds() / self.TIMEOUT_DIVISOR
		else:
			timeout_seconds = None
		
		self._serial_dev.timeout = timeout_seconds
	
	## Method: send
	## Implements <MessageChannel.send>.
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is not None: # Serial links are point-to-point.
			return ResultCode.INVALID_ARGUMENT
		
		if len(msg) > self.MAX_MESSAGE_LEN:
			return ResultCode.INVALID_MESSAGE
		
		self._prepare_send_buffer(dstadr, srcadr, msg, msg_id)
		
		n_bytes = self._serial_dev.write(self._send_buffer)
		
		#print(f'msg={str(msg)}') # DEBUG:
		#print(f'{len(self._send_buffer)} {self._send_buffer.hex()}') # DEBUG: 
		#print(f'n_bytes={n_bytes}') # DEBUG: 
		
		if n_bytes != len(self._send_buffer):
			return ResultCode.LINK_FAILURE
		
		return ResultCode.SUCCESS
	
	## Method: recv
	## Implements <MessageChannel.recv>.
	def recv(self):
		if self.timeout is not None:
			timeout_remaining = self.timeout
			t0 = get_timestamp()
		
		while True:
			if len(self._recv_buffer) < self.N_HEADER_BYTES: # Receive header.
				bytes_to_read = self.N_HEADER_BYTES - len(self._recv_buffer)
				header_bytes = self._serial_dev.read(bytes_to_read)
				
				if len(header_bytes) > 0:
					self._recv_buffer.extend(header_bytes)
					
					if len(self._recv_buffer) == self.N_HEADER_BYTES:
						res, header, msg_typ, msg_len, i = self._parse_recv_header(
							len(self._recv_buffer))
						
						if res != ResultCode.SUCCESS:
							return res
						
						self._recv_header = header
						self._recv_msg_typ = msg_typ
						self._recv_msg_len = msg_len
			else: # Header received. Receive value bytes.
				n_message_bytes = self.N_HEADER_BYTES + self._recv_msg_len
				
				if len(self._recv_buffer) < n_message_bytes:
					bytes_to_read = n_message_bytes - len(self._recv_buffer)
					value_bytes = self._serial_dev.read(bytes_to_read)
					if len(value_bytes) > 0:
						self._recv_buffer.extend(value_bytes)
				
				if len(self._recv_buffer) == n_message_bytes:
					res, msg = self._parse_recv_value(
						len(self._recv_buffer), self._recv_msg_typ, self._recv_msg_len,
						self.N_HEADER_BYTES)
					
					if res != ResultCode.SUCCESS:
						return res
					
					header = self._recv_header
					
					self._recv_buffer.clear()
					self._recv_header = None
					self._recv_msg_typ = None
					self._recv_msg_len = None
		
					return ResultCode.SUCCESS, header, msg, None
			
			#print(f'{len(self._recv_buffer)} {self._recv_buffer.hex()}') # DEBUG: 
			
			if self.timeout is not None:
				t1 = get_timestamp()
				dt = t1 - t0
				timeout_remaining -= dt
				t0 = t1
				
				if timeout_remaining <= ZERO_DURATION:
					return ResultCode.RECV_TIMEOUT, None, None, None
