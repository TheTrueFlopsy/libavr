
from . import inm

## File: helper.py
## The *helper* module contains the <InmHelper>, a convenience class
## for INM communication.
##
## CAUTION: This module is NOT inherently thread-safe. In a multi-threaded
## application, access to thread-shared objects from this module MUST be
## synchronized in application code.


## Class: InmHelper
## Convenience class for INM communication. Encapsulates a <MessageChannel> and provides
## a simplified interface for sending and receiving messages via that channel.
##
## An *InmHelper* can be used as a context manager, opening the encapsulated
## <MessageChannel> when the context is entered and then closing the channel
## upon exit from the context. If the attempt to open the message channel fails,
## a <MessageChannelError> is thrown by the *__enter__* method.
##
## Code Example:
## > import inm.helper as helper
## >
## > dstadr = 1
## > debug0_val = 0x02
## >
## > h = helper.InmHelper()  # Create InmHelper with default message channel.
## >
## > res = h.open()  # Open the message channel.
## > if res != h.RC.SUCCESS:  # Use convenient shorthand reference to enum type.
## >   print(f'Failed to open message channel: {res.name}')
## >
## > # Ask INM node 1 about its firmware version.
## > res, header, msg, link_adr = h.sendrecv(dstadr, h.Typ.REG_READ, h.Reg.FWVERSION)
## > if res != h.RC.SUCCESS:
## >   print(f'Failed to read from FWVERSION: {res.name}')
## >
## > r_index, fw_ver, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_REG_READ_RES payload.
## > print(f'Node {dstadr} has firmware version {fw_ver}.')
## >
## > # Write to debug register 0 at node 1.
## > res, header, msg, link_adr = h.sendrecv_mval(dstadr, h.Typ.REG_WRITE, (h.Reg.DEBUG0, debug0_val))
## > if res != h.RC.SUCCESS:
## >   print(f'Failed to write to DEBUG0: {res.name}')
## >
## > std_res, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_RESULT payload.
## > if std_res == h.Res.OK:  # successful register write at destination
## >   print(f'Wrote {debug0_val:#04x} to DEBUG0.')
## >
## > h.close()  # Close the message channel.
class InmHelper:
	## Variable: DEFAULT_SRCADR
	## Default INM source address.
	DEFAULT_SRCADR = 99
	
	## Variable: DEFAULT_IP_ADR
	## Default IP address of internal <InetMessageChannel>.
	DEFAULT_IP_ADR = '127.0.0.1'
	
	## Variable: DEFAULT_UDP_PORT
	## Default UDP port of internal <InetMessageChannel>.
	DEFAULT_UDP_PORT = 2999
	
	## Variable: DEFAULT_LINK_UDP_PORT
	## Default target UDP port of internal <InetMessageChannel>.
	DEFAULT_LINK_UDP_PORT = 3000
	
	## Variable: Typ
	## Shorthand alias of <StandardTypes>.
	Typ = inm.StandardTypes
	
	## Variable: Res
	## Shorthand alias of <StandardResults>.
	Res = inm.StandardResults
	
	## Variable: Reg
	## Shorthand alias of <StandardRegisters>.
	Reg = inm.StandardRegisters
	
	## Variable: RC
	## Shorthand alias of <ResultCode>.
	RC = inm.ResultCode
	
	## Variable: VC
	## Shorthand alias of <ValueConversions>.
	VC = inm.ValueConversions
	
	_SM_MIN_TYPE = inm.StandardMessage.MIN_TYPE_NUM
	_SM_MAX_TYPE = inm.StandardMessage.MAX_TYPE_NUM
	_LM_MIN_TYPE = inm.LargeMessage.MIN_TYPE_NUM
	_LM_MAX_TYPE = inm.LargeMessage.MAX_TYPE_NUM
	
	## Method: __init__
	## Instance initializer. Offers convenient default behaviors. For example, creating
	## an *InmHelper* with zero arguments should result in an internal <InetMessageChannel>
	## with a default debugging configuration (i.e. use UDP port 2999 to communicate with
	## port 3000 on the loopback interface, as INM node 99).
	##
	## Parameters:
	##   channel - The <MessageChannel> that the *InmHelper* should use to send and receive
	##     INM messages. If this is *None*, the other parameters will be used to initialize
	##     an internally created <InetMessageChannel>.
	##   msg_factory - The <MessageFactory> that the *InmHelper* should use to create INM
	##     <Message> objects. If this parameter is *None*, the *msg_factory* attribute
	##     of the encapsulated <MessageChannel> will be used.
	##   srcadr - INM source address of the encapsulated <MessageChannel>. Used only if
	##     the *channel* parameter is *None*. If this parameter is *None*, the value of
	##     the class attribute <DEFAULT_SRCADR> will be used.
	##   ip_adr - IP address of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. Specify the address as a hostname or
	##     dotted-decimal string. If this parameter is *None*, the value of the class
	##     attribute <DEFAULT_IP_ADR> will be used.
	##   udp_port - UDP port of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. If this parameter is *None*, the value of
	##     the class attribute <DEFAULT_UDP_PORT> will be used.
	##   tcp_port - TCP port of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. If this parameter is *None*, the port
	##     number used for the UDP port is also used for the TCP port.
	##     NOTE: In the current implementation of <InetMessageChannel>, no TCP socket
	##     is actually used, so this parameter makes no difference.
	##   timeout - Receive timeout of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. MUST be an instance of the standard library
	##     class *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   link_adr - INM link address (think of it as the router address of an IP network)
	##     of the internal <InetMessageChannel> that is created if the *channel* parameter
	##     is *None*. Should be a tuple of the format *(link_ip_adr, link_udp_port)*.
	##     If this parameter is *None*, the used *link_ip_adr* will be the same as the IP
	##     address of the encapsulated channel, while *link_udp_port* will have the value
	##     of the class attribute <DEFAULT_LINK_UDP_PORT>.
	def __init__(self, channel=None, msg_factory=None,
	             srcadr=None, ip_adr=None, udp_port=None, tcp_port=None,
	             timeout=None, link_adr=None):
		
		if channel is None:
			if srcadr is None:
				srcadr = 99
			
			if ip_adr is None:
				ip_adr = '127.0.0.1'
			
			if udp_port is None:
				udp_port = 2999
			
			if tcp_port is None:
				tcp_port = udp_port
			
			if link_adr is None:
				link_adr = (ip_adr, 3000)
			
			channel = inm.InetMessageChannel(srcadr, ip_adr, udp_port, tcp_port, timeout, msg_factory)
		
		if msg_factory is None:
			msg_factory = channel.msg_factory
		
		## Property: channel
		## The encapsulated <MessageChannel> of this *InmHelper*.
		self.channel = channel
		
		## Property: msg_factory
		## The encapsulated <MessageFactory> of this *InmHelper*.
		self.msg_factory = msg_factory
		
		## Property: link_adr
		## The INM link address this *InmHelper* uses to send messages.
		self.link_adr = link_adr
	
	def __enter__(self):
		if not self.channel.is_open():
			self.open()
		return self
	
	def __exit__(self, exc_type, exc_value, traceback):
		self.close()
		return False
	
	def _make_header(self, dstadr, msg_id=None, srcadr=None):
		if msg_id is None:
				msg_id = self.channel.peek_next_msg_id()
		
		if srcadr is None:
			srcadr = self.channel.srcadr
		
		return inm.MessageHeader(msg_id, dstadr, srcadr)
	
	## Method: open
	## Opens the encapsulated <channel>. Calls <MessageChannel.open>.
	##
	## Returns:
	##   <ResultCode.SUCCESS> if the channel was successfully opened, otherwise another
	##   <ResultCode> indicating what went wrong.
	def open(self):
		return self.channel.open()
	
	## Method: close
	## Closes the encapsulated <channel>. Calls <MessageChannel.close>.
	def close(self):
		self.channel.close()
	
	## Method: header_to_bytes
	## Constructs an INM message header and returns it as a bytes object. Useful for
	## finding out the exact on-wire representation of a header.
	##
	## Parameters:
	##   dstadr - Destination INM node address in the header to construct.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address in the header to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##
	## Returns:
	##   A bytes object containing the standard binary on-wire representation of the
	##   specified INM message header.
	def header_to_bytes(self, dstadr, msg_id=None, srcadr=None):
		header = self._make_header(dstadr, msg_id, srcadr)
		return header.to_bytes()
	
	## Method: msg_to_bytes
	## Constructs an INM message and returns it as a bytes object. Useful for finding
	## out the exact on-wire representation of a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to construct.
	##   typ - INM message type.
	##   val - INM message value.
	##   int_size - Field size in bytes of an integer message value. If this is *None*,
	##     the default *int_size* of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address of the message to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   with_header - Boolean flag specifying whether an INM header should be prepended
	##     to the constructed message.
	##
	## Returns:
	##   A bytes object containing the standard binary on-wire representation of the
	##   specified INM message. (Or *None*, if the message couldn't be constructed.)
	def msg_to_bytes(self, dstadr, typ, val, int_size=None, msg_id=None, srcadr=None, with_header=True):
		if self._SM_MIN_TYPE <= typ <= self._SM_MAX_TYPE:
			msg = self.msg_factory.make_msg(typ, val, int_size)
		elif self._LM_MIN_TYPE <= typ <= self._LM_MAX_TYPE:
			msg = self.msg_factory.make_large_msg(typ, val, int_size)
		else:
			return None
		
		if msg is None:
			return None
		
		header = self._make_header(dstadr, msg_id, srcadr) if with_header else None
		
		return self.msg_factory.msg_to_bytes(msg, header)
	
	## Method: msg_to_bytes_mval
	## Constructs an INM message with a multipart message value and returns it as a bytes
	## object. Useful for finding out the exact on-wire representation of a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to construct.
	##   typ - INM message type.
	##   mval - Multipart INM message value. MUST be a sequence.
	##   int_size - Field size in bytes of an integer message value. This may be
	##     a list or tuple containing a separate integer size for each element
	##     of the *val* argument. If this is *None*, the default *int_size*
	##     of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address of the message to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   with_header - Boolean flag specifying whether an INM header should be prepended
	##     to the constructed message.
	##
	## Returns:
	##   A bytes object containing the standard binary on-wire representation of the
	##   specified INM message. (Or *None*, if the message couldn't be constructed.)
	def msg_to_bytes_mval(self, dstadr, typ, mval, int_size=None, msg_id=None, srcadr=None, with_header=True):
		if self._SM_MIN_TYPE <= typ <= self._SM_MAX_TYPE:
			msg = self.msg_factory.make_msg_mval(typ, mval, int_size)
		elif self._LM_MIN_TYPE <= typ <= self._LM_MAX_TYPE:
			msg = self.msg_factory.make_large_msg_mval(typ, mval, int_size)
		else:
			return None
		
		if msg is None:
			return None
		
		header = self._make_header(dstadr, msg_id, srcadr) if with_header else None
		
		return self.msg_factory.msg_to_bytes(msg, header)
	
	## Method: header_to_hex
	## Constructs an INM message header and returns it as a list of hex strings.
	## Useful for finding out the exact on-wire representation of a header.
	##
	## Parameters:
	##   dstadr - Destination INM node address in the header to construct.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address in the header to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##
	## Returns:
	##   A list of two-character hexadecimal strings, corresponding to bytes in
	##   the standard binary on-wire representation of the specified INM message header.
	def header_to_hex(self, dstadr, msg_id=None, srcadr=None):
		header_bytes = self.header_to_bytes(dstadr, msg_id, srcadr)
		return [f'{b:02x}' for b in header_bytes]
	
	## Method: msg_to_hex
	## Constructs an INM message and returns it as a list of hex strings.
	## Useful for finding out the exact on-wire representation of a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to construct.
	##   typ - INM message type.
	##   val - INM message value.
	##   int_size - Field size in bytes of an integer message value. If this is *None*,
	##     the default *int_size* of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address of the message to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   with_header - Boolean flag specifying whether an INM header should be prepended
	##     to the constructed message.
	##
	## Returns:
	##   A list of two-character hexadecimal strings, corresponding to bytes in
	##   the standard binary on-wire representation of the specified INM message.
	##   (Or *None*, if the message couldn't be constructed.)
	def msg_to_hex(self, dstadr, typ, val, int_size=None, msg_id=None, srcadr=None, with_header=True):
		msg_bytes = self.msg_to_bytes(dstadr, typ, val, int_size, msg_id, srcadr, with_header)
		if msg_bytes is None:
			return None
		
		return [f'{b:02x}' for b in msg_bytes]
	
	## Method: msg_to_hex_mval
	## Constructs an INM message with a multipart message value and returns it as
	## list of hex strings. Useful for finding out the exact on-wire representation
	## of a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to construct.
	##   typ - INM message type.
	##   mval - Multipart INM message value. MUST be a sequence.
	##   int_size - Field size in bytes of an integer message value. This may be
	##     a list or tuple containing a separate integer size for each element
	##     of the *val* argument. If this is *None*, the default *int_size*
	##     of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, the next message ID to be generated
	##     by the <channel> will be used.
	##   srcadr - Source INM node address of the message to construct. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   with_header - Boolean flag specifying whether an INM header should be prepended
	##     to the constructed message.
	##
	## Returns:
	##   A list of two-character hexadecimal strings, corresponding to bytes in
	##   the standard binary on-wire representation of the specified INM message.
	##   (Or *None*, if the message couldn't be constructed.)
	def msg_to_hex_mval(self, dstadr, typ, mval, int_size=None, msg_id=None, srcadr=None, with_header=True):
		msg_bytes = self.msg_to_bytes_mval(dstadr, typ, mval, int_size, msg_id, srcadr, with_header)
		if msg_bytes is None:
			return None
		
		return [f'{b:02x}' for b in msg_bytes]
	
	## Method: send
	## Constructs and sends an INM message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   val - INM message value.
	##   int_size - Field size in bytes of an integer message value. If this is *None*,
	##     the default *int_size* of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     by the <channel>.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the value of the <link_adr> attribute will be used.
	##
	## Returns:
	##   A <ResultCode> indicating the outcome of the attempted send operation.
	def send(self, dstadr, typ, val, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		if self._SM_MIN_TYPE <= typ <= self._SM_MAX_TYPE:
			msg = self.msg_factory.make_msg(typ, val, int_size)
		elif self._LM_MIN_TYPE <= typ <= self._LM_MAX_TYPE:
			msg = self.msg_factory.make_large_msg(typ, val, int_size)
		else:
			return self.RC.INVALID_ARGUMENT
		
		if msg is None:
			return self.RC.INVALID_ARGUMENT
		
		if link_adr is None:
			link_adr = self.link_adr
		
		return self.channel.send(dstadr, msg, msg_id, srcadr, link_adr)
	
	## Method: send_mval
	## Constructs and sends an INM message with a multipart message value.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   mval - Multipart INM message value. MUST be a sequence.
	##   int_size - Field size in bytes of an integer message value. This may be
	##     a list or tuple containing a separate integer size for each element
	##     of the *val* argument. If this is *None*, the default *int_size*
	##     of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     by the <channel>.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the value of the <link_adr> attribute will be used.
	##
	## Returns:
	##   A <ResultCode> indicating the outcome of the attempted send operation.
	def send_mval(self, dstadr, typ, mval, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		if self._SM_MIN_TYPE <= typ <= self._SM_MAX_TYPE:
			msg = self.msg_factory.make_msg_mval(typ, mval, int_size)
		elif self._LM_MIN_TYPE <= typ <= self._LM_MAX_TYPE:
			msg = self.msg_factory.make_large_msg_mval(typ, mval, int_size)
		else:
			return self.RC.INVALID_ARGUMENT
		
		if msg is None:
			return self.RC.INVALID_ARGUMENT
		
		if link_adr is None:
			link_adr = self.link_adr
		
		return self.channel.send(dstadr, msg, msg_id, srcadr, link_adr)
	
	## Method: recv
	## Attempts to receive an INM message.
	##
	## Returns:
	##   res - A <ResultCode> indicating the outcome of the attempted receive operation.
	##   header - The <MessageHeader> of the received message, or *None* in case of failure.
	##   msg - The received <Message>, or *None* in case of failure.
	##   link_adr - The link address of the received message, or *None* in case of failure.
	def recv(self):
		return self.channel.recv()
	
	## Method: sendrecv
	## First constructs and sends an INM message, then attempts to receive one.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   val - INM message value.
	##   int_size - Field size in bytes of an integer message value. If this is *None*,
	##     the default *int_size* of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     by the <channel>.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the value of the <link_adr> attribute will be used.
	##
	## Returns:
	##   res - A <ResultCode> indicating the outcome of the attempted send/receive operation.
	##   header - The <MessageHeader> of the received message, or *None* in case of failure.
	##   msg - The received <Message>, or *None* in case of failure.
	##   link_adr - The link address of the received message, or *None* in case of failure.
	def sendrecv(self, dstadr, typ, val, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		res = self.send(dstadr, typ, val, int_size, msg_id, srcadr, link_adr)
		if res != self.RC.SUCCESS:
			return res, None, None, None
		
		return self.recv()
	
	## Method: sendrecv_mval
	## First constructs and sends an INM message with a multipart value,
	## then attempts to receive a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   mval - Multipart INM message value. MUST be a sequence.
	##   int_size - Field size in bytes of an integer message value. This may be
	##     a list or tuple containing a separate integer size for each element
	##     of the *val* argument. If this is *None*, the default *int_size*
	##     of the <msg_factory> will be used.
	##   msg_id - INM message ID. If this is *None*, a message ID will be generated
	##     by the <channel>.
	##   srcadr - Source INM node address of the message to send. If this is *None*,
	##     the configured source address of the <channel> will be used.
	##   link_adr - INM link address to use when sending the message. If this is *None*,
	##     the value of the <link_adr> attribute will be used.
	##
	## Returns:
	##   res - A <ResultCode> indicating the outcome of the attempted send/receive operation.
	##   header - The <MessageHeader> of the received message, or *None* in case of failure.
	##   msg - The received <Message>, or *None* in case of failure.
	##   link_adr - The link address of the received message, or *None* in case of failure.
	def sendrecv_mval(self, dstadr, typ, mval, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		res = self.send_mval(dstadr, typ, mval, int_size, msg_id, srcadr, link_adr)
		if res != self.RC.SUCCESS:
			return res, None, None, None
		
		return self.recv()
