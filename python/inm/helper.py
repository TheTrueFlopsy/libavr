
from . import inm

## File: helper.py
## The *helper* module contains the <InmHelper>, a convenience class
## for INM communication.

# ISSUE: Correctly entering INM commands at the Python prompt is still
#        tedious and error-prone. For example, to turn on a LED via the
#        "inm_node_demo.c" Uno program, one has to enter the not very
#        transparent or terse
#          h.send_mval(1, 0x43, (0x0100, 0, 1), (2, 1, 1))
#        In other words "Send a SET_VAR message (type 0x43) to node 1,
#        with a payload consisting of the 16-bit variable number 0x0100,
#        followed by the 8-bit index 0 and the 8-bit variable value 1."
#        Symbolic constants would help, but not very much.

# IDEA: Develop a command language for the INM protocol, with constructs
#       that allow type definitions (message types, payload structs, etc.)
#       directly via the language interpreter.
#         adr uno 1                          # INM address 1.
#         msg setvar 0x43                    # Message type 0x43.
#         data var_h [id:2, index:1]         # Data type, 2 fields, 2-byte, 1-byte.
#         data var8 [head:var_h, val:1]      # Data type, 2 fields: var_h, 1-byte.
#         tmpl var8_t:var8 [[_, _], _]        # Equivalent to implicit template.
#         tmpl led0:var8 [0x0100, 0, _]       # Template, type var8, field 2 unspecified.
#         tmpl led0:var8 [[0x0100, 0], _]     # Equivalent. Matches defined structure.
#         tmpl led0:var8 [0x0100, [0, _]]     # Error! Violates defined structure.
#         led0off := led0 [_, _, 0]      # Value, template led0, field 0 and 1 unspecified.
#         led0on:var8 := led0 [_, _, 1]  # Type specifier optional, must match template.
#         led0on := led0 1  # Equivalent. Additional args substituted for open template slots
#                                 from left to right.
#         show setvar
#           msg setvar 0x43
#         show var_h
#           data var_h [id:2, index:1]
#         show led0on     # Show maximally specified, omit template reference.
#           led0on:var8 := [head:var_h [id:2 0x0100, index:1 0x00], val:1 0x01]
#         show -t led0on  # Show only directly specified fields, include template reference.
#           led0on:var8 := led0 [head:var_h _, val:1 0x01]
#         show -v led0on  # Show maximally specified, include template reference.
#           led0on:var8 := led0 [head:var_h [id:2 0x0100, index:1 0x00], val:1 0x01]
#         show led0
#           tmpl led0:var8 [head:var_h [id:2 0x0100, index:1 0x00], val:1 _]
#         send uno setvar led0on  # Send message with type setvar and payload led0on.
#         send uno setvar led0 1  # Equivalent. One-off template substitution.
#         send uno setvar led0  # Error! Value not fully specified.
#         send uno setvar var8 0x200 0 0x11  # Data type works as template.

# ISSUE: Will the algorithm that checks substructure decompositions for compatibility
#        be slow or complicated? Are there fundamentally ambiguous cases?
# IDEA: Structure A is compatible with parent structure B iff the flattening of A is
#       equal to that of B and A only specifies substructures that are also specified in B.
#       (The second condition applies even if the substructure that is missing in B
#       conforms to hierarchy (i.e. either is contained in, contains or doesn't intersect
#       every substructure in B).)
# NOTE: The flattening of a structure is obtained by decomposing it fully into a sequence
#       of simple (i.e. non-composite) fields of known size.

# TODO: Verify that the code example works.
## Class: InmHelper
## Convenience class for INM communication. Encapsulates a <MessageChannel> and provides
## a simplified interface for sending and receiving messages via that channel.
##
## An *InmHelper* can be used as a context manager, opening the encapsulated
## <MessageChannel> when the context is entered and then closing the channel
## upon exit from the context.
##
## Code Example:
## > import inm
## > import inm.helper
## >
## > T  = inm.StandardTypes      # standard INM message types
## > Rs = inm.StandardResults    # INM protocol result codes (sent in responses)
## > G  = inm.StandardRegisters  # standard logical registers for INM nodes
## > Rc = inm.ResultCode         # INM module result codes (returned by methods)
## > C  = inm.ValueConversions   # INM message field type conversions
## > dstadr = 10
## > debug0_val = 0xdb
## >
## > with inm.helper.InmHelper() as h:  # Create InmHelper with default message channel.
## >   # Ask node 10 about its firmware version.
## >   res, header, msg, link_adr = h.sendrecv(dstadr, T.REG_READ, G.FWVERSION)
## >
## >   if res == Rc.SUCCESS:  # send/receive operation succeeded
## >     r_index, fw_ver, req_id = msg.format_mval(conv=C.Int)  # Unpack INM_REG_READ_RES payload.
## >     print(f'Node {dstadr} has firmware version {fw_ver}.')
## >
## >   # Write to debug register 0 at node 10.
## >   res, header, msg, link_adr = h.sendrecv_mval(dstadr, T.REG_WRITE, (G.DEBUG0, debug0_val))
## >
## >   if res == Rc.SUCCESS:  # send/receive operation succeeded
## >     std_res, req_id = msg.format_mval(conv=C.Int)  # Unpack INM_RESULT payload.
## >     if std_res = Rs.OK:  # successful register write at destination
## >       print(f'Wrote {debug0_val:#02x} to debug register 0.')
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
	##     the class attribute *DEFAULT_SRCADR* will be used.
	##   ip_adr - IP address of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. If this parameter is *None*, the value of
	##     the class attribute *DEFAULT_IP_ADR* will be used.
	##   udp_port - UDP port of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. If this parameter is *None*, the value of
	##     the class attribute *DEFAULT_UDP_PORT* will be used.
	##   tcp_port - TCP port of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. If this parameter is *None*, the port
	##     number used for the UDP port is also used for the TCP port.
	##
	##     NOTE: In the current implementation of <InetMessageChannel>, no TCP socket
	##     is actually used, so this parameter makes no difference.
	##   timeout - Receive timeout of the internal <InetMessageChannel> that is created if
	##     the *channel* parameter is *None*. Should be an instance of the standard library
	##     class *datetime.timedelta*. If this parameter is *None*, receive operations will
	##     never time out.
	##   link_adr - INM link address (think of it as the router address of an IP network)
	##     of the internal <InetMessageChannel> that is created if the *channel* parameter
	##     is *None*. Should be a tuple of the format *(link_ip_adr, link_udp_port)*.
	##     If this parameter is *None*, the used *link_ip_adr* will be the same as the IP
	##     address of the encapsulated channel, while *link_udp_port* will have the value
	##     of the class attribute *DEFAULT_LINK_UDP_PORT*.
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
		if typ >= inm.StandardMessage.MIN_TYPE_NUM and typ <= inm.StandardMessage.MAX_TYPE_NUM:
			msg = self.msg_factory.make_msg(typ, val, int_size)
		elif typ >= inm.LargeMessage.MIN_TYPE_NUM and typ <= inm.LargeMessage.MAX_TYPE_NUM:
			msg = self.msg_factory.make_large_msg(typ, val, int_size)
		else:
			return inm.ResultCode.INVALID_ARGUMENT
		
		if link_adr is None:
			link_adr = self.link_adr
		
		return self.channel.send(dstadr, msg, msg_id, srcadr, link_adr)
	
	## Method: send_mval
	## Constructs and sends an INM message with a multipart message value.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   mval - Multipart INM message value. Should be a list or tuple of values.
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
		if typ >= inm.StandardMessage.MIN_TYPE_NUM and typ <= inm.StandardMessage.MAX_TYPE_NUM:
			msg = self.msg_factory.make_msg_mval(typ, mval, int_size)
		elif typ >= inm.LargeMessage.MIN_TYPE_NUM and typ <= inm.LargeMessage.MAX_TYPE_NUM:
			msg = self.msg_factory.make_large_msg_mval(typ, mval, int_size)
		else:
			return inm.ResultCode.INVALID_ARGUMENT
		
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
		if res != inm.ResultCode.SUCCESS:
			return res, None, None, None
		
		return self.recv()
	
	## Method: sendrecv_mval
	## First constructs and sends an INM message with a multipart value,
	## then attempts to receive a message.
	##
	## Parameters:
	##   dstadr - Destination INM node address of the message to send.
	##   typ - INM message type.
	##   mval - Multipart INM message value. Should be a list or tuple of values.
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
		if res != inm.ResultCode.SUCCESS:
			return res, None, None, None
		
		return self.recv()
