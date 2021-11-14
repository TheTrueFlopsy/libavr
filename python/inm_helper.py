
import inm

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
#         val var8_t:var8 [[_, _], _]        # Equivalent to implicit template.
#         val led0:var8 [0x0100, 0, _]       # Template, type var8, field 2 unspecified.
#         val led0:var8 [[0x0100, 0], _]     # Equivalent. Matches defined structure.
#         val led0:var8 [0x0100, [0, _]]     # Error! Violates defined structure.
#         val led0off := led0 [_, _, 0]      # Value, template led0, field 0 and 1 unspecified.
#         val led0on:var8 := led0 [_, _, 1]  # Type specifier optional, must match template.
#         val led0on := led0 1  # Equivalent. Additional args substituted for open template slots
#                                 from left to right.
#         show setvar
#           msg setvar 0x43
#         show var_h
#           data var_h [id:2, index:1]
#         show led0on     # Show maximally specified, omit template reference.
#           val led0on:var8 [head:var_h [id:2 0x0100, index:1 0x00], val:1 0x01]
#         show -t led0on  # Show only directly specified fields, include template reference.
#           val led0on:var8 := led0 [head:var_h _, val:1 0x01]
#         show -v led0on  # Show maximally specified, include template reference.
#           val led0on:var8 := led0 [head:var_h [id:2 0x0100, index:1 0x00], val:1 0x01]
#         show led0
#           val led0:var8 [head:var_h [id:2 0x0100, index:1 0x00], val:1 _]
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

class InmHelper:
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
		
		self.channel = channel
		self.msg_factory = msg_factory
		self.link_adr = link_adr
	
	def __enter__(self):
		if not self.channel.is_open():
			self.open()
		return self
	
	def __exit__(self, exc_type, exc_value, traceback):
		self.close()
		return False
	
	def open(self):
		return self.channel.open()
	
	def close(self):
		self.channel.close()
	
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
	
	def recv(self):
		return self.channel.recv()
	
	def sendrecv(self, dstadr, typ, val, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		res = self.send(dstadr, typ, val, int_size, msg_id, srcadr, link_adr)
		if res != inm.ResultCode.SUCCESS:
			return res, None, None, None
		
		return self.recv()
	
	def sendrecv_mval(self, dstadr, typ, mval, int_size=None, msg_id=None, srcadr=None, link_adr=None):
		res = self.send_mval(dstadr, typ, mval, int_size, msg_id, srcadr, link_adr)
		if res != inm.ResultCode.SUCCESS:
			return res, None, None, None
		
		return self.recv()
