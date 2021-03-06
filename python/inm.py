
import datetime
import enum
import socket
import selectors

import serial

# TODO: Document this module.

_now = datetime.datetime.today
ZERO_DURATION = datetime.timedelta(0)

def get_timedelta(secs=0, musecs=0):
	return datetime.timedelta(seconds=secs, microseconds=musecs)

def bytes_to_binary(bs, big_endian=False):
	if big_endian:
		bs = reversed(bs)
	return ''.join(f'{b:08b}' for b in bs)
	#return ''.join('{0:08b}'.format(b) for b in bs)

def map_enum(enum_class, i, defval=None):
	if i in (e.value for e in enum_class):
		return enum_class(i)
	else:
		return defval


@enum.unique
class StandardTypes(enum.IntEnum):
	DEFAULT          = 0x00
	RESULT           = 0x01
	INM_RESULT       = 0x02
	REG_READ         = 0x03
	REG_READ_RES     = 0x04
	REG_WRITE        = 0x05
	REG_TOGGLE       = 0x06
	REG_RW_EXCH      = 0x07
	REG_WR_EXCH      = 0x08
	REGPAIR_READ     = 0x09
	REGPAIR_READ_RES = 0x0a
	REGPAIR_WRITE    = 0x0b
	REGPAIR_TOGGLE   = 0x0c
	REGPAIR_RW_EXCH  = 0x0d
	REGPAIR_WR_EXCH  = 0x0e
	MEMMON_DATA      = 0x0f
	MEMMON_CTRL      = 0x10
	APPLICATION      = 0x40


@enum.unique
class StandardResults(enum.IntEnum):
	OK              = 0x00
	TYPE            = 0x01
	LENGTH          = 0x02
	REGISTER        = 0x03
	NOT_IMPLEMENTED = 0x3f
	APPLICATION     = 0x40
	NONE            = 0xff


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


@enum.unique
class ResultCode(enum.Enum):
	SUCCESS          = 0
	RECV_TIMEOUT     = 1
	RECV_FAILURE     = 2
	UNROUTABLE       = 3
	LINK_FAILURE     = 4
	INVALID_HEADER   = 5
	INVALID_MESSAGE  = 6
	INVALID_ARGUMENT = 7
	INVALID_STATE    = 8


class Message:
	MESSAGE_TYP_SIZE = 1
	
	def __init__(self, typ, val):
		if not (self.MIN_TYPE_NUM <= typ <= self.MAX_TYPE_NUM):
			raise ValueError()
		
		if len(val) > self.MAX_MESSAGE_LEN:
			raise ValueError()
		
		self.typ = typ
		self.val = val
	
	def __len__(self):
		return len(self.val)
	
	def __str__(self):
		length = len(self.val)
		f_typ, f_val = self.format()
		return f'Message({f_typ}, {length}, {f_val})' # NOTE: Stupid Raspbian.
		#return 'Message({0}, {1}, {2})'.format(str(f_typ), length, str(f_val))
	
	def __repr__(self):
		return f'<{str(self)}>'
		#return '<{0}>'.format(str(self))
	
	def check_tl(self, typ, length):
		return self.typ == typ and len(self) >= length
	
	def format(self, formatter=None, conv=None):
		return self.format_type(formatter), self.format_val(formatter, conv)
	
	def format_type(self, formatter=None):
		if formatter is None:
			formatter = default_msg_factory
		return map_enum(formatter.type_enum_class, self.typ, self.typ)
	
	def format_val(self, formatter=None, conv=None):
		if formatter is None:
			formatter = default_msg_factory
		f_typ = self.format_type(formatter)
		return formatter.format_val(self.val, conv, f_typ)
	
	def format_mval(self, formatter=None, conv=None, size=None, n_vals=None):
		if formatter is None:
			formatter = default_msg_factory
		return formatter.format_mval(self.val, conv, size, n_vals)


class StandardMessage(Message):
	MESSAGE_LEN_SIZE = 1
	MIN_MESSAGE_SIZE = Message.MESSAGE_TYP_SIZE + MESSAGE_LEN_SIZE
	
	MIN_TYPE_NUM = 0x00
	MAX_TYPE_NUM = 0x7f
	MAX_MESSAGE_LEN = 0xff
	
	PROTOCOL_TYPE_NUM = 0x7f


class LargeMessage(Message):
	MESSAGE_LEN_SIZE = 4
	MIN_MESSAGE_SIZE = Message.MESSAGE_TYP_SIZE + MESSAGE_LEN_SIZE
	
	MIN_TYPE_NUM = 0x80
	MAX_TYPE_NUM = 0xff
	MAX_MESSAGE_LEN = 0xffffffff


class MessageFactory:
	
	def __init__(self):
		self.format_hex_str = False
		self.str_encoding = 'ascii'
		self.int_size = 1
		self.int_byteorder = 'little'
		self.default_val_conv = 'bytes'
		self.enum_format_val = True
		self.type_enum_class = StandardTypes
		self.res_enum_class = StandardResults
		self.reg_enum_class = StandardRegisters
	
	def make_val(self, val, int_size=None):
		if int_size is None:
			int_size = self.int_size
		
		b_val = None
		
		if isinstance(val, bytes):
			b_val = val
		elif isinstance(val, str):
			if self.format_hex_str:
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
		elif val is None:
			b_val = b''
		
		return b_val
	
	def make_mval(self, mval, int_size=None):
		if int_size is None or isinstance(int_size, int):
			old_int_size = int_size
			int_size = (old_int_size for _ in range(len(mval)))
		
		b_mval = b''
		
		for val, val_int_size in zip(mval, int_size):
			b_val = self.make_val(val, val_int_size)
			if b_val is None:
				return None
			b_mval += b_val
		
		return b_mval
	
	def format_val(self, val, conv=None, tlv_type=None):
		if self.enum_format_val and tlv_type is not None and len(val) >= 1:
			e_val = None
			
			if tlv_type in (StandardTypes.RESULT, StandardTypes.INM_RESULT):
				e_val = map_enum(self.res_enum_class, val[0], val[0])
			elif tlv_type in (StandardTypes.REG_READ_RES, StandardTypes.REGPAIR_READ_RES):
				e_val = map_enum(self.reg_enum_class, val[0], val[0])
			
			if e_val is None:
				pass
			elif len(val) == 1:
				return e_val
			else:
				i_val = self.format_val(val[1:], conv)
				return (e_val, i_val)
		
		if conv is None:
			conv = self.default_val_conv
		
		f_val = None
		
		if conv == 'hex':
			f_val = val.hex()
		elif conv == 'bin':
			f_val = bytes_to_binary(val, True)
		elif conv == 'int':
			f_val = int.from_bytes(val, self.int_byteorder)
		elif conv == 'str':
			f_val = str(val, self.str_encoding)
		elif conv == 'tuple':
			f_val = tuple(val)
		elif conv == 'list':
			f_val = list(val)
		elif conv == 'bytes':
			f_val = val
		else:
			pass
		
		return f_val
	
	def format_mval(self, mval, conv=None, size=None, n_vals=None):
		if n_vals is not None:
			pass
		elif conv is not None and not isinstance(conv, str):
			n_vals = len(conv)
		elif size is not None and not isinstance(size, int):
			n_vals = len(size)
		else:
			n_vals = 1
		
		if conv is None or isinstance(conv, str):
			old_conv = conv
			conv = (old_conv for _ in range(n_vals))
		
		if size is None:
			size = self.int_size
		
		if isinstance(size, int):
			old_size = size
			size = (old_size for _ in range(n_vals))
		
		f_mval = []
		val_offset = 0
		for val_conv, val_size in zip(conv, size):
			val = mval[val_offset:val_offset+val_size]
			f_val = self.format_val(val, val_conv)
			if f_val is None:
				#print(f'conv={val_conv} size={val_size}') # DEBUG: 
				return None
			f_mval.append(f_val)
			val_offset += val_size
		
		return f_mval
	
	def make_msg(self, typ, val, int_size=None):
		b_val = self.make_val(val, int_size)
		msg = StandardMessage(typ, b_val)
		return msg
	
	def make_msg_mval(self, typ, mval, int_size=None):
		b_mval = self.make_mval(mval, int_size)
		msg = StandardMessage(typ, b_mval)
		return msg
	
	def make_large_msg(self, typ, val, int_size=None):
		b_val = self.make_val(val, int_size)
		msg = LargeMessage(typ, b_val)
		return msg
	
	def make_large_msg_mval(self, typ, mval, int_size=None):
		b_mval = self.make_mval(mval, int_size)
		msg = LargeMessage(typ, b_mval)
		return msg

default_msg_factory = MessageFactory()


class MessageHeader:
	MESSAGE_ID_SIZE = 2
	DSTADR_SIZE = 1
	SRCADR_SIZE = 1
	HEADER_SIZE = MESSAGE_ID_SIZE + DSTADR_SIZE + SRCADR_SIZE
	
	MAX_MESSAGE_ID = 0xffff
	MAX_MESSAGE_ADR = 0xff
	LOCAL_ADR = 0x00
	BROADCAST_ADR = MAX_MESSAGE_ADR
	
	def __init__(self, msg_id, dstadr, srcadr):
		if not (0 <= msg_id <= self.MAX_MESSAGE_ID):
			raise ValueError()
		
		if not (0 <= dstadr <= self.MAX_MESSAGE_ADR):
			raise ValueError()
		
		if not (0 <= srcadr <= self.MAX_MESSAGE_ADR):
			raise ValueError()
		
		self.msg_id = msg_id
		self.dstadr = dstadr
		self.srcadr = srcadr
	
	def __str__(self):
		return f'MessageHeader({self.msg_id}, {self.dstadr}, {self.srcadr})'
		#return 'MessageHeader({0}, {1}, {2})'.format(
		#	self.msg_id, self.dstadr, self.srcadr)
	
	def __repr__(self):
		return f'<{str(self)}>'
		#return '<{0}>'.format(str(self))


class MessageChannelError(Exception):
	pass

class MessageChannel:
	
	make_default_selector = True
	
	def __init__(self, srcadr, timeout=None, msg_factory=None, selector=None):
		if msg_factory is None:
			msg_factory = default_msg_factory
		
		if selector is False:
			selector = None
		elif selector is None and MessageChannel.make_default_selector:
			selector = selectors.DefaultSelector()
		
		self.srcadr = srcadr
		self.next_msg_id = 0
		self.timeout = timeout
		self.msg_factory = msg_factory
		self.selector = selector
		
		self._is_open = False
	
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr})'
		#return '{0}({1})'.format(type(self).__name__, self.srcadr)
	
	def __repr__(self):
		return f'<{str(self)}>'
		#return '<{0}>'.format(str(self))
	
	def __enter__(self):
		res = self.open()
		if res not in (ResultCode.SUCCESS, ResultCode.INVALID_STATE):
			raise MessageChannelError(str(res))
		return self
	
	def __exit__(self, exc_type, exc_value, traceback):
		self.close()
		return False
	
	# CAUTION: This is not any sort of thread safe.
	def _get_next_msg_id(self):
		msg_id = self.next_msg_id
		self.next_msg_id = (msg_id + 1) % (MessageHeader.MAX_MESSAGE_ID + 1)
		return msg_id
	
	def _register_selectable(self, selectable):
		if self.selector is None:
			return None
		
		try:
			selector_key = self.selector.register(selectable, selectors.EVENT_READ)
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
	
	def _selectable_readable(self, selector_keys, timeout=0.0):
		if self.selector is None:
			return False
		
		selected = self.selector.select(timeout)
		return any((k, selectors.EVENT_READ) in selected for k in selector_keys)
	
	def is_open(self):
		return self._is_open
	
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		self._is_open = True
		return ResultCode.SUCCESS
	
	def close(self):
		self._is_open = False
	
	def readable(self):
		if not self.is_open():
			return False
		
		selector_keys = self.get_selector_keys()
		return self._selectable_readable(selector_keys)
	
	def get_selector_keys(self):
		return ()
	
	def set_timeout(self, timeout):
		self.timeout = timeout
	
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		raise NotImplementedError()
	
	def recv(self): # Returns (res, header, msg, link_adr).
		raise NotImplementedError()

def format_msg_info(obj, event=None, header=None, link_adr=None, event_colw=0):
	event_field = '' if event is None else f'{event:>{event_colw:d}} '
	#event_field = '' if event is None else '{1:>{0:d}} '.format(event_colw, event)
	
	header_field = '' if header is None else f'[{header.srcadr}.{header.msg_id} => {header.dstadr}]'
	#header_field = '' if header is None else '[{0}.{1} => {2}]'.format(
	#	, header.msg_id, header.dstadr)
	
	if isinstance(link_adr, tuple) and len(link_adr) > 1 \
	and isinstance(link_adr[0], MessageChannel):
		link_adr = link_adr[1]
	
	link_adr_field = '' if link_adr is None else link_adr
	
	return f'{event_field}{header_field}{link_adr_field}: {str(obj)}'
	#return '{0}{1}{2}: {3}'.format(event_field, header_field, link_adr_field, str(obj))


class RoutingMessageChannel(MessageChannel):
	
	TIMEOUT_DIVISOR = 10.0
	DEFAULT_CH_TIMEOUT = get_timedelta(musecs=10000)
	
	def __init__(self, srcadr, rtab, recv_channels, timeout=None, msg_factory=None):
		super().__init__(srcadr, timeout, msg_factory, False)
		
		self.rtab = rtab
		self.recv_channels = recv_channels
		self.close_recv_channels = True
		
		# NOTE: Set this to True to apply routing even to messages where the
		#       destination address equals the source address of the
		#       RoutingMessageChannel.
		self.relay_messages = False
	
	def open(self):
		if self.is_open():
			return ResultCode.INVALID_STATE
		
		self.set_timeout(self.timeout) # NOTE: Update timeouts in child channels.
		
		for ch in self.recv_channels:
			res = ch.open()
			if res not in (ResultCode.SUCCESS, ResultCode.INVALID_STATE):
				return res
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self.close_recv_channels:
			for ch in self.recv_channels:
				ch.close()
		
		self.rtab = None
		self.recv_channels = None
	
	# NOTE: The default implementation works for all currently defined
	#       MessageChannel types, and is more efficient.
	#def readable(self):
	#	return any(ch.readable() for ch in self.recv_channels)
	
	def get_selector_keys(self):
		return (k for ch in self.recv_channels for k in ch.get_selector_keys())
	
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if self.timeout is None:
			ch_timeout = self.DEFAULT_CH_TIMEOUT
		else:
			n_ch = max(1, len(self.recv_channels))
			ch_timeout = self.timeout / (self.TIMEOUT_DIVISOR * n_ch)
		
		for ch in self.recv_channels:
			ch.set_timeout(ch_timeout)
	
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None, in_link_adr=None):
		if msg_id is None:
			msg_id = self._get_next_msg_id()
		
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is None:
			link_adr = self.rtab.get(dstadr)
		elif link_adr is not None and not isinstance(link_adr, list):
			link_adr = [link_adr]
		
		if link_adr is None:
			return ResultCode.UNROUTABLE
		
		res = ResultCode.UNROUTABLE
		
		for link_adr_pair in link_adr:
			if link_adr_pair == in_link_adr:
				continue # NOTE: Never bounce a message back on the link it arrived on.
			
			ch, ch_link_adr = link_adr_pair
			#print(f'{dstadr} {ch} {ch_link_adr}') # DEBUG: 
			
			ch_res = ch.send(dstadr, msg, msg_id, srcadr, ch_link_adr)
			if ch_res != ResultCode.SUCCESS:
				res = ch_res
			elif res == ResultCode.UNROUTABLE:
				res = ResultCode.SUCCESS
		
		return res
	
	# ISSUE: Should this poll readable() (or use a selector) instead of relying
	#        on channel timeouts?
	def recv(self):
		avail_timeout = self.timeout
		
		while True:
			if self.timeout is not None:
				t0 = _now()
			
			for ch in self.recv_channels:
				ch_res, header, msg, ch_link_adr = ch.recv()
				
				if ch_res == ResultCode.SUCCESS:
					return ResultCode.SUCCESS, header, msg, (ch, ch_link_adr)
				elif ch_res != ResultCode.RECV_TIMEOUT:
					return ResultCode.LINK_FAILURE, None, None, (ch, ch_link_adr)
			
			if self.timeout is not None:
				t1 = _now()
				dt = t1 - t0
				avail_timeout -= dt
				t0 = t1
				
				if avail_timeout <= ZERO_DURATION:
					return ResultCode.RECV_TIMEOUT, None, None, None
	
	# Returns (delivery_flag, res, dstadr, srcadr, msg, link_adr).
	def route(self):
		delivery_flag = False
		res, header, msg, link_adr = self.recv()
		
		if res == ResultCode.SUCCESS:
			if header.dstadr == MessageHeader.BROADCAST_ADR:
				res = self.send(
					header.dstadr, msg, header.msg_id, header.srcadr, in_link_adr=link_adr)
				delivery_flag = True
			elif header.dstadr == self.srcadr:
				if self.relay_messages:
					res = self.send(
						header.dstadr, msg, header.msg_id, header.srcadr, in_link_adr=link_adr)
				delivery_flag = True
			else:
				res = self.send(
					header.dstadr, msg, header.msg_id, header.srcadr, in_link_adr=link_adr)
		
		return delivery_flag, res, header, msg, link_adr


class BinaryMessageChannel(MessageChannel):
	def __init__(self, srcadr, timeout=None, msg_factory=None, selector=None,
	             send_bfr_size=0, recv_bfr_size=0):
		
		super().__init__(srcadr, timeout, msg_factory, selector)
		
		self.send_buffer = bytearray(send_bfr_size)
		self.recv_buffer = bytearray(recv_bfr_size)
	
	# CAUTION: This is not any sort of thread safe.
	def _prepare_send_buffer(self, dstadr, srcadr, msg, msg_id=None):
		if msg_id is None:
			msg_id = self._get_next_msg_id()
		
		msg_id_b = self.msg_factory.make_val(msg_id, int_size=2)
		dstadr_b = self.msg_factory.make_val(dstadr, int_size=1)
		srcadr_b = self.msg_factory.make_val(srcadr, int_size=1)
		msg_typ_b = self.msg_factory.make_val(msg.typ, int_size=1)
		msg_len_b = self.msg_factory.make_val(len(msg.val), int_size=msg.MESSAGE_LEN_SIZE)
		
		self.send_buffer.clear()
		self.send_buffer.extend(msg_id_b)
		self.send_buffer.extend(dstadr_b)
		self.send_buffer.extend(srcadr_b)
		self.send_buffer.extend(msg_typ_b)
		self.send_buffer.extend(msg_len_b)
		self.send_buffer.extend(msg.val)
	
	def _parse_recv_header(self, n_bytes):
		if n_bytes < MessageHeader.HEADER_SIZE:
			return ResultCode.INVALID_HEADER, None, None, None, None
		
		msg_size = n_bytes - MessageHeader.HEADER_SIZE
		
		if msg_size < StandardMessage.MIN_MESSAGE_SIZE:
			return ResultCode.INVALID_MESSAGE, None, None, None, None
		
		#print(n_bytes) # DEBUG: 
		#print(self.msg_factory.format_val(self.recv_buffer[:n_bytes], 'hex')) # DEBUG: 
		
		i = 0
		msg_id = self.msg_factory.format_val(
			self.recv_buffer[i:i+MessageHeader.MESSAGE_ID_SIZE], 'int')
		i += MessageHeader.MESSAGE_ID_SIZE
		dstadr = self.recv_buffer[i]
		i += 1
		srcadr = self.recv_buffer[i]
		i += 1
		msg_typ = self.recv_buffer[i]
		i += 1
		
		# TODO: Check message IDs for duplicates.
		
		if msg_typ >= LargeMessage.MIN_TYPE_NUM:
			msg_len = self.msg_factory.format_val(
				self.recv_buffer[i:i+LargeMessage.MESSAGE_LEN_SIZE], 'int')
			i += LargeMessage.MESSAGE_LEN_SIZE
		else:
			msg_len = self.recv_buffer[i]
			i += 1
		
		header = MessageHeader(msg_id, dstadr, srcadr)
		return ResultCode.SUCCESS, header, msg_typ, msg_len, i
	
	def _parse_recv_value(self, n_bytes, msg_typ, msg_len, start_i):
		msg_val = bytes(self.recv_buffer[start_i:n_bytes])
		
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


class InetMessageChannel(BinaryMessageChannel):
	DEFAULT_IP_ADR = '127.0.0.1'
	DEFAULT_UDP_PORT = 2357
	DEFAULT_TCP_PORT = 2357
	
	MAX_DATAGRAM_SIZE = 0xffff
	
	def __init__(self, srcadr, ip_adr=None, udp_port=None, tcp_port=None,
		timeout=None, msg_factory=None, selector=None):
		
		if ip_adr is None:
			ip_adr = self.DEFAULT_IP_ADR
		if udp_port is None or udp_port == 0:
			udp_port = self.DEFAULT_UDP_PORT
		if tcp_port is None or tcp_port == 0:
			tcp_port = self.DEFAULT_TCP_PORT
		
		super().__init__(srcadr, timeout, msg_factory, selector, 0, self.MAX_DATAGRAM_SIZE)
		
		self.ip_adr = ip_adr
		self.udp_port = udp_port
		self.tcp_port = tcp_port
		self.udp_socket = None
		self.udp_selector_key = None
		self.tcp_socket = None
		self.tcp_selector_key = None
	
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr}, {self.ip_adr}, {self.udp_port})'
		#return '{0}({1}, {2}, {3})'.format(
		#	type(self).__name__, self.srcadr, self.ip_adr, self.udp_port)
	
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
		
		self.udp_socket = udp_socket
		self.udp_selector_key = self._register_selectable(self.udp_socket)
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self.udp_socket is not None:
			self._unregister_selectable(self.udp_socket)
			
			self.udp_socket.close()
			self.udp_socket = None
			self.udp_selector_key = None
		
		# TODO: Shut down and close the TCP socket, if it exists.
	
	def get_selector_keys(self):
		if self.udp_selector_key is None:
			return ()
		return self.udp_selector_key,
	
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if not self.is_open():
			return
		
		if self.timeout is not None:
			timeout_seconds = self.timeout.total_seconds()
		else:
			timeout_seconds = None
		
		self.udp_socket.settimeout(timeout_seconds)
	
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is None:
			return ResultCode.INVALID_ARGUMENT
		
		# TODO: Implement TCP transport for large messages.
		
		self._prepare_send_buffer(dstadr, srcadr, msg, msg_id)
		
		n_bytes = self.udp_socket.sendto(self.send_buffer, link_adr)
		
		if n_bytes != len(self.send_buffer):
			return ResultCode.LINK_FAILURE
		
		return ResultCode.SUCCESS
	
	def recv(self):
		try:
			# ISSUE: Does this always provide either nothing or a complete datagram?
			# NOTE: "man udp" seems to indicate that this is the case:
			#         "All receive operations return only one packet."
			#       Nothing is mentioned or implied about partial packets being
			#       received. That makes sense, since it would just be a PITA for
			#       no good reason.
			n_bytes, link_adr = self.udp_socket.recvfrom_into(
				self.recv_buffer, len(self.recv_buffer))
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


class SerialMessageChannel(BinaryMessageChannel):
	#DEFAULT_BAUDRATE = 9600
	DEFAULT_BAUDRATE = 38400
	
	N_HEADER_BYTES = MessageHeader.HEADER_SIZE + StandardMessage.MIN_MESSAGE_SIZE
	MAX_MESSAGE_LEN = 0xff - N_HEADER_BYTES
	
	TIMEOUT_DIVISOR = 10.0
	
	def __init__(self, srcadr, port, baudrate=None, timeout=None, msg_factory=None, selector=None):
		if baudrate is None:
			baudrate = self.DEFAULT_BAUDRATE
		
		super().__init__(srcadr, timeout, msg_factory, selector, 0, 0)
		
		self.port = port
		self.baudrate = baudrate
		self.serial_dev = None
		self.serial_selector_key = None
		
		self.recv_header = None
		self.recv_msg_typ = None
		self.recv_msg_len = None
	
	def __str__(self):
		return f'{type(self).__name__}({self.srcadr}, {self.port})'
		#return '{0}({1}, {2})'.format(
		#	type(self).__name__, self.srcadr, self.port)
	
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
		
		self.serial_dev = serial_dev
		self.serial_selector_key = self._register_selectable(self.serial_dev)
		
		self._is_open = True
		return ResultCode.SUCCESS
	
	def close(self):
		if not self.is_open():
			return
		
		self._is_open = False
		
		if self.serial_dev is not None:
			self._unregister_selectable(self.serial_dev)
			
			self.serial_dev.close()
			self.serial_dev = None
			self.serial_selector_key = None
	
	def get_selector_keys(self):
		if self.serial_selector_key is None:
			return ()
		return self.serial_selector_key,
	
	def set_timeout(self, timeout):
		self.timeout = timeout
		
		if not self.is_open():
			return
		
		if timeout is not None:
			timeout_seconds = timeout.total_seconds() / self.TIMEOUT_DIVISOR
		else:
			timeout_seconds = None
		
		self.serial_dev.timeout = timeout_seconds
	
	def send(self, dstadr, msg, msg_id=None, srcadr=None, link_adr=None):
		if srcadr is None:
			srcadr = self.srcadr
		
		if link_adr is not None: # Serial links are point-to-point.
			return ResultCode.INVALID_ARGUMENT
		
		if len(msg) > self.MAX_MESSAGE_LEN:
			return ResultCode.INVALID_MESSAGE
		
		self._prepare_send_buffer(dstadr, srcadr, msg, msg_id)
		
		n_bytes = self.serial_dev.write(self.send_buffer)
		
		#print(f'msg={str(msg)}') # DEBUG:
		#print(f'{len(self.send_buffer)} {self.send_buffer.hex()}') # DEBUG: 
		#print(f'n_bytes={n_bytes}') # DEBUG: 
		
		if n_bytes != len(self.send_buffer):
			return ResultCode.LINK_FAILURE
		
		return ResultCode.SUCCESS
	
	def recv(self):
		if self.timeout is not None:
			avail_timeout = self.timeout
			t0 = _now()
		
		while True:
			if len(self.recv_buffer) < self.N_HEADER_BYTES: # Receive header.
				bytes_to_read = self.N_HEADER_BYTES - len(self.recv_buffer)
				header_bytes = self.serial_dev.read(bytes_to_read)
				
				if len(header_bytes) > 0:
					self.recv_buffer.extend(header_bytes)
					
					if len(self.recv_buffer) == self.N_HEADER_BYTES:
						res, header, msg_typ, msg_len, i = self._parse_recv_header(
							len(self.recv_buffer))
						
						if res != ResultCode.SUCCESS:
							return res
						
						self.recv_header = header
						self.recv_msg_typ = msg_typ
						self.recv_msg_len = msg_len
			else: # Header received. Receive value bytes.
				n_message_bytes = self.N_HEADER_BYTES + self.recv_msg_len
				
				if len(self.recv_buffer) < n_message_bytes:
					bytes_to_read = n_message_bytes - len(self.recv_buffer)
					value_bytes = self.serial_dev.read(bytes_to_read)
					if len(value_bytes) > 0:
						self.recv_buffer.extend(value_bytes)
				
				if len(self.recv_buffer) == n_message_bytes:
					res, msg = self._parse_recv_value(
						len(self.recv_buffer), self.recv_msg_typ, self.recv_msg_len,
						self.N_HEADER_BYTES)
					
					if res != ResultCode.SUCCESS:
						return res
					
					header = self.recv_header
					
					self.recv_buffer.clear()
					self.recv_header = None
					self.recv_msg_typ = None
					self.recv_msg_len = None
		
					return ResultCode.SUCCESS, header, msg, None
			
			#print(f'{len(self.recv_buffer)} {self.recv_buffer.hex()}') # DEBUG: 
			
			if self.timeout is not None:
				t1 = _now()
				dt = t1 - t0
				avail_timeout -= dt
				t0 = t1
				
				if avail_timeout <= ZERO_DURATION:
					return ResultCode.RECV_TIMEOUT, None, None, None
