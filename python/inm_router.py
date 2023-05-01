#!/usr/bin/python3

import argparse
import enum
import importlib
import logging
import logging.handlers
import selectors
import sys

import inm.inm as inm
from inm.inm import ResultCode as _RC, ValueConversions as _VC
from inm.inm import StandardTypes as _ST, StandardResults as _SRes, StandardRegisters as _SR


_DEFAULT_UDP_PORT = inm.InetMessageChannel.DEFAULT_UDP_PORT
_DEFAULT_BAUDRATE = inm.SerialMessageChannel.DEFAULT_BAUDRATE
_MAX_BAUDRATE = 10_000_000
_MAX_INM_ADR = inm.MessageHeader.MAX_MESSAGE_ADR
_BROADCAST_ADR = inm.MessageHeader.BROADCAST_ADR

_DEFAULT_HOOK_INTERVAL = 1.0  # 1 second between main loop hook invocations

_prog_title = 'INM Router Script'
# 0xAABBCCDD
# A: major version, B: minor version, C: revision, D: build.
_prog_version = 0x01_00_05_01  # 1.0.5.1

_firmware_id_l = 0x01
_firmware_id_h = 0x03
_firmware_id = (_firmware_id_h << 8) | _firmware_id_l
_firmware_version = 0xff & _prog_version

_def_loglevel     = logging.INFO
_syslog_adr       = '/dev/log'  # TODO: Make this configurable.
_rootlog_handlers = []
_rootlog          = logging.getLogger()
_log              = logging.getLogger('inm_router')

_msg_factory = inm.default_msg_factory


def _init_logger(loglevel):
	global _rootlog_handler
	
	stream_handler = logging.StreamHandler()
	formatter = logging.Formatter('{levelname}:{name}: {message}', style='{')
	stream_handler.setFormatter(formatter)
	
	syslog_handler = logging.handlers.SysLogHandler(
		_syslog_adr, logging.handlers.SysLogHandler.LOG_DAEMON)
	formatter = logging.Formatter('{levelname}:{name}: {message}', style='{')
	syslog_handler.setFormatter(formatter)
	
	_rootlog.setLevel(loglevel)
	_rootlog.addHandler(stream_handler)
	_rootlog.addHandler(syslog_handler)
	_rootlog_handlers = [stream_handler, syslog_handler]

def _set_loglevel(loglevel):
	_rootlog.setLevel(loglevel)

def _set_log_format(debug_fmt):
	fmt = '{levelname}:{name}: {message}'
	
	if debug_fmt:
		fmt = '{levelname}:{name}:{funcName}:{lineno}: {message}'
	
	for handler in _rootlog_handlers:
		formatter = logging.Formatter(fmt, style='{')
		handler.setFormatter(formatter)

def _shutdown_logger():
	logging.shutdown()

class _BetterArgumentParser(argparse.ArgumentParser):
	def convert_arg_line_to_args(self, arg_line):
		if len(arg_line) == 0 or arg_line[0] == '#':  # Empty or comment line.
			return []
		elif arg_line[0] == ':':  # Verbatim argument line.
			return [arg_line[1:]]
		else:  # Space-separated argument line.
			return arg_line.split()

def _parse_args(args=None, namespace=None):
	desc_text = '''A TLV/INM message router script. Any argument
prefixed with a '@' is interpreted as the path of a file to read additional
arguments from. In such argument files, any line starting with a '#' is
interpreted as a comment and ignored. Any line starting with a ':' is
interpreted as a single verbatim argument. All other lines are interpreted
as zero or more whitespace-separated arguments. Note that the special cases
only apply if the '#' or ':' is the very first character on the line, it
must not be predeced by whitespace or anything else.'''
	
	arg_p = _BetterArgumentParser(description=desc_text, fromfile_prefix_chars='@')
	int0 = lambda x: int(x, 0)  # Allow prefixed integer arguments in base 2, 8, or 16.
	
	arg_p.add_argument('-A', '--inm-adr',
		help='INM address of router', metavar='INMADR',
		default=0, type=int0, dest='inm_adr')
	arg_p.add_argument('-C', '--cc-to',
		help='INM address of CC destination', metavar='INMADR',
		type=int0, action='append', dest='cc_to')
	arg_p.add_argument('-D', '--debug-log',
		help='use debug log format', action='store_true', dest='debug_log')
	arg_p.add_argument('-H', '--hook-module',
		help='name of application hook module', metavar='NAME', dest='hook_module')
	arg_p.add_argument('-I', '--ip-ch',
		help='IP channel specification: {channel number} {IP address} {UDP port}',
		metavar=('CH', 'IPADR', 'PORT'), nargs=3, action='append', dest='ip_channels')
	arg_p.add_argument('-L', '--loglevel',
		help='set numeric log level', default=_def_loglevel, type=int,
		metavar='LOGLEVEL', dest='loglevel')
	arg_p.add_argument('-R', '--relay',
		help='relay mode, route messages even when the router is the destination',
		action='store_true', dest='relay')
	arg_p.add_argument('-S', '--serial-ch',
		help='serial channel specification: {channel number} {serial port} {baud rate}',
		metavar=('CH', 'PORT', 'BAUD'), nargs=3, action='append', dest='serial_channels')
	arg_p.add_argument('-T', '--hook-interval',
		help='seconds between main loop hook invocations', metavar='SECS',
		default=_DEFAULT_HOOK_INTERVAL, type=float, dest='hook_interval')
	arg_p.add_argument('-V', '--version',
		help='print version number and exit', action='store_true', dest='print_version')
	arg_p.add_argument('-r', '--route',
		help='route specification: {INM destination} {channel number} {option,...} {link address ...}',
		metavar=('INMADR', 'ARG'), nargs='+', action='append', dest='routes')
	arg_p.add_argument('-v', '--verbose',
		help='increase log verbosity', action='count', default=0, dest='verbosity')
	
	return arg_p.parse_args(args, namespace)

def _parse_inm_adr(s):
	# NOTE: Because an int argument to int() is suddenly verboten when you specify a base.
	inm_adr = s if isinstance(s, int) else int(s, 0)
	if not (0 <= inm_adr <= _MAX_INM_ADR):
		_log.error(f'Invalid INM address {inm_adr} specified. Exiting.')
		sys.exit(2)
	return inm_adr

def _parse_ch_num(s, channels=None):
	ch_num = int(s, 0)  # Allow prefixed integer arguments in base 2, 8, or 16.
	if not (0 <= ch_num <= 0xffff):
		_log.error(f'Invalid channel number {ch_num} specified. Exiting.')
		sys.exit(3)
	
	if channels is not None and ch_num in channels:
		_log.error(f'Duplicate channel number {ch_num} specified. Exiting.')
		sys.exit(4)
	
	return ch_num

def _parse_port_num(s):
	port_num = int(s, 0)  # Allow prefixed integer arguments in base 2, 8, or 16.
	if not (0 <= port_num <= 0xffff):
		_log.error(f'Invalid port number {port_num} specified. Exiting.')
		sys.exit(5)
	elif port_num == 0:
		port_num = _DEFAULT_UDP_PORT
	return port_num

def _parse_baud_rate(s):
	baud = int(s)
	if not (0 <= baud <= _MAX_BAUDRATE):
		_log.error(f'Invalid baud rate {baud} specified. Exiting.')
		sys.exit(6)
	elif baud == 0:
		baud = _DEFAULT_BAUDRATE
	return baud

def _parse_serial_link_adr(link_adr):
	if len(link_adr) > 0:
		_log.error('Serial channels do not use link addresses. Exiting.')
		sys.exit(7)
	return None

def _parse_ip_link_adr(link_adr):
	if len(link_adr) == 1:
		return (link_adr[0], _DEFAULT_UDP_PORT)
	elif len(link_adr) == 2:
		return (link_adr[0], _parse_port_num(link_adr[1]))
	else:
		_log.error(f'Invalid IP link address {link_adr} specified. Exiting.')
		sys.exit(8)

class RouteOptions(enum.Flag):
	NONE = 0
	NO_BROADCAST = enum.auto()

def _parse_route_options(option_str):
	option_list = [s.strip() for s in option_str.split(',')]
	options = RouteOptions.NONE
	
	for option_name in option_list:
		if option_name == '-':
			pass  # NOPtion
		elif option_name == 'no-broadcast':
			options |= RouteOptions.NO_BROADCAST
		else:
			_log.error(f'Invalid route option {option_name} specified. Exiting.')
			sys.exit(9)
	
	return options

def _init_channels(args, inm_adr, def_selector):
	channels = {}
	
	if args.ip_channels is not None:
		for ch_num_, ip_adr, udp_port in args.ip_channels:
			ch_num_ = _parse_ch_num(ch_num_, channels)
			udp_port = _parse_port_num(udp_port)
			ch = inm.InetMessageChannel(
				inm_adr, ip_adr, udp_port, selector=def_selector, ch_num=ch_num_)
			channels[ch_num_] = ch
	
	if args.serial_channels is not None:
		for ch_num_, serial_port, baud in args.serial_channels:
			ch_num_ = _parse_ch_num(ch_num_, channels)
			baud = _parse_baud_rate(baud)
			ch = inm.SerialMessageChannel(
				inm_adr, serial_port, baud, selector=def_selector, ch_num=ch_num_)
			channels[ch_num_] = ch
	
	return channels

def _init_routes(args, channels):
	rtab = {}
	
	# TODO: Better error reporting (not just a stack trace) when a "routes" entry is malformed.
	for inm_adr, ch_num, options, *link_adr in args.routes:
		inm_adr = _parse_inm_adr(inm_adr)
		ch_num = _parse_ch_num(ch_num)
		options = _parse_route_options(options)
		
		if ch_num not in channels:
			_log.error(f'Routing channel number {ch_num} not found. Exiting.')
			sys.exit(10)
		
		ch = channels[ch_num]
		
		if isinstance(ch, inm.InetMessageChannel):
			link_adr = _parse_ip_link_adr(link_adr)
		elif isinstance(ch, inm.SerialMessageChannel):
			link_adr = _parse_serial_link_adr(link_adr)
		else:
			_log.error(f'Unrecognized routing channel {ch}. Exiting.')
			sys.exit(11)
		
		rentry = (ch, link_adr)
		
		if inm_adr not in rtab:
			rtab[inm_adr] = []
		
		# ISSUE: Should duplicate routes be filtered out?
		rtab[inm_adr].append(rentry)
		
		if not options & RouteOptions.NO_BROADCAST:
			if _BROADCAST_ADR not in rtab:
				rtab[_BROADCAST_ADR] = []
			
			if rentry not in rtab[_BROADCAST_ADR]:
				rtab[_BROADCAST_ADR].append(rentry)
	
	return rtab

def _handle_reg_read(header, msg, link_adr):
	reg_index = msg.format_val(conv=_VC.Int)
	reg_val = None
	response = None
	res = None
	
	if reg_index == _SR.FWID_L:
		reg_val = _firmware_id_l
	elif reg_index == _SR.FWID_H:
		reg_val = _firmware_id_h
	elif reg_index == _SR.FWVERSION:
		reg_val = _firmware_version
	else:
		res = _SRes.REGISTER
	
	if reg_val is not None:
		response = _msg_factory.make_msg_mval(
			_ST.INM_REG_READ_RES, (reg_index, reg_val, header.msg_id))
		res = _SRes.OK
	
	return response, res

def _handle_regpair_read(header, msg, link_adr):
	reg_index = msg.format_val(conv=_VC.Int)
	regpair_val = None
	response = None
	res = None
	
	if reg_index == _SR.PAIR_FWID:
		regpair_val = _firmware_id
	else:
		res = _SRes.REGISTER
	
	if regpair_val is not None:
		response = _msg_factory.make_msg_mval(
			_ST.INM_REGPAIR_READ_RES, (reg_index, regpair_val, header.msg_id))
		res = _SRes.OK
	
	return response, res

def _handle_msg(header, msg, link_adr):
	response = None
	res = None
	
	# IDEA: Add a message handling hook.
	if msg.typ == _ST.REG_READ:
		response, res = _handle_reg_read(header, msg, link_adr)
	elif msg.typ == _ST.REGPAIR_READ:
		response, res = _handle_regpair_read(header, msg, link_adr)
	# NOTE: Ignore unrecognized message types.
	
	if response is None and res != None and res != _SRes.OK:
		response = _msg_factory.make_msg_mval(_ST.INM_RESULT, (res, header.msg_id))
	
	return response


def _dotted_byte_format(i):
	a, b, c, d = 0xff & (i >> 24), 0xff & (i >> 16), 0xff & (i >> 8), 0xff & i
	return f'{a}.{b}.{c}.{d}'

def _default_loop_hook_init(msg_factory, rch):
	pass

def _default_loop_hook(rch):
	pass

def main():
	inm.default_msg_factory.default_val_conv = inm.ValueConversions.Hex
	
	args = _parse_args()
	version_str = _dotted_byte_format(_prog_version)
	
	_set_loglevel(max(args.loglevel - 10*args.verbosity, 0))
	_set_log_format(args.debug_log)
	
	if args.print_version:
		print(version_str)
		return
	
	_log.info(f'Version {version_str}')
	
	inm_adr = _parse_inm_adr(args.inm_adr)
	cc_to = ()
	if args.cc_to is not None:
		cc_to = tuple(_parse_inm_adr(cc_arg) for cc_arg in args.cc_to)
	hook_module_name = args.hook_module
	hook_interval = inm.get_timedelta(args.hook_interval)
	
	loop_hook_init = _default_loop_hook_init
	loop_hook =_default_loop_hook
	
	if hook_module_name is not None:
		hook_module = importlib.import_module(hook_module_name)
		loop_hook_init = hook_module.loop_hook_init
		loop_hook = hook_module.loop_hook
	
	# IDEA: Add a startup hook.
	
	with selectors.DefaultSelector() as selector:
		channels = _init_channels(args, inm_adr, selector)
		if len(channels) == 0:
			_log.warning('No message channels specified. Exiting.')
			sys.exit(1)
		
		rtab = _init_routes(args, channels)
		
		try:
			with inm.RoutingMessageChannel(inm_adr, rtab, channels, hook_interval, None, selector) as rch:
				rch.relay_messages = args.relay
				
				for cc_adr in cc_to:
					rch.add_cc_adr(cc_adr)
				
				latest_hook_call_t = inm.get_timestamp()
				loop_hook_init(_msg_factory, rch)
				
				while True:
					#t0 = inm.get_timestamp()  # DEBUG: 
					
					msg_for_me, res, header, msg, link_adr = rch.route()
					
					#t1 = inm.get_timestamp()  # DEBUG:
					#t_d_r = int(1.0e6 * (t1 - t0).total_seconds())
					#_log.info(f'Musecs in route():             {t_d_r: 8d}')
					#t0 = inm.get_timestamp()
					
					if res == _RC.SUCCESS:
						if msg_for_me:
							_log.debug(inm.format_msg_info(msg, 'RECV', header, link_adr, 8))
							
							response = _handle_msg(header, msg, link_adr)
							
							if response is not None:
								res = rch.send(header.srcadr, response, link_adr=link_adr)
								
								if res != _RC.SUCCESS:
									_log.info(inm.format_msg_info(res.name, 'REPLY_E', header, link_adr, 8))
						else:
							# IDEA: Add a "message routed" hook.
							_log.debug(inm.format_msg_info(msg, 'ROUTE', header, link_adr, 8))
					elif res != _RC.RECV_TIMEOUT:
						# IDEA: Add a "routing error" hook.
						_log.info(inm.format_msg_info(res.name, 'ERROR', header, link_adr, 8))
					
					now = inm.get_timestamp()
					if now - latest_hook_call_t >= hook_interval:
						latest_hook_call_t = now
						loop_hook(rch)
					
					#t1 = inm.get_timestamp()  # DEBUG:
					#t_d_nr = int(1.0e6 * (t1 - t0).total_seconds())
					#t_d = t_d_r + t_d_nr
					#_log.info(f'Musecs outside route():        {t_d_nr: 8d}')
					#_log.info(f'Total lap time (% in route()): {t_d: 8d} ({100.0*(t_d_r/t_d):0.3f}%)')
		except KeyboardInterrupt:
			_log.info('Interrupted. Cleaning up and exiting.')
	
	# IDEA: Add a shutdown hook.

if __name__ == '__main__':
	_init_logger(_def_loglevel)
	
	main()
	
	_shutdown_logger()
