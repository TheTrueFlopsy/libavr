#!/usr/bin/python3

import argparse
import selectors
import sys

import inm

_DEFAULT_UDP_PORT = inm.InetMessageChannel.DEFAULT_UDP_PORT
_MAX_INM_ADR = inm.MessageHeader.MAX_MESSAGE_ADR
_BROADCAST_ADR = inm.MessageHeader.BROADCAST_ADR

# TODO: Implement a TLV/INM console.

class _BetterArgumentParser(argparse.ArgumentParser):
	def convert_arg_line_to_args(self, arg_line):
		if len(arg_line) == 0 or arg_line[0] == '#': # Empty or comment line.
			return []
		elif arg_line[0] == ':': # Verbatim argument line.
			return [arg_line[1:]]
		else: # Space-separated argument line.
			return arg_line.split()

def _parse_args(args=None, namespace=None):
	arg_p = _BetterArgumentParser(
		description='A console-based TLV/INM message router.',
		fromfile_prefix_chars='@')
	
	arg_p.add_argument('-A', '--inm-adr',
		help='INM address of router', metavar='INM_ADR',
		default=0, type=int, dest='inm_adr')
	arg_p.add_argument('-I', '--ip-ch',
		help='<channel number> <IP address> <UDP port>', metavar='IP_CH',
		nargs=3, action='append', dest='ip_channels')
	arg_p.add_argument('-S', '--serial-ch',
		help='<channel number> <serial port>', metavar='SERIAL_CH',
		nargs=2, action='append', dest='serial_channels')
	arg_p.add_argument('-r', '--route',
		help='<INM destination> <channel number> <link address ...>', metavar='ROUTE',
		nargs='+', action='append', dest='routes')
	arg_p.add_argument('-R', '--relay',
		help='relay mode, route messages even when the router is the destination',
		action='store_true', dest='relay')
	
	return arg_p.parse_args(args, namespace)

def _parse_inm_adr(s):
	inm_adr = int(s)
	if inm_adr < 0 or inm_adr > _MAX_INM_ADR:
		print('Invalid INM address {0} specified. Exiting.'.format(inm_adr))
		sys.exit(2)
	return inm_adr

def _parse_ch_num(s, channels=None):
	ch_num = int(s)
	if ch_num < 0 or ch_num > 9999:
		print('Invalid channel number {0} specified. Exiting.'.format(ch_num))
		sys.exit(3)
	
	if channels is not None and ch_num in channels:
		print('Duplicate channel number {0} specified. Exiting.'.format(ch_num))
		sys.exit(4)
	
	return ch_num

def _parse_port_num(s):
	port_num = int(s)
	if port_num < 0 or port_num > 0xffff:
		print('Invalid port number {0} specified. Exiting.'.format(port_num))
		sys.exit(5)
	elif port_num == 0:
		port_num = _DEFAULT_UDP_PORT
	return port_num

def _parse_serial_link_adr(link_adr):
	if len(link_adr) > 0:
		print('Serial channels do not use link addresses. Exiting.')
		sys.exit(6)
	return None

def _parse_ip_link_adr(link_adr):
	if len(link_adr) == 1:
		return (link_adr[0], _DEFAULT_UDP_PORT)
	elif len(link_adr) == 2:
		return (link_adr[0], _parse_port_num(link_adr[1]))
	else:
		print('Invalid IP link address {0} specified. Exiting.'.format(link_adr))
		sys.exit(7)

def _init_channels(args, inm_adr, def_selector):
	channels = {}
	
	if args.ip_channels is not None:
		for ch_num, ip_adr, udp_port in args.ip_channels:
			ch_num = _parse_ch_num(ch_num, channels)
			udp_port = _parse_port_num(udp_port)
			ch = inm.InetMessageChannel(inm_adr, ip_adr, udp_port, selector=def_selector)
			channels[ch_num] = ch
	
	if args.serial_channels is not None:
		for ch_num, serial_port in args.serial_channels:
			ch_num = _parse_ch_num(ch_num, channels)
			ch = inm.SerialMessageChannel(inm_adr, serial_port, selector=def_selector)
			channels[ch_num] = ch
	
	return channels

def _init_routes(args, channels):
	rtab = {}
	
	for inm_adr, ch_num, *link_adr in args.routes:
		inm_adr = _parse_inm_adr(inm_adr)
		ch_num = _parse_ch_num(ch_num)
		
		if ch_num not in channels:
			print('Routing channel number {0} not found. Exiting.'.format(ch_num))
			sys.exit(10)
		
		ch = channels[ch_num]
		
		if isinstance(ch, inm.InetMessageChannel):
			link_adr = _parse_ip_link_adr(link_adr)
		elif isinstance(ch, inm.SerialMessageChannel):
			link_adr = _parse_serial_link_adr(link_adr)
		else:
			print('Unrecognized routing channel {0}. Exiting.'.format(ch))
			sys.exit(11)
		
		rentry = (ch, link_adr)
		
		if inm_adr not in rtab:
			rtab[inm_adr] = []
		
		# ISSUE: Should duplicate routes be filtered out?
		rtab[inm_adr].append(rentry)
		
		if _BROADCAST_ADR not in rtab:
			rtab[_BROADCAST_ADR] = []
		
		if rentry not in rtab[_BROADCAST_ADR]:
			rtab[_BROADCAST_ADR].append(rentry)
	
	return rtab

def main():
	inm.default_msg_factory.default_val_conv = 'hex'
	
	args = _parse_args()
	
	inm_adr = _parse_inm_adr(args.inm_adr)
	
	with selectors.DefaultSelector() as selector:
		channels = _init_channels(args, inm_adr, selector)
		if len(channels) == 0:
			print('No message channels specified. Exiting.')
			sys.exit(1)
		
		rtab = _init_routes(args, channels)
		channel_list = list(channels.values())
		
		try:
			with inm.RoutingMessageChannel(inm_adr, rtab, channel_list) as rch:
				rch.relay_messages = args.relay
				
				while True:
					msg_for_me, res, header, msg, link_adr = rch.route()
					
					if res == inm.ResultCode.SUCCESS:
						event_type = 'ACCEPT' if msg_for_me else 'ROUTE'
						print(inm.format_msg_info(msg, event_type, header, link_adr, 8))
					else:
						print(inm.format_msg_info(res.name, 'ERROR', header, link_adr, 8))
		except KeyboardInterrupt:
			print('Interrupted. Cleaning up and exiting.')

if __name__ == '__main__':
	main()
