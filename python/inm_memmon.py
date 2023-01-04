#!/usr/bin/python3

import argparse
import sys

import inm.inm as inm


# TODO: Figure out how to support color output.
#       Go full ncurses (probably the best bet)?

DEFAULT_IP_ADR_VAL = inm.InetMessageChannel.DEFAULT_IP_ADR
DEFAULT_UDP_PORT_VAL = inm.InetMessageChannel.DEFAULT_UDP_PORT
DEFAULT_SERIAL_PORT_VAL = '/dev/ttyUSB0'

DEFAULT_TLV_TYPE = inm.StandardTypes.MEMMON_DATA

class Visualizer:
	MEMMON_HEADER_SIZE = 3
	
	def __init__(self):
		self.prev_val = None
		self.curr_val = None
		self.min_val = None
		self.max_val = None
		self.one_in_n_count = 0
	
	def _make_val_label(self):
		if self.label_minmax:
			return '{1:{0}}<{2:{0}}<{3:{0}}'.format(
				self.label_format, self.min_val, self.curr_val, self.max_val)
		else:
			return '{1:{0}}'.format(self.label_format, self.curr_val)
	
	def _get_graph_range(self):
		low = self.range_low
		high = self.range_high
		
		if low == 'min':
			low = self.min_val
		elif isinstance(low, str):
			low = float(low)
			self.range_low = low
		
		if high == 'max':
			high = self.max_val
		elif isinstance(high, str):
			high = float(high)
			self.range_high = high
		
		if low >= high:
			lesser = min(low, high)
			greater = max(low, high)
			low = 0.5*lesser - 1.0
			high = 2.0*greater + 1.0
		
		return low, high
	
	def _get_line_char(self):
		if self.curr_val < self.prev_val:
			return self.fall_char
		elif self.curr_val > self.prev_val:
			return self.rise_char
		else:
			return self.flat_char
	
	def _make_graph_bar(self):
		low, high = self._get_graph_range()
		curr = self.curr_val
		width = self.graph_width
		
		if curr < low:
			return self.oor_char
		elif curr > high:
			return (width-1)*self.bar_char + self.oor_char
		else:
			pos = int(round(((width-1) * (curr - low)) / (high - low)))
			return (pos+1)*self.bar_char
	
	def _make_graph_line(self):
		low, high = self._get_graph_range()
		curr = self.curr_val
		width = self.graph_width
		
		if curr < low:
			return self.oor_char
		elif curr > high:
			return (width-1)*' ' + self.oor_char
		else:
			pos = int(round(((width-1) * (curr - low)) / (high - low)))
			line_ch = self._get_line_char()
			return pos*' ' + line_ch
	
	def _visualize_bar(self):
		label = self._make_val_label()
		bar = self._make_graph_bar()
		bar += (self.graph_width - len(bar)) * ' '
		return '[{0}]:{1}:'.format(label, bar)
	
	def _visualize_line(self):
		label = self._make_val_label()
		line = self._make_graph_line()
		line += (self.graph_width - len(line)) * ' '
		return '[{0}]:{1}:'.format(label, line)
	
	def _visualize_none(self):
		return self._make_val_label()
	
	def visualize(self, msg):
		mon_i, ptr, val = msg.format_mval(conv=inm.ValueConversions.Int, size=(1, 2, len(msg)-3))
		
		if mon_i != self.mon_i:
			return
		
		if self.mask_size > 0:
			val >>= self.mask_offset
			val &= (1 << self.mask_size) - 1
		
		if self.scale != 0.0:
			val = self.scale * val + self.offset
		
		if self.curr_val is None:
			self.prev_val = val
		else:
			self.prev_val = self.curr_val
		
		self.curr_val = val
		
		if self.min_val is None or val < self.min_val:
			self.min_val = val
		
		if self.max_val is None or val > self.max_val:
			self.max_val = val
		
		self.one_in_n_count += 1
		
		if self.one_in_n_count >= self.one_in_n:
			self.one_in_n_count = 0
			
			if self.graph_style == 'bar':
				s = self._visualize_bar()
			elif self.graph_style == 'line':
				s = self._visualize_line()
			else:
				s = self._visualize_none()
			
			print(s)


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
		description='A console-based TLV/INM memory monitor visualization tool.',
		fromfile_prefix_chars='@')
	
	arg_p.add_argument('-A', '--inm-adr',
		help='INM address to receive messages for', metavar='INM_ADR',
		default=0, type=int, dest='inm_adr')
	arg_p.add_argument('-I', '--ip-adr',
		help='IP address of interface to receive messages at', metavar='IP_ADR',
		nargs='?', default=None, const=DEFAULT_IP_ADR_VAL, dest='ip_adr')
	arg_p.add_argument('-U', '--udp-port',
		help='UDP port to receive messages at', metavar='UDP_PORT',
		default=DEFAULT_UDP_PORT_VAL, type=int, dest='udp_port')
	arg_p.add_argument('-S', '--serial-port',
		help='serial port to receive messages from', metavar='SERIAL',
		nargs='?', default=None, const=DEFAULT_SERIAL_PORT_VAL, dest='serial_port')
	
	arg_p.add_argument('-G', '--graph-style',
		help='graph style', metavar='STYLE',
		default='bar', dest='graph_style')
	arg_p.add_argument('--bar',
		help='bar graph character', metavar='CHAR',
		default='|', dest='bar_char')
	arg_p.add_argument('--rise',
		help='line graph rising character', metavar='CHAR',
		default='\\', dest='rise_char')
	arg_p.add_argument('--flat',
		help='line graph flat character', metavar='CHAR',
		default='|', dest='flat_char')
	arg_p.add_argument('--fall',
		help='line graph falling character', metavar='CHAR',
		default='/', dest='fall_char')
	arg_p.add_argument('--oor',
		help='out-of-range character', metavar='CHAR',
		default='@', dest='oor_char')
	
	arg_p.add_argument('-L', '--range-low',
		help='lower bound of visualized range', metavar='BOUND',
		default='0.0', dest='range_low')
	arg_p.add_argument('-H', '--range-high',
		help='upper bound of visualized range', metavar='BOUND',
		default='max', dest='range_high')
	arg_p.add_argument('-W', '--graph-width',
		help='width in columns of visualization graph', metavar='WIDTH',
		default=60, type=int, dest='graph_width')
	
	# TODO: Validate the specified format string.
	arg_p.add_argument('-f', '--label-format',
		help='value label numeric format string', metavar='FORMAT',
		default='4d', dest='label_format')
	arg_p.add_argument('-X', '--label-minmax',
		help='include min and max in value labels', action='store_true',
		dest='label_minmax')
	
	arg_p.add_argument('-t', '--tlv-type',
		help='TLV message type of monitor data messages', metavar='TYPE',
		default=DEFAULT_TLV_TYPE, type=int, dest='mon_msg_typ')
	arg_p.add_argument('-T', '--text-vis',
		help='output non-monitor messages as text',
		action='store_true', dest='text_vis')
	# TODO: Add support for several visualized monitors in the same console window.
	arg_p.add_argument('-i', '--index',
		help='monitor index to visualize', metavar='INDEX',
		default=0, type=int, dest='mon_i')
	arg_p.add_argument('--mask-size',
		help='value bitmask size', metavar='BITS',
		default=0, type=int, dest='mask_size')
	arg_p.add_argument('--mask-offset',
		help='value bitmask offset', metavar='BITS',
		default=0, type=int, dest='mask_offset')
	
	# TODO: Add selectable value aggregation (e.g. min, max, average).
	arg_p.add_argument('-N',
		help='print one output line for every N messages received', metavar='N',
		default=1, type=int, dest='one_in_n')
	arg_p.add_argument('-F', '--scale',
		help='raw monitor value scaling', metavar='SCALE',
		default=0.0, type=float, dest='scale')
	arg_p.add_argument('-O', '--offset',
		help='raw monitor value offset', metavar='OFFSET',
		default=0.0, type=float, dest='offset')
	
	return arg_p.parse_args(args, namespace)


def main():
	viz = Visualizer()
	_parse_args(namespace=viz)
	
	# TODO: Add argument validation (negative values, etc.).
	
	inm.default_msg_factory.default_val_conv = inm.ValueConversions.Hex
	
	if viz.ip_adr is not None:
		print(f'Opening IP message channel: ip_adr={viz.ip_adr} udp_port={viz.udp_port}')
		ch = inm.InetMessageChannel(viz.inm_adr, viz.ip_adr, viz.udp_port)
	elif viz.serial_port is not None:
		print(f'Opening serial message channel: {viz.serial_port}')
		ch = inm.SerialMessageChannel(viz.inm_adr, viz.serial_port)
	else:
		print('No message channel specified. Exiting.')
		sys.exit(1)
	
	try:
		with ch:
			while True:
				res, header, msg, link_adr = ch.recv()
				
				if res == inm.ResultCode.SUCCESS:
					if msg.typ == viz.mon_msg_typ:
						viz.visualize(msg)
					elif viz.text_vis:
						print(inm.format_msg_info(msg, 'RECV', header, link_adr, 8, '%H:%M:%S.%f'))
				else:
					print(inm.format_msg_info(res.name, 'ERROR', header, link_adr, 8, '%H:%M:%S.%f'))
	except KeyboardInterrupt:
		print('Interrupted. Cleaning up and exiting.')

if __name__ == '__main__':
	main()
