#!/usr/bin/python3

import random
import unittest

import tbouncer_cy


# ---<<< Helpers >>>---
class TestSignal:
	def __init__(self, pin_id, inpt, outpt):
		# NOTE: A pin ID is a (PORT, BIT) pair, where PORT is a single-character string
		#       and BIT is an integer in the interval [0,7].
		self.pin_id = pin_id
		self.inpt = inpt
		self.outpt = outpt

def try_get_at(seq, i, default=0):
	return seq[i] if i < len(seq) else default

def try_get_for_at(seq_table, seq_id, i, default=0):
	return try_get_at(seq_table[seq_id], i, default) if seq_id in seq_table else default

def pack_signals(signal_table, i):
	b, c, d = 0, 0, 0
	
	for bit in range(8):
		pin_id = ('b', bit)
		pin_value = try_get_for_at(signal_table, pin_id, i)
		b |= pin_value << bit
	
	for bit in range(8):
		pin_id = ('c', bit)
		pin_value = try_get_for_at(signal_table, pin_id, i)
		c |= pin_value << bit
	
	for bit in range(8):
		pin_id = ('d', bit)
		pin_value = try_get_for_at(signal_table, pin_id, i)
		d |= pin_value << bit
	
	return b, c, d

def check_output(tcase, port, i, exp, act):
	tcase.assertEqual(exp, act, f'{port}[{i}]: expected {exp:08b}, got {act:08b}.')

def apply_signals(tcase, signals):
	in_table = { s.pin_id : s.inpt for s in signals if s.inpt is not None }
	out_table = { s.pin_id : s.outpt for s in signals if s.outpt is not None }
	max_in_len = max(len(inpt) for inpt in in_table.values())
	max_out_len = max(len(outpt) for outpt in out_table.values())
	
	for i in range(max(max_in_len, max_out_len)):
		in_b, in_c, in_d = pack_signals(in_table, i)
		exp_out_b, exp_out_c, exp_out_d = pack_signals(out_table, i)
		
		act_out_b, act_out_c, act_out_d = tbouncer_cy.update(in_b, in_c, in_d)
		
		check_output(tcase, 'b', i, exp_out_b, act_out_b)
		check_output(tcase, 'c', i, exp_out_c, act_out_c)
		check_output(tcase, 'd', i, exp_out_d, act_out_d)

def random_input(n, max_between):
	return [ i % 2 for i in range(n) for j in range(random.randrange(1, max_between+1)) ]

def input_to_output(inpt, min_between=5, min_neq=2):
	prologue_len = min_between + min_neq - 1
	outpt = [ 0 for k in range(len(inpt)) ]
	
	if len(inpt) < prologue_len:
		return outpt
	
	if all(inpt[(prologue_len - min_neq):prologue_len]):
		outpt[prologue_len-1] = 1
	
	flipper = (1, 0)
	for j in range(prologue_len, len(inpt)):
		o1 = outpt[j-1]
		o0 = o1
		
		if (all(outpt[j-k] == o1 for k in range(2, prologue_len+1)) and
		    all(inpt[j-k] != o1 for k in range(0, min_neq))):
			o0 = flipper[o0]
		
		outpt[j] = o0
	
	return outpt


# ---<<< Test Cases >>>---
# NOTE: A new instance will be created for each test method invocation.
class TbouncerTest(unittest.TestCase):
	@classmethod
	def setUpClass(cls):
		random.seed()  # Do setup for the whole test class here.
	
	@classmethod
	def tearDownClass(cls):
		pass  # TODO: Do teardown/cleanup for the whole test class here.
	
	def setUp(self):
		pass  # TODO: Do setup for each test method invocation here.
	
	def tearDown(self):
		tbouncer_cy.shutdown()  # Do teardown/cleanup for each test method invocation here.
	
	def test_sanity(self):
		tbouncer_cy.init(0b00000001, 0, 0)
		
		inpt  = (0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0)
		outpt = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0)
		signals = (TestSignal(('b', 0), inpt, outpt),)
		apply_signals(self, signals)
	
	def test_spikes(self):
		tbouncer_cy.init(0, 0b00010000, 0b00001000)
		
		inpt_0  = (0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0)
		inpt_1  = (1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1)
		outpt_1 = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)
		signals = (TestSignal(('c', 4), inpt_0, None), TestSignal(('d', 3), inpt_1, outpt_1))
		apply_signals(self, signals)
	
	def test_debounce(self):
		tbouncer_cy.init(0b10000000, 0, 0)
		
		inpt  = (0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0)
		outpt = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0)
		signals = (TestSignal(('b', 7), inpt, outpt),)
		apply_signals(self, signals)
	
	def test_reset(self):
		tbouncer_cy.init(0b00000001, 0, 0)
		
		inpt  = (0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0)
		outpt = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0)
		signals = (TestSignal(('b', 0), inpt, outpt),)
		apply_signals(self, signals)
		
		tbouncer_cy.reset(0, 0b00010000, 0b00001000)
		
		inpt_0  = (0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0)
		inpt_1  = (1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1)
		outpt_1 = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1)
		signals = (TestSignal(('c', 4), inpt_0, None), TestSignal(('d', 3), inpt_1, outpt_1))
		apply_signals(self, signals)
		
		tbouncer_cy.reset(0b10000000, 0, 0)
		
		inpt  = (0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0)
		outpt = (0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0)
		signals = (TestSignal(('b', 7), inpt, outpt),)
		apply_signals(self, signals)
	
	def test_random(self):
		tbouncer_cy.init(0, 0, 0b00000010)
		
		inpt = random_input(25, 8)
		outpt = input_to_output(inpt)
		signals = (TestSignal(('d', 1), inpt, outpt),)
		apply_signals(self, signals)

if __name__ == '__main__':
	unittest.main()
