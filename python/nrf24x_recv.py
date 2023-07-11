#!/usr/bin/python3

import time

import inm.helper as helper

import nrf24x
from nrf24x import *

dstadr = 1
r_cmd = bytes((NRF24X_R_RX_PAYLOAD,))
payload = b'HELO'

nrf24x.verbose = True

with helper.InmHelper() as h:  # Create InmHelper with default message channel.
	# Ask INM node 1 about its firmware version.
	fw_ver = get_reg(h, dstadr, h.Reg.FWVERSION)
	print(f'Node {dstadr} has firmware version {fw_ver}.')
	
	# Ask INM node 1 about its firmware ID.
	fw_id = get_regpair(h, dstadr, h.Reg.FWID)
	print(f'Node {dstadr} has firmware ID {fw_id}.')
	
	# Read register CONFIG/0x00 from nRF24 chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x08)
	
	# Read register EN_AA/0x01 at nRF24 chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x01), 0x3f)
	
	# Read register SETUP/0x06 from nRF24 chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x06), 0x0f)
	
	# Read register STATUS/0x07 from nRF24 chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x0e)
	
	# Read register FIFO_STATUS/0x17 from nRF24 chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x11)
	
	# Set payload length in RX_PW_P0 register.
	set_reg(h, dstadr, nrf24x_reg(0x11), 0x04)
	get_reg_assert(h, dstadr, nrf24x_reg(0x11), 0x04)
	
	# Set PRIM_RX bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x09)
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x09)
	
	# Set PWR_UP bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x0b)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x0b)
	
	# Drive the CE pin high to enter RX Mode.
	set_reg(h, dstadr, NRF24X_IOPINS, 0x01)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x01)
	
	while True:
		time.sleep(0.01)  # Sleep for 10 milliseconds.
		
		# Read register STATUS/0x07 from nRF24 chip.
		nrf24x_st = get_reg(h, dstadr, nrf24x_reg(0x07))
		
		if not (NRF24X_RX_DR & nrf24x_st):
			continue  # No packet received.
		
		# Drive the CE pin low to exit RX Mode.
		set_reg(h, dstadr, NRF24X_IOPINS, 0x00)
		get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x00)
		
		# Clear RX_DR bit in STATUS register.
		set_reg(h, dstadr, nrf24x_reg(0x07), NRF24X_RX_DR)
		get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x0e)
		
		# Fetch received data with the R_RX_PAYLOAD command.
		res, header, msg, link_adr = h.sendrecv(dstadr, NRF24X_IN, r_cmd)
		abort_on_fail(res)
		
		if msg.typ != NRF24X_IN_RES:
			abort(f'Expected NRF24X_IN_RES, got {msg.typ}.')
		
		if len(msg) != len(payload):
			abort(f'Expected {len(payload)} bytes, got {len(msg)}.')
		
		r_data = msg.format(conv=h.VC.Bytes)  # Get NRF24X_IN_RES payload.
		if r_data != payload:  # unsuccessful data fetch from INM destination
			abort(f'Expected payload {payload}, got {r_data}.')
		
		break
	
	print('DONE')
