#!/usr/bin/python3

import time

import inm.helper as helper

import nrf24x
from nrf24x import *

srcadr_ = 96
udp_port_ = 2996
timeout_ = 1
dstadr = 2
r_cmd = bytes((NRF24X_R_RX_PAYLOAD,))
payload = b'HELO'
r_data = r_cmd + bytes((len(payload),))

nrf24x.verbose = True  # Log each nRF24x register operation on standard output.

with helper.InmHelper(srcadr=srcadr_, udp_port=udp_port_, timeout=timeout_) as h:
	# Ask target INM node about its firmware version.
	fw_ver = get_reg(h, dstadr, h.Reg.FWVERSION)
	print(f'Node {dstadr} has firmware version {fw_ver:#04x}.')
	
	# Ask target INM node about its firmware ID.
	fw_id = get_regpair(h, dstadr, h.Reg.FWID)
	print(f'Node {dstadr} has firmware ID {fw_id:#06x}.')
	
	# Power up the nRF24x.
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x04)
	set_reg(h, dstadr, NRF24X_IOPINS, 0x00)
	time.sleep(0.2)  # Sleep for 200 milliseconds.
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x02)
	
	# Read register CONFIG/0x00 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x08)
	
	# Read register EN_AA/0x01 at nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x01), 0x3f)
	
	# Read register SETUP_RETR/0x04 at nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x04), 0x03)
	
	# Read register SETUP/0x06 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x06), 0x0f)
	
	# Read register STATUS/0x07 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x0e)
	
	# Read register FIFO_STATUS/0x17 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x11)
	
	# Set PRIM_RX bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x09)
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x09)
	
	# Disable auto-acknowledge in EN_AA register.
	set_reg(h, dstadr, nrf24x_reg(0x01), 0x00)
	get_reg_assert(h, dstadr, nrf24x_reg(0x01), 0x00)
	
	# Disable auto-retransmit in SETUP_RETR register.
	set_reg(h, dstadr, nrf24x_reg(0x04), 0x00)
	get_reg_assert(h, dstadr, nrf24x_reg(0x04), 0x00)
	
	# Set payload length in RX_PW_P0 register.
	set_reg(h, dstadr, nrf24x_reg(0x11), 0x04)
	get_reg_assert(h, dstadr, nrf24x_reg(0x11), 0x04)
	
	# Set PWR_UP bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x0b)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x0b)
	
	# Drive the CE pin high to enter RX Mode.
	set_reg(h, dstadr, NRF24X_IOPINS, 0x01)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x03)
	
	print('Waiting for packet...')
	nrf24x.verbose = False
	
	while True:
		time.sleep(0.01)  # Sleep for 10 milliseconds.
		
		# Read register STATUS/0x07 from nRF24x chip.
		nrf24x_st = get_reg(h, dstadr, nrf24x_reg(0x07))
		
		if NRF24X_RX_DR & nrf24x_st:
			break  # Packet received!
	
	nrf24x.verbose = True
	
	# Packet received. Drive the CE pin low to exit RX Mode.
	set_reg(h, dstadr, NRF24X_IOPINS, 0x00)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x00)
	
	# Read register FIFO_STATUS/0x17 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x10)
	
	# Clear RX_DR bit in STATUS register.
	set_reg(h, dstadr, nrf24x_reg(0x07), NRF24X_RX_DR)
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x00)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x02)
	
	# Fetch received data with the R_RX_PAYLOAD command.
	res, header, msg, link_adr = h.sendrecv(dstadr, NRF24X_IN, r_data)
	abort_on_fail(res)
	
	if msg.typ != NRF24X_IN_RES:
		abort(f'Expected NRF24X_IN_RES, got {msg.typ}.')
	
	if len(msg) != len(payload):
		abort(f'Expected {len(payload)} bytes, got {len(msg)}.')
	
	r_data = msg.format_val(conv=h.VC.Bytes)  # Get NRF24X_IN_RES payload.
	if r_data != payload:  # unsuccessful data fetch from INM destination
		abort(f'Expected payload {payload}, got {r_data}.')
	
	print(f'Payload {r_data} received!')
	
	# Read register STATUS/0x07 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x0e)
	
	# Read register FIFO_STATUS/0x17 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x11)
	
	# Clear PWR_UP bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x09)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x09)
	
	# Power down the nRF24x.
	set_reg(h, dstadr, NRF24X_IOPINS, 0x04)
	time.sleep(0.2)  # Sleep for 200 milliseconds.
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x04)
	
	print('DONE')
