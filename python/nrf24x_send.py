#!/usr/bin/python3

import time

import inm.helper as helper

import nrf24x
from nrf24x import *

srcadr_ = 95
udp_port_ = 2995
dstadr = 1
w_cmd = bytes((NRF24X_W_TX_PAYLOAD,))
payload = b'HELO'
w_data = w_cmd + payload

nrf24x.verbose = True  # Log each nRF24x register operation on standard output.

with helper.InmHelper(srcadr=srcadr_, udp_port=udp_port_) as h:
	# Ask target INM node about its firmware version.
	fw_ver = get_reg(h, dstadr, h.Reg.FWVERSION)
	print(f'Node {dstadr} has firmware version {fw_ver:#04x}.')
	
	# Ask target INM node about its firmware ID.
	fw_id = get_regpair(h, dstadr, h.Reg.FWID)
	print(f'Node {dstadr} has firmware ID {fw_id:#06x}.')
	
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
	
	# Disable auto-acknowledge in EN_AA register.
	set_reg(h, dstadr, nrf24x_reg(0x01), 0x00)
	get_reg_assert(h, dstadr, nrf24x_reg(0x01), 0x00)
	
	# Disable auto-retransmit in SETUP_RETR register.
	set_reg(h, dstadr, nrf24x_reg(0x04), 0x00)
	get_reg_assert(h, dstadr, nrf24x_reg(0x04), 0x00)
	
	# Set PWR_UP bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x0a)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x0a)
	
	# Load data for transmission with the W_TX_PAYLOAD command.
	res, header, msg, link_adr = h.sendrecv(dstadr, NRF24X_OUT, w_data)
	abort_on_fail(res)
	
	if msg.typ != h.Typ.INM_RESULT:
		abort(f'Expected INM_RESULT, got {msg.typ}.')
	
	std_res, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_RESULT payload.
	if std_res != h.Res.OK:  # unsuccessful data load at INM destination
		abort(f'Data load at destination failed with result {std_res}.')
	
	# Read register FIFO_STATUS/0x17 from nRF24x chip.
	# NOTE: I'm seeing odd behavior, where the first read of FIFO_STATUS after
	# W_TX_PAYLOAD still reports TX_EMPTY=1, but the second reports TX_EMPTY=0
	# as expected. Am I doing something wrong? If so, where?
	#get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x11)
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x01)
	
	# Drive the CE pin high for more than 10 μs to trigger transmission.
	set_reg(h, dstadr, NRF24X_IOPINS, 0x01)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x01)
	set_reg(h, dstadr, NRF24X_IOPINS, 0x00)
	get_reg_assert(h, dstadr, NRF24X_IOPINS, 0x00)
	
	time.sleep(0.1)  # Sleep for 100 milliseconds.
	
	# Read register STATUS/0x07 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x2e)
	
	# Read register FIFO_STATUS/0x17 from nRF24x chip.
	get_reg_assert(h, dstadr, nrf24x_reg(0x17), 0x11)
	
	# Clear TX_DS bit in STATUS register.
	set_reg(h, dstadr, nrf24x_reg(0x07), NRF24X_TX_DS)
	get_reg_assert(h, dstadr, nrf24x_reg(0x07), 0x0e)
	
	# Clear PWR_UP bit in CONFIG register.
	set_reg(h, dstadr, nrf24x_reg(0x00), 0x08)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, dstadr, nrf24x_reg(0x00), 0x08)
	
	print('DONE')
