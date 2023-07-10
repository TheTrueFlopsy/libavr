#!/usr/bin/python3

import time

import inm.helper as helper

import nrf24x
from nrf24x import *

dstadr = 2
w_cmd = bytes((NRF24X_W_TX_PAYLOAD,))
payload = b'HELO'
w_data = w_cmd + payload

nrf24x.verbose = True

with helper.InmHelper() as h:  # Create InmHelper with default message channel.
	# Ask INM node 1 about its firmware version.
	fw_ver = get_reg(h, h.Reg.FWVERSION)
	print(f'Node {dstadr} has firmware version {fw_ver}.')
	
	# Ask INM node 1 about its firmware ID.
	fw_id = get_regpair(h, h.Reg.FWID)
	print(f'Node {dstadr} has firmware ID {fw_id}.')
	
	# Read register CONFIG/0x00 from nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x00), 0x08)
	
	# Read register EN_AA/0x01 at nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x01), 0x3f)
	
	# Read register SETUP/0x06 from nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x06), 0x0f)
	
	# Read register STATUS/0x07 from nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x07), 0x0e)
	
	# Read register FIFO_STATUS/0x17 from nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x17), 0x11)
	
	# Set PWR_UP bit in CONFIG register.
	set_reg(h, nrf24x_reg(0x00), 0x0a)
	time.sleep(0.01)  # Sleep for 10 milliseconds.
	get_reg_assert(h, nrf24x_reg(0x00), 0x0a)
	
	# Load data for transmission with the W_TX_PAYLOAD command.
	res, header, msg, link_adr = h.sendrecv(dstadr, NRF24X_OUT, w_data)
	abort_on_fail(res)
	
	if msg.typ != h.Typ.INM_RESULT:
		abort(f'Expected INM_RESULT, got {msg.typ}.')
	
	std_res, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_RESULT payload.
	if std_res != h.Res.OK:  # unsuccessful data load at INM destination
		abort(f'Data load at destination failed with result {std_res}.')
	
	# Read register FIFO_STATUS/0x17 from nRF24 chip.
	get_reg_assert(h, nrf24x_reg(0x17), 0x01)
	
	# Drive the CE pin high for more than 10 μs to trigger transmission.
	set_reg(h, NRF24X_IOPINS, 0x01)
	get_reg_assert(h, NRF24X_IOPINS, 0x01)
	set_reg(h, NRF24X_IOPINS, 0x00)
	get_reg_assert(h, NRF24X_IOPINS, 0x00)
	
	print('DONE')
