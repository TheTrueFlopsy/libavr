#!/usr/bin/python3

import inm.helper as helper

dstadr = 1
debug0_val = 0x02

with helper.InmHelper() as h:  # Create InmHelper with default message channel.
	# Ask INM node 1 about its firmware version.
	res, header, msg, link_adr = h.sendrecv(dstadr, h.Typ.REG_READ, h.Reg.FWVERSION)
	
	if res == h.RC.SUCCESS:  # send/receive operation succeeded
		r_index, fw_ver, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_REG_READ_RES payload.
		print(f'Node {dstadr} has firmware version {fw_ver}.')
	
	# Write to debug register 0 at node 1.
	res, header, msg, link_adr = h.sendrecv_mval(dstadr, h.Typ.REG_WRITE, (h.Reg.DEBUG0, debug0_val))
	
	if res == h.RC.SUCCESS:  # send/receive operation succeeded
		std_res, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_RESULT payload.
		if std_res == h.Res.OK:  # successful register write at destination
			print(f'Wrote {debug0_val:#04x} to DEBUG0.')
