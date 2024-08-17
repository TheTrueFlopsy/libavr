
import sys

import inm.inm as inm


NRF24X_OUT       = inm.StandardTypes.APPLICATION + 0
NRF24X_IN        = inm.StandardTypes.APPLICATION + 1
NRF24X_IN_RES    = inm.StandardTypes.APPLICATION + 2

NRF24X_STATUS    = inm.StandardRegisters.APPLICATION + 0
NRF24X_CMD_OUT_0 = inm.StandardRegisters.APPLICATION + 1
NRF24X_IOPINS    = inm.StandardRegisters.APPLICATION + 2
NRF24X_REG_START = inm.StandardRegisters.APPLICATION + 0x20

NRF24X_RX_DR  = 1 << 6
NRF24X_TX_DS  = 1 << 5
NRF24X_MAX_RT = 1 << 4

NRF24X_R_RX_PAYLOAD = 0b01100001  # 0x61
NRF24X_W_TX_PAYLOAD = 0b10100000  # 0xa0

verbose = False

def is_nrf24x_reg_num(n):
	return 0x00 <= n < 0x20

def is_nrf24x_reg(reg):
	return is_nrf24x_reg_num(reg - NRF24X_REG_START)

def nrf24x_reg(n):
	return NRF24X_REG_START + n if is_nrf24x_reg_num(n) else None

def nrf24x_reg_num(reg):
	return reg - NRF24X_REG_START if is_nrf24x_reg(reg) else None

def reg_to_str(reg):
	nrf_reg_num = nrf24x_reg_num(reg)
	
	if nrf_reg_num is not None:
		return f'{reg:#04x} ({nrf_reg_num:#04x})'
	else:
		return f'{reg:#04x}'

def abort(reason=None):
	if reason is not None:
		print('ABORT: ' + reason)
	sys.exit(1)

def abort_on_fail(res, reason=None):
	if res != inm.ResultCode.SUCCESS:
		if reason is None:
			reason = f'INM operation failed with result code {res.name}.'
		abort(reason)

def get_reg(h, dstadr, reg):
	res, header, msg, link_adr = h.sendrecv(dstadr, h.Typ.REG_READ, reg)
	abort_on_fail(res)
	
	if msg.typ != h.Typ.INM_REG_READ_RES:
		abort(f'Expected INM_REG_READ_RES, got {msg.typ}.')
	
	r_index, reg_val, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_REG_READ_RES payload.
	
	if verbose:
		print(f'Got value {reg_val:#04x} from register {reg_to_str(reg)}.')
	
	return reg_val

def get_reg_assert(h, dstadr, reg, expected_val):
	reg_val = get_reg(h, dstadr, reg)
	if reg_val != expected_val:  # unexpected value
		abort(f'Got value {reg_val:#04x} from register {reg_to_str(reg)}, expected {expected_val:#04x}.')

def get_regpair(h, dstadr, reg):
	res, header, msg, link_adr = h.sendrecv(dstadr, h.Typ.REGPAIR_READ, reg)
	abort_on_fail(res)
	
	if msg.typ != h.Typ.INM_REGPAIR_READ_RES:
		abort(f'Expected INM_REGPAIR_READ_RES, got {msg.typ}.')
	
	r_index, regpair_val, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_REGPAIR_READ_RES payload.
	
	if verbose:
		print(f'Got value {regpair_val:#06x} from register pair {reg:#04x}.')
	
	return regpair_val

def set_reg(h, dstadr, reg, reg_val):
	if verbose:
		print(f'Setting value {reg_val:#04x} in register {reg_to_str(reg)}.')
	
	res, header, msg, link_adr = h.sendrecv_mval(dstadr, h.Typ.REG_WRITE, (reg, reg_val))
	abort_on_fail(res)
	
	if msg.typ != h.Typ.INM_RESULT:
		abort(f'Expected INM_RESULT, got {msg.typ}.')
	
	std_res, req_id = msg.format_mval(conv=h.VC.Int)  # Unpack INM_RESULT payload.
	if std_res != h.Res.OK:  # unsuccessful register write at destination
		abort(f'Register write at destination failed with result {std_res}.')
