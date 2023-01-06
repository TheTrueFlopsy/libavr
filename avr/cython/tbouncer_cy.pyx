
from libc.stdint cimport uint8_t

cimport tbouncer

def init(uint8_t b_mask, uint8_t c_mask, uint8_t d_mask):
	cdef tbouncer.sched_time poll_delay
	poll_delay.l = 0
	poll_delay.h = 0
	
	tbouncer.tbouncer_init(
		0, poll_delay,
		0, b_mask, c_mask, d_mask,
		0, 0, 0)

def shutdown():
	tbouncer.tbouncer_update(0, 0, 0, 0, 0)  # Clear all PIN registers.
	tbouncer.tbouncer_shutdown()

def reset(uint8_t b_mask, uint8_t c_mask, uint8_t d_mask):
	cdef tbouncer.sched_time poll_delay
	poll_delay.l = 0
	poll_delay.h = 0
	
	tbouncer.tbouncer_update(0, 0, 0, 0, 0)  # Clear all PIN registers.
	tbouncer.tbouncer_shutdown()
	tbouncer.tbouncer_init(
		0, poll_delay,
		0, b_mask, c_mask, d_mask,
		0, 0, 0)

def update(uint8_t pinb, uint8_t pinc, uint8_t pind):
	tbouncer.tbouncer_update(0, pinb, pinc, pind, 0)
	return tbouncer.tbouncer_b, tbouncer.tbouncer_c, tbouncer.tbouncer_d
