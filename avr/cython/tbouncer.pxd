
from libc.stdint cimport uint8_t, uint16_t

cdef extern from "tbouncer.h":
	ctypedef struct sched_time:
		uint8_t l
		uint16_t h
	
	ctypedef uint16_t sched_catflags
	
	uint8_t tbouncer_b_mask
	uint8_t tbouncer_b
	uint8_t tbouncer_b_prev
	uint8_t tbouncer_b_diff
	
	uint8_t tbouncer_c_mask
	uint8_t tbouncer_c
	uint8_t tbouncer_c_prev
	uint8_t tbouncer_c_diff
	
	uint8_t tbouncer_d_mask
	uint8_t tbouncer_d
	uint8_t tbouncer_d_prev
	uint8_t tbouncer_d_diff
	
	void tbouncer_init(
		uint8_t task_num_cat, sched_time poll_delay,
		uint8_t a_mask, uint8_t b_mask, uint8_t c_mask, uint8_t d_mask,
		sched_catflags awaken_cats, uint8_t invoke_mask, uint8_t invoke_st)
	
	void tbouncer_shutdown()
	
	void tbouncer_update(uint8_t pina, uint8_t pinb, uint8_t pinc, uint8_t pind, uint8_t pinf)
