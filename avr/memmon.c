
#include <avr/io.h>

#include "memmon.h"

// ISSUE: This is peeping at avr-libc implementation details.
//        Is there a less hacky way to do it?
extern uint8_t __bss_end; // End of statically allocated RAM.
extern uint8_t *__brkval; // End of stdlib heap.

uint8_t memmon_max_monitors;
uint8_t memmon_n_monitors;

uint16_t memmon_msg_count;
uint16_t memmon_drop_count;
ptrdiff_t memmon_free_ram;

static memmon_spec *monitors;

static uint8_t task_st_num_cat;
static sched_time task_delay;

static uint8_t msg_tlv_typ;
static uint8_t msg_inm_dstadr;

static void memmon_handler(sched_task *task) {
	memmon_header head;
	
	memmon_free_ram = memmon_get_free_ram();
	
	for (head.mon_i = 0; head.mon_i < memmon_n_monitors; head.mon_i++) {
		memmon_spec *mon = monitors + head.mon_i;
		uint8_t delay_count = mon->delay_count + 1;
		
		if (delay_count == mon->delay_div) {
			uint8_t n_bytes = mon->size;
			
			mon->delay_count = 0; // Reset monitor delay count.
			memmon_msg_count++;
			
			head.ptr = mon->ptr;
			if (!ttlv_try_put_bytes(sizeof(head), MEMMON_MAKE_PTR(&head)))
				goto msg_dropped;
			
			if (!ttlv_try_put_bytes(n_bytes, head.ptr))
				goto msg_dropped;
			
			ttlv_xmit_inm_header.h.dstadr = msg_inm_dstadr;
			ttlv_xmit_header.h.type = msg_tlv_typ;
			ttlv_xmit_header.h.length = sizeof(head) + n_bytes;
			
			if (TTLV_IS_ERROR(ttlv_try_begin_xmit()))
				goto msg_dropped;
		}
		else
			mon->delay_count = delay_count;

		continue;
msg_dropped:
		memmon_drop_count++;
	}
	
	task->delay = task_delay; // Reset task delay.
}

ptrdiff_t memmon_get_free_ram(void) {
	if (__brkval == NULL) // Heap not initialized.
		return (const uint8_t*)AVR_STACK_POINTER_REG - &__bss_end;
	else
		return (const uint8_t*)AVR_STACK_POINTER_REG - __brkval;
}

void memmon_init(
	uint8_t task_num_cat, uint8_t max_mon, memmon_spec *mon_array,
	sched_time delay, uint8_t tlv_typ, uint8_t inm_dstadr)
{
	sched_task task;
	
	memmon_max_monitors = max_mon;
	memmon_n_monitors = 0;
	
	memmon_msg_count = 0;
	memmon_drop_count = 0;
	
	monitors = mon_array;
	
	task_st_num_cat = TASK_ST_NUM_CAT_MASK & task_num_cat;
	task_delay = delay;
	
	msg_tlv_typ = tlv_typ;
	msg_inm_dstadr = inm_dstadr;
	
	// Create memory monitoring task.
	task = (sched_task) {
		.st = task_num_cat,
		.delay = task_delay,
		.handler = memmon_handler
	};
	sched_add(&task);
}

uint8_t memmon_add(const memmon_spec *mon) {
	uint8_t mon_i;
	
	if (memmon_n_monitors >= memmon_max_monitors)
		return memmon_max_monitors;
	
	if (mon->size > MEMMON_MAX_SIZE)
		return memmon_max_monitors;
	
	mon_i = memmon_n_monitors;
	monitors[mon_i] = *mon;
	monitors[mon_i].delay_count = 0;
	memmon_n_monitors++;
	
	return mon_i;
}

uint8_t memmon_remove(uint8_t mon_i) {
	uint8_t prev_mon_i;
	
	if (mon_i >= memmon_n_monitors)
		return 0;
	
	prev_mon_i = mon_i;
	
	for (mon_i++; mon_i < memmon_n_monitors; mon_i++) {
		monitors[prev_mon_i] = monitors[mon_i];
		prev_mon_i = mon_i;
	}
	
	memmon_n_monitors--;
	
	return 1;
}

uint8_t memmon_remove_ptr(const uint8_t *mon_ptr) {
	uint8_t mon_i;
	
	for (mon_i = 0; mon_i < memmon_n_monitors; mon_i++)
		if (monitors[mon_i].ptr == mon_ptr)
			return memmon_remove(mon_i);
	
	return 0;
}

void memmon_shutdown(void) {
	sched_remove(TASK_ST_NUM_CAT_MASK, task_st_num_cat, 0);
}
