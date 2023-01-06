
#include <stdio.h>
#include <stdlib.h>
#include <threads.h>
#include <time.h>

#include "tbouncer.h"

// NOTE: #X means "wrap the value of X in double quotes, without macro expansion".
#define PRINT_SYM(S, F) printf(#S "=%" #F, S)
#define PRINTLN_SYM(S, F) printf(#S "=%" #F "\n", S)

static void print_digital(uint8_t prev, uint8_t curr) {
	if (curr && !prev)
		fputs("->|", stdout);
	else if (!curr && prev)
		fputs("|<-", stdout);
	else if (curr)
		fputs("  |", stdout);
	else
		fputs("|  ", stdout);
}

int main(int argc, char **argv) {
	//int n_samples = -1;
	int n_samples = 256;
	long repeat_max = 8;
	long sleep_ns = 100000000;
	uint8_t prev_x = 0, x = 0, y = 0;
	
	struct timespec sleep_time = { .tv_sec = 0, .tv_nsec = sleep_ns };
	
	srand(time(NULL));
	
	tbouncer_init(
		0, SCHED_TIME_ZERO,
		0, 1, 0, 0,
		0, 0, 0);
	
	fputs("             =X=   =Y=\n", stdout);
	
	for (int i = 0; i != n_samples; ) {
		int r = 1 + (int)((repeat_max * rand()) / RAND_MAX);
		
		for (int j = 0; j < r && i != n_samples; j++, i++) {
			uint8_t prev_y = y;
			
			tbouncer_update(0, x, 0, 0, 0);
			y = tbouncer_b;
			
			PRINT_SYM(i, 08d);
			fputs("   ", stdout);
			print_digital(prev_x, x);
			fputs("   ", stdout);
			print_digital(prev_y, y);
			fputs("\n", stdout);
			
			prev_x = x;
		}
		
		x = !x;
		thrd_sleep(&sleep_time, NULL);
	}
	
	tbouncer_shutdown();
	return 0;
}
