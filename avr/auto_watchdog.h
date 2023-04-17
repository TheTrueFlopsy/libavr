#ifndef AVR_AUTO_WATCHDOG_H
#define AVR_AUTO_WATCHDOG_H

/**
	File: auto_watchdog.h
	When this header is included in a firmware code file, the watchdog timer will be
	automatically disabled (via <WATCHDOG_DISABLE_ON_MCU_RESET>) during firmware
	initialization, before *main* is called.
	
	CAUTION: This header MUST NOT be included in more than one code file in the same
	firmware build. The recommended approach is to include it in the file that contains
	the *main* function, and only there.
	
	NOTE: If the macro *WATCHDOG_AUTODISABLE_SAVE_FLAGS* is defined, this header
	will include <WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS> instead of
	<WATCHDOG_DISABLE_ON_MCU_RESET>.
	
	NOTE: If the macro *WATCHDOG_DO_NOT_AUTODISABLE* is defined
	(and *WATCHDOG_AUTODISABLE_SAVE_FLAGS* is undefined), this header will NOT
	include any macro that disables the watchdog timer.
*/

#include "watchdog.h"

#if defined(WATCHDOG_AUTODISABLE_SAVE_FLAGS)
WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS
#elif defined(WATCHDOG_DO_NOT_AUTODISABLE)
#warning "Code disabling the watchdog timer on MCU reset not implicitly inserted."
#else
WATCHDOG_DISABLE_ON_MCU_RESET
#endif

#endif
