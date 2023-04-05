#ifndef AVR_AUTO_WATCHDOG_H
#define AVR_AUTO_WATCHDOG_H

#include "watchdog.h"

#if defined(WATCHDOG_AUTODISABLE_SAVE_FLAGS)
WATCHDOG_DISABLE_ON_MCU_RESET_SAVE_FLAGS
#elif defined(WATCHDOG_DO_NOT_AUTODISABLE)
#warning "Code disabling the watchdog timer on MCU reset not implicitly inserted."
#else
WATCHDOG_DISABLE_ON_MCU_RESET
#endif

#endif
