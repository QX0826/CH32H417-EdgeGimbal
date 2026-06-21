#ifndef __SYSTEM_CH32H417_H
#define __SYSTEM_CH32H417_H

#include "ch32h417.h"

extern uint32_t HCLKClock;
extern uint32_t SystemClock;
extern uint32_t SystemCoreClock;
extern void SystemAndCoreClockUpdate(void);
extern void SystemInit(void);

#endif
