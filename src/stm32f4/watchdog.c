// Initialization of STM32 watchdog timer.
//
// Copyright (C) 2018  Julian Birkemose <jgb@birkenet.dk>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include "sched.h"  //DECL_CONSTANT
#include "command.h"    //shutdown
#include "stm32f4xx.h"
#include "stm32f4xx_ll_iwdg.h"


/****************************************************************
 * watchdog handler
 ****************************************************************/

void
watchdog_reset(void)
{
    LL_IWDG_ReloadCounter(IWDG);
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    LL_IWDG_EnableWriteAccess(IWDG);
    /* IWDG timer is 40 KHz, configure to trigger every seconds */
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_16);
    LL_IWDG_SetReloadCounter(IWDG, 2500);
    LL_IWDG_Enable(IWDG);

}
DECL_INIT(watchdog_init);