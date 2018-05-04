// Main starting point for STM32F4xx boards.
//
// Copyright (C) 2018 Julian Birkemose <jgb@birkeborg.dk>
// Copyright (C) 2018 Grigori Goronzy <greg@kinoho.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include "command.h" // DECL_CONSTANT
#include "stm32f4xx.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_iwdg.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_adc.h"
#include "sched.h" // sched_main

DECL_CONSTANT(MCU, "stm32f446");

/****************************************************************
 * dynamic memory pool
 ****************************************************************/

static char dynmem_pool[15 * 1024];

// Return the start of memory available for dynamic allocations
void *
dynmem_start(void)
{
    return dynmem_pool;
}

// Return the end of memory available for dynamic allocations
void *
dynmem_end(void)
{
    return &dynmem_pool[sizeof(dynmem_pool)];
}

/****************************************************************
 * misc functions
 ****************************************************************/

void
command_reset(uint32_t *args)
{
    NVIC_SystemReset();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

void systemclock_config(void)
{
    /**
     * @brief  System Clock Configuration
     *         The system Clock is configured as follow : 
     *            System Clock source            = PLL (HSE)
     *            SYSCLK(Hz)                     = 180000000
     *            HCLK(Hz)                       = 180000000
     *            AHB Prescaler                  = 1
     *            APB1 Prescaler                 = 4
     *            APB2 Prescaler                 = 2
     *            HSE Frequency(Hz)              = 8000000
     *            PLL_M                          = 8
     *            PLL_N                          = 360
     *            PLL_P                          = 2
     *            PLL_R                          = 2
     *            VDD(V)                         = 3.3
     *            Main regulator output voltage  = Scale1 mode
     *            Flash Latency(WS)              = 5
     * @param  None
     * @retval None
     */

    /* Enable HSE oscillator */
    LL_RCC_HSE_Enable();
    while(LL_RCC_HSE_IsReady() != 1);
    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_8, 360, LL_RCC_PLLP_DIV_2);
    LL_RCC_PLL_Disable();
    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1);
    /* Sysclk activation on the main PLL */
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
    /* Set APB1 & APB2 prescaler */
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_4);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
    /* Set ADC Clock source */
    LL_ADC_SetCommonClock(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_SYNC_PCLK_DIV4);

    /* Set FLASH latency */
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_5);
    LL_FLASH_EnablePrefetch();

    /* Enable PWR clock */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* Update CMSIS variable */
    SystemCoreClockUpdate(); // SystemCoreClock = 180000000;
    /* Set systick to 1ms */
    LL_Init1msTick(SystemCoreClock);
    SysTick_Config(180000000 / 1000);

    /* Activation OverDrive Mode */
    //LL_PWR_EnableOverDriveMode();
    //while(LL_PWR_IsActiveFlag_OD() != 1);

    /* Activation OverDrive Switching */
    //LL_PWR_EnableOverDriveSwitching();
    //while(LL_PWR_IsActiveFlag_ODSW() != 1);
}

void adc_config(void)
{

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
    /* ADC might be in deep sleep, needs to be woken up twice in that case */
    LL_ADC_Enable(ADC1);
    LL_mDelay(1);
    LL_ADC_Enable(ADC1);
    LL_mDelay(1);
    /*LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1));*/
    LL_ADC_SetSequencersScanMode(ADC1, LL_ADC_SEQ_SCAN_DISABLE);
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_SOFTWARE);
    LL_ADC_REG_SetSequencerLength(ADC1, LL_ADC_REG_SEQ_SCAN_DISABLE);
}

void io_config(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOE);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    /* JTAG is normally not needed, but blocks ports like PB3, PB4 */
    //LL_GPIO_AF_Remap_SWJ_NOJTAG();
    /* Likewise, we don't need PB3 for TRACESWO output */
    LL_DBGMCU_SetTracePinAssignment(LL_DBGMCU_TRACE_NONE);
}

// Main entry point
int
main(void)
{
    SystemInit();
    LL_Init1msTick(SystemCoreClock);
    systemclock_config();
    adc_config();
    io_config();
    
    sendf("Initialization done, entering Schedule Main");
    sched_main();
    return 0;
}