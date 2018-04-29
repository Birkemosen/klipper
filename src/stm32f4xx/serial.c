// STM32F4 serial port
//
// Copyright (C) 2018 Julian Birkemose <jgb@birkeborg.dk>
// Copyright (C) 2018 Grigori Goronzy <greg@kinoho.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h>
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/serial_irq.h" // serial_rx_byte
#include "sched.h" // DECL_INIT
#include "stm32f4xx.h" // UART
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

void
serial_init(void)
{
    const uint32_t pclk = __LL_RCC_CALC_PCLK2_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler());

    LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART2);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
    LL_USART_SetBaudRate(USART2, pclk, LL_USART_OVERSAMPLING_8, CONFIG_SERIAL_BAUD);
    LL_USART_SetDataWidth(USART2, LL_USART_DATAWIDTH_8B);
    LL_USART_SetParity(USART2, LL_USART_PARITY_NONE);
    LL_USART_SetStopBitsLength(USART2, LL_USART_STOPBITS_1);
    LL_USART_SetHWFlowCtrl(USART2, LL_USART_HWCONTROL_NONE);
    LL_USART_SetTransferDirection(USART2, LL_USART_DIRECTION_TX_RX);
    LL_USART_EnableIT_RXNE(USART2);
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 1);
    LL_USART_Enable(USART2);

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    //LL_GPIO_AF_DisableRemap_USART2();
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);
}
DECL_INIT(serial_init);

void __visible
USART1_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(USART2) || LL_USART_IsActiveFlag_ORE(USART2))
        serial_rx_byte(LL_USART_ReceiveData8(USART2));
    if (LL_USART_IsActiveFlag_TXE(USART2)) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            LL_USART_DisableIT_TXE(USART2);
        else
            LL_USART_TransmitData8(USART2, data);
    }
}

void
serial_enable_tx_irq(void)
{
    LL_USART_EnableIT_TXE(USART2);
}