// GPIO functions on STM32F1
//
// Copyright (C) 2018 Grigori Goronzy <greg@kinoho.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "stm32f4xx.h"
#include "stm32f4xx_ll_spi.h"
#include "gpio.h"
#include "sched.h" // sched_shutdown
#include "board/irq.h"
#include "board/io.h"


/****************************************************************
 * Serial Peripheral Interface (SPI) pins
 ****************************************************************/

void spi_set_mode(SPI_TypeDef *spi, uint8_t mode)
{
    switch (mode) {
    case 0:
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);
        break;
    case 1:
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_LOW);
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
        break;
    case 2:
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_1EDGE);
        break;
    case 3:
        LL_SPI_SetClockPolarity(spi, LL_SPI_POLARITY_HIGH);
        LL_SPI_SetClockPhase(spi, LL_SPI_PHASE_2EDGE);
        break;
    default:
        shutdown("Invalid SPI mode");
    }
}

void spi_set_baudrate(SPI_TypeDef *spi, uint32_t rate)
{
    const uint32_t pclk = __LL_RCC_CALC_PCLK1_FREQ(SystemCoreClock, LL_RCC_GetAPB1Prescaler());
    const uint32_t prescaler = pclk / rate;

    uint32_t setting = LL_SPI_BAUDRATEPRESCALER_DIV256;
    if (prescaler <= 2)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV2;
    else if (prescaler <= 4)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV4;
    else if (prescaler <= 8)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV8;
    else if (prescaler <= 16)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV16;
    else if (prescaler <= 32)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV32;
    else if (prescaler <= 64)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV64;
    else if (prescaler <= 128)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV128;
    else if (prescaler <= 256)
        setting = LL_SPI_BAUDRATEPRESCALER_DIV256;

    LL_SPI_SetBaudRatePrescaler(spi, setting);
}

void spi_init_pins(void)
{
    /* SPI2 */
    /* PC2 - MISO */
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOC, LL_GPIO_PIN_2, LL_GPIO_OUTPUT_PUSHPULL);

    /* PC3 - MOSI */
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);

    /* PB10 - SCK */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_15, LL_GPIO_OUTPUT_PUSHPULL);
}

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    struct spi_config config;
    config.config = *SPI2;

    if (bus > 0 || !rate)
        shutdown("Invalid spi_setup parameters");

    spi_init_pins();
    spi_set_mode(&config.config, mode);
    spi_set_baudrate(&config.config, rate);

    return config;
}

void
spi_prepare(struct spi_config config)
{
    *SPI2 = config.config;
    LL_SPI_Enable(SPI2);
}

void
spi_transfer(struct spi_config config, uint8_t receive_data,
             uint8_t len, uint8_t *data)
{
    while (len--) {
        LL_SPI_TransmitData8(SPI2, *data);
        while (!LL_SPI_IsActiveFlag_TXE(SPI2));
        if (receive_data) {
            while (!LL_SPI_IsActiveFlag_RXNE(SPI2));
            *data = LL_SPI_ReceiveData8(SPI2);
        }
        data++;
    }

    while (LL_SPI_IsActiveFlag_BSY(SPI2));
    LL_SPI_Disable(SPI2);
}