/*
  neopixel_spi.c - SPI support for Neopixels

  TODO: use I2S interface for more precise timing?

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2024 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "driver.h"

#ifdef NEOPIXEL_SPI

#ifndef NEOPIXELS_NUM
#define NEOPIXELS_NUM 1
#endif

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

#define DMAirq(d, p) DMAirqI(d, p)
#define DMAirqI(d, p) DMA ## d ## _Stream ## p ## _IRQn

#define DMAhandler(d, p) DMAhandlerI(d, p)
#define DMAhandlerI(d, p) DMA ## d ## _Stream ## p ## _IRQHandler

#if NEOPIXEL_SPI == 11 || NEOPIXEL_SPI == 12
#define SPIPORT SPIport(1)
#else
#define SPIPORT SPIport(NEOPIXEL_SPI)
#endif

static SPI_HandleTypeDef spi_port = {
    .Instance = SPIPORT,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial = 10
};

#if NEOPIXEL_SPI == 1 || NEOPIXEL_SPI == 11 || NEOPIXEL_SPI == 12

#define DMA_TX_IRQ DMAirq(2, 3)
#define DMA_TX_IRQ_HANDLER DMAhandler(2, 3)

static DMA_HandleTypeDef spi_dma_tx = {
    .Instance = DMA2_Stream3,
    .Init.Channel = DMA_CHANNEL_3,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

#elif NEOPIXEL_SPI == 2

#define DMA_TX_IRQ DMAirq(1, 4)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 4)

static DMA_HandleTypeDef spi_dma_tx = {
    .Instance = DMA1_Stream4,
    .Init.Channel = DMA_CHANNEL_0,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

#elif NEOPIXEL_SPI == 3

#define DMA_TX_IRQ DMAirq(1, 7)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 7)

static DMA_HandleTypeDef spi_dma_tx = {
    .Instance = DMA1_Stream7,
    .Init.Channel = DMA_CHANNEL_0,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

#endif

static uint8_t leds[NEOPIXELS_NUM * 9 + 15] = {0};

void neopixels_write (void)
{
    while(spi_port.State != HAL_SPI_STATE_READY);

    HAL_SPI_Transmit_DMA(&spi_port, leds, (uint16_t)sizeof(leds));
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    uint32_t R = 0, G = 0, B = 0;
    uint8_t *led = &leds[device * 9], bitmask = 0b10000000;

    do {
        R <<= 3;
        R |= color.R & bitmask ? 0b110 : 0b100;
        G <<= 3;
        G |= color.G & bitmask ? 0b110 : 0b100;
        B <<= 3;
        B |= color.B & bitmask ? 0b110 : 0b100;
    } while(bitmask >>= 1);

    if(mask.G) {
        *led++ = (uint8_t)(G >> 16);
        *led++ = (uint8_t)(G >> 8);
        *led++ = (uint8_t)G;
    } else
        led += 3;

    if(mask.R) {
        *led++ = (uint8_t)(R >> 16);
        *led++ = (uint8_t)(R >> 8);
        *led++ = (uint8_t)R;
    } else
        led += 3;

    if(mask.B) {
        *led++ = (uint8_t)(B >> 16);
        *led++ = (uint8_t)(B >> 8);
        *led   = (uint8_t)B;
    }

#if NEOPIXELS_NUM == 1
    neopixels_write();
#endif
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

void neopixel_init (void)
{
    static bool init = false;

    if(!init) {

#if NEOPIXEL_SPI == 1

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_7,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        static const periph_pin_t sdi = {
            .function = Output_LED_Adressable,
            .group = PinGroup_LED,
            .port = GPIOA,
            .pin = 7,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 11

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 12

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin =  GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 2

        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_15,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 15,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

#elif NEOPIXEL_SPI == 3

        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_12,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF6_SPI3
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };
#endif

        HAL_SPI_Init(&spi_port);
        __HAL_SPI_ENABLE(&spi_port);

        __HAL_LINKDMA(&spi_port, hdmatx, spi_dma_tx);

        HAL_DMA_Init(&spi_dma_tx);

        HAL_NVIC_SetPriority(DMA_TX_IRQ, 0, 0);
        HAL_NVIC_EnableIRQ(DMA_TX_IRQ);

        hal.periph_port.register_pin(&sdi);

        hal.rgb.out = neopixel_out;
        hal.rgb.out_masked = neopixel_out_masked;
#if NEOPIXELS_NUM > 1
        hal.rgb.write = neopixels_write;
#endif
        hal.rgb.num_devices = NEOPIXELS_NUM;
        hal.rgb.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

        init = true;
    }
}

void DMA_TX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_tx);
}

#endif // NEOPIXEL_SPI
