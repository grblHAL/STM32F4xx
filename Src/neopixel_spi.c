/*
  neopixel_spi.c - SPI support for Neopixels

  TODO: use I2S interface for more precise timing?

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2024 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
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

neopixel_cfg_t neopixel = { .intensity = 255 };
static settings_changed_ptr settings_changed;

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb.num_devices != settings->rgb_strip0_length) {

        if(settings->rgb_strip0_length == 0)
            settings->rgb_strip0_length = hal.rgb.num_devices;
        else
            hal.rgb.num_devices = settings->rgb_strip0_length;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb.num_devices) {
            neopixel.num_bytes = hal.rgb.num_devices * 9 + 24;
            if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb.num_devices;
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

void neopixels_write (void)
{
    if(neopixel.leds) {

        while(spi_port.State == HAL_SPI_STATE_BUSY_TX);

        HAL_SPI_Transmit_DMA(&spi_port, neopixel.leds, neopixel.num_bytes);
    }
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_3bpp_pack(&neopixel.leds[device * 9], color, mask, neopixel.intensity);

        if(neopixel.num_leds == 1)
            neopixels_write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_3bpp_unpack(& neopixel.leds[device * 9], prev);
                neopixel_out(device, color);
            } while(device);

            neopixels_write();
        }
    }

    return prev;
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
        hal.rgb.set_intensity = neopixels_set_intensity;
#if NEOPIXELS_NUM > 1
        hal.rgb.write = neopixels_write;
#endif
        hal.rgb.num_devices = NEOPIXELS_NUM;
        hal.rgb.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        init = true;
    }
}

void DMA_TX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_tx);
}

#endif // NEOPIXEL_SPI
