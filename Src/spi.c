/*
  spi.c - SPI support for SD card, Trinamic & networking (WizNet) plugins

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2020-2023 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "main.h"
#include "driver.h"

#if SPI_ENABLE

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

#define DMAirq(d, p) DMAirqI(d, p)
#define DMAirqI(d, p) DMA ## d ## _Stream ## p ## _IRQn

#define DMAhandler(d, p) DMAhandlerI(d, p)
#define DMAhandlerI(d, p) DMA ## d ## _Stream ## p ## _IRQHandler

#if SPI_PORT == 11 || SPI_PORT == 12
#define SPIPORT SPIport(1)
#else
#define SPIPORT SPIport(SPI_PORT)
#endif

static SPI_HandleTypeDef spi_port = {
    .Instance = SPIPORT,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial = 10
};

#if SPI_PORT == 1 || SPI_PORT == 11 || SPI_PORT == 12

#define DMA_RX_IRQ DMAirq(2, 2)
#define DMA_TX_IRQ DMAirq(2, 3)
#define DMA_RX_IRQ_HANDLER DMAhandler(2, 2)
#define DMA_TX_IRQ_HANDLER DMAhandler(2, 3)

static DMA_HandleTypeDef spi_dma_rx = {
    .Instance = DMA2_Stream2,
    .Init.Channel = DMA_CHANNEL_3,
    .Init.Direction = DMA_PERIPH_TO_MEMORY,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

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

#elif SPI_PORT == 2

#define DMA_RX_IRQ DMAirq(1, 3)
#define DMA_TX_IRQ DMAirq(1, 4)
#define DMA_RX_IRQ_HANDLER DMAhandler(1, 3)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 4)

static DMA_HandleTypeDef spi_dma_rx = {
    .Instance = DMA1_Stream3,
    .Init.Channel = DMA_CHANNEL_0,
    .Init.Direction = DMA_PERIPH_TO_MEMORY,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

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

#elif SPI_PORT == 3

#define DMA_RX_IRQ DMAirq(1, 2)
#define DMA_TX_IRQ DMAirq(1, 7)
#define DMA_RX_IRQ_HANDLER DMAhandler(1, 2)
#define DMA_TX_IRQ_HANDLER DMAhandler(1, 7)

static DMA_HandleTypeDef spi_dma_rx = {
    .Instance = DMA1_Stream2,
    .Init.Channel = DMA_CHANNEL_0,
    .Init.Direction = DMA_PERIPH_TO_MEMORY,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.MemDataAlignment = DMA_PDATAALIGN_BYTE,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_VERY_HIGH,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

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

void spi_init (void)
{
    static bool init = false;

    /*
     * This doesn't really turn the power on, but initializes the
     * SSI port and pins needed to talk to the card.
     */

    if(!init) {

#if SPI_PORT == 1

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 5,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 6,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 7,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 11

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_5|GPIO_PIN_6,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_5;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 5,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOA,
            .pin = 6,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 12

        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin =  GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI1,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 3,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 4,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 5,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 2

        __HAL_RCC_SPI2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2,
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 14,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 15,
            .mode = { .mask = PINMODE_NONE }
        };

#elif SPI_PORT == 3

        __HAL_RCC_SPI3_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF6_SPI3
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 10,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 11,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE }
        };
#endif

        HAL_SPI_Init(&spi_port);
        __HAL_SPI_ENABLE(&spi_port);

        __HAL_LINKDMA(&spi_port, hdmarx, spi_dma_rx);
        __HAL_LINKDMA(&spi_port, hdmatx, spi_dma_tx);

        HAL_DMA_Init(&spi_dma_rx);
        HAL_DMA_Init(&spi_dma_tx);

        HAL_NVIC_SetPriority(DMA_RX_IRQ, 0, 0);
        HAL_NVIC_EnableIRQ(DMA_RX_IRQ);

        HAL_NVIC_SetPriority(DMA_TX_IRQ, 0, 0);
        HAL_NVIC_EnableIRQ(DMA_TX_IRQ);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);

        init = true;
    }
}

uint32_t spi_set_speed (uint32_t prescaler)
{
    uint32_t cur = spi_port.Instance->CR1 & SPI_BAUDRATEPRESCALER_256;

    spi_port.Instance->CR1 &= ~SPI_BAUDRATEPRESCALER_256;
    spi_port.Instance->CR1 |= prescaler;

    return cur;
}

uint8_t spi_get_byte (void)
{
	spi_port.Instance->DR = 0xFF; // Writing dummy data into Data register

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    return (uint8_t)spi_port.Instance->DR;
}

uint8_t spi_put_byte (uint8_t byte)
{
	spi_port.Instance->DR = byte;

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_TXE));
    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    __HAL_SPI_CLEAR_OVRFLAG(&spi_port);

    return (uint8_t)spi_port.Instance->DR;
}

void spi_write (uint8_t *data, uint16_t len)
{
    if(HAL_SPI_Transmit_DMA(&spi_port, data, len) == HAL_OK)
        while(spi_port.State != HAL_SPI_STATE_READY);

    __HAL_DMA_DISABLE(&spi_dma_tx);
}

void spi_read (uint8_t *data, uint16_t len)
{
    if(HAL_SPI_Receive_DMA(&spi_port, data, len) == HAL_OK)
        while(spi_port.State != HAL_SPI_STATE_READY);

    __HAL_DMA_DISABLE(&spi_dma_rx);
    __HAL_DMA_DISABLE(&spi_dma_tx);
}

void DMA_RX_IRQ_HANDLER (void)
{
  HAL_DMA_IRQHandler(&spi_dma_rx);
}

void DMA_TX_IRQ_HANDLER(void)
{
  HAL_DMA_IRQHandler(&spi_dma_tx);
}

#endif // SPI_ENABLE
