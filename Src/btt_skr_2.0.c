/*
  btt_skr_2.c - driver code for STM32F407xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include "driver.h"

#if defined(BOARD_BTT_SKR_20)

//#include "chip.h"

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#include "trinamic/common.h"


#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static uint16_t cs_bit[N_AXIS];
static GPIO_TypeDef *cs_port[N_AXIS];

// XXXXX replace with something better...
inline static void delay (void)
{
    volatile uint32_t dly = 10;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t sw_spi_xfer (uint8_t byte)
{
    uint_fast8_t msk = 0x80, res = 0;

    DIGITAL_OUT(TMC_SCK_PORT, TMC_SCK_PIN, 0);

    do {
        DIGITAL_OUT(TMC_MOSI_PORT, TMC_MOSI_PIN, (byte & msk) != 0);
        msk >>= 1;
        delay();
        res = (res << 1) | DIGITAL_IN(TMC_MISO_PORT, TMC_MISO_PIN);
        DIGITAL_OUT(TMC_SCK_PORT, TMC_SCK_PIN, 1);
        delay();
        if(msk)
            DIGITAL_OUT(TMC_SCK_PORT, TMC_SCK_PIN, 0);
    } while (msk);

    return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_RESET);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_SET);
    delay();
    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_RESET);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_SET);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_RESET);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    HAL_GPIO_WritePin(cs_port[driver.axis], cs_bit[driver.axis], GPIO_PIN_SET);

    return status;
}

#endif

#if TRINAMIC_ENABLE == 2209
#endif

void board_init (void)
{
#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set all output pins: push-pull, no pull-up, slow
    GPIO_InitStruct.Pin = TMC_MOSI_BIT;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TMC_SCK_BIT;
    HAL_GPIO_Init(TMC_SCK_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_SCK_PORT, TMC_SCK_BIT, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = TMC_CSX_BIT;
    HAL_GPIO_Init(TMC_CSX_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_CSX_PORT, TMC_CSX_BIT, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = TMC_CSY_BIT;
    HAL_GPIO_Init(TMC_CSY_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_CSY_PORT, TMC_CSY_BIT, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = TMC_CSZ_BIT;
    HAL_GPIO_Init(TMC_CSZ_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_CSZ_PORT, TMC_CSZ_BIT, GPIO_PIN_SET);

#ifdef A_AXIS
    GPIO_InitStruct.Pin = TMC_CSA_BIT;
    HAL_GPIO_Init(TMC_CSA_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_CSA_PORT, TMC_CSA_BIT, GPIO_PIN_SET);
#endif

#ifdef B_AXIS
    GPIO_InitStruct.Pin = TMC_CSB_BIT;
    HAL_GPIO_Init(TMC_CSB_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(TMC_CSA_PORT, TMC_CSA_BIT, GPIO_PIN_SET);
#endif

    // Set the input pin: input with pull-up
    GPIO_InitStruct.Pin = TMC_MISO_BIT;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TMC_MISO_PORT, &GPIO_InitStruct);

    // Save pin and port information
    cs_bit[X_AXIS] = TMC_CSX_BIT;
    cs_port[X_AXIS] = TMC_CSX_PORT;
    cs_bit[Y_AXIS] = TMC_CSY_BIT;
    cs_port[Y_AXIS] = TMC_CSY_PORT;
    cs_bit[Z_AXIS] = TMC_CSZ_BIT;
    cs_port[Z_AXIS] = TMC_CSZ_PORT;
#ifdef A_AXIS
    cs_bit[A_AXIS] = TMC_CSA_BIT;
    cs_port[A_AXIS] = TMC_CSA_PORT;
#endif
#ifdef B_AXIS
    cs_bit[B_AXIS] = TMC_CSB_BIT;
    cs_port[B_AXIS] = TMC_CSB_PORT;
#endif

#endif  // TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#if TRINAMIC_ENABLE == 2209
#endif  // TRINAMIC_ENABLE == 2209

    // The BTT SKR-2 has "Power Protection".
    // This GPIO pin must be driven to get any power to the steppers at all.
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = SAFE_PWR_BIT;
    HAL_GPIO_Init(SAFE_PWR_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(SAFE_PWR_PORT, SAFE_PWR_BIT, GPIO_PIN_SET);
    HAL_Delay(100);	// need a little time for the power to come up and driver to come out of reset
}

#endif  // BOARD_BTT_SKR_20

