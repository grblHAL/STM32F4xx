/*
  flexi_hal.c - driver code for STM32F4xx ARM processors on Flexi-HAL board

  Part of grblHAL

  Copyright (c) 2022 Expatria Technologies

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

#if defined(BOARD_FLEXI_HAL)

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

#if SDCARD_ENABLE
#include "../sdcard/sdcard.h"
#endif


void board_init (void)
{

#if SDCARD_ENABLE
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = 1 << SD_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SD_CS_PORT, &GPIO_InitStruct);
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    __HAL_RCC_GPIOB_CLK_ENABLE();    

    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, 1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
    volatile uint32_t dly = 1000;
    volatile uint32_t count = 100;

    while(--dly)
        __ASM volatile ("nop");  

    while(--count) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);         
        dly = 25;  // Reset dly before first delay
        while(--dly)    
            __ASM volatile ("nop");

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);         
        dly = 25;  // Reset dly before second delay
        while(--dly)
            __ASM volatile ("nop");
    }

    sdcard_getfs(); // Mounts SD card if not already mounted
#endif
}

#endif