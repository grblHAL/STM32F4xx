/*

  mks_robin_nano_v3.0.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#include "driver.h"

#ifdef BOARD_MKS_ROBIN_NANO_30

void board_init (void)
{
#if USB_SERIAL_CDC
    /* PA14 must be set low to enable USB D+ */
    GPIO_InitTypeDef GPIO_Init = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pin = GPIO_PIN_14,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(GPIOA, &GPIO_Init);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);
#endif
}

#endif
