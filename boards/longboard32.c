/*
  longboard32.c - driver code for STM32F4xx ARM processors on Flexi-HAL board

  Part of grblHAL

  Copyright (c) 2022 Expatria Technologies

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

#if defined(BOARD_LONGBOARD32)

extern void tmc_spi_init (void);
extern void neopixel_init (void);

void board_init (void)
{
    tmc_spi_init();
    neopixel_init();
}

#endif //BOARD_LONGBOARD32
