/*

  serial.h - stream interface for serial ports

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

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

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver.h"

#define UART_PORT GPIOD
#define UART_TX_PIN 8
#define UART_RX_PIN 9

const io_stream_t *serialInit (uint32_t baud_rate);
#ifdef SERIAL2_MOD
const io_stream_t *serial2Init(uint32_t baud_rate);
#endif
void serialRegisterStreams (void);

/*EOF*/
