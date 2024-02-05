/*

  serial.h - stream interface for serial ports

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io

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

/*

1  - GPIOA: TX =  9, RX = 10
11 - GPIOB: TX =  6, RX =  7
2  - GPIOA: TX =  2, RX =  3 - Nucleo-64 Virtual COM port
21 - GPIOD: TX =  5, RX =  6
3  - GPIOB: TX = 10, RX = 11
31 - GPIOC: TX = 10, RX = 11
32 - GPIOD: TX =  8, RX =  9 - Nucleo-144 Virtual COM port
33 - GPIOC: TX = 10, RX =  5
6  - GPIOC: TX =  6, RX =  7

*/

#pragma once

void serialRegisterStreams (void);

/*EOF*/
