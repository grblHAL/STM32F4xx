/*
  st_morpho_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

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

#if TRINAMIC_ENABLE == 2130
#include "trinamic/tmc2130.h"
#endif

#if TRINAMIC_ENABLE == 2209
#include "trinamic/tmc2209.h"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Nucleo-64 CNC Breakout"

#undef I2C_ENABLE
#undef EEPROM_ENABLE
//#undef EEPROM_IS_FRAM

#define I2C_ENABLE      1
#define I2C_PORT        1
#define EEPROM_ENABLE   1
//#define EEPROM_IS_FRAM  1
//#undef SPINDLE_SYNC_ENABLE
//#define SPINDLE_SYNC_ENABLE 1

#define IS_NUCLEO_BOB
#define HAS_IOPORTS
#define HAS_BOARD_INIT

#if SDCARD_ENABLE || TRINAMIC_SPI_ENABLE
#define SPI_PORT                1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOC
#define X_STEP_PIN              0
#define Y_STEP_PIN              5
#define Z_STEP_PIN              9
#if N_GANGED
#define STEP_OUTMODE            GPIO_MAP
#else
#define STEP_OUTMODE            GPIO_MAP
#endif
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         0
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PIN         11
#if N_GANGED
#define DIRECTION_OUTMODE       GPIO_BITBAND
#else
#define DIRECTION_OUTMODE       GPIO_MAP
#endif
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pins.

#define X_ENABLE_PORT           GPIOA
#define X_ENABLE_PIN            1
#define Y_ENABLE_PORT           GPIOB
#define Y_ENABLE_PIN            12
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            1
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOC
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             12
#define Z_LIMIT_PIN             10
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOC
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        12
#define M3_LIMIT_PORT           GPIOC
#define M3_LIMIT_PIN            11
#define M3_ENABLE_PORT          GPIOB
#define M3_ENABLE_PIN           6
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      3
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       10
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOC
#define RESET_PIN               2
#define FEED_HOLD_PIN           3
#define CYCLE_START_PIN         4
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         1
#endif
#define CONTROL_INMODE          GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT              GPIOC
#define PROBE_PIN               7

// Spindle encoder pins.
#if SPINDLE_SYNC_ENABLE

#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       14
#define SPINDLE_PULSE_PORT      GPIOD
#define SPINDLE_PULSE_PIN       2

#endif

#if MPG_MODE == 1
#ifndef SPI_PORT
#define MPG_MODE_PORT           GPIOC
#define MPG_MODE_PIN            8
#elif N_ABC_MOTORS == 0
#define MPG_MODE_PORT           GPIOC
#define MPG_MODE_PIN            11
#endif
#endif

// Auxiliary I/O
#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           14
#define AUXINPUT1_PORT          GPIOA
#define AUXINPUT1_PIN           15
#define AUXINPUT2_PORT          GPIOB
#define AUXINPUT2_PIN           13

#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          15
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          2

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIOB
#define I2C_STROBE_PIN          0
#else
#define AUXINPUT3_PORT          GPIOB
#define AUXINPUT3_PIN           0
#endif

#ifdef SPI_PORT

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               8
#endif

#if TRINAMIC_SPI_ENABLE
#define MOTOR_CS_PORT           GPIOB
#define MOTOR_CS_PIN            7
#endif

#else

#define AUXOUTPUT2_PORT         GPIOA
#define AUXOUTPUT2_PIN          6
#define AUXOUTPUT3_PORT         GPIOA
#define AUXOUTPUT3_PIN          5
#define AUXOUTPUT4_PORT         GPIOA
#define AUXOUTPUT4_PIN          7
#define AUXOUTPUT5_PORT         GPIOB
#define AUXOUTPUT5_PIN          7
#if MPG_MODE == 0
#define AUXOUTPUT6_PORT         GPIOC
#define AUXOUTPUT6_PIN          8
#endif
#endif

// EOF
