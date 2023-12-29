/*
  st_morpho_dac_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Nucleo-64 CNC Breakout (DAC)"
#define BOARD_URL "https://github.com/terjeio/CNC_Breakout_Nucleo64"

#ifndef WEB_BUILD
#undef I2C_ENABLE
#undef EEPROM_ENABLE
#undef EEPROM_IS_FRAM
#define I2C_ENABLE      1
#define EEPROM_ENABLE   1
#define EEPROM_IS_FRAM  1
#endif

#define SERIAL_PORT     2   // GPIOA: TX = 2, RX = 3
#define SERIAL1_PORT    1   // GPIOA: TX = 9, RX = 10
#if I2C_ENABLE
#define I2C_PORT        1   // PB8 = SCK, PB9 = SDA
#endif

#define HAS_BOARD_INIT
#define I2C_PORT 1
#define IS_NUCLEO_BOB

// Define step pulse output pins.
#define STEP_PORT               GPIOC
#define X_STEP_PIN              0
#define Y_STEP_PIN              5
#define Z_STEP_PIN              9
#define STEP_OUTMODE GPIO_MAP
//#define STEP_PINMODE        PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         0
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PIN         11
#define DIRECTION_OUTMODE       GPIO_MAP
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

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8
#else
#define AUXOUTPUT4_PORT         GPIOA
#define AUXOUTPUT4_PIN          8
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   5
#else
#define AUXOUTPUT5_PORT         GPIOB
#define AUXOUTPUT5_PIN          5
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      3
#else
#define AUXOUTPUT6_PORT         GPIOB
#define AUXOUTPUT6_PIN          3
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       10
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4

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

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOC
#define RESET_PIN               2
#define FEED_HOLD_PIN           3
#define CYCLE_START_PIN         4
#define CONTROL_INMODE GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT              GPIOC
#define PROBE_PIN               7

// Auxiliary I/O
#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           13
#define AUXINPUT1_PORT          GPIOB
#define AUXINPUT1_PIN           14
#define AUXINPUT2_PORT          GPIOC
#define AUXINPUT2_PIN           1

#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          15
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          2

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         GPIOB
#define I2C_STROBE_PIN          0
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               8
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT2_PORT
#define SAFETY_DOOR_PIN         AUXINPUT2_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT1_PORT
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

// EOF
