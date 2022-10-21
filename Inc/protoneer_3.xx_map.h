/*
  protoneer_3.xx_map.h - driver code for STM32F411 ARM processor on a Nucleo-F411RE board

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

#if N_AUTO_SQUARED || N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#define BOARD_NAME "Protoneer v3"
#define I2C_PORT 1
#define IS_NUCLEO_BOB
//#define PROTONEER_SPINDLE_PWM // Uncomment to use PA5 (D13) as spindle PWM output instead of spindle direction

// Define step pulse output pins.
#define X_STEP_PORT             GPIOA // D2
#define X_STEP_PIN              10
#define Y_STEP_PORT             GPIOB // D3
#define Y_STEP_PIN              3
#define Z_STEP_PORT             GPIOB // D4
#define Z_STEP_PIN              5
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOB // D5
#define X_DIRECTION_PIN         4
#define Y_DIRECTION_PORT        GPIOB // D6
#define Y_DIRECTION_PIN         10
#define Z_DIRECTION_PORT        GPIOA // D7
#define Z_DIRECTION_PIN         8
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOA // D8
#define STEPPERS_ENABLE_PIN     9
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOC // D9
#define X_LIMIT_PIN             7
#define Y_LIMIT_PORT            GPIOB // D10
#define Y_LIMIT_PIN             6
#define Z_LIMIT_PORT            GPIOA // D11
#define Z_LIMIT_PIN             7
#define LIMIT_INMODE            GPIO_BITBAND
#define Z_LIMIT_POLL

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        5
#endif

// Define spindle enable and spindle direction output pins.
#ifndef M3_AVAILABLE
#define SPINDLE_ENABLE_PORT     GPIOA // D12
#define SPINDLE_ENABLE_PIN      6
#ifdef PROTONEER_SPINDLE_PWM
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE // D13
#define SPINDLE_PWM_PIN         5
#else
#define SPINDLE_DIRECTION_PORT  GPIOA // D13
#define SPINDLE_DIRECTION_PIN   5
#endif
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB // A3
#define COOLANT_FLOOD_PIN       0

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOA // A0
#define RESET_PIN               0
#define FEED_HOLD_PIN           1 // A1
#define CYCLE_START_PIN         4 // A2
#define CONTROL_INMODE          GPIO_MAP

/**/
