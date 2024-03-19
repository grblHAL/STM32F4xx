/*
  uno_map.h - driver code for STM32F411 ARM processor on a Nucleo-F411RE board

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

  GrblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  GrblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with GrblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#define BOARD_NAME "Generic Uno"
#define BOARD_URL "https://www.makerfabs.com/arduino-cnc-shield-v3.html"

#define SERIAL_PORT     2   // GPIOA: TX = 2, RX = 3
#define I2C_PORT        1   // GPIOB: SCL = 8, SDA = 9
#define IS_NUCLEO_BOB

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
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOC // D9
#define X_LIMIT_PIN             7
#define Y_LIMIT_PORT            GPIOB // D10
#define Y_LIMIT_PIN             6
#ifdef VARIABLE_SPINDLE
  #define Z_LIMIT_PORT          GPIOA // D12
  #define Z_LIMIT_PIN           6
#else
  #define Z_LIMIT_PORT          GPIOA // D11
  #define Z_LIMIT_PIN           7
#endif
#define LIMIT_INMODE            GPIO_BITBAND

// Comment out if Z limit pin is not assigned to an interrupt enabled pin on a different port.

#define Z_LIMIT_POLL

#define AUXOUTPUT0_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT0_PIN          7
#define AUXOUTPUT1_PORT         GPIOA // Spindle direction
#define AUXOUTPUT1_PIN          5
#if DRIVER_SPINDLE_PWM_ENABLE
#define AUXOUTPUT2_PORT         GPIOB // Spindle enable, on morpho header
#define AUXOUTPUT2_PIN          7
#else
#define AUXOUTPUT2_PORT         GPIOA // Spindle enable
#define AUXOUTPUT2_PIN          6
#endif

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB // A3
#define COOLANT_FLOOD_PIN       0
#define COOLANT_MIST_PORT       GPIOC // A4
#define COOLANT_MIST_PIN        1

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOA
#define RESET_PIN               0 // A0
#define FEED_HOLD_PIN           1 // A1
#define CYCLE_START_PIN         4 // A2
#define CONTROL_INMODE          GPIO_MAP

#define AUXINPUT0_PORT          GPIOC // A5
#define AUXINPUT0_PIN           0

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT0_PORT
#define PROBE_PIN               AUXINPUT0_PIN
#endif

/**/
