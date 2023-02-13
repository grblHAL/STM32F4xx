/*
  stm32f401_uni_map.h - driver code for STM32F401CCU6 (Blackpill) breakout board

  Part of grblHAL

  Copyright (c) 2021 rvalotta
  Copyright (c) 2023 Am0k-GIT

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

/* Pin Assignments:
 * A0  Spindle En. (EXP 1-07)   | B0  Z Probe                     | C0  (N/A)
 * A1  Spindle Dir. (EXP 1-05)  | B1  E Limit                     | C1  (N/A)
 * A2  (EXP 1-03)               | B2  X Limit                     | C2  (N/A)
 * A3  RESET (EXP 1-04)         | B3  Spindle PWM MOSFET          | C3  (N/A)
 * A4  (EXP 2-07) SS            | B4  Y Direction                 | C4  (N/A)
 * A5  (EXP 2-09) SCK           | B5  Y Step                      | C5  (N/A)
 * A6  AUX0 (EXP 2-10) MISO     | B6  STOP (EXP 1- 8) SCL         | C6  (N/A)
 * A7  AUX1 (EXP 2-05) MOSI     | B7  START (EXP 1- 6) SCK        | C7  (N/A)
 * A8  Step En/Dis              | B8  X Direction                 | C8  (N/A)
 * A9  FLOOD MOSFET             | B9  X Step                      | C9  (N/A)
 * A10 MOSFET                   | B10 Y Limit                     | C10 (N/A)
 * A11 USB D-                   | B11 (N/A)                       | C11 (N/A)
 * A12 USB D+                   | B12 E Direction                 | C12 (N/A)
 * A13 DOOR (EXP 1-10)          | B13 E Step                      | C13 (EXP 1-9)
 * A14 Z Limit                  | B14 Z Direction                 | C14 (EXP 2-6)
 * A15 MIST MOSFET              | B15 Z Step                      | C15 (EXP 2-8)
 */

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "STM32F401_UNI"
#define BOARD_URL "https://github.com/Am0k-GIT/STM32F401CCU6_UNI"

// Define step pulse output pins.
#define STEP_PORT               GPIOB
#define X_STEP_PIN              9
#define Y_STEP_PIN              5
#define Z_STEP_PIN              15
#define STEP_OUTMODE            GPIO_MAP

#define DIRECTION_PORT          GPIOB
#define X_DIRECTION_PIN         8
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PIN         14
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOA
#define STEPPERS_ENABLE_PIN     8
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOB
#define X_LIMIT_PIN             2                            // X- Limit
#define Y_LIMIT_PORT            GPIOB
#define Y_LIMIT_PIN             10                           // Y- Limit
#define Z_LIMIT_PORT            GPIOA
#define Z_LIMIT_PIN             14                           // Z- Limit
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.

#if N_ABC_MOTORS > 0
#define M3_AVAILABLE            // E0
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             13
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        12
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            1
#define M3_ENABLE_PORT          STEPPERS_ENABLE_PORT
#define M3_ENABLE_PIN           STEPPERS_ENABLE_PIN
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOA
#define SPINDLE_ENABLE_PIN      0
#define SPINDLE_DIRECTION_PORT  GPIOA
#define SPINDLE_DIRECTION_PIN   1

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOB_BASE
#define SPINDLE_PWM_PIN         3

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOA
#define COOLANT_FLOOD_PIN       9
#define COOLANT_MIST_PORT       GPIOA
#define COOLANT_MIST_PIN        15

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOA
#define RESET_PIN               3
#define FEED_HOLD_PORT          GPIOB
#define FEED_HOLD_PIN           6
#define CYCLE_START_PORT        GPIOB
#define CYCLE_START_PIN         7
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        GPIOA
#define SAFETY_DOOR_PIN         13
#endif
#define CONTROL_INMODE          GPIO_BITBAND

// Define probe switch input pin.
#if !N_AUTO_SQUARED
#define PROBE_PORT              GPIOB
#define PROBE_PIN               0
#endif


// Spindle encoder pins.
#if SPINDLE_SYNC_ENABLE

#define RPM_COUNTER_N           2
#define RPM_TIMER_N             3
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       3
#define SPINDLE_PULSE_PORT      GPIOA
#define SPINDLE_PULSE_PIN       15

#endif

#if N_ABC_MOTORS == 0

#define HAS_IOPORTS

#endif

#define AUXINPUT0_PORT          GPIOA
#define AUXINPUT0_PIN           6
#define AUXOUTPUT1_PORT         GPIOA
#define AUXOUTPUT1_PIN          7

// NOT SUPPORTED
#if KEYPAD_ENABLE
#error Keypad not supported
#endif
// NOT SUPPORTED
#if SDCARD_ENABLE
//#error SDcard not supported
#endif

/* EOF */
