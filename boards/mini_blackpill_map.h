/*
  mini_blackpill_map.h - driver code for minimal STM32F4xx (Blackpill) breakout board 

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

/* Pin Assignments:
 *
 *                                    -----------
 *                                VB |           | +3V
 *                               C13 |           | GND
 *                 Coolant Mist  C14 |           | +5V
 *                Coolant Flood  C15 | *     - * | B9   Safety door / Aux In 0
 *                               RST |      |K|  | B8   Cycle Start
 *                       X Step   A0 |       -   | B7   Feed Hold
 *                  X Direction   A1 |           | B6   Reset/EStop
 *                       Y Step   A2 |           | B5
 *                  Y Direction   A3 |    / \    | B4
 *                       Z Step   A4 |   <MCU>   | B3   Spindle Index
 *                  Z Direction   A5 |    \ /    | A15  Spindle Pulse
 * Aux Out In 0/Out 1 / M3 Step   A6 |           | A12  USB D+
 *     Aux Out 0 / M3 Direction   A7 |   -   -   | A11  USB D-
 *               Steppers enable  B0 |  |R| |B|  | A10  UART RX
 *               Spindle Enable   B1 |   -   -   | A9   UART TX
 *            Spindle Direction   B2 |           | A8   Spindle PWM
 *                               B10 |           | B15  Probe
 *                               +3V |   -----   | B14  Z Limit
 *                               GND |  |     |  | B13  Y Limit
 *                               +5V |  | USB |  | B12  X Limit
 *                                   -----------
 */

#if N_AUTO_SQUARED || N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Minimal BlackPill grblHAL"
#define BOARD_URL "https://github.com/avizienis/Minimal-Black-Pill--STM32F4xx-BOB-for-grblHAL"

#define SERIAL_PORT             1 // GPIOA: TX = 9, RX = 10

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              2
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_MAP

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PIN         3
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     0

#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        7
#else
#define AUXOUTPUT1_PORT         GPIOA
#define AUXOUTPUT1_PIN          6
#define AUXOUTPUT0_PORT         GPIOA
#define AUXOUTPUT0_PIN          7
#endif

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define LIMIT_INMODE            GPIO_SHIFT12

#define AUXOUTPUT2_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT2_PIN          8
#define AUXOUTPUT3_PORT         GPIOB // Spindle direction
#define AUXOUTPUT3_PIN          2
#define AUXOUTPUT4_PORT         GPIOB // Spindle enable
#define AUXOUTPUT4_PIN          1

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        AUXOUTPUT2_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOC
#define COOLANT_FLOOD_PIN       15
#define COOLANT_MIST_PORT       GPIOC
#define COOLANT_MIST_PIN        14

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOB
#define RESET_PIN               6
#define FEED_HOLD_PIN           7
#define CYCLE_START_PIN         8
#define CONTROL_INMODE          GPIO_SHIFT6

#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           9
#define AUXINPUT1_PORT          GPIOB
#define AUXINPUT1_PIN           15

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

// NOT SUPPORTED
#if SDCARD_ENABLE
#error SDcard not supported
#endif

/* EOF */
