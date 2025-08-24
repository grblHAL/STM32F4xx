/*
  stm32f401_uni_map.h - driver code for STM32F401CCU6 (Blackpill) breakout board

  Part of grblHAL

  Copyright (c) 2021 rvalotta
  Copyright (c) 2023 Am0k-GIT

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

/* Pin Assignments:
 * A0  (EXP 1-07) Spindle En.   | B0  Z Probe                     | C0  (N/A)
 * A1  (EXP 1-05) Spindle Dir.  | B1  E Limit                     | C1  (N/A)
 * A2  (EXP 1-03)               | B2  X Limit                     | C2  (N/A)
 * A3  (EXP 1-04) RESET         | B3  Spindle PWM MOSFET          | C3  (N/A)
 * A4  (EXP 2-07) SS            | B4  Y Direction                 | C4  (N/A)
 * A5  (EXP 2-09) SCK           | B5  Y Step                      | C5  (N/A)
 * A6  (EXP 2-10) MISO          | B6  (EXP 1- 8) SCL/STOP         | C6  (N/A)
 * A7  (EXP 2-05) MOSI          | B7  (EXP 1- 6) SCK/START        | C7  (N/A)
 * A8  Step En/Dis              | B8  X Direction                 | C8  (N/A)
 * A9  FLOOD MOSFET             | B9  X Step                      | C9  (N/A)
 * A10 MOSFET                   | B10 Y Limit                     | C10 (N/A)
 * A11 USB D-                   | B11 (N/A)                       | C11 (N/A)
 * A12 USB D+                   | B12 E Direction                 | C12 (N/A)
 * A13 (EXP 1-10) DOOR          | B13 E Step                      | C13 (EXP 1-9)
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

#define AUXOUTPUT0_PORT          GPIOA  // Spindle enable
#define AUXOUTPUT0_PIN           0
#define AUXOUTPUT1_PORT          GPIOB  // Spindle PWM
#define AUXOUTPUT1_PIN           3
#define AUXOUTPUT2_PORT          GPIOA  // Spindle direction
#define AUXOUTPUT2_PIN           1
#define AUXOUTPUT3_PORT          GPIOA  // Coolant flood
#define AUXOUTPUT3_PIN           9
#define AUXOUTPUT4_PORT          GPIOA  // Coolant mist
#define AUXOUTPUT4_PIN           15

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT0_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT1_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT1_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT2_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT2_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          GPIOA  // Reset/EStop
#define AUXINPUT0_PIN           3
#define AUXINPUT1_PORT          GPIOB  // Feed hold
#define AUXINPUT1_PIN           6
#define AUXINPUT2_PORT          GPIOB  // Cycle start
#define AUXINPUT2_PIN           7
#define AUXINPUT3_PORT          GPIOB  // Probe
#define AUXINPUT3_PIN           0
#define AUXINPUT4_PORT          GPIOA  // Door
#define AUXINPUT4_PIN           13

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT0_PORT
#define RESET_PIN               AUXINPUT0_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT1_PORT
#define FEED_HOLD_PIN           AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT2_PORT
#define CYCLE_START_PIN         AUXINPUT2_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT3_PORT
#define PROBE_PIN               AUXINPUT3_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT4_PORT
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT4_PORT
#define MOTOR_FAULT_PIN         AUXINPUT4_PIN
#endif

#if SDCARD_ENABLE
  #define SDCARD_SDIO           0
  #define SPI_PORT              1                                  // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
  #define SD_CS_PORT            GPIOA
  #define SD_CS_PIN             4
#endif

/* EOF */
