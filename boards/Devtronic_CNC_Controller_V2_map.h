/*
  Devtronic_CNC_Controller_map.h - driver code for STM32F4xx (Blackpill) breakout board

  Part of GrblHAL

  Copyright (c) 2023 @nickshl & Terje Io

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

/* Pin Assignments(V2 have optional RS-485 support instead PWM spindle):
 *
 *                     B Direction -   - B Step
 *                                  | |
 *                                G S D +
 *                                N C I 3
 *                                D K O V
 *                              -----------
 *                          VB |           | +3V
 *            Step En/Dis  C13 |           | GND
 *          Coolant Flood  C14 |           | +5V
 *           Coolant Mist  C15 | *     - * | B9   I2C SDA
 *                         RST |      |K|  | B8   I2C SCL
 *                  Reset   A0 |       -   | B7   Feed Hold
 * Safety door/I2C strobe   A1 |           | B6   Cycle Start
 *     Spindle Enable(TX)   A2 |           | B5   Probe
 *  Spindle Direction(RX)   A3 |    / \    | B4   Encoder Index
 *                 X Step   A4 |   <MCU>   | B3   Encoder Pulse (B)
 *            X Direction   A5 |    \ /    | A15  Encoder Pulse (A)
 *                 Y Step   A6 |           | A12  USB D+
 *            Y Direction   A7 |   -   -   | A11  USB D-
 *                 Z Step   B0 |  |R| |B|  | A10  MPG/DRO RX
 *            Z Direction   B1 |   -   -   | A9   MPG/DRO TX
 *                 A Step   B2 |           | A8   Spindle PWM(EN)
 *            A Direction  B10 |           | B15  A Limit
 *                         +3V |   -----   | B14  Z Limit
 *                         GND |  |     |  | B13  Y Limit
 *                         +5V |  | USB |  | B12  X Limit
 *                              -----------
 */

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if SPINDLE_SYNC_ENABLE
#define BOARD_NAME "Devtronic CNC Controller V2 with Spindle Sync"
#else
#define BOARD_NAME "Devtronic CNC Controller V2"
#endif

#define SERIAL_PORT   1   // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT  2   // GPIOA: TX = 2, RX = 3
#define I2C_PORT      1   // GPIOB: SCL = 8, SDA = 9

// Define step pulse output pins.
#define X_STEP_PORT             GPIOA
#define X_STEP_PIN              4
#define Y_STEP_PORT             GPIOA
#define Y_STEP_PIN              6
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              0
#define STEP_OUTMODE            GPIO_BITBAND

#define X_DIRECTION_PORT        GPIOA
#define X_DIRECTION_PIN         5
#define Y_DIRECTION_PORT        GPIOA
#define Y_DIRECTION_PIN         7
#define Z_DIRECTION_PORT        GPIOB
#define Z_DIRECTION_PIN         1
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOC
#define STEPPERS_ENABLE_PIN     13
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             2
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        10
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            15
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOA
#define M4_STEP_PIN             13
#define M4_DIRECTION_PORT       GPIOA
#define M4_DIRECTION_PIN        14
#elif N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOB
#define M3_STEP_PIN             2
#define M3_DIRECTION_PORT       GPIOB
#define M3_DIRECTION_PIN        10
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            15
#else
#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          2
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          10
#define AUXINPUT1_PORT          GPIOB
#define AUXINPUT1_PIN           15
#endif

#define AUXOUTPUT2_PORT         GPIOA // Spindle enable / RS-485 TX
#define AUXOUTPUT2_PIN          2
#define AUXOUTPUT3_PORT         GPIOA // Spindle direction / RS-485 RX
#define AUXOUTPUT3_PIN          3
#define AUXOUTPUT4_PORT         GPIOA // Spindle PWM / RS-485 DIR
#define AUXOUTPUT4_PIN          8

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

#if MODBUS_ENABLE
#define MODBUS_SERIAL_PORT      1
#define MODBUS_RTU_STREAM       1
#define MODBUS_DIR_AUX          4
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOC
#define COOLANT_FLOOD_PIN       15
#define COOLANT_MIST_PORT       GPIOC
#define COOLANT_MIST_PIN        14

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOA
#define RESET_PIN               0
#define FEED_HOLD_PORT          GPIOB
#define FEED_HOLD_PIN           7
#define CYCLE_START_PORT        GPIOB
#define CYCLE_START_PIN         6
#define CONTROL_INMODE          GPIO_BITBAND

// Spindle encoder pins.
#if SPINDLE_ENCODER_ENABLE
#define RPM_COUNTER_N           2
#define RPM_TIMER_N             3
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       4
#define SPINDLE_PULSE_PORT      GPIOA
#define SPINDLE_PULSE_PIN       15
#else
#define AUXINPUT2_PORT          GPIOB
#define AUXINPUT2_PIN           4
#define AUXINPUT3_PORT          GPIOB
#define AUXINPUT3_PIN           3
#define AUXINPUT4_PORT          GPIOA
#define AUXINPUT4_PIN           15
#endif

#define AUXINPUT0_PORT          GPIOA
#define AUXINPUT0_PIN           1
#define AUXINPUT5_PORT          GPIOB
#define AUXINPUT5_PIN           5

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT5_PORT
#define PROBE_PIN               AUXINPUT5_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT0_PORT
#define I2C_STROBE_PIN          AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
#error SDcard not supported
#endif

/* EOF */
