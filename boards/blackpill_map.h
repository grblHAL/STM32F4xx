/*
  blackpill_map.h - driver code for STM32F4xx (Blackpill) breakout board

  Part of grblHAL

  Copyright (c) 2021 rvalotta

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

/* Custom OpenPnP / Router Pin Assignments:
 *
 *                               -----------
 *                            VB |           | +3V
 *                           C13 |           | GND
 *            Steppers Enable C14 | *     - * | B9   Aux In 0 (Unused)
 *                               |           | B8   Cycle Start
 *                           RST |     |K|   | B7   Feed Hold
 *                        X Dir  A1 |       -   | B6   Reset/EStop
 *                        Y Dir  A0 |           | B5   Coolant Mist
 *                       Z Step  A2 |           | B4   
 *                        Z Dir  A3 |    / \    | B3   
 *                       Aux Out 0 A4 |   <MCU>   | A15  
 *                       Coolant Flood A5 |    \ /    | A12  USB D+
 *                     Aux Out 1 A6 |           | A11  USB D-
 *                        X Step  A7 |   -   -   | A10  Spindle Direction (PB10)
 *                        X Step  B0 |  |R| |B|  | A9   
 *                    Spindle On  B1 |   -   -   | A8   Spindle PWM / Svet_PWM
 *                                B2 |           | B15  Aux In 1
 *                           +3V |     ----- | B14  Z Limit
 *                           GND |    |     | | B13  Y Limit
 *                           +5V |    | USB | | B12  X Limit
 *                               -----------
 */

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "Custom BlackPill OpenPnP"

// Define step pulse output pins.
// X_STEP: PB0, Y_STEP: PA1, Z_STEP: PA2
#define STEP_OUTMODE            GPIO_MAP
#define X_STEP_PORT             GPIOB
#define X_STEP_PIN              0
#define Y_STEP_PORT             GPIOA
#define Y_STEP_PIN              1
#define Z_STEP_PORT             GPIOA
#define Z_STEP_PIN              2

// Define step direction output pins.
// X_DIR: PA7, Y_DIR: PA0, Z_DIR: PA3
#define DIRECTION_OUTMODE       GPIO_MAP
#define X_DIRECTION_PORT        GPIOA
#define X_DIRECTION_PIN         7
#define Y_DIRECTION_PORT        GPIOA
#define Y_DIRECTION_PIN         0
#define Z_DIRECTION_PORT        GPIOA
#define Z_DIRECTION_PIN         3

// Define stepper driver enable/disable output pin.
// Steppers Enable: PC14
#define STEPPERS_ENABLE_PORT    GPIOC
#define STEPPERS_ENABLE_PIN     14
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
// X_LIMIT: PB12, Y_LIMIT: PB13, Z_LIMIT: PB14
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            15
#endif
#endif

// Define Auxiliary Outputs
#define AUXOUTPUT0_PORT         GPIOA // Aux Out 0 (P0)
#define AUXOUTPUT0_PIN          4
#define AUXOUTPUT1_PORT         GPIOA // Aux Out 1 (Fan 0)
#define AUXOUTPUT1_PIN          6
#define AUXOUTPUT2_PORT         GPIOA // Spindle PWM (Svet_PWM)
#define AUXOUTPUT2_PIN          8
#define AUXOUTPUT3_PORT         GPIOB // Spindle Direction
#define AUXOUTPUT3_PIN          10
#define AUXOUTPUT4_PORT         GPIOB // Spindle On (KL_1)
#define AUXOUTPUT4_PIN          1
#define AUXOUTPUT5_PORT         GPIOA // Coolant Flood
#define AUXOUTPUT5_PIN          5
#define AUXOUTPUT6_PORT         GPIOB // Coolant Mist
#define AUXOUTPUT6_PIN          5

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT2_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT3_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT3_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT5_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT5_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT6_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT6_PIN
#endif

// Auxiliary Inputs
#define AUXINPUT0_PORT          GPIOB // Aux In 0 (Unused - PB9)
#define AUXINPUT0_PIN           9
#if !N_AUTO_SQUARED
#define AUXINPUT1_PORT          GPIOB // Aux In 1 (PB15)
#define AUXINPUT1_PIN           15
#endif
#define AUXINPUT2_PORT          GPIOB // Reset / Emergency Stop (PB6)
#define AUXINPUT2_PIN           6
#define AUXINPUT3_PORT          GPIOB // Feed Hold (PB7)
#define AUXINPUT3_PIN           7
#define AUXINPUT4_PORT          GPIOB // Cycle Start (PB8)
#define AUXINPUT4_PIN           8

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

#if PROBE_ENABLE && defined(AUXINPUT1_PIN)
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

/* EOF */
