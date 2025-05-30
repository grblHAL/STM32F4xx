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

/* Pin Assignments:
 *
 *                               -----------
 *                           VB |           | +3V
 *                          C13 |           | GND
 *           Coolant Flood  C14 |           | +5V
 *            Coolant Mist  C15 | *     - * | B9   Safety door / Aux In 0
 *                          RST |      |K|  | B8   Cycle Start
 *                  X Step   A0 |       -   | B7   Feed Hold
 *             X Direction   A1 |           | B6   Reset/EStop
 *                  Y Step   A2 |           | B5
 *             Y Direction   A3 |    / \    | B4   Coolant Mist
 *                  Z Step   A4 |   <MCU>   | B3   Spindle Index
 *             Z Direction   A5 |    \ /    | A15  Spindle Pulse
 *    Aux In 0/1 / M3 Step   A6 |           | A12  USB D+
 * Aux In 1 / M3 Direction   A7 |   -   -   | A11  USB D-
 *          Steppers enable  B0 |  |R| |B|  | A10  M4 Direction
 *          Spindle Enable   B1 |   -   -   | A9   M4 Step
 *       Spindle Direction   B2 |           | A8   Spindle PWM
 *                M4 Limit  B10 |           | B15  Probe / M3 Limit
 *                          +3V |   -----   | B14  Z Limit
 *                          GND |  |     |  | B13  Y Limit
 *                          +5V |  | USB |  | B12  X Limit
 *                               -----------
 */

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "BlackPill"

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              2
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_MAP

#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PIN         3
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     0
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
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

#if N_ABC_MOTORS == 0
#define AUXOUTPUT0_PORT         GPIOA
#define AUXOUTPUT0_PIN          7
#define AUXOUTPUT1_PORT         GPIOA
#define AUXOUTPUT1_PIN          6
#endif
#define AUXOUTPUT2_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT2_PIN          8
#define AUXOUTPUT3_PORT         GPIOB // Spindle direction
#define AUXOUTPUT3_PIN          2
#define AUXOUTPUT4_PORT         GPIOB // Spindle enable
#define AUXOUTPUT4_PIN          1
#define AUXOUTPUT5_PORT         GPIOC // Coolant flood
#define AUXOUTPUT5_PIN          15
#define AUXOUTPUT6_PORT         GPIOC // Coolant mist
#define AUXOUTPUT6_PIN          14

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

// Spindle encoder pins.
#if SPINDLE_ENCODER_ENABLE

#define RPM_COUNTER_N           2
#define RPM_TIMER_N             3
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       3
#define SPINDLE_PULSE_PORT      GPIOA
#define SPINDLE_PULSE_PIN       15

#endif

#define AUXINPUT0_PORT          GPIOB // Safety door
#define AUXINPUT0_PIN           9
#if !N_AUTO_SQUARED
#define AUXINPUT1_PORT          GPIOB // Probe
#define AUXINPUT1_PIN           15
#endif
#define AUXINPUT2_PORT          GPIOB // Reset/EStop
#define AUXINPUT2_PIN           6
#define AUXINPUT3_PORT          GPIOB // Feed hold
#define AUXINPUT3_PIN           7
#define AUXINPUT4_PORT          GPIOB // Cycle start
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

// NOT SUPPORTED
#if SDCARD_ENABLE
//#error SDcard not supported
#endif

/* EOF */
