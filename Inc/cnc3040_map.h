/*
  cnc3040_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

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

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_AXIS == 5
#define BOARD_NAME "CNC 3040 5-axis"
#elif N_AXIS == 4
#define BOARD_NAME "CNC 3040 4-axis"
#else
#define BOARD_NAME "CNC 3040"
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              2
#define Z_STEP_PIN              4
#define STEP_OUTMODE            GPIO_MAP
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PIN         3
#define Z_DIRECTION_PIN         5
#define DIRECTION_OUTMODE       GPIO_MAP
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     9
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             2
#define Y_LIMIT_PIN             10
#define Z_LIMIT_PIN             12
#define LIMIT_INMODE            GPIO_MAP

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT            STEP_PORT
#define M4_STEP_PIN             9
#define M4_DIRECTION_PORT       DIRECTION_PORT
#define M4_DIRECTION_PIN        10
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      1
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   0

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       3
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOB
#define RESET_PIN               5
#define FEED_HOLD_PIN           14
#define CYCLE_START_PIN         15
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         8
#endif
#define CONTROL_INMODE          GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT              GPIOB
#define PROBE_PIN               13
