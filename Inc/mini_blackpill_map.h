/*
  mini_blackpill_map.h - driver code for minimal STM32F4xx (Blackpill) breakout board 

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#define BOARD_NAME "Minimal BlackPill grblHAL"

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif

#ifdef CNC_BLACKPILL
#undef CNC_BLACKPILL
#endif

#define CNC_BLACKPILL           1
#define EEPROM_ENABLE           0 // Disabled for now for BlackPill - pin differences... // Only change if BoosterPack does not have EEPROM mounted

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              2
#define Z_STEP_PIN              4
#define X_STEP_BIT              (1<<X_STEP_PIN)
#define Y_STEP_BIT              (1<<Y_STEP_PIN)
#define Z_STEP_BIT              (1<<Z_STEP_PIN)
#define STEP_OUTMODE            GPIO_MAP

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         1
#define Y_DIRECTION_PIN         3
#define Z_DIRECTION_PIN         5
#define X_DIRECTION_BIT         (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT         (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT         (1<<Z_DIRECTION_PIN)
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOB
#define STEPPERS_ENABLE_PIN     0
#define STEPPERS_ENABLE_BIT     (1<<STEPPERS_ENABLE_PIN)

#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOA
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        7
#endif

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define X_LIMIT_BIT             (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT             (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT             (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE            GPIO_SHIFT12

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      1
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   2
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT        GPIOA
#define SPINDLE_PWM_PIN         8
#define SPINDLE_PWM_BIT         (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOC
#define COOLANT_FLOOD_PIN       15
#define COOLANT_FLOOD_BIT       (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT       GPIOC
#define COOLANT_MIST_PIN        14
#define COOLANT_MIST_BIT        (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOB
#define RESET_PIN               6
#define FEED_HOLD_PIN           7
#define CYCLE_START_PIN         8
#define SAFETY_DOOR_PIN         9
#define CONTROL_INMODE          GPIO_SHIFT6
#define RESET_BIT               (1<<RESET_PIN)
#define FEED_HOLD_BIT           (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT         (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT         (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK            (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)

// Define probe switch input pin.
#define PROBE_PORT              GPIOB
#define PROBE_PIN               15
#define PROBE_BIT               (1<<PROBE_PIN)

// NOT SUPPORTED
#if KEYPAD_ENABLE
#error Keypad not supported
#endif
// NOT SUPPORTED
#if SDCARD_ENABLE
#error SDcard not supported
#endif

/* EOF */
