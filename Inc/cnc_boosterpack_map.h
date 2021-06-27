/*
  cnc_boosterpack_map.h - driver code for STM32F4xx processor (on Blackpill board)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "CNC BoosterPack"

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif

#define CNC_BOOSTERPACK 1
#define EEPROM_ENABLE   0 // Disabled for now for BlackPill - pin differences... // Only change if BoosterPack does not have EEPROM mounted

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              1
#define Z_STEP_PIN              2
#define X_STEP_BIT              (1<<X_STEP_PIN)
#define Y_STEP_BIT              (1<<Y_STEP_PIN)
#define Z_STEP_BIT              (1<<Z_STEP_PIN)
#define STEP_OUTMODE            GPIO_MAP
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         4
#define Y_DIRECTION_PIN         5
#define Z_DIRECTION_PIN         6
#define X_DIRECTION_BIT         (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT         (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT         (1<<Z_DIRECTION_PIN)
#define DIRECTION_OUTMODE       GPIO_MAP
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT   GPIOA
#define STEPPERS_ENABLE_PIN    15
#define STEPPERS_ENABLE_BIT    (1<<STEPPERS_ENABLE_PIN)
#define STEPPERS_ENABLE_MASK   STEPPERS_ENABLE_BIT
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define X_LIMIT_BIT             (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT             (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT             (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             3
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            15
#endif
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      1
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   0
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
#define RESET_BIT               (1<<RESET_PIN)
#define FEED_HOLD_PIN           7
#define FEED_HOLD_BIT           (1<<FEED_HOLD_PIN)
#define CYCLE_START_PIN         8
#define CYCLE_START_BIT         (1<<CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN         9
#define SAFETY_DOOR_BIT         (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK            (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK            (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT)
#endif
#define CONTROL_INMODE          GPIO_SHIFT6

// Define probe switch input pin.
#define PROBE_PORT              GPIOA
#define PROBE_PIN               7
#define PROBE_BIT               (1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_PORT             GPIOB
#define KEYPAD_STROBE_PIN       15
#define KEYPAD_STROBE_BIT       (1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               3
#define SD_CS_BIT               (1<<SD_CS_PIN)
#endif

/* EOF */
