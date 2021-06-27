/*
  st_morpho_dac_map.h - driver code for STM32F4xx ARM processors

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

#define BOARD_NAME "Nucleo-64 CNC Breakout (DAC)"
#define HAS_IOPORTS
#define HAS_BOARD_INIT
#define I2C_PORT 1
#define IS_NUCLEO_BOB
#define SPINDLE_SYNC_ENABLE

// Define step pulse output pins.
#define STEP_PORT               GPIOC
#define X_STEP_PIN              0
#define Y_STEP_PIN              5
#define Z_STEP_PIN              9
#define X_STEP_BIT              (1<<X_STEP_PIN)
#define Y_STEP_BIT              (1<<Y_STEP_PIN)
#define Z_STEP_BIT              (1<<Z_STEP_PIN)
#define STEP_OUTMODE GPIO_MAP
//#define STEP_PINMODE        PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         0
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PIN         11
#define X_DIRECTION_BIT         (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT         (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT         (1<<Z_DIRECTION_PIN)
#define DIRECTION_OUTMODE       GPIO_MAP
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pins.
#define X_ENABLE_PORT           GPIOA
#define X_ENABLE_PIN            1
#define Y_ENABLE_PORT            GPIOB
#define Y_ENABLE_PIN            12
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            1
#define X_ENABLE_BIT            (1<<X_ENABLE_PIN)
#define Y_ENABLE_BIT            (1<<Y_ENABLE_PIN)
#define Z_ENABLE_BIT            (1<<Z_ENABLE_PIN)
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOC
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             12
#define Z_LIMIT_PIN             10
#define X_LIMIT_BIT             (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT             (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT             (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE            GPIO_BITBAND

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIOB
#define SPINDLE_ENABLE_PIN      3
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  GPIOB
#define SPINDLE_DIRECTION_PIN   5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT        GPIOA
#define SPINDLE_PWM_PIN         8
#define SPINDLE_PWM_BIT         (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       10
#define COOLANT_FLOOD_BIT       (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4
#define COOLANT_MIST_BIT        (1<<COOLANT_MIST_PIN)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOC
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        12
#define M3_LIMIT_PORT           GPIOC
#define M3_LIMIT_PIN            11
#define M3_ENABLE_PORT          GPIOA
#define M3_ENABLE_PIN           6
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOC
#define RESET_PIN               2
#define RESET_BIT               (1<<RESET_PIN)
#define FEED_HOLD_PIN           3
#define FEED_HOLD_BIT           (1<<FEED_HOLD_PIN)
#define CYCLE_START_PIN         4
#define CYCLE_START_BIT         (1<<CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN         1
#define SAFETY_DOOR_BIT         (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK            (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK            (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT)
#endif
#define CONTROL_INMODE GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT              GPIOC
#define PROBE_PIN               7
#define PROBE_BIT               (1<<PROBE_PIN)

// Spindle encoder pins.

#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       14
#define SPINDLE_INDEX_BIT       (1<<SPINDLE_INDEX_PIN)

#define SPINDLE_PULSE_PORT      GPIOD
#define SPINDLE_PULSE_PIN       2
#define SPINDLE_PULSE_BIT       (1<<SPINDLE_PULSE_PIN)

// Auxiliary I/O
#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           13
#define AUXINPUT1_PORT          GPIOB
#define AUXINPUT1_PIN           14
#define AUXINPUT_MASK           (1<<AUXINPUT0_PIN|1<<AUXINPUT1_PIN)

#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          15
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          2

#if KEYPAD_ENABLE
#define KEYPAD_PORT             GPIOB
#define KEYPAD_STROBE_PIN       0
#define KEYPAD_STROBE_BIT       (1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               8
#define SD_CS_BIT               (1<<SD_CS_PIN)
#endif

// EOF
