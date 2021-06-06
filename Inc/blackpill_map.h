/*
  blackpill_map.h - driver code for STM32F4xx (Blackpill) breakout board

  Part of GrblHAL

  Copyright (c) 2021 rvalotta

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

/* Default Pin Assignments:
 * A0  X Step       | B0  Step En/Dis       | C0  (N/A)
 * A1  X Direction  | B1  Spindle Enable    | C1  (N/A)
 * A2  Y Step       | B2  Spindle Direction | C2  (N/A)
 * A3  Y Direction  | B3                    | C3  (N/A)
 * A4  Z Step       | B4                    | C4  (N/A)
 * A5  Z Direction  | B5                    | C5  (N/A)
 * A6  A Step       | B6  Reset             | C6  (N/A)
 * A7  A Direction  | B7  Feed Hold         | C7  (N/A)
 * A8  Spindle PWM  | B8  Cycle Start       | C8  (N/A)
 * A9  Y2 Step      | B9  Door Safety       | C9  (N/A)
 * A10 Y2 Direction | B10 Y2 Limit          | C10 (N/A)
 * A11              | B11                   | C11 (N/A)
 * A12              | B12 X Limit           | C12 (N/A)
 * A13              | B13 Y Limit           | C13
 * A14              | B14 Z Limit           | C14 Coolant Flood
 * A15              | B15 Probe             | C15 coolant Mist
 */

#define BOARD_NAME "BlackPill"

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif

#ifdef CNC_BLACKPILL
#undef CNC_BLACKPILL
#endif

#define CNC_BLACKPILL   1
#define EEPROM_ENABLE   0 // Disabled for now for BlackPill - pin differences... // Only change if BoosterPack does not have EEPROM mounted
#define X_GANGED        0
#define X_AUTO_SQUARE   0
#define Y_GANGED        0
#define Y_AUTO_SQUARE   1
#define Z_GANGED        0
#define Z_AUTO_SQUARE   0

// Define step pulse output pins.
#define STEP_PORT           GPIOA
#define DIRECTION_PORT      GPIOA

#define X_STEP_PIN          0
#define X_STEP_BIT          (1<<X_STEP_PIN)
#define X_DIRECTION_PIN     1
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)

#if X_GANGED || X_AUTO_SQUARE
 #define X2_STEP_PIN         9
 #define X2_STEP_BIT         (1<<X2_STEP_PIN)
 #define X2_DIRECTION_PIN    10
 #define X2_DIRECTION_BIT    (1<<X2_DIRECTION_PIN)
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PIN       10 // PORT GPIOB
  #define X2_LIMIT_BIT       (1<<X2_LIMIT_PIN)
 #endif
#endif

#define Y_STEP_PIN          2
#define Y_STEP_BIT          (1<<Y_STEP_PIN)
#define Y_DIRECTION_PIN     3
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)

#if Y_GANGED || Y_AUTO_SQUARE
 #define Y2_STEP_PIN         9
 #define Y2_STEP_BIT         (1<<Y2_STEP_PIN)
 #define Y2_DIRECTION_PIN    10
 #define Y2_DIRECTION_BIT    (1<<Y2_DIRECTION_PIN)
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PIN       10 // PORT GPIOB Doubling up on Z- Axis Limit Switch as they will never be used at the same time
  #define Y2_LIMIT_BIT       (1<<Y2_LIMIT_PIN)
 #endif
#endif

#define Z_STEP_PIN          4
#define Z_STEP_BIT          (1<<Z_STEP_PIN)
#define Z_DIRECTION_PIN     5
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)

#if Z_GANGED || Z_AUTO_SQUARE
 #define Z2_STEP_PIN         9
 #define Z2_STEP_BIT         (1<<Z2_STEP_PIN)
 #define Z2_DIRECTION_PIN    10
 #define Z2_DIRECTION_BIT    (1<<Z2_DIRECTION_PIN)
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PIN       10 // PORT GPIOB
  #define Z2_LIMIT_BIT       (1<<Z2_LIMIT_PIN)
 #endif

#endif

#if N_AXIS > 3
 #define A_STEP_PIN          6
 #define A_STEP_BIT          (1<<A_STEP_PIN)
 #define A_DIRECTION_PIN     7
 #define A_DIRECTION_BIT     (1<<A_DIRECTION_PIN)

  #define STEP_MASK           (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT) // All step bits
  #define DIRECTION_MASK      (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT) // All direction bits
#else
  #define STEP_MASK           (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
  #define DIRECTION_MASK      (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
#endif

// TODO: Get AutoSquaring and Ganged Axis to work with GPIO_MAP mode
#define STEP_OUTMODE        GPIO_BITBAND
#define DIRECTION_OUTMODE   GPIO_BITBAND
//#define STEP_OUTMODE    GPIO_MAP
//#define DIRECTION_OUTMODE   GPIO_MAP


// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT   GPIOB
#define STEPPERS_DISABLE_PIN    0
#define STEPPERS_DISABLE_BIT    (1<<STEPPERS_DISABLE_PIN)
#define STEPPERS_DISABLE_MASK   STEPPERS_DISABLE_BIT

// Define homing/hard limit switch input pins.
#define LIMIT_PORT       GPIOB
#define X_LIMIT_PIN      12
#define Y_LIMIT_PIN      13
#define Z_LIMIT_PIN      14
#define X_LIMIT_BIT      (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT      (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT      (1<<Z_LIMIT_PIN)
#if N_AXIS > 3
// Not tested, might need to remap due to B15 being mapped to probe
#define A_LIMIT_PIN      15
#define A_LIMIT_BIT      (1<<A_LIMIT_PIN)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT) // All limit bits
#else
#if defined(X2_LIMIT_BIT)
#define LIMIT_MASK       (X_LIMIT_BIT|X2_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#elif defined(Y2_LIMIT_BIT)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Y2_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#elif defined(Z2_LIMIT_BIT)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|Z2_LIMIT_BIT) // All limit bits
#else
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#endif

#endif
#define LIMIT_INMODE GPIO_SHIFT12

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
//#error SDcard not supported
#endif

/* EOF */
