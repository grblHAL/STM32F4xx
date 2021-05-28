/*
  skr_pro_v1_1_map.h - driver code for STM32F407 ARM processors

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if SPI_PORT == 1
#error Port SPI1 is not supported!
#endif

#define BOARD_NAME "BTT SKR PRO v1.1"

#define I2C1_ALT_PINMAP // GPIOB, SCL_PIN = 6, SDA_PIN = 7

// Define step pulse output pins.
#define X_STEP_PORT     GPIOE
#define X_STEP_PIN      9                   // X
#define Y_STEP_PORT     GPIOE
#define Y_STEP_PIN      11                  // Y
#define Z_STEP_PORT     GPIOE
#define Z_STEP_PIN      13                  // Z
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#if N_AXIS > 3
#define A_STEP_PORT     GPIOE
#define A_STEP_PIN      14                  // E0
#define A_STEP_BIT      (1<<A_STEP_PIN)
#endif
#if N_AXIS > 4
#define B_STEP_PORT     GPIOD
#define B_STEP_PIN      15                  // E1
#define B_STEP_BIT      (1<<B_STEP_PIN)
#endif
#if N_AXIS > 5
#define C_STEP_PORT     GPIOD
#define C_STEP_PIN      13                  // E2
#define C_STEP_BIT      (1<<C_STEP_PIN)
#endif
#define STEP_OUTMODE GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT    GPIOF
#define X_DIRECTION_PIN     1
#define Y_DIRECTION_PORT    GPIOE
#define Y_DIRECTION_PIN     8
#define Z_DIRECTION_PORT    GPIOC
#define Z_DIRECTION_PIN     2
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)
#if N_AXIS > 3
#define A_DIRECTION_PORT    GPIOA
#define A_DIRECTION_PIN     0
#define A_DIRECTION_BIT     (1<<A_DIRECTION_PIN)
#endif
#if N_AXIS > 4
#define B_DIRECTION_PORT    GPIOE
#define B_DIRECTION_PIN     7
#define B_DIRECTION_BIT     (1<<B_DIRECTION_PIN)
#endif
#if N_AXIS > 5
#define C_DIRECTION_PORT    GPIOG
#define C_DIRECTION_PIN     9
#define C_DIRECTION_BIT     (1<<C_DIRECTION_PIN)
#endif
#define DIRECTION_OUTMODE   GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define X_STEPPERS_DISABLE_PORT   GPIOF
#define X_STEPPERS_DISABLE_PIN    2
#define Y_STEPPERS_DISABLE_PORT   GPIOD
#define Y_STEPPERS_DISABLE_PIN    7
#define Z_STEPPERS_DISABLE_PORT   GPIOC
#define Z_STEPPERS_DISABLE_PIN    0
#define X_STEPPERS_DISABLE_BIT    (1<<X_STEPPERS_DISABLE_PIN)
#define Y_STEPPERS_DISABLE_BIT    (1<<Y_STEPPERS_DISABLE_PIN)
#define Z_STEPPERS_DISABLE_BIT    (1<<Z_STEPPERS_DISABLE_PIN)
#if N_AXIS > 3
#define A_STEPPERS_DISABLE_PORT   GPIOC
#define A_STEPPERS_DISABLE_PIN    3
#define A_STEPPERS_DISABLE_BIT    (1<<A_STEPPERS_DISABLE_PIN)
#endif
#if N_AXIS > 4
#define B_STEPPERS_DISABLE_PORT   GPIOA
#define B_STEPPERS_DISABLE_PIN    3
#define B_STEPPERS_DISABLE_BIT    (1<<B_STEPPERS_DISABLE_PIN)
#endif
#if N_AXIS > 5
#define C_STEPPERS_DISABLE_PORT   GPIOF
#define C_STEPPERS_DISABLE_PIN    0
#define C_STEPPERS_DISABLE_BIT    (1<<C_STEPPERS_DISABLE_PIN)
#endif


// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT     GPIOB
#define X_LIMIT_PIN      10                                      // X- Limit
#define Y_LIMIT_PORT     GPIOE
#define Y_LIMIT_PIN      12                                      // Y- Limit
#define Z_LIMIT_PORT     GPIOG
#define Z_LIMIT_PIN      8                                       // Z- Limit
#define X_LIMIT_BIT      (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT      (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT      (1<<Z_LIMIT_PIN)
#if N_AXIS > 3
#define A_LIMIT_PORT     GPIOE
#define A_LIMIT_PIN      15                                      // E0- Limit
#define A_LIMIT_BIT      (1<<A_LIMIT_PIN)
#endif
#if N_AXIS > 4
#define B_LIMIT_PORT     GPIOE
#define B_LIMIT_PIN      10                                      // E1- Limit
#define B_LIMIT_BIT      (1<<B_LIMIT_PIN)
#endif
#if N_AXIS > 5
#define C_LIMIT_PORT     GPIOG
#define C_LIMIT_PIN      5                                       // E2- Limit
#define C_LIMIT_BIT      (1<<C_LIMIT_PIN)
#endif
#if N_AXIS == 6
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT)
#elif N_AXIS == 5
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT)
#elif N_AXIS == 4
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT)
#else
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT)
#endif
#define LIMIT_INMODE     GPIO_BITBAND

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOE
#define SPINDLE_ENABLE_PIN          5                            // FAN1
#define SPINDLE_ENABLE_BIT          (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT      GPIOE
#define SPINDLE_DIRECTION_PIN       6                            // FAN2
#define SPINDLE_DIRECTION_BIT       (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIOA
#define SPINDLE_PWM_PIN             8                            // EXT1 - PA8
#define SPINDLE_PWM_BIT             (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOB
#define COOLANT_FLOOD_PIN           1                            // HEAT0
#define COOLANT_FLOOD_BIT           (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT           GPIOD
#define COOLANT_MIST_PIN            14                           // HEAT1
#define COOLANT_MIST_BIT            (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT          GPIOG
#define RESET_PIN           4                            // E0 Limit
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_PORT      GPIOD
#define FEED_HOLD_PIN       11                           // E1 Limit
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_PORT    GPIOG
#define CYCLE_START_PIN     2                            // E2 Limit
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PORT    GPIOG
#define SAFETY_DOOR_PIN     6                            // EXP1 PG4
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define MASK                (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT)
#endif
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT                  GPIOA
#define PROBE_PIN                   1                            // BLTouch PA1
#define PROBE_BIT                   (1<<PROBE_PIN)

#if SDCARD_ENABLE
#define SD_CS_PORT  GPIOB
#define SD_CS_PIN   12
#define SD_CS_BIT   (1<<SD_CS_PIN)
#define SPI_PORT    2 // GPIOB, SCK_PIN = 13, MISO_PIN = 14, MOSI_PIN = 15 (EXP2 Header)
#endif
