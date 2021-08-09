/*
  skr_2_map.h - driver code for STM32F407 ARM processors

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
#error "BTT SKR-2 supports 5 motors max."
#endif

#if TRINAMIC_ENABLE == 2130
#include "trinamic\tmc2130.h"
#endif

#if TRINAMIC_ENABLE == 5160
#include "trinamic\tmc5160.h"
#endif

#if TRINAMIC_ENABLE == 2209
#include "trinamic\tmc2209.h"
#endif

#ifndef STM32F407xx
#error "This board has STM32F407 processor, select a corresponding build!"
#endif

#define BOARD_NAME "BTT SKR-2"
#define HAS_BOARD_INIT

#define I2C_PORT 1      // GPIOB, SCL_PIN = 8, SDA_PIN = 9
//#define I2C1_ALT_PINMAP // GPIOB, SCL_PIN = 6, SDA_PIN = 7

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  2                   // X
#define Y_STEP_PORT                 GPIOD
#define Y_STEP_PIN                  5                   // Y
#define Z_STEP_PORT                 GPIOA
#define Z_STEP_PIN                  15                  // Z
#define X_STEP_BIT                  (1<<X_STEP_PIN)
#define Y_STEP_BIT                  (1<<Y_STEP_PIN)
#define Z_STEP_BIT                  (1<<Z_STEP_PIN)
#define STEP_OUTMODE                GPIO_BITBAND
//#define STEP_PINMODE                PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOE
#define X_DIRECTION_PIN             1
#define Y_DIRECTION_PORT            GPIOD
#define Y_DIRECTION_PIN             4
#define Z_DIRECTION_PORT            GPIOA
#define Z_DIRECTION_PIN             8
#define X_DIRECTION_BIT             (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT             (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT             (1<<Z_DIRECTION_PIN)
#define DIRECTION_OUTMODE           GPIO_BITBAND
//#define DIRECTION_PINMODE           PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOE
#define X_ENABLE_PIN                3
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                6
#define Z_ENABLE_PORT               GPIOD
#define Z_ENABLE_PIN                1
#define X_ENABLE_BIT                (1<<X_ENABLE_PIN)
#define Y_ENABLE_BIT                (1<<Y_ENABLE_PIN)
#define Z_ENABLE_BIT                (1<<Z_ENABLE_PIN)
//#define STEPPERS_ENABLE_PINMODE   PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOC
#define X_LIMIT_PIN                 1                           // X- Limit
#define Y_LIMIT_PORT                GPIOC
#define Y_LIMIT_PIN                 3                           // Y- Limit
#define Z_LIMIT_PORT                GPIOC
#define Z_LIMIT_PIN                 0                           // Z- Limit
#define X_LIMIT_BIT                 (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT                 (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT                 (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE                GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                // E0
#define M3_STEP_PORT                GPIOD
#define M3_STEP_PIN                 15
#define M3_DIRECTION_PORT           GPIOD
#define M3_DIRECTION_PIN            14
#define M3_LIMIT_PORT               GPIOC
#define M3_LIMIT_PIN                2
#define M3_ENABLE_PORT              GPIOC
#define M3_ENABLE_PIN               7
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE                // E1
#define M4_STEP_PORT                GPIOD
#define M4_STEP_PIN                 11
#define M4_DIRECTION_PORT           GPIOD
#define M4_DIRECTION_PIN            10
#define M4_LIMIT_PORT               GPIOA
#define M4_LIMIT_PIN                0
#define M4_ENABLE_PORT              GPIOD
#define M4_ENABLE_PIN               13
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOB
#define SPINDLE_ENABLE_PIN          6                           // FAN1
#define SPINDLE_ENABLE_BIT          (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT      GPIOB
#define SPINDLE_DIRECTION_PIN       5                           // FAN2
#define SPINDLE_DIRECTION_BIT       (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIOB
#define SPINDLE_PWM_PIN             0                           // EXP1 - PB0, pin 9
#define SPINDLE_PWM_BIT             (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOB
#define COOLANT_FLOOD_PIN           3                           // HEAT0
#define COOLANT_FLOOD_BIT           (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT           GPIOB
#define COOLANT_MIST_PIN            4                           // HEAT1
#define COOLANT_MIST_BIT            (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT                  GPIOC
#define RESET_PIN                   2                           // E0 Limit
#define RESET_BIT                   (1<<RESET_PIN)

#define FEED_HOLD_PORT              GPIOE
#define FEED_HOLD_PIN               6                           // RGB
#define FEED_HOLD_BIT               (1<<FEED_HOLD_PIN)

#define CYCLE_START_PORT            GPIOE
#define CYCLE_START_PIN             5                           // Servos
#define CYCLE_START_BIT             (1<<CYCLE_START_PIN)

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PORT            GPIOB
#define SAFETY_DOOR_PIN             1                           // EXP1 - PB1, pin 8
#define SAFETY_DOOR_BIT             (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK                (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK                (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT)
#endif
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT                  GPIOE
#define PROBE_PIN                   4                       // BLTouch PE4
#define PROBE_BIT                   (1<<PROBE_PIN)

// XXXXX
// SKR-2 has SD/MMC interface and does not work in SPI mode
#if SDCARD_ENABLE
#error "SD SDIO/MMC interface does not support SPI."
#endif

// The BTT SKR-2 uses software SPI
// MISO pin is also SWCLK from JTAG port, so can't debug with Trinamic SPI drivers:-(
#define TMC_MOSI_PORT               GPIOE
#define TMC_MOSI_PIN                14
#define TMC_MOSI_BIT                (1 << TMC_MOSI_PIN)
#define TMC_SCK_PORT                GPIOE
#define TMC_SCK_PIN                 15
#define TMC_SCK_BIT                 (1 << TMC_SCK_PIN)
#define TMC_MISO_PORT               GPIOA

// BigTreeTech used PA14 (SWCLK) as MOT_MISO.
// For debugging, change this to PA6 (on EXP2) and jumper directly to MISO pins on TMC2130s.
#ifdef DEBUG
#define TMC_MISO_PIN                6		// temporary EXP2-1 to use while debugging.  real one is PA14
#else
#define TMC_MISO_PIN                14
#endif
#define TMC_MISO_BIT                (1 << TMC_MISO_PIN)

// The CS pins are also the UART pins for 1 wire serial Trinamic drivers (2208, 2209)
#define TMC_CSX_PORT                GPIOE
#define TMC_CSX_PIN                 0
#define TMC_CSX_BIT                 (1 << TMC_CSX_PIN)
#define TMC_CSY_PORT                GPIOD
#define TMC_CSY_PIN                 3
#define TMC_CSY_BIT                 (1 << TMC_CSY_PIN)
#define TMC_CSZ_PORT                GPIOD
#define TMC_CSZ_PIN                 0
#define TMC_CSZ_BIT                 (1 << TMC_CSZ_PIN)
#define TMC_CSA_PORT                GPIOC
#define TMC_CSA_PIN                 6
#define TMC_CSA_BIT                 (1 << TMC_CSA_PIN)
#define TMC_CSB_PORT                GPIOD
#define TMC_CSB_PIN                 12
#define TMC_CSB_BIT                 (1 << TMC_CSB_PIN)

// Safe Power Control
#define SAFE_PWR_PORT               GPIOC
#define SAFE_PWR_PIN                13
#define SAFE_PWR_BIT                (1 << SAFE_PWR_PIN)

// EOF
