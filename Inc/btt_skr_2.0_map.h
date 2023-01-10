/*
  btt_skr_2.0_map.h - driver code for STM32F407 ARM processors

  Part of grblHAL

  Copyright (c) 2021 fitch22

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

#if !defined(STM32F407xx) || HSE_VALUE != 8000000
#error "This board has STM32F407 processor with a 8MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT SKR-2"
#define BOARD_URL "https://github.com/bigtreetech/SKR-2"
#define HAS_BOARD_INIT

#define I2C_PORT 1      // GPIOB, SCL_PIN = 8, SDA_PIN = 9
//#define I2C1_ALT_PINMAP // GPIOB, SCL_PIN = 6, SDA_PIN = 7

// If we want to debug, we need to use USART1
#if defined(DEBUG) && defined(USB_SERIAL_CDC)
#undef USB_SERIAL_CDC
#endif

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  2                   // X
#define Y_STEP_PORT                 GPIOD
#define Y_STEP_PIN                  5                   // Y
#define Z_STEP_PORT                 GPIOA
#define Z_STEP_PIN                  15                  // Z
#define STEP_OUTMODE                GPIO_BITBAND
//#define STEP_PINMODE                PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOE
#define X_DIRECTION_PIN             1
#define Y_DIRECTION_PORT            GPIOD
#define Y_DIRECTION_PIN             4
#define Z_DIRECTION_PORT            GPIOA
#define Z_DIRECTION_PIN             8
#define DIRECTION_OUTMODE           GPIO_BITBAND
//#define DIRECTION_PINMODE           PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOE
#define X_ENABLE_PIN                3
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                6
#define Z_ENABLE_PORT               GPIOD
#define Z_ENABLE_PIN                1
//#define STEPPERS_ENABLE_PINMODE   PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOC
#define X_LIMIT_PIN                 1                           // X- Limit
#define Y_LIMIT_PORT                GPIOC
#define Y_LIMIT_PIN                 3                           // Y- Limit
#define Z_LIMIT_PORT                GPIOC
#define Z_LIMIT_PIN                 0                           // Z- Limit
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
// The normal limit pin for E1 is PCA0, but bit 0 already has an interrupt (Z_LIMIT_PIN).
// PC15 is normally used for PWRDET but is used for M4_LIMIT_PIN instead.
// If using TMC drivers, jumper from PWRDET connector pin 3 to DIAG pin on driver.
#define M4_LIMIT_PORT               GPIOC                       // orig GPIOA
#define M4_LIMIT_PIN                15                          // orig 0
#define M4_ENABLE_PORT              GPIOD
#define M4_ENABLE_PIN               13
#endif

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOB
#define SPINDLE_ENABLE_PIN          6                           // FAN1
#define SPINDLE_DIRECTION_PORT      GPIOB
#define SPINDLE_DIRECTION_PIN       5                           // FAN2

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE       GPIOB_BASE
#define SPINDLE_PWM_PIN             0                           // EXP1 - PB0, pin 9
// Alt. spindle output, comment out definitions above and uncomment these to change:
//#define SPINDLE_PWM_PORT_BASE       GPIOE_BASE
//#define SPINDLE_PWM_PIN             5                         // SERVOS - PE5, pin 1

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOB
#define COOLANT_FLOOD_PIN           3                           // HEAT0
#define COOLANT_MIST_PORT           GPIOB
#define COOLANT_MIST_PIN            4                           // HEAT1

// Define user-control controls (cycle start, reset, feed hold) input pins.
// These are all available on EXP2 along with electrical RESET* (EXP2-8)
#define CONTROL_PORT                GPIOA
#define RESET_PIN                   4                           // Exp2-4
#define FEED_HOLD_PIN               5                           // Exp2-2
#define CYCLE_START_PIN             6                           // Exp2-1

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            GPIOA
#define SAFETY_DOOR_PIN             7                           // EXP2-6
#endif
#define CONTROL_INMODE              GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT                  GPIOE
#define PROBE_PIN                   4                           // BLTouch PE4

// Safe Power Control
#define STEPPERS_POWER_PORT         GPIOC
#define STEPPERS_POWER_PIN          13

// XXXXX
// SKR-2 has SD/MMC interface and does not work in SPI mode
#if SDCARD_ENABLE
#error "SD SDIO/MMC interface does not support SPI."
#endif

#if TRINAMIC_UART_ENABLE

#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE        2

#define MOTOR_UARTX_PORT            GPIOE
#define MOTOR_UARTX_PIN             0
#define MOTOR_UARTY_PORT            GPIOD
#define MOTOR_UARTY_PIN             3
#define MOTOR_UARTZ_PORT            GPIOD
#define MOTOR_UARTZ_PIN             0

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOC
#define MOTOR_UARTM3_PIN            6
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOD
#define MOTOR_UARTM4_PIN            12
#endif

#elif TRINAMIC_SPI_ENABLE

// The BTT SKR-2 uses software SPI
// MISO pin is also SWCLK from JTAG port, so can't debug with Trinamic SPI drivers:-(
#define TRINAMIC_MOSI_PORT          GPIOE
#define TRINAMIC_MOSI_PIN           14
#define TRINAMIC_SCK_PORT           GPIOE
#define TRINAMIC_SCK_PIN            15

// BigTreeTech used PA14 (SWCLK) as MOT_MISO.
// For debugging, change this to PA6 (on EXP2) and jumper directly to MISO pins on TMC2130s.
#ifdef DEBUG
#define TRINAMIC_MISO_PORT          GPIOE
#define TRINAMIC_MISO_PIN           7		// temporary EXP2-3 to use while debugging.  real one is PA14
#else
#define TRINAMIC_MISO_PORT          GPIOA
#define TRINAMIC_MISO_PIN           14
#endif

#define MOTOR_CSX_PORT              GPIOE
#define MOTOR_CSX_PIN               0
#define MOTOR_CSY_PORT              GPIOD
#define MOTOR_CSY_PIN               3
#define MOTOR_CSZ_PORT              GPIOD
#define MOTOR_CSZ_PIN               0

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOC
#define MOTOR_CSM3_PIN              6
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOD
#define MOTOR_CSM4_PIN              12
#endif

#endif

// EOF
