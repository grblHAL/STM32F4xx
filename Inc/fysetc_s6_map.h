/*
  fysetc_s6_map.h - driver code for STM32F407 ARM processors

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

#if N_ABC_MOTORS > 3
#error "Axis configuration is not supported!"
#endif

#if !defined(STM32F446xx) || HSE_VALUE != 12000000
#error "This board has STM32F446 processor with a 12MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "Fysetc S6 V2.0"

#undef I2C_ENABLE
#undef EEPROM_ENABLE

#define I2C_ENABLE 1
#define I2C_PORT 1 // PB8 / PB9
// #define I2C1_ALT_PINMAP // GPIOB, SCL_PIN = 6, SDA_PIN = 7
#define EEPROM_ENABLE 1 // 2K single byte addressing

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOE
#define X_STEP_PIN                  11
#define Y_STEP_PORT                 GPIOD
#define Y_STEP_PIN                  8
#define Z_STEP_PORT                 GPIOD
#define Z_STEP_PIN                  14
#define STEP_OUTMODE                GPIO_BITBAND
//#define STEP_PINMODE                PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOE
#define X_DIRECTION_PIN             10
#define Y_DIRECTION_PORT            GPIOB
#define Y_DIRECTION_PIN             12
#define Z_DIRECTION_PORT            GPIOD
#define Z_DIRECTION_PIN             13
#define DIRECTION_OUTMODE           GPIO_BITBAND
//#define DIRECTION_PINMODE           PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOE
#define X_ENABLE_PIN                9
#define Y_ENABLE_PORT               GPIOD
#define Y_ENABLE_PIN                9
#define Z_ENABLE_PORT               GPIOD
#define Z_ENABLE_PIN                15
//#define STEPPERS_ENABLE_PINMODE   PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOB
#define X_LIMIT_PIN                 14                          // X- Limit
#define Y_LIMIT_PORT                GPIOB
#define Y_LIMIT_PIN                 13                          // Y- Limit
#define Z_LIMIT_PORT                GPIOA
#define Z_LIMIT_PIN                 0                           // Z- Limit
#define LIMIT_INMODE                GPIO_BITBAND

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOB
#define SPINDLE_ENABLE_PIN          1                           // FAN1
#define SPINDLE_DIRECTION_PORT      GPIOB
#define SPINDLE_DIRECTION_PIN       2                           // FAN2

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT_BASE       GPIOB_BASE
#define SPINDLE_PWM_PIN             0                           // FAN0

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOB
#define COOLANT_FLOOD_PIN           1                           // HEAT0
#define COOLANT_MIST_PORT           GPIOD
#define COOLANT_MIST_PIN            14                          // HEAT1

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT                  GPIOA
#define RESET_PIN                   1                           // X+ Limit
#define FEED_HOLD_PORT              GPIOA
#define FEED_HOLD_PIN               2                           // Y+ Limit
#define CYCLE_START_PORT            GPIOA
#define CYCLE_START_PIN             3                           // Z+ Limit

#if SAFETY_DOOR_ENABLE
//#define SAFETY_DOOR_PORT            GPIOG
//#define SAFETY_DOOR_PIN             6                           // EXP1 PG4
#endif
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PORT                  GPIOA
#define PROBE_PIN                   1                           // BLTouch PA1

#if SDCARD_ENABLE
#define SD_CS_PORT                  GPIOA
#define SD_CS_PIN                   4
#define SPI_PORT                    1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7 (EXP2 Header)
#endif

#if TRINAMIC_UART_ENABLE

#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE        2

#define MOTOR_UARTX_PORT            GPIOE
#define MOTOR_UARTX_PIN             8
#define MOTOR_UARTY_PORT            GPIOC
#define MOTOR_UARTY_PIN             4
#define MOTOR_UARTZ_PORT            GPIOD
#define MOTOR_UARTZ_PIN             12

#endif

