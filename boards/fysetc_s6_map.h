/*
  fysetc_s6_map.h - driver code for STM32F407 ARM processors

  Part of grblHAL

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

#if N_ABC_MOTORS > 3
#error "Axis configuration is not supported!"
#endif

#if !defined(STM32F446xx) || HSE_VALUE != 12000000
#error "This board has STM32F446 processor with a 12MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "Fysetc S6 V2.0"
#define BOARD_URL "https://github.com/FYSETC/FYSETC-S6"

#undef I2C_ENABLE
#undef EEPROM_ENABLE

#define SERIAL_PORT     1   // GPIOA: TX = 9, RX = 10
#define I2C_ENABLE      1
#define I2C_PORT        1   // GPIOB: SCL = 8, SDA = 9
#ifndef WEB_BUILD
#define EEPROM_ENABLE   16  // 2K single byte addressing
#endif

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

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                // E0
#define M3_STEP_PORT                GPIOD
#define M3_STEP_PIN                 5
#define M3_DIRECTION_PORT           GPIOD
#define M3_DIRECTION_PIN            6
//#define M3_LIMIT_PORT               GPIOA
//#define M3_LIMIT_PIN                1
#define M3_ENABLE_PORT              GPIOD
#define M3_ENABLE_PIN               4
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                // E1
#define M4_STEP_PORT                GPIOE
#define M4_STEP_PIN                 6
#define M4_DIRECTION_PORT           GPIOC
#define M4_DIRECTION_PIN            13
//#define M4_LIMIT_PORT               GPIOA
//#define M4_LIMIT_PIN                2
#define M4_ENABLE_PORT              GPIOE
#define M4_ENABLE_PIN               5
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 3
#define M5_AVAILABLE                // E2
#define M5_STEP_PORT                GPIOE
#define M5_STEP_PIN                 2
#define M5_DIRECTION_PORT           GPIOE
#define M5_DIRECTION_PIN            4
//#define M5_LIMIT_PORT               GPIOA
//#define M5_LIMIT_PIN                3
#define M5_ENABLE_PORT              GPIOE
#define M5_ENABLE_PIN               3
#endif

#define AUXOUTPUT0_PORT             GPIOB // Spindle PWM, FAN0
#define AUXOUTPUT0_PIN              0
#define AUXOUTPUT1_PORT             GPIOB // Spindle direction, FAN2
#define AUXOUTPUT1_PIN              2
#define AUXOUTPUT2_PORT             GPIOB // Spindle enable, FAN1
#define AUXOUTPUT2_PIN              1
#define AUXOUTPUT3_PORT             GPIOB // Coolant flood, HEAT0
#define AUXOUTPUT3_PIN              1
#define AUXOUTPUT4_PORT             GPIOD // Coolant mist, HEAT1
#define AUXOUTPUT4_PIN              14

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT              GPIOA // BLTouch PA1
#define AUXINPUT0_PIN               1
#define AUXINPUT1_PORT              GPIOC // Reset/EStop
#define AUXINPUT1_PIN               1
#define AUXINPUT2_PORT              GPIOA // Feed hold
#define AUXINPUT2_PIN               2
#define AUXINPUT3_PORT              GPIOA // Cycle start
#define AUXINPUT3_PIN               3

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT                  AUXINPUT1_PORT
#define RESET_PIN                   AUXINPUT1_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT              AUXINPUT2_PORT
#define FEED_HOLD_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT            AUXINPUT3_PORT
#define CYCLE_START_PIN             AUXINPUT3_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT0_PORT
#define PROBE_PIN                   AUXINPUT0_PIN
#endif

#define LED_R_PORT                  GPIOB
#define LED_R_PIN                   6
#define LED_G_PORT                  GPIOB
#define LED_G_PIN                   5
#define LED_B_PORT                  GPIOB
#define LED_B_PIN                   7

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

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOA
#define MOTOR_UARTM3_PIN            15
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOC
#define MOTOR_UARTM4_PIN            5
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT           GPIOE
#define MOTOR_UARTM5_PIN            10
#endif

#endif // TRINAMIC_UART_ENABLE

#if TRINAMIC_SPI_ENABLE

#define TRINAMIC_SPI_PORT           4 // GPIOE, SCK_PIN = 12, MISO_PIN = 13, MOSI_PIN = 14

#define MOTOR_CSX_PORT              GPIOE
#define MOTOR_CSX_PIN               7
#define MOTOR_CSY_PORT              GPIOE
#define MOTOR_CSY_PIN               15
#define MOTOR_CSZ_PORT              GPIOD
#define MOTOR_CSZ_PIN               10

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOD
#define MOTOR_CSM3_PIN              7
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOC
#define MOTOR_CSM4_PIN              14
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT             GPIOC
#define MOTOR_CSM5_PIN              15
#endif

#endif // TRINAMIC_SPI_ENABLE
