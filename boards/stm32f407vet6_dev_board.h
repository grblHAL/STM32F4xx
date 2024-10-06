/*
  stm32f407vet6_dev_board.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "STM32F407VET6 Dev Board"
#define BOARD_URL "https://github.com/terjeio/CNC_Breakout_Nucleo64"

/*

P4:
-------------------
3V3  -  1  2 - GND
PA5  -  3  4 - PA6
PA3  -  5  6 - PA4
PC0  -  7  8 - PA0
PE6  -  9 10 - PC13
PE4  - 11 12 - PE5
PE1  - 13 14 - PE0
PB7  - 15 16 - PD4

P5:
-------------------
3V3  -  1  2 - GND
PC7  -  3  4 - PA8
PD15 -  5  6 - PC6
PD13 -  7  8 - PD14
PD11 -  9 10 - PD12
PD9  - 11 12 - PD10
PB1  - 13 14 - PD8
PB0  - 15 16 - NC

PA0 - SS
PA3 - LX
PA4 - LY
PA5 LZ
PA6 LM3
PA8 LM4
PB0 - SPWM
PB1 - Start
PB7 - Hold
PC0 - Reset
PC6 - SPON
PC7 - SPDIR
PC13 - DOOR
PE0 DM4
PE1 DM3
PE4 DX
PE5 DY
PE6 DZ
PD4 SEna
PD8 FLOOD
PD9 MIST
PD10
PD11 SX
PD12 SY
PD13 SZ
PD14 SM3
PD15 SM4

 */

/*
#ifndef WEB_BUILD
#undef I2C_ENABLE
#undef EEPROM_ENABLE
//#undef EEPROM_IS_FRAM
#define I2C_ENABLE      1
#define EEPROM_ENABLE   16
//#define EEPROM_IS_FRAM  1
#endif
*/
#define SERIAL_PORT     1   // GPIOA: TX = 9, RX = 10
#if I2C_ENABLE
#define I2C_PORT        1   // GPIOB: SCL = 8, SDA = 9
#endif

//#define IS_NUCLEO_BOB
//#define HAS_BOARD_INIT

#if SDCARD_ENABLE
#define SDCARD_SDIO             1
#endif

#define SPI_PORT               21   // GOPIB: SCK = 10, GPIOC: MISO - 2, MOSI - 3

// Define step pulse output pins.

#define STEP_PORT               GPIOD
#define X_STEP_PIN              11
#define Y_STEP_PIN              12
#define Z_STEP_PIN              13
#if N_GANGED
#define STEP_OUTMODE            GPIO_BITBAND
#else
#define STEP_OUTMODE            GPIO_MAP
#endif

// Define step direction output pins.
#define DIRECTION_PORT          GPIOE
#define X_DIRECTION_PIN         4
#define Y_DIRECTION_PIN         5
#define Z_DIRECTION_PIN         6
#if N_GANGED
#define DIRECTION_OUTMODE       GPIO_BITBAND
#else
#define DIRECTION_OUTMODE       GPIO_MAP
#endif

// Define stepper driver enable/disable output pins.
#define STEPPERS_ENABLE_PORT    GPIOD
#define STEPPERS_ENABLE_PIN     4
/*
#define X_ENABLE_PORT           GPIOA
#define X_ENABLE_PIN            1
#define Y_ENABLE_PORT           GPIOB
#define Y_ENABLE_PIN            12
*/
#define Z_ENABLE_PORT           GPIOD
#define Z_ENABLE_PIN            10

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOA
#define X_LIMIT_PIN             3
#define Y_LIMIT_PIN             4
#define Z_LIMIT_PIN             5
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS >= 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOD
#define M3_STEP_PIN             14
#define M3_DIRECTION_PORT       GPIOE
#define M3_DIRECTION_PIN        0
#define M3_LIMIT_PORT           GPIOA
#define M3_LIMIT_PIN            6
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOD
#define M4_STEP_PIN             15
#define M4_DIRECTION_PORT       GPIOE
#define M4_DIRECTION_PIN        1
#define M4_LIMIT_PORT           GPIOA
#define M4_LIMIT_PIN            8
#endif

/*
#define AUXOUTPUT0_PORT         GPIOB // Aux 0
#define AUXOUTPUT0_PIN          15
#if !ETHERNET_ENABLE
#define AUXOUTPUT1_PORT         GPIOB // Aux 1
#define AUXOUTPUT1_PIN          2
#endif
#ifndef SPI_PORT
#define AUXOUTPUT2_PORT         GPIOA // SDO
#define AUXOUTPUT2_PIN          6
#define AUXOUTPUT3_PORT         GPIOA // SCK
#define AUXOUTPUT3_PIN          5
#endif
*/
#define AUXOUTPUT4_PORT         GPIOB // Spindle PWM
#define AUXOUTPUT4_PIN          0
#define AUXOUTPUT5_PORT         GPIOC // Spindle direction
#define AUXOUTPUT5_PIN          6
#define AUXOUTPUT6_PORT         GPIOC // Spindle enable
#define AUXOUTPUT6_PIN          7

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     AUXOUTPUT6_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT6_PIN
#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT        AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT5_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT5_PIN
#endif
#endif //DRIVER_SPINDLE_ENABLE

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOD
#define COOLANT_FLOOD_PIN       8
#define COOLANT_MIST_PORT       GPIOD
#define COOLANT_MIST_PIN        9

// Define user-control controls (cycle start, reset, feed hold) input pins.
/*
#define CONTROL_PORT            GPIOC
#define RESET_PIN               0
#define FEED_HOLD_PIN           7
#define CYCLE_START_PIN         13
#define CONTROL_INMODE          GPIO_BITBAND
*/
#define RESET_PORT              GPIOC
#define RESET_PIN               0
#define FEED_HOLD_PORT          GPIOB
#define FEED_HOLD_PIN           1
#define CYCLE_START_PORT        GPIOB
#define CYCLE_START_PIN         7
#define CONTROL_INMODE          GPIO_BITBAND

#ifdef SPI_PORT
#define SPI_CS_PORT             GPIOE
#define SPI_CS_PIN              7
#define SPI_IRQ_PORT            GPIOE
#define SPI_IRQ_PIN             9
#define SPI_RST_PORT            GPIOE
#define SPI_RST_PIN             8
#endif

// LED 1 - 3, all red
#define LED_R_PORT              GPIOE
#define LED_R_PIN               13
#define LED_G_PORT              GPIOE
#define LED_G_PIN               14
#define LED_B_PORT              GPIOE
#define LED_B_PIN               15

// Button 1 - 3
// PE10 - 12

//#define NEOPIXEL_GPO
//#define LED_PORT                GPIOA
//#define LED_PIN                 7

//#define NEOPIXEL_SPI            1 // PA7
//#define NEOPIXELS_NUM           6

#if !SPINDLE_SYNC_ENABLE
#define AUXINPUT0_PORT          GPIOC
#define AUXINPUT0_PIN           13
#endif
/*
#if !SPINDLE_ENCODER_ENABLE && !QEI_ENABLE
#define AUXINPUT1_PORT          GPIOA
#define AUXINPUT1_PIN           15
#endif

#ifndef M3_LIMIT_PIN
#define AUXINPUT2_PORT          GPIOC
#define AUXINPUT2_PIN           11
#endif

#define AUXINPUT3_PORT          GPIOB
#define AUXINPUT3_PIN           0
#define AUXINPUT4_PORT          GPIOC
#define AUXINPUT4_PIN           1
*/
#define AUXINPUT5_PORT          GPIOE
#define AUXINPUT5_PIN           2
/*
#ifndef SPI_PORT
#define AUXINPUT6_PORT          GPIOC
#define AUXINPUT6_PIN           8
#elif N_ABC_MOTORS == 0
#define AUXINPUT6_PORT          GPIOC
#define AUXINPUT6_PIN           11
#endif
*/

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT5_PORT
#define PROBE_PIN               AUXINPUT5_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

/*
#if MOTOR_FAULT_ENABLE && defined(AUXINPUT1_PORT)
#define MOTOR_FAULT_PORT        AUXINPUT1_PORT
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

#if MOTOR_WARNING_ENABLE && !I2C_STROBE_ENABLE
#define MOTOR_WARNING_PORT      AUXINPUT3_PORT
#define MOTOR_WARNING_PIN       AUXINPUT3_PIN
#endif

#if MPG_ENABLE == 1
#define MPG_AUX_ENABLE
#define MPG_MODE_PORT           AUXINPUT6_PORT
#define MPG_MODE_PIN            AUXINPUT6_PIN
#endif

#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PORT      GPIOA
#define SPINDLE_PULSE_PIN       0
#if SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       14
#endif
#endif
*/
#if QEI_ENABLE && !SPINDLE_SYNC_ENABLE
#define QEI_A_PORT              GPIOA
#define QEI_A_PIN               15
#define QEI_B_PORT              GPIOB
#define QEI_B_PIN               14
#endif // QEI_ENABLE

#if QEI_SELECT_ENABLE
#if !I2C_STROBE_ENABLE
#define QEI_SELECT_PORT         AUXINPUT3_PORT
#define QEI_SELECT_PIN          AUXINPUT3_PIN
#elif !SAFETY_DOOR_ENABLE
#define QEI_SELECT_PORT         AUXINPUT4_PORT
#define QEI_SELECT_PIN          AUXINPUT4_PIN
#endif
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define SERIAL1_PORT           21 // GPIOD: TX = 5, RX = 6
#if MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED
#define MODBUS_RTU_STREAM       1
#define MODBUS_DIR_AUX          0 // GPIOD 7
#endif
#endif

#define FLASH_CS_PORT           GPIOE
#define FLASH_CS_PIN            3

#define CAN_PORT                GPIOD
#define CAN_RX_PIN              0
#define CAN_TX_PIN              1

// EOF
