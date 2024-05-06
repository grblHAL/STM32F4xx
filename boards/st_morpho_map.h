/*
  st_morpho_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#if TRINAMIC_ENABLE == 2130
#include "trinamic/tmc2130.h"
#endif

#if TRINAMIC_ENABLE == 2209
#include "trinamic/tmc2209.h"
#endif

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if SDCARD_ENABLE && ETHERNET_ENABLE
#error "SD card and networking plugins cannot be enabled together!"
#endif

#define BOARD_NAME "Nucleo-64 CNC Breakout"
#define BOARD_URL "https://github.com/terjeio/CNC_Breakout_Nucleo64"

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
#define SERIAL_PORT     2  	// GPIOA: TX = 2, RX = 3
#define SERIAL1_PORT    1   // GPIOA: TX = 9, RX = 10
#if I2C_ENABLE
#define I2C_PORT        1   // GPIOB: SCL = 8, SDA = 9
#endif

#define IS_NUCLEO_BOB
#define HAS_BOARD_INIT

#if SDCARD_ENABLE || TRINAMIC_SPI_ENABLE || ETHERNET_ENABLE
#define SPI_PORT                1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOC
#define X_STEP_PIN              0
#define Y_STEP_PIN              5
#define Z_STEP_PIN              9
#if N_GANGED
#define STEP_OUTMODE            GPIO_BITBAND
#else
#define STEP_OUTMODE            GPIO_MAP
#endif

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         0
#define Y_DIRECTION_PIN         4
#define Z_DIRECTION_PIN         11
#if N_GANGED
#define DIRECTION_OUTMODE       GPIO_BITBAND
#else
#define DIRECTION_OUTMODE       GPIO_MAP
#endif

// Define stepper driver enable/disable output pins.
#define X_ENABLE_PORT           GPIOA
#define X_ENABLE_PIN            1
#define Y_ENABLE_PORT           GPIOB
#define Y_ENABLE_PIN            12
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            1

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOC
#define X_LIMIT_PIN             13
#define Y_LIMIT_PIN             12
#define Z_LIMIT_PIN             10
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOC
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOA
#define M3_DIRECTION_PIN        12
#define M3_LIMIT_PORT           GPIOC
#define M3_LIMIT_PIN            11
#define M3_ENABLE_PORT          GPIOB
#define M3_ENABLE_PIN           6
#endif

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

#define AUXOUTPUT4_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT4_PIN          8
#define AUXOUTPUT5_PORT         GPIOB // Spindle direction
#define AUXOUTPUT5_PIN          5
#define AUXOUTPUT6_PORT         GPIOB // Spindle enable
#define AUXOUTPUT6_PIN          3

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
#define COOLANT_FLOOD_PORT      GPIOB
#define COOLANT_FLOOD_PIN       10
#define COOLANT_MIST_PORT       GPIOB
#define COOLANT_MIST_PIN        4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOC
#define RESET_PIN               2
#define FEED_HOLD_PIN           3
#define CYCLE_START_PIN         4
#define CONTROL_INMODE          GPIO_BITBAND

#ifdef SPI_PORT
#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               8
#endif
#if TRINAMIC_SPI_ENABLE
#define MOTOR_CS_PORT           GPIOB
#define MOTOR_CS_PIN            7
#endif
#endif // SPI_PORT

/*
#define LED_R_PORT              GPIOA
#define LED_R_PIN               7
#define LED_G_PORT              GPIOB
#define LED_G_PIN               7
#define LED_B_PORT              GPIOC
#define LED_B_PIN               8
*/

//#define NEOPIXEL_GPO
//#define LED_PORT                GPIOA
//#define LED_PIN                 7

//#define NEOPIXEL_SPI            1 // PA7
//#define NEOPIXELS_NUM           6

#if !SPINDLE_SYNC_ENABLE && !QEI_ENABLE
#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           14
#endif

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
#define AUXINPUT5_PORT          GPIOC
#define AUXINPUT5_PIN           7

#ifndef SPI_PORT
#define AUXINPUT6_PORT          GPIOC
#define AUXINPUT6_PIN           8
#elif N_ABC_MOTORS == 0
#define AUXINPUT6_PORT          GPIOB
#define AUXINPUT6_PIN           6
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT5_PORT
#define PROBE_PIN               AUXINPUT5_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT4_PORT
#define SAFETY_DOOR_PIN         AUXINPUT4_PIN
#endif

#if MOTOR_FAULT_ENABLE && defined(AUXINPUT1_PORT)
#define MOTOR_FAULT_PORT        AUXINPUT1_PORT
#define MOTOR_FAULT_PIN         AUXINPUT1_PIN
#endif

#if MOTOR_WARNING_ENABLE && !I2C_STROBE_ENABLE
#define MOTOR_WARNING_PORT      AUXINPUT3_PORT
#define MOTOR_WARNING_PIN       AUXINPUT3_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT3_PORT
#define I2C_STROBE_PIN          AUXINPUT3_PIN
#endif

#if MPG_MODE == 1
#define MPG_AUX_ENABLE
#define MPG_MODE_PORT           AUXINPUT6_PORT
#define MPG_MODE_PIN            AUXINPUT6_PIN
#endif

#if SPINDLE_ENCODER_ENABLE
#define SPINDLE_PULSE_PORT      GPIOD  // NOTE: this pin is connected to PA15
#define SPINDLE_PULSE_PIN       2
#if SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_PORT      GPIOB
#define SPINDLE_INDEX_PIN       14
#endif
#endif

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

#if ETHERNET_ENABLE
#undef SPI_ENABLE
#define SPI_ENABLE 1
#define SPI_CS_PORT             GPIOC
#define SPI_CS_PIN              8
#define SPI_IRQ_PORT            GPIOB
#define SPI_IRQ_PIN             0
#define SPI_RST_PORT            GPIOB // AUXOUTPUT1
#define SPI_RST_PIN             2
#endif

// EOF
