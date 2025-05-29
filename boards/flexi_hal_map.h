/*
  flexi_hal_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2022 Expatria Technologies

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

#if TRINAMIC_ENABLE
#error "Trinamic plugin not supported!"
#endif

#if !(defined(STM32F446xx) && HSE_VALUE == 25000000) && !(defined(DEBUG) && IS_NUCLEO_DEVKIT == 144)
#error "This board has STM32F446 processor with a 25MHz crystal, select a corresponding build!"
#endif

#if defined(DEBUG) && IS_NUCLEO_DEVKIT == 144
#define ENABLE_SWD
#endif

#if MPG_ENABLE && ETHERNET_ENABLE
#error "Networking and MGP Serial mode cannot be enabled together!"
#endif

#ifndef BOARD_NAME
#define BOARD_NAME "Flexi-HAL"
#endif
#define BOARD_URL "https://github.com/Expatria-Technologies/Flexi-HAL"

#define WIZCHIP_SPI_PRESCALER SPI_BAUDRATEPRESCALER_4

#if !(IS_NUCLEO_DEVKIT == 144)
#undef I2C_ENABLE
#undef EEPROM_ENABLE
#define I2C_ENABLE 1
#define I2C_FASTMODE
#define EEPROM_ENABLE 128
#endif

#define HAS_BOARD_INIT

#if MODBUS_ENABLE
#define SERIAL2_PORT 33
#define MODBUS_RTU_STREAM 2
#endif

#if MPG_ENABLE == 1
#define MPG_MODE_PORT           GPIOA
#define MPG_MODE_PIN            15
#undef MPG_STREAM
#define MPG_STREAM 1
#define SERIAL1_PORT 1
#endif

//********on first revision of this board Y step/dir was flipped.  Use below config?

// Define step pulse output pins.
#define X_STEP_PORT             GPIOA
#define X_STEP_PIN              3
#define Y_STEP_PORT             GPIOC
#define Y_STEP_PIN              1
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              8
#define STEP_OUTMODE            GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOC
#define X_DIRECTION_PIN         2
#define Y_DIRECTION_PORT        GPIOC
#define Y_DIRECTION_PIN         0
#define Z_DIRECTION_PORT        GPIOC
#define Z_DIRECTION_PIN         15
#define DIRECTION_OUTMODE       GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#ifdef ENABLE_SWD
#define STEPPERS_ENABLE_PORT       GPIOB
#define STEPPERS_ENABLE_PIN        13
#define STEPPERS_ENABLE_OUTMODE    GPIO_BITBAND
#else
#define X_ENABLE_PORT               GPIOA
#define X_ENABLE_PIN                14
#define Y_ENABLE_PORT               GPIOA
#define Y_ENABLE_PIN                14
#define Z_ENABLE_PORT               GPIOA
#define Z_ENABLE_PIN                13
#define STEPPERS_ENABLE_OUTMODE    GPIO_BITBAND
#endif

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOA
#define X_LIMIT_PIN             5
#define Y_LIMIT_PORT            GPIOB
#define Y_LIMIT_PIN             9
//bill mill uses one of the encoder inputs for Z limit
#if (BILLMILL)
  #define Z_LIMIT_PORT            GPIOA
  #define Z_LIMIT_PIN             0
#else
  #define Z_LIMIT_PORT            GPIOC
  #define Z_LIMIT_PIN             13
#endif
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PORT            GPIOD
#define M3_STEP_PIN             2
#define M3_DIRECTION_PORT       GPIOC
#define M3_DIRECTION_PIN        12
#define M3_LIMIT_PORT           GPIOB
#define M3_LIMIT_PIN            6
#ifdef ENABLE_SWD
#define M3_ENABLE_PORT          GPIOB
#define M3_ENABLE_PIN           13
#else
#define M3_ENABLE_PORT          GPIOA
#define M3_ENABLE_PIN           14
#endif
#endif

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PORT            GPIOB
#define M4_STEP_PIN             14
#define M4_DIRECTION_PORT       GPIOB
#define M4_DIRECTION_PIN        15
#define M4_LIMIT_PORT           GPIOC
#define M4_LIMIT_PIN            14
#ifdef ENABLE_SWD
#define M3_ENABLE_PORT          GPIOB
#define M3_ENABLE_PIN           13
#else
#define M4_ENABLE_PORT          GPIOA
#define M4_ENABLE_PIN           14
#endif
#endif

#define AUXOUTPUT0_PORT         GPIOB
#define AUXOUTPUT0_PIN          13
#define AUXOUTPUT1_PORT         GPIOB
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         GPIOA
#define AUXOUTPUT2_PIN          4
#define AUXOUTPUT3_PORT         GPIOA
#define AUXOUTPUT3_PIN          6
#define AUXOUTPUT4_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT4_PIN          8
#define AUXOUTPUT5_PORT         GPIOB // Spindle direction
#define AUXOUTPUT5_PIN          1
#define AUXOUTPUT6_PORT         GPIOB // Spindle enable
#define AUXOUTPUT6_PIN          2
#define AUXOUTPUT7_PORT         GPIOC // Coolant flood
#define AUXOUTPUT7_PIN          9
#define AUXOUTPUT8_PORT         GPIOA // Coolant mist
#define AUXOUTPUT8_PIN          7

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT6_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT6_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT4_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT5_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT5_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT7_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT7_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT8_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT8_PIN
#endif

//bill mill re-assigns auxinput0 to what was Z limit
#if (BILLMILL)
#define AUXINPUT0_PORT          GPIOC
#define AUXINPUT0_PIN           13
#else
#define AUXINPUT0_PORT          GPIOA
#define AUXINPUT0_PIN           0
#endif
#define AUXINPUT1_PORT          GPIOA
#define AUXINPUT1_PIN           1
#define AUXINPUT2_PORT          GPIOA
#define AUXINPUT2_PIN           2
#define AUXINPUT3_PORT          GPIOC // Safety door
#define AUXINPUT3_PIN           4
#define AUXINPUT4_PORT          GPIOB // Probe input
#define AUXINPUT4_PIN           7
#define AUXINPUT5_PORT          GPIOB // I2C strobe input
#define AUXINPUT5_PIN           10
#if N_ABC_MOTORS != 2
#define AUXINPUT6_PORT          GPIOC
#define AUXINPUT6_PIN           14
#endif
#define AUXINPUT7_PORT          GPIOB // Reset/EStop
#define AUXINPUT7_PIN           12
#define AUXINPUT8_PORT          GPIOC // Feed hold
#define AUXINPUT8_PIN           8
#define AUXINPUT9_PORT          GPIOC // Cycle start
#define AUXINPUT9_PIN           11

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT7_PORT
#define RESET_PIN               AUXINPUT7_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT8_PORT
#define FEED_HOLD_PIN           AUXINPUT8_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT9_PORT
#define CYCLE_START_PIN         AUXINPUT9_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT4_PORT
#define PROBE_PIN               AUXINPUT4_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT3_PORT
#define SAFETY_DOOR_PIN         AUXINPUT3_PIN  
#endif

#if MOTOR_WARNING_ENABLE
#define MOTOR_WARNING_PORT      AUXINPUT2_PORT
#define MOTOR_WARNING_PIN       AUXINPUT2_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PORT         AUXINPUT5_PORT
#define I2C_STROBE_PIN          AUXINPUT5_PIN
#endif

#if SDCARD_ENABLE || ETHERNET_ENABLE
#define SPI_PORT                13 // GPIOB, SCK_PIN = 3, MISO_PIN = 4, MOSI_PIN = 5  probably needs fixing
#endif

#if ETHERNET_ENABLE
//CS is JOG_SW
#undef SPI_ENABLE
#define SPI_ENABLE 1
#define SPI_CS_PORT             GPIOA //CS_JOG_SW
#define SPI_CS_PIN              15
#define SPI_IRQ_PORT            GPIOC //PRU_RESET
#define SPI_IRQ_PIN             3
#define SPI_RST_PORT            GPIOA // TXD_INT
#define SPI_RST_PIN             9
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               10
#endif
