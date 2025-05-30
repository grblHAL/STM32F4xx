/*
  mks_eagle_map.h - driver code for STM32F407 ARM processors

  Part of grblHAL

  Copyright (c) 2021 qbazd
  Copyright (c) 2023 Terje Io

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
#error "MKS EAGLE supports 5 motors max."
#endif

#if IS_NUCLEO_DEVKIT
// When debugging with Nucleo-144
#define IS_NUCLEO_BOB
#define SERIAL_PORT    32   // GPIOD: TX = 8, RX = 9
#define SERIAL1_PORT    2   // GPIOB: TX = 10, RX = 11
#elif !defined(STM32F407xx) || HSE_VALUE != 8000000
#error "This board has STM32F407 processor with a 8MHz crystal, select a corresponding build!"
#else
#undef EEPROM_ENABLE
#define EEPROM_ENABLE  32
#define SERIAL_PORT     1   // GPIOA: TX = 9, RX = 10
#define SERIAL1_PORT    3   // GPIOB: TX = 10, RX = 11
#endif

#define BOARD_NAME "MKS EAGLE"
#define BOARD_URL "https://github.com/makerbase-mks/MKS-EAGLE"

#undef I2C_ENABLE
#define I2C_ENABLE      1
#define I2C_PORT        1
#define I2C1_ALT_PINMAP     // GPIOB, SCL_PIN = 6, SDA_PIN = 7

// If we want to debug, we need to use USART1
//#if defined(DEBUG) && defined(USB_SERIAL_CDC)
//#undef USB_SERIAL_CDC
//#endif

// Define step pulse output pins.
#define X_STEP_PORT             GPIOE
#define X_STEP_PIN              3                   // X
#define Y_STEP_PORT             GPIOE
#define Y_STEP_PIN              0                   // Y
#define Z_STEP_PORT             GPIOB
#define Z_STEP_PIN              5                   // Z
#define STEP_OUTMODE            GPIO_BITBAND
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define X_DIRECTION_PORT        GPIOE
#define X_DIRECTION_PIN         2
#define Y_DIRECTION_PORT        GPIOB
#define Y_DIRECTION_PIN         9
#define Z_DIRECTION_PORT        GPIOB
#define Z_DIRECTION_PIN         4
#define DIRECTION_OUTMODE       GPIO_BITBAND
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT           GPIOE
#define X_ENABLE_PIN            4
#define Y_ENABLE_PORT           GPIOE
#define Y_ENABLE_PIN            1
#define Z_ENABLE_PORT           GPIOB
#define Z_ENABLE_PIN            8
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

//// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOA
#define X_LIMIT_PIN             15                          // X- Limit
#define Y_LIMIT_PORT            GPIOD
#define Y_LIMIT_PIN             2                           // Y- Limit
#define Z_LIMIT_PORT            GPIOC
#define Z_LIMIT_PIN             8                           // Z- Limit
#define LIMIT_INMODE            GPIO_BITBAND

//Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE            // E0
#define M3_STEP_PORT            GPIOD
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       GPIOD
#define M3_DIRECTION_PIN        3
#define M3_LIMIT_PORT           GPIOA                 //PW_DET
#define M3_LIMIT_PIN            13
#define M3_ENABLE_PORT          GPIOB
#define M3_ENABLE_PIN           3
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE            // E1
#define M4_STEP_PORT            GPIOD
#define M4_STEP_PIN             15
#define M4_DIRECTION_PORT       GPIOA
#define M4_DIRECTION_PIN        1
#define M4_LIMIT_PORT           GPIOE
#define M4_LIMIT_PIN            7
#define M4_ENABLE_PORT          GPIOA
#define M4_ENABLE_PIN           3
#endif

#define AUXOUTPUT0_PORT         GPIOA   // Spindle PWM, 3D touch
#define AUXOUTPUT0_PIN          8
#define AUXOUTPUT1_PORT         GPIOB   // Spindle direction, FAN2
#define AUXOUTPUT1_PIN          1
#define AUXOUTPUT2_PORT         GPIOA   // Spindle enable, HOTBED
#define AUXOUTPUT2_PIN          0
#define AUXOUTPUT3_PORT         GPIOE   // Coolant flood, HEATER 1
#define AUXOUTPUT3_PIN          5
#define AUXOUTPUT4_PORT         GPIOB   // Coolant mist, FAN2
#define AUXOUTPUT4_PIN          1

// Define driver spindle pins

#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT     AUXOUTPUT2_PORT
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT        AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT  AUXOUTPUT1_PORT
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT      AUXOUTPUT3_PORT
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT       AUXOUTPUT4_PORT
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

#define AUXINPUT0_PORT          GPIOC // EXP1 PC5
#define AUXINPUT0_PIN           5
#define AUXINPUT1_PORT          GPIOC // Z+ Limit
#define AUXINPUT1_PIN           4
#define AUXINPUT2_PORT          GPIOC // Reset/EStop
#define AUXINPUT2_PIN           0
#define AUXINPUT3_PORT          GPIOC // Feed hold
#define AUXINPUT3_PIN           1
#define AUXINPUT4_PORT          GPIOA // Cycle start
#if IS_NUCLEO_DEVKIT
#define AUXINPUT4_PIN           10    //  - when debugging with Nucleo-144
#else
#define AUXINPUT4_PIN           4     //  - MT_DET1
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT2_PORT
#define RESET_PIN               AUXINPUT2_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT3_PORT
#define FEED_HOLD_PIN           AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT4_PORT
#define CYCLE_START_PIN         AUXINPUT4_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT              AUXINPUT1_PORT
#define PROBE_PIN               AUXINPUT1_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOC
#define SD_CS_PIN               9
#define SPI_PORT                3 // GPIOC, SCK_PIN = 10, MISO_PIN = 11, MOSI_PIN = 12 (SPI3)
#endif

#if TRINAMIC_UART_ENABLE

#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE    2

#define MOTOR_UARTX_PORT        GPIOD
#define MOTOR_UARTX_PIN         5
#define MOTOR_UARTY_PORT        GPIOD
#define MOTOR_UARTY_PIN         7
#define MOTOR_UARTZ_PORT        GPIOD
#define MOTOR_UARTZ_PIN         4

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT       GPIOD
#define MOTOR_UARTM3_PIN        9
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT       GPIOD
#define MOTOR_UARTM4_PIN        8
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT       GPIOD
#define MOTOR_UARTM5_PIN        4
#endif

#endif // TRINAMIC_UART_ENABLE

#if TRINAMIC_SPI_ENABLE

#define TRINAMIC_SPI_PORT       3 // GPIOC, SCK_PIN = 10, MISO_PIN = 12, MOSI_PIN = 11

#define MOTOR_CSX_PORT          GPIOA
#define MOTOR_CSX_PIN           15
#define MOTOR_CSY_PORT          GPIOB
#define MOTOR_CSY_PIN           8
#define MOTOR_CSZ_PORT          GPIOB
#define MOTOR_CSZ_PIN           9

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT         GPIOB
#define MOTOR_CSM3_PIN          3
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT         GPIOG
#define MOTOR_CSM4_PIN          15
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT         GPIOG
#define MOTOR_CSM5_PIN          12
#endif

#endif // TRINAMIC_SPI_PORT
