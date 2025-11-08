/*
  btt_octopus_pro_map.h - Board map for BIGTREETECH Octopus Pro v1.1

  Part of grblHAL

  Copyright (c) 2024 Joe Corelli
  Copyright (c) 2024 Jon Escombe
  Copyright (c) 2025 Michael Griffin

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.
*/

#if N_ABC_MOTORS > 8
#error "Octopus Pro v1.1 board map is only configured for 8 motors max."
#endif

#if !((defined(STM32F446xx) || defined(STM32F429xx)) && HSE_VALUE == 12000000) && !(defined(DEBUG) && IS_NUCLEO_DEVKIT == 144)
#error "This board has a STM32F429 or STM32F446 processor with 12MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "BTT Octopus Pro v1.1"
#define BOARD_URL "https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro"

#define SERIAL_PORT                 1       // GPIOA: TX = 9, RX = 10,  USART 1
#define SERIAL1_PORT                21      // GPIOD: TX = 5, RX = 6,   USART 2
#define SERIAL2_PORT                32      // GPIOD: TX = 8, RX = 9,   USART 3
#define I2C_PORT                    1       // GPIOB: SCL = 8, SDA = 9
#define SPI_PORT                    1       // GPIOA: SCK = 5, MISO = 6, MOSI = 7

//we have multiple SPI but only one can be used, for now
//#define SPI_PORT                  12       // GPIOB: SCK = 3, MISO = 4, MOSI = 5

//#define TRINAMIC_SOFT_SPI

// Motor Reference:
// Motor-1: X-axis
// Motor-2: Y-axis
// Motor-3: Z-axis
// Motor-4: M3-axis
// Motor-5: M4-axis
// Motor-6: M5-axis
// Motor-7: M6-axis
// Motor-8: M7-axis

// Define step pulse output pins.
#define X_STEP_PORT                 GPIOF
#define X_STEP_PIN                  13
#define Y_STEP_PORT                 GPIOG
#define Y_STEP_PIN                  0
#define Z_STEP_PORT                 GPIOF
#define Z_STEP_PIN                  11
#define STEP_OUTMODE                GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT            GPIOF
#define X_DIRECTION_PIN             12
#define Y_DIRECTION_PORT            GPIOG
#define Y_DIRECTION_PIN             1
#define Z_DIRECTION_PORT            GPIOG
#define Z_DIRECTION_PIN             3
#define DIRECTION_OUTMODE           GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define X_ENABLE_PORT               GPIOF
#define X_ENABLE_PIN                14
#define Y_ENABLE_PORT               GPIOF
#define Y_ENABLE_PIN                15
#define Z_ENABLE_PORT               GPIOG
#define Z_ENABLE_PIN                5

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT                GPIOG
#define X_LIMIT_PIN                 6       // MIN1
#define Y_LIMIT_PORT                GPIOG
#define Y_LIMIT_PIN                 9       // MIN2
#define Z_LIMIT_PORT                GPIOG
#define Z_LIMIT_PIN                 10       // MIN3
#define LIMIT_INMODE                GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE                        // Motor-4
#define M3_STEP_PORT                GPIOG
#define M3_STEP_PIN                 4
#define M3_DIRECTION_PORT           GPIOC
#define M3_DIRECTION_PIN            1
#define M3_LIMIT_PORT               GPIOG
#define M3_LIMIT_PIN                11       // MIN4
#define M3_ENABLE_PORT              GPIOA
#define M3_ENABLE_PIN               2       // EN for M3 motor
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 1
#define M4_AVAILABLE                        // Motor-5
#define M4_STEP_PORT                GPIOF
#define M4_STEP_PIN                 9
#define M4_DIRECTION_PORT           GPIOF
#define M4_DIRECTION_PIN            10
#define M4_LIMIT_PORT               GPIOG
#define M4_LIMIT_PIN                12       // MIN5
#define M4_ENABLE_PORT              GPIOG
#define M4_ENABLE_PIN               2      // EN for M4
#endif

// Define ganged axis or C axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 2
#define M5_AVAILABLE                        // Motor-6
#define M5_STEP_PORT                GPIOC
#define M5_STEP_PIN                 13
#define M5_DIRECTION_PORT           GPIOF
#define M5_DIRECTION_PIN            0
#define M5_LIMIT_PORT               GPIOG
#define M5_LIMIT_PIN                13      // MIN6
#define M5_ENABLE_PORT              GPIOF
#define M5_ENABLE_PIN               1       // EN for M5 motor
#endif

#if N_ABC_MOTORS > 3
#define M6_AVAILABLE                        // Motor-7
#define M6_STEP_PORT                GPIOE
#define M6_STEP_PIN                 2
#define M6_DIRECTION_PORT           GPIOE
#define M6_DIRECTION_PIN            3
#define M6_LIMIT_PORT               GPIOG
#define M6_LIMIT_PIN                14      // MIN6
#define M6_ENABLE_PORT              GPIOD
#define M6_ENABLE_PIN               4       // EN for M5 motor
#endif

#if N_ABC_MOTORS > 4
#define M7_AVAILABLE                        // Motor-8
#define M7_STEP_PORT                GPIOE
#define M7_STEP_PIN                 6
#define M7_DIRECTION_PORT           GPIOA
#define M7_DIRECTION_PIN            14
#define M7_LIMIT_PORT               GPIOG
#define M7_LIMIT_PIN                15      // MIN6
#define M7_ENABLE_PORT              GPIOE
#define M7_ENABLE_PIN               0       // EN for M5 motor
#endif

#define AUXOUTPUT0_PORT             GPIOA   // Spindle PWM - FAN0
#define AUXOUTPUT0_PIN              8

#define AUXOUTPUT1_PORT             GPIOE   // - FAN1
#define AUXOUTPUT1_PIN              5

#define AUXOUTPUT2_PORT             GPIOD   // - FAN2
#define AUXOUTPUT2_PIN              12

#define AUXOUTPUT3_PORT             GPIOD   // - FAN3
#define AUXOUTPUT3_PIN              13

#define AUXOUTPUT4_PORT             GPIOD   // Spindle enable - FAN4
#define AUXOUTPUT4_PIN              14

#define AUXOUTPUT5_PORT             GPIOE   // Spindle direction - FAN5
#define AUXOUTPUT5_PIN              15

#define AUXOUTPUT6_PORT             GPIOA   // Coolant flood - HE0
#define AUXOUTPUT6_PIN              0

#define AUXOUTPUT7_PORT             GPIOA   // Coolant mist - HE1
#define AUXOUTPUT7_PIN              3

/*
//we need more auxoutput ports defined in outputpin[] to use the full outputs
#define AUXOUTPUT8_PORT             GPIOB   // - HE2
#define AUXOUTPUT8_PIN              0

#define AUXOUTPUT9_PORT             GPIOB   // - HE3
#define AUXOUTPUT9_PIN              11

#define AUXOUTPUT10_PORT            GPIOA   // - Bed-out
#define AUXOUTPUT10_PIN             1

#define AUXOUTPUT11_PORT            GPIOB   // - RGB
#define AUXOUTPUT11_PIN             10
*/

// Define driver spindle pins.
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PORT         AUXOUTPUT4_PORT
#define SPINDLE_ENABLE_PIN          AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PORT            AUXOUTPUT0_PORT
#define SPINDLE_PWM_PIN             AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PORT      AUXOUTPUT5_PORT
#define SPINDLE_DIRECTION_PIN       AUXOUTPUT5_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PORT          AUXOUTPUT6_PORT
#define COOLANT_FLOOD_PIN           AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PORT           AUXOUTPUT7_PORT
#define COOLANT_MIST_PIN            AUXOUTPUT7_PIN
#endif

#define AUXINPUT0_PORT              GPIOC   // Safety door - PWR-DET
#define AUXINPUT0_PIN               0
#define AUXINPUT1_PORT              GPIOB   // Probe - Z probe "left"
#define AUXINPUT1_PIN               6
#define AUXINPUT2_PORT              GPIOB   // Z probe "right"
#define AUXINPUT2_PIN               7
#define AUXINPUT3_PORT              GPIOB   //Button on PCB
#define AUXINPUT3_PIN               2
#define AUXINPUT4_PORT              GPIOF   // Reset - TB
#define AUXINPUT4_PIN               3
#define AUXINPUT5_PORT              GPIOF   // Feed hold - T0
#define AUXINPUT5_PIN               4
#define AUXINPUT6_PORT              GPIOF   // Cycle start - T1
#define AUXINPUT6_PIN               5

#define AUXINTPUT0_ANALOG_PORT      GPIOF   //T2
#define AUXINTPUT0_ANALOG_PIN       6

#define AUXINTPUT1_ANALOG_PORT      GPIOF   //T3
#define AUXINTPUT1_ANALOG_PIN       7

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT                  AUXINPUT4_PORT
#define RESET_PIN                   AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT              AUXINPUT5_PORT
#define FEED_HOLD_PIN               AUXINPUT5_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT            AUXINPUT6_PORT
#define CYCLE_START_PIN             AUXINPUT6_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT            AUXINPUT0_PORT
#define SAFETY_DOOR_PIN             AUXINPUT0_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PORT                  AUXINPUT1_PORT
#define PROBE_PIN                   AUXINPUT1_PIN
#endif

#if SDCARD_ENABLE
#define SDCARD_SDIO                 1
#ifndef M6_LIMIT_PORT
#define SD_DETECT_PORT              GPIOC
#define SD_DETECT_PIN               14
#endif
#endif

//Pins not used
// DC Probe pin? Connected to an EL357C on GPIOC5 conflicts with T1/cycle start but this may be preferred as it is optocoupled. Only one?
// PS-ON is not used due to conflict with limit switches. Can be used as an analog in.

#if TRINAMIC_UART_ENABLE //Not tested, use with care

#define MOTOR_UARTX_PORT            GPIOC
#define MOTOR_UARTX_PIN             4
#define MOTOR_UARTY_PORT            GPIOD
#define MOTOR_UARTY_PIN             11
#define MOTOR_UARTZ_PORT            GPIOC
#define MOTOR_UARTZ_PIN             6

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT           GPIOC
#define MOTOR_UARTM3_PIN            7
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT           GPIOF
#define MOTOR_UARTM4_PIN            2
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_UARTM5_PORT           GPIOE
#define MOTOR_UARTM5_PIN            4
#endif

#ifdef  M6_AVAILABLE
#define MOTOR_UARTM6_PORT           GPIOE
#define MOTOR_UARTM6_PIN            1
#endif

#ifdef  M7_AVAILABLE
#define MOTOR_UARTM7_PORT           GPIOD
#define MOTOR_UARTM7_PIN            3
#endif

#elif TRINAMIC_SPI_ENABLE

#ifdef TRINAMIC_SOFT_SPI // Software SPI implementation

#define TRINAMIC_MOSI_PORT          GPIOA
#define TRINAMIC_MOSI_PIN           7
#define TRINAMIC_SCK_PORT           GPIOA
#define TRINAMIC_SCK_PIN            5
#define TRINAMIC_MISO_PORT          GPIOA
#define TRINAMIC_MISO_PIN           6

#endif //TRINAMIC_SOFT_SPI

#define MOTOR_CSX_PORT              GPIOC
#define MOTOR_CSX_PIN               4
#define MOTOR_CSY_PORT              GPIOD
#define MOTOR_CSY_PIN               11
#define MOTOR_CSZ_PORT              GPIOC
#define MOTOR_CSZ_PIN               6

#ifdef  M3_AVAILABLE
#define MOTOR_CSM3_PORT             GPIOC
#define MOTOR_CSM3_PIN              7
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_CSM4_PORT             GPIOF
#define MOTOR_CSM4_PIN              2
#endif

#ifdef  M5_AVAILABLE
#define MOTOR_CSM5_PORT             GPIOE
#define MOTOR_CSM5_PIN              4
#endif

#ifdef  M6_AVAILABLE
#define MOTOR_CSM6_PORT             GPIOE
#define MOTOR_CSM6_PIN              1
#endif

#ifdef  M7_AVAILABLE
#define MOTOR_CSM7_PORT             GPIOD
#define MOTOR_CSM7_PIN              3
#endif

#endif

#define CAN_PORT                    GPIOD
#define CAN_RX_PIN                  0
#define CAN_TX_PIN                  1

// EOF
