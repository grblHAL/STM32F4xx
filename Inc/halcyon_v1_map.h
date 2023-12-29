/*
  halcyon_v1_map.h - driver code for Halcyon CNC board, based on STM32F401RCT6

  Part of grblHAL

  Copyright (c) 2023 Am0k-GIT

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

/* Pin Assignments:
 *
 * A0  TM2_CH1 PWM (HOTBED / TTL / 0-10V) | B0  ADC8 (T_HOTBED)                 | C0  X_DIR                             | D0  (N/A)
 * A1  TM2_CH2 PWM (HOTEND)               | B1  ADC9 (T_HOTEND)                 | C1  X_STEP                            | D1  (N/A)
 * A2  TM2_CH3 PWM (FAN0 / FLOOD)         | B2  PS_ON (pulldown)                | C2  Y_DIR                             | D2  (EXP 2-08) SD_DET
 * A3  TM2_CH4 PWM (FAN1 / MIST)          | B3  (EXP 1-03) LCD_D7 / I2C_STROBE  | C3  Y_STEP                            | D3  (N/A)
 * A4  (EXP 2-07) SD_SS                   | B4  A_MIN                           | C4  Z_DIR                             | D4  (N/A)
 * A5  (EXP 2-09) SCK                     | B5  Z_MIN                           | C5  Z_STEP                            | D5  (N/A)
 * A6  (EXP 2-10) MISO                    | B6  (EXP 1-07) LCD_RS / SCL / TX    | C6  A_DIR                             | D6  (N/A)
 * A7  (EXP 2-05) MOSI                    | B7  (EXP 1-08) LCD_EN / SDA / RX    | C7  A_STEP                            | D7  (N/A)
 * A8  Enable motors                      | B8  X_MIN                           | C8  B_DIR                             | D8  (N/A)
 * A9  TX                                 | B9  Y_MIN                           | C9  B_STEP                            | D9  (N/A)
 * A10 RX                                 | B10 PROBE_IN (servo0)               | C10 (EXP 1-08) BEEP / CYCLE_START     | D10 (N/A)
 * A11 USB D-                             | B11 (N/A)                           | C11 (EXP 1-08) LCD_D5 / FEED_HOLD     | D11 (N/A)
 * A12 USB D+                             | B12 PROBE_OUT (Z_PROBE)             | C12 (EXP 1-08) LCD_D6                 | D12 (N/A)
 * A13 SWDIO                              | B13 FILAMENT / DOOR                 | C13 (EXP 1-09) BTN_ENC                | D13 (N/A)
 * A14 SWCLK                              | B14 B_MIN                           | C14 (EXP 2-08) BTN_EN1 / SPINDEL_DIR  | D14 (N/A)
 * A15 (EXP 1-06) LCD_D4                  | B15 (EXP 1-08) KILL / E-STOP        | C15 (EXP 2-06) BTN_EN2 / SPINDEL_EN   | D15 (N/A)
 */

/*
 *                             _________                                                        _________                            
 *                        5V   |  1  2 | GND                            (E-STOP / RESET)  PB15  |  1  2 | GND                        
 * (LCD_D7 / I2C_STROBE)  PB3  |  3  4 | PC12  (LCD_D6)                                   RESET |  3  4 | PD2  (SD_DET)                        
 *  (LCD_D5 / FEED_HOLD)  PC11 |  5  6 | PA15  (LCD_D4)                           (MOSI)  PA7   |  5  6 | PC15 (BTN_EN2 / SPINDEL_EN)             
 *   (LCD_RS / SCL / TX)  PB6  |  7  8 | PB7   (LCD_EN / SDA / RX)                (SD_SS) PA4   |  7  8 | PC14 (BTN_EN1 / SPINDEL_DIR)             
 *             (BTN_ENC)  PC13 |  9  10| PC10  (BEEP / CYCLE_START)               (SCK)   PA5   |  9  10| PA6  (MISO)                
 *                             ---------                                                        ---------                            
 *                               EXP1                                                              EXP2                              
 */
                        
 
#define BOARD_NAME "HALCYON"
#define BOARD_URL "https://github.com/Am0k-GIT/HALCYON"

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if KEYPAD_ENABLE == 1
  #define I2C_ENABLE            1
  #define I2C_PORT              1
  #define I2C1_ALT_PINMAP                                            // GPIOB, SCL_PIN = 6, SDA_PIN = 7
  #define I2C_STROBE_ENABLE     1
  #define I2C_STROBE_PORT       GPIOB
  #define I2C_STROBE_PIN        3
#endif

#if KEYPAD_ENABLE == 2
#error "UART KEYPAD connection is not supported!"
#endif

// Define step pulse output pins.
#define STEP_PORT               GPIOC
#define X_STEP_PIN              1
#define Y_STEP_PIN              3
#define Z_STEP_PIN              5
#define STEP_OUTMODE            GPIO_MAP

#define DIRECTION_PORT          GPIOC
#define X_DIRECTION_PIN         0
#define Y_DIRECTION_PIN         2
#define Z_DIRECTION_PIN         4
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOA
#define STEPPERS_ENABLE_PIN     8
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT            GPIOB
#define X_LIMIT_PIN             8                                    // X- Limit
#define Y_LIMIT_PORT            GPIOB
#define Y_LIMIT_PIN             9                                    // Y- Limit
#define Z_LIMIT_PORT            GPIOB
#define Z_LIMIT_PIN             5                                    // Z- Limit
#define LIMIT_INMODE            GPIO_BITBAND

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE            // A
#define M3_STEP_PORT            GPIOC
#define M3_STEP_PIN             7
#define M3_DIRECTION_PORT       GPIOC
#define M3_DIRECTION_PIN        6
#define M3_LIMIT_PORT           GPIOB 
#define M3_LIMIT_PIN            4
#define M3_ENABLE_PORT          STEPPERS_ENABLE_PORT
#define M3_ENABLE_PIN           STEPPERS_ENABLE_PIN
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE            // B
#define M4_STEP_PORT            GPIOC
#define M4_STEP_PIN             9
#define M4_DIRECTION_PORT       GPIOC
#define M4_DIRECTION_PIN        8
#define M4_LIMIT_PORT           GPIOB 
#define M4_LIMIT_PIN            14
#define M4_ENABLE_PORT          STEPPERS_ENABLE_PORT
#define M4_ENABLE_PIN           STEPPERS_ENABLE_PIN
#endif

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
#define SPINDLE_PWM_PIN         0
#else
#define AUXOUTPUT0_PORT         GPIOA
#define AUXOUTPUT0_PIN          0
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PORT  GPIOC
#define SPINDLE_DIRECTION_PIN   15
#else
#define AUXOUTPUT1_PORT         GPIOC
#define AUXOUTPUT1_PIN          15
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PORT     GPIOC
#define SPINDLE_ENABLE_PIN      14
#else
#define AUXOUTPUT2_PORT         GPIOC
#define AUXOUTPUT2_PIN          14
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOA
#define COOLANT_FLOOD_PIN       2
#define COOLANT_MIST_PORT       GPIOA
#define COOLANT_MIST_PIN        3

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define RESET_PORT              GPIOB
#define RESET_PIN               15
#define FEED_HOLD_PORT          GPIOC
#define FEED_HOLD_PIN           13
#define CYCLE_START_PORT        GPIOC
#define CYCLE_START_PIN         10
#define CONTROL_INMODE          GPIO_BITBAND

#define AUXINPUT0_PORT          GPIOB
#define AUXINPUT0_PIN           13

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PORT        AUXINPUT0_PORT
#define SAFETY_DOOR_PIN         AUXINPUT0_PIN
#endif

#if MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PORT        AUXINPUT0_PORT
#define MOTOR_FAULT_PIN         AUXINPUT0_PIN
#endif

// Define probe switch input pin.
#if !N_AUTO_SQUARED
#define PROBE_PORT              GPIOB
#define PROBE_PIN               12
#endif

#if SDCARD_ENABLE
  #define SDCARD_SDIO           0
	#define SPI_PORT            1                                  // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
	#define SD_CS_PORT          GPIOA
	#define SD_CS_PIN           4
#endif

#if TRINAMIC_UART_ENABLE

#define MOTOR_UART_PORT         GPIOA
#define MOTOR_UART_RX_PIN       10
#define MOTOR_UART_TX_PIN       9

#undef TRINAMIC_UART_ENABLE
#define TRINAMIC_UART_ENABLE    2

#define MOTOR_UARTX_PORT        MOTOR_UART_PORT
#define MOTOR_UARTX_PIN         MOTOR_UART_TX_PIN
#define MOTOR_UARTY_PORT        MOTOR_UART_PORT
#define MOTOR_UARTY_PIN         MOTOR_UART_TX_PIN
#define MOTOR_UARTZ_PORT        MOTOR_UART_PORT
#define MOTOR_UARTZ_PIN         MOTOR_UART_TX_PIN

#ifdef  M3_AVAILABLE
#define MOTOR_UARTM3_PORT       MOTOR_UART_PORT
#define MOTOR_UARTM3_PIN        MOTOR_UART_TX_PIN
#endif

#ifdef  M4_AVAILABLE
#define MOTOR_UARTM4_PORT       MOTOR_UART_PORT
#define MOTOR_UARTM4_PIN        MOTOR_UART_TX_PIN
#endif

#endif

/* EOF */
