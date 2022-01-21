/*
  my_machine_map.h - driver code for F4MPV5 (STM32F407) board

  Part of GrblHAL

  Copyright (c) 2021 atlesg

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

/* Default Pin Assignments:
 * A0  X Step       | B0  Step En/Dis       | C0  (N/A)
 * A1  X Direction  | B1  Spindle Enable    | C1  (N/A)
 * A2  Y Step       | B2  Spindle Direction | C2  (N/A)
 * A3  Y Direction  | B3                    | C3  (N/A)
 * A4  Z Step       | B4                    | C4  (N/A)
 * A5  Z Direction  | B5                    | C5  (N/A)
 * A6  A Step       | B6  Reset             | C6  (N/A)
 * A7  A Direction  | B7  Feed Hold         | C7  (N/A)
 * A8  Spindle PWM  | B8  Cycle Start       | C8  (N/A)
 * A9  Y2 Step      | B9  Door Safety       | C9  (N/A)
 * A10 Y2 Direction | B10 Y2 Limit          | C10 (N/A)
 * A11              | B11                   | C11 (N/A)
 * A12              | B12 X Limit           | C12 (N/A)
 * A13              | B13 Y Limit           | C13
 * A14              | B14 Z Limit           | C14 Coolant Flood
 * A15              | B15 Probe             | C15 coolant Mist
 */



#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#if !defined(STM32F407xx) || HSE_VALUE != 8000000
#error "This board has STM32F407 processor with a 8MHz crystal, select a corresponding build!"
#endif

#define BOARD_NAME "F4MPV5_HOSOI"
#define HAS_BOARD_INIT



// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              2 			// JM1
#define Y_STEP_PIN              4				// JM2
#define Z_STEP_PIN              6				// JM3
#define STEP_OUTMODE            GPIO_MAP


#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         3				// JM1
#define Y_DIRECTION_PIN         5				// JM2
#define Z_DIRECTION_PIN         7				// JM3
#define DIRECTION_OUTMODE       GPIO_MAP

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOA		// JM4-PU
#define STEPPERS_ENABLE_PIN     6
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOD
#define X_LIMIT_PIN             13				// N12
#define Y_LIMIT_PIN             14				// N13
#define Z_LIMIT_PIN             15			// N14
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             6
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            15
#endif
#endif

  // Define spindle enable and spindle direction output pins.
//#define SPINDLE_ENABLE_PORT     GPIOB
//#define SPINDLE_ENABLE_PIN      0				// JM5 - PU
//#define SPINDLE_DIRECTION_PORT  GPIOB
//#define SPINDLE_DIRECTION_PIN   1				// JM5 - DR

#define SPINDLE_CW_PORT					GPIOE
#define SPINDLE_CW_PIN					2
#define SPINDLE_CCW_PORT				GPIOE
#define SPINDLE_CCW_PIN					3

// Define spindle PWM output pin.
//#define SPINDLE_PWM_PORT_BASE   GPIOA_BASE
//#define SPINDLE_PWM_PIN         8

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIOE
#define COOLANT_FLOOD_PIN       1
//#define COOLANT_MIST_PORT       GPIOB
//#define COOLANT_MIST_PIN        4

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOD
#define RESET_PIN               0				// N0
#define FEED_HOLD_PIN           1				// N1
#define CYCLE_START_PIN         2				// N2
#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN         1
#endif
#define CONTROL_INMODE          GPIO_SHIFT6

// Define probe switch input pin.
#define PROBE_PORT              GPIOD
#define PROBE_PIN               12			// N15

// Define main grbl uart port
#define UART_INSTANCE						2

// Define Spindle port and pins
#define MODBUS_SERIAL_PORT			3
//#define MODBUS_DIR_AUX					5

// Define I2C Port - avoid confliction with USART3 port
#undef I2C_PORT

/* EOF */
