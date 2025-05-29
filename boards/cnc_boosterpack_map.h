/*
  cnc_boosterpack_map.h - driver code for STM32F4xx processor (on Blackpill board)

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

#if N_ABC_MOTORS > 1
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "CNC BoosterPack"

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif

#ifndef WEB_BUILD
#undef I2C_ENABLE
#undef EEPROM_ENABLE
#define CNC_BOOSTERPACK 1
#define I2C_ENABLE      0
#define EEPROM_ENABLE   0 // Disabled for now for BlackPill - pin differences... // Only change if BoosterPack does not have EEPROM mounted
#endif
#define SERIAL_PORT     1   // GPIOA: TX = 9, RX = 10

// Define step pulse output pins.
#define STEP_PORT               GPIOA
#define X_STEP_PIN              0
#define Y_STEP_PIN              1
#define Z_STEP_PIN              2
#define STEP_OUTMODE            GPIO_MAP
//#define STEP_PINMODE            PINMODE_OD // Uncomment for open drain outputs

// Define step direction output pins.
#define DIRECTION_PORT          GPIOA
#define X_DIRECTION_PIN         4
#define Y_DIRECTION_PIN         5
#define Z_DIRECTION_PIN         6
#define DIRECTION_OUTMODE       GPIO_MAP
//#define DIRECTION_PINMODE       PINMODE_OD // Uncomment for open drain outputs

// Define stepper driver enable/disable output pin.
#define STEPPERS_ENABLE_PORT    GPIOA
#define STEPPERS_ENABLE_PIN     15
#define STEPPERS_ENABLE_MASK    STEPPERS_ENABLE_BIT
//#define STEPPERS_ENABLE_PINMODE PINMODE_OD // Uncomment for open drain outputs

// Define homing/hard limit switch input pins.
#define LIMIT_PORT              GPIOB
#define X_LIMIT_PIN             12
#define Y_LIMIT_PIN             13
#define Z_LIMIT_PIN             14
#define LIMIT_INMODE            GPIO_SHIFT12

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 1
#define M3_AVAILABLE
#define M3_STEP_PORT            STEP_PORT
#define M3_STEP_PIN             3
#define M3_DIRECTION_PORT       DIRECTION_PORT
#define M3_DIRECTION_PIN        7
#if N_AUTO_SQUARED
#define M3_LIMIT_PORT           LIMIT_PORT
#define M3_LIMIT_PIN            15
#endif
#endif

#define AUXOUTPUT0_PORT         GPIOA // Spindle PWM
#define AUXOUTPUT0_PIN          8
#define AUXOUTPUT1_PORT         GPIOB // Spindle direction
#define AUXOUTPUT1_PIN          0
#define AUXOUTPUT2_PORT         GPIOB // Spindle enable
#define AUXOUTPUT2_PIN          1
#define AUXOUTPUT3_PORT         GPIOC // Coolant flood
#define AUXOUTPUT3_PIN          15
#define AUXOUTPUT4_PORT         GPIOC // Coolant mist
#define AUXOUTPUT4_PIN          14

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

#define AUXINPUT0_PORT          GPIOB // Safety door
#define AUXINPUT0_PIN           9
#define AUXINPUT1_PORT          GPIOA // Probe
#define AUXINPUT1_PIN           7
#define AUXINPUT2_PORT          GPIOB
#define AUXINPUT2_PIN           15
#define AUXINPUT3_PORT          GPIOB // Reset/EStop
#define AUXINPUT3_PIN           6
#define AUXINPUT4_PORT          GPIOB // Feed hold
#define AUXINPUT4_PIN           7
#define AUXINPUT5_PORT          GPIOB // Cycle start
#define AUXINPUT5_PIN           8

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PORT              AUXINPUT3_PORT
#define RESET_PIN               AUXINPUT3_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PORT          AUXINPUT4_PORT
#define FEED_HOLD_PIN           AUXINPUT4_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PORT        AUXINPUT5_PORT
#define CYCLE_START_PIN         AUXINPUT5_PIN
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

#if I2C_STROBE_ENABLE
#define I2C_STROBE_POR          AUXINPUT2_PORT
#define I2C_STROBE_PIN          AUXINPUT2_PIN
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT              GPIOA
#define SD_CS_PIN               3
#endif

/* EOF */
