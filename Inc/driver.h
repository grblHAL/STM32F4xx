/*

  driver.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"
#include "grbl/crossbar.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

#define timer(p) timerN(p)
#define timerN(p) TIM ## p
#define timerINT(p) timeri(p)
#define timeri(p) TIM ## p ## _IRQn
#define timerHANDLER(p) timerh(p)
#define timerh(p) TIM ## p ## _IRQHandler

// Configuration
// Set value to 1 to enable, 0 to disable

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef ESTOP_ENABLE
#define ESTOP_ENABLE        0
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif
#ifndef ODOMETER_ENABLE
#define ODOMETER_ENABLE     0
#endif
#ifndef PPI_ENABLE
#define PPI_ENABLE       	0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE     0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
#endif

#define CNC_BOOSTERPACK     0

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

// Define timer allocations.

#define STEPPER_TIMER_N             5
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)

#define PULSE_TIMER_N               4
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_IRQn            timerINT(PULSE_TIMER_N)
#define PULSE_TIMER_IRQHandler      timerHANDLER(PULSE_TIMER_N)

#define SPINDLE_PWM_TIMER_N         1
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)

#define DEBOUNCE_TIMER_N            9
#define DEBOUNCE_TIMER              timer(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_IRQn         TIM1_BRK_TIM9_IRQn       // !
#define DEBOUNCE_TIMER_IRQHandler   TIM1_BRK_TIM9_IRQHandler // !

#define RPM_COUNTER_N               3
#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER_N                 2
#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#define PPI_TIMER_N                 2
#define PPI_TIMER                   timer(PPI_TIMER_N)
#define PPI_TIMER_IRQn              timerINT(PPI_TIMER_N)
#define PPI_TIMER_IRQHandler        timerHANDLER(PPI_TIMER_N)

#ifdef BOARD_CNC_BOOSTERPACK
  #if N_AXIS > 3
    #error Max number of axes is 3!
  #endif
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #if EEPROM_ENABLE
    #error EEPROM plugin not supported!
  #endif
  #include "cnc3040_map.h"
#elif defined(BOARD_SKR_PRO_1_1)
  #include "skr_pro_v1_1_map.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "protoneer_3.xx_map.h"
#elif defined(BOARD_GENERIC_UNO)
  #include "uno_map.h"
#elif defined(BOARD_MORPHO_CNC)
  #include "st_morpho_map.h"
#elif defined(BOARD_MORPHO_DAC_CNC)
  #include "st_morpho_dac_map.h"
#elif defined(BOARD_MINI_BLACKPILL)
  #include "mini_blackpill_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else // default board
  #include "generic_map.h"
#endif


#if defined(BOARD_PROTONEER_3XX) || defined(BOARD_GENERIC_UNO) || defined(BOARD_MORPHO_CNC) || defined(BOARD_MORPHO_DAC_CNC)
#if USB_SERIAL_CDC
#error "Nucleo based boards does not support USB CDC communication!"
#endif
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if SPINDLE_HUANYANG
#include "spindle/huanyang.h"
#endif

#ifndef VFD_SPINDLE
#define VFD_SPINDLE 0
#endif

#ifdef MODBUS_ENABLE
#define SERIAL2_MOD
#endif

#ifndef SPI_PORT
#define SPI_PORT 1
#endif

#if EEPROM_ENABLE|| KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
  #if defined(NUCLEO_F411) || defined(NUCLEO_F446) || defined(BOARD_SKR_PRO_1_1)
    #define I2C_PORT 1 // GPIOB, SCL_PIN = 8, SDA_PIN = 9
  #else
    #define I2C_PORT 2 // GPIOB, SCL_PIN = 10, SDA_PIN = 11
  #endif
#endif

#if TRINAMIC_ENABLE
  #include "motors/trinamic.h"
  #ifndef TRINAMIC_MIXED_DRIVERS
    #define TRINAMIC_MIXED_DRIVERS 1
  #endif
  #if TRINAMIC_ENABLE == 2209
    #ifdef MODBUS_ENABLE
      #error "Cannot use TMC2209 drivers with Modbus spindle!"
    #else
      #define SERIAL2_MOD
    #endif
  #endif
#endif

// End configuration

#if KEYPAD_ENABLE && !defined(KEYPAD_PORT)
#error Keypad plugin not supported!
#endif

#if SDCARD_ENABLE && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif

#ifndef X_STEP_PORT
#define X_STEP_PORT STEP_PORT
#endif
#ifndef Y_STEP_PORT
#define Y_STEP_PORT STEP_PORT
#endif
#ifndef Z_STEP_PORT
#define Z_STEP_PORT STEP_PORT
#endif
#ifndef A_STEP_PORT
#define A_STEP_PORT STEP_PORT
#endif
#ifndef B_STEP_PORT
#define B_STEP_PORT STEP_PORT
#endif
#ifndef C_STEP_PORT
#define C_STEP_PORT STEP_PORT
#endif

#ifndef X_DIRECTION_PORT
#define X_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Y_DIRECTION_PORT
#define Y_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Z_DIRECTION_PORT
#define Z_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef A_DIRECTION_PORT
#define A_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef B_DIRECTION_PORT
#define B_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef C_DIRECTION_PORT
#define C_DIRECTION_PORT DIRECTION_PORT
#endif

#ifndef X_LIMIT_PORT
#define X_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Y_LIMIT_PORT
#define Y_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Z_LIMIT_PORT
#define Z_LIMIT_PORT LIMIT_PORT
#endif
#ifndef A_LIMIT_PORT
#define A_LIMIT_PORT LIMIT_PORT
#endif
#ifndef B_LIMIT_PORT
#define B_LIMIT_PORT LIMIT_PORT
#endif
#ifndef C_LIMIT_PORT
#define C_LIMIT_PORT LIMIT_PORT
#endif

#ifndef RESET_PORT
#define RESET_PORT CONTROL_PORT
#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT CONTROL_PORT
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT CONTROL_PORT
#endif
#if SAFETY_DOOR_ENABLE && !defined(SAFETY_DOOR_PORT)
#define SAFETY_DOOR_PORT CONTROL_PORT
#endif

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
} input_signal_t;

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

bool driver_init (void);
void Driver_IncTick (void);
#ifdef HAS_BOARD_INIT
void board_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);

#endif

#endif // __DRIVER_H__
