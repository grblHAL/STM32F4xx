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

#define ESTOP_ENABLE 0

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "grbl/driver_opts.h"

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

#define DIGITAL_IN(port, pin) BITBAND_PERI(port->IDR, pin)
#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI(port->ODR, pin) = on; }

#define timer(p) timerN(p)
#define timerN(p) TIM ## p
#define timerINT(p) timeri(p)
#define timeri(p) TIM ## p ## _IRQn
#define timerHANDLER(p) timerh(p)
#define timerh(p) TIM ## p ## _IRQHandler

// Configuration, do not change here

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
#elif defined(BOARD_BLACKPILL)
  #include "blackpill_map.h"
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

#if (defined(NUCLEO_F411) || defined(NUCLEO_F446)) && !defined(IS_NUCLEO_BOB)
#warning "Board map is not for Nucleo based boards and firmware may not work!"
#endif

#if defined(IS_NUCLEO_BOB) && USB_SERIAL_CDC
#error "Nucleo based boards does not support USB CDC communication!"
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

#if MODBUS_ENABLE || BLUETOOTH_ENABLE
#define SERIAL2_MOD
#endif

#if TRINAMIC_ENABLE
  #include "motors/trinamic.h"
  #ifndef TRINAMIC_MIXED_DRIVERS
    #define TRINAMIC_MIXED_DRIVERS 1
  #endif
  #if TRINAMIC_ENABLE == 2209
    #if MODBUS_ENABLE
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

#ifndef I2C_PORT
#define I2C_PORT 2 // GPIOB, SCL_PIN = 10, SDA_PIN = 11
#endif

#ifndef SPI_PORT
#define SPI_PORT 1
#endif

#ifndef STEP_PINMODE
#define STEP_PINMODE PINMODE_OUTPUT
#endif

#ifndef DIRECTION_PINMODE
#define DIRECTION_PINMODE PINMODE_OUTPUT
#endif

#ifndef STEPPERS_ENABLE_PINMODE
#define STEPPERS_ENABLE_PINMODE PINMODE_OUTPUT
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

#ifndef AUXINPUT_MASK
#define AUXINPUT_MASK 0
#endif

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_irq_mode_t irq_mode;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
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
void board_init (void);
#endif
#ifdef HAS_IOPORTS
void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (uint32_t bit);
#endif

#endif // __DRIVER_H__
