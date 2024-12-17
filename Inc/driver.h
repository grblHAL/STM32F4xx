/*

  driver.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"
#include "pwm.h"

#if defined(_WIZCHIP_) && _WIZCHIP_ > 0
#undef ETHERNET_ENABLE
#define ETHERNET_ENABLE 1
#endif

#if defined(MCP3221_ENABLE)
#define I2C_ENABLE 1
#endif

#define OPTS_POSTPROCESSING

#include "grbl/driver_opts.h"

#include "timers.h"

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

#define DIGITAL_IN(port, pin) BITBAND_PERI((port)->IDR, pin)
#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI((port)->ODR, pin) = on; }

#define timer(t) timerN(t)
#define timerN(t) TIM ## t
#define timerBase(t) timerbase(t)
#define timerbase(t) TIM ## t ## _BASE
#define timerINT(t) timerint(t)
#define timerint(t) TIM ## t ## _IRQn
#define timerHANDLER(t) timerhandler(t)
#define timerCLKEN(t) timerclken(t)
#define timerclken(t) __HAL_RCC_TIM ## t ## _CLK_ENABLE
#define timerhandler(t) TIM ## t ## _IRQHandler
#define timerCCEN(c, n) timerccen(c, n)
#define timerccen(c, n) TIM_CCER_CC ## c ## n ## E
#define timerCCMR(p, c) timerccmr(p, c)
#define timerccmr(p, c) TIM ## p->CCMR ## c
#define timerOCM(p, c) timerocm(p, c)
#define timerocm(p, c) TIM_CCMR ## p ##_OC ## c ## M_1|TIM_CCMR ## p ##_OC ## c ## M_2
#define timerOCMC(p, c) timerocmc(p, c)
#define timerocmc(p, c) (TIM_CCMR ## p ##_OC ## c ## M|TIM_CCMR ## p ##_CC ## c ## S)
#define timerCCR(t, c) timerccr(t, c)
#define timerccr(t, c) TIM ## t->CCR ## c
#define timerCCP(c, n) timerccp(c, n)
#define timerccp(c, n) TIM_CCER_CC ## c ## n ## P
#define timerCR2OIS(c, n) timercr2ois(c, n)
#define timercr2ois(c, n) TIM_CR2_OIS ## c ## n
#define timerAF(t, f) timeraf(t, f)
#define timeraf(t, f) GPIO_AF ## f ## _TIM ## t
#define timerAPB2(t) (t == 1 || t == 8 || t == 9 || t == 10 || t == 11)

#define usart(t) usartN(t)
#define usartN(t) USART ## t
#define usartINT(t) usartint(t)
#define usartint(t) USART ## t ## _IRQn
#define usartHANDLER(t) usarthandler(t)
#define usarthandler(t) USART ## t ## _IRQHandler
#define usartCLKEN(t) usartclken(t)
#define usartclken(t) __HAL_RCC_USART ## t ## _CLK_ENABLE
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

#ifndef IS_NUCLEO_DEVKIT
#if defined(NUCLEO_F401) || defined(NUCLEO_F411) || defined(NUCLEO_F446)
#define IS_NUCLEO_DEVKIT 64
#elif defined(NUCLEO144_F446)
#define IS_NUCLEO_DEVKIT 144
#else
#define IS_NUCLEO_DEVKIT 0
#endif
#endif

#ifdef BOARD_BTT_SKR_PRO_1_2
#define BOARD_BTT_SKR_PRO_1_1
#endif

#ifdef BOARD_CNC_BOOSTERPACK
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_BLACKPILL)
  #include "boards/blackpill_map.h"
#elif defined(BOARD_BLACKPILL_ALT2)
  #include "boards/blackpill_alt2_map.h"
#elif defined(BOARD_DEVTRONIC_CNC_V2)
  #include "boards/Devtronic_CNC_Controller_V2_map.h"
#elif defined(BOARD_DEVTRONIC_CNC)
  #include "boards/Devtronic_CNC_Controller_map.h"
#elif defined(BOARD_BTT_SKR_PRO_1_1)
  #include "boards/btt_skr_pro_v1_1_map.h"
#elif defined(BOARD_BTT_SKR_20)
  #include "boards/btt_skr_2.0_map.h"
#elif defined(BOARD_BTT_SKR_20_DAC)
  #include "boards/btt_skr_2.0_dac_map.h"
#elif defined(BOARD_FYSETC_S6)
  #include "boards/fysetc_s6_map.h"
#elif defined(BOARD_PROTONEER_3XX)
  #include "boards/protoneer_3.xx_map.h"
#elif defined(BOARD_GENERIC_UNO)
  #include "boards/uno_map.h"
#elif defined(BOARD_MORPHO_CNC)
  #include "boards/st_morpho_map.h"
#elif defined(BOARD_MORPHO_DAC_CNC)
  #include "boards/st_morpho_dac_map.h"
#elif defined(BOARD_MINI_BLACKPILL)
  #include "boards/mini_blackpill_map.h"
#elif defined(BOARD_FLEXI_HAL)
  #include "boards/flexi_hal_map.h"
#elif defined(BOARD_STM32F401_UNI)
  #include "boards/stm32f401_uni_map.h"
#elif defined(BOARD_HALCYON_V1)
  #include "boards/halcyon_v1_map.h"
#elif defined(BOARD_MKS_ROBIN_NANO_30)
  #include "boards/mks_robin_nano_v3.0_map.h"
#elif defined(BOARD_LONGBOARD32)
  #include "boards/longboard32_map.h"
#elif defined(BOARD_MKS_EAGLE)
  #include "boards/mks_eagle_map.h"
#elif defined(BOARD_BTT_OCTOPUS_PRO)
  #include "boards/btt_octopus_pro_map.h"
#elif defined(BOARD_STM32F407VET6_DEV)
  #include "boards/stm32f407vet6_dev_board.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#else // default board
  #include "boards/generic_map.h"
#endif

#if IS_NUCLEO_DEVKIT == 64 && !defined(IS_NUCLEO_BOB)
#warning "Board map is not for Nucleo based boards and firmware may not work!"
#endif

#if IS_NUCLEO_DEVKIT == 64 && USB_SERIAL_CDC
#error "Nucleo64 based boards does not support USB CDC communication!"
#endif

#if ETHERNET_ENABLE && !defined(SPI_IRQ_PIN)
#error "Board does not support ethernet!"
#endif

// Define timer allocations.

#define STEPPER_TIMER_N             5
#define STEPPER_TIMER_BASE          timerBase(STEPPER_TIMER_N)
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_CLKEN         timerCLKEN(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)

#define PULSE_TIMER_N               4
#define PULSE_TIMER_BASE            timerBase(PULSE_TIMER_N)
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_CLKEN           timerCLKEN(PULSE_TIMER_N)
#define PULSE_TIMER_IRQn            timerINT(PULSE_TIMER_N)
#define PULSE_TIMER_IRQHandler      timerHANDLER(PULSE_TIMER_N)

#if SPINDLE_ENCODER_ENABLE

#ifndef RPM_COUNTER_N
#define RPM_COUNTER_N   3
#endif
#ifndef RPM_TIMER_N
#define RPM_TIMER_N     2
#endif

#if PULSE2_TIMER_N == RPM_COUNTER_N || PULSE2_TIMER_N == RPM_TIMER_N
#error Timer conflict: spindle sync and step inject!
#endif
#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_BASE            timerBase(RPM_COUNTER_N)
#define RPM_COUNTER_CLKEN           timerCLKEN(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_BASE              timerBase(RPM_TIMER_N)
#define RPM_TIMER_CLKEN             timerCLKEN(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#endif //  SPINDLE_ENCODER_ENABLE

#if TRINAMIC_UART_ENABLE == 2
#ifndef TMC_UART_TIMER_N
#define TMC_UART_TIMER_N            7
#endif
#define TMC_UART_TIMER_BASE         timerBase(TMC_UART_TIMER_N)
#endif

#define IS_TIMER_CLAIMED(INSTANCE) (((INSTANCE) == STEPPER_TIMER_BASE) || \
                                    ((INSTANCE) == PULSE_TIMER_BASE) || \
                                    ((INSTANCE) == RPM_TIMER_BASE) || \
                                    ((INSTANCE) == RPM_COUNTER_BASE) || \
                                    ((INSTANCE) == TMC_UART_TIMER_BASE))

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#include "grbl/driver_opts2.h"

#if SDCARD_ENABLE
#ifndef SDCARD_SDIO
#define SDCARD_SDIO 0
#endif
#if !SDCARD_SDIO && !defined(SD_CS_PORT)
#error SD card plugin not supported!
#endif
#endif

#if I2C_ENABLE && !defined(I2C_PORT)
#define I2C_PORT 2 // GPIOB, SCL_PIN = 10, SDA_PIN = 11
#endif

#if SPI_ENABLE && !defined(SPI_PORT)
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

#if defined(AUXOUTPUT0_PWM_PORT) || defined(AUXOUTPUT1_PWM_PORT) ||\
     defined(AUXOUTPUT0_ANALOG_PORT) || defined(AUXOUTPUT1_ANALOG_PORT) ||\
      defined(AUXINTPUT0_ANALOG_PORT) || defined(AUXINTPUT1_ANALOG_PORT) ||\
       defined(MCP3221_ENABLE)
#define AUX_ANALOG 1
#else
#define AUX_ANALOG 0
#endif

typedef struct {
    pin_function_t id;
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint32_t bit;
    GPIO_TypeDef *port;
    pin_group_t group;
    uint8_t user_port;
    volatile bool active;
    ioport_interrupt_callback_ptr interrupt_callback;
    ADC_HandleTypeDef *adc;
    const char *description;
} input_signal_t;

typedef struct {
    float value;
    ioports_pwm_t data;
    const pwm_signal_t *port;
} pwm_out_t;

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
    pwm_out_t *pwm;
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
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);
#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#if AUX_ANALOG
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#endif
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
