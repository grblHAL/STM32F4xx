/*

  driver.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2024 Terje Io

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

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "main.h"

#if defined(_WIZCHIP_) && _WIZCHIP_ > 0
#undef ETHERNET_ENABLE
#define ETHERNET_ENABLE 1
#endif

#if defined(MCP3221_ENABLE)
#define I2C_ENABLE 1
#endif

#include "grbl/driver_opts.h"

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

#define DIGITAL_IN(port, pin) BITBAND_PERI((port)->IDR, pin)
#define DIGITAL_OUT(port, pin, on) { BITBAND_PERI((port)->ODR, pin) = on; }

#define timer(t) timerN(t)
#define timerN(t) TIM ## t
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
#define TIMER_CLOCK_MUL(d) (d == RCC_HCLK_DIV1 ? 1 : 2)

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
  #if N_AXIS > 3
    #error Max number of axes is 3!
  #endif
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_CNC3040)
  #if EEPROM_ENABLE
    #error EEPROM plugin not supported!
  #endif
  #include "boards/cnc3040_map.h"
#elif defined(BOARD_BLACKPILL)
  #include "boards/blackpill_map.h"
#elif defined(BOARD_BLACKPILL_ALT2)
  #include "boards/blackpill_alt2_map.h"
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
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#else // default board
  #include "boards/generic_map.h"
#endif

#if DRIVER_SPINDLE_ENABLE && !defined(SPINDLE_ENABLE_PIN)
#warning "Selected spindle is not supported!"
#undef DRIVER_SPINDLE_ENABLE
#define DRIVER_SPINDLE_ENABLE 0
#endif

#if DRIVER_SPINDLE_DIR_ENABLE && !defined(SPINDLE_DIRECTION_PIN)
#warning "Selected spindle is not fully supported - no direction output!"
#undef DRIVER_SPINDLE_DIR_ENABLE
#define DRIVER_SPINDLE_DIR_ENABLE 0
#endif

#if DRIVER_SPINDLE_PWM_ENABLE && (!DRIVER_SPINDLE_ENABLE || !defined(SPINDLE_PWM_PIN))
#warning "Selected spindle is not supported!"
#undef DRIVER_SPINDLE_PWM_ENABLE
#define DRIVER_SPINDLE_PWM_ENABLE 0
#ifdef SPINDLE_PWM_PORT_BASE
#undef SPINDLE_PWM_PORT_BASE
#endif
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
#define STEPPER_TIMER               timer(STEPPER_TIMER_N)
#define STEPPER_TIMER_CLKEN         timerCLKEN(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQn          timerINT(STEPPER_TIMER_N)
#define STEPPER_TIMER_IRQHandler    timerHANDLER(STEPPER_TIMER_N)

#define PULSE_TIMER_N               4
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_CLKEN           timerCLKEN(PULSE_TIMER_N)
#define PULSE_TIMER_IRQn            timerINT(PULSE_TIMER_N)
#define PULSE_TIMER_IRQHandler      timerHANDLER(PULSE_TIMER_N)

#if defined(STM32F407xx) || defined(STM32F429xx) || defined(STM32F446xx)
#define PULSE2_TIMER_N              7
#define PULSE2_TIMER                timer(PULSE2_TIMER_N)
#define PULSE2_TIMER_CLKEN          timerCLKEN(PULSE2_TIMER_N)
#define PULSE2_TIMER_IRQn           timerINT(PULSE2_TIMER_N)
#define PULSE2_TIMER_IRQHandler     timerHANDLER(PULSE2_TIMER_N)
#endif

#ifdef SPINDLE_PWM_PORT_BASE

#if SPINDLE_PWM_PORT_BASE == GPIOA_BASE
  #if SPINDLE_PWM_PIN == 0 // PA0 - TIM2_CH1
    #define SPINDLE_PWM_TIMER_N     2
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 5 // PA5 - TIM2_CH1
    #define SPINDLE_PWM_TIMER_N     2
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 7 // PA7 - TIM1_CH1N
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   1
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 8 // PA8 - TIM1_CH1
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 11 // PA11 - TIM1_CH4
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    4
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    1
  #endif
#elif SPINDLE_PWM_PORT_BASE == GPIOB_BASE
  #if SPINDLE_PWM_PIN == 0 // PB0 - TIM1_CH2N
    #define SPINDLE_PWM_TIMER_N     1
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   1
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 3 // PB3 - TIM2_CH2
    #define SPINDLE_PWM_TIMER_N     2
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    1
  #elif SPINDLE_PWM_PIN == 4 // PB4 - TIM3_CH1
    #define SPINDLE_PWM_TIMER_N     3
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    2
  #elif SPINDLE_PWM_PIN == 9 // PB9 - TIM11_CH1
    #define SPINDLE_PWM_TIMER_N     11
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    3
  #endif
#elif SPINDLE_PWM_PORT_BASE == GPIOE_BASE
  #if SPINDLE_PWM_PIN == 5 // PE5 - TIM9_CH1
    #define SPINDLE_PWM_TIMER_N     9
    #define SPINDLE_PWM_TIMER_CH    1
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    3
  #elif SPINDLE_PWM_PIN == 6 // PE6 - TIM9_CH2
    #define SPINDLE_PWM_TIMER_N     9
    #define SPINDLE_PWM_TIMER_CH    2
    #define SPINDLE_PWM_TIMER_INV   0
    #define SPINDLE_PWM_TIMER_AF    3
  #endif
#endif

#if SPINDLE_PWM_TIMER_CH == 1 || SPINDLE_PWM_TIMER_CH == 2
#define SPINDLE_PWM_CCR 1
#else
#define SPINDLE_PWM_CCR 2
#endif
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_CLKEN     timerCLKEN(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_CCR       timerCCR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_TIMER_CCMR      timerCCMR(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_CCR)
#define SPINDLE_PWM_CCMR_OCM_SET    timerOCM(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#define SPINDLE_PWM_CCMR_OCM_CLR    timerOCMC(SPINDLE_PWM_CCR, SPINDLE_PWM_TIMER_CH)
#if SPINDLE_PWM_TIMER_INV
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, N)
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, N)
#else
#define SPINDLE_PWM_CCER_EN         timerCCEN(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CCER_POL        timerCCP(SPINDLE_PWM_TIMER_CH, )
#define SPINDLE_PWM_CR2_OIS         timerCR2OIS(SPINDLE_PWM_TIMER_CH, )
#endif

#define SPINDLE_PWM_PORT            ((GPIO_TypeDef *)SPINDLE_PWM_PORT_BASE)
#define SPINDLE_PWM_AF              timerAF(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_AF)
#define SPINDLE_PWM_CLKEN           timerCLKEN(SPINDLE_PWM_TIMER_N)

#endif // SPINDLE_PWM_PORT_BASE

#if defined(SPINDLE_PWM_PIN) && !defined(SPINDLE_PWM_TIMER_N)
#ifdef SPINDLE_PWM_PORT
#error Map spindle port by defining SPINDLE_PWM_PORT_BASE in the map file!
#else
#error Spindle PWM not supported on mapped pin!
#endif
#endif

#ifdef AUXOUTPUT0_PWM_PORT_BASE

#if AUXOUTPUT0_PWM_PORT_BASE == GPIOA_BASE
  #if AUXOUTPUT0_PWM_PIN == 5 // PA5 - TIM2_CH1
    #define AUXOUTPUT0_PWM_TIMER_N     2
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #elif AUXOUTPUT0_PWM_PIN == 7 // PA7 - TIM1_CH1N
    #define AUXOUTPUT0_PWM_TIMER_N     1
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   1
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #elif AUXOUTPUT0_PWM_PIN == 8 // PA8 - TIM1_CH1
    #define AUXOUTPUT0_PWM_TIMER_N     1
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #endif
#elif AUXOUTPUT0_PWM_PORT_BASE == GPIOB_BASE
  #if AUXOUTPUT0_PWM_PIN == 0 // PB0 - TIM1_CH2N
    #define AUXOUTPUT0_PWM_TIMER_N     1
    #define AUXOUTPUT0_PWM_TIMER_CH    2
    #define AUXOUTPUT0_PWM_TIMER_INV   1
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #elif AUXOUTPUT0_PWM_PIN == 2 // PB2 - TIM2_CH4
    #define AUXOUTPUT0_PWM_TIMER_N     2
    #define AUXOUTPUT0_PWM_TIMER_CH    4
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #elif AUXOUTPUT0_PWM_PIN == 3 // PB3 - TIM2_CH2
    #define AUXOUTPUT0_PWM_TIMER_N     2
    #define AUXOUTPUT0_PWM_TIMER_CH    2
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    1
  #elif AUXOUTPUT0_PWM_PIN == 4 // PB4 - TIM3_CH1
    #define AUXOUTPUT0_PWM_TIMER_N     3
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    2
  #elif AUXOUTPUT0_PWM_PIN == 9 // PB9 - TIM11_CH1
    #define AUXOUTPUT0_PWM_TIMER_N     11
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    3
  #endif
#elif AUXOUTPUT0_PWM_PORT_BASE == GPIOC_BASE
  #if AUXOUTPUT0_PWM_PIN == 8 // PC8 - TIM3_CH3
    #define AUXOUTPUT0_PWM_TIMER_N     3
    #define AUXOUTPUT0_PWM_TIMER_CH    3
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    2
  #endif
#elif AUXOUTPUT0_PWM_PORT_BASE == GPIOE_BASE
  #if AUXOUTPUT0_PWM_PIN == 5 // PE5 - TIM9_CH1
    #define AUXOUTPUT0_PWM_TIMER_N     9
    #define AUXOUTPUT0_PWM_TIMER_CH    1
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    3
  #elif AUXOUTPUT0_PWM_PIN == 6 // PE6 - TIM9_CH2
    #define AUXOUTPUT0_PWM_TIMER_N     9
    #define AUXOUTPUT0_PWM_TIMER_CH    2
    #define AUXOUTPUT0_PWM_TIMER_INV   0
    #define AUXOUTPUT0_PWM_TIMER_AF    3
  #endif
#endif

#if AUXOUTPUT0_PWM_TIMER_CH == 1 || AUXOUTPUT0_PWM_TIMER_CH == 2
#define AUXOUTPUT0_PWM_CCR 1
#else
#define AUXOUTPUT0_PWM_CCR 2
#endif
#define AUXOUTPUT0_PWM_TIMER           timer(AUXOUTPUT0_PWM_TIMER_N)
#define AUXOUTPUT0_PWM_TIMER_CCR       timerCCR(AUXOUTPUT0_PWM_TIMER_N, AUXOUTPUT0_PWM_TIMER_CH)
#define AUXOUTPUT0_PWM_TIMER_CCMR      timerCCMR(AUXOUTPUT0_PWM_TIMER_N, AUXOUTPUT0_PWM_CCR)
#define AUXOUTPUT0_PWM_CCMR_OCM_SET    timerOCM(AUXOUTPUT0_PWM_CCR, AUXOUTPUT0_PWM_TIMER_CH)
#define AUXOUTPUT0_PWM_CCMR_OCM_CLR    timerOCMC(AUXOUTPUT0_PWM_CCR, AUXOUTPUT0_PWM_TIMER_CH)
#if AUXOUTPUT0_PWM_TIMER_INV
#define AUXOUTPUT0_PWM_CCER_EN         timerCCEN(AUXOUTPUT0_PWM_TIMER_CH, N)
#define AUXOUTPUT0_PWM_CCER_POL        timerCCP(AUXOUTPUT0_PWM_TIMER_CH, N)
#define AUXOUTPUT0_PWM_CR2_OIS         timerCR2OIS(AUXOUTPUT0_PWM_TIMER_CH, N)
#else
#define AUXOUTPUT0_PWM_CCER_EN         timerCCEN(AUXOUTPUT0_PWM_TIMER_CH, )
#define AUXOUTPUT0_PWM_CCER_POL        timerCCP(AUXOUTPUT0_PWM_TIMER_CH, )
#define AUXOUTPUT0_PWM_CR2_OIS         timerCR2OIS(AUXOUTPUT0_PWM_TIMER_CH, )
#endif

#define AUXOUTPUT0_PWM_PORT            ((GPIO_TypeDef *)AUXOUTPUT0_PWM_PORT_BASE)
#define AUXOUTPUT0_PWM_AF              timerAF(AUXOUTPUT0_PWM_TIMER_N, AUXOUTPUT0_PWM_TIMER_AF)
#define AUXOUTPUT0_PWM_CLKEN           timerCLKEN(AUXOUTPUT0_PWM_TIMER_N)

#endif // AUXOUTPUT0_PWM_PORT_BASE

#ifdef AUXOUTPUT1_PWM_PORT_BASE

#if AUXOUTPUT1_PWM_PORT_BASE == GPIOA_BASE
  #if AUXOUTPUT1_PWM_PIN == 5 // PA5 - TIM2_CH1
    #define AUXOUTPUT1_PWM_TIMER_N     2
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #elif AUXOUTPUT1_PWM_PIN == 7 // PA7 - TIM1_CH1N
    #define AUXOUTPUT1_PWM_TIMER_N     1
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   1
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #elif AUXOUTPUT1_PWM_PIN == 8 // PA8 - TIM1_CH1
    #define AUXOUTPUT1_PWM_TIMER_N     1
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #endif
#elif AUXOUTPUT1_PWM_PORT_BASE == GPIOB_BASE
  #if AUXOUTPUT1_PWM_PIN == 0 // PB0 - TIM1_CH2N
    #define AUXOUTPUT1_PWM_TIMER_N     1
    #define AUXOUTPUT1_PWM_TIMER_CH    2
    #define AUXOUTPUT1_PWM_TIMER_INV   1
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #elif AUXOUTPUT1_PWM_PIN == 2 // PB2 - TIM2_CH4
    #define AUXOUTPUT1_PWM_TIMER_N     2
    #define AUXOUTPUT1_PWM_TIMER_CH    4
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #elif AUXOUTPUT1_PWM_PIN == 3 // PB3 - TIM2_CH2
    #define AUXOUTPUT1_PWM_TIMER_N     2
    #define AUXOUTPUT1_PWM_TIMER_CH    2
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    1
  #elif AUXOUTPUT1_PWM_PIN == 4 // PB4 - TIM3_CH1
    #define AUXOUTPUT1_PWM_TIMER_N     3
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    2
  #elif AUXOUTPUT1_PWM_PIN == 9 // PB9 - TIM11_CH1
    #define AUXOUTPUT1_PWM_TIMER_N     11
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    3
  #endif
#elif AUXOUTPUT1_PWM_PORT_BASE == GPIOC_BASE
  #if AUXOUTPUT1_PWM_PIN == 8 // PC8 - TIM3_CH3
    #define AUXOUTPUT1_PWM_TIMER_N     3
    #define AUXOUTPUT1_PWM_TIMER_CH    3
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    2
  #endif
#elif AUXOUTPUT1_PWM_PORT_BASE == GPIOE_BASE
  #if AUXOUTPUT1_PWM_PIN == 5 // PE5 - TIM9_CH1
    #define AUXOUTPUT1_PWM_TIMER_N     9
    #define AUXOUTPUT1_PWM_TIMER_CH    1
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    3
  #elif AUXOUTPUT1_PWM_PIN == 6 // PE6 - TIM9_CH2
    #define AUXOUTPUT1_PWM_TIMER_N     9
    #define AUXOUTPUT1_PWM_TIMER_CH    2
    #define AUXOUTPUT1_PWM_TIMER_INV   0
    #define AUXOUTPUT1_PWM_TIMER_AF    3
  #endif
#endif

#if AUXOUTPUT1_PWM_TIMER_CH == 1 || AUXOUTPUT1_PWM_TIMER_CH == 2
#define AUXOUTPUT1_PWM_CCR 1
#else
#define AUXOUTPUT1_PWM_CCR 2
#endif
#define AUXOUTPUT1_PWM_TIMER           timer(AUXOUTPUT1_PWM_TIMER_N)
#define AUXOUTPUT1_PWM_TIMER_CCR       timerCCR(AUXOUTPUT1_PWM_TIMER_N, AUXOUTPUT1_PWM_TIMER_CH)
#define AUXOUTPUT1_PWM_TIMER_CCMR      timerCCMR(AUXOUTPUT1_PWM_TIMER_N, AUXOUTPUT1_PWM_CCR)
#define AUXOUTPUT1_PWM_CCMR_OCM_SET    timerOCM(AUXOUTPUT1_PWM_CCR, AUXOUTPUT1_PWM_TIMER_CH)
#define AUXOUTPUT1_PWM_CCMR_OCM_CLR    timerOCMC(AUXOUTPUT1_PWM_CCR, AUXOUTPUT1_PWM_TIMER_CH)
#if AUXOUTPUT1_PWM_TIMER_INV
#define AUXOUTPUT1_PWM_CCER_EN         timerCCEN(AUXOUTPUT1_PWM_TIMER_CH, N)
#define AUXOUTPUT1_PWM_CCER_POL        timerCCP(AUXOUTPUT1_PWM_TIMER_CH, N)
#define AUXOUTPUT1_PWM_CR2_OIS         timerCR2OIS(AUXOUTPUT1_PWM_TIMER_CH, N)
#else
#define AUXOUTPUT1_PWM_CCER_EN         timerCCEN(AUXOUTPUT1_PWM_TIMER_CH, )
#define AUXOUTPUT1_PWM_CCER_POL        timerCCP(AUXOUTPUT1_PWM_TIMER_CH, )
#define AUXOUTPUT1_PWM_CR2_OIS         timerCR2OIS(AUXOUTPUT1_PWM_TIMER_CH, )
#endif

#define AUXOUTPUT1_PWM_PORT            ((GPIO_TypeDef *)AUXOUTPUT1_PWM_PORT_BASE)
#define AUXOUTPUT1_PWM_AF              timerAF(AUXOUTPUT1_PWM_TIMER_N, AUXOUTPUT1_PWM_TIMER_AF)
#define AUXOUTPUT1_PWM_CLKEN           timerCLKEN(AUXOUTPUT1_PWM_TIMER_N)

#endif // AUXOUTPUT1_PWM_PORT_BASE

#if defined(AUXOUTPUT0_PWM_PORT_BASE) || defined(AUXOUTPUT1_PWM_PORT_BASE) ||\
     defined(AUXOUTPUT0_ANALOG_PORT) || defined( AUXOUTPUT1_ANALOG_PORT) ||\
      defined(MCP3221_ENABLE)
#define AUX_ANALOG 1
#else
#define AUX_ANALOG 0
#endif

#if !defined(PULSE2_TIMER_N) && STEP_INJECT_ENABLE
#if SPINDLE_PWM_TIMER_N == 2 || SPINDLE_PWM_TIMER_N == 9
#define PULSE2_TIMER_N              3
#else
#define PULSE2_TIMER_N              2
#endif
#define PULSE2_TIMER                timer(PULSE2_TIMER_N)
#define PULSE2_TIMER_CLKEN          timerCLKEN(PULSE2_TIMER_N)
#define PULSE2_TIMER_IRQn           timerINT(PULSE2_TIMER_N)
#define PULSE2_TIMER_IRQHandler     timerHANDLER(PULSE2_TIMER_N)
#endif

#if SPINDLE_PWM_TIMER_N == 9
#define DEBOUNCE_TIMER_N            13
#define DEBOUNCE_TIMER_IRQn         TIM8_UP_TIM13_IRQn       // !
#define DEBOUNCE_TIMER_IRQHandler   TIM8_UP_TIM13_IRQHandler // !
#else
#define DEBOUNCE_TIMER_N            9
#define DEBOUNCE_TIMER_IRQn         TIM1_BRK_TIM9_IRQn       // !
#define DEBOUNCE_TIMER_IRQHandler   TIM1_BRK_TIM9_IRQHandler // !
#endif
#define DEBOUNCE_TIMER              timer(DEBOUNCE_TIMER_N)
#define DEBOUNCE_TIMER_CLKEN        timerCLKEN(DEBOUNCE_TIMER_N)

#if SPINDLE_ENCODER_ENABLE

#ifndef RPM_COUNTER_N
#define RPM_COUNTER_N   3
#endif
#ifndef RPM_TIMER_N
#define RPM_TIMER_N     2
#endif

#if SPINDLE_PWM_TIMER_N == RPM_COUNTER_N || SPINDLE_PWM_TIMER_N == RPM_TIMER_N
#error Timer conflict: spindle sync and spindle PWM!
#endif
#if PULSE2_TIMER_N == RPM_COUNTER_N || PULSE2_TIMER_N == RPM_TIMER_N
#error Timer conflict: spindle sync and step inject!
#endif
#define RPM_COUNTER                 timer(RPM_COUNTER_N)
#define RPM_COUNTER_CLKEN           timerCLKEN(RPM_COUNTER_N)
#define RPM_COUNTER_IRQn            timerINT(RPM_COUNTER_N)
#define RPM_COUNTER_IRQHandler      timerHANDLER(RPM_COUNTER_N)

#define RPM_TIMER                   timer(RPM_TIMER_N)
#define RPM_TIMER_CLKEN             timerCLKEN(RPM_TIMER_N)
#define RPM_TIMER_IRQn              timerINT(RPM_TIMER_N)
#define RPM_TIMER_IRQHandler        timerHANDLER(RPM_TIMER_N)

#elif PPI_ENABLE

#if SPINDLE_PWM_TIMER_N == 2
#error Timer conflict: laser PPI and spindle PWM!
#endif

#define PPI_TIMER_N                 2
#define PPI_TIMER                   timer(PPI_TIMER_N)
#define PPI_TIMER_CLKEN             timerCLKEN(PPI_TIMER_N)
#define PPI_TIMER_IRQn              timerINT(PPI_TIMER_N)
#define PPI_TIMER_IRQHandler        timerHANDLER(PPI_TIMER_N)

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

#if USB_SERIAL_CDC && defined(SERIAL_PORT)
#define SP0 1
#else
#define SP0 0
#endif

#ifdef SERIAL1_PORT
#define SP1 1
#else
#define SP1 0
#endif

#ifdef SERIAL2_PORT
#define SP2 1
#else
#define SP2 0
#endif

#if MODBUS_ENABLE
#define MODBUS_TEST 1
#else
#define MODBUS_TEST 0
#endif

#if TRINAMIC_UART_ENABLE && !defined(MOTOR_UARTX_PORT)
#define TRINAMIC_TEST 1
#else
#define TRINAMIC_TEST 0
#endif

#if MPG_ENABLE
#define MPG_TEST 1
#else
#define MPG_TEST 0
#endif

#if KEYPAD_ENABLE == 2 && MPG_ENABLE == 0
#define KEYPAD_TEST 1
#else
#define KEYPAD_TEST 0
#endif

#if (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + TRINAMIC_TEST + (BLUETOOTH_ENABLE ? 1 : 0)) > (SP0 + SP1 + SP2)
#error "Too many options that uses a serial port are enabled!"
#endif

#undef SP0
#undef SP1
#undef SP2
#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef MPG_TEST
#undef TRINAMIC_TEST

#if MPG_MODE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

#if TRINAMIC_ENABLE
  #include "motors/trinamic.h"
  #ifndef TRINAMIC_MIXED_DRIVERS
    #define TRINAMIC_MIXED_DRIVERS 1
  #endif
#endif

// End configuration

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PORT)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PORT)
#error I2C strobe not supported!
#endif

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

typedef struct {
    pin_function_t id;
    GPIO_TypeDef *port;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    volatile bool active;
    volatile bool debounce;
    pin_cap_t cap;
    pin_mode_t mode;
    ADC_HandleTypeDef *adc;
    ioport_interrupt_callback_ptr interrupt_callback;
    aux_ctrl_t *aux_ctrl;
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
void gpio_irq_enable (const input_signal_t *input, pin_irq_mode_t irq_mode);
#ifdef HAS_BOARD_INIT
void board_init (void);
#endif

void ioports_init(pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#if AUX_ANALOG
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
#endif
void ioports_event (uint32_t bit);

#endif // __DRIVER_H__
