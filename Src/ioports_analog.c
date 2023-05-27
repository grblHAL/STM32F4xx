/*
  ioports_analog.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public Licens
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if AUX_ANALOG

#include "grbl/ioports.h"

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;

//static wait_on_input_ptr wait_on_input_digital;
static set_pin_description_ptr set_pin_description_digital;
static get_pin_info_ptr get_pin_info_digital;
static claim_port_ptr claim_digital;
//static swap_pins_ptr swap_pins_digital;

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;

#ifdef AUXOUTPUT0_PWM_PORT_BASE

ioports_pwm_t pwm0;

static void pwm0_out (float value)
{
    uint_fast16_t pwm_value = ioports_compute_pwm_value(&pwm0, value);

    if(pwm_value == pwm0.off_value) {
        if(pwm0.always_on) {
            AUXOUTPUT0_PWM_TIMER_CCR = pwm0.off_value;
#if AUXOUTPUT0_PWM_TIMER_N == 1
            AUXOUTPUT0_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
            AUXOUTPUT0_PWM_TIMER_CCR = pwm_value;
        } else
#if AUXOUTPUT0_PWM_TIMER_N == 1
            AUXOUTPUT0_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
#else
            AUXOUTPUT0_PWM_TIMER_CCR = 0;
#endif
    } else {
        AUXOUTPUT0_PWM_TIMER_CCR = pwm_value;
#if AUXOUTPUT0_PWM_TIMER_N == 1
        AUXOUTPUT0_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
    }
}

static void init_pwm0 (xbar_t *pin, pwm_config_t *config)
{
    static bool init_ok = false;

    RCC_ClkInitTypeDef clock;
    uint32_t latency, prescaler = 0;

    if(!init_ok) {

        init_ok = true;

        AUXOUTPUT0_PWM_CLOCK_ENA();

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pin = (1 << AUXOUTPUT0_PWM_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Alternate = AUXOUTPUT0_PWM_AF
        };
        HAL_GPIO_Init(AUXOUTPUT0_PWM_PORT, &GPIO_Init);
    }

    HAL_RCC_GetClockConfig(&clock, &latency);

    do {
        prescaler++;
#if AUXOUTPUT0_PWM_TIMER_N == 1
        ioports_precompute_pwm_values(config, &pwm0, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
        ioports_precompute_pwm_values(config, &pwm0, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
#endif
    } while(pwm0.period > 65530);

    AUXOUTPUT0_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

    TIM_Base_InitTypeDef timerInitStructure = {
        .Prescaler = prescaler - 1,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = pwm0.period - 1,
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };

    TIM_Base_SetConfig(AUXOUTPUT0_PWM_TIMER, &timerInitStructure);

    AUXOUTPUT0_PWM_TIMER->CCER &= ~AUXOUTPUT0_PWM_CCER_EN;
    AUXOUTPUT0_PWM_TIMER_CCMR &= ~AUXOUTPUT0_PWM_CCMR_OCM_CLR;
    AUXOUTPUT0_PWM_TIMER_CCMR |= AUXOUTPUT0_PWM_CCMR_OCM_SET;
    AUXOUTPUT0_PWM_TIMER_CCR = 0;
#if AUXOUTPUT0_PWM_TIMER_N == 1
    AUXOUTPUT0_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
#endif
    if(config->invert) {
        AUXOUTPUT0_PWM_TIMER->CCER |= AUXOUTPUT0_PWM_CCER_POL;
        AUXOUTPUT0_PWM_TIMER->CR2 |= AUXOUTPUT0_PWM_CR2_OIS;
    } else {
        AUXOUTPUT0_PWM_TIMER->CCER &= ~AUXOUTPUT0_PWM_CCER_POL;
        AUXOUTPUT0_PWM_TIMER->CR2 &= ~AUXOUTPUT0_PWM_CR2_OIS;
    }
    AUXOUTPUT0_PWM_TIMER->CCER |= AUXOUTPUT0_PWM_CCER_EN;
    AUXOUTPUT0_PWM_TIMER->CR1 |= TIM_CR1_CEN;

    pwm0_out(config->min);
}

#endif

#ifdef AUXOUTPUT1_PWM_PORT_BASE

ioports_pwm_t pwm1;

static void pwm1_out (float value)
{
    uint_fast16_t pwm_value = ioports_compute_pwm_value(&pwm1, value);

    if(pwm_value == pwm1.off_value) {
        if(pwm1.always_on) {
            AUXOUTPUT1_PWM_TIMER_CCR = pwm1.off_value;
#if AUXOUTPUT1_PWM_TIMER_N == 1
            AUXOUTPUT1_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
            AUXOUTPUT1_PWM_TIMER_CCR = pwm_value;
        } else
#if AUXOUTPUT1_PWM_TIMER_N == 1
            AUXOUTPUT1_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
#else
            AUXOUTPUT1_PWM_TIMER_CCR = 0;
#endif
    } else {
        AUXOUTPUT1_PWM_TIMER_CCR = pwm_value;
#if AUXOUTPUT1_PWM_TIMER_N == 1
        AUXOUTPUT1_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
#endif
    }
}

static void init_pwm1 (xbar_t *pin, pwm_config_t *config)
{
    static bool init_ok = false;

    RCC_ClkInitTypeDef clock;
    uint32_t latency, prescaler = 0;

    if(!init_ok) {

        init_ok = true;

        AUXOUTPUT1_PWM_CLOCK_ENA();

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pin = (1 << AUXOUTPUT1_PWM_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Alternate = AUXOUTPUT1_PWM_AF
        };
        HAL_GPIO_Init(AUXOUTPUT1_PWM_PORT, &GPIO_Init);
    }

    HAL_RCC_GetClockConfig(&clock, &latency);

    do {
        prescaler++;
#if AUXOUTPUT1_PWM_TIMER_N == 1
        ioports_precompute_pwm_values(config, &pwm1, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
        ioports_precompute_pwm_values(config, &pwm1, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
#endif
    } while(pwm1.period > 65530);

    AUXOUTPUT1_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

    TIM_Base_InitTypeDef timerInitStructure = {
        .Prescaler = prescaler - 1,
        .CounterMode = TIM_COUNTERMODE_UP,
        .Period = pwm1.period - 1,
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0
    };

    TIM_Base_SetConfig(AUXOUTPUT1_PWM_TIMER, &timerInitStructure);

    AUXOUTPUT1_PWM_TIMER->CCER &= ~AUXOUTPUT1_PWM_CCER_EN;
    AUXOUTPUT1_PWM_TIMER_CCMR &= ~AUXOUTPUT1_PWM_CCMR_OCM_CLR;
    AUXOUTPUT1_PWM_TIMER_CCMR |= AUXOUTPUT1_PWM_CCMR_OCM_SET;
    AUXOUTPUT1_PWM_TIMER_CCR = 0;
#if AUXOUTPUT1_PWM_TIMER_N == 1
    AUXOUTPUT1_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
#endif
    if(config->invert) {
        AUXOUTPUT1_PWM_TIMER->CCER |= AUXOUTPUT1_PWM_CCER_POL;
        AUXOUTPUT1_PWM_TIMER->CR2 |= AUXOUTPUT1_PWM_CR2_OIS;
    } else {
        AUXOUTPUT1_PWM_TIMER->CCER &= ~AUXOUTPUT1_PWM_CCER_POL;
        AUXOUTPUT1_PWM_TIMER->CR2 &= ~AUXOUTPUT1_PWM_CR2_OIS;
    }
    AUXOUTPUT1_PWM_TIMER->CCER |= AUXOUTPUT1_PWM_CCER_EN;
    AUXOUTPUT1_PWM_TIMER->CR1 |= TIM_CR1_CEN;

    pwm1_out(config->min);
}

#endif

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {
        port = ioports_map(analog.out, port);
#ifdef AUXOUTPUT0_PWM_PORT_BASE
        if(aux_out_analog[port].port == AUXOUTPUT0_PWM_PORT && aux_out_analog[port].pin == AUXOUTPUT0_PWM_PIN)
            pwm0_out(value);
#endif
#ifdef AUXOUTPUT1_PWM_PORT_BASE
        if(aux_out_analog[port].port == AUXOUTPUT1_PWM_PORT && aux_out_analog[port].pin == AUXOUTPUT1_PWM_PIN)
            pwm1_out(value);
#endif
    }

    return port < analog.out.n_ports;
}

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if(type == Port_Digital)
        return get_pin_info_digital ? get_pin_info_digital(type, dir, port) : NULL;

    else if(dir == Port_Output) {

        memset(&pin, 0, sizeof(xbar_t));

        if(port < analog.out.n_ports) {
            port = ioports_map(analog.out, port);
            pin.mode = aux_out_analog[port].mode;
            pin.mode.output = pin.mode.analog = On;
            pin.cap = pin.mode;
            pin.function = aux_out_analog[port].id;
            pin.group = aux_out_analog[port].group;
            pin.pin = aux_out_analog[port].pin;
            pin.bit = 1 << aux_out_analog[port].pin;
            pin.port = (void *)aux_out_analog[port].port;
            pin.description = aux_out_analog[port].description;
#ifdef AUXOUTPUT0_PWM_PORT_BASE
            if(aux_out_analog[port].port == AUXOUTPUT0_PWM_PORT && aux_out_analog[port].pin == AUXOUTPUT0_PWM_PIN)
                pin.config = (xbar_config_ptr)init_pwm0;
#endif
#ifdef AUXOUTPUT1_PWM_PORT_BASE
            if(aux_out_analog[port].port == AUXOUTPUT1_PWM_PORT && aux_out_analog[port].pin == AUXOUTPUT1_PWM_PIN)
                pin.config = (xbar_config_ptr)init_pwm1;
#endif
            info = &pin;
        }
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description)
{
    if(type == Port_Analog) {
        if(dir == Port_Output && port < analog.out.n_ports)
            aux_out_analog[ioports_map(analog.out, port)].description = description;
    } else if(set_pin_description_digital)
        set_pin_description_digital(type, dir, port, description);
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital)
        return claim_digital ? claim_digital(type, dir, port, description) : false;

    else if(dir == Port_Output) {

        if((ok = analog.out.map && *port < analog.out.n_ports && !aux_out_analog[*port].mode.claimed)) {

            uint8_t i;

            hal.port.num_analog_out--;

            for(i = ioports_map_reverse(&analog.out, *port); i < hal.port.num_analog_out; i++) {
                analog.out.map[i] = analog.out.map[i + 1];
                aux_out_analog[analog.out.map[i]].description = iports_get_pnum(analog, i);
            }

            aux_out_analog[*port].mode.claimed = On;
            aux_out_analog[*port].description = description;

            analog.out.map[hal.port.num_analog_out] = *port;
            *port = hal.port.num_analog_out;
        }
    }

    return ok;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    set_pin_description_digital = hal.port.set_pin_description;
    hal.port.set_pin_description = set_pin_description;

    if(ioports_add(&analog, Port_Analog, aux_inputs->n_pins, aux_outputs->n_pins))  {

        uint_fast8_t i;

        if(analog.in.n_ports) {
//            wait_on_input_digital = hal.port.wait_on_input;
//            hal.port.wait_on_input = wait_on_input;
        }

        if(analog.out.n_ports) {

            pwm_config_t config = {
                .freq_hz = 5000.0f,
                .min = 0.0f,
                .max = 100.0f,
                .off_value = 0.0f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .invert = Off
            };

            hal.port.analog_out = analog_out;

            for(i = 0; i < analog.out.n_ports; i++) {
#ifdef AUXOUTPUT0_PWM_PORT_BASE
                if(aux_out_analog[i].port == AUXOUTPUT0_PWM_PORT && aux_out_analog[i].pin == AUXOUTPUT0_PWM_PIN)
                    init_pwm0(NULL, &config);
#endif
#ifdef AUXOUTPUT1_PWM_PORT_BASE
                if(aux_out_analog[i].port == AUXOUTPUT1_PWM_PORT && aux_out_analog[i].pin == AUXOUTPUT1_PWM_PIN)
                    init_pwm1(NULL, &config);
#endif
            }
        }

        claim_digital = hal.port.claim;
        hal.port.claim = claim;
        get_pin_info_digital = hal.port.get_pin_info;
        hal.port.get_pin_info = get_pin_info;
//        swap_pins = hal.port.swap_pins;
//        hal.port.swap_pins = swap_pins;

    } else
        hal.port.set_pin_description = set_pin_description_digital;
}

#endif
