/*
  ioports_analog.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io

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

#ifdef AUXOUTPUT0_PWM_PORT_BASE
#define PWM_OUT0 1
#else
#define PWM_OUT0 0
#endif

#ifdef AUXOUTPUT1_PWM_PORT_BASE
#define PWM_OUT1 1
#else
#define PWM_OUT1 0
#endif

#define AUX_ANALOG_OUT (PWM_OUT0 + PWM_OUT1)

#include "grbl/ioports.h"

typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
    uint8_t alt;
    ADC_TypeDef *adc;
    uint32_t ch;
} adc_map_t;

static const adc_map_t adc_map[] = {
    { GPIOA,  0, 123, ADC1, ADC_CHANNEL_0 },
    { GPIOA,  1, 123, ADC1, ADC_CHANNEL_1 },
    { GPIOA,  2, 123, ADC1, ADC_CHANNEL_2 },
    { GPIOA,  3, 123, ADC1, ADC_CHANNEL_3 },
    { GPIOA,  4,  12, ADC2, ADC_CHANNEL_4 },
    { GPIOA,  5,  12, ADC2, ADC_CHANNEL_5 },
    { GPIOA,  6,  12, ADC2, ADC_CHANNEL_6 },
    { GPIOA,  7,  12, ADC2, ADC_CHANNEL_7 },
    { GPIOB,  0,  12, ADC2, ADC_CHANNEL_8 },
    { GPIOB,  1,  12, ADC2, ADC_CHANNEL_9 },
    { GPIOC,  0, 123, ADC1, ADC_CHANNEL_10 },
    { GPIOC,  1, 123, ADC1, ADC_CHANNEL_11 },
    { GPIOC,  2, 123, ADC1, ADC_CHANNEL_12 },
    { GPIOC,  3, 123, ADC1, ADC_CHANNEL_13 },
    { GPIOC,  4,  12, ADC2, ADC_CHANNEL_14 },
    { GPIOC,  5,  12, ADC2, ADC_CHANNEL_15 },
    { GPIOF,  3,   3, ADC3, ADC_CHANNEL_9 },
    { GPIOF,  4,   3, ADC3, ADC_CHANNEL_14 },
    { GPIOF,  5,   3, ADC3, ADC_CHANNEL_15 },
    { GPIOF,  6,   3, ADC3, ADC_CHANNEL_4 },
    { GPIOF,  7,   3, ADC3, ADC_CHANNEL_5 },
    { GPIOF,  8,   3, ADC3, ADC_CHANNEL_6 },
    { GPIOF,  9,   3, ADC3, ADC_CHANNEL_7 },
    { GPIOF, 10,   3, ADC3, ADC_CHANNEL_8 }
};

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;

static set_pin_description_ptr set_pin_description_digital;
static get_pin_info_ptr get_pin_info_digital;
static claim_port_ptr claim_digital;
//static swap_pins_ptr swap_pins_digital;
static wait_on_input_ptr wait_on_input_digital;

#ifdef MCP3221_ENABLE

#include "MCP3221.h"

static xbar_t analog_in;
static enumerate_pins_ptr on_enumerate_pins;

static void enumerate_pins (bool low_level, pin_info_ptr pin_info, void *data)
{
    on_enumerate_pins(low_level, pin_info, data);

    pin_info(&analog_in, data);
}

#endif // MCP3221_ENABLE

#if AUX_ANALOG_OUT

typedef struct {
    ioports_pwm_t data;
    float value;
    void (*set_value)(uint_fast8_t ch, float value);
} pwm_out_t;

static pwm_out_t pwm_out[AUX_ANALOG_OUT] = {0};

static float pwm_get_value (struct xbar *output)
{
    int_fast8_t ch = output->function - Output_Analog_Aux0;

    return ch >= 0 && ch < analog.out.n_ports ? pwm_out[ch].value : -1.0f;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        port = ioports_map(analog.out, port);

        uint_fast8_t ch = aux_out_analog[port].id - Output_Analog_Aux0;

        pwm_out[ch].set_value(ch, value);
    }

    return port < analog.out.n_ports;
}

#endif

#ifdef AUXOUTPUT0_PWM_PORT_BASE

static void pwm0_out (uint_fast8_t ch, float value)
{
    uint_fast16_t pwm_value = ioports_compute_pwm_value(&pwm_out[ch].data, value);

    pwm_out[ch].value = value;

    if(pwm_value == pwm_out[ch].data.off_value) {
        if(pwm_out[ch].data.always_on) {
            AUXOUTPUT0_PWM_TIMER_CCR = pwm_out[ch].data.off_value;
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

static bool init_pwm0 (xbar_t *pin, pwm_config_t *config)
{
    static bool init_ok = false;

    bool ok;
    RCC_ClkInitTypeDef clock;
    uint32_t latency, prescaler = 0;
    uint_fast8_t ch = pin->function - Output_Analog_Aux0;
    ioports_pwm_t *pwm_data = &pwm_out[ch].data;

    if(!init_ok) {

        init_ok = true;

        AUXOUTPUT0_PWM_CLKEN();

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pin = (1 << AUXOUTPUT0_PWM_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Alternate = AUXOUTPUT0_PWM_AF
        };
        HAL_GPIO_Init(AUXOUTPUT0_PWM_PORT, &GPIO_Init);

        pwm_out[ch].set_value = pwm0_out;
    }

    HAL_RCC_GetClockConfig(&clock, &latency);

    do {
        prescaler++;
#if AUXOUTPUT0_PWM_TIMER_N == 1
        ok = ioports_precompute_pwm_values(config, pwm_data, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
        ok = ioports_precompute_pwm_values(config, pwm_data, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
#endif
    } while(ok && pwm_data->period > 65530);

    if(ok) {

        AUXOUTPUT0_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

        TIM_Base_InitTypeDef timerInitStructure = {
            .Prescaler = prescaler - 1,
            .CounterMode = TIM_COUNTERMODE_UP,
            .Period = pwm_data->period - 1,
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

        aux_out_analog[ch].mode.pwm = !config->servo_mode;
        aux_out_analog[ch].mode.servo_pwm = config->servo_mode;

        pwm0_out(ch, config->min);
    }

    return ok;
}

#endif // AUXOUTPUT0_PWM_PORT_BASE

#ifdef AUXOUTPUT1_PWM_PORT_BASE

static void pwm1_out (uint_fast8_t ch, float value)
{
    uint_fast16_t pwm_value = ioports_compute_pwm_value(&pwm_out[ch].data, value);

    pwm_out[ch].value = value;

    if(pwm_value == pwm_out[ch].data.off_value) {
        if(pwm_out[ch].data.always_on) {
            AUXOUTPUT1_PWM_TIMER_CCR = pwm_out[ch].data.off_value;
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

static bool init_pwm1 (xbar_t *pin, pwm_config_t *config)
{
    static bool init_ok = false;

    bool ok;
    RCC_ClkInitTypeDef clock;
    uint32_t latency, prescaler = 0;
    uint_fast8_t ch = pin->function - Output_Analog_Aux0;
    ioports_pwm_t *pwm_data = &pwm_out[ch].data;

    if(!init_ok) {

        init_ok = true;

        AUXOUTPUT1_PWM_CLKEN();

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pin = (1 << AUXOUTPUT1_PWM_PIN),
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Alternate = AUXOUTPUT1_PWM_AF
        };
        HAL_GPIO_Init(AUXOUTPUT1_PWM_PORT, &GPIO_Init);

        pwm_out[ch].set_value = pwm1_out;
    }

    HAL_RCC_GetClockConfig(&clock, &latency);

    do {
        prescaler++;
#if AUXOUTPUT1_PWM_TIMER_N == 1
        ok = ioports_precompute_pwm_values(config, pwm_data, (HAL_RCC_GetPCLK2Freq() * TIMER_CLOCK_MUL(clock.APB2CLKDivider)) / prescaler);
#else
        ok = ioports_precompute_pwm_values(config, pwm_data, (HAL_RCC_GetPCLK1Freq() * TIMER_CLOCK_MUL(clock.APB1CLKDivider)) / prescaler);
#endif
    } while(ok && pwm_data->period > 65530);

    if(ok) {

        AUXOUTPUT1_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

        TIM_Base_InitTypeDef timerInitStructure = {
            .Prescaler = prescaler - 1,
            .CounterMode = TIM_COUNTERMODE_UP,
            .Period = pwm_data->period - 1,
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

        aux_out_analog[ch].mode.pwm = !config->servo_mode;
        aux_out_analog[ch].mode.servo_pwm = config->servo_mode;

        pwm1_out(ch, config->min);
    }

    return ok;
}

#endif // AUXOUTPUT1_PWM_PORT_BASE

static int32_t wait_on_input_dummy (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return -1;
}

static int32_t wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(type == Port_Digital)
        return wait_on_input_digital(type, port, wait_mode, timeout);

    port = ioports_map(analog.in, port);

#ifdef MCP3221_ENABLE
    if(port == analog_in.pin)
        value = (int32_t)MCP3221_read();
    else
#endif
    if(port < analog.in.n_ports && aux_in_analog[port].adc) {
        HAL_ADC_Start(aux_in_analog[port].adc);
        if(HAL_ADC_PollForConversion(aux_in_analog[port].adc, 2) == HAL_OK)
            value = HAL_ADC_GetValue(aux_in_analog[port].adc);
    }

    return value;
}

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;
    xbar_t *info = NULL;

    if(type == Port_Digital)
        return get_pin_info_digital ? get_pin_info_digital(type, dir, port) : NULL;

    else switch(dir) {

        case Port_Input:
            if(port < analog.in.n_ports) {
                port = ioports_map(analog.in, port);
#ifdef MCP3221_ENABLE
                if(port == analog_in.pin)
                    info = &analog_in;
                else
#endif
                {
                    pin.mode = aux_in_analog[port].mode;
                    pin.cap = aux_in_analog[port].cap;
                    pin.cap.claimable = !pin.mode.claimed;
                    pin.function = aux_in_analog[port].id;
                    pin.group = aux_in_analog[port].group;
                    pin.pin = aux_in_analog[port].pin;
                    pin.bit = 1 << aux_in_analog[port].pin;
                    pin.port = (void *)aux_in_analog[port].port;
                    pin.description = aux_in_analog[port].description;
                    info = &pin;
                }
            }
            break;

        case Port_Output:
#if AUX_ANALOG_OUT
            memset(&pin, 0, sizeof(xbar_t));

            if(port < analog.out.n_ports) {
                port = ioports_map(analog.out, port);
                pin.mode = aux_out_analog[port].mode;
                pin.mode.pwm = !pin.mode.servo_pwm; //?? for easy filtering
                XBAR_SET_CAP(pin.cap, pin.mode);
                pin.function = aux_out_analog[port].id;
                pin.group = aux_out_analog[port].group;
                pin.pin = aux_out_analog[port].pin;
                pin.bit = 1 << aux_out_analog[port].pin;
                pin.port = (void *)aux_out_analog[port].port;
                pin.description = aux_out_analog[port].description;
                pin.get_value = pwm_get_value;
    #ifdef AUXOUTPUT0_PWM_PORT_BASE
                if(pin.port == AUXOUTPUT0_PWM_PORT && pin.pin == AUXOUTPUT0_PWM_PIN)
                    pin.config = (xbar_config_ptr)init_pwm0;
    #endif
    #ifdef AUXOUTPUT1_PWM_PORT_BASE
                if(pin.port == AUXOUTPUT1_PWM_PORT && pin.pin == AUXOUTPUT1_PWM_PIN)
                    pin.config = (xbar_config_ptr)init_pwm1;
    #endif
                info = &pin;
            }
#endif // AUX_ANALOG_OUT
            break;
    }

    return info;
}

static void set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description)
{
    if(type == Port_Analog) {
        if(dir == Port_Input && port < analog.in.n_ports) {
            port = ioports_map(analog.in, port);
#ifdef MCP3221_ENABLE
            if(port == analog_in.pin)
                analog_in.description = description;
            else
#endif
            aux_in_analog[port].description = description;
        } else if(port < analog.out.n_ports)
            aux_out_analog[ioports_map(analog.out, port)].description = description;
    } else if(set_pin_description_digital)
        set_pin_description_digital(type, dir, port, description);
}

static bool claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;

    if(type == Port_Digital)
        return claim_digital ? claim_digital(type, dir, port, description) : false;

    else switch(dir) {

        case Port_Input:

            if((ok = analog.in.map && *port < analog.in.n_ports && !(
#ifdef MCP3221_ENABLE
                    *port == analog_in.pin ? analog_in.mode.claimed :
#endif
                    aux_in_analog[*port].mode.claimed))) {

                uint8_t i;

                hal.port.num_analog_in--;

                for(i = ioports_map_reverse(&analog.in, *port); i < hal.port.num_analog_in; i++) {
                    analog.in.map[i] = analog.in.map[i + 1];
#ifdef MCP3221_ENABLE
                    if(analog_in.pin == analog.in.map[i])
                        analog_in.description = iports_get_pnum(analog, i);
                    else
#endif
                    aux_in_analog[analog.in.map[i]].description = iports_get_pnum(analog, i);
                }

#ifdef MCP3221_ENABLE
                if(*port == analog_in.pin) {
                    analog_in.mode.claimed = On;
                    analog_in.description = description;
                } else
#endif
                {
                    aux_in_analog[*port].mode.claimed = On;
                    aux_in_analog[*port].description = description;
                }
                analog.in.map[hal.port.num_analog_in] = *port;
                *port = hal.port.num_analog_in;
            }
            break;

        case Port_Output:
#if AUX_ANALOG_OUT
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
#endif
            break;
    }

    return ok;
}

void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    uint8_t p_pins = aux_inputs->n_pins;

    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    set_pin_description_digital = hal.port.set_pin_description;
    hal.port.set_pin_description = set_pin_description;

#ifdef MCP3221_ENABLE

    pin_group_pins_t aux_in = {
        .n_pins = 1
    };

    analog_in.function = Input_Analog_Aux0 + aux_inputs->n_pins;
    analog_in.group = PinGroup_AuxInput;
    analog_in.pin = aux_inputs->n_pins;
    analog_in.port = "MCP3221:";

    if(MCP3221_init()) {
        analog_in.mode.analog = On;
        if(aux_inputs)
            aux_inputs->n_pins++;
        else
            aux_inputs = &aux_in;
    } else
        analog_in.description = "No power";

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = enumerate_pins;

#endif // MCP3221_ENABLE

    if(ioports_add(&analog, Port_Analog, aux_inputs->n_pins, aux_outputs->n_pins)) {

        if(p_pins) {

            GPIO_InitTypeDef gpio_init = {
                .Mode = GPIO_MODE_ANALOG,
                .Pull = GPIO_NOPULL
            };

            ADC_ChannelConfTypeDef adc_config = {
                .Rank = 1,
                .SamplingTime = ADC_SAMPLETIME_3CYCLES
            };

            uint_fast8_t i;

            for(i = 0; i < p_pins; i++) {

                uint_fast8_t j = sizeof(adc_map) / sizeof(adc_map_t);

                do {
                    j--;
                    if(adc_map[j].port == aux_inputs->pins.inputs[i].port && adc_map[j].pin == aux_inputs->pins.inputs[i].pin) {

                        ADC_HandleTypeDef *adc;

                        if((adc = calloc(sizeof(ADC_HandleTypeDef), 1))) {

                            if(adc_map[j].alt == 3)
                                __HAL_RCC_ADC3_CLK_ENABLE();
                            else if(adc_map[j].alt == 12)
                                __HAL_RCC_ADC2_CLK_ENABLE();
                            else
                                __HAL_RCC_ADC1_CLK_ENABLE();

                            gpio_init.Pin = aux_inputs->pins.inputs[i].bit;
                            HAL_GPIO_Init(aux_inputs->pins.inputs[i].port, &gpio_init);

                            adc_config.Channel = adc_map[j].ch;

                            adc->Instance = adc_map[j].adc;
                            adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
                            adc->Init.Resolution = ADC_RESOLUTION_12B;
                            adc->Init.ScanConvMode = DISABLE;
                            adc->Init.ContinuousConvMode = DISABLE;
                            adc->Init.DiscontinuousConvMode = DISABLE;
                            adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
                            adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
                            adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
                            adc->Init.NbrOfConversion = 1;
                            adc->Init.DMAContinuousRequests = DISABLE;
                            adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
                            if(HAL_ADC_Init(adc) == HAL_OK && HAL_ADC_ConfigChannel(adc, &adc_config) == HAL_OK)
                                aux_inputs->pins.inputs[i].adc = adc;
                            else
                                analog.in.n_ports--;
                        }
                        break;
                    }
                } while(j);
            }
        }

        if(analog.in.n_ports) {
            if((wait_on_input_digital = hal.port.wait_on_input) == NULL)
                wait_on_input_digital = wait_on_input_dummy;
            hal.port.wait_on_input = wait_on_input;
        }

#if AUX_ANALOG_OUT

        if(analog.out.n_ports) {

            xbar_t *pin;
            uint_fast8_t i;
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
                if((pin = get_pin_info(Port_Analog, Port_Output, i)))
                    pin->config(pin, &config);
            }
        }

#endif // AUX_ANALOG_OUT

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
