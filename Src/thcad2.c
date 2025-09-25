/*
  thcad2.c - analog input from frequency, for Mesa THCAD2 converter and STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#include "driver.h"

#if THCAD2_ENABLE

#include "grbl/plugins.h"
#include "grbl/ioports.h"

#define THCAD2_PORT GPIOA // TIM2_CH2 - PA1 and PB3 supported
#define THCAD2_PIN  1

#define PULSE_COUNTER_N     2
#define GATE_TIMER_N        3

#define DMA_INSTANCE_N      1
#define DMA_STREAM_N        2
#define DMA_CHANNEL_N       5

#define PULSE_COUNTER       timer(PULSE_COUNTER_N)
#define PULSE_COUNTER_BASE  timerBase(PULSE_COUNTER_N)
#define PULSE_COUNTER_CLKEN timerCLKEN(PULSE_COUNTER_N)

#define GATE_TIMER          timer(GATE_TIMER_N)
#define GATE_TIMER_BASE     timerBase(GATE_TIMER_N)
#define GATE_TIMER_CLKEN    timerCLKEN(GATE_TIMER_N)

#define DMA_INSTANCE        DMAinstance(DMA_INSTANCE_N)
#define DMA_STREAM          DMAstream(DMA_INSTANCE_N, DMA_STREAM_N)
#define DMA_CHANNEL         DMAchannel(DMA_CHANNEL_N)
#define DMA_INSTANCE_CLKEN  DMA_CLKEN(DMA_INSTANCE_N)
#define DMA_TX_IRQ          DMAirq(DMA_INSTANCE_N, DMA_STREAM_N)
#define DMA_TX_IRQ_HANDLER  DMAhandler(DMA_INSTANCE_N, DMA_STREAM_N)

static float value = -1.0f;
static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

static TIM_HandleTypeDef gate_timer = {
    .Instance = GATE_TIMER,
    .Init.Period = 999,
    .Init.CounterMode = TIM_COUNTERMODE_UP,
    .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .Init.RepetitionCounter = 0,
    .Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE
};

static DMA_HandleTypeDef pwm_dma = {
    .Instance = DMA_STREAM,
    .Init.Channel = DMA_CHANNEL,
    .Init.Direction = DMA_PERIPH_TO_MEMORY,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_DISABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
    .Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_MEDIUM,
    .Init.FIFOMode = DMA_FIFOMODE_ENABLE
};

uint counter_slice, gate_slice;
uint timer_dma_chan;
uint16_t csr_stopval = 0;
uint32_t timers_enable;

static xbar_t thcad2 = {
    .id = 0,
    .pin = THCAD2_PIN,
    .function = Input_Analog_Aux0,
    .group = PinGroup_AuxInputAnalog,
    .port = &value,
    .cap = {
        .input = On,
        .analog = On,
        .resolution = Resolution_12bit,
        .external = On,
        .claimable = On
    },
    .mode = {
        .output = Off,
        .analog = On
    }
};

static float thcad2_in_state (xbar_t *input)
{
    return value;
}

static int32_t thcad2_wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? (int32_t)value : -1;
}

static bool set_pin_function (xbar_t *input, pin_function_t function)
{
    if(input->id == thcad2.id)
        thcad2.function = function;

    return input->id == thcad2.id;
}

static xbar_t *thcad2_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(dir == Port_Input && port < analog.in.n_ports) {
        pin.get_value = thcad2_in_state;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void thcad2_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        thcad2.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(!low_level)
        pin.port = "THCAD2:";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInputAnalog)
        *(pin_function_t *)fn = max(*(pin_function_t *)fn, pin->function + 1);
}

static inline void get_sample (void)
{
    value = (float)csr_stopval;

    GATE_TIMER->DIER |= TIM_DIER_UDE;

    HAL_DMA_Start_IT(&pwm_dma, (uint32_t)&(PULSE_COUNTER->CNT), (uint32_t)&csr_stopval, 1);

    PULSE_COUNTER->EGR |= TIM_EGR_UG;
}

void thcad2_init (void)
{
    io_analog_t ports = {
        .ports = &analog,
        .get_pin_info = thcad2_get_pin_info,
        .wait_on_input = thcad2_wait_on_input,
        .set_pin_description = thcad2_set_pin_description
    };

    hal.enumerate_pins(false, get_next_port, &thcad2.function);

    analog.in.n_ports = 1;

    if(timer_claim(GATE_TIMER) &&
        timer_claim(PULSE_COUNTER) &&
         ioports_add_analog(&ports)) {

        GATE_TIMER_CLKEN();
        GATE_TIMER->PSC = timer_clk_enable(GATE_TIMER) / 10000000UL * 100 - 1;
        GATE_TIMER->ARR = 1020;
        GATE_TIMER->CR1 = TIM_CR1_CKD_1|TIM_CR1_URS|TIM_CR1_OPM;
        GATE_TIMER->SMCR = TIM_SMCR_TS_0;
        GATE_TIMER->SMCR |= TIM_SMCR_SMS_1|TIM_SMCR_SMS_2;

        PULSE_COUNTER_CLKEN();
#if 1
        PULSE_COUNTER->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2;
#else
        PULSE_COUNTER->SMCR = TIM_SMCR_ECE;
#endif
        PULSE_COUNTER->PSC = 0;
        PULSE_COUNTER->ARR = 65535;
        PULSE_COUNTER->CR2 |= TIM_CR2_MMS_1|TIM_CR2_TI1S;
        PULSE_COUNTER->CR1 |= TIM_CR1_CEN;

        GPIO_InitTypeDef GPIO_Init = {
            .Pin = 1 << THCAD2_PIN,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
#if PULSE_COUNTER_N == 2
            .Alternate = GPIO_AF1_TIM2
#else
            .Alternate = GPIO_AF2_TIM3
#endif
        };

        HAL_GPIO_Init(THCAD2_PORT, &GPIO_Init);

        static const periph_pin_t ssp = {
            .function = Input_SpindlePulse,
            .group = PinGroup_SpindlePulse,
            .port = THCAD2_PORT,
            .pin = THCAD2_PIN,
            .mode = { .mask = PINMODE_NONE }
        };

        hal.periph_port.register_pin(&ssp);

        DMA_INSTANCE_CLKEN();

        HAL_DMA_Init(&pwm_dma);
        __HAL_LINKDMA(&gate_timer, hdma[TIM_DMA_ID_UPDATE], pwm_dma);

        HAL_NVIC_SetPriority(DMA_TX_IRQ, 3, 0);
        HAL_NVIC_EnableIRQ(DMA_TX_IRQ);

        get_sample();
    }

    on_enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = onEnumeratePins;
}

void DMA_TX_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&pwm_dma);

    get_sample();
}

#endif // THCAD2_ENABLE
