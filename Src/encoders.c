/*

  encoders.c - driver code for STM32F4xx ARM processors

  *** WIP ***

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if SPINDLE_ENCODER_ENABLE

#include "grbl/spindle_sync.h"

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

#if RPM_TIMER_N != 2
static volatile uint32_t rpm_timer_ovf = 0;
#define RPM_TIMER_RESOLUTION 1
#define RPM_TIMER_COUNT (RPM_TIMER->CNT | (rpm_timer_ovf << 16))
#else
#define RPM_TIMER_RESOLUTION 1
#define RPM_TIMER_COUNT RPM_TIMER->CNT
#endif

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;

    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = RPM_TIMER_COUNT - spindle_encoder.timer.last_pulse;

    // if 16 bit RPM timer and RPM_TIMER_COUNT < spindle_encoder.timer.last_pulse then what?

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->CNT - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AtSpeed:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            spindle_data.state_programmed.at_speed = !spindle_data.at_speed_enabled || (spindle_data.rpm >= spindle_data.rpm_low_limit && spindle_data.rpm <= spindle_data.rpm_high_limit);
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindle_encoder.spin_lock);

    uint32_t timeout = uwTick + 1000; // 1 second

    uint32_t index_count = spindle_encoder.counter.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_encoder.counter.index_count && uwTick <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }

    RPM_TIMER->EGR |= TIM_EGR_UG; // Reload RPM timer
    RPM_COUNTER->CR1 &= ~TIM_CR1_CEN;

#if RPM_TIMER_N != 2
    rpm_timer_ovf = 0;
#endif

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->EGR |= TIM_EGR_UG;
    RPM_COUNTER->CCR1 = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CR1 |= TIM_CR1_CEN;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        spindle_set_at_speed_range(spindle, &spindle_data, rpm);
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
    }
}

void spindle_encoder_cfg (settings_t *settings)
{
    static const spindle_data_ptrs_t encoder_data = {
        .get = spindleGetData,
        .reset = spindleDataReset
    };

    static bool init_ok = false, event_claimed = false;

    if(!init_ok) {

        init_ok = true;

        RPM_TIMER_CLKEN();
    #if timerAPB2(RPM_TIMER_N)
        RPM_TIMER->PSC = HAL_RCC_GetPCLK2Freq() * 2 / 1000000UL * RPM_TIMER_RESOLUTION - 1;
    #else
        RPM_TIMER->PSC = HAL_RCC_GetPCLK1Freq() * 2 / 1000000UL * RPM_TIMER_RESOLUTION - 1;
    #endif
    #if RPM_TIMER_N == 2
        RPM_TIMER->CR1 = TIM_CR1_CKD_1;
    #else
        RPM_TIMER->CR1 = TIM_CR1_CKD_1|TIM_CR1_URS;
        RPM_TIMER->DIER |= TIM_DIER_UIE;
        HAL_NVIC_EnableIRQ(RPM_TIMER_IRQn);
        HAL_NVIC_SetPriority(RPM_TIMER_IRQn, 0, 0);
    #endif
        RPM_TIMER->CR1 |= TIM_CR1_CEN;

        RPM_COUNTER_CLKEN();
    #if SPINDLE_ENCODER_CLK == 1
        RPM_COUNTER->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2;
    #else
        RPM_COUNTER->SMCR = TIM_SMCR_ECE;
    #endif
        RPM_COUNTER->PSC = 0;
        RPM_COUNTER->ARR = 65535;
        RPM_COUNTER->DIER = TIM_DIER_CC1IE;

        HAL_NVIC_EnableIRQ(RPM_COUNTER_IRQn);

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Mode = GPIO_MODE_AF_PP,
            .Pin = (1 << SPINDLE_PULSE_PIN),
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Alternate = GPIO_AF2_TIM3
        };

        HAL_GPIO_Init(SPINDLE_PULSE_PORT, &GPIO_Init);

        static const periph_pin_t ssp = {
            .function = Input_SpindlePulse,
            .group = PinGroup_SpindlePulse,
            .port = SPINDLE_PULSE_PORT,
            .pin = SPINDLE_PULSE_PIN,
            .mode = { .mask = PINMODE_NONE }
        };

        hal.periph_port.register_pin(&ssp);
    }

    if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL)) {

        hal.driver_cap.spindle_encoder_index_event = On;

        if(spindle_encoder.ppr != settings->spindle.ppr) {

            spindle_ptrs_t *spindle;

            hal.spindle_data.reset = spindleDataReset;
            if((spindle = spindle_get(0)))
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

            if(!event_claimed) {
                event_claimed = true;
                on_spindle_programmed = grbl.on_spindle_programmed;
                grbl.on_spindle_programmed = onSpindleProgrammed;
            }

            spindle_encoder.ppr = settings->spindle.ppr;
            spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = 250000UL / RPM_TIMER_RESOLUTION; // 250ms
            spindle_encoder.rpm_factor = (60.0f * 1000000.0f / RPM_TIMER_RESOLUTION) / (float)spindle_encoder.ppr;
            spindleDataReset();
        }
    } else {
        spindle_encoder.ppr = 0;
        hal.spindle_data.reset = NULL;
        hal.driver_cap.spindle_encoder_index_event = Off;
    }

    spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);
}

ISR_CODE void RPM_COUNTER_IRQHandler (void)
{
    spindle_encoder.spin_lock = true;

    __disable_irq();
    uint32_t tval = RPM_TIMER_COUNT;
    uint16_t cval = RPM_COUNTER->CNT;
    __enable_irq();

    RPM_COUNTER->SR = ~TIM_SR_CC1IF;
    RPM_COUNTER->CCR1 = (uint16_t)(RPM_COUNTER->CCR1 + spindle_encoder.tics_per_irq);

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

#if RPM_TIMER_N != 2

ISR_CODE void RPM_TIMER_IRQHandler (void)
{
    RPM_TIMER->SR &= ~TIM_SR_UIF;

    rpm_timer_ovf++;
}

#endif

void spindle_encoder_index_event (void)
{
    uint32_t rpm_count = RPM_COUNTER->CNT;
    spindle_encoder.timer.last_index = RPM_TIMER_COUNT;

    if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
        spindle_encoder.error_count++;

    spindle_encoder.counter.last_index = rpm_count;
    spindle_encoder.counter.index_count++;

    if(hal.spindle_encoder_on_index)
        hal.spindle_encoder_on_index(spindle_encoder.counter.index_count);
}

#endif // SPINDLE_ENCODER_ENABLE

#if QEI_ENABLE && defined(QEI_PORT)

#include "grbl/task.h"
#include "grbl/encoders.h"

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t af;
    GPIO_TypeDef *port_a;
    GPIO_TypeDef *port_b;
    TIM_TypeDef *timer;
} stm32_encoder_hw_t;

typedef struct {
    encoder_t encoder;
    encoder_data_t data;
    encoder_cfg_t settings;
    encoder_event_t event;
    void *context;
    int32_t count;
    int16_t count_h;
    int32_t vel_count;
    volatile uint32_t vel_timeout;
    uint32_t vel_timestamp;
    const stm32_encoder_hw_t *st_encoder;
    encoder_on_event_ptr on_event;
} stm32_encoder_t;

static const stm32_encoder_hw_t encoders[] = {
    { .port_a = GPIOA, .pin_a = 6, .port_b = GPIOA, .pin_b = 7, .af = GPIO_AF2_TIM3, .timer = timer(3) }
};

static stm32_encoder_t qei = {0};

static encoder_data_t *encoder_get_data (encoder_t *encoder)
{
    return &qei.data;
}

static bool qei_configure (encoder_t *encoder, encoder_cfg_t *settings)
{
    if(qei.vel_timeout != settings->vel_timeout)
        qei.vel_timestamp = hal.get_elapsed_ticks();

    memcpy(&qei.settings, settings, sizeof(encoder_cfg_t));

    return true;
}

static void encoder_reset (encoder_t *encoder)
{
    qei.vel_timeout = 0;
    qei.data.position = qei.count = qei.vel_count = 0;
    qei.vel_timestamp = uwTick;
//    qei.vel_timeout = qei.encoder.axis != 0xFF ? QEI_VELOCITY_TIMEOUT : 0;
    qei.st_encoder->timer->CNT = 0; //stop/start timer?
}

static void encoder_poll (void *data)
{
    if(qei.on_event && qei.count != qei.st_encoder->timer->CNT) {

        qei.data.position = (qei.count_h << 16) | qei.st_encoder->timer->CNT;

        qei.event.position_changed = On;

        // encoder->timer->CR1 & TIM_CR1_DIR -> 0 = up, !=0 down

        qei.on_event(&qei.encoder, &qei.event, qei.context);

        if(qei.vel_timeout && !(--qei.vel_timeout)) {
            qei.data.velocity = abs(qei.count - qei.vel_count) * 1000 / (uwTick - qei.vel_timestamp);
            qei.vel_timestamp = uwTick;
            qei.vel_timeout = qei.settings.vel_timeout;
            if((qei.event.position_changed = qei.data.velocity == 0))
                qei.on_event(&qei.encoder, &qei.event, qei.context);
            qei.vel_count = qei.count;
        }
    }
}

static void encoder_overflow (void *context)
{
    stm32_encoder_t *qei = (stm32_encoder_t *)context;

    if(qei->st_encoder->timer->CR1 & TIM_CR1_DIR)
        qei->count_h++;
    else
        qei->count_h--;
}

static bool encoder_claim (encoder_on_event_ptr event_handler, void *context)
{
    if(event_handler == NULL || qei.on_event)
        return false;

    qei.context = context;
    qei.on_event = event_handler;
    qei.encoder.reset = encoder_reset;
    qei.encoder.get_data = encoder_get_data;
    qei.encoder.configure = qei_configure;
    qei.st_encoder->timer->DIER |= TIM_DIER_UIE;
    qei.st_encoder->timer->CR1 = TIM_CR1_CEN|TIM_CR1_URS;

    task_add_systick(encoder_poll, NULL);

    return true;
}

void driver_encoders_init (void)
{
    hal_timer_t timer;

    const stm32_encoder_hw_t *encoder = &encoders[0];

    if((timer = timer_claim(encoder->timer))) {

        timer_clk_enable(encoder->timer);

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL
        };

        timer_cfg_t cfg = {
            .context = &qei,
            .timeout_callback = encoder_overflow
        };

        GPIO_Init.Pin = (1 << encoder->pin_a);
        GPIO_Init.Alternate = encoder->af;
        HAL_GPIO_Init(encoder->port_a, &GPIO_Init);

        GPIO_Init.Pin = (1 << encoder->pin_b);
        GPIO_Init.Alternate = encoder->af;
        HAL_GPIO_Init(encoder->port_b, &GPIO_Init);

        encoder->timer->SMCR = TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_ETF_0|TIM_SMCR_ETF_1;
        encoder->timer->ARR = 0xFFFF;
        encoder->timer->CCMR1 = TIM_CCMR1_CC1S_0|TIM_CCMR1_CC2S_0;

        qei.st_encoder = encoder;
        qei.encoder.claim = encoder_claim;

        timerCfg(timer, &cfg);

        encoder_register(&qei.encoder);
    }
}

#endif
