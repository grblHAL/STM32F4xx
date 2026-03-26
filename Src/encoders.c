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

#include "grbl/task.h"
#include "grbl/encoders.h"
#include "grbl/spindle_sync.h"

typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t af;
    GPIO_TypeDef *port_a;
    GPIO_TypeDef *port_b;
    TIM_TypeDef *timer;
} stm32_qei_hw_t;

typedef struct {
    uint8_t pin;
    uint8_t af;
    bool ecm; // external clock mode
    GPIO_TypeDef *port;
    TIM_TypeDef *timer;
} stm32_pcnt_hw_t;

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
    spindle_encoder_t sp;
    hal_timer_t t;
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *ccr; // shortcut to timer CCR3 register
    const stm32_qei_hw_t *st_encoder;
    encoder_on_event_ptr on_event;
} stm32_encoder_t;

#if SPINDLE_ENCODER_ENABLE

typedef struct {
    int16_t count_h;
    __IO uint32_t *cr1; // shortcut to timer CR1 register
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *ccr; // shortcut to timer CCR3 register
    TIM_TypeDef *timer;
    settings_changed_ptr settings_changed;
} spindle_encoder_hw_t;

static struct {
    uint32_t count_h;
    __IO uint32_t *cnt; // shortcut to timer CNT register
    __IO uint32_t *egr; // shortcut to timer EGR register
} timestamp;

#define TIMESTAMP               (*timestamp.cnt | timestamp.count_h)
#define TIMESTAMP_RESOLUTION    1 // microseconds

#ifdef SPINDLE_PULSE_PIN

static const stm32_pcnt_hw_t counters[] = {
#ifdef TIM8
    { .port = GPIOA, .pin = 0, .af = GPIO_AF3_TIM8, .timer = timer(8), .ecm = false },
#endif
#if !IS_TIMER_CLAIMED(TIM2_BASE)
    { .port = GPIOA, .pin = 5, .af = GPIO_AF1_TIM2, .timer = timer(2), .ecm = false },
    { .port = GPIOA, .pin = 15, .af = GPIO_AF1_TIM2, .timer = timer(2), .ecm = false },
#endif
#if !IS_TIMER_CLAIMED(TIM1_BASE)
    { .port = GPIOA, .pin = 12, .af = GPIO_AF1_TIM1, .timer = timer(1), .ecm = false },
#endif
#if !IS_TIMER_CLAIMED(TIM3_BASE)
    { .port = GPIOB, .pin = 4, .af = GPIO_AF2_TIM3, .timer = timer(3), .ecm = true },
    { .port = GPIOD, .pin = 2, .af = GPIO_AF2_TIM3, .timer = timer(3), .ecm = false }
#endif
};

#endif

static spindle_encoder_hw_t sp_encoder;
static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;

    spindle_encoder_counter_t encoder;

//    while(spindle_encoder.spin_lock);

    __disable_irq();

    if(spindle_data.ccw != !!(sp_encoder.timer->CR1 & TIM_CR1_DIR)) {
        if((spindle_data.ccw = !spindle_data.ccw) /*&& spindle_encoder.timer.pulse_length == 0*/)
            *sp_encoder.ccr = (uint16_t)(*sp_encoder.cnt - spindle_encoder.tics_per_irq);
        else
            *sp_encoder.ccr = (uint16_t)(*sp_encoder.cnt + spindle_encoder.tics_per_irq);
    }

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = TIMESTAMP - spindle_encoder.timer.last_pulse;

    // if 16 bit RPM timer and TIMESTAMP < spindle_encoder.timer.last_pulse then what?

    __enable_irq();

    // If no spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (uint16_t)(((uint16_t)*sp_encoder.cnt - (uint16_t)encoder.last_count)) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)*sp_encoder.cnt - (uint16_t)encoder.last_count);
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

    timestamp.count_h = 0;
    *timestamp.egr |= TIM_EGR_UG; // Reset timestamp timer

    sp_encoder.timer->CR1 &= ~TIM_CR1_CEN;

    spindle_data.ccw = !!(sp_encoder.timer->CR1 & TIM_CR1_DIR);
    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = TIMESTAMP;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    sp_encoder.timer->EGR |= TIM_EGR_UG;
    *sp_encoder.ccr = spindle_data.ccw ? -spindle_encoder.tics_per_irq : spindle_encoder.tics_per_irq;
    sp_encoder.timer->CR1 |= TIM_CR1_CEN;
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

static void spindle_encoder_cfg (settings_t *settings, settings_changed_flags_t changed)
{
    static const spindle_data_ptrs_t encoder_data = {
        .get = spindleGetData,
        .reset = spindleDataReset
    };

    static bool event_claimed = false;

    sp_encoder.settings_changed(settings, changed);

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
            if(spindle_encoder.tics_per_irq & 0x1) spindle_encoder.tics_per_irq++;
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = 250000UL / TIMESTAMP_RESOLUTION; // 250ms
            spindle_encoder.rpm_factor = (60.0f * 1000000.0f / TIMESTAMP_RESOLUTION) / (float)spindle_encoder.ppr;
            spindleDataReset();
        }
    } else {
        spindle_encoder.ppr = 0;
        hal.spindle_data.reset = NULL;
        hal.driver_cap.spindle_encoder_index_event = Off;
    }

    spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);
}

ISR_CODE void spindle_encoder_index_event (void)
{
    uint32_t rpm_count = *sp_encoder.cnt;
    spindle_encoder.timer.last_index = TIMESTAMP;

    if(spindle_encoder.counter.index_count && (uint16_t)(rpm_count - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
        spindle_encoder.error_count++;

    spindle_encoder.counter.last_index = rpm_count;
    spindle_encoder.counter.index_count++;

    if(hal.spindle_encoder_on_index)
        hal.spindle_encoder_on_index(spindle_encoder.counter.index_count);
}

ISR_CODE void spindle_encoder_irq (void *context)
{
    spindle_encoder.spin_lock = true;

    spindle_encoder_hw_t *qei = (spindle_encoder_hw_t *)context;

    __disable_irq();
    uint32_t tval = TIMESTAMP;
    uint16_t cval = *qei->cnt;
    __enable_irq();

    if(*qei->cr1 & TIM_CR1_DIR)
        *qei->ccr = (uint16_t)(*qei->ccr - spindle_encoder.tics_per_irq);
    else
        *qei->ccr = (uint16_t)(*qei->ccr + spindle_encoder.tics_per_irq);

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

ISR_CODE static void spindle_encoder_overflow (void *context)
{
    spindle_encoder_hw_t *qei = (spindle_encoder_hw_t *)context;

    if(qei->timer->CR1 & TIM_CR1_DIR)
        qei->count_h--;
    else
        qei->count_h++;
}

ISR_CODE static void tstamp_overflow (void *context)
{
    timestamp.count_h = ((timestamp.count_h >> 16) + 1) << 16;
}

#endif // SPINDLE_ENCODER_ENABLE

#if QEI_ENABLE && defined(QEI_PORT)

static const stm32_qei_hw_t encoders[] = {
    { .port_a = GPIOA, .pin_a = 6, .port_b = GPIOA, .pin_b = 7, .af = GPIO_AF2_TIM3, .timer = timer(3) }
};

static uint_fast8_t n_encoders = 0;
static stm32_encoder_t qei[QEI_ENABLE] = {0};

static encoder_data_t *encoder_get_data (encoder_t *encoder)
{
    return &((stm32_encoder_t *)encoder->hw)->data;
}

ISR_CODE static void encoder_overflow (void *context)
{
    spindle_encoder_hw_t *qei = (spindle_encoder_hw_t *)context;

    if(qei->timer->CR1 & TIM_CR1_DIR)
        qei->count_h--;
    else
        qei->count_h++;
}

static bool encoder_configure (encoder_t *encoder, encoder_cfg_t *settings)
{
    stm32_encoder_t *qei = (stm32_encoder_t *)encoder->hw;

    if(qei->vel_timeout != settings->vel_timeout)
        qei->vel_timestamp = hal.get_elapsed_ticks();

    memcpy(&qei->settings, settings, sizeof(encoder_cfg_t));

    timer_cfg_t cfg = {
        .context = qei,
        .timeout_callback = encoder_overflow
    };

    timerCfg(qei->t, &cfg);

    return true;
}

static void encoder_reset (encoder_t *encoder)
{
    stm32_encoder_t *qei = (stm32_encoder_t *)encoder->hw;

    qei->vel_timeout = 0;
    qei->data.position = qei->count = qei->vel_count = 0;
    qei->vel_timestamp = uwTick;
//    qei->vel_timeout = qei->encoder.axis != 0xFF ? QEI_VELOCITY_TIMEOUT : 0;
    qei->st_encoder->timer->CNT = 0; //stop/start timer?
}

static void encoder_poll (void *data)
{
    stm32_encoder_t *qei = (stm32_encoder_t *)data;

    if(qei->on_event && qei->count != qei->st_encoder->timer->CNT) {

        qei->data.position = (qei->count_h << 16) | qei->st_encoder->timer->CNT;

        qei->event.position_changed = On;

        // encoder->timer->CR1 & TIM_CR1_DIR -> 0 = up, !=0 down

        qei->on_event(&qei->encoder, &qei->event, qei->context);

        if(qei->vel_timeout && !(--qei->vel_timeout)) {
            qei->data.velocity = abs(qei->count - qei->vel_count) * 1000 / (uwTick - qei->vel_timestamp);
            qei->vel_timestamp = uwTick;
            qei->vel_timeout = qei->settings.vel_timeout;
            if((qei->event.position_changed = qei->data.velocity == 0))
                qei->on_event(&qei->encoder, &qei->event, qei->context);
            qei->vel_count = qei->count;
        }

        qei->count = qei->st_encoder->timer->CNT;
    }
}

static bool encoder_claim (encoder_t *encoder, encoder_on_event_ptr event_handler, void *context)
{
    stm32_encoder_t *qei = (stm32_encoder_t *)encoder->hw;

    if(event_handler == NULL || qei->on_event)
        return false;

    timer_cfg_t cfg = {
        .context = qei,
        .encoder_mode = true,
        .timeout_callback = encoder_overflow
    };

    timerCfg(qei->t, &cfg);

    qei->context = context;
    qei->on_event = event_handler;
    qei->encoder.reset = encoder_reset;
    qei->encoder.get_data = encoder_get_data;
    qei->encoder.configure = encoder_configure;
    qei->st_encoder->timer->DIER |= TIM_DIER_UIE;
    qei->st_encoder->timer->CR1 = TIM_CR1_CEN|TIM_CR1_URS;

    task_add_systick(encoder_poll, qei);

    return true;
}

static bool encoder_add (uint32_t id)
{
    if(id > sizeof(encoders) / sizeof(stm32_qei_hw_t) || n_encoders == QEI_ENABLE)
        return false;

    hal_timer_t timer;
    const stm32_qei_hw_t *encoder = &encoders[id];

    if((timer = timer_claim(encoder->timer))) {

        stm32_encoder_t *hw = &qei[n_encoders++];

        timer_clk_enable(encoder->timer);

        GPIO_InitTypeDef GPIO_Init = {
            .Speed = GPIO_SPEED_FREQ_LOW,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL
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

        hw->t = timer;
        hw->cnt = &encoder->timer->CNT;
        hw->ccr = &encoder->timer->CCR3;
        hw->st_encoder = encoder;
        hw->encoder.hw = hw;
        hw->encoder.claim = encoder_claim;
//        hw->encoder.caps.spindle_rpm = On;

#if SPINDLE_ENCODER_ENABLE

        if(sp_encoder.settings_changed == NULL && timer_get_cap(encoder->timer).comp3) {

            sp_encoder.settings_changed = hal.settings_changed;
            hal.settings_changed = spindle_encoder_cfg;

            sp_encoder.cr1 = &encoder->timer->CR1;
            sp_encoder.cnt = &encoder->timer->CNT;
            sp_encoder.ccr = &encoder->timer->CCR3;
            sp_encoder.timer = encoder->timer;

            timer_cfg_t cfg = {
                .context = &sp_encoder,
                .timeout_callback = timer_get_resolution(encoder->timer) == Timer_16bit ? spindle_encoder_overflow : NULL,
                .irq2_callback = spindle_encoder_irq
            };

            timerCfg(timer, &cfg);

            encoder->timer->CR1 = TIM_CR1_CEN|TIM_CR1_URS;

        } else
#endif
        encoder_register(&hw->encoder);
    }

    return !!timer;
}

#endif // QEI_ENABLE && defined(QEI_PORT)

void encoder_pin_claimed (uint8_t port, xbar_t *pin)
{
#if QEI_ENABLE && defined(QEI_A_PIN) && defined(QEI_B_PIN)
    _encoder_pin_claimed(port, pin);
#endif
}

#if SPINDLE_ENCODER_ENABLE || (QEI_ENABLE && defined(QEI_PORT))

void driver_encoders_init (void)
{
#if SPINDLE_ENCODER_ENABLE

    hal_timer_t timer;

    if((timer = timer_claim(RPM_TIMER))) {

        RPM_TIMER->PSC = timer_clk_enable(RPM_TIMER) / 1000000UL * TIMESTAMP_RESOLUTION - 1;
        if(timer_get_resolution(RPM_TIMER) == Timer_16bit) {
            timer_cfg_t cfg = {
                .timeout_callback = tstamp_overflow
            };
            timerCfg(timer, &cfg);
        }
        RPM_TIMER->CR1 = TIM_CR1_CKD_1|TIM_CR1_URS|TIM_CR1_CEN;

        timestamp.cnt = &RPM_TIMER->CNT;
        timestamp.egr = &RPM_TIMER->EGR;
    }

#ifdef SPINDLE_PULSE_PIN

    uint_fast8_t idx = sizeof(counters) / sizeof(stm32_pcnt_hw_t);

    if(idx) do {
        idx--;
        if(counters[idx].port == SPINDLE_PULSE_PORT && counters[idx].pin == SPINDLE_PULSE_PIN && (timer = timer_claim(counters[idx].timer))) {

            sp_encoder.timer = counters[idx].timer;

            timer_clk_enable(sp_encoder.timer);

            sp_encoder.cr1 = &sp_encoder.timer->CR1;
            sp_encoder.cnt = &sp_encoder.timer->CNT;
            sp_encoder.ccr = &sp_encoder.timer->CCR1;
            sp_encoder.timer->PSC = 0;
            sp_encoder.timer->ARR = 65535;
            sp_encoder.timer->CCER = TIM_CCER_CC1E;
            sp_encoder.timer->SMCR = counters[idx].ecm ? (TIM_SMCR_SMS_0|TIM_SMCR_SMS_1|TIM_SMCR_SMS_2|TIM_SMCR_ETF_2|TIM_SMCR_ETF_3|TIM_SMCR_TS_0|TIM_SMCR_TS_2) : TIM_SMCR_ECE;

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

            timer_cfg_t cfg = {
                .context = &sp_encoder,
                .timeout_callback = timer_get_resolution(sp_encoder.timer) == Timer_16bit ? spindle_encoder_overflow : NULL,
                .irq0_callback = spindle_encoder_irq
            };

            timerCfg(timer, &cfg);

            static const periph_pin_t ssp = {
                .function = Input_SpindlePulse,
                .group = PinGroup_SpindlePulse,
                .port = SPINDLE_PULSE_PORT,
                .pin = SPINDLE_PULSE_PIN,
                .mode = { .mask = PINMODE_NONE }
            };

            hal.periph_port.register_pin(&ssp);

            sp_encoder.settings_changed = hal.settings_changed;
            hal.settings_changed = spindle_encoder_cfg;

            break;
        }
    } while(idx);

#endif // SPINDLE_PULSE_PIN
#endif // SPINDLE_ENCODER_ENABLE

#ifdef QEI_PORT
    encoder_add(QEI_PORT);
#endif
}

#endif
