/*
  driver_spindles.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2025 Terje Io

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

#if DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE

#include <math.h>

#include "pwm.h"

#include "grbl/pin_bits_masks.h"
#include "grbl/nvs_buffer.h"
#include "grbl/task.h"

static settings_changed_ptr settings_changed;

#if PPI_ENABLE

#include "laser/ppi.h"

static void (*ppi_spindle_on)(spindle_ptrs_t *spindle) = NULL;
static void (*ppi_spindle_off)(spindle_ptrs_t *spindle) = NULL;
static spindle_ptrs_t *ppi_spindle = NULL;
static hal_timer_t ppi_timer;

static void spindlePulseOff (void *context)
{
    ppi_spindle_off(ppi_spindle);
}

static void spindlePulseOn (spindle_ptrs_t *spindle, uint_fast16_t pulse_length)
{
    hal.timer.start(ppi_timer, pulse_length);
    ppi_spindle_on(spindle);
}

#endif

#if DRIVER_SPINDLE_ENABLE

static spindle_id_t spindle_id = -1;

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

static spindle_pwm_t spindle_pwm = { .offset = -1 };
static pwm_signal_t spindle_timer = {};

#endif // SPINDLE_PWM

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, settings.pwm_spindle.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
    }
  #elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
  #endif
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #ifdef SPINDLE_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, !settings.pwm_spindle.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
    }
  #elif defined(SPINDLE_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !settings.pwm_spindle.invert.on);
#endif
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
    DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, ccw ^ settings.pwm_spindle.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off(spindle);
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

// Variable spindle control functions

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

static void pwm_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        *spindle_timer.ccr = spindle->context.pwm->off_value;
        if(spindle_timer.timer == TIM1)
            spindle_timer.timer->BDTR |= TIM_BDTR_MOE;
//??        *spindle_timer.ccr = pwm_value;
    } else {
        if(spindle_timer.timer == TIM1)
            spindle_timer.timer->BDTR &= ~TIM_BDTR_MOE;
        else
            *spindle_timer.ccr = 0;
    }
}

// Sets spindle speed
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                *spindle_timer.ccr = spindle->context.pwm->pwm_overdrive;
        } else
            pwm_off(spindle);

    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle_on(spindle);

        *spindle_timer.ccr = pwm_value;
        if(spindle_timer.timer == TIM1)
            spindle_timer.timer->BDTR |= TIM_BDTR_MOE;
    }
}

static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle_off(spindle);
        pwm_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle_off(spindle);
        else {
            spindle_on(spindle);
            spindleSetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

static bool spindleConfig (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if(spindle_timer.timer) {

        uint32_t prescaler = 1, clock_hz = pwm_get_clock_hz(&spindle_timer);

        if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_hz / prescaler)) {

            while(spindle_pwm.period > 65534) {
                prescaler++;
                spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.pwm_spindle, clock_hz / prescaler);
            }

            pwm_config(&spindle_timer, prescaler, spindle_pwm.period, spindle_pwm.flags.invert_pwm);

            spindle_pwm.flags.invert_pwm = Off; // Handled in hardware
            spindle->set_state = spindleSetStateVariable;

            if((spindle_pwm.flags.laser_mode_disable = settings.pwm_spindle.flags.laser_mode_disable)) {
#if PPI_ENABLE
                if(ppi_spindle == spindle)
                    ppi_spindle = NULL;
                spindle->pulse_on = NULL;
            }
            if(ppi_timer) {
                spindle->pulse_on = spindlePulseOn;
                ppi_spindle = spindle;
                ppi_spindle_on = spindle_on;
                ppi_spindle_off = spindle_off;
#endif
            }
        } else {
            if(spindle->context.pwm->flags.enable_out)
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
            spindle->pulse_on = NULL;
            spindle->set_state = spindleSetState;
        }
    } else // PWM output not available
        spindle->set_state = spindleSetState;

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#endif // SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {settings.pwm_spindle.invert.mask};

#ifdef SPINDLE_ENABLE_PIN
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
#endif
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
#endif
    state.value ^= settings.pwm_spindle.invert.mask;
#ifdef SPINDLE_PWM_PIN
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->state_programmed.encoder_error;
    }
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

static spindle_id_t spindle1_id = -1;
static spindle1_pwm_settings_t *spindle_config;

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static spindle_pwm_t spindle1_pwm = { .offset = -1 };
static pwm_signal_t spindle1_timer = {};

#endif // SPINDLE_PWM

// Static spindle (off, on cw & on ccw)

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #ifdef SPINDLE1_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, spindle_config->cfg.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
    }
  #elif defined(SPINDLE1_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
  #endif
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle_config->cfg.invert.on);
#endif
}

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #ifdef SPINDLE1_DIRECTION_PIN
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, !spindle_config->cfg.invert.ccw);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
    }
  #elif defined(SPINDLE1_ENABLE_PIN)
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle_config->cfg.invert.on);
#endif
}

inline static void spindle1_dir (bool ccw)
{
#ifdef SPINDLE1_DIRECTION_PIN
    DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, ccw ^ spindle_config->cfg.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle
static void spindle1SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle1_off(spindle);
    else {
        spindle1_dir(state.ccw);
        spindle1_on(spindle);
    }
}

// Variable spindle control functions

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static void pwm1_off (spindle_ptrs_t *spindle)
{
    if(spindle->context.pwm->flags.always_on) {
        *spindle1_timer.ccr = spindle->context.pwm->off_value;
        if(spindle1_timer.timer == TIM1)
            spindle1_timer.timer->BDTR |= TIM_BDTR_MOE;
//??        *spindle_timer.ccr = pwm_value;
    } else {
        if(spindle1_timer.timer == TIM1)
            spindle1_timer.timer->BDTR &= ~TIM_BDTR_MOE;
        else
            *spindle1_timer.ccr = 0;
    }
}

// Sets spindle speed
static void spindle1SetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {

        if(spindle->context.pwm->flags.rpm_controlled) {
            spindle1_off(spindle);
            if(spindle->context.pwm->flags.laser_off_overdrive)
                *spindle1_timer.ccr = spindle->context.pwm->pwm_overdrive;
        } else
            pwm1_off(spindle);

    } else {

        if(!spindle->context.pwm->flags.enable_out && spindle->context.pwm->flags.rpm_controlled)
            spindle1_on(spindle);

        *spindle1_timer.ccr = pwm_value;
        if(spindle1_timer.timer == TIM1)
            spindle1_timer.timer->BDTR |= TIM_BDTR_MOE;
    }
}

static uint_fast16_t spindle1GetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle
static void spindle1SetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on)) {
        spindle1_off(spindle);
        pwm1_off(spindle);
    } else {
#ifdef SPINDLE_DIRECTION_PIN
        if(!spindle->context.pwm->flags.cloned)
            spindle1_dir(state.ccw);
#endif
        if(rpm == 0.0f && spindle->context.pwm->flags.rpm_controlled)
            spindle1_off(spindle);
        else {
            spindle1_on(spindle);
            spindle1SetSpeed(spindle, spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false));
        }
    }
}

static bool spindle1Config (spindle_ptrs_t *spindle)
{
    if(spindle == NULL)
        return false;

    if(spindle1_timer.timer) {

        uint32_t prescaler = 1, clock_hz = pwm_get_clock_hz(&spindle1_timer);

        if(spindle_precompute_pwm_values(spindle, &spindle1_pwm, &spindle_config->cfg, clock_hz / prescaler)) {

            while(spindle1_pwm.period > 65534) {
                prescaler++;
                spindle_precompute_pwm_values(spindle, &spindle1_pwm, &spindle_config->cfg, clock_hz / prescaler);
            }

            pwm_config(&spindle1_timer, prescaler, spindle1_pwm.period, spindle1_pwm.flags.invert_pwm);

            spindle1_pwm.flags.invert_pwm = Off; // Handled in hardware
            spindle->set_state = spindle1SetStateVariable;

#if PPI_ENABLE
            if(ppi_spindle == NULL && ppi_timer) {
                spindle->pulse_on = spindlePulseOn;
                ppi_spindle = spindle;
                ppi_spindle_on = spindle1_on;
                ppi_spindle_off = spindle1_off;
            } else if(ppi_spindle != spindle)
                spindle->pulse_on = NULL;
#endif

        } else {
            if(spindle->context.pwm->flags.enable_out)
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
            spindle->pulse_on = NULL;
            spindle->set_state = spindle1SetState;
        }
    } else // PWM output not available
        spindle->set_state = spindle1SetState;

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle1_pwm : NULL);

    return true;
}

#endif // SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {spindle_config->cfg.invert.mask};

#ifdef SPINDLE1_ENABLE_PIN
    state.on = DIGITAL_IN(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN);
#endif
#ifdef SPINDLE1_DIRECTION_PIN
    state.ccw = DIGITAL_IN(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN);
#endif
    state.value ^= spindle_config->cfg.invert.mask;

#ifdef SPINDLE1_PWM_PIN
    state.on |= spindle->param->state.on;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle->get_data) {
        spindle_data_t *spindle_data = spindle->get_data(SpindleData_AtSpeed);
        state.at_speed = spindle_data->state_programmed.at_speed;
        state.encoder_error = spindle_data->state_programmed.encoder_error;
    }
#endif

    return state;
}

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static void spindle1_settings_changed (spindle1_pwm_settings_t *settings)
{
    spindle1Config(spindle_get_hal(spindle1_id, SpindleHAL_Configured));
}

#endif

#endif // DRIVER_SPINDLE1_ENABLE

// Configures peripherals when settings are initialized or changed
static void settingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }
#endif

    //settings.offset_lock.encoder_spindle

}

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    if(aux_ctrl->function == Output_SpindlePWM) {
        const pwm_signal_t *pwm_timer;
        if((pwm_timer = pwm_claim(aux_ctrl->gpio.port, aux_ctrl->gpio.pin)))
            memcpy(&spindle_timer, pwm_timer, sizeof(pwm_signal_t));
        else {
            aux_ctrl->port = IOPORT_UNASSIGNED;
            return false;
        }
    }
#endif

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
    if(aux_ctrl->function == Output_Spindle1PWM) {
        const pwm_signal_t *pwm_timer;
        if((pwm_timer = pwm_claim(aux_ctrl->gpio.port, aux_ctrl->gpio.pin)))
            memcpy(&spindle1_timer, pwm_timer, sizeof(pwm_signal_t));
        else {
            aux_ctrl->port = IOPORT_UNASSIGNED;
            return false;
        }
    }
#endif

    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->port, NULL))) {
        ioport_set_function(pin, aux_ctrl->function, NULL);
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
        if(aux_ctrl->function == Output_SpindlePWM) {
            ((output_signal_t *)aux_ctrl->output)->mode.pwm = On;
            pwm_enable(&spindle_timer);
        }
#endif
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
        if(aux_ctrl->function == Output_Spindle1PWM) {
            ((output_signal_t *)aux_ctrl->output)->mode.pwm = On;
            pwm_enable(&spindle1_timer);
        }
#endif
    } else
        aux_ctrl->port = IOPORT_UNASSIGNED;

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

void driver_spindles_init (void)
{
#if DRIVER_SPINDLE_ENABLE && defined(SPINDLE_ENABLE_PIN)

 #if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM0,
  #else
        .ref_id = SPINDLE_PWM0_NODIR,
  #endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    if(!(spindle_timer.timer && (spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME)) != -1))
        task_run_on_startup(report_warning, "PWM spindle failed to initialize!");

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_ONOFF0_DIR,
  #else
        .ref_id = SPINDLE_ONOFF0,
  #endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

 #endif

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE && defined(SPINDLE1_ENABLE_PIN)

 #if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

    static const spindle_ptrs_t spindle1 = {
        .type = SpindleType_PWM,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
        .ref_id = SPINDLE_PWM1,
  #else
        .ref_id = SPINDLE_PWM1_NODIR,
  #endif
        .config = spindle1Config,
        .update_pwm = spindle1SetSpeed,
        .set_state = spindle1SetStateVariable,
        .get_state = spindle1GetState,
        .get_pwm = spindle1GetPWM,
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
            .rpm_range_locked = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    if(spindle1_timer.timer && (spindle_config = spindle1_settings_add(false))) {
        if((spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
            spindle1_settings_register(spindle1.cap, spindle1_settings_changed);
        else
            task_run_on_startup(report_warning, "PWM2 spindle failed to initialize!");
    }

 #else

   static const spindle_ptrs_t spindle1 = {
       .type = SpindleType_Basic,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
       .ref_id = SPINDLE_ONOFF1_DIR,
  #else
       .ref_id = SPINDLE_ONOFF1,
  #endif
       .set_state = spindle1SetState,
       .get_state = spindle1GetState,
       .cap = {
           .gpio_controlled = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
           .direction = On
  #endif
       }
   };

   if((spindle_config = spindle1_settings_add(false)) && (spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
       spindle1_settings_register(spindle1.cap, NULL);

 #endif

#endif // DRIVER_SPINDLE1_ENABLE

    settings_changed = hal.settings_changed;
    hal.settings_changed = settingsChanged;

#if PPI_ENABLE

    static timer_cfg_t cfg = {
        .single_shot = On,
        .timeout_callback = spindlePulseOff
    };

    if((ppi_timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 1000))) {
        hal.timer.configure(ppi_timer, &cfg);
        ppi_init();
    }

#endif
}

#else

bool aux_out_claim_explicit (aux_ctrl_out_t *aux_ctrl)
{
    xbar_t *pin;

    if((pin = ioport_claim(Port_Digital, Port_Output, &aux_ctrl->port, NULL)))
        ioport_set_function(pin, aux_ctrl->function, NULL);
    else
        aux_ctrl->port = IOPORT_UNASSIGNED;

    return aux_ctrl->port != IOPORT_UNASSIGNED;
}

#endif // DRIVER_SPINDLE_ENABLE || DRIVER_SPINDLE1_ENABLE
