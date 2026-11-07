/*
  driver_spindles.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io

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

static settings_changed_ptr on_settings_changed;

#if PPI_ENABLE

#include "laser/ppi.h"

static hal_timer_t ppi_timer;
static laser_ppi_t laser_ppi = {0};

static void spindlePulseOff (void *context)
{
    laser_ppi.spindle_off(laser_ppi.spindle);
}

static void spindlePulseOn (spindle_ptrs_t *spindle, uint_fast16_t pulse_length)
{
    hal.timer.start(ppi_timer, pulse_length);
    laser_ppi.spindle_on(spindle);
}

#endif // PPI_ENABLE

#if DRIVER_SPINDLE_ENABLE

static spindle_id_t spindle_id = -1;

// Static spindle (off, on cw & on ccw)

#if !(DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) // PWM only spindle

inline static void spindle_on (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = On;
}

inline static void spindle_off (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = Off;
}

inline static void spindle_dir (bool ccw)
{
    UNUSED(ccw);
}

#else

inline static void spindle_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, spindle->context.pwm->settings->invert.on);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
    }
  #else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
  #endif
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
#endif
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, !spindle->context.pwm->settings->invert.on);
    } else {
        DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
    }
  #else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
#endif
}

inline static void spindle_dir (spindle_ptrs_t *spindle, bool ccw)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(!(spindle->context.pwm->flags.cloned || spindle->cap.clone)) {
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, ccw ^ spindle->context.pwm->settings->invert.ccw);
    }
#else
    UNUSED(ccw);
    UNUSED(spindle);
#endif
}

#endif

// Start or stop spindle
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!(spindle->context.pwm->flags.cloned ? state.ccw : state.on))
        spindle_off(spindle);
    else {
        spindle_dir(spindle, state.ccw);
        spindle_on(spindle);
    }
}

// Variable spindle control functions

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM

static pwm_signal_t spindle_timer = {};

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
        spindle_dir(spindle, state.ccw);
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
    static spindle_pwm_t spindle_pwm = { .offset = -1 };

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

#if PPI_ENABLE
            if(spindle_pwm.flags.laser_mode_disable) {
                if(laser_ppi.spindle == spindle)
                    laser_ppi.spindle = NULL;
                spindle->pulse_on = NULL;
            } else if(ppi_timer) {
                laser_ppi.spindle = spindle;
                laser_ppi.spindle_on = spindle_on;
                laser_ppi.spindle_off = spindle_off;
                spindle->pulse_on = spindlePulseOn;
            }
#endif
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

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    on_settings_changed(settings, changed);

    if(changed.spindle) {
        spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
        if(spindle_id == spindle_get_default())
            spindle_select(spindle_id);
    }
}

#endif // SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { spindle->context.pwm->settings->invert.mask };

#if (DRIVER_SPINDLE_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) == 0 // PWM only spindle
    state.on = spindle->context.pwm->flags.enable_out ^ state.on;
#else
    state.on = DIGITAL_IN(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
    if(spindle->cap.direction)
        state.ccw = DIGITAL_IN(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
  #endif
#endif

    state.value ^= spindle->context.pwm->settings->invert.mask;

#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
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

// Static spindle (off, on cw & on ccw)

#if (DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) == 0 // PWM only spindle

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = On;
}

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
    spindle->context.pwm->flags.enable_out = Off;
}

inline static void spindle1_dir (bool ccw)
{
    UNUSED(ccw);
}

#else

inline static void spindle1_off (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
    spindle->context.pwm->flags.enable_out = Off;
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, spindle->context.pwm->settings->invert.on);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
    }
  #else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
  #endif
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, spindle->context.pwm->settings->invert.on);
#endif
}

inline static void spindle1_on (spindle_ptrs_t *spindle)
{
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->context.pwm->flags.cloned) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, !spindle->context.pwm->settings->invert.on);
    } else {
        DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
    }
  #else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
  #endif
  #if SPINDLE_ENCODER_ENABLE
    if(!spindle->context.pwm->flags.enable_out && spindle->reset_data)
        spindle->reset_data();
  #endif
    spindle->context.pwm->flags.enable_out = On;
#else
    DIGITAL_OUT(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN, !spindle->context.pwm->settings->invert.on);
#endif
}

inline static void spindle1_dir (spindle_ptrs_t *spindle, bool ccw)
{
#if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(!(spindle->context.pwm->flags.cloned || spindle->cap.clone)) {
        DIGITAL_OUT(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN, ccw ^ spindle->context.pwm->settings->invert.ccw);
#else
    UNUSED(ccw);
    UNUSED(spindle);
#endif
}

#endif

// Start or stop spindle
static void spindle1SetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!(spindle->context.pwm->flags.cloned ? state.ccw : state.on))
        spindle1_off(spindle);
    else {
        spindle1_dir(spindle, state.ccw);
        spindle1_on(spindle);
    }
}

// Variable spindle control functions

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM

static pwm_signal_t spindle1_timer = {};

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
        spindle1_dir(spindle, state.ccw);
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
    static spindle_pwm_t spindle_pwm = { .offset = -1 };

    if(spindle == NULL)
        return false;

    if(spindle1_timer.timer) {

        uint32_t prescaler = 1, clock_hz = pwm_get_clock_hz(&spindle1_timer);

        if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &spindle_config->cfg, clock_hz / prescaler)) {

            while(spindle_pwm.period > 65534) {
                prescaler++;
                spindle_precompute_pwm_values(spindle, &spindle_pwm, &spindle_config->cfg, clock_hz / prescaler);
            }

            pwm_config(&spindle1_timer, prescaler, spindle_pwm.period, spindle_pwm.flags.invert_pwm);

            spindle_pwm.flags.invert_pwm = Off; // Handled in hardware
            spindle->set_state = spindle1SetStateVariable;

#if PPI_ENABLE
            if(spindle_pwm.flags.laser_mode_disable) {
                if(laser_ppi.spindle == spindle)
                    laser_ppi.spindle = NULL;
                spindle->pulse_on = NULL;
            } else if(laser_ppi.spindle == NULL && ppi_timer) {
                laser_ppi.spindle = spindle;
                laser_ppi.spindle_on = spindle1_on;
                laser_ppi.spindle_off = spindle1_off;
                spindle->pulse_on = spindlePulseOn;
            }
#endif
        } else {
            if(spindle->context.pwm->flags.enable_out)
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
            spindle->pulse_on = NULL;
            spindle->set_state = spindle1SetState;
        }
    } else // PWM output not available
        spindle->set_state = spindle1SetState;

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

static void spindle1_settings_changed (spindle1_pwm_settings_t *settings)
{
    spindle1Config(spindle_get_hal(spindle1_id, SpindleHAL_Configured));
}

#endif // SPINDLE_PWM

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindle1GetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = { spindle->context.pwm->settings->invert.mask };

#if !(DRIVER_SPINDLE1_ENABLE & (SPINDLE_ENA|SPINDLE_DIR)) // PWM only spindle
    state.on = spindle->context.pwm->flags.enable_out ^ state.on;
#else
    state.on = DIGITAL_IN(SPINDLE1_ENABLE_PORT, SPINDLE1_ENABLE_PIN);
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
    if(spindle->cap.direction)
        state.ccw = DIGITAL_IN(SPINDLE1_DIRECTION_PORT, SPINDLE1_DIRECTION_PIN);
  #endif
#endif

    state.value ^= spindle->context.pwm->settings->invert.mask;

#if DRIVER_SPINDLE1_ENABLE & SPINDLE_PWM
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

#endif // DRIVER_SPINDLE1_ENABLE

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
#if DRIVER_SPINDLE_ENABLE

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
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
            .enable = On,
  #endif
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    if(!(spindle_timer.timer && (spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME)) != -1))
        task_run_on_startup(report_warning, "PWM spindle failed to initialize!");

    on_settings_changed = grbl.on_settings_changed;
    grbl.on_settings_changed = onSettingsChanged;

 #elif DRIVER_SPINDLE_ENABLE & SPINDLE_ENA

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
            .enable = On,
  #if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
            .direction = On
  #endif
        }
    };

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

 #endif

#endif // DRIVER_SPINDLE_ENABLE

#if DRIVER_SPINDLE1_ENABLE

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
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA
           .enable = On,
  #endif
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

 #elif DRIVER_SPINDLE1_ENABLE & SPINDLE_ENA

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
           .enable = On,
  #if DRIVER_SPINDLE1_ENABLE & SPINDLE_DIR
           .direction = On
  #endif
       }
   };

   if((spindle_config = spindle1_settings_add(false)) && (spindle1_id = spindle_register(&spindle1, DRIVER_SPINDLE1_NAME)) != -1)
       spindle1_settings_register(spindle1.cap, NULL);

 #endif

#endif // DRIVER_SPINDLE1_ENABLE

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
