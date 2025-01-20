/*
  longboard32.c - driver code for STM32F4xx ARM processors on Sienci SLB boards

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io
  Copyright (c) 2022 Expatria Technologies

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

#include "grbl/protocol.h"
#include "grbl/state_machine.h"

#if defined(BOARD_LONGBOARD32) || defined(BOARD_LONGBOARD32_EXT)

#ifndef SLB_TLS_AUX_INPUT
#define SLB_TLS_AUX_INPUT 3
#endif

static probe_state_t tls_input = {
    .connected = On
};

static probe_state_t state = {
    .connected = On
};

static bool probe_away;
static xbar_t toolsetter;
static driver_setup_ptr driver_setup;
static on_report_options_ptr on_report_options;
static probe_get_state_ptr hal_probe_get_state;
static probe_configure_ptr hal_probe_configure;

extern pin_group_pins_t *get_motor_fault_inputs (void);

static void clear_ringleds (void *data)
{
    if(hal.rgb1.num_devices && hal.rgb1.out) {

        uint_fast8_t idx;
        for(idx = 0; idx < hal.rgb1.num_devices; idx++)
            hal.rgb1.out(idx, (rgb_color_t){ .value = 0 });

        hal.rgb1.write();
    }
}

// redirected probing function for SLB OR.
static probe_state_t getProbeState (void)
{
    //get the probe state from the HAL
    state = hal_probe_get_state();
    //get the probe state from the plugin
    tls_input.triggered = (bool)toolsetter.get_value(&toolsetter) ^ tls_input.inverted;

    //OR the result and return, unless it is an away probe in which case AND the result.
    if(probe_away)
        state.triggered &= tls_input.triggered;
    else
        state.triggered |= tls_input.triggered;

    state.tls_triggered = tls_input.triggered;

    return state;
}

// redirected probing function for SLB OR.
static void probeConfigure (bool is_probe_away, bool probing)
{
    tls_input.triggered = Off;
    tls_input.is_probing = probing;
    tls_input.inverted = is_probe_away ? !settings.probe.invert_toolsetter_input : settings.probe.invert_toolsetter_input;

    probe_away = is_probe_away;

    if(hal_probe_configure)
        hal_probe_configure(is_probe_away, probing);
}

#if TRINAMIC_ENABLE

static on_state_change_ptr on_state_change;

static void onStateChanged (sys_state_t state)
{
    static bool estop = false;

    if(estop && !(state & (STATE_ESTOP|STATE_ALARM))) {
        estop = false;
        if(hal.stepper.status)
            hal.stepper.status(true);
    }

    if(state == STATE_ESTOP)
        estop = true;

    if(on_state_change)
        on_state_change(state);
}

#else

static control_signals_get_state_ptr hal_control_get_state;
static pin_group_pins_t *fault_inputs;
static stepper_status_t stepper_status = {};

static void poll_motor_fault (void *data)
{
    stepper_status.fault.state = 0;

    if(settings.motor_fault_enable.mask) {

        uint_fast8_t idx;

        task_add_delayed(poll_motor_fault, NULL, 25);

        for(idx = 0; idx < fault_inputs->n_pins; idx++) {
            uint8_t axis = xbar_fault_pin_to_axis(fault_inputs->pins.inputs[idx].id);
            if(bit_istrue(settings.motor_fault_enable.mask, bit(axis))) {
                input_signal_t *input = &fault_inputs->pins.inputs[idx];
                if(DIGITAL_IN(input->port, input->pin) ^ input->mode.inverted)
                    xbar_stepper_state_set(&stepper_status.fault, axis, fault_inputs->pins.inputs[idx].id >= Input_MotorFaultX_2);
            }
        }

        if(stepper_status.fault.state && !(state_get() & (STATE_ALARM|STATE_ESTOP))) {
            control_signals_t signals = hal_control_get_state();
            signals.motor_fault = On;
            hal.control.interrupt_callback(signals);
        }
    }
}

static stepper_status_t getDriverStatus (bool reset)
{
    if(reset)
        stepper_status.fault.state = 0;

    return stepper_status;
}

static control_signals_t getControlState (void)
{
    control_signals_t state = hal_control_get_state();

    state.motor_fault = stepper_status.fault.state != 0;

    return state;
}

#endif // TRINAMIC_ENABLE

static bool driverSetup (settings_t *settings)
{
    hal.homing.get_state = NULL; // for now, StallGuard sensorless homing not yet in use. Later check if sensorless homing is enabled.
    hal.home_cap.a.bits = hal.home_cap.b.bits = 0;
    xbar_set_homing_source();

#if !TRINAMIC_ENABLE
    if((hal.signals_cap.motor_fault = settings->motor_fault_enable.value && (fault_inputs = get_motor_fault_inputs()))) {

        task_add_delayed(poll_motor_fault, NULL, 25);

        hal.stepper.status = getDriverStatus;

        hal_control_get_state = hal.control.get_state;
        hal.control.get_state = getControlState;
    }
#endif

    return driver_setup(settings);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("SLB Probing", "0.03");
}

void board_init (void)
{
    xbar_t *port;
    uint8_t tool_probe_port = SLB_TLS_AUX_INPUT;

#if defined(MODBUS_DIR_AUX) && !(MODBUS_ENABLE & MODBUS_RTU_DIR_ENABLED)

    if((port = ioport_get_info(Port_Digital, Port_Output, MODBUS_DIR_AUX)) && !port->mode.claimed) {
        uint8_t modbus_dir = MODBUS_DIR_AUX;
        ioport_claim(Port_Digital, Port_Output, &modbus_dir, "N/A");
    }

#endif

    hal.signals_pullup_disable_cap.probe_triggered = Off; // SLB has isolated inputs

    if((port = ioport_get_info(Port_Digital, Port_Input, tool_probe_port)) && !port->mode.claimed) {

        memcpy(&toolsetter, port, sizeof(xbar_t));

        ioport_claim(Port_Digital, Port_Input, &tool_probe_port, "Toolsetter");

        driver_setup = hal.driver_setup;
        hal.driver_setup = driverSetup;

        hal_probe_get_state = hal.probe.get_state;
        hal.probe.get_state = getProbeState;

        hal_probe_configure = hal.probe.configure;
        hal.probe.configure = probeConfigure;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        hal.driver_cap.toolsetter = On;
    } else
        protocol_enqueue_foreground_task(report_warning, "SLB toolsetter: configured port number is not available!");

    protocol_enqueue_foreground_task(clear_ringleds, NULL);

#if TRINAMIC_ENABLE

    on_state_change = grbl.on_state_change;
    grbl.on_state_change = onStateChanged;

#endif
}

#endif //BOARD_LONGBOARD32
