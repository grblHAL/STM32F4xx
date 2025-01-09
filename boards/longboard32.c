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

#if defined(BOARD_LONGBOARD32)

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
static probe_get_state_ptr SLB_get_state;
static probe_configure_ptr SLB_probeConfigure;

// redirected probing function for SLB OR.
static probe_state_t probeSLBGetState (void)
{
    //get the probe state from the HAL
    state = SLB_get_state();
    //get the probe state from the plugin
    tls_input.triggered = (bool)toolsetter.get_value(&toolsetter) ^ tls_input.inverted;

    //OR the result and return, unless it is an away probe in which case AND the result.
    if(probe_away)
        state.triggered &= tls_input.triggered;
    else
        state.triggered |= tls_input.triggered;

    return state;
}

// redirected probing function for SLB OR.
static void probeSLBConfigure (bool is_probe_away, bool probing)
{
    tls_input.inverted = is_probe_away ? !settings.probe.invert_toolsetter_input : settings.probe.invert_toolsetter_input;

    tls_input.triggered = Off;

    tls_input.is_probing = probing;

    probe_away = is_probe_away;

    //call the HAL function.
    if(SLB_probeConfigure)
        SLB_probeConfigure(is_probe_away, probing);
}

#if TRINAMIC_ENABLE

static on_state_change_ptr on_state_change;

static void onStateChanged (sys_state_t state)
{
    static bool estop = false;

    if(estop && !(state & (STATE_ESTOP|STATE_ALARM))) {
        estop = false;
        trinamic_drivers_reinit();
    }

    if(state == STATE_ESTOP)
        estop = true;

    if(on_state_change)
        on_state_change(state);
}

#endif // TRINAMIC_ENABLE

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("SLB Probing", "0.02");
}

static bool driverSetup (settings_t *settings)
{
    hal.homing.get_state = NULL; // for now, StallGuard sensorless homing not yet in use. Later check if sensorless homing is enabled.

    return driver_setup(settings);
}

static void clear_ringleds (void *data)
{
    if(hal.rgb1.num_devices && hal.rgb1.out) {

        uint_fast8_t idx;
        for(idx = 0; idx < hal.rgb1.num_devices; idx++)
            hal.rgb1.out(idx, (rgb_color_t){ .value = 0 });

        hal.rgb1.write();
    }
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

        SLB_get_state = hal.probe.get_state;
        hal.probe.get_state = probeSLBGetState;

        SLB_probeConfigure = hal.probe.configure;
        hal.probe.configure = probeSLBConfigure;

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
