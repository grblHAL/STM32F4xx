/*
  Devtronic_CNC_Controller.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io & nickshl

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if defined(BOARD_DEVTRONIC_CNC)

static xbar_t rx_pin;
static on_execute_realtime_ptr on_execute_realtime, on_execute_delay;

static bool     mpg_input_state = false;
static uint32_t mpg_counter = 0u;
static uint32_t last_ms = 0u;

static void uart_rx_poll (void)
{
    uint32_t ms = hal.get_elapsed_ticks();

    if(last_ms != ms)
    {
        last_ms = ms;

        // Check if pin state isn't changed
        if(mpg_input_state == (DIGITAL_IN((GPIO_TypeDef *)rx_pin.port, rx_pin.pin) == 1))
        {
            // Check 100 times
            if(mpg_counter < 100u) mpg_counter++;
            // On 101 time
            else if(mpg_counter == 100u)
            {
                // Try to switch mode
                stream_mpg_enable(mpg_input_state);
                // And increase counter to prevent entering this statement again
                mpg_counter++;
            }
            else
            {
                ; // Do nothing - MISRA rule
            }
        }
        else // If pin state have changed
        {
            // Save new pin state
            mpg_input_state = (DIGITAL_IN((GPIO_TypeDef *)rx_pin.port, rx_pin.pin) == 1);
            // And clear the counter
            mpg_counter = 0u;
        }
    }
}

static void uart_poll_realtime (sys_state_t grbl_state)
{
    on_execute_realtime(grbl_state);

    uart_rx_poll();
}

static void uart_poll_delay (sys_state_t grbl_state)
{
    on_execute_delay(grbl_state);

    uart_rx_poll();
}

static void get_rx_pin (xbar_t *pin, void *data)
{
    if((pin->function == Input_RX) && (pin->group == (PinGroup_UART + (uint8_t)((uint32_t)data))))
    {
        memcpy(&rx_pin, pin, sizeof(xbar_t));
    }
}

static bool claim_stream (io_stream_properties_t const *sstream)
{
    if((sstream->type == StreamType_Serial) && !sstream->flags.claimed)
    {
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(sstream->instance, 115200, NULL, NULL), false, NULL);

        if(hal.driver_cap.mpg_mode)
        {
            hal.enumerate_pins(true, get_rx_pin, (void *)((uint32_t)sstream->instance));
        }
    }

    return hal.driver_cap.mpg_mode;
}

void board_init (void)
{
    if(!hal.driver_cap.mpg_mode && stream_enumerate_streams(claim_stream) && rx_pin.port)
    {
        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = uart_poll_realtime;

        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = uart_poll_delay;
    }
}

#endif
