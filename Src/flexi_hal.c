/*
  flexi_hal.c - driver code for STM32F4xx ARM processors on Flexi-HAL board

  Part of grblHAL

  Copyright (c) 2022 Expatria Technologies

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if defined(BOARD_FLEXI_HAL) && KEYPAD_ENABLE

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;
static bool pendant_tx_active = 0;

static FMPI2C_HandleTypeDef *i2c_port;

void I2C_PendantRead (uint32_t i2cAddr, uint8_t memaddress, uint8_t size, uint8_t * data, keycode_callback_ptr callback)
{

    uint32_t ms = hal.get_elapsed_ticks();  //50 ms timeout
    uint32_t timeout_ms = ms + 50;

    if(keypad_callback != NULL || pendant_tx_active) //we are in the middle of a read
        return;

    keycode = 'r';
    keypad_callback = callback;

    while((HAL_FMPI2C_Mem_Read_IT(i2c_port, i2cAddr << 1, memaddress, 1, data, size)) != HAL_OK){
        if (ms > timeout_ms){
            keypad_callback = NULL;
            keycode = 0;
            i2c_init();
            return;
        }
        hal.delay_ms(1, NULL);
        ms = hal.get_elapsed_ticks();
    }
}

void I2C_PendantWrite (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes)
{

    uint32_t ms = hal.get_elapsed_ticks();  //50 ms timeout
    uint32_t timeout_ms = ms + 50;

    if(keypad_callback != NULL || pendant_tx_active) //we are in the middle of a read
        return;
    pendant_tx_active = 1;

    while((HAL_FMPI2C_Master_Transmit_IT(i2c_port,  i2cAddr<<1, buf, bytes) != HAL_OK)){
        if (ms > timeout_ms){
            pendant_tx_active = 0;
            i2c_init();
            return;
        }
        hal.delay_ms(1, NULL);
        ms = hal.get_elapsed_ticks();
    }
}

void HAL_FMPI2C_MemRxCpltCallback(FMPI2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}

void HAL_FMPI2C_MasterTxCpltCallback(FMPI2C_HandleTypeDef *hi2c)
{
    pendant_tx_active = 0;
}

// called from stream drivers while tx is blocking, returns false to terminate

#if 0
bool flexi_stream_tx_blocking (void)
{
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.

    grbl.on_execute_realtime(state_get());

    return !(sys.rt_exec_state & EXEC_RESET);
}
#endif

void board_init (void)
{
    i2c_port = I2C_GetPort();
}

#endif
