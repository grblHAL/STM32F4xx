/*
  i2c.h - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2018-2023 Terje Io

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

#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include "driver.h"

#if I2C_ENABLE

#include "grbl/plugins.h"

#ifdef I2C_FASTMODE

#define I2C_GetState HAL_FMPI2C_GetState
#define I2C_STATE_READY HAL_FMPI2C_STATE_READY
#define I2C_Mem_Read HAL_FMPI2C_Mem_Read
#define I2C_Mem_Write HAL_FMPI2C_Mem_Write
#define I2C_Master_Transmit_IT HAL_FMPI2C_Master_Transmit_IT
#define I2C_Master_Receive_IT HAL_FMPI2C_Master_Receive_IT

FMPI2C_HandleTypeDef *I2C_GetPort (void);

#else

#define I2C_GetState HAL_I2C_GetState
#define I2C_STATE_READY HAL_I2C_STATE_READY
#define I2C_Mem_Read HAL_I2C_Mem_Read
#define I2C_Mem_Write HAL_I2C_Mem_Write
#define I2C_Master_Transmit_IT HAL_I2C_Master_Transmit_IT
#define I2C_Master_Receive_IT HAL_I2C_Master_Receive_IT

I2C_HandleTypeDef *I2C_GetPort (void);

#endif

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

#include "motors/trinamic.h"
#include "trinamic/tmc_i2c_interface.h"

#define I2C_ADR_I2CBRIDGE 0x47

#endif

void i2c_init (void);
bool i2c_probe (uint_fast16_t i2c_address);
bool i2c_send (uint_fast16_t i2cAddr, uint8_t *buf, size_t size, bool block);
bool i2c_receive (uint_fast16_t i2cAddr, uint8_t *buf, size_t size, bool block);
void i2c_get_keycode (uint_fast16_t i2cAddr, keycode_callback_ptr callback);

#endif // I2C_ENABLE
#endif // __I2C_DRIVER_H__
