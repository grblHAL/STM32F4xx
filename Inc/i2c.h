/*
  i2c.h - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2018-2025 Terje Io

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

#pragma once

#include "driver.h"

#if I2C_ENABLE

#include "grbl/plugins.h"

#ifdef I2C_FASTMODE

#define I2C_GetState           HAL_FMPI2C_GetState
#define I2C_STATE_READY        HAL_FMPI2C_STATE_READY
#define I2C_Mem_Read           HAL_FMPI2C_Mem_Read
#define I2C_Mem_Write          HAL_FMPI2C_Mem_Write
#define I2C_Mem_Write_DMA      HAL_FMPI2C_Mem_Write_DMA
#define I2C_Master_Transmit_IT HAL_FMPI2C_Master_Transmit_IT
#define I2C_Master_Receive_IT  HAL_FMPI2C_Master_Receive_IT
#define I2C_Master_Transmit_DMA HAL_FMPI2C_Master_Transmit_DMA
FMPI2C_HandleTypeDef *I2C_GetPort (void);

#else

#define I2C_GetState           HAL_I2C_GetState
#define I2C_STATE_READY        HAL_I2C_STATE_READY
#define I2C_Mem_Read           HAL_I2C_Mem_Read
#define I2C_Mem_Write          HAL_I2C_Mem_Write
#define I2C_Mem_Write_DMA      HAL_I2C_Mem_Write_DMA
#define I2C_Master_Transmit_IT HAL_I2C_Master_Transmit_IT
#define I2C_Master_Receive_IT  HAL_I2C_Master_Receive_IT
#define I2C_Master_Transmit_DMA HAL_I2C_Master_Transmit_DMA

I2C_HandleTypeDef *I2C_GetPort (void);

#endif

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

#include "motors/trinamic.h"
#include "trinamic/tmc_i2c_interface.h"

#define I2C_ADR_I2CBRIDGE 0x47

#endif

#endif // I2C_ENABLE
