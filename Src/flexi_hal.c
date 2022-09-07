/*
  flexi_hal.c - driver code for STM32F4xx ARM processors on Flexi-HAL board

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io
  Modifications for Flexi-Hal Copyright (c) 2022 Expatria Technologies

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

#if defined(BOARD_FLEXI_HAL)

#include <math.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;
static bool pendant_tx_active = 0;

  #define I2C1_SCL_PIN 6
  #define I2C1_SDA_PIN 7

static FMPI2C_HandleTypeDef i2c_port = {
    .Instance = FMPI2C1,
    //.Init.Timing = 0xC0000E12, //100 KHz
    //.Init.Timing = 0x0020081B, //1000 KHz
    .Init.Timing =0x00401650, //400 KHz
    .Init.OwnAddress1 = 0,
    .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .Init.OwnAddress2 = 0,
    .Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK,
    .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE
};

void i2c_init (void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = (1 << I2C1_SCL_PIN)|(1 << I2C1_SDA_PIN),
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF4_FMPI2C1
    };
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    __HAL_RCC_FMPI2C1_CLK_ENABLE();
    HAL_FMPI2CEx_ConfigAnalogFilter(&i2c_port, FMPI2C_ANALOGFILTER_ENABLE);

    HAL_FMPI2C_Init(&i2c_port);
    __HAL_FMPI2C_ENABLE(&i2c_port);

        /* FMPI2C1 interrupt Init */
    HAL_NVIC_SetPriority(FMPI2C1_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(FMPI2C1_EV_IRQn);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = GPIOC,
        .pin = 6,
        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = GPIOC,
        .pin = 7,
        .mode = { .mask = PINMODE_OD }
    };


    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);
}

void FMPI2C1_EV_IRQHandler(void)
{
  HAL_FMPI2C_EV_IRQHandler(&i2c_port);
}

void FMPI2C1_ER_IRQHandler(void)
{
  HAL_FMPI2C_ER_IRQHandler(&i2c_port);
}

void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
	//wait for bus to be ready
	while (HAL_FMPI2C_GetState(&i2c_port) != HAL_FMPI2C_STATE_READY);

    HAL_FMPI2C_Master_Transmit_IT(&i2c_port,  i2cAddr<<1, buf, bytes);

    if (block)
    	while (HAL_FMPI2C_GetState(&i2c_port) != HAL_FMPI2C_STATE_READY);
}

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{
    while (HAL_FMPI2C_GetState(&i2c_port) != HAL_FMPI2C_STATE_READY);

    HAL_StatusTypeDef ret;
    
    if(read)
        ret = HAL_FMPI2C_Mem_Read(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes, i2c->data, i2c->count, 100);
    else {
        ret = HAL_FMPI2C_Mem_Write(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes, i2c->data, i2c->count, 100);
#if !EEPROM_IS_FRAM
        hal.delay_ms(6, NULL);
#endif
    }
    i2c->data += i2c->count;

    return ret == HAL_OK ? NVS_TransferResult_OK : NVS_TransferResult_Failed;
}

void I2C_PendantRead (uint32_t i2cAddr, uint8_t memaddress, uint8_t size, uint8_t * data, keycode_callback_ptr callback)
{
    
    uint32_t ms = hal.get_elapsed_ticks();  //50 ms timeout
    uint32_t timeout_ms = ms + 50;
    
    if(keypad_callback != NULL || pendant_tx_active) //we are in the middle of a read
        return;

    keycode = 'r';
    keypad_callback = callback;

    while((HAL_FMPI2C_Mem_Read_IT(&i2c_port, i2cAddr << 1, memaddress, 1, data, size)) != HAL_OK){
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

    while((HAL_FMPI2C_Master_Transmit_IT(&i2c_port,  i2cAddr<<1, buf, bytes) != HAL_OK)){
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

void HAL_FMPI2C_MasterRxCpltCallback(FMPI2C_HandleTypeDef *hi2c)
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

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    keycode = 0;
    keypad_callback = callback;

    HAL_FMPI2C_Master_Receive_IT(&i2c_port, i2cAddr << 1, &keycode, 1);
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
//put new stuff here.


}

#endif


