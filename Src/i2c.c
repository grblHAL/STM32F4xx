/*
  i2c.c - I2C support for EEPROM, keypad and Trinamic plugins

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

#include <main.h>

#include "i2c.h"
#include "grbl/hal.h"

#if I2C_ENABLE

#ifdef I2C_FASTMODE

#ifndef I2C_KHZ
#define I2C_KHZ 400
#endif

#define I2CPORT FMPI2C1
#define I2C_SCL_PIN 6
#define I2C_SDA_PIN 7
#define I2C_GPIO GPIOC
#define I2C_GPIO_AF GPIO_AF4_FMPI2C1
#define I2C_CLKENA __HAL_RCC_FMPI2C1_CLK_ENABLE
#define I2C_IRQEVT FMPI2C1_EV_IRQn
#define I2C_IRQERR FMPI2C1_ER_IRQn
#define I2C_IRQEVT_Handler FMPI2C1_EV_IRQHandler
#define I2C_IRQERR_Handler FMPI2C1_ER_IRQHandler

#define I2C_EV_IRQHandler HAL_FMPI2C_EV_IRQHandler
#define I2C_ER_IRQHandler HAL_FMPI2C_ER_IRQHandler

#else

#ifndef I2C_KHZ
#define I2C_KHZ 100
#endif

#define I2C_GPIO GPIOB

#if I2C_PORT == 1

#ifdef I2C1_ALT_PINMAP
  #define I2C_SCL_PIN 6
  #define I2C_SDA_PIN 7
#else
  #define I2C_SCL_PIN 8
  #define I2C_SDA_PIN 9
#endif

#else // 2

#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 11

#endif

#define I2Cport(p) I2CportI(p)
#define I2CportI(p) I2C ## p
#define I2CportCLK(p) I2CportCLKI(p)
#define I2CportCLKI(p) __HAL_RCC_I2C ## p ## _CLK_ENABLE
#define I2CportAF(p) I2CportAFI(p)
#define I2CportAFI(p) GPIO_AF4_I2C ## p
#define I2CportEvt(p, e) I2CportEvtI(p, e)
#define I2CportEvtI(p, e) I2C ## p ## _ ## e ## _IRQn
#define I2CportHandler(p, e) I2CportHandlerI(p, e)
#define I2CportHandlerI(p, e) I2C ## p ## _ ## e ## _IRQHandler

#define I2CPORT I2Cport(I2C_PORT)
#define I2C_IRQEVT I2CportEvt(I2C_PORT, EV)
#define I2C_IRQERR I2CportEvt(I2C_PORT, ER)
#define I2C_IRQEVT_Handler I2CportHandler(I2C_PORT, EV)
#define I2C_IRQERR_Handler I2CportHandler(I2C_PORT, ER)
#define I2C_CLKENA I2CportCLK(I2C_PORT)
#define I2C_GPIO_AF I2CportAF(I2C_PORT)

#define I2C_EV_IRQHandler HAL_I2C_EV_IRQHandler
#define I2C_ER_IRQHandler HAL_I2C_ER_IRQHandler

#endif

#ifdef I2C_FASTMODE

static FMPI2C_HandleTypeDef i2c_port = {
    .Instance = I2CPORT,
#if I2C_KHZ == 100
    .Init.Timing = 0xC0000E12, //100 KHz
#elif I2C_KHZ == 1000
    .Init.Timing = 0x0020081B, //1000 KHz
#else
    .Init.Timing = 0x00401650, //400 KHz
#endif
    .Init.OwnAddress1 = 0,
    .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .Init.OwnAddress2 = 0,
    .Init.OwnAddress2Masks = FMPI2C_OA2_NOMASK,
    .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE
};

FMPI2C_HandleTypeDef *I2C_GetPort (void)
{
    return &i2c_port;
}

#else

static I2C_HandleTypeDef i2c_port = {
    .Instance = I2CPORT,
    .Init.ClockSpeed = I2C_KHZ * 1000,
    .Init.DutyCycle = I2C_DUTYCYCLE_2,
    .Init.OwnAddress1 = 0,
    .Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    .Init.DualAddressMode = I2C_DUALADDRESS_DISABLE,
    .Init.OwnAddress2 = 0,
    .Init.GeneralCallMode = I2C_GENERALCALL_DISABLE,
    .Init.NoStretchMode = I2C_NOSTRETCH_DISABLE
};

I2C_HandleTypeDef *I2C_GetPort (void)
{
    return &i2c_port;
}

#endif

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;

void i2c_init (void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin = (1 << I2C_SCL_PIN)|(1 << I2C_SDA_PIN),
        .Mode = GPIO_MODE_AF_OD,
        .Pull = GPIO_PULLUP,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = I2C_GPIO_AF
    };
    HAL_GPIO_Init(I2C_GPIO, &GPIO_InitStruct);

    I2C_CLKENA();

#ifdef I2C_FASTMODE
    HAL_FMPI2C_Init(&i2c_port);
    HAL_FMPI2CEx_ConfigAnalogFilter(&i2c_port, FMPI2C_ANALOGFILTER_ENABLE);
#else
    HAL_I2C_Init(&i2c_port);
#endif

    HAL_NVIC_EnableIRQ(I2C_IRQEVT);
    HAL_NVIC_EnableIRQ(I2C_IRQERR);

    static const periph_pin_t scl = {
        .function = Output_SCK,
        .group = PinGroup_I2C,
        .port = I2C_GPIO,
        .pin = I2C_SCL_PIN,
        .mode = { .mask = PINMODE_OD }
    };

    static const periph_pin_t sda = {
        .function = Bidirectional_SDA,
        .group = PinGroup_I2C,
        .port = I2C_GPIO,
        .pin = I2C_SDA_PIN,
        .mode = { .mask = PINMODE_OD }
    };

    hal.periph_port.register_pin(&scl);
    hal.periph_port.register_pin(&sda);
}

bool i2c_probe (uint_fast16_t i2cAddr)
{
    //wait for bus to be ready
    while (I2C_GetState(&i2c_port) != I2C_STATE_READY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

#ifdef I2C_FASTMODE
    return HAL_FMPI2C_IsDeviceReady(&i2c_port, i2cAddr << 1, 4, 10) == HAL_OK;
#else
    return HAL_I2C_IsDeviceReady(&i2c_port, i2cAddr << 1, 4, 10) == HAL_OK;
#endif
}

bool i2c_send (uint_fast16_t i2cAddr, uint8_t *buf, size_t size, bool block)
{
    //wait for bus to be ready
    while (I2C_GetState(&i2c_port) != I2C_STATE_READY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    bool ok = I2C_Master_Transmit_IT(&i2c_port, i2cAddr << 1, buf, size) == HAL_OK;

    if (ok && block) {
        while (I2C_GetState(&i2c_port) != I2C_STATE_READY) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }

    return ok;
}

void i2c_get_keycode (uint_fast16_t i2cAddr, keycode_callback_ptr callback)
{
    keycode = 0;
    keypad_callback = callback;

    I2C_Master_Receive_IT(&i2c_port, i2cAddr << 1, &keycode, 1);
}

void I2C_IRQEVT_Handler (void)
{
    I2C_EV_IRQHandler(&i2c_port);
}

void I2C_IRQERR_Handler (void)
{
    I2C_ER_IRQHandler(&i2c_port);
}

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{
    while (I2C_GetState(&i2c_port) != I2C_STATE_READY) {
        if(!hal.stream_blocking_callback())
            return NVS_TransferResult_Failed;
    }

    //    while (HAL_I2C_IsDeviceReady(&i2c_port, (uint16_t)(0xA0), 3, 100) != HAL_OK);
    HAL_StatusTypeDef ret;

    if(read)
        ret = I2C_Mem_Read(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes == 2 ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, i2c->data, i2c->count, 100);
    else {
        ret = I2C_Mem_Write(&i2c_port, i2c->address << 1, i2c->word_addr, i2c->word_addr_bytes == 2 ? I2C_MEMADD_SIZE_16BIT : I2C_MEMADD_SIZE_8BIT, i2c->data, i2c->count, 100);
#if !EEPROM_IS_FRAM
        hal.delay_ms(5, NULL);
#endif
    }
    i2c->data += i2c->count;

    return ret == HAL_OK ? NVS_TransferResult_OK : NVS_TransferResult_Failed;
}

#endif // EEPROM_ENABLE

#ifdef I2C_FASTMODE
void HAL_FMPI2C_MasterRxCpltCallback (FMPI2C_HandleTypeDef *hi2c)
#else
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
#endif
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static uint16_t axis = 0xFF;
static const uint8_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[5] = {0};
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        I2C_Mem_Write(&i2c_port, tmc_addr, axis, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

        axis = driver.axis;
    }

    I2C_Mem_Read(&i2c_port, tmc_addr, (uint16_t)reg->addr.idx, I2C_MEMADD_SIZE_8BIT, buffer, 5, 100);

    status = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[4];
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        buffer[0] = driver.axis | 0x80;
        I2C_Mem_Write(&i2c_port, tmc_addr, axis, I2C_MEMADD_SIZE_8BIT, buffer, 1, 100);

        axis = driver.axis;
    }

    buffer[0] = (reg->payload.value >> 24) & 0xFF;
    buffer[1] = (reg->payload.value >> 16) & 0xFF;
    buffer[2] = (reg->payload.value >> 8) & 0xFF;
    buffer[3] = reg->payload.value & 0xFF;

    reg->addr.write = 1;
    I2C_Mem_Write(&i2c_port, tmc_addr, (uint16_t)reg->addr.idx, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    reg->addr.write = 0;

    return status;
}

#endif // TRINAMIC_ENABLE && TRINAMIC_I2C

#endif // I2C_ENABLE
