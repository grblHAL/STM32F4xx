/*
  st_morpho.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#if defined(BOARD_MORPHO_CNC)

#include <math.h>
#include <string.h>

#include "main.h"
#include "spi.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

#if TRINAMIC_SPI_ENABLE

static struct {
    GPIO_TypeDef *port;
    uint32_t pin;
} cs;

static uint_fast8_t n_motors;
static TMC_spi_datagram_t datagram[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    static TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
    volatile uint32_t dly = 100;

    datagram[driver.seq].addr.value = reg->addr.value;
    datagram[driver.seq].addr.write = 0;

    DIGITAL_OUT(cs.port, cs.pin, 0);

    do {
        spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
    } while(idx);

    while(--dly);

    DIGITAL_OUT(cs.port, cs.pin, 1);

    dly = 50;
    while(--dly);

    DIGITAL_OUT(cs.port, cs.pin, 0);

    idx = n_motors;
    do {
        res = spi_put_byte(datagram[--idx].addr.value);

        if(idx == driver.seq) {
            status = res;
            reg->payload.data[3] = spi_get_byte();
            reg->payload.data[2] = spi_get_byte();
            reg->payload.data[1] = spi_get_byte();
            reg->payload.data[0] = spi_get_byte();
        } else {
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
        }
    } while(idx);

    dly = 100;
    while(--dly);

    DIGITAL_OUT(cs.port, cs.pin, 1);

    dly = 50;
    while(--dly);

    spi_set_speed(f_spi);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
    volatile uint32_t dly = 100;

    memcpy(&datagram[driver.seq], reg, sizeof(TMC_spi_datagram_t));
    datagram[driver.seq].addr.write = 1;

    DIGITAL_OUT(cs.port, cs.pin, 0);

    do {
        res = spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(datagram[idx].payload.data[3]);
        spi_put_byte(datagram[idx].payload.data[2]);
        spi_put_byte(datagram[idx].payload.data[1]);
        spi_put_byte(datagram[idx].payload.data[0]);

        if(idx == driver.seq) {
            status = res;
            datagram[idx].addr.idx = 0; // TMC_SPI_STATUS_REG;
            datagram[idx].addr.write = 0;
        }
    } while(idx);

    while(--dly);

    DIGITAL_OUT(cs.port, cs.pin, 1);

    dly = 50;
    while(--dly);

    spi_set_speed(f_spi);

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->function == Output_MotorChipSelect) {
        cs.pin = gpio->pin;
        cs.port = (GPIO_TypeDef *)gpio->port;
    }
}

static void if_init (uint8_t motors, axes_signals_t axisflags)
{
    n_motors = motors;
    hal.enumerate_pins(true, add_cs_pin, NULL);
}

#endif


void board_init (void)
{
#if TRINAMIC_SPI_ENABLE

    trinamic_driver_if_t driver = {
        .on_drivers_init = if_init
    };

    spi_init();

    uint_fast8_t idx = TMC_N_MOTORS_MAX;
    do {
        datagram[--idx].addr.idx = 0; //TMC_SPI_STATUS_REG;
    } while(idx);

    trinamic_if_init(&driver);

#elif TRINAMIC_UART_ENABLE

    extern void tmc_uart_init (void);

    tmc_uart_init();

    #endif
}

#endif
