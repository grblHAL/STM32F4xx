/*
  btt_skr_2.0.c - driver code for STM32F407xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 fitch22

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

#if defined(BOARD_BTT_SKR_20)

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#include "trinamic/common.h"

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static uint16_t cs_pin[N_AXIS];
static GPIO_TypeDef *cs_port[N_AXIS];

static struct {
    GPIO_TypeDef *port;
    uint16_t bit;
} cs[TMC_N_MOTORS_MAX];

// XXXXX replace with something better...
inline static void delay (void)
{
    volatile uint32_t dly = 10;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t sw_spi_xfer (uint8_t byte)
{
    uint_fast8_t msk = 0x80, res = 0;

    DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);

    do {
        DIGITAL_OUT(TRINAMIC_MOSI_PORT, TRINAMIC_MOSI_PIN, (byte & msk) != 0);
        msk >>= 1;
        delay();
        res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, TRINAMIC_MISO_PIN);
        DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);
        delay();
        if(msk)
            DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);
    } while (msk);

    return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 0);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 1);
    delay();
    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 0);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 1);

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 0);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    DIGITAL_OUT(cs_port[driver.axis], cs_pin[driver.axis], 1);

    return status;
}

#endif

static void add_cs_pin (xbar_t *pin)
{
    if(pin->group == PinGroup_MotorChipSelect) {
        switch(pin->function) {

            case Output_MotorChipSelectX:
                cs[X_AXIS].bit = pin->bit;
                cs[X_AXIS].port = (GPIO_TypeDef *)pin->port;
                break;
            case Output_MotorChipSelectY:
                cs[Y_AXIS].bit = pin->bit;
                cs[Y_AXIS].port = (GPIO_TypeDef *)pin->port;
                break;
            case Output_MotorChipSelectZ:
                cs[Z_AXIS].bit = pin->bit;
                cs[Z_AXIS].port = (GPIO_TypeDef *)pin->port;
                break;
            case Output_MotorChipSelectM3:
                cs[3].bit = pin->bit;
                cs[3].port = (GPIO_TypeDef *)pin->port;
                break;
            case Output_MotorChipSelectM4:
                cs[4].bit = pin->bit;
                cs[5].port = (GPIO_TypeDef *)pin->port;
                break;

            default:
                break;
        }
    }

    if(pin->group == PinGroup_StepperPower) {
		if(pin->function == Output_StepperPower) {
			DIGITAL_OUT((GPIO_TypeDef *)(pin->port), pin->pin, 1);
			HAL_Delay(100);
		}
    }
}

#if TRINAMIC_ENABLE == 2209
#endif

static void if_init(axes_signals_t enabled)
{
    static bool init_ok = false;

    if(!init_ok) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        // Set all output pins: push-pull, no pull-up, slow
        GPIO_InitStruct.Pin = TRINAMIC_MOSI_BIT;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(TRINAMIC_MOSI_PORT, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = TRINAMIC_SCK_BIT;
        HAL_GPIO_Init(TRINAMIC_SCK_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);

        GPIO_InitStruct.Pin = MOTOR_CSX_BIT;
        HAL_GPIO_Init(MOTOR_CSX_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(MOTOR_CSX_PORT, MOTOR_CSX_PIN, 1);

        GPIO_InitStruct.Pin = MOTOR_CSY_BIT;
        HAL_GPIO_Init(MOTOR_CSY_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(MOTOR_CSY_PORT, MOTOR_CSY_PIN, 1);

        GPIO_InitStruct.Pin = MOTOR_CSZ_BIT;
        HAL_GPIO_Init(MOTOR_CSZ_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(MOTOR_CSZ_PORT, MOTOR_CSZ_PIN, 1);

#ifdef A_AXIS
        GPIO_InitStruct.Pin = MOTOR_CSM3_BIT;
        HAL_GPIO_Init(MOTOR_CSM3_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(MOTOR_CSM3_PORT, MOTOR_CSM3_PIN, 1);
#endif

#ifdef B_AXIS
        GPIO_InitStruct.Pin = MOTOR_CSM4_BIT;
        HAL_GPIO_Init(MOTOR_CSM4_PORT, &GPIO_InitStruct);
        DIGITAL_OUT(MOTOR_CSM4_PORT, MOTOR_CSM4_PIN, 1);
#endif

        // Set the input pin: input with pull-up
        GPIO_InitStruct.Pin = TRINAMIC_MISO_BIT;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(TRINAMIC_MISO_PORT, &GPIO_InitStruct);

        // Save pin and port information
        // XXXXX This may be redundant with enumerate_pins
        cs_pin[X_AXIS] = MOTOR_CSX_PIN;
        cs_port[X_AXIS] = MOTOR_CSX_PORT;
        cs_pin[Y_AXIS] = MOTOR_CSY_PIN;
        cs_port[Y_AXIS] = MOTOR_CSY_PORT;
        cs_pin[Z_AXIS] = MOTOR_CSZ_PIN;
        cs_port[Z_AXIS] = MOTOR_CSZ_PORT;
#ifdef A_AXIS
        cs_pin[A_AXIS] = MOTOR_CSM3_PIN;
        cs_port[A_AXIS] = MOTOR_CSM3_PORT;
#endif
#ifdef B_AXIS
        cs_bit[B_AXIS] = MOTOR_CSM4_PIN;
        cs_port[B_AXIS] = MOTOR_CSM4_PORT;
#endif

        hal.enumerate_pins(true, add_cs_pin);
    }
}

void board_init (void)
{

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

    static trinamic_driver_if_t driver_if = {
        .on_drivers_init = if_init
    };

    trinamic_if_init(&driver_if);


#endif  // TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#if TRINAMIC_ENABLE == 2209
#endif  // TRINAMIC_ENABLE == 2209

}

#endif  // BOARD_BTT_SKR_20

