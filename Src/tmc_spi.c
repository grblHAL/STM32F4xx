/*
  tmc_spi.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io
  SoftSPI implementation Copyright (c) 2021 fitch22

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

#if TRINAMIC_SPI_ENABLE

#include "trinamic/common.h"

// XXXXX replace with something better...
inline static void delay (uint32_t delay)
{
    volatile uint32_t dly = delay;

    while(--dly)
        __ASM volatile ("nop");
}

#if defined(TRINAMIC_SOFTSPI)

// Per driver CS and soft SPI ports, supports 40 bit datagrams

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_20BIT
#error "20 bit Trinamic SPI datagrams not yet supported!"
#endif

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

static uint8_t sw_spi_xfer (uint8_t byte)
{
  uint_fast8_t msk = 0x80, res = 0;

  DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);

  do {
    DIGITAL_OUT(TRINAMIC_MOSI_PORT, TRINAMIC_MOSI_PIN, (byte & msk) != 0);
    msk >>= 1;
    delay(10);
    res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, TRINAMIC_MISO_PIN);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);
    delay(10);
    if (msk)
      DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 0);
  } while (msk);

  return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
  TMC_spi_status_t status;

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  datagram->payload.value = 0;

  datagram->addr.write = 0;
  spi_put_byte(datagram->addr.value);
  spi_put_byte(0);
  spi_put_byte(0);
  spi_put_byte(0);
  spi_put_byte(0);

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
  delay(10);
  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  status = spi_put_byte(datagram->addr.value);
  datagram->payload.data[3] = spi_get_byte();
  datagram->payload.data[2] = spi_get_byte();
  datagram->payload.data[1] = spi_get_byte();
  datagram->payload.data[0] = spi_get_byte();

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

  return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
  TMC_spi_status_t status;

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

  datagram->addr.write = 1;
  status = spi_put_byte(datagram->addr.value);
  spi_put_byte(datagram->payload.data[3]);
  spi_put_byte(datagram->payload.data[2]);
  spi_put_byte(datagram->payload.data[1]);
  spi_put_byte(datagram->payload.data[0]);

  DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

  return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
  if (gpio->group == PinGroup_MotorChipSelect) {
    switch (gpio->function) {

    case Output_MotorChipSelectX:
      cs[X_AXIS].port = (GPIO_TypeDef *)gpio->port;
      cs[X_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectY:
      cs[Y_AXIS].port = (GPIO_TypeDef *)gpio->port;
      cs[Y_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectZ:
      cs[Z_AXIS].port = (GPIO_TypeDef *)gpio->port;
      cs[Z_AXIS].pin = gpio->pin;
      break;

    case Output_MotorChipSelectM3:
      cs[3].port = (GPIO_TypeDef *)gpio->port;
      cs[3].pin = gpio->pin;
      break;

    case Output_MotorChipSelectM4:
      cs[4].port = (GPIO_TypeDef *)gpio->port;
      cs[4].pin = gpio->pin;
      break;

    default:
      break;
    }
  }
}

static void if_init(uint8_t motors, axes_signals_t enabled)
{
  static bool init_ok = false;

  UNUSED(motors);

  if (!init_ok) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set all output pins: push-pull, no pull-up, slow
    GPIO_InitStruct.Pin = 1 << TRINAMIC_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TRINAMIC_MOSI_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = 1 << TRINAMIC_SCK_PIN;
    HAL_GPIO_Init(TRINAMIC_SCK_PORT, &GPIO_InitStruct);
    DIGITAL_OUT(TRINAMIC_SCK_PORT, TRINAMIC_SCK_PIN, 1);

    // Set the input pin: input with pull-up
    GPIO_InitStruct.Pin = 1 << TRINAMIC_MISO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TRINAMIC_MISO_PORT, &GPIO_InitStruct);

    hal.enumerate_pins(true, add_cs_pin, NULL);
  }
}

void tmc_spi_init (void)
{
  static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

  trinamic_if_init(&driver_if);
}

#else // HW SPI

#ifdef TRINAMIC_SPI_PORT

#define SPIport(p) SPIportI(p)
#define SPIportI(p) SPI ## p

#if SPI_PORT == 11
#define TMC_SPI_PORT SPIport(1)
#else
#define TMC_SPI_PORT SPIport(TRINAMIC_SPI_PORT)
#endif

#define spi_init() _spi_init()
#define spi_get_byte() _spi_get_byte()
#define spi_put_byte(d) _spi_put_byte(d)

static SPI_HandleTypeDef spi_port = {
    .Instance = TMC_SPI_PORT,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
    .Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
    .Init.CRCPolynomial = 10
};

static uint8_t _spi_get_byte (void)
{
    spi_port.Instance->DR = 0xFF; // Writing dummy data into Data register

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    return (uint8_t)spi_port.Instance->DR;
}

static uint8_t _spi_put_byte (uint8_t byte)
{
    spi_port.Instance->DR = byte;

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_TXE));
    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    __HAL_SPI_CLEAR_OVRFLAG(&spi_port);

    return (uint8_t)spi_port.Instance->DR;
}

static void _spi_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

#if TRINAMIC_SPI_PORT == 2

        __HAL_RCC_SPI2_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_2|GPIO_PIN_3,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitTypeDef GPIO_InitStruct2 = {
            .Pin = GPIO_PIN_13,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF5_SPI2
        };
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOB,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 3,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 2,
            .mode = { .mask = PINMODE_NONE }
        };

#elif TRINAMIC_SPI_PORT == 3

        __HAL_RCC_SPI3_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF6_SPI3
        };
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 10,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 11,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOC,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE }
        };

#elif TRINAMIC_SPI_PORT == 4

        __HAL_RCC_SPI4_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = GPIO_AF6_SPI4
        };
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 12,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 14,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = GPIOE,
            .pin = 13,
            .mode = { .mask = PINMODE_NONE }
        };

#endif

        HAL_SPI_Init(&spi_port);
        __HAL_SPI_ENABLE(&spi_port);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);
    }
}

#else // shared SPI port

#include "spi.h"

#endif // TRINAMIC_SPI_PORT

 #if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_CS_SINGLE

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_20BIT
#error "20 bit Trinamic SPI datagrams not yet supported!"
#endif

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
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
#endif

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

    delay(100);
    DIGITAL_OUT(cs.port, cs.pin, 1);
    delay(50);
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

    delay(100);
    DIGITAL_OUT(cs.port, cs.pin, 1);
    delay(50);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
#endif

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

    delay(100);
    DIGITAL_OUT(cs.port, cs.pin, 1);
    delay(50);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

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

void tmc_spi_init (void)
{
    trinamic_driver_if_t driver = {
        .on_drivers_init = if_init
    };

    spi_init();

    uint_fast8_t idx = TMC_N_MOTORS_MAX;
    do {
        datagram[--idx].addr.idx = 0; //TMC_SPI_STATUS_REG;
    } while(idx);

    trinamic_if_init(&driver);
}

#else // separate CS pins

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
#endif

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);
    delay(10);
    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
#endif

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi20_datagram_t tmc_spi20_write (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram)
{
    TMC_spi20_datagram_t status = {0};
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
#else
    while(__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_BSY)) {};
#endif

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 0);

    status.data[2] = spi_put_byte(datagram->data[2]);
    status.data[1] = spi_put_byte(datagram->data[1]);
    status.data[0] = spi_put_byte(datagram->data[0]);

    DIGITAL_OUT(cs[driver.id].port, cs[driver.id].pin, 1);

    status.value >>= 4;

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if (gpio->group == PinGroup_MotorChipSelect)
      switch (gpio->function) {

        case Output_MotorChipSelectX:
            cs[X_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[X_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectY:
            cs[Y_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Y_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectZ:
            cs[Z_AXIS].port = (GPIO_TypeDef *)gpio->port;
            cs[Z_AXIS].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM3:
            cs[3].port = (GPIO_TypeDef *)gpio->port;
            cs[3].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM4:
            cs[4].port = (GPIO_TypeDef *)gpio->port;
            cs[4].pin = gpio->pin;
            break;

        case Output_MotorChipSelectM5:
            cs[5].port = (GPIO_TypeDef *)gpio->port;
            cs[5].pin = gpio->pin;
            break;

        default:
            break;
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    UNUSED(motors);

    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    spi_init();

    trinamic_if_init(&driver_if);
}
#endif
#endif // HW SPI
#endif // TRINAMIC_SPI_ENABLE
