/*
  btt_skr_1.1.c - driver code for STM32F407xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 fitch22, Terje Io

  Some software serial code is ported from Arduino.  Credit belongs to the many
  authors that contributed to that project.

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

#if defined(BOARD_BTT_SKR_PRO_1_1)

#if TRINAMIC_SPI_ENABLE

#include "trinamic/common.h"

static SPI_HandleTypeDef spi_port = {
    .Instance = SPI3,
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

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

// XXXXX replace with something better...
inline static void delay (void)
{
    volatile uint32_t dly = 10;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t spi_get_byte (void)
{
    spi_port.Instance->DR = 0xFF; // Writing dummy data into Data register

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    return (uint8_t)spi_port.Instance->DR;
}

static uint8_t spi_put_byte (uint8_t byte)
{
    spi_port.Instance->DR = byte;

    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_TXE));
    while(!__HAL_SPI_GET_FLAG(&spi_port, SPI_FLAG_RXNE));

    __HAL_SPI_CLEAR_OVRFLAG(&spi_port);

    return (uint8_t)spi_port.Instance->DR;
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
    delay();
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

static void add_cs_pin (xbar_t *gpio)
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
    static bool init_ok = false;

    UNUSED(motors);

    if (!init_ok) {

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
            .function = Output_SCK,
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

        HAL_SPI_Init(&spi_port);
        __HAL_SPI_ENABLE(&spi_port);

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);
        hal.enumerate_pins(true, add_cs_pin);
    }
}

void board_init (void)
{
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    trinamic_if_init(&driver_if);
}

#endif  // TRINAMIC_SPI_ENABLE

#if TRINAMIC_UART_ENABLE

#include "trinamic/common.h"

#define SWS_BAUDRATE            100000      // 10us bit period, near upper limit @ 168MHz
#define START_DELAY             46          // delay in us * timer clock freq
#define ABORT_TIMEOUT           5           // ms
#define TWELVE_BIT_TIMES        1           // in ms rounded up (1 is smallest we can go)
#define HALFDUPLEX_SWITCH_DELAY 4           // defined in bit-periods
#define RCV_BUF_SIZE            16          // read packet is 8 bytes

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} tmc_uart_t;

typedef struct {
    volatile int_fast16_t bit_count;
    volatile bool busy;
    volatile uint_fast16_t data;
} tmc_uart_tx_buffer_t;

typedef struct {
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    volatile int_fast16_t bit_count;
    volatile uint_fast16_t irq_count;
    bool overflow;
    bool busy;
    uint8_t data[RCV_BUF_SIZE];
} tmc_uart_rx_buffer_t;

static TIM_HandleTypeDef htim7 = {
    .Instance = TIM7,
    .Init.Prescaler = 0,
    .Init.CounterMode = TIM_COUNTERMODE_UP,
    .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE
};

static uint16_t period_div_2;
static tmc_uart_tx_buffer_t tx_buf;
static tmc_uart_rx_buffer_t rx_buf;
static tmc_uart_t uart[TMC_N_MOTORS_MAX], *active_uart;

static inline void setTX ()
{
    static GPIO_InitTypeDef GPIO_InitStruct = {
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL
    };

    GPIO_InitStruct.Pin = 1 << active_uart->pin;
    HAL_GPIO_Init(active_uart->port, &GPIO_InitStruct);

    DIGITAL_OUT(uart->port, uart->pin, 1);
}

static inline void setRX (void)
{
    static GPIO_InitTypeDef GPIO_InitStruct = {
        .Mode = GPIO_MODE_INPUT,
        .Pull = GPIO_PULLUP
    };

    GPIO_InitStruct.Pin = 1 << active_uart->pin;
    HAL_GPIO_Init(active_uart->port, &GPIO_InitStruct);
}

/**
  * @brief Software Serial send byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  */
static inline void send (void)
{
    if(tx_buf.bit_count++ < 9) {
        DIGITAL_OUT(active_uart->port, active_uart->pin, tx_buf.data & 1);  // drive bit out
        tx_buf.data >>= 1;                                                  // shift to next bit
    } else {
        DIGITAL_OUT(active_uart->port, active_uart->pin, 1);                // STOP bit (1)
        tx_buf.busy = false;                                                // we are done with STOP bit
    }
}

static void write_n (uint8_t data[], uint32_t length)
{
    uint_fast8_t i = 0;

    while(tx_buf.busy);                             // should not be anything pending but...

    tx_buf.data = data[i++] << 1;                   // form first word with START bit (0)
    tx_buf.bit_count = 0;                           // reset bit counter
    tx_buf.busy = true;
    length--;

    __HAL_TIM_SET_COUNTER(&htim7, START_DELAY);     // initialize counter with START delay
    htim7.Instance->CR1 &= ~TIM_CR1_UDIS;           // enable interrupt
    send();                                         // force first bit without interrupt

    while(tx_buf.busy);                             // wait for 1st byte to finish

    do {
        tx_buf.data = data[i++] << 1;               // form next word
        tx_buf.bit_count = 0;
        tx_buf.busy = true;
        while(tx_buf.busy);                         // wait for byte to finish
    } while(--length);

    tx_buf.busy = true;                             // wait one more bit period
    while(tx_buf.busy);                             // for the last STOP bit

    htim7.Instance->CR1 |= TIM_CR1_UDIS;            // disable interrupt
}

/**
  * @brief Software Serial Read Byte
  * @param None
  * @retval Byte read
  *
  * Returns the next byte from the receive buffer or -1 on underflow.
  */
static int16_t read_byte (void)
{
    if (rx_buf.tail == rx_buf.head)
        return -1;

    int16_t byte = (int16_t)rx_buf.data[rx_buf.tail];   // get next byte
    rx_buf.tail = BUFNEXT(rx_buf.tail, rx_buf);

    return byte;
}

/**
  * @brief Software Serial receive byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  * Tihs is only called if tx_busy == true;
  */
static void rcv (void)
{
    static volatile uint32_t rx_byte;

    bool inbit = DIGITAL_IN(active_uart->port, active_uart->pin);

//    hal.port.digital_out(4, 1);

    if(rx_buf.bit_count == -1) {                                // -1 means waiting for START (0)
        if(!inbit) {
            rx_buf.bit_count = 0;                               // 0: START bit received
            rx_byte = 0;
        }
    } else if(rx_buf.bit_count >= 8) {                          // >= 8 means waiting for STOP
        if(inbit) {
            uint_fast16_t next = BUFNEXT(rx_buf.head, rx_buf);
            if (next != rx_buf.tail) {                          // room in buffer?
                rx_buf.data[rx_buf.head] = rx_byte;             // save new byte
                rx_buf.head = next;
            } else                                              // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
                rx_buf.overflow = true;
        }
        rx_buf.bit_count = -1;                                  // wait for next START
    } else {
        rx_byte >>= 1;                                          // shift previous
        if (inbit)
            rx_byte |= 0x80;                                    // OR in new
        rx_buf.bit_count++;                                     // Preprare for next bit
    }

//    hal.port.digital_out(4, 0);
}

static void stop_listening (void)
{
  uint8_t count = rx_buf.irq_count;

  // Need to wait HALFDUPLEX_SWITCH_DELAY sample periods for TMC2209 to release it's transmitter
  /*
   * Some Notes are in order here:
   * There are 20K pulldowns on the PDN_UART pin on the stepper driver board.
   * These pulldowns are small enough to overpower the pull-up on the input to
   * the STM32, and this is enough to cause a glitch as defined by TMC (<16 clocks).
   * So, we know the TMC is driving a 1 since we detected a STOP already, we just
   * want to turn our driver back on just before it turns its driver off so we don't
   * get a glitch.  This is one advantage of the FYSETC stepper drivers over
   * the BTT stepper drivers, they have 1K in series with the PDN_UART pin.
   * Either way, we are not really fighting the PDN_UART pin since both
   * the TMC2209 chip and the STM32 are driving high.
   */
    while ((rx_buf.irq_count + 256 - count) % 256 < HALFDUPLEX_SWITCH_DELAY);

    setTX();      // turn driver on just before the TMC driver goes off

    // Wait one more bit period without which the TMC won't respond
    while ((rx_buf.irq_count + 256 - count) % 256 < (HALFDUPLEX_SWITCH_DELAY + 1));

    // Now we can wrap things up
    rx_buf.busy = false;
    htim7.Instance->CR1 |= TIM_CR1_UDIS; // Disable output event
}

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr)
{
    static TMC_uart_write_datagram_t wdgr = {0};
    static TMC_uart_write_datagram_t bad = {0};

    // Remember the port and pin we are using
    active_uart = &uart[driver.id];

    // claim the semaphore or wait until free
    //
    // purge anything in buffer
    rx_buf.tail = rx_buf.head = 0;

    write_n(rdgr->data, sizeof(TMC_uart_read_datagram_t)); // send read request

    // we just sent last STOP bit, now switch to input and look for START
    setRX();

    // Look for START (0)
    // If read request had CRC error or some other issue, we won't get a reply
    // so we need to timeout
    uint32_t ms = hal.get_elapsed_ticks();

    do {
        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            hal.delay_ms(TWELVE_BIT_TIMES + 1, NULL);
            setTX();                // turn on our driver
            return &bad;            // return {0}
        }
    } while (DIGITAL_IN(active_uart->port, active_uart->pin));

    // Now that we found a START bit, set timer to 1/2 bit width and start interrupts
    // This allows us to sample reliably in the center of each receive bit
    rx_buf.busy = true;
    rx_buf.bit_count = -1; // look for START bit

    __HAL_TIM_SET_COUNTER(&htim7, period_div_2);
    htim7.Instance->CR1 &= ~TIM_CR1_UDIS;

    // Wait for read response
    int16_t res;
    ms = hal.get_elapsed_ticks();

    for (uint32_t i = 0; i < 8;) {

        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            rx_buf.busy = false;
            htim7.Instance->CR1 |= TIM_CR1_UDIS;        // turn off interrupts
            hal.delay_ms(TWELVE_BIT_TIMES + 1, NULL);
            setTX();                                    // turn on our driver
            return &bad;
        }

        if ((res = read_byte()) != -1)
            wdgr.data[i++] = res;
    }

    // purge anything left in buffer
    rx_buf.tail = rx_buf.head = 0;

    stop_listening();

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    // Remember the port and pin we are using
    active_uart = &uart[driver.id];

    write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));
}

void HAL_TIM_Base_MspInit (TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance == TIM7) {
        __HAL_RCC_TIM7_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
    }
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef* htim)
{
    if (tx_buf.busy)
        send();
    else if (rx_buf.busy) {
        rcv();
        rx_buf.irq_count++;
    }
}

static void MX_TIM7_Init (void)
{
    RCC_ClkInitTypeDef clock;
    uint32_t latency, timer_clock_freq;

    // Determine the period based on the timer clock
    HAL_RCC_GetClockConfig(&clock, &latency);

    timer_clock_freq = HAL_RCC_GetPCLK1Freq() * (clock.APB1CLKDivider == 0 ? 1 : 2);
    htim7.Init.Period = (uint16_t)(((float)timer_clock_freq / (float)SWS_BAUDRATE) + 0.5f);

    period_div_2 = (uint16_t)(((float)htim7.Init.Period / 2.0f) + 0.5f);         // save this for use by receive

    if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
        Error_Handler();
    }
}

static void add_uart_pin (xbar_t *gpio)
{
    if (gpio->group == PinGroup_MotorUART)
      switch (gpio->function) {

        case Bidirectional_MotorUARTX:
            uart[X_AXIS].port = (GPIO_TypeDef *)gpio->port;
            uart[X_AXIS].pin = gpio->pin;
            break;

        case Bidirectional_MotorUARTY:
            uart[Y_AXIS].port = (GPIO_TypeDef *)gpio->port;
            uart[Y_AXIS].pin = gpio->pin;
            break;

        case Bidirectional_MotorUARTZ:
            uart[Z_AXIS].port = (GPIO_TypeDef *)gpio->port;
            uart[Z_AXIS].pin = gpio->pin;
            break;

        case Bidirectional_MotorUARTM3:
            uart[3].port = (GPIO_TypeDef *)gpio->port;
            uart[3].pin = gpio->pin;
            break;

        case Bidirectional_MotorUARTM4:
            uart[4].port = (GPIO_TypeDef *)gpio->port;
            uart[4].pin = gpio->pin;
            break;

        case Bidirectional_MotorUARTM5:
            uart[5].port = (GPIO_TypeDef *)gpio->port;
            uart[5].pin = gpio->pin;
            break;

        default:
            break;
    }
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    static bool init_ok = false;

    UNUSED(motors);

    if (!init_ok) {

        MX_TIM7_Init();
        htim7.Instance->CR1 |= TIM_CR1_UDIS;    // Disable update events
        HAL_TIM_Base_Start_IT(&htim7);

        hal.enumerate_pins(true, add_uart_pin);
    }
}

void driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    config->address = 0;
}

void board_init (void)
{
    static trinamic_driver_if_t driver_if = {
        .on_drivers_init = if_init,
        .on_driver_preinit = driver_preinit
    };

    trinamic_if_init(&driver_if);
}

void TIM7_IRQHandler (void)
{
    HAL_TIM_IRQHandler(&htim7);
}

#endif // TRINAMIC_UART_ENABLE

#endif  // BOARD_BTT_SKR_20
