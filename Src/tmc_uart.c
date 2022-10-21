/*
  tmc_uart.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2021-2022 fitch22, Terje Io

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

#if TRINAMIC_UART_ENABLE == 1

#include <string.h>

#include "serial.h"

static io_stream_t tmc_uart;

TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *dgr)
{
    static TMC_uart_write_datagram_t wdgr = {0};
    volatile uint32_t dly = 50, ms = hal.get_elapsed_ticks();

//    tmc_uart.reset_write_buffer();
    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_read_datagram_t));

    while(tmc_uart.get_tx_buffer_count());

    while(--dly);

    tmc_uart.disable_rx(false);
    tmc_uart.reset_read_buffer();

    // Wait for response with 2ms timeout
    while(tmc_uart.get_rx_buffer_count() < 8) {
        if(hal.get_elapsed_ticks() - ms >= 3)
            break;
    }

    if((tmc_uart.get_rx_buffer_count()) >= 8) {
        wdgr.data[0] = tmc_uart.read();
        wdgr.data[1] = tmc_uart.read();
        wdgr.data[2] = tmc_uart.read();
        wdgr.data[3] = tmc_uart.read();
        wdgr.data[4] = tmc_uart.read();
        wdgr.data[5] = tmc_uart.read();
        wdgr.data[6] = tmc_uart.read();
        wdgr.data[7] = tmc_uart.read();
    } else
        wdgr.msg.addr.value = 0xFF;

    tmc_uart.disable_rx(true);

    dly = 5000;
    while(--dly);

    return &wdgr;
}

void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    tmc_uart.write_n((char *)dgr->data, sizeof(TMC_uart_write_datagram_t));
    while(tmc_uart.get_tx_buffer_count());
}

void tmc_uart_init (void)
{
    memcpy(&tmc_uart, serial2Init(230400), sizeof(io_stream_t));

    tmc_uart.disable_rx(true);
    tmc_uart.set_enqueue_rt_handler(stream_buffer_all);
}

#endif // TRINAMIC_UART_ENABLE == 1

#if TRINAMIC_UART_ENABLE == 2

#include "trinamic/common.h"

#ifndef TMC_UART_TIMER_N
#define TMC_UART_TIMER_N        7
#endif
#define TMC_UART_TIMER          timer(TMC_UART_TIMER_N)
#define TMC_UART_IRQn           timerINT(TMC_UART_TIMER_N)
#define TMC_UART_IRQHandler     timerHANDLER(TMC_UART_TIMER_N)
#define TMC_UART_CLKENA         timerCLKENA(TMC_UART_TIMER_N)

#define SWS_BAUDRATE            100000      // 10us bit period, near upper limit @ 168MHz
#define START_DELAY             0 // 46          // delay in us * timer clock freq
#define ABORT_TIMEOUT           5           // ms
#define TWELVE_BIT_TIMES        1           // in ms rounded up (1 is smallest we can go)
#define HALFDUPLEX_SWITCH_DELAY 4           // defined in bit-periods
#define STOP_BIT                (1<<9)      // Stop bit position in output shift "register".
#define RCV_BUF_SIZE            16          // read packet is 8 bytes

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} tmc_uart_t;

typedef struct {
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

static uint32_t period_div_2;
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

    DIGITAL_OUT(active_uart->port, active_uart->pin, 1);
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
    if(tx_buf.data) {
        DIGITAL_OUT(active_uart->port, active_uart->pin, tx_buf.data & 1);  // drive bit out
        tx_buf.data >>= 1;                                                  // shift to next bit
    } else
        tx_buf.busy = false;                                                // we are done with STOP bit
}

static void write_n (uint8_t data[], uint32_t length)
{
    uint_fast8_t i = 0;

    setTX();
    while(tx_buf.busy);                             // should not be anything pending but...

    tx_buf.data = (data[i++] << 1) | STOP_BIT;      // form first word with START and STOP bits
    tx_buf.busy = true;
    length--;

    TMC_UART_TIMER->CNT = 0;                        // initialize counter and
    TMC_UART_TIMER->CR1 &= ~TIM_CR1_UDIS;           // enable interrupt

    while(tx_buf.busy);                             // wait for 1st byte to finish

    do {
        tx_buf.data = (data[i++] << 1) | STOP_BIT;  // form next word
        tx_buf.busy = true;
        while(tx_buf.busy);                         // wait for byte to finish
    } while(--length);

    TMC_UART_TIMER->CR1 |= TIM_CR1_UDIS;            // disable interrupt
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

//    hal.port.digital_out(0, 1);

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

//    hal.port.digital_out(0, 0);
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
    TMC_UART_TIMER->CR1 |= TIM_CR1_UDIS; // Disable output event
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

    while (DIGITAL_IN(active_uart->port, active_uart->pin)) {
        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            hal.delay_ms(TWELVE_BIT_TIMES + 1, NULL);
            setTX();                // turn on our driver
            return &bad;            // return {0}
        }
    }

    // Now that we found a START bit, set timer to 1/2 bit width and start interrupts
    // This allows us to sample reliably in the center of each receive bit
    rx_buf.busy = true;
    rx_buf.bit_count = -1; // look for START bit

    TMC_UART_TIMER->CNT = period_div_2 + START_DELAY;
    TMC_UART_TIMER->CR1 &= ~TIM_CR1_UDIS;

    // Wait for read response
    int16_t res;
    ms = hal.get_elapsed_ticks();

    for (uint32_t i = 0; i < 8;) {

        if (hal.get_elapsed_ticks() - ms > ABORT_TIMEOUT) {
            rx_buf.busy = false;
            TMC_UART_TIMER->CR1 |= TIM_CR1_UDIS;        // turn off interrupts
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

static void add_uart_pin (xbar_t *gpio, void *data)
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

        RCC_ClkInitTypeDef clock;
        uint32_t latency, timer_clock_freq;

        // Determine the period based on the timer clock
        HAL_RCC_GetClockConfig(&clock, &latency);

        timer_clock_freq = HAL_RCC_GetPCLK1Freq() * (clock.APB1CLKDivider == 0 ? 1 : 2);

        TMC_UART_CLKENA();
        TMC_UART_TIMER->CR1 &= ~TIM_CR1_CEN;
        TMC_UART_TIMER->SR &= ~TIM_SR_UIF;
        TMC_UART_TIMER->CNT = 0;
        TMC_UART_TIMER->ARR = (timer_clock_freq / SWS_BAUDRATE) & 0xFFFFFFFE;
        TMC_UART_TIMER->CR1 |= TIM_CR1_UDIS|TIM_CR1_ARPE|TIM_CR1_CEN;   // Disable update events and enable timer
        TMC_UART_TIMER->DIER |= TIM_DIER_UIE;

        period_div_2 = TMC_UART_TIMER->ARR >> 1;                        // save this for use by receive

        HAL_NVIC_SetPriority(TMC_UART_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TMC_UART_IRQn);

        hal.enumerate_pins(true, add_uart_pin, NULL);
    }
}

void driver_preinit (motor_map_t motor, trinamic_driver_config_t *config)
{
    config->address = 0;
}

void tmc_uart_init (void)
{
    static trinamic_driver_if_t driver_if = {
        .on_drivers_init   = if_init,
        .on_driver_preinit = driver_preinit
    };

    trinamic_if_init(&driver_if);
}

void TMC_UART_IRQHandler (void)
{
    TMC_UART_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag

    if (tx_buf.busy)
        send();
    else if (rx_buf.busy) {
        rcv();
        rx_buf.irq_count++;
    }
}

#endif // TRINAMIC_UART_ENABLE == 2
