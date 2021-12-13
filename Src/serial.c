/*

  serial.c - serial port implementation for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include <string.h>

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

#ifdef SERIAL2_MOD
static stream_rx_buffer_t rxbuf2 = {0};
static stream_tx_buffer_t txbuf2 = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command2 = protocol_enqueue_realtime_command;
#endif

#if IS_NUCLEO_DEVKIT

  #define USART USART2
  #define USART_IRQHandler USART2_IRQHandler

#else

  #define USART USART1
  #define USART_IRQHandler USART1_IRQHandler

#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .claim = serialInit
    },
#ifdef SERIAL2_MOD
    {
      .type = StreamType_Serial,
      .instance = 1,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .flags.modbus_ready = On,
      .claim = serial2Init
    }
#endif
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c)
{
    uint16_t next_head = BUFNEXT(txbuf.head, txbuf);    // Get pointer to next free slot in buffer

    while(txbuf.tail == next_head) {                    // While TX buffer full
        if(!hal.stream_blocking_callback())             // check if blocking for space,
            return false;                               // exit if not (leaves TX buffer in an inconsistent state)
    }
    txbuf.data[txbuf.head] = c;                         // Add data to buffer,
    txbuf.head = next_head;                             // update head pointer and
    USART->CR1 |= USART_CR1_TXEIE;                      // enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
#if IS_NUCLEO_DEVKIT
    USART->CR1 = USART_CR1_RE|USART_CR1_TE;
    USART->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud_rate);
    USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);
#else
    USART->CR1 = USART_CR1_RE|USART_CR1_TE;
    USART->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baud_rate);
    USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);
#endif

    return true;
}

static bool serialDisable (bool disable)
{
    if(disable)
        USART->CR1 &= ~USART_CR1_RXNEIE;
    else
        USART->CR1 |= USART_CR1_RXNEIE;

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.connected = On,
        .read = serialGetC,
        .write = serialWriteS,
        .write_n =  serialWrite,
        .write_char = serialPutC,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
//        .get_rx_buffer_count = serialRxCount,
//        .get_tx_buffer_count = serialTxCount,
//        .reset_write_buffer = serialTxFlush,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .disable_rx = serialDisable,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    __HAL_RCC_GPIOA_CLK_ENABLE();

#if IS_NUCLEO_DEVKIT

    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    serialSetBaudRate(baud_rate);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port  = GPIOA,
        .pin   = 2,
        .mode  = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };
    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = GPIOA,
        .pin = 3,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };


#else

    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    serialSetBaudRate(baud_rate);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port = GPIOA,
        .pin = 9,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Primary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = GPIOA,
        .pin = 10,
        .mode = { .mask = PINMODE_NONE },
        .description = "Primary UART"
    };

#endif

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

void USART_IRQHandler (void)
{
    if(USART->SR & USART_SR_RXNE) {
        uint32_t data = USART->DR;
        if(!enqueue_realtime_command((char)data)) {             // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = (char)data;            // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
    }

    if((USART->SR & USART_SR_TXE) && (USART->CR1 & USART_CR1_TXEIE)) {
        uint_fast16_t tail = txbuf.tail;            // Get buffer pointer
        USART->DR = txbuf.data[tail];               // Send next character
        txbuf.tail = tail = BUFNEXT(tail, txbuf);   // and increment pointer
        if(tail == txbuf.head)                      // If buffer empty then
            USART->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
   }
}

#ifdef SERIAL2_MOD

#if IS_NUCLEO_DEVKIT

#define UART2 USART1
#define UART2_IRQHandler USART1_IRQHandler

#else

#define UART2 USART2
#define UART2_IRQHandler USART2_IRQHandler

#endif

//
// Returns number of free characters in serial input buffer
//
static uint16_t serial2RxFree (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serial2RxCount (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serial2RxFlush (void)
{
    rxbuf2.tail = rxbuf2.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serial2RxCancel (void)
{
    rxbuf2.data[rxbuf2.head] = ASCII_CAN;
    rxbuf2.tail = rxbuf2.head;
    rxbuf2.head = BUFNEXT(rxbuf2.head, rxbuf2);
}

//
// Writes a character to the serial output stream
//
static bool serial2PutC (const char c)
{
    uint32_t next_head = BUFNEXT(txbuf2.head, txbuf2);   // Set and update head pointer

    while(txbuf2.tail == next_head) {           // While TX buffer full
        if(!hal.stream_blocking_callback())     // check if blocking for space,
            return false;                       // exit if not (leaves TX buffer in an inconsistent state)
        UART2->CR1 |= USART_CR1_TXEIE;          // Enable TX interrupts???
    }

    txbuf2.data[txbuf2.head] = c;               // Add data to buffer
    txbuf2.head = next_head;                    // and update head pointer

    UART2->CR1 |= USART_CR1_TXEIE;              // Enable TX interrupts

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serial2WriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
static void serial2Write(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
static void serial2TxFlush (void)
{
    UART2->CR1 &= ~USART_CR1_TXEIE;     // Disable TX interrupts
    txbuf2.tail = txbuf2.head;
}

//
// Returns number of characters pending transmission
//
static uint16_t serial2TxCount (void)
{
    uint32_t tail = txbuf2.tail, head = txbuf2.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (UART2->SR & USART_SR_TC ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serial2GetC (void)
{
    uint_fast16_t tail = rxbuf2.tail;       // Get buffer pointer

    if(tail == rxbuf2.head)
        return -1; // no data available

    char data = rxbuf2.data[tail];          // Get next character
    rxbuf2.tail = BUFNEXT(tail, rxbuf2);    // and update pointer

    return (int16_t)data;
}

static bool serial2SuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf2, suspend);
}

static bool serial2SetBaudRate (uint32_t baud_rate)
{
#if IS_NUCLEO_DEVKIT
    UART2->CR1 = USART_CR1_RE|USART_CR1_TE;
    UART2->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baud_rate);
    UART2->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);
#else
    UART2->CR1 = USART_CR1_RE|USART_CR1_TE;
    UART2->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud_rate);
    UART2->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);
#endif

    return true;
}

static bool serial2Disable (bool disable)
{
    if(disable)
        UART2->CR1 &= ~USART_CR1_RXNEIE;
    else
        UART2->CR1 |= USART_CR1_RXNEIE;

    return true;
}

static bool serial2EnqueueRtCommand (char c)
{
    return enqueue_realtime_command2(c);
}

static enqueue_realtime_command_ptr serial2SetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command2;

    if(handler)
        enqueue_realtime_command2 = handler;

    return prev;
}

const io_stream_t *serial2Init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .instance = 1,
        .state.connected = On,
        .read = serial2GetC,
        .write = serial2WriteS,
        .write_n =  serial2Write,
        .write_char = serial2PutC,
        .enqueue_rt_command = serial2EnqueueRtCommand,
        .get_rx_buffer_free = serial2RxFree,
        .get_rx_buffer_count = serial2RxCount,
        .get_tx_buffer_count = serial2TxCount,
        .reset_write_buffer = serial2TxFlush,
        .reset_read_buffer = serial2RxFlush,
        .cancel_read_buffer = serial2RxCancel,
        .suspend_read = serial2SuspendInput,
        .disable_rx = serial2Disable,
        .set_baud_rate = serial2SetBaudRate,
        .set_enqueue_rt_handler = serial2SetRtHandler
    };

    if(serial[1].flags.claimed)
        return NULL;

    serial[1].flags.claimed = On;

#if IS_NUCLEO_DEVKIT

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Pin = GPIO_PIN_9|GPIO_PIN_10,
        .Alternate = GPIO_AF7_USART1
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    serial2SetBaudRate(baud_rate);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .port = GPIOA,
        .pin = 9,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .port = GPIOA,
        .pin = 10,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

#else

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Pin = GPIO_PIN_2|GPIO_PIN_3,
        .Alternate = GPIO_AF7_USART2
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    serial2SetBaudRate(baud_rate);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART2,
        .port = GPIOA,
        .pin = 2,
        .mode = { .mask = PINMODE_OUTPUT },
        .description = "Secondary UART"
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART2,
        .port = GPIOA,
        .pin = 3,
        .mode = { .mask = PINMODE_NONE },
        .description = "Secondary UART"
    };

#endif

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

    return &stream;
}

void UART2_IRQHandler (void)
{
    if(UART2->SR & USART_SR_RXNE) {
        uint32_t data = UART2->DR;
        if(!enqueue_realtime_command2((char)data)) {            // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf2.head, rxbuf2);  // Get and increment buffer pointer
            if(next_head == rxbuf2.tail)                        // If buffer full
                rxbuf2.overflow = 1;                            // flag overflow
            else {
                rxbuf2.data[rxbuf2.head] = (char)data;          // if not add data to buffer
                rxbuf2.head = next_head;                        // and update pointer
            }
        }
    }

    if((UART2->SR & USART_SR_TXE) && (UART2->CR1 & USART_CR1_TXEIE)) {
        uint_fast16_t tail = txbuf2.tail;           // Get buffer pointer
        UART2->DR = txbuf2.data[tail];              // Send next character
        txbuf2.tail = tail = BUFNEXT(tail, txbuf2); // and increment pointer
        if(tail == txbuf2.head)                     // If buffer empty then
            UART2->CR1 &= ~USART_CR1_TXEIE;         // disable UART TX interrupt
   }
}
#endif

