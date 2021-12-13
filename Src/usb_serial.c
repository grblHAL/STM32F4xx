/*

  usb_serial.c - USB serial port implementation for STM32F103C8 ARM processors

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

#include "driver.h"

#if USB_SERIAL_CDC

#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer2_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

//
// Returns number of free characters in the input buffer
//
static uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
static void usbRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool usb_write (void)
{
    static uint8_t dummy = 0;

    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;

    while(CDC_Transmit_FS((uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    if(txbuf.length % 64 == 0) {
        while(CDC_Transmit_FS(&dummy, 0) == USBD_BUSY) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }

    txbuf.use_tx2data = !txbuf.use_tx2data;
    txbuf.s = txbuf.use_tx2data ? txbuf.data2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
static bool usbPutC (const char c)
{
    static uint8_t buf[1];

    *buf = c;

    while(CDC_Transmit_FS(buf, 1) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == ASCII_LF)
            usb_write();
    }
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usbWrite (const char *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        usb_write();
    }
}

//
// usbGetC - returns -1 if no data available
//
static int16_t usbGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

// NOTE: USB interrupt priority should be set lower than stepper/step timer to avoid jitter
// It is set in HAL_PCD_MspInit() in usbd_conf.c
const io_stream_t *usbInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .read = usbGetC,
        .write = usbWriteS,
        .write_char = usbPutC,
        .write_n = usbWrite,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usbRxFree,
        .reset_read_buffer = usbRxFlush,
        .cancel_read_buffer = usbRxCancel,
        .suspend_read = usbSuspendInput,
        .set_enqueue_rt_handler = usbSetRtHandler
    };

    MX_USB_DEVICE_Init();

    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;

    return &stream;
}

// NOTE: add a call to this function as the first line CDC_Receive_FS() in usbd_cdc_if.c
void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {
        if(!enqueue_realtime_command(*data)) {                  // Check and strip realtime commands,
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = *data;                 // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
        data++;                                                 // next...
    }
}

#endif
