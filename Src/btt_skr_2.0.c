/*
  btt_skr_2.0.c - driver code for STM32F407xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 fitch22

  Software serial code is ported from Arduino.  Credit belongs to the many
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

#if defined(BOARD_BTT_SKR_20)

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#include "trinamic/common.h"

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

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


#endif

#if TRINAMIC_ENABLE == 2209

#include "trinamic/common.h"

#define SWS_BAUDRATE            19200
#define OVERSAMPLE              3           // samples per bit
#define REPLY_DELAY             2           // ms
#define ABORT_TIMEOUT           5           // ms
#define HALFDUPLEX_SWITCH_DELAY 5           // defined in bit-periods
#define RCV_BUF_SIZE            16          // read packet is 8 bytes

TIM_HandleTypeDef htim7;

static GPIO_TypeDef *port;
static uint16_t pin;
static bool timer_on = false;

static int32_t tx_tick_cnt;
static volatile int32_t rx_tick_cnt;
static uint32_t tx_buffer;
static int32_t tx_bit_cnt;
static uint32_t rx_buffer;
static int32_t rx_bit_cnt;
static volatile bool active_listener = false;
static volatile bool active_in = false;
static volatile bool active_out = false;
static volatile bool output_pending = false;

static uint8_t receive_buffer[RCV_BUF_SIZE];
static volatile uint8_t wr_ptr = 0;;
static volatile uint8_t rd_ptr = 0;;
static bool buffer_overflow = false;

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr);
static void write_n(uint8_t data[], uint32_t length);
static size_t write_byte(uint8_t c);
static void send(void);
TMC_uart_write_datagram_t* tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr);
static int32_t rx_buffer_count(void);
static int32_t read_byte(void);
static void rcv(void);
static inline void pinMode(GPIO_TypeDef *port, uint16_t pin, bool mode);
static void stop_listening(void);
static inline void setTX();
static inline void setRX();
static inline void setRXTX(bool input);
static void enable_timer(void);
static void disable_timer(void);
static void MX_TIM7_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
    // Remember the port and pin we are using
    port = cs[driver.id].port;
    pin = cs[driver.id].pin;

    write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));
    disable_timer();
    HAL_Delay(REPLY_DELAY);
}

static void write_n(uint8_t data[], uint32_t length)
{
  for (uint32_t i = 0; i < length; i++)
  {
    write_byte(data[i]);
  }
  while (active_out) // wait for last write to finish
    ;
}

/**
  * @brief Software Serial Write Byte
  * @param Byte to be written
  * @retval Number of bytes written = 1
  * 
  * 1)  Announce we (will) have a word to send by setting output_pending true
  * 2)  Wait for any output tranmission to finish indicated by active_out
  * 3)  Form new output word
  * 4)  Enable the timer interrupt
  * 5)  Set pin to OUTPUT if not already
  * 6)  Negate output_pending
  * 7)  Set active_out true: this enables the interrupt service routine to call send()
  */
static size_t write_byte(uint8_t c)
{
    // wait for previous transmit to complete
    output_pending = true;

    while (active_out)                          // wait for any transmission to finish
        ;

    tx_buffer = c << 1 | 0x200;                 // OR in START and STOP bits
    tx_bit_cnt = 0;
    tx_tick_cnt = OVERSAMPLE;
    enable_timer();                             // turn timer on if not already
    setRXTX(false);                             // switch to TX if we are in RX mode
    output_pending = false;
    active_out = true;                          // interrupt handler may now call send()
    return 1;
}

/**
  * @brief Software Serial send byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  */
static void send(void)
{
    // //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);		// delete me when done debugging
  if (--tx_tick_cnt <= 0) {     // if tx_tick_cnt > 0 interrupt is discarded. Only when tx_tick_cnt reach 0 we set TX pin.
    if (tx_bit_cnt++ < 10) {    // tx_bit_cnt < 10 transmission is not fiisehed (10 = 1 start +8 bits + 1 stop)
      // send data (including start and stop bits)
      DIGITAL_OUT(port, pin, tx_buffer & 1);
      tx_buffer >>= 1;
      tx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks to send next bit
    } else {                    // tx_bit_count >= 10, Transmission finished
      tx_tick_cnt = 1;          // Now, check every tick
      if (output_pending) {     // is another byte ready to go?
        active_out = false;     // yes, clear the block

        // When in half-duplex mode, wait for HALFDUPLEX_SWITCH_DELAY bit-periods after the byte has
        // been transmitted before allowing the switch to RX mode
      } else if (tx_bit_cnt > 10 + OVERSAMPLE * HALFDUPLEX_SWITCH_DELAY) {
        if (active_listener) {
          setRXTX(true);            // switch to receive mode
        }
        active_out = false;
      }
    }
  }
}

TMC_uart_write_datagram_t* tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr)
{
	static TMC_uart_write_datagram_t wdgr = {0};
	static TMC_uart_write_datagram_t bad = {0};

	// Remember the port and pin we are using
	port = cs[driver.id].port;
	pin = cs[driver.id].pin;

	// claim the semaphore or wait until free
	//

    // purge anything in buffer
    while(rx_buffer_count())
        read_byte();

    active_listener = true;                                     // let send() switch us from TX to RX
    write_n(rdgr->data, sizeof(TMC_uart_read_datagram_t));      // send read request
    HAL_Delay(REPLY_DELAY);

    // scan for the rx frame and read it
	uint32_t sync_target = ((rdgr->data[0])<<16) | 0xFF00 | rdgr->data[2];
	uint32_t sync = 0;
    uint8_t timeout = ABORT_TIMEOUT;
	uint32_t ms = hal.get_elapsed_ticks();

    // match first 3 bytes with read datagram but with slave address = 0xFF
	do {
		uint32_t ms2 = hal.get_elapsed_ticks();
		if (ms2 != ms) {
			ms = ms2;
			timeout--;
		}
		if (timeout == 0) {
            stop_listening();
            return &bad;
        } else {
        	int16_t res = read_byte();
            if(res != -1) {
                sync <<= 8;
                sync |= res & 0xFF;
                sync &= 0xFFFFFF;
            }
        }
	} while (sync != sync_target);

    // wait for the rest of the recieve packet to come in
    wdgr.data[0] = rdgr->data[0];
    wdgr.data[1] = 0xff;
    wdgr.data[2] = rdgr->data[2];
    timeout = ABORT_TIMEOUT;
    ms = hal.get_elapsed_ticks();
    for(uint8_t i = 3; i < 8;) {
        uint32_t ms2 = hal.get_elapsed_ticks();
        if(ms2 != ms) {
            ms = ms2;
            timeout--;
        }
        if(!timeout) {
          stop_listening();
            return &bad;
        }

        int16_t res = read_byte();
        if(res != -1) {
            wdgr.data[i] = res;
            i++;
        }
    }

    // purge anything left in buffer
    while(rx_buffer_count())
        read_byte();

  stop_listening();
	return &wdgr;
}

/**
  * @brief Get Read Buffer Count
  * @param None
  * @returnval Count
  */
 static int32_t rx_buffer_count(void)
 {
     return (rd_ptr + RCV_BUF_SIZE - wr_ptr) % RCV_BUF_SIZE;
 }

/**
  * @brief Software Serial Read Byte
  * @param None
  * @retval Byte read
  * 
  * Returns the next byte from the receive buffer or -1 on underflow.
  */
static int32_t read_byte(void)
{
    if(rd_ptr == wr_ptr) {
        return -1;
    }

    uint8_t byte = receive_buffer[rd_ptr];    // get next byte
    rd_ptr = (rd_ptr + 1) % RCV_BUF_SIZE;
    return byte;
}

/**
  * @brief Software Serial receive byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  * Tihs is only called if active_in == true;
  */
static void rcv(void)
{
  if (--rx_tick_cnt <= 0) { // if rx_tick_cnt > 0 interrupt is discarded. Only when rx_tick_cnt reach 0 RX pin is considered
    bool inbit = DIGITAL_IN(port, pin);
    if (rx_bit_cnt == -1) {  // rx_bit_cnt = -1 :  waiting for start bit
      if (!inbit) {             // START?
        rx_bit_cnt = 0;         // rx_bit_cnt == 0 : start bit received
        rx_tick_cnt = OVERSAMPLE + 1;   // Wait 1 bit (OVERSAMPLE ticks) + 1 tick in order to sample RX pin in the middle of the edge (and not too close to the edge)
        rx_buffer = 0;
      } else {
        rx_tick_cnt = 1; // Still aiting for START.  Set count to 1 so we check next interrupt.
      }
    } else if (rx_bit_cnt >= 8) {           // rx_bit_cnt >= 8 : now waiting for STOP
      if (inbit) {                          // Is it STOP?
        uint8_t next = (wr_ptr + 1) % RCV_BUF_SIZE;
        if (next != rd_ptr) {                 // room in buffer?
          receive_buffer[wr_ptr] = rx_buffer; // save new byte
          wr_ptr = next;
        } else {                // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
          buffer_overflow = true;
        }
      }
      // Full trame received. Look for START at next interrupt
      rx_tick_cnt = 1;
      rx_bit_cnt = -1;
    } else {
      rx_buffer >>= 1;          // shift previous
      if (inbit) {
        rx_buffer |= 0x80;      // OR in new
      }
      rx_bit_cnt++;             // Preprare for next bit
      rx_tick_cnt = OVERSAMPLE; // Wait OVERSAMPLE ticks before sampling next bit
    }
  }
}

/**
  * @brief Set UART Pin Mode
  * @param port, pin, and mode
  * @retval None
  * If mode == true, set the UART pin to OUTPUT_PP,
  * otherwise set the UART pin to INPUT with PULLUP
  */
static inline void pinMode(GPIO_TypeDef *port, uint16_t pin, bool mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = 1 << pin;
    if(mode) {              // output if true
    	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    	GPIO_InitStruct.Pull = GPIO_NOPULL;
    } else {                // input with pull-up
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
	HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void stop_listening(void)
{
    while (active_out)    // wait for any output to complete
      ;
    setRXTX(false);       // switch back to TX
    active_listener = false;
    active_in = false;
    disable_timer();      // turn off ints
}

static inline void setTX()
{
    DIGITAL_OUT(port, pin, 1);
//   if (_inverse_logic) {
//     LL_GPIO_ResetOutputPin(_transmitPinPort, _transmitPinNumber);
//   } else {
//     LL_GPIO_SetOutputPin(_transmitPinPort, _transmitPinNumber);
//   }
    pinMode(port, pin, true);
}

static inline void setRX()
{
  pinMode(port, pin, false);    // INPUT with PULLUP
}

static inline void setRXTX(bool input)
{
    if (input) {
      if (!active_in) {
        setRX();
        rx_bit_cnt = -1; // rx_bit_cnt = -1 :  waiting for start bit
        rx_tick_cnt = 2; // 2 : next interrupt will be discarded. 2 interrupts required to consider RX pin level
        active_in = true;
      }
    } else {
        if(active_in) {
            setTX();
            active_in = false;
        }
    }

}

static void enable_timer(void)
{
    if(!timer_on) {
        __HAL_TIM_SET_COUNTER(&htim7, 0);	        // Clear timer counter
        htim7.Instance->CR1 &= ~(TIM_CR1_UDIS);		// Enable output event, this starts the interrupt
        timer_on = true;
    }
}

static void disable_timer(void)
{
    if(timer_on) {
        htim7.Instance->CR1 |= (TIM_CR1_UDIS);		// Disable output event
        timer_on = false;
    }
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  RCC_ClkInitTypeDef clock;
  uint32_t latency;
  uint32_t timer_clock_freq;
  uint16_t period;

  // Determine the period based on the timer clock
  HAL_RCC_GetClockConfig(&clock, &latency);
  timer_clock_freq = HAL_RCC_GetPCLK1Freq() * (clock.APB1CLKDivider == 0 ? 1 : 2);
  period = (uint16_t)((double)timer_clock_freq / (SWS_BAUDRATE * OVERSAMPLE) + 0.5);

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = period;
  htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
    /* TIM7 interrupt Init */
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    //HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);		// delete me when done debugging

    if(active_out)
    	send();
    if(active_in)
    	rcv();
}

#endif  // TRINAMIC_ENABLE == 2209

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160 || TRINAMIC_ENABLE == 2209

static void add_cs_pin (xbar_t *gpio)
{
    if(gpio->group == PinGroup_MotorChipSelect) {
        switch(gpio->function) {

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

    if(!init_ok) {
#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160
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
#endif
#if TRINAMIC_ENABLE == 2209
		MX_TIM7_Init();
		htim7.Instance->CR1 |= TIM_CR1_UDIS;			// Disable update events
		HAL_TIM_Base_Start_IT(&htim7);
#endif
        hal.enumerate_pins(true, add_cs_pin);
    }
}

#endif

void board_init (void)
{

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160 || TRINAMIC_ENABLE == 2209

    static trinamic_driver_if_t driver_if = {
        .on_drivers_init = if_init
    };

    trinamic_if_init(&driver_if);


#endif  // TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160 || TRINAMIC_ENABLE == 2209

}

#endif  // BOARD_BTT_SKR_20

