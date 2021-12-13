/*
  btt_skr_2.0.c - driver code for STM32F407xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 fitch22

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

#endif  // TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#if TRINAMIC_ENABLE == 2209

#include "trinamic/common.h"

#define SWS_BAUDRATE            100000      // 10us bit period, near upper limit @ 168MHz
#define START_DELAY             46          // delay in us * timer clock freq
#define ABORT_TIMEOUT           5           // ms
#define TWELVE_BIT_TIMES        1           // in ms rounded up (1 is smallest we can go)
#define HALFDUPLEX_SWITCH_DELAY 4           // defined in bit-periods
#define RCV_BUF_SIZE            16          // read packet is 8 bytes

TIM_HandleTypeDef htim7;

static GPIO_TypeDef *port;
static uint16_t pin;
static uint16_t period_div_2;
static volatile uint32_t tx_buffer;
static volatile int32_t tx_bit_count;
static volatile uint32_t rx_buffer;
static volatile int32_t rx_bit_count;
static volatile bool rx_busy = false;
static volatile bool tx_busy = false;

static volatile uint8_t receive_buffer[RCV_BUF_SIZE];
static volatile uint8_t wr_ptr = 0;
static volatile uint8_t rd_ptr = 0;
static bool buffer_overflow = false;
static volatile uint8_t rx_irq_count = 0;

static struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} cs[TMC_N_MOTORS_MAX];

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr);
static void write_n(uint8_t data[], uint32_t length);
static void send(void);
TMC_uart_write_datagram_t* tmc_uart_read(trinamic_motor_t driver, TMC_uart_read_datagram_t *rdgr);
static int32_t rx_buffer_count(void);
static int32_t read_byte(void);
static void rcv(void);
static inline void pinMode(GPIO_TypeDef *port, uint16_t pin, bool mode);
static void stop_listening(void);
static inline void setTX();
static inline void setRX();
static void MX_TIM7_Init(void);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);

void tmc_uart_write(trinamic_motor_t driver, TMC_uart_write_datagram_t *dgr)
{
  // Remember the port and pin we are using
  port = cs[driver.id].port;
  pin = cs[driver.id].pin;

  write_n(dgr->data, sizeof(TMC_uart_write_datagram_t));
}

static void write_n(uint8_t data[], uint32_t length)
{
  uint8_t i;

  while(tx_busy)                                // should not be anything pending but...
    ;
  
  tx_buffer = data[0] << 1;                     // form first word with START bit (0)
  tx_bit_count = 0;                             // reset bit counter
  __HAL_TIM_SET_COUNTER(&htim7, START_DELAY);   // initialize counter with START delay
  htim7.Instance->CR1 &= ~(TIM_CR1_UDIS);       // enable interrupt
  tx_busy = true;
  send();                                       // force first bit without interrupt

  while(tx_busy)                                // wait for 1st byte to finish
    ;

  for(i = 1; i < length; i++) {
      tx_buffer = data[i] << 1;                 // form next word
      tx_bit_count = 0;
      tx_busy = true;
      while(tx_busy)                            // wait for byte to finish
        ;
  }
  tx_busy = true;                               // wait one more bit period
  while(tx_busy)                                // for the last STOP bit
    ;

  htim7.Instance->CR1 |= TIM_CR1_UDIS;          // disable interrupt
}

/**
  * @brief Software Serial send byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  */
static void send(void)
{
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);		// delete me when done debugging

  if(tx_bit_count++ < 9) {
    DIGITAL_OUT(port, pin, tx_buffer & 1);  // drive bit out
    tx_buffer >>= 1;                        // shift to next bit
  } else {
    DIGITAL_OUT(port, pin, 1);              // STOP bit (1)
    tx_busy = false;                        // we are done with STOP bit
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
  while (rx_buffer_count())
    read_byte();

  write_n(rdgr->data, sizeof(TMC_uart_read_datagram_t)); // send read request

  // we just sent last STOP bit, now switch to input and look for START
  setRX();

  // Look for START (0)
  // If read request had CRC error or some other issue, we won't get a reply
  // so we need to timeout
  uint8_t timeout = ABORT_TIMEOUT;
  uint32_t ms = hal.get_elapsed_ticks();
  bool inbit;
  do {
    uint32_t ms2 = hal.get_elapsed_ticks();
    if (ms2 != ms)
    {
      ms = ms2;
      if (--timeout == 0)
      {
        HAL_Delay(TWELVE_BIT_TIMES);
        setTX();                  // turn on our driver
        return &bad;              // return {0}
      }
    }
    inbit = DIGITAL_IN(port, pin);
  } while (inbit);

  // Now that we found a START bit, set timer to 1/2 bit width and start interrupts
  // This allows us to sample reliably in the center of each receive bit
  rx_busy = true;
  rx_bit_count = -1; // look for START bit
  __HAL_TIM_SET_COUNTER(&htim7, period_div_2);
  htim7.Instance->CR1 &= ~(TIM_CR1_UDIS);

  // Wait for read response
  timeout = ABORT_TIMEOUT;
  ms = hal.get_elapsed_ticks();
  for (uint8_t i = 0; i < 8;) {
    uint32_t ms2 = hal.get_elapsed_ticks();
    if (ms2 != ms) {
      ms = ms2;
      if (--timeout == 0) {
        rx_busy = false;
        htim7.Instance->CR1 |= (TIM_CR1_UDIS);  // turn off interrupts
        HAL_Delay(TWELVE_BIT_TIMES);
        setTX();                                // turn on our driver
        return &bad;
      }
    }
    int16_t res = read_byte();
    if (res != -1) {
      wdgr.data[i] = res;
      i++;
    }
  }

  // purge anything left in buffer
  while (rx_buffer_count())
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
  if (rd_ptr == wr_ptr) {
    return -1;
  }

  uint8_t byte = receive_buffer[rd_ptr]; // get next byte
  rd_ptr = (rd_ptr + 1) % RCV_BUF_SIZE;
  return byte;
}

/**
  * @brief Software Serial receive byte
  * @param None
  * @retval None
  * This is called by the interrupt handler.
  * Tihs is only called if tx_busy == true;
  */
static void rcv(void)
{
  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_4);		// delete me when done debugging

  bool inbit = DIGITAL_IN(port, pin);

  if(rx_bit_count == -1) {        // -1 means waiting for START (0)
    if(!inbit) {
      rx_bit_count = 0;           // 0: START bit received
      rx_buffer = 0;
    }
  } else if(rx_bit_count >= 8) {  // >= 8 means waiting for STOP
      if(inbit) {
        uint8_t next = (wr_ptr + 1) % RCV_BUF_SIZE;
        if (next != rd_ptr) {     // room in buffer?
          receive_buffer[wr_ptr] = rx_buffer; // save new byte
          wr_ptr = next;
        } else {                  // rx_bit_cnt = x  with x = [0..7] correspond to new bit x received
          buffer_overflow = true;
        }
      }
      rx_bit_count = -1;          // wait for next START
  } else {
      rx_buffer >>= 1;            // shift previous
      if (inbit) {
        rx_buffer |= 0x80;        // OR in new
      }
      rx_bit_count++;             // Preprare for next bit
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
  if (mode) {   // output if true
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  } else {      // input with pull-up
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  }
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void stop_listening(void)
{
  uint8_t count = rx_irq_count;

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
  while ((rx_irq_count + 256 - count) % 256 < HALFDUPLEX_SWITCH_DELAY)
    ;
  setTX();		// turn driver on just before the TMC driver goes off

  // Wait one more bit period without which the TMC won't respond
  while ((rx_irq_count + 256 - count) % 256 < (HALFDUPLEX_SWITCH_DELAY + 1))
    ;

  // Now we can wrap things up
  rx_busy = false;
  htim7.Instance->CR1 |= (TIM_CR1_UDIS); // Disable output event
}

static inline void setTX()
{
	DIGITAL_OUT(port, pin, 1);
	pinMode(port, pin, true);
}

static inline void setRX()
{
  pinMode(port, pin, false);    // INPUT with PULLUP
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
  period = ((uint16_t)((double)timer_clock_freq / SWS_BAUDRATE) + 0.5);

  period_div_2 = (period / 2.0f) + 0.5;         // save this for use by receive

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
  if(htim_base->Instance==TIM7) {
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

	if (tx_busy)
		send();
	if (rx_busy) {
		rcv();
		rx_irq_count++;
	}
}

#endif  // TRINAMIC_ENABLE == 2209

#if TRINAMIC_ENABLE

static void add_cs_pin (xbar_t *gpio)
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

/*     // Delete when done debugging
    GPIO_InitTypeDef GPIO_InitStruct = {0};
	  GPIO_InitStruct.Pin = 1 << 4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
 */
#endif
        hal.enumerate_pins(true, add_cs_pin);
  }
}

#endif

void board_init (void)
{

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160 || TRINAMIC_ENABLE == 2209

  static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

  trinamic_if_init(&driver_if);

#endif // TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160 || TRINAMIC_ENABLE == 2209
}

#if TRINAMIC_ENABLE == 2209

void TIM7_IRQHandler (void)
{
    HAL_TIM_IRQHandler(&htim7);
}

#endif

#endif  // BOARD_BTT_SKR_20

