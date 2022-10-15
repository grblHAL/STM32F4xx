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
#if defined(BOARD_BTT_SKR_20) || defined(BOARD_BTT_SKR_20_DAC)

#if defined(BOARD_BTT_SKR_20_DAC)
// For the DAC to work, need to add HAL_DAC_MODULE_ENABLED to the build configuration
#include "stm32f4xx_hal_dac.h"

static DAC_HandleTypeDef hdac;
static void update_dac(uint32_t dac_value);
static void my_settings_changed (settings_t *settings);
static void my_set_state (spindle_state_t state, float rpm);
static settings_changed_ptr settings_changed;
static spindle_set_state_ptr set_state;
#endif

#if TRINAMIC_SPI_ENABLE

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

#endif

#if defined(BOARD_BTT_SKR_20_DAC)
/**
* @brief DAC MSP Initialization
* This function configures the hardware resources used in this example
* @param hdac: DAC handle pointer
* @retval None
*/
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hdac->Instance==DAC)
  {
  /* USER CODE BEGIN DAC_MspInit 0 */

  /* USER CODE END DAC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DAC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**DAC GPIO Configuration
    PA5     ------> DAC_OUT2
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN DAC_MspInit 1 */

  /* USER CODE END DAC_MspInit 1 */
  }

}

static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

static inline void update_dac(uint32_t dac_value)
{
  hdac.Instance->DHR12R2 = dac_value;
}

static void my_set_state (spindle_state_t state, float rpm)
{
  static float my_rpm;
  uint32_t dac_value;

  if(!state.on || rpm == 0.0f) {
    update_dac(0);
  } else {
    if(rpm < settings.spindle.rpm_min)
      my_rpm = settings.spindle.rpm_min;
    else
      if(rpm > settings.spindle.rpm_max)
        my_rpm = settings.spindle.rpm_max;
      else
        my_rpm = rpm;

    // want max output to be 2.5V but range of dac is full 3.3V
    // result is value from 0 to ~3103
    dac_value = (my_rpm / settings.spindle.rpm_max) * (2.5 / 3.3) * 4096.0f;

    update_dac(dac_value);
  }

  if(set_state)
    set_state(state, rpm);
}

static void my_settings_changed (settings_t *settings)
{
  settings_changed(settings);

  // claim the set_state pointer in case it gets overwritten
  if(hal.spindle.set_state != my_set_state) {
    set_state = hal.spindle.set_state;
    hal.spindle.set_state = my_set_state;
  }
}
#endif

void board_init (void)
{
#if TRINAMIC_SPI_ENABLE
  static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};
  trinamic_if_init(&driver_if);
#elif TRINAMIC_UART_ENABLE
    extern void tmc_uart_init (void);
    tmc_uart_init();
#endif

#if defined(BOARD_BTT_SKR_20_DAC)
  // Initialize analog output pin PA5 used for Mach 3 type spindle
  MX_DAC_Init();
  DAC->DHR12R2 = 0;                       // set dac output to 0
  __HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_2); // enable the dac

  // Intercept the settings_changed pointer
  settings_changed = hal.settings_changed;
  hal.settings_changed = my_settings_changed;
#endif
}

#endif  // BOARD_BTT_SKR_20
