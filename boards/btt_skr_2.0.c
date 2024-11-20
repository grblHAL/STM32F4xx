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

#if defined(BOARD_BTT_SKR_20_DAC)

// For the DAC to work, need to add HAL_DAC_MODULE_ENABLED to the build configuration
#include "stm32f4xx_hal_dac.h"

static DAC_HandleTypeDef hdac;
static void update_dac(uint32_t dac_value);
static void my_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm);
static on_spindle_select_ptr on_spindle_select;
static spindle_set_state_ptr set_state;

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

static void my_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
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
    set_state(spindle, state, rpm);
}

static bool my_spindle_select (spindle_ptrs_t *spindle)
{
  on_spindle_select(spindle);

  // claim the set_state pointer in case it gets overwritten
  if(spindle->type == SpindleType_PWM && spindle->set_state != my_set_state) {
    set_state = spindle->set_state;
    spindle->set_state = my_set_state;
  }

  return true;
}

void board_init (void)
{
  // Initialize analog output pin PA5 used for Mach 3 type spindle
  MX_DAC_Init();
  DAC->DHR12R2 = 0;                       // set dac output to 0
  __HAL_DAC_ENABLE(&hdac, DAC_CHANNEL_2); // enable the dac

  // Intercept the settings_changed pointer
  on_spindle_select = grbl.on_spindle_select;
  grbl.on_spindle_select = my_spindle_select;
}

#endif  // BOARD_BTT_SKR_20_DAC
