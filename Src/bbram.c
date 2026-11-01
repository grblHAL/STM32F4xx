/*

  bbram.c - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#if BBRAM_ENABLE

#include "grbl/hal.h"
#include "grbl/crc.h"

static uint8_t getByte (uint32_t addr)
{
    return *((uint8_t *)BKPSRAM_BASE + addr);
}

static void putByte (uint32_t addr, uint8_t value)
{
    *((uint8_t *)BKPSRAM_BASE + addr) = value;
}

static nvs_transfer_result_t writeBlock (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    memcpy((void *)(BKPSRAM_BASE + destination), source, size);

    if(size > 0 && with_checksum) {
        destination += size;
        uint16_t checksum = calc_checksum(source, size);
        putByte(destination, checksum & 0xFF);
#if NVS_CRC_BYTES > 1
        putByte(++destination, checksum >> 8);
#endif
    }

    return true;
}

static nvs_transfer_result_t readBlock (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    memcpy(destination, (void *)(BKPSRAM_BASE + source ), size);

    source += size;

#if NVS_CRC_BYTES == 1
    return !with_checksum || calc_checksum(destination, size) == getByte(source);
#else
    return !with_checksum || calc_checksum(destination, size) == (getByte(source) | (getByte(source + 1) << 8));
#endif
}

bool bbram_init (void)
{
    ADC_HandleTypeDef adc1 = {
        .Instance = ADC1,
        .Init = {
            .Resolution = ADC_RESOLUTION_12B,
            .ScanConvMode = DISABLE,
            .ContinuousConvMode = DISABLE,
            .DiscontinuousConvMode = DISABLE,
            .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
            .ExternalTrigConv = ADC_SOFTWARE_START,
            .DataAlign = ADC_DATAALIGN_RIGHT,
            .NbrOfConversion = 1,
            .DMAContinuousRequests = DISABLE,
            .EOCSelection = ADC_EOC_SINGLE_CONV
        }
    };
    ADC_ChannelConfTypeDef config = {
        .Channel = ADC_CHANNEL_VBAT,
        .Rank = 1,
        .SamplingTime = ADC_SAMPLETIME_480CYCLES
    };

    __HAL_RCC_ADC1_CLK_ENABLE();
    ADC->CCR |= ADC_CCR_VBATE;

    if(HAL_ADC_Init(&adc1) == HAL_OK && HAL_ADC_ConfigChannel(&adc1, &config) == HAL_OK) {

        HAL_ADC_Start(&adc1);
        HAL_ADC_PollForConversion(&adc1, 100);

        if((float)HAL_ADC_GetValue(&adc1) * (3.3f / 4096.0f) * 2.0f >= 1.5f) { // F407: 3V3 Vref and 1/2 divider.

            __HAL_RCC_PWR_CLK_ENABLE();
            HAL_PWR_EnableBkUpAccess();
            HAL_PWR_EnableBkUpReg();
            __HAL_RCC_BKPSRAM_CLK_ENABLE();

            hal.nvs.type = NVS_BBRAM;
            hal.nvs.size_max = 1024 * 4;
            hal.nvs.get_byte = getByte;
            hal.nvs.put_byte = putByte;
            hal.nvs.memcpy_to_nvs = writeBlock;
            hal.nvs.memcpy_from_nvs = readBlock;
        }

        HAL_ADC_DeInit(&adc1);
    }

    return hal.nvs.type == NVS_BBRAM;
}

#endif // BBRAM_ENABLE
