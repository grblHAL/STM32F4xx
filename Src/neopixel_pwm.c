/*
  neopixel_pwm.c - PWM support for Neopixels, non-blocking

  Part of grblHAL driver for STM32F7xx

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

// *** EXPERIMENTAL - incomplete ***

#include "main.h"
#include "driver.h"

#ifdef NEOPIXEL_PWM

#define timerCH(c) timerch(c)
#define timerch(c) TIM_CHANNEL_ ## c

#define DMAinstance(d) DMAinstanceI(d)
#define DMAinstanceI(d) DMA ## d
#define DMAstream(d, p) DMAstreamI(d, p)
#define DMAstreamI(d, p) DMA ## d ## _Stream ## p
#define DMAchannel(c) DMAchannelI(c)
#define DMAchannelI(c) DMA_CHANNEL_ ## c
#define DMA_CLKEN(d) dmaclken(d)
#define dmaclken(d) __HAL_RCC_DMA ## d ## _CLK_ENABLE

//---

// PC7
#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   4
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    7
#define DMA_CHANNEL_N   7

//---

#define PWM_TIMER timer(PWM_TIMER_N)
#define PWM_CHANNEL timerCH(PWM_CHANNEL_N)
#define PWM_GPIO_AF timerAF(PWM_TIMER_N, 3)
#define PWM_TIMER_CLKEN timerCLKEN(PWM_TIMER_N)

#define DMA_INSTANCE DMAinstance(DMA_INSTANCE_N)
#define DMA_STREAM DMAstream(DMA_INSTANCE_N, DMA_STREAM_N)
#define DMA_CHANNEL DMAchannel(DMA_CHANNEL_N)
#define DMA_INSTANCE_CLKEN DMA_CLKEN(DMA_INSTANCE_N)
#define DMA_TX_IRQ DMAirq(DMA_INSTANCE_N, DMA_CHANNEL_N)
#define DMA_TX_IRQ_HANDLER DMAhandler(DMA_INSTANCE_N, DMA_CHANNEL_N)

static TIM_HandleTypeDef pwm_timer = {
    .Instance = PWM_TIMER,
    .Init.Prescaler = 0,
    .Init.CounterMode = TIM_COUNTERMODE_UP,
    .Init.ClockDivision = TIM_CLOCKDIVISION_DIV1,
    .Init.RepetitionCounter = 0,
    .Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE
};

static DMA_HandleTypeDef pwm_dma = {
    .Instance = DMA_STREAM,
    .Init.Channel = DMA_CHANNEL,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
    .Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_MEDIUM,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

typedef struct {
    uint16_t t_low;
    uint16_t t_high;
} rgb_timing_t;

static volatile bool busy = false;
static neopixel_cfg_t neopixel = { .intensity = 255 };
static settings_changed_ptr settings_changed;
static rgb_timing_t timing;

static inline void rgb_24bpp_pack (uint16_t *led, rgb_color_t color, rgb_color_mask_t mask, uint8_t intensity, rgb_timing_t timing)
{
    uint8_t bitmask = 0b10000000;

    color = rgb_set_intensity(color, intensity);

    if(mask.G) {
        do {
            *led++ = (color.G & bitmask) ? timing.t_high : timing.t_low;
        } while(bitmask >>= 1);
    } else
        led += 8;

    if(mask.R) {
        bitmask = 0b10000000;
        do {
            *led++ = (color.R & bitmask) ? timing.t_high : timing.t_low;
        } while(bitmask >>= 1);
    } else
        led += 8;

    if(mask.B) {
        bitmask = 0b10000000;
        do {
            *led++ = (color.B & bitmask) ? timing.t_high : timing.t_low;
        } while(bitmask >>= 1);
    }
}

static inline rgb_color_t rgb_24bpp_unpack (uint16_t *led, uint8_t intensity, rgb_timing_t timing)
{
    rgb_color_t color = {0};

    if(intensity) {

        uint8_t bitmask = 0b10000000;

        do {
            if(*led++ == timing.t_high)
                color.G |= bitmask;
        } while(bitmask >>= 1);

        bitmask = 0b10000000;

        do {
            if(*led++ == timing.t_high)
                color.R |= bitmask;
        } while(bitmask >>= 1);

        bitmask = 0b10000000;

        do {
            if(*led++ == timing.t_high)
                color.B |= bitmask;
        } while(bitmask >>= 1);

        color = rgb_reset_intensity(color, intensity);
    }

    return color;
}

static inline void _write (void)
{
//    while(busy);

    if(!busy)
        busy = HAL_TIM_PWM_Start_DMA(&pwm_timer, PWM_CHANNEL, (uint32_t *)neopixel.leds, neopixel.num_bytes >> 1) == HAL_OK;
}

void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_24bpp_pack((uint16_t *)&neopixel.leds[device * 24 * 2], color, mask, neopixel.intensity, timing);

        if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_24bpp_unpack((uint16_t *)&neopixel.leds[device * 24 * 2], prev, timing);
                neopixel_out(device, color);
            } while(device);

            if(neopixel.num_leds != 1)
                _write();
        }
    }

    return prev;
}

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0) {

        hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 24 * 2 + 40; // 40 -> 80?
            if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb0.num_devices;
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

extern uint32_t timer_clk_enable (TIM_TypeDef *timer);

void neopixel_init (void)
{
    static const periph_pin_t sdi = {
        .function = Output_LED_Adressable,
        .group = PinGroup_LED,
        .port = LED_PORT,
        .pin = LED_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Neopixels"
    };

    static bool init = false;

    if(!init) {

   //     PWM_TIMER_CLKEN();
        DMA_INSTANCE_CLKEN();

        TIM_ClockConfigTypeDef sClockSourceConfig = {
            .ClockSource = TIM_CLOCKSOURCE_INTERNAL
        };

        TIM_OC_InitTypeDef sConfigOC = {
            .OCMode = TIM_OCMODE_PWM1,
            .Pulse = 0,
            .OCPolarity = TIM_OCPOLARITY_HIGH,
            .OCNPolarity = TIM_OCNPOLARITY_HIGH,
            .OCFastMode = TIM_OCFAST_DISABLE,
            .OCIdleState = TIM_OCIDLESTATE_RESET,
            .OCNIdleState = TIM_OCNIDLESTATE_RESET
        };

        GPIO_InitTypeDef GPIO_InitStruct = {
            .Pin = 11 << LED_PIN,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = PWM_GPIO_AF
        };

        pwm_timer.Init.Period = timer_clk_enable(pwm_timer.Instance) / 800000 - 1;

        timing.t_low = pwm_timer.Init.Period / 3;
        timing.t_high = timing.t_low * 2;

        if(HAL_TIM_Base_Init(&pwm_timer) == HAL_OK &&
            HAL_TIM_ConfigClockSource(&pwm_timer, &sClockSourceConfig) == HAL_OK &&
             HAL_TIM_PWM_Init(&pwm_timer) == HAL_OK &&
              HAL_TIM_PWM_ConfigChannel(&pwm_timer, &sConfigOC, PWM_CHANNEL) == HAL_OK) {

            HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

            HAL_DMA_Init(&pwm_dma);
            __HAL_LINKDMA(&pwm_timer, hdma[TIM_DMA_ID_CC4], pwm_dma);

            HAL_NVIC_SetPriority(DMA_TX_IRQ, 3, 0);
            HAL_NVIC_EnableIRQ(DMA_TX_IRQ);

            hal.periph_port.register_pin(&sdi);

            hal.rgb0.out = neopixel_out;
            hal.rgb0.out_masked = neopixel_out_masked;
            hal.rgb0.set_intensity = neopixels_set_intensity;
            hal.rgb0.write = neopixels_write;
            hal.rgb0.flags = (rgb_properties_t){ .is_strip = On };
            hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

            settings_changed = hal.settings_changed;
            hal.settings_changed = onSettingsChanged;
        }

        init = true;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
    busy = false;
    HAL_TIM_PWM_Stop_DMA(htim, PWM_CHANNEL);
}

void DMA_TX_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&pwm_dma);
}

#endif // NEOPIXEL_PWM
