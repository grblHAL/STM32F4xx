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

#if defined(LED_PWM_PORT) || defined(LED1_PWM_PORT)

#ifdef LED1_PWM_PORT
#define PWM_LEDS 2
#else
#define PWM_LEDS 1
#endif

// LED0

#if LED_PWM_PORT == GPIOB_BASE && LED_PWM_PIN == 0

#define PWM_TIMER_N     3
#define PWM_CHANNEL_N   3
#define PWM_GPIO_AF_N   2
#define DMA_INSTANCE_N  1
#define DMA_STREAM_N    7
#define DMA_CHANNEL_N   5

#elif LED_PWM_PORT == GPIOC_BASE && LED_PWM_PIN == 6

#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   1
#define PWM_GPIO_AF_N   3
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    2
#define DMA_CHANNEL_N   7

#elif LED_PWM_PORT == GPIOC_BASE && LED_PWM_PIN == 7

#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   2
#define PWM_GPIO_AF_N   3
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    3
#define DMA_CHANNEL_N   7

#elif LED_PWM_PORT == GPIOC_BASE && LED_PWM_PIN == 8

#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   3
#define PWM_GPIO_AF_N   3
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    4
#define DMA_CHANNEL_N   7

#elif LED_PWM_PORT == GPIOC_BASE && LED_PWM_PIN == 9

#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   4
#define PWM_GPIO_AF_N   3
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    7
#define DMA_CHANNEL_N   7

#endif

// LED1

#if LED1_PWM_PORT == GPIOB_BASE && LED1_PWM_PIN == 1

#define PWM1_TIMER_N    3
#define PWM1_CHANNEL_N  4
#define PWM1_GPIO_AF_N  2
#define DMA1_INSTANCE_N	1
#define DMA1_STREAM_N   2
#define DMA1_CHANNEL_N  5

#elif LED1_PWM_PORT == GPIOC_BASE && LED1_PWM_PIN == 6

#define PWM1_TIMER_N    8
#define PWM1_CHANNEL_N  1
#define PWM1_GPIO_AF_N  3
#define DMA1_INSTANCE_N 2
#define DMA1_STREAM_N   2
#define DMA1_CHANNEL_N  7

#elif LED1_PWM_PORT == GPIOC_BASE && LED1_PWM_PIN == 7

#define PWM1_TIMER_N    8
#define PWM1_CHANNEL_N  2
#define PWM1_GPIO_AF_N  3
#define DMA1_INSTANCE_N 2
#define DMA1_STREAM_N   3
#define DMA1_CHANNEL_N  7

#elif LED1_PWM_PORT == GPIOC_BASE && LED1_PWM_PIN == 8

#define PWM1_TIMER_N     8
#define PWM1_CHANNEL_N   3
#define PWM1_GPIO_AF_N   3
#define DMA1_INSTANCE_N  2
#define DMA1_STREAM_N    4
#define DMA1_CHANNEL_N   7

#elif LED1_PWM_PORT == GPIOC_BASE && LED1_PWM_PIN == 9

#define PWM_TIMER_N     8
#define PWM_CHANNEL_N   4
#define PWM_GPIO_AF_N   3
#define DMA_INSTANCE_N  2
#define DMA_STREAM_N    7
#define DMA_CHANNEL_N   7

#endif

#if PWM_LEDS == 2
#if PWM_TIMER_N != PWM1_TIMER_N
#error 'PWM LEDs must share the same timer!'
#endif
#if LED_PWM_PORT == LED1_PWM_PORT && LED_PWM_PIN == LED1_PWM_PIN
#error 'You cannot be serious!'
#endif
#endif

//---

#define timerDMAID(c) timerdmaid(c)
#define timerdmaid(c) TIM_DMA_ID_CC ## c
#define timerACH(c) timerach(c)
#define timerach(c) HAL_TIM_ACTIVE_CHANNEL_ ## c

#define PWM_TIMER           timer(PWM_TIMER_N)
#define PWM_CHANNEL         timerCH(PWM_CHANNEL_N)
#define PWM_ACTIVE_CHANNEL  timerACH(PWM_CHANNEL_N)
#define PWM_GPIO_AF         timerAF(PWM_TIMER_N, PWM_GPIO_AF_N)
#define PWM_TIMER_CLKEN     timerCLKEN(PWM_TIMER_N)
#define PWM_TIMER_DMA_ID    timerDMAID(PWM_CHANNEL_N)

#define DMA_INSTANCE        DMAinstance(DMA_INSTANCE_N)
#define DMA_STREAM          DMAstream(DMA_INSTANCE_N, DMA_STREAM_N)
#define DMA_CHANNEL         DMAchannel(DMA_CHANNEL_N)
#define DMA_INSTANCE_CLKEN  DMA_CLKEN(DMA_INSTANCE_N)
#define DMA_TX_IRQ          DMAirq(DMA_INSTANCE_N, DMA_STREAM_N)
#define DMA_TX_IRQ_HANDLER  DMAhandler(DMA_INSTANCE_N, DMA_STREAM_N)

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

#if PWM_LEDS == 2

#define PWM1_CHANNEL         timerCH(PWM1_CHANNEL_N)
#define PWM1_ACTIVE_CHANNEL  timerACH(PWM1_CHANNEL_N)
#define PWM1_GPIO_AF         timerAF(PWM_TIMER_N, PWM1_GPIO_AF_N)
//#define PWM_TIMER_CLKEN     timerCLKEN(PWM1_TIMER_N)
#define PWM1_TIMER_DMA_ID    timerDMAID(PWM1_CHANNEL_N)

#define DMA1_INSTANCE        DMAinstance(DMA1_INSTANCE_N)
#define DMA1_STREAM          DMAstream(DMA1_INSTANCE_N, DMA1_STREAM_N)
#define DMA1_CHANNEL         DMAchannel(DMA1_CHANNEL_N)
#define DMA1_INSTANCE_CLKEN  DMA_CLKEN(DMA1_INSTANCE_N)
#define DMA1_TX_IRQ          DMAirq(DMA1_INSTANCE_N, DMA1_STREAM_N)
#define DMA1_TX_IRQ_HANDLER  DMAhandler(DMA1_INSTANCE_N, DMA1_STREAM_N)

static DMA_HandleTypeDef pwm1_dma = {
    .Instance = DMA1_STREAM,
    .Init.Channel = DMA1_CHANNEL,
    .Init.Direction = DMA_MEMORY_TO_PERIPH,
    .Init.PeriphInc = DMA_PINC_DISABLE,
    .Init.MemInc = DMA_MINC_ENABLE,
    .Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
    .Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD,
    .Init.Mode = DMA_NORMAL,
    .Init.Priority = DMA_PRIORITY_MEDIUM,
    .Init.FIFOMode = DMA_FIFOMODE_DISABLE
};

#endif // PWM_LEDS == 2

static uint16_t t_high;

typedef struct {
	volatile bool busy;
	neopixel_cfg_t cfg;
	HAL_TIM_ActiveChannel channel;
} pwm_strip_t;

static settings_changed_ptr settings_changed;

static inline void rgb_24bpp_pack (uint16_t *led, rgb_color_t color, rgb_color_mask_t mask, uint8_t intensity, uint16_t t_high)
{
    uint8_t bitmask = 0b10000000;

    color = rgb_set_intensity(color, intensity);

    if(mask.G) {
        do {
            *led++ = (color.G & bitmask) ? t_high : (t_high >> 1);
        } while(bitmask >>= 1);
    } else
        led += 8;

    if(mask.R) {
        bitmask = 0b10000000;
        do {
            *led++ = (color.R & bitmask) ? t_high : (t_high >> 1);
        } while(bitmask >>= 1);
    } else
        led += 8;

    if(mask.B) {
        bitmask = 0b10000000;
        do {
            *led++ = (color.B & bitmask) ? t_high : (t_high >> 1);
        } while(bitmask >>= 1);
    }
}

static inline rgb_color_t rgb_24bpp_unpack (uint16_t *led, uint8_t intensity, uint16_t t_high)
{
    rgb_color_t color = {0};

    if(intensity) {

        uint8_t bitmask = 0b10000000;

        do {
            if(*led++ == t_high)
                color.G |= bitmask;
        } while(bitmask >>= 1);

        bitmask = 0b10000000;

        do {
            if(*led++ == t_high)
                color.R |= bitmask;
        } while(bitmask >>= 1);

        bitmask = 0b10000000;

        do {
            if(*led++ == t_high)
                color.B |= bitmask;
        } while(bitmask >>= 1);

        color = rgb_reset_intensity(color, intensity);
    }

    return color;
}

// LED0

static pwm_strip_t strip0 = { .cfg.intensity = 255 };

static inline void _write (void)
{
    if(!strip0.busy)
    	strip0.busy = HAL_TIM_PWM_Start_DMA(&pwm_timer, PWM_CHANNEL, (uint32_t *)strip0.cfg.leds, strip0.cfg.num_bytes >> 1) == HAL_OK;
}

static void neopixels_write (void)
{
    if(strip0.cfg.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(strip0.cfg.num_leds && device < strip0.cfg.num_leds) {

        rgb_24bpp_pack((uint16_t *)&strip0.cfg.leds[device * 24 * 2], color, mask, strip0.cfg.intensity, t_high);

        if(strip0.cfg.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = strip0.cfg.intensity;

    if(strip0.cfg.intensity != intensity) {

    	strip0.cfg.intensity = intensity;

        if(strip0.cfg.num_leds) {

            uint_fast16_t device = strip0.cfg.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_24bpp_unpack((uint16_t *)&strip0.cfg.leds[device * 24 * 2], prev, t_high);
                neopixel_out(device, color);
            } while(device);

            if(strip0.cfg.num_leds != 1)
                _write();
        }
    }

    return prev;
}

void DMA_TX_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&pwm_dma);
}

#if PWM_LEDS == 2

// LED1

static pwm_strip_t strip1 = { .cfg.intensity = 255 };

static inline void _write1 (void)
{
    if(!strip1.busy)
    	strip1.busy = HAL_TIM_PWM_Start_DMA(&pwm_timer, PWM1_CHANNEL, (uint32_t *)strip1.cfg.leds, strip1.cfg.num_bytes >> 1) == HAL_OK;
}

static void neopixels1_write (void)
{
    if(strip1.cfg.num_leds > 1)
        _write1();
}

static void neopixel1_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(strip1.cfg.num_leds && device < strip1.cfg.num_leds) {

        rgb_24bpp_pack((uint16_t *)&strip1.cfg.leds[device * 24 * 2], color, mask, strip1.cfg.intensity, t_high);

        if(strip1.cfg.num_leds == 1)
            _write1();
    }
}

static void neopixel1_out (uint16_t device, rgb_color_t color)
{
    neopixel1_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels1_set_intensity (uint8_t intensity)
{
    uint8_t prev = strip1.cfg.intensity;

    if(strip1.cfg.intensity != intensity) {

        strip1.cfg.intensity = intensity;

        if(strip1.cfg.num_leds) {

            uint_fast16_t device = strip1.cfg.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_24bpp_unpack((uint16_t *)&strip1.cfg.leds[device * 24 * 2], prev, t_high);
                neopixel1_out(device, color);
            } while(device);

            if(strip1.cfg.num_leds != 1)
                _write1();
        }
    }

    return prev;
}

void DMA1_TX_IRQ_HANDLER(void)
{
    HAL_DMA_IRQHandler(&pwm1_dma);
}

#endif // PWM_LEDS == 2

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(hal.rgb0.out == neopixel_out &&
        (strip0.cfg.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0)) {

    	hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(strip0.cfg.leds) {
            free(strip0.cfg.leds);
            strip0.cfg.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
        	strip0.cfg.num_bytes = hal.rgb0.num_devices * 24 * 2 + 40; // 40 -> 80?
            if((strip0.cfg.leds = calloc(strip0.cfg.num_bytes, sizeof(uint8_t))) == NULL)
            	hal.rgb0.num_devices = 0;
        }
        strip0.cfg.num_leds = hal.rgb0.num_devices;

        rgb_clear(&hal.rgb0);
    }

#if PWM_LEDS == 2

    if(hal.rgb1.out == neopixel1_out &&
       (strip1.cfg.leds == NULL || hal.rgb1.num_devices != settings->rgb_strip.length1)) {

    	hal.rgb1.num_devices = settings->rgb_strip.length0;

        if(strip1.cfg.leds) {
            free(strip1.cfg.leds);
            strip1.cfg.leds = NULL;
        }

        if(hal.rgb1.num_devices) {
        	strip1.cfg.num_bytes = hal.rgb1.num_devices * 24 * 2 + 40; // 40 -> 80?
            if((strip1.cfg.leds = calloc(strip1.cfg.num_bytes, sizeof(uint8_t))) == NULL)
            	hal.rgb1.num_devices = 0;
        }
        strip1.cfg.num_leds = hal.rgb1.num_devices;

        rgb_clear(&hal.rgb1);
    }

#endif // PWM_LEDS == 2

    if(settings_changed)
        settings_changed(settings, changed);
}

extern bool timer_claim (TIM_TypeDef *timer);
extern uint32_t timer_clk_enable (TIM_TypeDef *timer);

void neopixel_pwm_init (void)
{
    static const periph_pin_t leds0 = {
        .function = Output_LED_Adressable,
        .group = PinGroup_LED,
        .port = ((GPIO_TypeDef *)LED_PWM_PORT),
        .pin = LED_PWM_PIN,
        .mode = { .mask = PINMODE_NONE },
        .description = "Neopixels"
    };

    static bool init = false;

    uint8_t leds = 0;

    if(!init && timer_claim(pwm_timer.Instance)) {

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

        GPIO_InitTypeDef gpio = {
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLDOWN,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
            .Alternate = PWM_GPIO_AF
        };

        pwm_timer.Init.Period = timer_clk_enable(pwm_timer.Instance) / 800000 - 1;

        t_high = (pwm_timer.Init.Period / 3) * 2;

        DMA_INSTANCE_CLKEN();

        if(HAL_TIM_Base_Init(&pwm_timer) == HAL_OK &&
            HAL_TIM_ConfigClockSource(&pwm_timer, &sClockSourceConfig) == HAL_OK &&
             HAL_TIM_PWM_Init(&pwm_timer) == HAL_OK) {

			if(hal.rgb0.out == NULL && HAL_TIM_PWM_ConfigChannel(&pwm_timer, &sConfigOC, PWM_CHANNEL) == HAL_OK) {

				leds++;

				gpio.Pin = 1 << LED_PWM_PIN;
				HAL_GPIO_Init(((GPIO_TypeDef *)LED_PWM_PORT), &gpio);

				HAL_DMA_Init(&pwm_dma);
				__HAL_LINKDMA(&pwm_timer, hdma[PWM_TIMER_DMA_ID], pwm_dma);

				HAL_NVIC_SetPriority(DMA_TX_IRQ, 3, 0);
				HAL_NVIC_EnableIRQ(DMA_TX_IRQ);

				hal.periph_port.register_pin(&leds0);

				strip0.channel = PWM_ACTIVE_CHANNEL;
				strip0.cfg.intensity = 255;
				hal.rgb0.out = neopixel_out;
				hal.rgb0.out_masked = neopixel_out_masked;
				hal.rgb0.set_intensity = neopixels_set_intensity;
				hal.rgb0.write = neopixels_write;
				hal.rgb0.flags = (rgb_properties_t){ .is_strip = On };
				hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };
			}

#if PWM_LEDS == 2

			static const periph_pin_t leds1 = {
				.function = Output_LED1_Adressable,
				.group = PinGroup_LED,
				.port = ((GPIO_TypeDef *)LED1_PWM_PORT),
				.pin = LED1_PWM_PIN,
				.mode = { .mask = PINMODE_NONE },
				.description = "Neopixels"
			};

			if(hal.rgb1.out == NULL && HAL_TIM_PWM_ConfigChannel(&pwm_timer, &sConfigOC, PWM1_CHANNEL) == HAL_OK) {

				gpio.Pin = 1 << LED1_PWM_PIN;
				HAL_GPIO_Init(((GPIO_TypeDef *)LED1_PWM_PORT), &gpio);

				HAL_DMA_Init(&pwm1_dma);
				__HAL_LINKDMA(&pwm_timer, hdma[PWM1_TIMER_DMA_ID], pwm1_dma);

				HAL_NVIC_SetPriority(DMA1_TX_IRQ, 3, 0);
				HAL_NVIC_EnableIRQ(DMA1_TX_IRQ);

				hal.periph_port.register_pin(&leds1);

				strip1.channel = PWM1_ACTIVE_CHANNEL;
				hal.rgb1.out = neopixel1_out;
				hal.rgb1.out_masked = neopixel1_out_masked;
				hal.rgb1.set_intensity = neopixels1_set_intensity;
				hal.rgb1.write = neopixels1_write;
				hal.rgb1.flags = (rgb_properties_t){ .is_strip = On };
				hal.rgb1.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };
			}
#endif //  PWM_LEDS == 2
        }

        if(leds) {
            settings_changed = hal.settings_changed;
            hal.settings_changed = onSettingsChanged;
        }
    }

    init = true;
}

void HAL_TIM_PWM_PulseFinishedCallback (TIM_HandleTypeDef *htim)
{
#if PWM_LEDS == 2
	if(strip0.channel == htim->Channel)
		strip0.busy = false;
	else
		strip1.busy = false;
#else
	strip0.busy = false;
#endif

	HAL_TIM_PWM_Stop_DMA(htim, PWM_CHANNEL);
}

#endif
