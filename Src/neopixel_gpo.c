/*
  neopixel_gpo.c - GPIO support for Neopixels, blocking

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2024-2025 Terje Io

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

// TODO: tune for different variants + debug vs release builds...

#include "driver.h"

#if defined(LED_GPO_PORT) || defined(LED1_GPO_PORT)

#include "grbl/modbus.h"

typedef struct {
    int zeroHigh;
    int zeroLow;
    int oneHigh;
    int oneLow;
} ws2812_led_t;

// for SLB @ ?MHZ...
static ws2812_led_t ws2812 = {
    .zeroHigh = 3,
    .zeroLow = 7,
    .oneHigh = 7,
    .oneLow = 3,
};

static settings_changed_ptr settings_changed;

// LED0

#ifdef LED_GPO_PIN

static neopixel_cfg_t strip0 = { .intensity = 255 };

static inline void _write (void)
{
    volatile uint32_t t;
    uint32_t i = strip0.num_bytes;
    uint8_t *led = strip0.leds, v, mask;

    if(led && !modbus_isbusy()) {

        __disable_irq();

        do {
            v = *led++;
            mask = 0b10000000;
            do {
                if(v & mask) {
                    DIGITAL_OUT(LED_GPO_PORT, LED_GPO_PIN, 1);
                    t = ws2812.oneHigh;
                    while(--t)
                        __ASM volatile ("nop");
                    DIGITAL_OUT(LED_GPO_PORT, LED_GPO_PIN, 0);
                    t = ws2812.oneLow;
                    while(--t)
                        __ASM volatile ("nop");
                } else {
                    DIGITAL_OUT(LED_GPO_PORT, LED_GPO_PIN, 1);
                    t = ws2812.zeroHigh;
                    while(--t)
                        __ASM volatile ("nop");
                    DIGITAL_OUT(LED_GPO_PORT, LED_GPO_PIN, 0);
                    t = ws2812.zeroLow;
                    while(--t)
                        __ASM volatile ("nop");
                }
            } while((mask >>= 1));
        } while(--i);

        __enable_irq();
    }
}

static void neopixels_write (void)
{
    if(strip0.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(strip0.num_leds && device < strip0.num_leds) {

        rgb_1bpp_pack(&strip0.leds[device * 3], color, mask, strip0.intensity);

        if(strip0.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = strip0.intensity;

    if(strip0.intensity != intensity) {

        strip0.intensity = intensity;

        if(strip0.num_leds) {

            uint_fast16_t device = strip0.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_1bpp_unpack(&strip0.leds[device * 3], prev);
                neopixel_out(device, color);
            } while(device);

            if(strip0.num_leds != 1)
                _write();
        }
    }

    return prev;
}

#endif // LED_GPO_PIN

#ifdef LED1_GPO_PIN

static neopixel_cfg_t strip1 = { .intensity = 255 };

static void _write1 (void)
{
    volatile uint32_t t;
    uint32_t i = strip1.num_bytes;
    uint8_t *led = strip1.leds, v, mask;

    if(led && !modbus_isbusy()) {

        __disable_irq();

        do {
            v = *led++;
            mask = 0b10000000;
            do {
                if(v & mask) {
                    DIGITAL_OUT(LED1_GPO_PORT, LED1_GPO_PIN, 1);
                    t = ws2812.oneHigh;
                    while(--t)
                        __ASM volatile ("nop");
                    DIGITAL_OUT(LED1_GPO_PORT, LED1_GPO_PIN, 0);
                    t = ws2812.oneLow;
                    while(--t)
                        __ASM volatile ("nop");
                } else {
                    DIGITAL_OUT(LED1_GPO_PORT, LED1_GPO_PIN, 1);
                    t = ws2812.zeroHigh;
                    while(--t)
                        __ASM volatile ("nop");
                    DIGITAL_OUT(LED1_GPO_PORT, LED1_GPO_PIN, 0);
                    t = ws2812.zeroLow;
                    while(--t)
                        __ASM volatile ("nop");
                }
            } while((mask >>= 1));
        } while(--i);

        __enable_irq();
    }
}

void neopixels1_write (void)
{
    if(strip1.num_leds > 1)
        _write1();
}

static void neopixel1_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(strip1.num_leds && device < strip1.num_leds) {

        rgb_1bpp_pack(&strip1.leds[device * 3], color, mask, strip1.intensity);

        if(strip1.num_leds == 1)
            _write1();
    }
}

static void neopixel1_out (uint16_t device, rgb_color_t color)
{
    neopixel1_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels1_set_intensity (uint8_t intensity)
{
    uint8_t prev = strip1.intensity;

    if(strip1.intensity != intensity) {

    	strip1.intensity = intensity;

        if(strip1.num_leds) {

            uint_fast16_t device = strip1.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_1bpp_unpack(&strip1.leds[device * 3], prev);
                neopixel1_out(device, color);
            } while(device);

            if(strip1.num_leds != 1)
                _write1();
        }
    }

    return prev;
}

#endif // LED1_GPO_PIN

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
#ifdef LED_GPO_PIN

    if(hal.rgb0.out == neopixel_out && (strip0.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip.length0)) {

        hal.rgb0.num_devices = settings->rgb_strip.length0;

        if(strip0.leds) {
            free(strip0.leds);
            strip0.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            strip0.num_bytes = hal.rgb0.num_devices * 3;
            if((strip0.leds = calloc(strip0.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        strip0.num_leds = hal.rgb0.num_devices;
    }

#endif // LED_GPO_PIN

#ifdef LED1_GPO_PIN

    if(hal.rgb1.out == neopixel1_out && (strip1.leds == NULL || hal.rgb1.num_devices != settings->rgb_strip.length1)) {

        hal.rgb1.num_devices = settings->rgb_strip.length1;

        if(strip1.leds) {
            free(strip1.leds);
            strip1.leds = NULL;
        }

        if(hal.rgb1.num_devices) {
        	strip1.num_bytes = hal.rgb1.num_devices * 3;
            if((strip1.leds = calloc(strip1.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb1.num_devices = 0;
        }

        strip1.num_leds = hal.rgb1.num_devices;
    }

#endif // LED1_GPO_PIN

    if(settings_changed)
        settings_changed(settings, changed);
}

void neopixel_gpo_init (void)
{
    static bool init = false;

    uint8_t leds = 0;

    if(!init) {

        init = true;

        GPIO_InitTypeDef gpio = {
            .Mode = GPIO_MODE_OUTPUT_PP,
            .Pull = GPIO_NOPULL,
            .Speed = GPIO_SPEED_FREQ_VERY_HIGH
        };

#ifdef LED_GPO_PIN

        static const periph_pin_t leds0 = {
            .function = Output_LED_Adressable,
            .group = PinGroup_LED,
            .port = LED_GPO_PORT,
            .pin = LED_GPO_PIN,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

        if(hal.rgb0.out == NULL) {

        	leds++;
            hal.rgb0.out = neopixel_out;
            hal.rgb0.out_masked = neopixel_out_masked;
            hal.rgb0.set_intensity = neopixels_set_intensity;
            hal.rgb0.write = neopixels_write;
            hal.rgb0.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
            hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

            gpio.Pin = 1 << LED_GPO_PIN;
            HAL_GPIO_Init(LED_GPO_PORT, &gpio);

            hal.periph_port.register_pin(&leds0);
        }

#endif // LED_GPO_PIN

#ifdef LED1_GPO_PIN

        static const periph_pin_t leds1 = {
            .function = Output_LED1_Adressable,
            .group = PinGroup_LED,
            .port = LED1_GPO_PORT,
            .pin = LED1_GPO_PIN,
            .mode = { .mask = PINMODE_NONE },
            .description = "Neopixels"
        };

        if(hal.rgb1.out == NULL) {

        	leds++;
            hal.rgb1.out = neopixel1_out;
            hal.rgb1.out_masked = neopixel1_out_masked;
            hal.rgb1.set_intensity = neopixels1_set_intensity;
            hal.rgb1.write = neopixels1_write;
            hal.rgb1.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
            hal.rgb1.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

            gpio.Pin = 1 << LED1_GPO_PIN;
            HAL_GPIO_Init(LED1_GPO_PORT, &gpio);

            hal.periph_port.register_pin(&leds1);
        }

#endif // LED1_GPO_PIN

        if(leds) {

        	// TODO: verify and add others...
        	if(hal.f_mcu >= 168) {
        		ws2812.zeroHigh = ws2812.oneLow = 4;
        		ws2812.zeroLow = ws2812.oneHigh = 9;
        	}

            settings_changed = hal.settings_changed;
            hal.settings_changed = onSettingsChanged;
        }
    }
}

#endif
