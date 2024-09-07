/*
  neopixel_gpo.c - GPIO support for Neopixels, blocking

  Part of grblHAL driver for STM32F4xx

  Copyright (c) 2024 Terje Io

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

#ifdef NEOPIXEL_GPO

typedef struct {
    int zeroHigh;
    int zeroLow;
    int oneHigh;
    int oneLow;
} ws2812_led_t;

static ws2812_led_t ws2812 = {
    .zeroHigh = 3,
    .zeroLow = 7,
    .oneHigh = 7,
    .oneLow = 3,
};

static settings_changed_ptr settings_changed;
static neopixel_cfg_t neopixel = { .intensity = 255 };
#ifdef LED1_PIN
static neopixel_cfg_t neopixel1 = { .intensity = 255 };
#endif


static inline void _write (void)
{
    volatile uint32_t t;
    uint32_t i = neopixel.num_bytes;
    uint8_t *led = neopixel.leds, v, mask;

    __disable_irq();

    if(led) do {
        v = *led++;
        mask = 0b10000000;
        do {
            if(v & mask) {
                DIGITAL_OUT(LED_PORT, LED_PIN, 1);
                t = ws2812.oneHigh;
                while(--t)
                    __ASM volatile ("nop");
                DIGITAL_OUT(LED_PORT, LED_PIN, 0);
                t = ws2812.oneLow;
                while(--t)
                    __ASM volatile ("nop");
            } else {
                DIGITAL_OUT(LED_PORT, LED_PIN, 1);
                t = ws2812.zeroHigh;
                while(--t)
                    __ASM volatile ("nop");
                DIGITAL_OUT(LED_PORT, LED_PIN, 0);
                t = ws2812.zeroLow;
                while(--t)
                    __ASM volatile ("nop");
            }
        } while((mask >>= 1));
    } while(--i);

    __enable_irq();
}

void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
        _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_1bpp_pack(&neopixel.leds[device * 3], color, mask, neopixel.intensity);

        if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    neopixel_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds) {

            uint_fast16_t device = neopixel.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_1bpp_unpack(&neopixel.leds[device * 3], prev);
                neopixel_out(device, color);
            } while(device);

            if(neopixel.num_leds != 1)
                _write();
        }
    }

    return prev;
}

#ifdef LED1_PIN

static void _write1 (void)
{
    volatile uint32_t t;
    uint32_t i = neopixel1.num_bytes;
    uint8_t *led = neopixel1.leds, v, mask;

    __disable_irq();

    if(led) do {
        v = *led++;
        mask = 0b10000000;
        do {
            if(v & mask) {
                DIGITAL_OUT(LED1_PORT, LED1_PIN, 1);
                t = ws2812.oneHigh;
                while(--t)
                    __ASM volatile ("nop");
                DIGITAL_OUT(LED1_PORT, LED1_PIN, 0);
                t = ws2812.oneLow;
                while(--t)
                    __ASM volatile ("nop");
            } else {
                DIGITAL_OUT(LED1_PORT, LED1_PIN, 1);
                t = ws2812.zeroHigh;
                while(--t)
                    __ASM volatile ("nop");
                DIGITAL_OUT(LED1_PORT, LED1_PIN, 0);
                t = ws2812.zeroLow;
                while(--t)
                    __ASM volatile ("nop");
            }
        } while((mask >>= 1));
    } while(--i);

    __enable_irq();
}


void neopixels1_write (void)
{
    if(neopixel1.num_leds > 1)
        _write1();
}

static void neopixel1_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel1.num_leds && device < neopixel1.num_leds) {

        rgb_1bpp_pack(&neopixel1.leds[device * 3], color, mask, neopixel1.intensity);

        if(neopixel1.num_leds == 1)
            _write1();
    }
}

static void neopixel1_out (uint16_t device, rgb_color_t color)
{
    neopixel1_out_masked(device, color, (rgb_color_mask_t){ .mask = 0xFF });
}

static uint8_t neopixels1_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel1.intensity;

    if(neopixel1.intensity != intensity) {

        neopixel1.intensity = intensity;

        if(neopixel1.num_leds) {

            uint_fast16_t device = neopixel1.num_leds;
            do {
                device--;
                rgb_color_t color = rgb_1bpp_unpack(&neopixel1.leds[device * 3], prev);
                neopixel1_out(device, color);
            } while(device);

            if(neopixel1.num_leds != 1)
                _write1();
        }
    }

    return prev;
}

#endif  // LED1_PIN

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip0_length) {

        if(settings->rgb_strip0_length == 0)
            settings->rgb_strip0_length = hal.rgb0.num_devices;
        else
            hal.rgb0.num_devices = settings->rgb_strip0_length;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 3;
            if((neopixel.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb0.num_devices = 0;
        }

        neopixel.num_leds = hal.rgb0.num_devices;
    }

#ifdef LED1_PIN

    if(neopixel1.leds == NULL || hal.rgb1.num_devices != settings->rgb_strip1_length) {

        if(settings->rgb_strip1_length == 0)
            settings->rgb_strip1_length = hal.rgb1.num_devices;
        else
            hal.rgb1.num_devices = settings->rgb_strip1_length;

        if(neopixel1.leds) {
            free(neopixel1.leds);
            neopixel1.leds = NULL;
        }

        if(hal.rgb1.num_devices) {
            neopixel1.num_bytes = hal.rgb1.num_devices * 3;
            if((neopixel1.leds = calloc(neopixel.num_bytes, sizeof(uint8_t))) == NULL)
                hal.rgb1.num_devices = 0;
        }

        neopixel1.num_leds = hal.rgb1.num_devices;
    }

#endif

    if(settings_changed)
        settings_changed(settings, changed);
}

void neopixel_init (void)
{
    static bool init = false;

    if(!init) {

        init = true;

        hal.rgb0.out = neopixel_out;
        hal.rgb0.out_masked = neopixel_out_masked;
        hal.rgb0.set_intensity = neopixels_set_intensity;
        hal.rgb0.write = neopixels_write;
        hal.rgb0.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
        hal.rgb0.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

#ifdef LED1_PIN

        hal.rgb1.out = neopixel1_out;
        hal.rgb1.out_masked = neopixel1_out_masked;
        hal.rgb1.set_intensity = neopixels1_set_intensity;
        hal.rgb0.write = neopixels_write;
        hal.rgb1.flags = (rgb_properties_t){ .is_blocking = On, .is_strip = On };
        hal.rgb1.cap = (rgb_color_t){ .R = 255, .G = 255, .B = 255 };

#endif

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;
    }
}

#endif // NEOPIXEL_GPO
