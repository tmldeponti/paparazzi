/*
 * Copyright (C) 2019 Xavier Paris
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file modules/light/light_scheduler.c
 * @brief basic LED scheduler based on WS2812 RGB led driver
 *
 * @author Xavier Paris
 * @maintainer Gautier Hattenberger <gautier.hattenberger@enac.fr>
 */

#include "modules/light/light_scheduler.h"
#include "modules/light/light_ws2812_arch.h"

#ifndef LIGHT_SCHEDULER_MODE
#define LIGHT_SCHEDULER_MODE LIGHT_SCHEDULER_MODE_RAINBOW
#endif

#ifndef LIGHT_SCHEDULER_RED
#define LIGHT_SCHEDULER_RED 255
#endif

#ifndef LIGHT_SCHEDULER_GREEN
#define LIGHT_SCHEDULER_GREEN 255
#endif

#ifndef LIGHT_SCHEDULER_BLUE
#define LIGHT_SCHEDULER_BLUE 255
#endif

#ifndef LIGHT_SCHEDULER_BRIGHTNESS
#define LIGHT_SCHEDULER_BRIGHTNESS 255
#endif

uint8_t light_scheduler_mode = LIGHT_SCHEDULER_MODE;
uint8_t light_scheduler_red = LIGHT_SCHEDULER_RED;
uint8_t light_scheduler_green = LIGHT_SCHEDULER_GREEN;
uint8_t light_scheduler_blue = LIGHT_SCHEDULER_BLUE;
uint8_t light_scheduler_brightness = LIGHT_SCHEDULER_BRIGHTNESS;

static uint32_t s = 0;

/** set one led, scaled by the brightness setting */
static void light_scheduler_set(uint32_t n, uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t k = light_scheduler_brightness;
  light_ws2812_arch_set(n, (r * k) / 255, (g * k) / 255, (b * k) / 255);
}

void light_scheduler_init(void)
{
  light_ws2812_arch_init();
}

void light_scheduler_periodic(void)
{
  uint32_t n, s0;
  for (n = 0; n < WS2812_NB_LEDS; n++) {
    if (light_scheduler_mode == LIGHT_SCHEDULER_MODE_COLOR) {
      light_scheduler_set(n, light_scheduler_red, light_scheduler_green, light_scheduler_blue);
    } else if (light_scheduler_mode == LIGHT_SCHEDULER_MODE_OFF) {
      light_ws2812_arch_set(n, 0, 0, 0);
    } else if (WS2812_NB_LEDS == 32) {
      if (n<=7) light_scheduler_set(n, 0,255,0);  // LEFT = GREEN
      else if (n<=15)
        light_scheduler_set(n, 0,0,255);           // BACK = BLUE
      else if (n<=23)
        light_scheduler_set(n, 255,0,0);           // RIGHT = RED
      else
        light_scheduler_set(n, 255,255,255);       // FRONT = WHITE
    } else {
      s0 = s + 10 * n;
      light_scheduler_set(n, s0 % 255, (s0 + 85) % 255, (s0 + 170) % 255);
    }
  }
  s += 10;
}
