/*

  timers.h - driver code for STM32F7xx ARM processors

  Part of grblHAL

  Copyright (c) 2024 Terje Io

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

#pragma once

/* Internal API */

bool timer_claim (TIM_TypeDef *timer);
bool timer_is_claimed (TIM_TypeDef *timer);
uint32_t timer_clk_enable (TIM_TypeDef *timer);
uint32_t timer_get_clock_hz (TIM_TypeDef *timer);

/* HAL API */

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase);
bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg);
bool timerStart (hal_timer_t timer, uint32_t period);
bool timerStop (hal_timer_t timer);

/**/
