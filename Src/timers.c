/*

  timers.c - driver code for STM32F7xx ARM processors

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

#include "driver.h"

typedef struct {
    TIM_TypeDef *timer;
    timer_resolution_t resolution;
    bool claimed;
    timer_cfg_t cfg;
    timer_cap_t cap;
    IRQn_Type irq;
    uint32_t freq_hz;
} dtimer_t;

#define LAST_TIDX -1

static dtimer_t timers[] = {
#if !IS_TIMER_CLAIMED(TIM1_BASE)
    {
        .timer = TIM1,
        .irq = TIM1_UP_TIM10_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM1) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM1),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM1)
        }
    },
#endif
#if !IS_TIMER_CLAIMED(TIM2_BASE)
    {
        .timer = TIM2,
        .irq = TIM2_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM2) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM2),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM2)
        }
    },
#endif
#if !IS_TIMER_CLAIMED(TIM3_BASE)
    {
        .timer = TIM3,
        .irq = TIM3_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM3) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM3),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM3)
        }
    },
#endif
#if !IS_TIMER_CLAIMED(TIM4_BASE)
    {
        .timer = TIM4,
        .irq = TIM4_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM4) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM4),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM4)
        }
    },
#endif
#if !IS_TIMER_CLAIMED(TIM5_BASE)
    {
        .timer = TIM5,
        .irq = TIM5_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM5) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM5),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM5)
        }
    },
#endif
#if defined(TIM6) && !IS_TIMER_CLAIMED(TIM6_BASE)
    {
        .timer = TIM6,
#ifdef STM32F412Vx
        .irq = TIM6_IRQn,
#else
        .irq = TIM6_DAC_IRQn,
#endif
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM6) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM6),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM6)
        }
    },
#endif
#if defined(TIM7) && !IS_TIMER_CLAIMED(TIM7_BASE)
    {
        .timer = TIM7,
        .irq = TIM7_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM7) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM7),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM7)
        }
    },
#endif
#ifdef TIM8
    {
        .timer = TIM8,
        .irq = TIM8_UP_TIM13_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM8) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM8),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM8)
        }
    },
#endif
#ifdef TIM9
    {
        .timer = TIM9,
        .irq = TIM1_BRK_TIM9_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM9) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM9),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM9)
        }
    },
#endif
#ifdef TIM10
    {
        .timer = TIM10,
        .irq = TIM1_UP_TIM10_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM10) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM10),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM10)
        }
    },
#endif
#ifdef TIM11
    {
        .timer = TIM11,
        .irq = TIM1_TRG_COM_TIM11_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM11) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM11),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM11)
        }
    },
#endif
#ifdef TIM12
    {
        .timer = TIM12,
        .irq = TIM8_BRK_TIM12_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM12) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM12),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM12)
        }
    },
#endif
#ifdef TIM13
    {
        .timer = TIM13,
        .irq = TIM8_UP_TIM13_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM13) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM13),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM13)
        }
    },
#endif
#ifdef TIM14
    {
        .timer = TIM14,
        .irq = TIM8_TRG_COM_TIM14_IRQn,
        .resolution = IS_TIM_32B_COUNTER_INSTANCE(TIM14) ? Timer_32bit : Timer_16bit,
        .cap = {
          .comp1 = IS_TIM_CC1_INSTANCE(TIM14),
          .comp2 = IS_TIM_CC2_INSTANCE(TIM14)
        }
    }
#endif
};

static dtimer_t *timer_get (TIM_TypeDef *timer)
{
    dtimer_t *dtimer = NULL;
    uint_fast8_t idx = sizeof(timers) / sizeof(dtimer_t);

    do {
        if(timers[--idx].timer == timer) {
            dtimer = &timers[idx];
            break;
        }
    } while(idx);

    return dtimer;
}

bool timer_claim (TIM_TypeDef *timer)
{
    bool claimed = false;
    dtimer_t *dtimer;

    if((claimed = (dtimer = timer_get(timer)) && !dtimer->claimed))
        dtimer->claimed = true;

    return claimed;
}

bool timer_is_claimed (TIM_TypeDef *timer)
{
    dtimer_t *dtimer = timer_get(timer);

    return dtimer && dtimer->claimed;
}

uint32_t timer_get_clock_hz (TIM_TypeDef *timer)
{
    dtimer_t *dtimer = timer_get(timer);

    return dtimer ? dtimer->freq_hz : 0;
}

uint32_t timer_clk_enable (TIM_TypeDef *timer)
{
    bool apb2_clock = false;
    dtimer_t *dtimer = timer_get(timer);

    if(dtimer) {

        switch((uint32_t)timer) {

#if !IS_TIMER_CLAIMED(TIM1_BASE)
            case (uint32_t)TIM1:
                apb2_clock = true;
                __HAL_RCC_TIM1_CLK_ENABLE();
                break;
#endif
#if !IS_TIMER_CLAIMED(TIM2_BASE)
            case (uint32_t)TIM2:
                __HAL_RCC_TIM2_CLK_ENABLE();
                break;
#endif
#if !IS_TIMER_CLAIMED(TIM3_BASE)
            case (uint32_t)TIM3:
                __HAL_RCC_TIM3_CLK_ENABLE();
                break;
#endif
#if !IS_TIMER_CLAIMED(TIM4_BASE)
            case (uint32_t)TIM4:
                __HAL_RCC_TIM4_CLK_ENABLE();
                break;
#endif
#if !IS_TIMER_CLAIMED(TIM5_BASE)
            case (uint32_t)TIM5:
                __HAL_RCC_TIM5_CLK_ENABLE();
                break;
#endif
#if !IS_TIMER_CLAIMED(TIM6_BASE)
            case (uint32_t)TIM6:
                __HAL_RCC_TIM6_CLK_ENABLE();
                break;
#endif
#if defined(TIM7) && !IS_TIMER_CLAIMED(TIM7_BASE)
            case (uint32_t)TIM7:
                __HAL_RCC_TIM7_CLK_ENABLE();
                break;
#endif
#ifdef TIM8
            case (uint32_t)TIM8:
                apb2_clock = true;
                __HAL_RCC_TIM8_CLK_ENABLE();
                break;
#endif
#ifdef TIM9
            case (uint32_t)TIM9:
                apb2_clock = true;
                __HAL_RCC_TIM9_CLK_ENABLE();
                break;
#endif
#ifdef TIM10
            case (uint32_t)TIM10:
                apb2_clock = true;
                __HAL_RCC_TIM10_CLK_ENABLE();
                break;
#endif
#ifdef TIM11
            case (uint32_t)TIM11:
                apb2_clock = true;
                __HAL_RCC_TIM11_CLK_ENABLE();
                break;
#endif
#ifdef TIM12
            case (uint32_t)TIM12:
                __HAL_RCC_TIM12_CLK_ENABLE();
                break;
#endif
#ifdef TIM13
            case (uint32_t)TIM13:
                __HAL_RCC_TIM13_CLK_ENABLE();
                break;
#endif
#ifdef TIM14
            case (uint32_t)TIM14:
                __HAL_RCC_TIM14_CLK_ENABLE();
                break;
#endif
        }

        dtimer->freq_hz = (apb2_clock ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq()) * 2;
    }

    return dtimer ? dtimer->freq_hz : 0;
}

hal_timer_t timerClaim (timer_cap_t cap, uint32_t timebase)
{
    hal_timer_t t;
    uint_fast8_t idx, n_timers = sizeof(timers) / sizeof(dtimer_t);

    for(idx = 0; idx < n_timers; idx++) {
        if((t = timers[idx].claimed ? NULL : &timers[idx])) {

            uint32_t f = timer_clk_enable(timers[idx].timer) / 1000;

            timers[idx].claimed = On;
            timers[idx].timer->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE /*|TIM_CR1_URS*/;
            timers[idx].timer->PSC = ((f * timebase) / 1000000) - 1;
            timers[idx].timer->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF|TIM_SR_CC2IF);
            timers[idx].timer->CNT = 0;

            HAL_NVIC_SetPriority(timers[idx].irq, 0, 1);
            NVIC_EnableIRQ(timers[idx].irq);
            break;
        }
    }

    return t;
}

bool timerCfg (hal_timer_t timer, timer_cfg_t *cfg)
{
    bool ok;

    if((ok = timer != NULL)) {

        memcpy(&((dtimer_t *)timer)->cfg, cfg, sizeof(timer_cfg_t));

        if(cfg->single_shot)
            ((dtimer_t *)timer)->timer->CR1 |= TIM_CR1_OPM;
        else
            ((dtimer_t *)timer)->timer->CR1 &= ~TIM_CR1_OPM;

        if((ok = cfg->irq0_callback && ((dtimer_t *)timer)->cap.comp1))
            ((dtimer_t *)timer)->timer->DIER |= TIM_DIER_CC1IE;
        else
            ((dtimer_t *)timer)->timer->DIER &= ~TIM_DIER_CC1IE;

        if(ok && (ok = cfg->irq1_callback && ((dtimer_t *)timer)->cap.comp2))
            ((dtimer_t *)timer)->timer->DIER |= TIM_DIER_CC2IE;
        else
            ((dtimer_t *)timer)->timer->DIER &= ~TIM_DIER_CC2IE;

        if(ok && cfg->timeout_callback)
            ((dtimer_t *)timer)->timer->DIER |= TIM_DIER_UIE;
        else
            ((dtimer_t *)timer)->timer->DIER &= ~TIM_DIER_UIE;
    }

    return ok;
}

bool timerStart (hal_timer_t timer, uint32_t period)
{
    period++;

    if(((dtimer_t *)timer)->cfg.irq1_callback) {
        ((dtimer_t *)timer)->timer->CCR2 = period;
        period += ((dtimer_t *)timer)->cfg.irq1;
    }

    if(((dtimer_t *)timer)->cfg.irq0_callback) {
        ((dtimer_t *)timer)->timer->CCR1 = period;
        period += ((dtimer_t *)timer)->cfg.irq0;
    }

    ((dtimer_t *)timer)->timer->ARR = period;

    if(!(((dtimer_t *)timer)->timer->CR1 & TIM_CR1_CEN)) {
    //    ((dtimer_t *)timer)->timer->CNT = ((dtimer_t *)timer)->timer->ARR;
        ((dtimer_t *)timer)->timer->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF|TIM_SR_CC2IF);
        if(!(((dtimer_t *)timer)->timer->DIER & TIM_DIER_UIE)) {
            ((dtimer_t *)timer)->timer->DIER |= TIM_DIER_UIE;
            ((dtimer_t *)timer)->timer->EGR = TIM_EGR_UG;
        }
        ((dtimer_t *)timer)->timer->CR1 |= TIM_CR1_CEN;
    }

    return true;
}

bool timerStop (hal_timer_t timer)
{
    ((dtimer_t *)timer)->timer->DIER = 0;
    ((dtimer_t *)timer)->timer->CR1 &= ~TIM_CR1_CEN;

    return true;
}

__attribute__((always_inline)) static inline void _irq_handler (TIM_TypeDef *timer, timer_cfg_t *cfg)
{
    uint32_t irq = timer->SR & timer->DIER;

    timer->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF|TIM_SR_CC2IF);

    if(irq & TIM_SR_UIF)
        cfg->timeout_callback(cfg->context);

    if(irq & TIM_SR_CC1IF)
        cfg->irq0_callback(cfg->context);

    if(irq & TIM_SR_CC2IF)
        cfg->irq1_callback(cfg->context);
}

#if !IS_TIMER_CLAIMED(TIM1_BASE)

enum {
  TIM1_TIDX = LAST_TIDX,
  TIM1_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM1_IDX

void TIM1_UP_TIM10_IRQHandler (void)
{
    _irq_handler(TIM1, &timers[TIM1_IDX].cfg);
}

#endif // TIM1

#if !IS_TIMER_CLAIMED(TIM2_BASE)

enum {
  TIM2_TIDX = LAST_TIDX,
  TIM2_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM2_IDX

void TIM2_IRQHandler (void)
{
    _irq_handler(TIM2, &timers[TIM2_IDX].cfg);
}

#endif // TIM2

#if !IS_TIMER_CLAIMED(TIM3_BASE)

enum {
  TIM3_TIDX = LAST_TIDX,
  TIM3_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM3_IDX

void TIM3_IRQHandler (void)
{
    _irq_handler(TIM3, &timers[TIM3_IDX].cfg);
}

#endif // TIM3

#if !IS_TIMER_CLAIMED(TIM4_BASE)

enum {
  TIM4_TIDX = LAST_TIDX,
  TIM4_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM4_IDX

void TIM4_IRQHandler (void)
{
    _irq_handler(TIM4, &timers[TIM4_IDX].cfg);
}

#endif // TIM4

#if !IS_TIMER_CLAIMED(TIM5_BASE)

enum {
  TIM5_TIDX = LAST_TIDX,
  TIM5_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM5_IDX

void TIM5_IRQHandler (void)
{
    _irq_handler(TIM5, &timers[TIM5_IDX].cfg);
}

#endif // TIM5

#if !IS_TIMER_CLAIMED(TIM6_BASE)

enum {
  TIM6_TIDX = LAST_TIDX,
  TIM6_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM6_IDX

void TIM6_DAC_IRQHandler (void)
{
    _irq_handler(TIM6, &timers[TIM6_IDX].cfg);
}

#endif // TIM6

#if defined(TIM7) && !IS_TIMER_CLAIMED(TIM7_BASE)

enum {
  TIM7_TIDX = LAST_TIDX,
  TIM7_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM7_IDX

void TIM7_IRQHandler (void)
{
    _irq_handler(TIM7, &timers[TIM7_IDX].cfg);
}

#endif // TIM7

#ifdef TIM8

enum {
  TIM8_TIDX = LAST_TIDX,
  TIM8_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM8_IDX

void TIM8_IRQHandler (void)
{
    _irq_handler(TIM8, &timers[TIM8_IDX].cfg);
}

#endif // TIM8

#ifdef TIM9

enum {
  TIM9_TIDX = LAST_TIDX,
  TIM9_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM9_IDX

void TIM9_IRQHandler (void)
{
    _irq_handler(TIM9, &timers[TIM9_IDX].cfg);
}

#endif // TIM9

#ifdef TIM10

enum {
  TIM10_TIDX = LAST_TIDX,
  TIM10_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM10_IDX

void TIM10_IRQHandler (void)
{
    _irq_handler(TIM10, &timers[TIM10_IDX].cfg);
}

#endif // TIM10

#ifdef TIM11

enum {
  TIM11_TIDX = LAST_TIDX,
  TIM11_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM11_IDX

void TIM11_IRQHandler (void)
{
    _irq_handler(TIM11, &timers[TIM11_IDX].cfg);
}

#endif // TIM11

#ifdef TIM12

enum {
  TIM12_TIDX = LAST_TIDX,
  TIM12_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM12_IDX

void TIM12_IRQHandler (void)
{
    _irq_handler(TIM12, &timers[TIM12_IDX].cfg);
}

#endif // TIM12

#ifdef TIM13

enum {
  TIM13_TIDX = LAST_TIDX,
  TIM13_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM13_IDX

void TIM13_IRQHandler (void)
{
    _irq_handler(TIM13, &timers[TIM13_IDX].cfg);
}

#endif // TIM13

#ifdef TIM14

enum {
  TIM14_TIDX = LAST_TIDX,
  TIM14_IDX
};
#undef LAST_TIDX
#define LAST_TIDX TIM14_IDX

void TIM14_IRQHandler (void)
{
    _irq_handler(TIM14, &timers[TIM14_IDX].cfg);
}

#endif // TIM14
