/**
 * bitband_compat.h
 * STM32U585 (Cortex-M33) 向け BITBAND_PERI 互換マクロ
 */
#ifndef BITBAND_COMPAT_H
#define BITBAND_COMPAT_H

#include "stm32u5xx.h"

/* GPIO ピンをアトミックに HIGH に設定 */
#define UNO_Q_GPIO_SET(port, pin)  ((port)->BSRR = (1u << (pin)))

/* GPIO ピンをアトミックに LOW に設定 */
#define UNO_Q_GPIO_CLR(port, pin)  ((port)->BRR  = (1u << (pin)))

/* DIGITAL_IN の代替: IDR から1ビット読み取り */
#define DIGITAL_IN(port, pin)       (((port)->IDR >> (pin)) & 1u)

/* DIGITAL_OUT の代替: BSRR/BRR でアトミック書き込み */
#define DIGITAL_OUT(port, pin, on) \
    do { if (on) { UNO_Q_GPIO_SET((port),(pin)); } \
         else    { UNO_Q_GPIO_CLR((port),(pin)); } } while(0)

#endif /* BITBAND_COMPAT_H */
