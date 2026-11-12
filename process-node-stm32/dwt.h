/*
 * Microsecond timing from the Cortex-M4 cycle counter (DWT CYCCNT, at the core clock). It wraps
 * after 2^32 cycles (268 s at 16 MHz), so differences of two readings are always right below that.
 */
#ifndef DWT_H
#define DWT_H

#include "stm32f4xx.h"
#include <stdint.h>

static inline void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline uint32_t dwt_cycles(void)
{
    return DWT->CYCCNT;
}

static inline uint32_t dwt_cycles_to_us(uint32_t cycles)
{
    return cycles / (SystemCoreClock / 1000000U);
}

static inline void dwt_delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t n = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < n)
    {
    }
}

#endif
