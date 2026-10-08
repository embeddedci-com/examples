/*
 * Pure process logic for the node: ADC scaling and filtering. No MCU or HAL, so
 * tests/test_proc.c runs it on the host (make test-host).
 */
#ifndef PROC_H
#define PROC_H

#include <stdint.h>

#define PROC_ADC_FULL_SCALE 4095U
#define PROC_VREFINT_CAL_MV 3300U /* VREFINT_CAL was taken at VDDA = 3.3 V */

/* VDDA from a VREFINT reading and its factory calibration value. Falls back to
 * the nominal 3.3 V when either is missing (0), so a bad read never divides by 0. */
static inline uint32_t proc_vdda_mv(uint32_t vrefint_raw, uint32_t vrefint_cal)
{
    if (vrefint_raw == 0U || vrefint_cal == 0U)
    {
        return PROC_VREFINT_CAL_MV;
    }
    return (PROC_VREFINT_CAL_MV * vrefint_cal + vrefint_raw / 2U) / vrefint_raw;
}

/* 12-bit ADC count to millivolts against VDDA, rounded. */
static inline uint32_t proc_raw_to_mv(uint32_t raw, uint32_t vdda_mv)
{
    if (raw > PROC_ADC_FULL_SCALE)
    {
        raw = PROC_ADC_FULL_SCALE;
    }
    return (raw * vdda_mv + PROC_ADC_FULL_SCALE / 2U) / PROC_ADC_FULL_SCALE;
}

/* First-order low-pass on millivolts: state is mV << 4 so small steps are not
 * lost to rounding; shift sets the time constant (n samples ~ 2^shift). Seed the
 * state with proc_ema_seed() so it does not ramp up from 0 at boot. */
static inline uint32_t proc_ema_seed(uint32_t mv)
{
    return mv << 4;
}

static inline uint32_t proc_ema_step(uint32_t *state, uint32_t mv, uint32_t shift)
{
    int32_t s = (int32_t)*state;
    s += (((int32_t)(mv << 4)) - s) >> shift;
    *state = (uint32_t)s;
    return (*state + 8U) >> 4;
}

#endif
