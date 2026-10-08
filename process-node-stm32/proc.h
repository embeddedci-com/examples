/*
 * Pure process logic for the node: ADC scaling, filtering, the temperature
 * sensor conversion and the thermostat's PI controller. No MCU or HAL, so
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

/* Temperature sensor on the analog input: 0.5 V at 0 degC, 20 mV/degC (an
 * analog sensor like a TMP36 with more gain). Temperatures are in deci-degC. */
#define PROC_SENSOR_MV_AT_0C 500
#define PROC_SENSOR_MV_PER_DC 2 /* 20 mV/degC = 2 mV per 0.1 degC */

static inline int32_t proc_mv_to_dc(uint32_t mv)
{
    return ((int32_t)mv - PROC_SENSOR_MV_AT_0C) / PROC_SENSOR_MV_PER_DC;
}

/* PI controller, integer, gains in Q8. Output is clamped to [out_min, out_max];
 * while clamped the integrator only moves back toward the range (conditional
 * integration), so a long saturation does not wind it up. */
typedef struct
{
    int32_t kp_q8;     /* output per unit of error, Q8 */
    int32_t ki_q8;     /* added to the integrator per step per unit of error, Q8 */
    int32_t out_min;
    int32_t out_max;
    int32_t integ_q8;  /* integrator state, output units Q8 */
} proc_pi_t;

static inline void proc_pi_reset(proc_pi_t *pi, int32_t out)
{
    pi->integ_q8 = out * 256; /* bumpless: start from the current output */
}

static inline int32_t proc_pi_step(proc_pi_t *pi, int32_t err)
{
    int32_t p_q8 = pi->kp_q8 * err;
    int32_t integ_q8 = pi->integ_q8 + pi->ki_q8 * err;
    int32_t out = (p_q8 + integ_q8) / 256;
    if (out > pi->out_max)
    {
        out = pi->out_max;
        if (err < 0)
        {
            pi->integ_q8 = integ_q8;
        }
    }
    else if (out < pi->out_min)
    {
        out = pi->out_min;
        if (err > 0)
        {
            pi->integ_q8 = integ_q8;
        }
    }
    else
    {
        pi->integ_q8 = integ_q8;
    }
    /* Keep the integrator itself inside the output range too. */
    if (pi->integ_q8 > pi->out_max * 256)
    {
        pi->integ_q8 = pi->out_max * 256;
    }
    if (pi->integ_q8 < pi->out_min * 256)
    {
        pi->integ_q8 = pi->out_min * 256;
    }
    return out;
}

#endif
