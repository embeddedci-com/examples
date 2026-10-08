/*
 * Thermostat: holds the temperature on the analog input at a setpoint by driving
 * the heater on the analog output through a PI controller. Pure logic (no HAL);
 * main.c feeds it readings every THERMO_PERIOD_MS and applies its output.
 */
#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#include "proc.h"
#include <stdint.h>

#define THERMO_PERIOD_MS 20U
#define THERMO_OUT_MIN_MV 200  /* the DAC's buffered swing */
#define THERMO_OUT_MAX_MV 3000
#define THERMO_SP_MIN_DC 0     /* 0.0 degC */
#define THERMO_SP_MAX_DC 1000  /* 100.0 degC */
#define THERMO_BAND_DC 10      /* "at setpoint" = within 1.0 degC */
/* Plant ~0.3 deci-degC per mV, ~0.3 s time constant (see tests/sim_thermostat.c). */
#define THERMO_KP_Q8 768       /* 3 mV per 0.1 degC */
#define THERMO_KI_Q8 51        /* 0.2 mV per 0.1 degC per step: Ti ~ 0.3 s */

typedef struct
{
    uint8_t on;
    int32_t sp_dc;
    int32_t pv_dc;
    int32_t out_mv;
    uint32_t steps;
    uint32_t in_band_steps; /* consecutive steps within THERMO_BAND_DC */
    proc_pi_t pi;
} thermo_t;

static inline void thermo_init(thermo_t *t)
{
    t->on = 0U;
    t->sp_dc = 0;
    t->pv_dc = 0;
    t->out_mv = 0;
    t->steps = 0U;
    t->in_band_steps = 0U;
    t->pi.kp_q8 = THERMO_KP_Q8;
    t->pi.ki_q8 = THERMO_KI_Q8;
    t->pi.out_min = THERMO_OUT_MIN_MV;
    t->pi.out_max = THERMO_OUT_MAX_MV;
    t->pi.integ_q8 = 0;
}

/* Returns 0, or -1 for a setpoint outside the range (nothing changes). */
static inline int thermo_start(thermo_t *t, int32_t sp_dc, int32_t current_out_mv)
{
    if (sp_dc < THERMO_SP_MIN_DC || sp_dc > THERMO_SP_MAX_DC)
    {
        return -1;
    }
    if (!t->on)
    {
        if (current_out_mv < THERMO_OUT_MIN_MV)
        {
            current_out_mv = THERMO_OUT_MIN_MV;
        }
        proc_pi_reset(&t->pi, current_out_mv);
        t->out_mv = current_out_mv;
        t->steps = 0U;
    }
    t->sp_dc = sp_dc;
    t->in_band_steps = 0U;
    t->on = 1U;
    return 0;
}

static inline void thermo_stop(thermo_t *t)
{
    t->on = 0U;
    t->in_band_steps = 0U;
}

/* One control step with the latest analog-input reading; returns the heater mV. */
static inline int32_t thermo_step(thermo_t *t, uint32_t pv_mv)
{
    t->pv_dc = proc_mv_to_dc(pv_mv);
    int32_t err = t->sp_dc - t->pv_dc;
    t->out_mv = proc_pi_step(&t->pi, err);
    t->steps++;
    if (err <= THERMO_BAND_DC && err >= -THERMO_BAND_DC)
    {
        t->in_band_steps++;
    }
    else
    {
        t->in_band_steps = 0U;
    }
    return t->out_mv;
}

#endif
