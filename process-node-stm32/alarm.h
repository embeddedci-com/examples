/*
 * Alarms: pure logic (no HAL), evaluated every ENV_PERIOD_MS by main.c, which turns
 * a change into the alarm output (PB0), a UART event and a CAN frame.
 *
 *   overtemp  environment temperature above the limit; clears 2.0 degC below it.
 *             Held while the environment sensor is gone: it cannot be shown cool.
 *   pv_fault  the process sensor reads outside its plausible range while the
 *             thermostat regulates (open or shorted sensor).
 *   env_lost  the environment sensor was there and stopped answering.
 *
 * Every alarm needs ALARM_DEBOUNCE consecutive evaluations to set or to clear.
 */
#ifndef ALARM_H
#define ALARM_H

#include <stdint.h>

#define ALARM_OVERTEMP 0x01U
#define ALARM_PV_FAULT 0x02U
#define ALARM_ENV_LOST 0x04U
#define ALARM_COUNT 3U

#define ALARM_DEBOUNCE 2U
#define ALARM_HYST_DC 20
#define ALARM_LIMIT_DEFAULT_DC 500 /* 50.0 degC */
#define ALARM_LIMIT_MIN_DC (-400)
#define ALARM_LIMIT_MAX_DC 850     /* the BMP280's range */
#define ALARM_PV_MIN_MV 300U       /* the sensor reads 500 mV at 0 degC */
#define ALARM_PV_MAX_MV 3100U

typedef struct
{
    uint8_t active;
    int32_t limit_dc;
    uint8_t cnt[ALARM_COUNT];
    uint32_t events; /* evaluations that changed something */
} alarm_t;

typedef struct
{
    uint8_t env_ok;
    uint8_t env_lost;
    int32_t env_dc;
    uint8_t regulating;
    uint32_t pv_mv;
} alarm_in_t;

static inline void alarm_init(alarm_t *a)
{
    a->active = 0U;
    a->limit_dc = ALARM_LIMIT_DEFAULT_DC;
    for (uint32_t i = 0; i < ALARM_COUNT; i++)
    {
        a->cnt[i] = 0U;
    }
    a->events = 0U;
}

static inline const char *alarm_name(uint8_t bit)
{
    switch (bit)
    {
    case ALARM_OVERTEMP: return "overtemp";
    case ALARM_PV_FAULT: return "pv_fault";
    case ALARM_ENV_LOST: return "env_lost";
    default: return "?";
    }
}

/* One evaluation; returns the bits that changed (set or cleared). */
static inline uint8_t alarm_eval(alarm_t *a, const alarm_in_t *in)
{
    uint8_t want = 0U;
    if (in->env_ok)
    {
        int32_t thr = (a->active & ALARM_OVERTEMP) ? a->limit_dc - ALARM_HYST_DC : a->limit_dc;
        if (in->env_dc > thr)
        {
            want |= ALARM_OVERTEMP;
        }
    }
    else
    {
        want |= (uint8_t)(a->active & ALARM_OVERTEMP);
    }
    if (in->regulating && (in->pv_mv < ALARM_PV_MIN_MV || in->pv_mv > ALARM_PV_MAX_MV))
    {
        want |= ALARM_PV_FAULT;
    }
    if (in->env_lost)
    {
        want |= ALARM_ENV_LOST;
    }

    uint8_t changed = 0U;
    for (uint32_t i = 0; i < ALARM_COUNT; i++)
    {
        uint8_t bit = (uint8_t)(1U << i);
        if ((want & bit) != (a->active & bit))
        {
            if (++a->cnt[i] >= ALARM_DEBOUNCE)
            {
                a->active ^= bit;
                changed |= bit;
                a->cnt[i] = 0U;
            }
        }
        else
        {
            a->cnt[i] = 0U;
        }
    }
    if (changed)
    {
        a->events++;
    }
    return changed;
}

#endif
