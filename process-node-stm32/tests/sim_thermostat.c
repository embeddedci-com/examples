/* Closed-loop simulation of the thermostat against the pod's plant emulation,
 * with the fabric's own integer damping (v += (k * delta) >>> 15, floor). Used by
 * test_proc.c to pick the controller gains; the plant numbers match
 * tests/test_thermostat.py. */
#include "proc.h"
#include "thermostat.h"

/* Measurement noise for the simulation (mV, zero mean); none unless a test sets it. */
static int sim_noise_amp_mv;
static unsigned sim_rng = 12345U;

static double sim_noise_mv(void)
{
    if (sim_noise_amp_mv == 0)
    {
        return 0.0;
    }
    sim_rng = sim_rng * 1103515245U + 12345U;
    return (double)((int)((sim_rng >> 16) % (unsigned)(2 * sim_noise_amp_mv + 1)) - sim_noise_amp_mv);
}

typedef struct
{
    double ambient_c;
    double c_per_mv; /* steady-state degC per mV of heater drive */
    double mv_full;  /* pod output volts at code 65535, in mV */
    int k;           /* fabric damping, Q15 */
    double tick_s;   /* fabric tick */
} plant_t;

static int32_t plant_target_code(const plant_t *p, int32_t heater_mv)
{
    double t = p->ambient_c + p->c_per_mv * heater_mv;
    double mv = PROC_SENSOR_MV_AT_0C + 10.0 * PROC_SENSOR_MV_PER_DC * t;
    double code = mv * 65535.0 / p->mv_full;
    if (code < 0)
    {
        code = 0;
    }
    if (code > 65535)
    {
        code = 65535;
    }
    return (int32_t)code;
}

typedef struct
{
    plant_t p;
    thermo_t th;
    int32_t v;      /* fabric output code */
    int32_t heater; /* node output, mV */
} sim_t;

/* Starts settled at ambient with the heater at its minimum, controller off. */
static void sim_init(sim_t *s, const plant_t *p)
{
    s->p = *p;
    thermo_init(&s->th);
    s->heater = THERMO_OUT_MIN_MV;
    s->v = plant_target_code(p, s->heater);
}

/* Runs `seconds` of closed loop; stores the pv of every control step in pv_dc[]
 * (up to max_n) and returns how many. */
static int sim_run(sim_t *s, double seconds, int32_t *pv_dc, int max_n)
{
    int ticks_per_ctl = (int)(THERMO_PERIOD_MS / 1000.0 / s->p.tick_s + 0.5);
    int n = 0;
    for (double t = 0; t < seconds && n < max_n; t += THERMO_PERIOD_MS / 1000.0)
    {
        for (int i = 0; i < ticks_per_ctl; i++)
        {
            int32_t delta = plant_target_code(&s->p, s->heater) - s->v;
            s->v += (s->p.k * delta) >> 15; /* arithmetic shift: floors, like >>> */
        }
        uint32_t mv = (uint32_t)(s->v * s->p.mv_full / 65535.0 + 0.5 + sim_noise_mv());
        if (s->th.on)
        {
            s->heater = thermo_step(&s->th, mv);
        }
        pv_dc[n++] = proc_mv_to_dc(mv);
    }
    return n;
}
