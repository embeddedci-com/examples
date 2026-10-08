/* Host unit test for proc.h and thermostat.h (scaling, filter, PI, closed loop against a
 * simulated plant). Run: make test-host */
#include "proc.h"
#include "thermostat.h"
#include "alarm.h"
#include "bmp280_comp.h"
#include "sim_thermostat.c"
#include <stdio.h>
#include <stdlib.h>

static int failures;

static void expect_eq(const char *what, long got, long want)
{
    if (got != want)
    {
        printf("FAIL %s: got %ld want %ld\n", what, got, want);
        failures++;
    }
}

static void expect_near(const char *what, long got, long want, long tol)
{
    if (labs(got - want) > tol)
    {
        printf("FAIL %s: got %ld want %ld +-%ld\n", what, got, want, tol);
        failures++;
    }
}

int main(void)
{
    /* VDDA: VREFINT reads its cal value exactly at 3.3 V, more counts at a lower VDDA. */
    expect_eq("vdda at cal", proc_vdda_mv(1500, 1500), 3300);
    expect_eq("vdda lower", proc_vdda_mv(1650, 1500), 3000);
    expect_eq("vdda higher", proc_vdda_mv(1375, 1500), 3600);
    expect_eq("vdda no read", proc_vdda_mv(0, 1500), 3300);
    expect_eq("vdda no cal", proc_vdda_mv(1500, 0), 3300);

    /* Scaling: ends, midpoint, clamp. */
    expect_eq("mv zero", proc_raw_to_mv(0, 3300), 0);
    expect_eq("mv full", proc_raw_to_mv(4095, 3300), 3300);
    expect_eq("mv mid", proc_raw_to_mv(2048, 3300), 1650);
    expect_eq("mv clamp", proc_raw_to_mv(5000, 3300), 3300);
    expect_eq("mv vdda 3.0", proc_raw_to_mv(4095, 3000), 3000);

    /* EMA: seeded, holds a constant; steps converge, from below and above. */
    uint32_t st = proc_ema_seed(1000);
    expect_eq("ema hold", proc_ema_step(&st, 1000, 3), 1000);
    uint32_t v = 0;
    for (int i = 0; i < 100; i++)
    {
        v = proc_ema_step(&st, 2000, 3);
    }
    expect_near("ema up", v, 2000, 1);
    for (int i = 0; i < 100; i++)
    {
        v = proc_ema_step(&st, 0, 3);
    }
    expect_near("ema down", v, 0, 1);
    /* One step moves 1/8 of the way with shift 3. */
    st = proc_ema_seed(0);
    expect_eq("ema one step", proc_ema_step(&st, 800, 3), 100);

    /* Sensor: 0.5 V at 0 degC, 20 mV/degC. */
    expect_eq("sensor 0C", proc_mv_to_dc(500), 0);
    expect_eq("sensor 60C", proc_mv_to_dc(1700), 600);
    expect_eq("sensor -10C", proc_mv_to_dc(300), -100);

    /* PI: P only, clamps, anti-windup. */
    proc_pi_t pi = {.kp_q8 = 256, .ki_q8 = 0, .out_min = 200, .out_max = 3000};
    proc_pi_reset(&pi, 1000);
    expect_eq("pi p", proc_pi_step(&pi, 100), 1100);
    expect_eq("pi clamp hi", proc_pi_step(&pi, 5000), 3000);
    expect_eq("pi clamp lo", proc_pi_step(&pi, -5000), 200);
    pi.kp_q8 = 0;
    pi.ki_q8 = 256;
    proc_pi_reset(&pi, 1000);
    for (int i = 0; i < 1000; i++)
    {
        (void)proc_pi_step(&pi, 100); /* saturated high for a long time */
    }
    expect_eq("pi windup capped", pi.integ_q8, 3000 * 256);
    expect_eq("pi unwinds at once", proc_pi_step(&pi, -100), 2900);

    /* Thermostat setpoint range. */
    thermo_t th;
    thermo_init(&th);
    expect_eq("sp too low", thermo_start(&th, -1, 0), -1);
    expect_eq("sp too high", thermo_start(&th, 1001, 0), -1);
    expect_eq("sp ok", thermo_start(&th, 600, 0), 0);
    expect_eq("bumpless start", th.out_mv, THERMO_OUT_MIN_MV);

    /* Closed loop against the pod's plant (fabric integer damping), the numbers the HIL
     * test uses: ambient 20 degC, 0.03 degC/mV, ~0.3 s. Settles, no big overshoot, holds. */
    static int32_t pv[1000];
    const plant_t plant = {20.0, 0.03, 3300.0, 150, 65535.0 / 48e6};
    for (int noise = 0; noise <= 15; noise += 15)
    {
        sim_noise_amp_mv = noise;
        sim_t sim;
        sim_init(&sim, &plant);
        thermo_start(&sim.th, 600, sim.heater);
        int n = sim_run(&sim, 10.0, pv, 1000);
        int32_t peak = -9999;
        for (int i = 0; i < n; i++)
        {
            peak = pv[i] > peak ? pv[i] : peak;
        }
        expect_near("loop overshoot", peak, 600, 20);
        for (int i = n - 100; i < n; i++) /* last 2 s within 1 degC */
        {
            expect_near("loop holds", pv[i], 600, 10);
        }
        /* Settled within 3 s: from step 150 on, never out of band again (noise-free). */
        if (noise == 0)
        {
            for (int i = 150; i < n; i++)
            {
                expect_near("loop settles in 3 s", pv[i], 600, 10);
            }
        }

        /* Step down 60 -> 40 degC. */
        thermo_start(&sim.th, 400, sim.heater);
        n = sim_run(&sim, 10.0, pv, 1000);
        expect_near("step down holds", pv[n - 1], 400, 10 + noise / 2);
    }
    sim_noise_amp_mv = 0;

    /* Unreachable setpoint (weak heater: max 20 + 0.015 * 3000 = 65 degC): the output saturates,
     * and after a reachable setpoint it recovers at once, without windup. */
    const plant_t weak = {20.0, 0.015, 3300.0, 150, 65535.0 / 48e6};
    sim_t sim;
    sim_init(&sim, &weak);
    thermo_start(&sim.th, 900, sim.heater);
    int n = sim_run(&sim, 20.0, pv, 1000);
    expect_eq("saturates at max", sim.heater, THERMO_OUT_MAX_MV);
    expect_near("saturated temperature", pv[n - 1], 650, 15);
    thermo_start(&sim.th, 400, sim.heater);
    n = sim_run(&sim, 10.0, pv, 1000);
    for (int i = 200; i < n; i++) /* within 4 s */
    {
        expect_near("recovers without windup", pv[i], 400, 10);
    }

    /* BMP280 compensation: the datasheet's worked example (section 8.2). */
    bmp280_calib_t cal = {27504, 26435, -1000, 36477, -10685, 3024, 2855, 140, -7, 15500, -14600, 6000};
    int32_t t_fine;
    expect_eq("bmp280 temp", bmp280_temp_cdc(&cal, 519888, &t_fine), 2508);
    expect_eq("bmp280 t_fine", t_fine, 128422);
    expect_eq("bmp280 press", bmp280_press_pa(&cal, 415148, t_fine), 100653);
    uint8_t raw[24] = {0x70, 0x6B, 0x43, 0x67, 0x18, 0xFC};
    bmp280_calib_t parsed;
    bmp280_calib_parse(raw, &parsed);
    expect_eq("calib t1", parsed.t1, 27504);
    expect_eq("calib t2", parsed.t2, 26435);
    expect_eq("calib t3", parsed.t3, -1000);

    /* Alarms: debounce, hysteresis, hold while the sensor is gone, pv only while regulating. */
    alarm_t al;
    alarm_init(&al);
    alarm_in_t in = {.env_ok = 1, .env_dc = 250, .pv_mv = 1000};
    expect_eq("quiet", alarm_eval(&al, &in), 0);
    in.env_dc = 510;
    expect_eq("one hot sample is not enough", alarm_eval(&al, &in), 0);
    in.env_dc = 250;
    expect_eq("spike forgotten", alarm_eval(&al, &in), 0);
    in.env_dc = 510;
    (void)alarm_eval(&al, &in);
    expect_eq("overtemp after 2", alarm_eval(&al, &in), ALARM_OVERTEMP);
    expect_eq("active", al.active, ALARM_OVERTEMP);
    in.env_dc = 490; /* inside the hysteresis band */
    expect_eq("hyst 1", alarm_eval(&al, &in), 0);
    expect_eq("hyst 2", alarm_eval(&al, &in), 0);
    in.env_dc = 0; /* sensor gone while hot: hold */
    in.env_ok = 0;
    in.env_lost = 1;
    (void)alarm_eval(&al, &in);
    expect_eq("env lost after 2", alarm_eval(&al, &in), ALARM_ENV_LOST);
    expect_eq("overtemp held", al.active, ALARM_OVERTEMP | ALARM_ENV_LOST);
    in.env_ok = 1;
    in.env_lost = 0;
    in.env_dc = 470;
    (void)alarm_eval(&al, &in);
    expect_eq("both clear", alarm_eval(&al, &in), ALARM_OVERTEMP | ALARM_ENV_LOST);
    in.pv_mv = 50; /* open sensor, not regulating: nothing */
    (void)alarm_eval(&al, &in);
    expect_eq("pv ignored when idle", alarm_eval(&al, &in), 0);
    in.regulating = 1;
    (void)alarm_eval(&al, &in);
    expect_eq("pv fault", alarm_eval(&al, &in), ALARM_PV_FAULT);
    in.pv_mv = 3200;
    expect_eq("pv high still fault", alarm_eval(&al, &in), 0);
    expect_eq("events", al.events, 4);

    if (failures)
    {
        printf("%d proc check(s) failed\n", failures);
        return 1;
    }
    printf("all proc checks passed\n");
    return 0;
}
