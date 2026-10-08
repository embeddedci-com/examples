/* Host unit test for proc.h (ADC scaling and filtering). Run: make test-host */
#include "proc.h"
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

    if (failures)
    {
        printf("%d proc check(s) failed\n", failures);
        return 1;
    }
    printf("all proc checks passed\n");
    return 0;
}
