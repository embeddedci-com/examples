/* Host test for mcp2515_timing.h: every setting must hit the bitrate exactly
 * and obey the MCP2515 segment rules. */
#include "mcp2515_timing.h"
#include <stdio.h>

static int failures;

#define CHECK(cond, ...)                 \
    do                                   \
    {                                    \
        if (!(cond))                     \
        {                                \
            printf("FAIL: " __VA_ARGS__); \
            printf("\n");                \
            failures++;                  \
        }                                \
    } while (0)

static void expect_ok(uint32_t osc, uint32_t rate)
{
    mcp2515_timing_t t;
    int r = mcp2515_calc_timing(osc, rate, &t);
    CHECK(r == 0, "%u Hz @ %u bit/s: no setting", osc, rate);
    if (r != 0)
    {
        return;
    }
    uint32_t got = osc / (2U * (t.brp + 1U) * t.tq);
    CHECK(got == rate, "%u Hz @ %u: got %u bit/s", osc, rate, got);
    CHECK(t.tq == 1U + t.prseg + t.ps1 + t.ps2, "%u @ %u: segments don't add up", osc, rate);
    CHECK(t.ps2 >= 2U && t.ps2 <= 8U, "%u @ %u: ps2=%u", osc, rate, t.ps2);
    CHECK(t.ps1 >= 1U && t.ps1 <= 8U, "%u @ %u: ps1=%u", osc, rate, t.ps1);
    CHECK(t.prseg >= 1U && t.prseg <= 8U, "%u @ %u: prseg=%u", osc, rate, t.prseg);
    CHECK(t.prseg + t.ps1 >= t.ps2, "%u @ %u: prseg+ps1 < ps2", osc, rate);
    uint32_t sp = (1U + t.prseg + t.ps1) * 100U / t.tq;
    CHECK(sp >= 70U && sp <= 85U, "%u @ %u: sample point %u%%", osc, rate, sp);
    CHECK(t.cnf1 == t.brp, "%u @ %u: cnf1", osc, rate);
    CHECK(t.cnf2 == (0x80U | ((t.ps1 - 1U) << 3) | (t.prseg - 1U)), "%u @ %u: cnf2", osc, rate);
    CHECK(t.cnf3 == t.ps2 - 1U, "%u @ %u: cnf3", osc, rate);
    printf("ok   %8u Hz @ %7u bit/s: brp=%u tq=%u prseg=%u ps1=%u ps2=%u sp=%u%% cnf=%02X %02X %02X\n",
           osc, rate, t.brp, t.tq, t.prseg, t.ps1, t.ps2, sp, t.cnf1, t.cnf2, t.cnf3);
}

static void expect_fail(uint32_t osc, uint32_t rate)
{
    mcp2515_timing_t t;
    CHECK(mcp2515_calc_timing(osc, rate, &t) != 0, "%u Hz @ %u: should be impossible", osc, rate);
}

int main(void)
{
    static const uint32_t rates[] = {125000U, 250000U, 500000U};
    for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++)
    {
        expect_ok(8000000U, rates[i]);
        expect_ok(16000000U, rates[i]);
    }
    expect_ok(16000000U, 1000000U);
    expect_fail(8000000U, 1000000U); /* 4 TQ per bit: too few */
    expect_fail(8000000U, 777777U);
    expect_fail(8000000U, 0U);

    if (failures)
    {
        printf("%d failure(s)\n", failures);
        return 1;
    }
    printf("all timing checks passed\n");
    return 0;
}
