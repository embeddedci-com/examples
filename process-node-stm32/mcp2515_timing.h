/*
 * MCP2515 bit-timing solver (header-only, no MCU dependencies so the host unit
 * test can include it).
 *
 * One bit = SyncSeg (1 TQ) + PRSEG + PHSEG1 + PHSEG2, TQ = 2 * (BRP + 1) / Fosc.
 * We pick the smallest BRP that gives an exact bitrate with 8..25 TQ per bit
 * (more TQ = finer sample-point placement), aim for a ~80% sample point and
 * keep the MCP2515 rules: PHSEG2 >= 2, PRSEG + PHSEG1 >= PHSEG2, every segment
 * 1..8 TQ, SJW = 1 TQ.
 */
#ifndef MCP2515_TIMING_H
#define MCP2515_TIMING_H

#include <stdint.h>

typedef struct
{
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
    uint8_t brp;     /* baud-rate prescaler (register value) */
    uint8_t tq;      /* time quanta per bit */
    uint8_t prseg;   /* TQ */
    uint8_t ps1;     /* TQ */
    uint8_t ps2;     /* TQ */
} mcp2515_timing_t;

#define MCP2515_TQ_MIN 8U
#define MCP2515_TQ_MAX 25U

/* Returns 0 and fills *t, or -1 when no exact setting exists (e.g. 1 Mbit/s
 * from an 8 MHz crystal needs 4 TQ per bit, below the minimum). */
static inline int mcp2515_calc_timing(uint32_t osc_hz, uint32_t bitrate, mcp2515_timing_t *t)
{
    if (osc_hz == 0U || bitrate == 0U)
    {
        return -1;
    }
    for (uint32_t brp = 0U; brp < 64U; brp++)
    {
        uint32_t div = 2U * (brp + 1U) * bitrate;
        if (div > osc_hz)
        {
            return -1;
        }
        if ((osc_hz % div) != 0U)
        {
            continue;
        }
        uint32_t n = osc_hz / div;
        if (n > MCP2515_TQ_MAX)
        {
            continue;
        }
        if (n < MCP2515_TQ_MIN)
        {
            return -1; /* a larger BRP only makes n smaller */
        }

        uint32_t sp = (n * 8U + 5U) / 10U; /* TQ before the sample point, ~80% */
        uint32_t ps2 = n - sp;
        if (ps2 < 2U)
        {
            ps2 = 2U;
        }
        uint32_t tseg1 = n - 1U - ps2;
        uint32_t prseg = tseg1 / 2U;
        uint32_t ps1 = tseg1 - prseg;
        if (ps1 > 8U)
        {
            prseg += ps1 - 8U;
            ps1 = 8U;
        }
        if (prseg > 8U || ps2 > 8U || prseg + ps1 < ps2)
        {
            continue;
        }

        t->brp = (uint8_t)brp;
        t->tq = (uint8_t)n;
        t->prseg = (uint8_t)prseg;
        t->ps1 = (uint8_t)ps1;
        t->ps2 = (uint8_t)ps2;
        t->cnf1 = (uint8_t)brp;                                  /* SJW = 1 TQ */
        t->cnf2 = (uint8_t)(0x80U | ((ps1 - 1U) << 3) | (prseg - 1U)); /* BTLMODE=1 */
        t->cnf3 = (uint8_t)(ps2 - 1U);
        return 0;
    }
    return -1;
}

#endif /* MCP2515_TIMING_H */
