/*
 * modbus.c - the Modbus RTU slave of the RS-485 mock (see modbus.h). Modbus Application Protocol
 * V1.1b3 for the function codes and exceptions, Modbus over Serial Line V1.02 for the RTU frame.
 */
#include "modbus.h"

#include <string.h>

uint16_t mb_crc16(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    }
    return crc;
}

void mb_init(mb_slave_t *s, uint8_t addr)
{
    memset(s, 0, sizeof(*s));
    s->addr = addr;
    for (unsigned i = 0; i < MB_INPUT_REGS; i++)
        s->input[i] = (uint16_t)(0x1000u + i);
}

static uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

static int coil_get(const mb_slave_t *s, unsigned i) { return (s->coil[i / 8u] >> (i % 8u)) & 1; }
static void coil_set(mb_slave_t *s, unsigned i, int on)
{
    if (on) s->coil[i / 8u] = (uint8_t)(s->coil[i / 8u] | (1u << (i % 8u)));
    else    s->coil[i / 8u] = (uint8_t)(s->coil[i / 8u] & ~(1u << (i % 8u)));
}

/* in range: first .. first + qty - 1 inside a table of `size` */
static int fits(unsigned first, unsigned qty, unsigned size) { return first < size && qty <= size - first; }

/* The PDU (function code onwards) of request `q` (qn bytes) -> reply PDU in r. Returns its length,
   or a negative exception code. */
static int pdu(mb_slave_t *s, const uint8_t *q, size_t qn, uint8_t *r)
{
    uint8_t fn = q[0];
    r[0] = fn;
    switch (fn) {
    case 0x01: {                                   /* Read Coils */
        if (qn != 5) return -MB_EX_VALUE;
        unsigned first = be16(q + 1), qty = be16(q + 3);
        if (qty < 1u || qty > 2000u) return -MB_EX_VALUE;
        if (!fits(first, qty, MB_COILS)) return -MB_EX_ADDRESS;
        unsigned nb = (qty + 7u) / 8u;
        r[1] = (uint8_t)nb;
        memset(r + 2, 0, nb);
        for (unsigned i = 0; i < qty; i++)
            if (coil_get(s, first + i)) r[2 + i / 8u] = (uint8_t)(r[2 + i / 8u] | (1u << (i % 8u)));
        return (int)(2u + nb);
    }
    case 0x03:                                     /* Read Holding Registers */
    case 0x04: {                                   /* Read Input Registers */
        if (qn != 5) return -MB_EX_VALUE;
        unsigned first = be16(q + 1), qty = be16(q + 3);
        if (qty < 1u || qty > 125u) return -MB_EX_VALUE;
        const uint16_t *tab = fn == 0x03 ? s->hold : s->input;
        unsigned size = fn == 0x03 ? MB_HOLD_REGS : MB_INPUT_REGS;
        if (!fits(first, qty, size)) return -MB_EX_ADDRESS;
        r[1] = (uint8_t)(2u * qty);
        for (unsigned i = 0; i < qty; i++) put16(r + 2 + 2u * i, tab[first + i]);
        return (int)(2u + 2u * qty);
    }
    case 0x05: {                                   /* Write Single Coil */
        if (qn != 5) return -MB_EX_VALUE;
        unsigned a = be16(q + 1), v = be16(q + 3);
        if (v != 0xFF00u && v != 0x0000u) return -MB_EX_VALUE;
        if (a >= MB_COILS) return -MB_EX_ADDRESS;
        coil_set(s, a, v == 0xFF00u);
        memcpy(r, q, 5);
        return 5;
    }
    case 0x06: {                                   /* Write Single Register */
        if (qn != 5) return -MB_EX_VALUE;
        unsigned a = be16(q + 1);
        if (a >= MB_HOLD_REGS) return -MB_EX_ADDRESS;
        s->hold[a] = be16(q + 3);
        memcpy(r, q, 5);
        return 5;
    }
    case 0x08: {                                   /* Diagnostics */
        if (qn < 3) return -MB_EX_VALUE;
        if (be16(q + 1) != 0x0000u) return -MB_EX_FUNCTION;   /* only "return query data" */
        memcpy(r, q, qn);
        return (int)qn;
    }
    case 0x0F: {                                   /* Write Multiple Coils */
        if (qn < 6) return -MB_EX_VALUE;
        unsigned first = be16(q + 1), qty = be16(q + 3), nb = q[5];
        if (qty < 1u || qty > 1968u || nb != (qty + 7u) / 8u || qn != 6u + nb) return -MB_EX_VALUE;
        if (!fits(first, qty, MB_COILS)) return -MB_EX_ADDRESS;
        for (unsigned i = 0; i < qty; i++) coil_set(s, first + i, (q[6 + i / 8u] >> (i % 8u)) & 1);
        memcpy(r, q, 5);
        return 5;
    }
    case 0x10: {                                   /* Write Multiple Registers */
        if (qn < 6) return -MB_EX_VALUE;
        unsigned first = be16(q + 1), qty = be16(q + 3), nb = q[5];
        if (qty < 1u || qty > 123u || nb != 2u * qty || qn != 6u + nb) return -MB_EX_VALUE;
        if (!fits(first, qty, MB_HOLD_REGS)) return -MB_EX_ADDRESS;
        for (unsigned i = 0; i < qty; i++) s->hold[first + i] = be16(q + 6 + 2u * i);
        memcpy(r, q, 5);
        return 5;
    }
    default:
        return -MB_EX_FUNCTION;
    }
}

size_t mb_frame(mb_slave_t *s, const uint8_t *f, size_t n, uint8_t *out)
{
    if (n < 4u || n > MB_FRAME_MAX) {
        s->crc_errors++;
        return 0;
    }
    uint16_t crc = mb_crc16(f, n - 2u);
    if (f[n - 2u] != (uint8_t)crc || f[n - 1u] != (uint8_t)(crc >> 8)) {
        s->crc_errors++;
        return 0;
    }
    uint8_t a = f[0];
    if (a != 0u && a != s->addr) {
        s->other_addr++;
        return 0;
    }
    s->requests++;
    s->last_fn = f[1];
    uint8_t r[MB_FRAME_MAX];
    int len = pdu(s, f + 1, n - 3u, r);
    if (a == 0u) {                 /* broadcast: done, never answered */
        s->broadcasts++;
        return 0;
    }
    size_t rn;
    out[0] = s->addr;
    if (len < 0) {
        s->exceptions++;
        out[1] = (uint8_t)(f[1] | 0x80u);
        out[2] = (uint8_t)(-len);
        rn = 3;
    } else {
        memcpy(out + 1, r, (size_t)len);
        rn = 1u + (size_t)len;
    }
    crc = mb_crc16(out, rn);
    out[rn] = (uint8_t)crc;
    out[rn + 1u] = (uint8_t)(crc >> 8);
    s->replies++;
    return rn + 2u;
}
