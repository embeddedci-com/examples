/*
 * progemu_pic.c - PIC ICSP with low-voltage entry, the model of test_prog_pic.c in benchpod-firmware
 * (stm32h563/test), ported to this board: the chip latches data on the falling clock edge, drives
 * read data after the rising edge, takes the 32-bit LVP key while MCLR is low, and checks the
 * spec's waits (entry hold, program and erase time, the delay between commands) against the
 * cycle counter.
 *
 * Four chips, one engine:
 *   pic    PIC16F1xxx-like: 6-bit commands, key LSb first with 33 clocks, 14-bit words, DEV 0xA4 and REV 2
 *          packed in one ID word (id a4 00 02 00)
 *   pic8   PIC16F18xxx-like: 8-bit commands, 14-bit words, device 0x30B2 rev 0x2001 (b2 30 01 20)
 *   pic18  PIC18 Q43/Q84-like: 8-bit commands, byte PC, Program Data with a payload (no row latches),
 *          Bulk Erase with a region payload, config bytes, device 0x7A31 rev 0x2003
 *   pic18q10  PIC18 Q10-like: Bulk Erase picked by the PC, 16-bit config words, device 0x7B10 rev 0x2004
 * Flash is 4096 words (8 KB), EEPROM 256 bytes, config and user ID words in "config".
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define KEY 0x4D434850u
#define NW  4096u       /* flash words */
#define EESZ 256u
#define NCFG 32u

enum { K_A, K_B16, K_B18, K_Q10 };
enum { CELL_FL, CELL_CFG, CELL_EE, CELL_RO };
enum { S_CMD, S_PW, S_PR };

typedef struct {
    int kind;
    uint8_t set8, byt;
    uint16_t mask;
    unsigned rw;
    uint32_t cp_pc;
    int cp_bit;
    uint16_t fl[NW], cfgw[NCFG], uid[16], ee[EESZ], rev, dev, idw, rsv;
    uint8_t key_lsb;               /* set A: key LSb first, 33 clocks */
    uint32_t tpint, bulk_need, bulk_mask;   /* PIC18 Q: TPINT and TERAB in us, OR of bulk payloads */
    uint16_t lat[64];
    uint8_t latv[64];
    uint16_t lat1;
    uint8_t ld_data;
    /* engine */
    int prev_clk;
    uint8_t nrst_low, prog, first_cmd, first_rise;
    uint32_t keyreg;
    int nkey;
    int st, clen, plen, k;
    uint32_t sh;
    uint8_t cmd;
    uint32_t frame;
    int out;
    uint32_t pc;
    uint32_t ts;            /* cycle stamp of the last clock fall / rise, or of the reset edge */
    uint32_t busy_us;       /* wait the next command must observe, in us */
    /* counters */
    uint32_t lrd_pc[4], lrd_v[4], nlrd;   /* the last reads served: PC and value */
    uint32_t cmds, timing_err, bad_cmds, ran, entries, bulk, rowerase, programs, bad_key;
} chip_t;

_Static_assert(sizeof(chip_t) <= PE_POOL_BYTES, "pic state");
#define C (*(chip_t *)(void *)pe_pool)
static pe_counts_t s_counts;

static uint32_t idle_cyc(void) { return pe_now - C.ts; }

static uint16_t *cell(uint32_t pc, int data_sp, uint16_t *mask, int *kind)
{
    chip_t *c = &C;
    *mask = c->mask;
    *kind = CELL_CFG;
    if (!c->byt)
    {
        if (c->kind == K_A && data_sp)
        {
            if (pc >= EESZ) return NULL;
            *mask = 0xFF; *kind = CELL_EE; return &c->ee[pc];
        }
        if (c->kind == K_B16 && pc >= 0x7000 && pc < 0x7000 + EESZ)
        {
            *mask = 0xFF; *kind = CELL_EE; return &c->ee[pc - 0x7000];
        }
        if (pc < 0x8000)
        {
            if (pc >= NW) return NULL;
            *kind = CELL_FL; return &c->fl[pc];
        }
        pc -= 0x8000;
        if (c->kind == K_A)      /* DEV<8:0> and REV<4:0> in one word at 0x8006; 0x8005 reserved */
        {
            if (pc == 5) { *kind = CELL_RO; return &c->rsv; }
            if (pc == 6) { *kind = CELL_RO; return &c->idw; }
        }
        if (pc == 5) { *kind = CELL_RO; return &c->rev; }
        if (pc == 6) { *kind = CELL_RO; return &c->dev; }
        return pc < NCFG ? &c->cfgw[pc] : NULL;
    }
    if (pc < 2 * NW) { *kind = CELL_FL; return &c->fl[pc / 2]; }
    *mask = 0xFFFF;
    if (pc >= 0x200000 && pc < 0x200020 && !(pc & 1)) return &c->uid[(pc - 0x200000) / 2];
    if (c->kind == K_Q10)
    {
        if (pc >= 0x300000 && pc < 0x300020 && !(pc & 1)) return &c->cfgw[16 + (pc - 0x300000) / 2];
    }
    else if (pc >= 0x300000 && pc < 0x300010)
    {
        *mask = 0xFF;
        return &c->cfgw[16 + pc - 0x300000];
    }
    uint32_t eeb = c->kind == K_Q10 ? 0x310000u : 0x380000u;
    *mask = 0xFF;
    if (pc >= eeb && pc < eeb + EESZ) { *kind = CELL_EE; return &c->ee[pc - eeb]; }
    *mask = 0xFFFF;
    if (pc == 0x3FFFFC) { *kind = CELL_RO; return &c->rev; }
    if (pc == 0x3FFFFE) { *kind = CELL_RO; return &c->dev; }
    return NULL;
}

static int locked(void)
{
    uint16_t m; int k;
    uint16_t *p = cell(C.cp_pc, 0, &m, &k);
    return p && !((*p >> C.cp_bit) & 1);
}

static unsigned widx(uint32_t pc) { return C.byt ? pc / 2 : pc; }

static unsigned step(uint32_t pc, int ds)
{
    uint16_t m; int k;
    return cell(pc, ds, &m, &k) && m == 0xFFFF && C.byt ? 2 : 1;   /* 16-bit words step by 2 bytes */
}

static void erase_all(int cfg, int ee)
{
    chip_t *c = &C;
    pe_fill16(c->fl, c->mask, NW);
    if (cfg)
    {
        for (unsigned i = 0; i < NCFG; i++) c->cfgw[i] = !c->byt ? 0x3FFF : c->kind == K_Q10 ? 0xFFFF : 0xFF;
        if (c->byt) for (unsigned i = 0; i < 16; i++) c->uid[i] = 0xFFFF;
    }
    if (ee) for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF;
}

static void do_program(void)
{
    chip_t *c = &C;
    uint16_t m; int k;
    uint16_t *p = cell(c->pc, c->ld_data, &m, &k);
    c->programs++;
    if (!p) return;
    if (k == CELL_FL)
    {
        if (locked()) return;
        unsigned wi = widx(c->pc), base = wi - wi % c->rw;
        for (unsigned j = 0; j < c->rw; j++)
        {
            if (c->latv[j] && base + j < NW) c->fl[base + j] &= c->lat[j];
        }
        memset(c->latv, 0, sizeof c->latv);
    }
    else if (k != CELL_RO)
    {
        *p &= (uint16_t)(c->lat1 | ~m);
    }
    c->busy_us = 2800;
}

static void row_erase(void)
{
    chip_t *c = &C;
    c->rowerase++;
    unsigned wi = widx(c->pc), base = wi - wi % c->rw;
    if (locked()) return;
    for (unsigned j = 0; j < c->rw && base + j < NW; j++) c->fl[base + j] = c->mask;
    c->busy_us = 2800;
}

static void load(uint32_t data, int inc, int dsp)
{
    chip_t *c = &C;
    uint16_t m; int k;
    c->ld_data = (uint8_t)dsp;
    uint16_t *p = cell(c->pc, dsp, &m, &k);
    if (p && k == CELL_FL)
    {
        unsigned j = widx(c->pc) % c->rw;
        c->lat[j] = (uint16_t)(data & m);
        c->latv[j] = 1;
    }
    else
    {
        c->lat1 = (uint16_t)(data & m);
    }
    if (inc) c->pc += step(c->pc, dsp);
}

static uint32_t rd(int dsp, int inc)
{
    chip_t *c = &C;
    uint16_t m; int k;
    uint16_t *p = cell(c->pc, dsp, &m, &k);
    uint32_t v = 0;
    if (p)
    {
        v = *p & m;
        if (k == CELL_FL && locked()) v = 0;
    }
    if (inc) c->pc += step(c->pc, dsp);
    return v;
}

/* PIC18 Q Program Data: the word is written at once, TPINT (TPDFM for EEPROM and Q43/Q84 config bytes) */
static void qprog(uint32_t d, int inc)
{
    chip_t *c = &C;
    uint16_t m; int k;
    uint16_t *p = cell(c->pc, 0, &m, &k);
    c->programs++;
    if (p && k == CELL_FL) { if (!locked()) *p &= (uint16_t)((d & m) | ~m); }
    else if (p && k != CELL_RO) *p &= (uint16_t)((d & m) | ~m);
    c->busy_us = (p && (k == CELL_EE || (k == CELL_CFG && m == 0xFF))) ? 11000 : c->tpint;
    if (inc) c->pc += step(c->pc, 0);
}

static void bulk_erase(uint32_t d)
{
    chip_t *c = &C;
    c->bulk++;
    if (c->kind == K_B18)                   /* payload: bit 0 EEPROM, 1 flash, 2 user ID, 3 config */
    {
        c->bulk_mask |= d;
        if (d & 2) pe_fill16(c->fl, 0xFFFF, NW);
        if (d & 4) for (unsigned i = 0; i < 16; i++) c->uid[i] = 0xFFFF;
        if (d & 8) for (unsigned i = 16; i < NCFG; i++) c->cfgw[i] = 0xFF;
        if (d & 1) for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF;
        c->busy_us = c->bulk_need;
        return;
    }
    if (c->kind == K_Q10)                   /* no payload: the PC picks the regions */
    {
        if (c->pc >= 0x300000 && c->pc < 0x300100)
        {
            pe_fill16(c->fl, 0xFFFF, NW);
            for (unsigned i = 0; i < 16; i++) c->uid[i] = 0xFFFF;
            for (unsigned i = 16; i < NCFG; i++) c->cfgw[i] = 0xFFFF;
        }
        else if (c->pc >= 0x310000 && c->pc < 0x310400)
        {
            for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF;
        }
        c->busy_us = c->bulk_need;
        return;
    }
    if (c->pc == 0x8000u) erase_all(1, 1); else erase_all(0, 0);
    c->busy_us = 5000;
}

/* A command with no payload, or the payload arrived. */
static void exec(uint32_t d)
{
    chip_t *c = &C;
    if (!c->set8)
    {
        switch (c->cmd)
        {
        case 0x00: c->pc = 0x8000; break;
        case 0x02: load(d, 0, 0); break;
        case 0x03: load(d, 0, 1); break;
        case 0x06: c->pc++; break;
        case 0x16: c->pc = 0; break;
        case 0x08: do_program(); break;
        case 0x09: bulk_erase(0); break;
        case 0x0B: for (unsigned i = 0; i < EESZ; i++) c->ee[i] = 0xFF; c->bulk++; c->busy_us = 5000; break;
        case 0x11: row_erase(); break;
        default: c->bad_cmds++;
        }
    }
    else
    {
        switch (c->cmd)
        {
        case 0x80: c->pc = d; break;
        case 0x00: if (c->byt) c->bad_cmds++; else load(d, 0, 0); break;
        case 0x02: if (c->byt) c->bad_cmds++; else load(d, 1, 0); break;
        case 0xF8: c->pc += step(c->pc, 0); break;
        case 0xC0: case 0xE0:
            if (c->byt) qprog(d, c->cmd == 0xE0);
            else if (c->cmd == 0xE0) do_program();
            else c->bad_cmds++;
            break;
        case 0x18: bulk_erase(d); break;
        case 0xF0: row_erase(); break;
        default: c->bad_cmds++;
        }
    }
}

static void cmd_done(void)
{
    chip_t *c = &C;
    int w, r;
    c->cmds++;
    if (!c->set8) { w = c->cmd == 0x00 || c->cmd == 0x02 || c->cmd == 0x03; r = c->cmd == 0x04 || c->cmd == 0x05; }
    else
    {
        w = c->cmd == 0x80 || (!c->byt && (c->cmd == 0x00 || c->cmd == 0x02))
            || (c->byt && (c->cmd == 0xC0 || c->cmd == 0xE0)) || (c->kind == K_B18 && c->cmd == 0x18);
        r = c->cmd == 0xFC || c->cmd == 0xFE;
    }
    c->k = 0; c->sh = 0;
    if (w) { c->st = S_PW; return; }
    if (r)
    {
        int inc = c->set8 && c->cmd == 0xFE;
        uint32_t pc0 = c->pc;
        c->frame = rd(c->cmd == 0x05, inc) << 1;
        c->lrd_pc[c->nlrd & 3] = pc0; c->lrd_v[c->nlrd & 3] = c->frame >> 1; c->nlrd++;
        c->st = S_PR;
        return;
    }
    exec(0);
    c->st = S_CMD;
}

static void on_nrst(int low)
{
    chip_t *c = &C;
    c->nrst_low = (uint8_t)low;
    c->prog = 0; c->keyreg = 0; c->nkey = 0; c->st = S_CMD; c->k = 0; c->sh = 0;
    c->ts = pe_now;
    c->busy_us = 0; c->out = -1; c->pc = 0;
    memset(c->latv, 0, sizeof c->latv);
    c->first_rise = (uint8_t)low;
    if (!low) c->ran++;
}

/* The bit of a read the next rising edge is to bring. A real chip changes its output within 80 ns
   after the rising edge; this node needs about a microsecond. A pod only samples while the clock is
   high, so the bit is put on the wire as soon as the falling edge has let the previous one go: it is
   there when the pod looks, however late the interrupt is. The first bit (start bit, always 0, which
   the pod drops) still comes with the first rising edge. */
static int read_bit_at(int k)
{
    int idx = k + pe_skew;                  /* pe_skew is 0 unless a diagnostic run asked for it */
    if (idx < 0 || idx >= C.plen) return 0;
    return (int)((C.frame >> (C.set8 ? C.plen - 1 - idx : idx)) & 1U);
}

static int read_bit(void)
{
    return read_bit_at(C.k);
}

static void rise(void)
{
    chip_t *c = &C;
    uint32_t us = pe_cyc_us;
    if (!c->nrst_low) return;
    if (c->first_rise)
    {
        if (idle_cyc() < us) c->timing_err++;
        c->first_rise = 0;
    }
    if (c->prog)
    {
        if (c->st == S_CMD && c->k == 0 && c->sh == 0)
        {
            if (c->first_cmd && idle_cyc() < (c->byt ? 1000U : 250U) * us) c->timing_err++;     /* TENTH */
            if (idle_cyc() < c->busy_us * us) c->timing_err++;              /* program / erase time */
            if (idle_cyc() < us) c->timing_err++;                            /* TDLY after a command */
            c->first_cmd = 0; c->busy_us = 0;
        }
        if (c->st != S_CMD && c->k == 0 && idle_cyc() < us) c->timing_err++;   /* TDLY command to data */
        if (c->st == S_PR) c->out = read_bit();
    }
    c->ts = pe_now;
}

static void fall(int bit)
{
    chip_t *c = &C;
    if (!c->nrst_low) return;
    c->ts = pe_now;
    if (!c->prog)
    {
        if (c->nkey < 32)
        {
            if (c->key_lsb) c->keyreg |= (uint32_t)bit << c->nkey; else c->keyreg = (c->keyreg << 1) | (uint32_t)bit;
        }
        if (++c->nkey == (c->key_lsb ? 33 : 32))
        {
            if (c->keyreg == KEY) { c->prog = 1; c->first_cmd = 1; c->entries++; }
            else c->bad_key++;
        }
        return;
    }
    if (c->st == S_CMD)
    {
        /* count the bits of the command with k, the value in sh (k = 0 and sh = 0 mark a new one) */
        if (c->set8) c->sh = (c->sh << 1) | (uint32_t)bit; else c->sh |= (uint32_t)bit << c->k;
        c->k++;
        if (c->k == c->clen) { c->cmd = (uint8_t)c->sh; cmd_done(); }
    }
    else if (c->st == S_PW)
    {
        if (c->set8) c->sh = (c->sh << 1) | (uint32_t)bit; else c->sh |= (uint32_t)bit << c->k;
        if (++c->k == c->plen) { exec(c->sh >> 1); c->st = S_CMD; c->k = 0; c->sh = 0; }
    }
    else
    {
        c->out = -1;
        if (++c->k == c->plen) { c->st = S_CMD; c->k = 0; c->sh = 0; }
        else if (c->k >= 1) c->out = read_bit();      /* the next bit is on the wire before its rising edge */
    }
}

static void edge(int clk, int pod)
{
    chip_t *c = &C;
    int prev = c->prev_clk;
    c->prev_clk = clk;
    if (clk == prev) return;
    if (clk) rise(); else fall(pod < 0 ? 1 : pod);
}

static int drive(void)
{
    return C.prog && C.st == S_PR ? C.out : -1;
}

/* The rising edge drives the next bit of a read, the falling edge lets go. */
static int peek(int clk)
{
    chip_t *c = &C;
    if (!c->prog || c->st != S_PR || !c->nrst_low) return -1;
    if (clk == 0) return c->k + 1 < c->plen ? read_bit_at(c->k + 1) : -1;
    return read_bit();
}

static void chip_init(int kind)
{
    chip_t *c = &C;
    memset(c, 0, sizeof *c);
    c->kind = kind;
    c->set8 = kind != K_A;
    c->byt = kind >= K_B18;
    c->key_lsb = kind == K_A;
    c->tpint = kind == K_Q10 ? 65 : 75;
    c->bulk_need = kind == K_Q10 ? 75000 : 11000;
    c->mask = c->byt ? 0xFFFF : 0x3FFF;
    c->clen = c->set8 ? 8 : 6;
    c->plen = c->set8 ? 24 : 16;
    c->rw = kind == K_A ? 8 : 16;
    c->prev_clk = 1;
    c->out = -1;
    erase_all(1, 1);
    if (kind == K_A)   { c->dev = 0xA4; c->rev = 0x02; c->idw = (uint16_t)(c->dev << 5 | c->rev); c->cp_pc = 0x8007; c->cp_bit = 5; }
    if (kind == K_B16) { c->dev = 0x30B2; c->rev = 0x2001; c->cp_pc = 0x800B;   c->cp_bit = 1; }
    if (kind == K_Q10) { c->dev = 0x7B10; c->rev = 0x2004; c->cp_pc = 0x300004; c->cp_bit = 3; }
    if (kind == K_B18) { c->dev = 0x7A31; c->rev = 0x2003; c->cp_pc = 0x300004; c->cp_bit = 3; }
    memset(&s_counts, 0, sizeof s_counts);
}

static void init_a(void)   { chip_init(K_A); }
static void init_b16(void) { chip_init(K_B16); }
static void init_b18(void) { chip_init(K_B18); }
static void init_q10(void) { chip_init(K_Q10); }

static void do_lock(void)
{
    uint16_t m; int k;
    uint16_t *p = cell(C.cp_pc, 0, &m, &k);
    if (p) *p &= (uint16_t)~(1u << C.cp_bit);
}

static const pe_counts_t *counts(void)
{
    s_counts.frames = C.cmds;
    s_counts.errors = C.timing_err + C.bad_cmds + C.bad_key;
    s_counts.timing = C.timing_err;
    return &s_counts;
}

static void stat(void)
{
    printf("PROG_EMU pic entries=%lu bad_key=%lu ran=%lu bad_cmds=%lu timing_err=%lu programs=%lu "
           "bulk_erase=%lu row_erase=%lu locked=%d\r\n",
           (unsigned long)C.entries, (unsigned long)C.bad_key, (unsigned long)C.ran,
           (unsigned long)C.bad_cmds, (unsigned long)C.timing_err, (unsigned long)C.programs,
           (unsigned long)C.bulk, (unsigned long)C.rowerase, locked());
    printf("PROG_EMU pic_lastrd n=%lu", (unsigned long)C.nlrd);
    for (uint32_t i = C.nlrd > 4 ? C.nlrd - 4 : 0; i < C.nlrd; i++)
        printf(" %lx:%04lx", (unsigned long)C.lrd_pc[i & 3], (unsigned long)C.lrd_v[i & 3]);
    printf("\r\n");
}

/* the memories as bytes: words little endian */
static void rd_flash(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t o = a + i;
        uint16_t w = o / 2 < NW ? C.fl[o / 2] : 0xFFFF;
        b[i] = (uint8_t)((o & 1U) ? w >> 8 : w);
    }
}

static void rd_ee(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < EESZ ? (uint8_t)C.ee[a + i] : 0xFF;
}

static void rd_cfg(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t o = a + i;
        uint16_t w = o / 2 < NCFG ? C.cfgw[o / 2] : 0xFFFF;
        b[i] = (uint8_t)((o & 1U) ? w >> 8 : w);
    }
}

static void rd_uid(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t o = a + i;
        uint16_t w = o / 2 < 16U ? C.uid[o / 2] : 0xFFFF;
        b[i] = (uint8_t)((o & 1U) ? w >> 8 : w);
    }
}

static const pe_mem_t k_mem[] = {
    { "flash",  NW * 2U, rd_flash },
    { "eeprom", EESZ,    rd_ee },
    { "config", NCFG * 2U, rd_cfg },
    { "uid", 32U, rd_uid },
};

#define PIC_CHIP(sym, nm, descr, ini) \
    const pe_chip_t sym = { .name = nm, .what = descr, .uart = 0, .reset = 1, .init = ini, \
        .lock = do_lock, .stat = stat, .counts = counts, .mem = k_mem, \
        .nmem = sizeof k_mem / sizeof k_mem[0], .edge = edge, .drive = drive, .peek = peek, .nrst = on_nrst }

PIC_CHIP(pe_pic, "pic", "PIC16F1xxx-like, 6-bit commands, key LSb first, id a4000200 (DEV 0a4 REV 02 in one word)", init_a);
PIC_CHIP(pe_pic8, "pic8", "PIC16F18xxx-like, 8-bit commands, id b2300120 (device 30b2 rev 2001)", init_b16);
PIC_CHIP(pe_pic18, "pic18", "PIC18 Q43/Q84-like, 8-bit commands, byte PC, Program Data, id 317a0320", init_b18);
PIC_CHIP(pe_pic18q10, "pic18q10", "PIC18 Q10-like, Bulk Erase by PC, 16-bit config words, id 107b0420", init_q10);
