/*
 * progemu_pic.c - PIC ICSP with low-voltage entry, the model of test_prog_pic.c in benchpod-firmware
 * (stm32h563/test), ported to this board: the chip latches data on the falling clock edge, drives
 * read data after the rising edge, takes the 32-bit LVP key while MCLR is low, and checks the
 * spec's waits (entry hold, program and erase time, the delay between commands) against the
 * cycle counter.
 *
 * Three chips, one engine:
 *   pic    PIC16F1xxx-like: 6-bit commands, 14-bit words, device 0x2A5 rev 0x2002 (id a5 02 02 20)
 *   pic8   PIC16F18xxx-like: 8-bit commands, 14-bit words, device 0x30B2 rev 0x2001 (b2 30 01 20)
 *   pic18  PIC18 Q-like: 8-bit commands, byte PC, 16-bit words, device 0x7A31 rev 0x2003
 * Flash is 4096 words (8 KB), EEPROM 256 bytes, config and user ID words in "config".
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define KEY 0x4D434850u
#define NW  4096u       /* flash words */
#define EESZ 256u
#define NCFG 32u

enum { K_A, K_B16, K_B18 };
enum { CELL_FL, CELL_CFG, CELL_EE, CELL_RO };
enum { S_CMD, S_PW, S_PR };

typedef struct {
    int kind;
    uint8_t set8, byt;
    uint16_t mask;
    unsigned rw;
    uint32_t cp_pc;
    int cp_bit;
    uint16_t fl[NW], cfgw[NCFG], ee[EESZ], rev, dev;
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
        if (pc == 5) { *kind = CELL_RO; return &c->rev; }
        if (pc == 6) { *kind = CELL_RO; return &c->dev; }
        return pc < NCFG ? &c->cfgw[pc] : NULL;
    }
    if (pc < 2 * NW) { *kind = CELL_FL; return &c->fl[pc / 2]; }
    *mask = 0xFF;
    if (pc >= 0x200000 && pc < 0x200008) return &c->cfgw[pc - 0x200000];
    if (pc >= 0x300000 && pc < 0x300010) return &c->cfgw[16 + pc - 0x300000];
    if (pc >= 0x380000 && pc < 0x380000 + EESZ) { *kind = CELL_EE; return &c->ee[pc - 0x380000]; }
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
    for (unsigned i = 0; i < NW; i++) c->fl[i] = c->mask;
    if (cfg)
    {
        for (unsigned i = 0; i < NCFG; i++) c->cfgw[i] = c->byt ? 0xFF : 0x3FFF;
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

static void bulk_erase(void)
{
    chip_t *c = &C;
    c->bulk++;
    uint32_t base = c->byt ? 0x300000u : 0x8000u;
    if (c->pc == base) erase_all(1, 1); else erase_all(0, 0);
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
        case 0x09: bulk_erase(); break;
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
        case 0x00: load(d, 0, 0); break;
        case 0x02: load(d, 1, 0); break;
        case 0xF8: c->pc += step(c->pc, 0); break;
        case 0xE0: do_program(); break;
        case 0x18: bulk_erase(); break;
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
    else          { w = c->cmd == 0x80 || c->cmd == 0x00 || c->cmd == 0x02; r = c->cmd == 0xFC || c->cmd == 0xFE; }
    c->k = 0; c->sh = 0;
    if (w) { c->st = S_PW; return; }
    if (r)
    {
        int inc = c->set8 && c->cmd == 0xFE;
        c->frame = rd(c->cmd == 0x05, inc) << 1;
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
            if (c->first_cmd && idle_cyc() < 250U * us) c->timing_err++;     /* TENTH */
            if (idle_cyc() < c->busy_us * us) c->timing_err++;              /* program / erase time */
            if (idle_cyc() < us) c->timing_err++;                            /* TDLY after a command */
            c->first_cmd = 0; c->busy_us = 0;
        }
        if (c->st != S_CMD && c->k == 0 && idle_cyc() < us) c->timing_err++;   /* TDLY command to data */
        if (c->st == S_PR) c->out = (int)((c->frame >> (c->set8 ? c->plen - 1 - c->k : c->k)) & 1);
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
        c->keyreg = (c->keyreg << 1) | (uint32_t)bit;
        if (++c->nkey == 32)
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

static void chip_init(int kind)
{
    chip_t *c = &C;
    memset(c, 0, sizeof *c);
    c->kind = kind;
    c->set8 = kind != K_A;
    c->byt = kind == K_B18;
    c->mask = c->byt ? 0xFFFF : 0x3FFF;
    c->clen = c->set8 ? 8 : 6;
    c->plen = c->set8 ? 24 : 16;
    c->rw = kind == K_A ? 8 : 16;
    c->prev_clk = 1;
    c->out = -1;
    erase_all(1, 1);
    if (kind == K_A)   { c->dev = 0x2A5;  c->rev = 0x2002; c->cp_pc = 0x8007;   c->cp_bit = 5; }
    if (kind == K_B16) { c->dev = 0x30B2; c->rev = 0x2001; c->cp_pc = 0x800B;   c->cp_bit = 1; }
    if (kind == K_B18) { c->dev = 0x7A31; c->rev = 0x2003; c->cp_pc = 0x300004; c->cp_bit = 3; }
    memset(&s_counts, 0, sizeof s_counts);
}

static void init_a(void)   { chip_init(K_A); }
static void init_b16(void) { chip_init(K_B16); }
static void init_b18(void) { chip_init(K_B18); }

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

static const pe_mem_t k_mem[] = {
    { "flash",  NW * 2U, rd_flash },
    { "eeprom", EESZ,    rd_ee },
    { "config", NCFG * 2U, rd_cfg },
};

#define PIC_CHIP(sym, nm, descr, ini) \
    const pe_chip_t sym = { .name = nm, .what = descr, .uart = 0, .reset = 1, .init = ini, \
        .lock = do_lock, .stat = stat, .counts = counts, .mem = k_mem, \
        .nmem = sizeof k_mem / sizeof k_mem[0], .edge = edge, .drive = drive, .nrst = on_nrst }

PIC_CHIP(pe_pic, "pic", "PIC16F1xxx-like, 6-bit commands, id a5020220 (device 02a5 rev 2002)", init_a);
PIC_CHIP(pe_pic8, "pic8", "PIC16F18xxx-like, 8-bit commands, id b2300120 (device 30b2 rev 2001)", init_b16);
PIC_CHIP(pe_pic18, "pic18", "PIC18 Q-like, 8-bit commands, byte PC, id 317a0320 (device 7a31 rev 2003)", init_b18);
