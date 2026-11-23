/*
 * progemu_updi.c - AVR UPDI, the model of test_prog_updi.c in benchpod-firmware (stm32h563/test),
 * ported to this board: a tinyAVR (NVM version 0) and an AVR DA (NVM version 2) that parse UPDI
 * frames byte by byte, with CS registers, the NVMProg and NVMErase keys, the lock state and a
 * small NVM controller, so a wrong sequence (no erase first, a byte access to the flash page
 * buffer, no command before a store) shows up as a failed read-back or a counted violation.
 *
 * Timing per the datasheets: the UPDI starts disabled; a low pulse or a BREAK enables it and the
 * first SYNC must start 200 us to 14 ms after the pulse (within 5 ms of the BREAK's frame), else the
 * UPDI disables itself (`timing_viol`); a v2 EEPROM byte takes 10 ms to write, a streamed (no ACK)
 * byte sent meanwhile is lost (`drops`), an ACKed one is held until the write is done; NVMPROG
 * shows up 2 ms after the reset release.
 *
 * The wire side (progemu.c) measures SYNC, takes the bytes after it and sends the replies; this
 * file is the chip behind it.
 *
 *   updi   tinyAVR-like (NVM v0), 16 KB flash at 0x8000 (64-byte pages), 256 B EEPROM,
 *          10 fuse bytes, 32-byte user row; signature 1e 94 21 (id 1e9421)
 *   updimega  megaAVR 0-like (NVM v0), flash at 0x4000 (the pod picks the base from the SIB family),
 *          signature 1e 94 26 (id 1e9426)
 *   updi2  AVR DA-like (NVM v2), 16 KB flash at 0x800000 (512-byte pages), 512 B EEPROM,
 *          16 fuse bytes, 32-byte user row; signature 1e 97 0a (id 1e970a)
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define FLASH_SZ 0x4000u

enum { R_FLASH, R_EE, R_FUSE, R_USER, R_SIG, R_N };
typedef struct { uint32_t base, size; uint8_t *d; } reg_t;

enum { P_SYNC, P_OP, P_ARGS, P_DATA };
enum { C_NONE, C_LDS, C_STS, C_STS_DATA, C_STPTR, C_STCS, C_RPT, C_KEY, C_ST };

typedef struct {
    int nvm;                       /* 0 or 2 */
    char sib[17];
    reg_t r[R_N];
    uint32_t fpage, epage, upage;
    uint8_t flash[FLASH_SZ], ee[512], fuse[16], user[32], sig[16];
    /* link */
    int ps, cls;
    uint8_t op, arg[16];
    unsigned need, na;
    uint32_t units, ptr, rpt, addr;
    uint8_t cs[16];
    uint8_t disabled, locked, prog, inreset, keynvm, keyerase;
    /* NVM controller */
    uint8_t ctrla;
    int busy;
    int busy_polls;
    uint8_t err;
    uint32_t nv_addr;
    uint8_t nv_data;
    uint8_t pb[512], pbd[512];
    int pb_reg;
    uint32_t pb_page;
    /* the UPDI's own timing: disabled until a pulse or a BREAK enables it, then the first SYNC must
       start inside a window (TDebZ); a v2 EEPROM or fuse byte takes 10 ms to write; NVMPROG shows up
       a little after the reset release */
    uint8_t enabled, await, busy_on;
    uint32_t t_open, t_close, busy_until, prog_at;
    uint32_t timing_viol, drops, pulses;
    /* counters */
    uint32_t frames, bad_access, bad_frame, breaks, keys, chip_erases, resets;
    uint32_t st_words, st_bytes;
} chip_t;

_Static_assert(sizeof(chip_t) <= PE_POOL_BYTES, "updi state");
#define C (*(chip_t *)(void *)pe_pool)

static pe_counts_t s_counts;

static int is_busy10(void)
{
    return C.busy_on && (int32_t)(C.busy_until - pe_now) > 0;
}

/* A store while the previous byte is still being written: with ACKs the UPDI holds the byte (and its
   ACK) until the write is done; streamed without ACKs (RSD) the byte is lost. 1 = dropped. */
static int store_while_busy(void)
{
    if (!is_busy10()) return 0;
    if (C.cs[2] & 8) { C.drops++; return 1; }
    pe_updi_hold(C.busy_until);
    return 0;
}

static void reply(const uint8_t *b, size_t n) { pe_updi_tx(b, (unsigned)n); }
static void ack(void) { uint8_t a = 0x40; reply(&a, 1); }

static reg_t *find(uint32_t addr, uint32_t *off)
{
    for (int i = 0; i < R_N; i++)
    {
        if (addr >= C.r[i].base && addr < C.r[i].base + C.r[i].size)
        {
            *off = addr - C.r[i].base;
            return &C.r[i];
        }
    }
    return NULL;
}

static void nvm_busy(void) { C.busy = C.busy_polls; }

static uint8_t mrd(uint32_t addr)
{
    chip_t *c = &C;
    if (addr >= 0x1000 && addr < 0x1010)
    {
        switch (addr - 0x1000)
        {
        case 0: return c->ctrla;
        case 2: { uint8_t v = c->err; if (c->busy > 0) { v |= 1; c->busy--; }
                  if (is_busy10()) v |= 1;
                  return v; }
        case 6: return c->nv_data;
        case 8: return (uint8_t)c->nv_addr;
        case 9: return (uint8_t)(c->nv_addr >> 8);
        }
        return 0;
    }
    uint32_t off;
    reg_t *r = find(addr, &off);
    if (!r) return 0xFF;
    return r->d[off];
}

/* flash-like region index for the page-sized helpers */
static uint32_t page_of(int ri) { return ri == R_FLASH ? C.fpage : ri == R_USER ? C.upage : C.epage; }
static int ridx(const reg_t *r) { return (int)(r - C.r); }

static void pb_clear(void) { memset(C.pb, 0xFF, sizeof(C.pb)); memset(C.pbd, 0, sizeof(C.pbd)); }

static void erase_page(int ri, uint32_t page)
{
    uint32_t ps = page_of(ri), lim = C.r[ri].size;
    for (uint32_t i = page * ps; i < (page + 1) * ps && i < lim; i++) C.r[ri].d[i] = 0xFF;
}

static void erase_flash_ee(void)
{
    memset(C.r[R_FLASH].d, 0xFF, C.r[R_FLASH].size);
    memset(C.r[R_EE].d, 0xFF, C.r[R_EE].size);
    C.chip_erases++;
}

static void v0_command(uint8_t cmd)
{
    chip_t *c = &C;
    nvm_busy();
    reg_t *r = c->pb_reg >= 0 ? &c->r[c->pb_reg] : NULL;
    uint32_t ps = r ? page_of(c->pb_reg) : 0;
    switch (cmd)
    {
    case 4: pb_clear(); c->pb_reg = -1; break;                       /* PBC */
    case 1: case 3:                                                  /* WP, ERWP */
    {
        if (!r) { c->bad_access++; break; }
        int flash = c->pb_reg == R_FLASH;
        if (cmd == 3 && flash) erase_page(c->pb_reg, c->pb_page);
        for (uint32_t i = 0; i < ps; i++)
        {
            uint32_t o = c->pb_page * ps + i;
            if (o >= r->size) break;
            if (flash) r->d[o] &= c->pb[i];
            else if (c->pbd[i]) { if (cmd == 3) r->d[o] = 0xFF; r->d[o] &= c->pb[i]; }
        }
        break;
    }
    case 5: erase_flash_ee(); break;                                 /* CHER */
    case 7:                                                          /* WFU */
        if (c->nv_addr >= c->r[R_FUSE].base && c->nv_addr < c->r[R_FUSE].base + c->r[R_FUSE].size)
            c->r[R_FUSE].d[c->nv_addr - c->r[R_FUSE].base] = c->nv_data;
        else c->bad_access++;
        break;
    default: if (cmd) c->bad_access++;
    }
}

static void mwr1(uint32_t addr, uint8_t v, int dsz)
{
    chip_t *c = &C;
    if (addr >= 0x1000 && addr < 0x1010)
    {
        switch (addr - 0x1000)
        {
        case 0:
            c->ctrla = v;
            if (c->nvm == 0) v0_command(v);
            else if (v == 0x20) { erase_flash_ee(); nvm_busy(); }    /* CHER */
            else if (v) nvm_busy();
            break;
        case 6: c->nv_data = v; break;
        case 8: c->nv_addr = (c->nv_addr & ~0xFFu) | v; break;
        case 9: c->nv_addr = (c->nv_addr & ~0xFF00u) | ((uint32_t)v << 8); break;
        case 10: c->nv_addr = (c->nv_addr & 0xFFFFu) | ((uint32_t)v << 16); break;
        }
        return;
    }
    uint32_t off;
    reg_t *r = find(addr, &off);
    if (!r || r == &c->r[R_SIG]) { c->bad_access++; return; }
    int ri = ridx(r);
    if (c->nvm == 0)
    {
        if (ri == R_FUSE) { c->bad_access++; return; }                 /* fuses only via WFU */
        if (ri == R_FLASH && dsz != 2) c->bad_access++;                /* word access to the flash buffer */
        uint32_t ps = page_of(ri);
        if (c->pb_reg < 0) c->pb_reg = ri, c->pb_page = off / ps;
        if (c->pb_reg != ri || c->pb_page != off / ps) { c->bad_access++; return; }
        c->pb[off % ps] = v;
        c->pbd[off % ps] = 1;
        return;
    }
    /* NVM v2: no page buffer, the command decides */
    uint8_t cmd = c->ctrla;
    int fl = ri == R_FLASH || ri == R_USER;
    if (fl)
    {
        if (cmd == 0x08) { erase_page(ri, off / page_of(ri)); return; }                    /* FLPER */
        if (cmd == 0x02) { if (dsz != 2 && ri == R_FLASH) c->bad_access++; r->d[off] &= v; return; }   /* FLWR */
    }
    else
    {
        if (cmd == 0x13) { r->d[off] = v; C.busy_on = 1; C.busy_until = pe_now + 10000U * pe_cyc_us; return; }                                        /* EEERWR */
        if (cmd == 0x12) { r->d[off] &= v; return; }                                       /* EEWR */
    }
    c->bad_access++;
}

static void mwr(uint32_t addr, const uint8_t *p, int dsz)
{
    for (int i = 0; i < dsz; i++) mwr1(addr + (uint32_t)i, p[i], dsz);
}

static void unlock_erase(void)
{
    erase_flash_ee();
    memset(C.r[R_USER].d, 0xFF, C.r[R_USER].size);
    C.locked = 0;
    C.keyerase = 0;
    C.cs[7] &= ~0x08;
}

static void stcs(unsigned reg, uint8_t v)
{
    chip_t *c = &C;
    if (reg == 8)
    {
        if (v == 0x59) c->inreset = 1;
        else if (v == 0 && c->inreset)
        {
            c->inreset = 0;
            c->resets++;
            if (c->keyerase) unlock_erase();
            else if (c->keynvm && !c->locked) { c->prog = 1; c->prog_at = pe_now + 2000U * pe_cyc_us; }
            c->keynvm = 0;
            if (!c->prog) c->cs[7] &= ~0x10;
        }
        return;
    }
    if (reg == 3 && (v & 4)) { c->disabled = 1; c->prog = 0; }
    c->cs[reg & 15] = v;
}

static void args_done(void)
{
    chip_t *c = &C;
    c->na = 0;
    switch (c->cls)
    {
    case C_LDS:
    {
        unsigned asz = ((c->op >> 2) & 3) + 1, dsz = (c->op & 3) + 1;
        uint32_t a = 0;
        for (unsigned i = 0; i < asz; i++) a |= (uint32_t)c->arg[i] << (8 * i);
        uint8_t o[4];
        for (unsigned i = 0; i < dsz; i++) o[i] = mrd(a + i);
        reply(o, dsz);
        c->ps = P_SYNC;
        break;
    }
    case C_STS:
    {
        unsigned asz = ((c->op >> 2) & 3) + 1;
        c->addr = 0;
        for (unsigned i = 0; i < asz; i++) c->addr |= (uint32_t)c->arg[i] << (8 * i);
        ack();
        c->cls = C_STS_DATA;
        c->need = (c->op & 3) + 1;
        c->units = 1;
        c->ps = P_DATA;
        break;
    }
    case C_STPTR:
    {
        unsigned n = (c->op & 3) + 1;
        c->ptr = 0;
        for (unsigned i = 0; i < n; i++) c->ptr |= (uint32_t)c->arg[i] << (8 * i);
        ack();
        c->ps = P_SYNC;
        break;
    }
    case C_STCS: stcs(c->op & 15, c->arg[0]); c->ps = P_SYNC; break;
    case C_RPT: c->rpt = (uint32_t)c->arg[0] + 1; c->ps = P_SYNC; break;
    case C_KEY:
    {
        char k[9];
        for (int i = 0; i < 8; i++) k[i] = (char)c->arg[7 - i];
        k[8] = 0;
        if (!strcmp(k, "NVMProg ")) { c->keynvm = 1; c->cs[7] |= 0x10; c->keys++; }
        else if (!strcmp(k, "NVMErase")) { c->keyerase = 1; c->cs[7] |= 0x08; c->keys++; }
        else c->bad_frame++;
        c->ps = P_SYNC;
        break;
    }
    default: break;
    }
}

static void data_done(void)
{
    chip_t *c = &C;
    unsigned dsz = (c->op & 3) + 1;
    c->na = 0;
    if (c->cls == C_STS_DATA)
    {
        if (store_while_busy()) { c->ps = P_SYNC; return; }
        mwr(c->addr, c->arg, (int)dsz);
        if (!(c->cs[2] & 8)) ack();
        c->ps = P_SYNC;
        return;
    }
    /* ST ptr++ with data */
    if (store_while_busy()) { c->ptr += dsz; if (--c->units == 0) c->ps = P_SYNC; return; }
    if (dsz == 2) c->st_words++; else c->st_bytes++;
    mwr(c->ptr, c->arg, (int)dsz);
    c->ptr += dsz;
    if (!(c->cs[2] & 8)) ack();
    if (--c->units == 0) c->ps = P_SYNC;
}

static void decode_op(void)
{
    chip_t *c = &C;
    uint8_t op = c->op;
    c->ps = P_SYNC;
    c->na = 0;
    c->frames++;
    switch (op >> 4)
    {
    case 0x0: c->cls = C_LDS; c->need = ((op >> 2) & 3) + 1; c->ps = P_ARGS; break;
    case 0x4: c->cls = C_STS; c->need = ((op >> 2) & 3) + 1; c->ps = P_ARGS; break;
    case 0x2:
    {
        unsigned mode = (op >> 2) & 3, dsz = (op & 3) + 1;
        uint32_t units = c->rpt ? c->rpt : 1;
        c->rpt = 0;
        if (mode == 2)
        {
            uint8_t o[3] = { (uint8_t)c->ptr, (uint8_t)(c->ptr >> 8), (uint8_t)(c->ptr >> 16) };
            reply(o, dsz);
            break;
        }
        for (uint32_t u = 0; u < units; u++)
        {
            uint8_t o[4];
            for (unsigned i = 0; i < dsz; i++) o[i] = mrd(c->ptr + i);
            reply(o, dsz);
            if (mode == 1) c->ptr += dsz;
        }
        break;
    }
    case 0x6:
    {
        unsigned mode = (op >> 2) & 3;
        if (mode == 2) { c->cls = C_STPTR; c->need = (op & 3) + 1; c->ps = P_ARGS; break; }
        c->cls = C_ST;
        c->need = (op & 3) + 1;
        c->units = c->rpt ? c->rpt : 1;
        c->rpt = 0;
        c->ps = P_DATA;
        break;
    }
    case 0x8:
    {
        unsigned reg = op & 15;
        uint8_t v = c->cs[reg];
        if (reg == 0x0B) v = (uint8_t)((c->locked ? 1 : 0) | ((c->prog && (int32_t)(pe_now - c->prog_at) >= 0) ? 8 : 0));
        reply(&v, 1);
        break;
    }
    case 0xC: c->cls = C_STCS; c->need = 1; c->ps = P_ARGS; break;
    case 0xA: c->cls = C_RPT; c->need = 1; c->ps = P_ARGS; break;
    case 0xE:
        if (op & 4) reply((const uint8_t *)c->sib, 16);
        else { c->cls = C_KEY; c->need = 8u << (op & 3); c->ps = P_ARGS; }
        break;
    default: c->bad_frame++;
    }
}

static void rx(uint8_t b)
{
    chip_t *c = &C;
    if (c->disabled) return;
    switch (c->ps)
    {
    case P_SYNC: if (b == 0x55) c->ps = P_OP; else c->bad_frame++; break;
    case P_OP: c->op = b; decode_op(); break;
    case P_ARGS: c->arg[c->na++] = b; if (c->na == c->need) args_done(); break;
    case P_DATA: c->arg[c->na++] = b; if (c->na == c->need) data_done(); break;
    }
}

/* The wire side measured a SYNC byte. */
static void sync(void)
{
    if (C.ps == P_SYNC) C.ps = P_OP;
}

static void brk(void)
{
    chip_t *c = &C;
    c->breaks++;
    c->ps = P_SYNC;
    c->disabled = 0;
    c->cs[2] = c->cs[3] = 0;
    c->rpt = 0;
    /* a BREAK enables the UPDI: the SYNC must follow within a few ms of its frame (pe_break_frame_us
       after the line goes high again: the two stop bits at 300 baud) */
    c->enabled = 1;
    c->await = 1;
    c->t_open = pe_now;
    c->t_close = pe_now + (pe_break_frame_us + 5000U) * pe_cyc_us;
}

/* A low pulse (a 0x00 at the link rate) enables a disabled UPDI; the first SYNC may start 200 us to
   14 ms after it. */
static void pulse(void)
{
    chip_t *c = &C;
    if (c->enabled) return;
    c->enabled = 1;
    c->await = 1;
    c->pulses++;
    c->t_open = pe_now + 200U * pe_cyc_us;
    c->t_close = pe_now + 14000U * pe_cyc_us;
}

/* A frame starts (the first falling edge of a SYNC): 1 when the UPDI takes it. A SYNC outside the
   window after the enable disables the UPDI again. */
static int sync_gate(void)
{
    chip_t *c = &C;
    if (!c->enabled) return 0;
    if (c->await)
    {
        c->await = 0;
        if ((int32_t)(pe_now - c->t_open) < 0 || (int32_t)(c->t_close - pe_now) < 0)
        {
            c->timing_viol++;
            c->enabled = 0;
            return 0;
        }
    }
    return 1;
}

static int want_sync(void) { return C.ps == P_SYNC && !C.disabled; }

static void chip_init(int nvm)
{
    chip_t *c = &C;
    memset(c, 0, sizeof *c);
    c->nvm = nvm;
    c->busy_polls = 2;
    memset(c->flash, 0xFF, sizeof c->flash);
    memset(c->ee, 0xFF, sizeof c->ee);
    memset(c->fuse, 0xFF, sizeof c->fuse);
    memset(c->user, 0xFF, sizeof c->user);
    memset(c->sig, 0xFF, sizeof c->sig);
    c->r[R_FLASH].d = c->flash;
    c->r[R_EE].d = c->ee;
    c->r[R_FUSE].d = c->fuse;
    c->r[R_USER].d = c->user;
    c->r[R_SIG].d = c->sig;
    c->r[R_FLASH].size = FLASH_SZ;
    if (nvm == 0)
    {
        memcpy(c->sib, "tinyAVR P:0D:0-3", 16);
        c->r[R_FLASH].base = 0x8000;
        c->r[R_EE].base = 0x1400;    c->r[R_EE].size = 256;
        c->r[R_FUSE].base = 0x1280;  c->r[R_FUSE].size = 10;
        c->r[R_USER].base = 0x1300;  c->r[R_USER].size = 32;
        c->fpage = 64; c->epage = 32; c->upage = 32;
        c->sig[0] = 0x1E; c->sig[1] = 0x94; c->sig[2] = 0x21;
    }
    else
    {
        memcpy(c->sib, "AVR     P:2D:1-3", 16);
        c->r[R_FLASH].base = 0x800000;
        c->r[R_EE].base = 0x1400;      c->r[R_EE].size = 512;
        c->r[R_FUSE].base = 0x1050;    c->r[R_FUSE].size = 16;
        c->r[R_USER].base = 0x1080;    c->r[R_USER].size = 32;
        c->fpage = 512; c->epage = 1; c->upage = 32;
        c->sig[0] = 0x1E; c->sig[1] = 0x97; c->sig[2] = 0x0A;
    }
    c->r[R_SIG].base = 0x1100;
    c->r[R_SIG].size = 16;
    c->cs[0] = 0x30;
    c->ps = P_SYNC;
    c->pb_reg = -1;
    pb_clear();
    memset(&s_counts, 0, sizeof s_counts);
}

static void init0(void) { chip_init(0); }
static void init_mega(void)
{
    chip_init(0);
    memcpy(C.sib, "megaAVR P:0D:0-3", 16);
    C.r[R_FLASH].base = 0x4000;                         /* megaAVR 0: flash at 0x4000 */
    C.sig[1] = 0x94; C.sig[2] = 0x26;                   /* ATmega1608 */
}
static void init2(void) { chip_init(2); }
static void do_lock(void) { C.locked = 1; }

static const pe_counts_t *counts(void)
{
    s_counts.frames = C.frames;
    s_counts.errors = C.bad_access + C.bad_frame + C.timing_viol + C.drops;
    s_counts.timing = C.timing_viol;
    return &s_counts;
}

static void stat(void)
{
    printf("PROG_EMU updi nvm=%d frames=%lu bad_frame=%lu bad_access=%lu breaks=%lu keys=%lu "
           "chip_erases=%lu resets=%lu st_words=%lu st_bytes=%lu pulses=%lu timing_viol=%lu drops=%lu locked=%u prog=%u\r\n",
           C.nvm, (unsigned long)C.frames, (unsigned long)C.bad_frame, (unsigned long)C.bad_access,
           (unsigned long)C.breaks, (unsigned long)C.keys, (unsigned long)C.chip_erases,
           (unsigned long)C.resets, (unsigned long)C.st_words, (unsigned long)C.st_bytes,
           (unsigned long)C.pulses, (unsigned long)C.timing_viol, (unsigned long)C.drops, C.locked, C.prog);
}

static void rd_flash(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < FLASH_SZ ? C.flash[a + i] : 0xFF;
}
static void rd_ee(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < C.r[R_EE].size ? C.ee[a + i] : 0xFF;
}
static void rd_fuse(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < C.r[R_FUSE].size ? C.fuse[a + i] : 0xFF;
}
static void rd_user(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < 32U ? C.user[a + i] : 0xFF;
}

static const pe_mem_t k_mem0[] = {
    { "flash", FLASH_SZ, rd_flash }, { "eeprom", 256, rd_ee }, { "config", 10, rd_fuse }, { "user", 32, rd_user },
};
static const pe_mem_t k_mem2[] = {
    { "flash", FLASH_SZ, rd_flash }, { "eeprom", 512, rd_ee }, { "config", 16, rd_fuse }, { "user", 32, rd_user },
};

#define UPDI_CHIP(sym, nm, descr, ini, mems) \
    const pe_chip_t sym = { .name = nm, .what = descr, .uart = 1, .reset = 0, .init = ini, .lock = do_lock, \
        .stat = stat, .counts = counts, .mem = mems, .nmem = 4, .rx = rx, .sync = sync, .brk = brk, \
        .pulse = pulse, .sync_gate = sync_gate, .want_sync = want_sync }

UPDI_CHIP(pe_updi, "updi", "tinyAVR-like UPDI (NVM v0): 16 KB flash at 0x8000, 64-byte pages, id 1e9421", init0, k_mem0);
UPDI_CHIP(pe_updi2, "updi2", "AVR DA-like UPDI (NVM v2): 16 KB flash at 0x800000, 512-byte pages, id 1e970a", init2, k_mem2);
UPDI_CHIP(pe_updimega, "updimega", "megaAVR 0-like UPDI (NVM v0): flash at 0x4000, family string megaAVR, id 1e9426", init_mega, k_mem0);
