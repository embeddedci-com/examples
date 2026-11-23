/*
 * progemu_pdi.c - XMEGA PDI, the model of test_prog_pdi.c in benchpod-firmware (stm32h563/test),
 * ported to this board. PDI_CLK is the clock (PB14), PDI_DATA the data wire (PB12); there is no
 * reset pin wire. The chip enables its PDI after 16 idle clocks with data high followed by a start
 * bit, then takes 12-bit frames (start, 8 data bits LSB first, even parity, 2 stop bits) on the
 * rising clock edge and answers after the falling edge, with two idle guard bits first. It has the
 * NVM controller behind LDS/STS (page buffer, erase and write commands, chip erase, fuse write)
 * with busy times, a lock byte, a user signature row.
 *
 * Per the XMEGA AU manual: the key is taken only after RESET 0x59 (STCS RESET); enable needs data
 * forced high at least 1 us, then 16 idle clocks, with the clock running within 100 us; the PDI turns
 * itself off when the clock is slower than about 10 kHz (a rising edge more than 100 us after the
 * last one: `dropouts`); a chip erase switches the bus off for its 20 ms (NVMEN reads 0, nothing
 * answers); NVM space reads (fuses, lock bits, memory) need READ_NVM (0x43) loaded; a fuse write is
 * started by a PDI write to the fuse address; the flash and user row page buffers latch words.
 *
 * The part: 20 KB flash (16 KB application + 4 KB boot, 128-byte pages), 1 KB EEPROM (32-byte
 * pages), 128-byte user row, fuses 0-6 and the lock byte; device id 1e 94 41 (id 1e9441).
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define FLASH_SIZE 0x5000u
#define FPG 128u
#define EE_SIZE 1024u
#define EPG 32u
#define A_NVM 0x010001C0u

typedef struct {
    uint8_t flash[FLASH_SIZE], eeprom[EE_SIZE], user[FPG], fuse[8], lock, buf[FPG], ebuf[EPG];
    uint8_t cmd, nvm_addr[3], nvm_data;
    uint8_t cs_status, cs_ctrl, cs_reset;
    uint32_t ptr, busy_until;
    uint8_t busy;
    int prev_clk, idle1;
    uint8_t enabled, nvmen;
    int rxn;
    uint16_t rxv;
    uint8_t inframe;
    uint8_t op;
    int need, reps;
    uint8_t args[8];
    int nargs;
    uint8_t out[64];
    int outn, outi, cur;
    /* word latching in the flash/user page buffer */
    uint8_t lowtmp;
    /* PDI physical layer: enable timing, the ~10 kHz turn-off, the bus-off time of a chip erase */
    int prev_pod;
    uint8_t en_arm, en_ok, erase_on;
    uint32_t en_t, last_rise, erase_until;
    uint32_t dropouts, breaks;
    uint32_t frames_rx, parity_bad, bad_key, enables, early_nvm;
    uint32_t low_max_cyc;
    uint32_t fall_t;
} xm_t;

_Static_assert(sizeof(xm_t) <= PE_POOL_BYTES, "pdi state");
#define X (*(xm_t *)(void *)pe_pool)

static int locked(void) { return (X.lock & 3u) != 3u; }

/* A chip erase switches the PDI bus off: NVMEN reads 0 and the data space does not answer. */
static int bus_off(void)
{
    return X.erase_on && (int32_t)(X.erase_until - pe_now) > 0;
}

static int is_busy(void)
{
    return X.busy && (int32_t)(X.busy_until - pe_now) > 0;
}

static void set_busy(uint32_t us)
{
    X.busy = 1;
    X.busy_until = pe_now + us * pe_cyc_us;
}

static uint8_t mem_read(uint32_t a)
{
    xm_t *t = &X;
    if (a >= 0x01000090u && a < 0x01000093u)
    {
        static const uint8_t id[3] = { 0x1E, 0x94, 0x41 };
        return id[a - 0x01000090u];
    }
    if (!t->nvmen || bus_off()) return 0;
    if (a == A_NVM + 0x0F) return is_busy() ? 0x80 : 0;
    if (a == A_NVM + 0x0A) return t->cmd;
    if (t->cmd != 0x43) return 0;                              /* NVM space reads need READ_NVM loaded */
    if (a == 0x008F0027u) return t->lock;
    if (a >= 0x008F0020u && a < 0x008F0028u) return t->fuse[a - 0x008F0020u];
    if (locked()) return 0;
    if (a >= 0x00800000u && a < 0x00800000u + FLASH_SIZE) return t->flash[a - 0x00800000u];
    if (a >= 0x008C0000u && a < 0x008C0000u + EE_SIZE) return t->eeprom[a - 0x008C0000u];
    if (a >= 0x008E0400u && a < 0x008E0400u + FPG) return t->user[a - 0x008E0400u];
    return 0;
}

static void exec_ex(void)
{
    xm_t *t = &X;
    switch (t->cmd)
    {
    case 0x26: memset(t->buf, 0xFF, sizeof t->buf); break;
    case 0x36: memset(t->ebuf, 0xFF, sizeof t->ebuf); break;
    case 0x40:
        memset(t->flash, 0xFF, sizeof t->flash);
        memset(t->eeprom, 0xFF, sizeof t->eeprom);
        t->lock = 0xFF;
        set_busy(20000);
        t->erase_on = 1;
        t->erase_until = pe_now + 20000U * pe_cyc_us;
        break;
    }
}

static void mem_write(uint32_t a, uint8_t v)
{
    xm_t *t = &X;
    if (a >= 0x01000000u && a < 0x01000000u + 0x200u)                  /* I/O: the NVM controller */
    {
        if (!t->nvmen || bus_off()) return;
        uint32_t r = a - A_NVM;
        if (r < 3) t->nvm_addr[r] = v;
        else if (r == 4) t->nvm_data = v;
        else if (r == 0x0A) t->cmd = v;
        else if (r == 0x0B && (v & 1)) exec_ex();
        return;
    }
    if (!t->nvmen || bus_off()) return;
    if (a >= 0x008F0020u && a < 0x008F0028u)                        /* fuse write: started by this PDI write */
    {
        if (t->cmd == 0x4C && !locked()) { t->fuse[a - 0x008F0020u] &= v; set_busy(4000); }
        return;
    }
    int fl = a >= 0x00800000u && a < 0x00800000u + FLASH_SIZE;
    int ee = a >= 0x008C0000u && a < 0x008C0000u + EE_SIZE;
    int us = a >= 0x008E0400u && a < 0x008E0400u + FPG;
    if (t->cmd == 0x23 && (fl || us))                              /* word-wise buffer: the high byte latches the pair */
    {
        uint32_t o = (a - (fl ? 0x00800000u : 0x008E0400u)) % FPG;
        if (!(o & 1u)) t->lowtmp = v; else { t->buf[o - 1] = t->lowtmp; t->buf[o] = v; }
        return;
    }
    if (t->cmd == 0x33 && ee) { t->ebuf[(a - 0x008C0000u) % EPG] = v; return; }
    if (locked()) return;
    if ((fl || ee || us) && t->cmd != 0 && is_busy()) t->early_nvm++;
    if (fl && (t->cmd == 0x2E || t->cmd == 0x2F))
    {
        uint32_t o = (a - 0x00800000u) / FPG * FPG;
        if (t->cmd == 0x2F) memset(&t->flash[o], 0xFF, FPG);
        for (uint32_t i = 0; i < FPG; i++) t->flash[o + i] &= t->buf[i];
        memset(t->buf, 0xFF, sizeof t->buf);
        set_busy(4000);
    }
    else if (ee && (t->cmd == 0x34 || t->cmd == 0x35))
    {
        uint32_t o = (a - 0x008C0000u) / EPG * EPG;
        if (t->cmd == 0x35) memset(&t->eeprom[o], 0xFF, EPG);
        for (uint32_t i = 0; i < EPG; i++) t->eeprom[o + i] &= t->ebuf[i];
        memset(t->ebuf, 0xFF, sizeof t->ebuf);
        set_busy(3000);
    }
    else if (us && t->cmd == 0x18) { memset(t->user, 0xFF, sizeof t->user); set_busy(3000); }
    else if (us && t->cmd == 0x1A)
    {
        for (uint32_t i = 0; i < FPG; i++) t->user[i] &= t->buf[i];
        memset(t->buf, 0xFF, sizeof t->buf);
        set_busy(3000);
    }
}

static void queue_reply(uint8_t reply)
{
    xm_t *t = &X;
    int n = 0, ones = 0;
    t->out[n++] = 1; t->out[n++] = 1;                    /* guard time 2 */
    t->out[n++] = 0;
    for (int i = 0; i < 8; i++)
    {
        int bit = (reply >> i) & 1;
        ones += bit;
        t->out[n++] = (uint8_t)bit;
    }
    t->out[n++] = (uint8_t)(ones & 1);
    t->out[n++] = 1;
    t->out[n++] = 1;
    t->outn = n;
    t->outi = 0;
}

static void rx_byte(uint8_t b)
{
    xm_t *t = &X;
    if (t->need > 0)
    {
        t->args[t->nargs++] = b;
        if (--t->need) return;
        uint32_t a = t->args[0] | t->args[1] << 8 | (uint32_t)t->args[2] << 16 | (uint32_t)t->args[3] << 24;
        switch (t->op)
        {
        case 0x0C: queue_reply(mem_read(a)); break;
        case 0x4C: t->need = 1; t->op = 0x4D; t->nargs = 0; t->ptr = a; return;  /* address taken; data next */
        case 0x6B: t->ptr = a; break;
        case 0xA0: t->reps = t->args[0] + 1; break;
        case 0xE0:
        {
            static const uint8_t key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };
            if (t->cs_reset == 0x59 && memcmp(t->args, key, 8) == 0) { t->nvmen = 1; t->cs_status |= 2; }
            else t->bad_key++;
            break;
        }
        }
        if (t->op == 0x60 || t->op == 0x64)
        {
            mem_write(t->ptr, b);
            if (t->op == 0x64) t->ptr++;
        }
        if ((t->op & 0xF0) == 0xC0)
        {
            if ((t->op & 0xF) == 2) t->cs_ctrl = b;
            else if ((t->op & 0xF) == 1) t->cs_reset = b;
        }
        if (t->op == 0x4D) { mem_write(t->ptr, b); }
        return;
    }
    t->op = b;
    t->nargs = 0;
    if (b == 0x0C) t->need = 4;
    else if (b == 0x4C) t->need = 4;
    else if (b == 0x6B) t->need = 4;
    else if (b == 0xA0) t->need = 1;
    else if (b == 0xE0) t->need = 8;
    else if ((b & 0xF0) == 0xC0) t->need = 1;
    else if (b == 0x60 || b == 0x64) t->need = 1;
    else if (b == 0x24) { queue_reply(mem_read(t->ptr)); t->ptr++; }
    else if ((b & 0xF0) == 0x80)
    {
        uint8_t st = t->cs_status;
        if (bus_off()) st &= (uint8_t)~0x02u;
        queue_reply((b & 0xF) == 0 ? st : (b & 0xF) == 2 ? t->cs_ctrl : t->cs_reset);
    }
}

/* REPEAT: the next instruction runs reps times; ST *(ptr++) takes reps data bytes. */
static void rx_frame(uint8_t b)
{
    xm_t *t = &X;
    t->frames_rx++;
    if (t->reps > 0 && t->need == 0 && t->op == 0x64)             /* data bytes of a repeated ST */
    {
        mem_write(t->ptr++, b);
        if (--t->reps == 0) t->op = 0;
        return;
    }
    rx_byte(b);
    if (t->op == 0x64 && t->reps > 0) { t->need = 0; }
}

static void drop_pdi(void)                      /* clock slower than ~10 kHz: the PDI turns itself off */
{
    xm_t *t = &X;
    if (t->enabled) t->dropouts++;
    t->enabled = t->nvmen = 0;
    t->cs_status = t->cs_reset = 0;
    t->inframe = 0; t->need = 0; t->reps = 0; t->outn = t->outi = 0; t->cur = -1; t->idle1 = 0;
}

/* The pod's data line changed (progemu.c: EXTI on PB12 while the pod drives it): data forced high
   arms the enable timing. */
static void data_edge(int lvl)
{
    xm_t *t = &X;
    if (lvl == 1 && t->prev_pod != 1 && !t->enabled) { t->en_arm = 1; t->en_t = pe_now; }
    t->prev_pod = lvl;
}

static void edge(int clk, int pod)
{
    xm_t *t = &X;
    if (pod == 1 && t->prev_pod != 1 && !t->enabled) { t->en_arm = 1; t->en_t = pe_now; }
    t->prev_pod = pod;
    int rise = t->prev_clk == 0 && clk == 1;
    if (rise)
    {
        if (pe_now - t->last_rise > 100U * pe_cyc_us) drop_pdi();
        t->last_rise = pe_now;
        if (t->en_arm)                                       /* first clock after data went high */
        {
            uint32_t d = pe_now - t->en_t;
            t->en_ok = d >= pe_cyc_us && d <= 100U * pe_cyc_us;     /* high >= 1 us, clocks within 100 us */
            t->en_arm = 0;
        }
    }
    if (t->prev_clk == 1 && clk == 0)
    {
        t->fall_t = pe_now;
        t->cur = t->outi < t->outn ? t->out[t->outi++] : -1;
    }
    else if (rise)
    {
        uint32_t low = pe_now - t->fall_t;
        if (low > t->low_max_cyc) t->low_max_cyc = low;
        if (pod >= 0)
        {
            if (!t->inframe)
            {
                if (pod == 1) { if (t->idle1 < 1000) t->idle1++; }
                else
                {
                    t->inframe = 1; t->rxn = 0; t->rxv = 0;
                    if (!t->enabled && t->idle1 >= 16 && t->en_ok) { t->enabled = 1; t->enables++; }
                    t->en_ok = 0;
                    t->idle1 = 0;
                }
            }
            else
            {
                t->rxv |= (uint16_t)(pod << t->rxn);
                if (++t->rxn == 11)
                {
                    t->inframe = 0;
                    uint8_t d = (uint8_t)t->rxv;
                    int ones = 0;
                    for (int i = 0; i < 8; i++) ones += (d >> i) & 1;
                    int par = (t->rxv >> 8) & 1, st = (t->rxv >> 9) & 3;
                    if (d == 0 && par == 0 && st == 0) t->breaks++;
                    if (!t->enabled) { }
                    else if (par != (ones & 1) || st != 3) t->parity_bad++;
                    else rx_frame(d);
                }
            }
        }
    }
    t->prev_clk = clk;
}

static int drive(void) { return X.cur; }

static pe_counts_t s_counts;

static void init(void)
{
    xm_t *t = &X;
    memset(t, 0, sizeof *t);
    memset(t->flash, 0xFF, sizeof t->flash);
    memset(t->eeprom, 0xFF, sizeof t->eeprom);
    memset(t->user, 0xFF, sizeof t->user);
    memset(t->fuse, 0xFF, sizeof t->fuse);
    memset(t->buf, 0xFF, sizeof t->buf);
    memset(t->ebuf, 0xFF, sizeof t->ebuf);
    t->lock = 0xFF; t->cur = -1; t->prev_clk = 1; t->prev_pod = -1;
    memset(&s_counts, 0, sizeof s_counts);
}

static void do_lock(void) { X.lock = 0xFC; }

static const pe_counts_t *counts(void)
{
    s_counts.frames = X.frames_rx;
    s_counts.errors = X.parity_bad + X.bad_key + X.early_nvm;
    s_counts.timing = X.early_nvm;
    return &s_counts;
}

static void stat(void)
{
    printf("PROG_EMU pdi enables=%lu dropouts=%lu breaks=%lu frames=%lu parity_bad=%lu bad_key=%lu early_nvm=%lu nvmen=%u "
           "lock=%02x clk_low_max_us=%lu\r\n",
           (unsigned long)X.enables, (unsigned long)X.dropouts, (unsigned long)X.breaks,
           (unsigned long)X.frames_rx, (unsigned long)X.parity_bad,
           (unsigned long)X.bad_key, (unsigned long)X.early_nvm, X.nvmen, X.lock,
           (unsigned long)(X.low_max_cyc / pe_cyc_us));
}

static void rd_flash(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < FLASH_SIZE ? X.flash[a + i] : 0xFF;
}

static void rd_ee(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < EE_SIZE ? X.eeprom[a + i] : 0xFF;
}

static void rd_user(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < FPG ? X.user[a + i] : 0xFF;
}

/* fuse bytes 0-6, then the lock byte */
static void rd_cfg(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t o = a + i;
        b[i] = o < 7 ? X.fuse[o] : o == 7 ? X.lock : 0xFF;
    }
}

static const pe_mem_t k_mem[] = {
    { "flash", FLASH_SIZE, rd_flash }, { "eeprom", EE_SIZE, rd_ee },
    { "user", FPG, rd_user }, { "config", 8, rd_cfg },
};

const pe_chip_t pe_pdi = {
    .name = "pdi", .what = "AVR XMEGA PDI: 20 KB flash (128-byte pages), 1 KB EEPROM, id 1e9441",
    .uart = 0, .reset = 0, .init = init, .lock = do_lock, .stat = stat, .counts = counts,
    .mem = k_mem, .nmem = 4, .edge = edge, .drive = drive, .data = data_edge,
};
