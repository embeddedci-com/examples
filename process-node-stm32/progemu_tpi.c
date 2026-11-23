/*
 * progemu_tpi.c - AVR TPI (an ATtiny10), the model of test_prog_tpi.c in benchpod-firmware
 * (stm32h563/test), ported to this board. The chip is active while its RESET pin (PB0) is low:
 * it samples data on the rising TPICLK edge, changes its output after the falling edge, takes the
 * 8-byte NVM key, and has the NVM controller (chip erase, section erase, word write) with its
 * busy times. 12-bit frames: start, 8 data bits LSB first, even parity, 2 stop bits.
 *
 * ATtiny10: 1 KB flash at 0x4000 (16-byte sections), configuration byte at 0x3F40, lock bits at
 * 0x3F00, signature 1e 90 03 at 0x3FC0 (id 1e9003).
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define FLASH_BASE 0x4000u
#define FLASH_SIZE 1024u
#define PG 16u

typedef struct {
    uint8_t flash[FLASH_SIZE], config, lock, sig[3];
    uint8_t cs_tpisr, cs_pcr, nvmcmd;
    uint16_t ptr;
    uint8_t nvmen, nrst_low, busy;
    uint32_t busy_until;
    int prev_clk;
    /* receive side */
    int rxn;
    uint16_t rxv;
    uint8_t inframe;
    uint8_t op;
    int need;
    uint8_t args[8];
    int nargs;
    /* transmit side */
    uint8_t out[32];
    int outn, outi, cur;
    /* counters */
    uint32_t frames_rx, parity_bad, bad_key, resets, early_nvm;
} tiny_t;

_Static_assert(sizeof(tiny_t) <= PE_POOL_BYTES, "tpi state");
#define T (*(tiny_t *)(void *)pe_pool)

static int locked(void) { return (T.lock & 3u) != 3u; }

static int is_busy(void)
{
    return T.busy && (int32_t)(T.busy_until - pe_now) > 0;
}

static void set_busy(uint32_t us)
{
    T.busy = 1;
    T.busy_until = pe_now + us * pe_cyc_us;
}

static uint8_t mem_read(uint16_t a)
{
    tiny_t *t = &T;
    if (!t->nvmen) return 0;
    if (a == 0x32) return is_busy() ? 0x80 : 0;
    if (a == 0x33) return t->nvmcmd;
    if (a == 0x3F00) return t->lock;
    if (a >= 0x3FC0 && a < 0x3FC3) return t->sig[a - 0x3FC0];
    if (a == 0x3F40) return t->config;
    if (a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE) return locked() ? 0 : t->flash[a - FLASH_BASE];
    return 0;
}

static void mem_write(uint16_t a, uint8_t v)
{
    tiny_t *t = &T;
    if (!t->nvmen) return;
    if (a == 0x33) { t->nvmcmd = v; return; }
    int fl = a >= FLASH_BASE && a < FLASH_BASE + FLASH_SIZE;
    int cf = a == 0x3F40 || a == 0x3F41;
    if (!fl && !cf) return;
    if (t->nvmcmd != 0 && is_busy()) t->early_nvm++;
    if (t->nvmcmd == 0x10 && fl && (a & 1u))                 /* chip erase: dummy write to a high byte */
    {
        memset(t->flash, 0xFF, sizeof t->flash);
        t->config = 0xFF;
        t->lock = 0xFF;
        set_busy(15000);
    }
    else if (t->nvmcmd == 0x14)                              /* section erase */
    {
        if (fl)
        {
            uint16_t b = (uint16_t)((a - FLASH_BASE) / PG * PG);
            memset(&t->flash[b], 0xFF, PG);
        }
        else t->config = 0xFF;
        set_busy(4000);
    }
    else if (t->nvmcmd == 0x1D)                              /* word write: ANDs into the erased cell */
    {
        if (locked()) return;
        if (fl) t->flash[a - FLASH_BASE] &= v;
        else if (a == 0x3F40) t->config &= v;
        if (a & 1u) set_busy(4000);
    }
}

static void rx_byte(uint8_t b)
{
    tiny_t *t = &T;
    t->frames_rx++;
    if (t->need > 0)
    {
        t->args[t->nargs++] = b;
        if (--t->need) return;
        switch (t->op & 0xF0)
        {
        case 0x60:
            if ((t->op & 0xFC) == 0x68)
            {
                if (t->op & 1) t->ptr = (uint16_t)((t->ptr & 0xFF) | b << 8);
                else t->ptr = (uint16_t)((t->ptr & 0xFF00) | b);
            }
            else
            {
                mem_write(t->ptr, b);
                if (t->op & 4) t->ptr++;
            }
            break;
        case 0xC0:
            if ((t->op & 0xF) == 2) t->cs_pcr = b;
            else if ((t->op & 0xF) == 0)
            {
                t->cs_tpisr = b;
                if (!(b & 2)) t->nvmen = 0;
            }
            break;
        case 0xE0:
        {
            static const uint8_t key[8] = { 0xFF, 0x88, 0xD8, 0xCD, 0x45, 0xAB, 0x89, 0x12 };
            if (memcmp(t->args, key, 8) == 0) { t->nvmen = 1; t->cs_tpisr |= 2; }
            else t->bad_key++;
            break;
        }
        }
        return;
    }
    t->op = b;
    t->nargs = 0;
    uint8_t reply = 0;
    int answer = 0;
    if ((b & 0xFB) == 0x20) { reply = mem_read(t->ptr); if (b & 4) t->ptr++; answer = 1; }
    else if ((b & 0xFB) == 0x60) t->need = 1;
    else if ((b & 0xFE) == 0x68) t->need = 1;
    else if ((b & 0xF0) == 0x80) { reply = ((b & 0xF) == 0) ? t->cs_tpisr : (b & 0xF) == 2 ? t->cs_pcr : 0; answer = 1; }
    else if ((b & 0xF0) == 0xC0) t->need = 1;
    else if (b == 0xE0) t->need = 8;
    if (answer)
    {
        int n = 0;
        for (int g = 0; g < 2; g++) t->out[n++] = 1;           /* guard time 2 */
        t->out[n++] = 0;
        int ones = 0;
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
}

static void edge(int clk, int pod)
{
    tiny_t *t = &T;
    if (!t->nrst_low) { t->prev_clk = clk; return; }
    if (t->prev_clk == 1 && clk == 0)                           /* falling edge: the chip changes its output */
    {
        t->cur = t->outi < t->outn ? t->out[t->outi++] : -1;
    }
    else if (t->prev_clk == 0 && clk == 1 && pod >= 0)          /* rising edge: the chip samples */
    {
        if (!t->inframe)
        {
            if (pod == 0) { t->inframe = 1; t->rxn = 0; t->rxv = 0; }
        }
        else
        {
            t->rxv |= (uint16_t)(pod << t->rxn);
            if (++t->rxn == 11)                                 /* 8 data + parity + 2 stop */
            {
                t->inframe = 0;
                uint8_t d = (uint8_t)t->rxv;
                int ones = 0;
                for (int i = 0; i < 8; i++) ones += (d >> i) & 1;
                int par = (t->rxv >> 8) & 1, st = (t->rxv >> 9) & 3;
                if (par != (ones & 1) || st != 3) t->parity_bad++; else rx_byte(d);
            }
        }
    }
    t->prev_clk = clk;
}

static int drive(void) { return T.nrst_low ? T.cur : -1; }

static void on_nrst(int low)
{
    tiny_t *t = &T;
    t->nrst_low = (uint8_t)low;
    if (low)
    {
        t->resets++;
        t->nvmen = 0; t->inframe = 0; t->need = 0; t->outn = t->outi = 0; t->cur = -1; t->cs_tpisr = 0;
    }
}

static pe_counts_t s_counts;

static void init(void)
{
    tiny_t *t = &T;
    memset(t, 0, sizeof *t);
    memset(t->flash, 0xFF, sizeof t->flash);
    t->config = 0xFF; t->lock = 0xFF;
    t->sig[0] = 0x1E; t->sig[1] = 0x90; t->sig[2] = 0x03;
    t->cur = -1;
    t->prev_clk = 1;
    memset(&s_counts, 0, sizeof s_counts);
}

static void do_lock(void) { T.lock = 0xFC; }

static const pe_counts_t *counts(void)
{
    s_counts.frames = T.frames_rx;
    s_counts.errors = T.parity_bad + T.bad_key + T.early_nvm;
    s_counts.timing = T.early_nvm;
    return &s_counts;
}

static void stat(void)
{
    printf("PROG_EMU tpi resets=%lu frames=%lu parity_bad=%lu bad_key=%lu early_nvm=%lu nvmen=%u "
           "lock=%02x config=%02x\r\n",
           (unsigned long)T.resets, (unsigned long)T.frames_rx, (unsigned long)T.parity_bad,
           (unsigned long)T.bad_key, (unsigned long)T.early_nvm, T.nvmen, T.lock, T.config);
}

static void rd_flash(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < FLASH_SIZE ? T.flash[a + i] : 0xFF;
}

/* config byte, then the lock bits */
static void rd_cfg(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t o = a + i;
        b[i] = o == 0 ? T.config : o == 1 ? T.lock : 0xFF;
    }
}

static const pe_mem_t k_mem[] = { { "flash", FLASH_SIZE, rd_flash }, { "config", 2, rd_cfg } };

const pe_chip_t pe_tpi = {
    .name = "tpi", .what = "AVR TPI, ATtiny10: 1 KB flash, 16-byte sections, id 1e9003",
    .uart = 0, .reset = 1, .init = init, .lock = do_lock, .stat = stat, .counts = counts,
    .mem = k_mem, .nmem = 2, .edge = edge, .drive = drive, .nrst = on_nrst,
};
