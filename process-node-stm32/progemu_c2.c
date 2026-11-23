/*
 * progemu_c2.c - Silicon Labs C2, the model of test_prog_c2.c in benchpod-firmware (stm32h563/test),
 * ported to this board: the chip follows the C2 frames cycle by cycle, latches host bits on the
 * rising clock edge and drives its own bits after the rising edge too (AN127 1.4: a host reads at
 * least 120 ns after it). Registers DEVICEID, REVID,
 * FPCTL and FPDAT; the FPI (device erase with its confirm bytes, page erase, block read and write,
 * derivative); a flash with AND-only programming and a lock byte (the last flash byte).
 *
 * C2CK is also the chip's reset: a clock low of 20 us or more resets the chip (the host's timing
 * is the cycle counter at the edges). Device 0x32, revision 0x03, derivative 0x21 (id 320321),
 * 8 KB flash with 512-byte pages, lock byte at 0x1FFF.
 */
#include <stdio.h>
#include <string.h>

#include "progemu.h"

#define FLASH_SZ   0x2000u
#define LOCK_ADDR  0x1FFFu
#define PAGE       512u
#define AR_FPCTL   0x02u
#define FPDAT_ADDR 0xB4u
#define DEVID 0x32u
#define REVID 0x03u
#define DERIV 0x21u

typedef enum { S_START, S_INS, S_ARW, S_ARR, S_LEN, S_DW, S_WAIT, S_DR, S_STOP } c2st_t;
typedef enum { F_IDLE, F_CONF, F_PAGE, F_PAGE0, F_PARAMS_R, F_PARAMS_W, F_DATA_W, F_DIRECT } fpist_t;

typedef struct {
    uint8_t  flash[FLASH_SZ];
    uint8_t  locked;
    /* counters */
    uint8_t  sfr_log[16][2];     /* writes to other C2 addresses, and direct FPI writes */
    unsigned nsfr, unprepared;   /* ... and erase/write commands that came before the required setup */
    uint32_t resets, early_fpdat, erase_dev, erase_page, bad_confirm, fpctl_enable, transfers;
    uint32_t max_low_us;         /* longest clock low that was not a reset */
    uint32_t long_low;           /* a low of 20 us or more that reset the chip (also the entry) */
    /* C2 state */
    int      prev_clk;
    uint32_t fall_t;
    c2st_t   st;
    unsigned nb;
    uint32_t acc;
    unsigned ins;
    uint8_t  ar, outv, is_read;
    int      drive, last_drive;
    unsigned wait_left;
    unsigned fpctl_stage;
    uint8_t  enabled;
    uint32_t enable_until, busy_until;
    uint8_t  enable_wait, busy;
    /* FPI */
    fpist_t  fs;
    unsigned nconf, np;
    uint8_t  conf_bad, pbuf[3];
    uint32_t waddr, wremain;
    uint8_t  last_page;
    uint8_t  out[300];
    unsigned nout, oh;
} c2m_t;

_Static_assert(sizeof(c2m_t) <= PE_POOL_BYTES, "c2 state");
#define M (*(c2m_t *)(void *)pe_pool)

#define WAIT_CYCLES 2u

/* The device setup AN127 Table 3.6 asks for before erase and write, for this device (0x32: BB2/UB1) */
static const uint8_t k_need[3][2] = { { 0xFF, 0x80 }, { 0xEF, 0x02 }, { 0xA9, 0x00 } };

static void sfr_note(uint8_t a, uint8_t v)
{
    if (M.nsfr < 16) { M.sfr_log[M.nsfr][0] = a; M.sfr_log[M.nsfr][1] = v; M.nsfr++; }
}

static int prepared(void)
{
    for (unsigned i = 0; i < 3; i++)
    {
        int seen = 0;
        for (unsigned j = 0; j < M.nsfr; j++)
        {
            if (M.sfr_log[j][0] == k_need[i][0] && M.sfr_log[j][1] == k_need[i][1]) seen = 1;
        }
        if (!seen) return 0;
    }
    return 1;
}

static int is_busy(void)
{
    return M.busy && (int32_t)(M.busy_until - pe_now) > 0;
}

static void set_busy(uint32_t us)
{
    M.busy = 1;
    M.busy_until = pe_now + us * pe_cyc_us;
}

static void out_push(uint8_t b) { M.out[M.nout++] = b; }

static void fpi_byte(uint8_t b)
{
    c2m_t *m = &M;
    switch (m->fs)
    {
    case F_IDLE:
        m->nout = m->oh = 0;
        switch (b)
        {
        case 0x02: out_push(0x0D); out_push(DERIV); break;
        case 0x03: if (!prepared()) m->unprepared++;
            out_push(0x0D); m->fs = F_CONF; m->nconf = 0; m->conf_bad = 0; break;
        case 0x06: out_push(m->locked ? 0x80 : 0x0D); if (!m->locked) { m->fs = F_PARAMS_R; m->np = 0; } break;
        case 0x07: if (!prepared()) m->unprepared++;
            out_push(m->locked ? 0x80 : 0x0D); if (!m->locked) { m->fs = F_PARAMS_W; m->np = 0; } break;
        case 0x08: if (!prepared()) m->unprepared++;
            out_push(m->locked ? 0x80 : 0x0D); if (!m->locked) m->fs = F_PAGE; break;
        case 0x0A: out_push(0x0D); m->fs = F_DIRECT; m->np = 0; break;
        default: out_push(0x01); break;
        }
        break;
    case F_CONF:
    {
        static const uint8_t conf[3] = { 0xDE, 0xAD, 0xA5 };
        if (b != conf[m->nconf]) m->conf_bad = 1;
        if (++m->nconf == 3)
        {
            m->fs = F_IDLE;
            if (m->conf_bad)
            {
                m->bad_confirm++;
                out_push(0x82);
            }
            else
            {
                m->erase_dev++;
                memset(m->flash, 0xFF, sizeof m->flash);
                m->locked = 0;
                set_busy(30000);
                out_push(0x0D);
            }
        }
        break;
    }
    case F_PAGE:                  /* the page number: accepted, the erase waits for the 0x00 byte */
        m->last_page = b;
        m->fs = F_PAGE0;
        out_push(0x0D);
        break;
    case F_PAGE0:                 /* 0x00 starts the erase; the third status comes when it is done */
        m->fs = F_IDLE;
        m->erase_page++;
        if ((m->last_page + 1u) * PAGE <= FLASH_SZ) memset(&m->flash[m->last_page * PAGE], 0xFF, PAGE);
        set_busy(40000);
        out_push(0x0D);
        break;
    case F_DIRECT:                /* address, length 1, data */
        m->pbuf[m->np++] = b;
        if (m->np == 3) { sfr_note(m->pbuf[0], m->pbuf[2]); m->fs = F_IDLE; }
        break;
    case F_PARAMS_R:
    case F_PARAMS_W:
        m->pbuf[m->np++] = b;
        if (m->np == 3)
        {
            uint32_t a = ((uint32_t)m->pbuf[0] << 8) | m->pbuf[1];
            uint32_t n = m->pbuf[2] ? m->pbuf[2] : 256u;
            m->nout = m->oh = 0;
            out_push(0x0D);
            if (m->fs == F_PARAMS_R)
            {
                for (uint32_t i = 0; i < n; i++)
                {
                    out_push(a + i < FLASH_SZ ? m->flash[a + i] : 0xFF);
                }
                m->fs = F_IDLE;
            }
            else
            {
                m->waddr = a;
                m->wremain = n;
                m->fs = F_DATA_W;
            }
        }
        break;
    case F_DATA_W:
        if (m->waddr < FLASH_SZ) m->flash[m->waddr] &= b;      /* programming only clears bits */
        if (m->waddr == LOCK_ADDR && m->flash[LOCK_ADDR] != 0xFF) m->locked = 1;
        m->waddr++;
        set_busy(30);
        if (--m->wremain == 0)
        {
            out_push(0x0D);
            m->fs = F_IDLE;
        }
        break;
    }
}

static int out_ready(void) { return !is_busy() && M.oh < M.nout; }

static void dr_write_done(uint8_t v)
{
    c2m_t *m = &M;
    if (m->ar == AR_FPCTL)
    {
        if (v == 0x02) m->fpctl_stage = 1;
        else if (v == 0x04 && m->fpctl_stage == 1) m->fpctl_stage = 2;
        else if (v == 0x01 && m->fpctl_stage >= 1)
        {
            m->enabled = 1;
            m->fpctl_enable++;
            m->enable_wait = 1;
            m->enable_until = pe_now + 20000U * pe_cyc_us;
            m->fpctl_stage = 0;
        }
        else m->fpctl_stage = 0;
    }
    else if (m->ar == FPDAT_ADDR)
    {
        if (m->enable_wait && (int32_t)(m->enable_until - pe_now) <= 0) m->enable_wait = 0;
        if (!m->enabled || m->enable_wait || is_busy())
        {
            m->early_fpdat++;
            return;
        }
        fpi_byte(v);
    }
    else if (m->ar != 0 && m->ar != 1)
    {
        sfr_note(m->ar, v);
    }
}

static void chip_reset(void)
{
    c2m_t *m = &M;
    m->resets++;
    m->st = S_START;
    m->nb = 0;
    m->ar = 0;
    m->drive = -1;
    m->enabled = 0;
    m->fpctl_stage = 0;
    m->fs = F_IDLE;
    m->nout = m->oh = 0;
    m->busy = 0;
    m->enable_wait = 0;
    m->nsfr = 0;
}

/* The chip's drive for the cycle this rising edge starts: valid from the edge until the next rising
   edge (the falling edge changes nothing), as AN127 1.4 has it: a host reads after the rising edge. */
static int drive_peek(int consume)
{
    c2m_t *m = &M;
    switch (m->st)
    {
    case S_ARR: case S_DR: return (m->outv >> m->nb) & 1u;
    case S_WAIT:
        if (m->wait_left > 0) { if (consume) m->wait_left--; return 0; }
        if (m->is_read && m->ar == FPDAT_ADDR && !out_ready()) return 0;
        return 1;
    default: return -1;
    }
}

static int drive_for(void) { return drive_peek(1); }

static void rising(int bit)
{
    c2m_t *m = &M;
    int d = drive_for();
    m->drive = d;
    switch (m->st)
    {
    case S_START: m->st = S_INS; m->nb = 0; m->acc = 0; break;
    case S_INS:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 2)
        {
            m->ins = m->acc; m->nb = 0; m->acc = 0;
            if (m->ins == 3) m->st = S_ARW;
            else if (m->ins == 2)
            {
                m->st = S_ARR;
                m->outv = (uint8_t)((out_ready() ? 1u : 0u) | (is_busy() ? 2u : 0u));
            }
            else
            {
                m->is_read = (m->ins == 0);
                m->st = S_LEN;
            }
        }
        break;
    case S_ARW:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 8) { m->ar = (uint8_t)m->acc; m->st = S_STOP; }
        break;
    case S_ARR: if (++m->nb == 8) m->st = S_STOP; break;
    case S_LEN:
        if (++m->nb == 2)
        {
            m->nb = 0; m->acc = 0;
            if (m->is_read) { m->st = S_WAIT; m->wait_left = WAIT_CYCLES; }
            else m->st = S_DW;
        }
        break;
    case S_DW:
        m->acc |= (uint32_t)bit << m->nb;
        if (++m->nb == 8)
        {
            dr_write_done((uint8_t)m->acc);
            m->st = S_WAIT;
            m->wait_left = WAIT_CYCLES;
            m->nb = 0;
        }
        break;
    case S_WAIT:
        if (d == 1)
        {
            m->nb = 0;
            if (m->is_read)
            {
                if (m->ar == 0) m->outv = DEVID;
                else if (m->ar == 1) m->outv = REVID;
                else if (m->ar == FPDAT_ADDR && m->oh < m->nout) m->outv = m->out[m->oh++];
                else m->outv = 0xFF;
                m->st = S_DR;
            }
            else m->st = S_STOP;
        }
        break;
    case S_DR: if (++m->nb == 8) m->st = S_STOP; break;
    case S_STOP: m->st = S_START; m->transfers++; break;
    }
}

static void edge(int clk, int pod)
{
    c2m_t *m = &M;
    if (clk == m->prev_clk) return;
    m->prev_clk = clk;
    if (clk == 0)
    {
        m->fall_t = pe_now;
    }
    else
    {
        uint32_t low_us = (pe_now - m->fall_t) / pe_cyc_us;
        if (low_us >= 20)
        {
            m->long_low++;
            chip_reset();
        }
        else
        {
            if (low_us > m->max_low_us) m->max_low_us = low_us;
            rising(pod < 0 ? 1 : pod);
        }
    }
}

static int drive(void) { return M.drive; }

/* c2early: the value for the cycle the next rising edge starts is put on the wire during the
   preceding low phase, so a slow interrupt still has it there when the pod samples (the pod reads
   at least 120 ns after the rising edge). A pod that wrongly samples while the clock is low reads
   the new bit instead of the old one and goes unnoticed; use `c2` to catch that. */
static int drive_early(void)
{
    if (M.prev_clk == 0 && M.st != S_START) return drive_peek(0);
    return M.drive;
}

static pe_counts_t s_counts;

static void init(void)
{
    c2m_t *m = &M;
    memset(m, 0, sizeof *m);
    memset(m->flash, 0xFF, sizeof m->flash);
    m->prev_clk = 1;
    m->drive = -1;
    memset(&s_counts, 0, sizeof s_counts);
}

static void do_lock(void)
{
    M.flash[LOCK_ADDR] &= 0xFEu;
    M.locked = 1;
}

static const pe_counts_t *counts(void)
{
    s_counts.frames = M.transfers;
    s_counts.errors = M.early_fpdat + M.bad_confirm + M.unprepared;
    s_counts.timing = M.early_fpdat;
    return &s_counts;
}

static void stat(void)
{
    printf("PROG_EMU c2 unprepared=%lu resets=%lu early_fpdat=%lu bad_confirm=%lu fpctl_enable=%lu erase_device=%lu "
           "erase_page=%lu max_low_us=%lu locked=%u\r\n",
           (unsigned long)M.unprepared, (unsigned long)M.resets, (unsigned long)M.early_fpdat, (unsigned long)M.bad_confirm,
           (unsigned long)M.fpctl_enable, (unsigned long)M.erase_dev, (unsigned long)M.erase_page,
           (unsigned long)M.max_low_us, M.locked);
}

static void rd_flash(uint32_t a, uint8_t *b, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++) b[i] = a + i < FLASH_SZ ? M.flash[a + i] : 0xFF;
}

static const pe_mem_t k_mem[] = { { "flash", FLASH_SZ, rd_flash } };

const pe_chip_t pe_c2early = {
    .name = "c2early", .what = "Silicon Labs C2 as `c2`, data driven early (a slow interrupt still meets the pod's read), id 320321",
    .uart = 0, .reset = 0, .init = init, .lock = do_lock, .stat = stat, .counts = counts,
    .mem = k_mem, .nmem = 1, .edge = edge, .drive = drive_early,
};

const pe_chip_t pe_c2 = {
    .name = "c2", .what = "Silicon Labs C2, 8 KB flash, 512-byte pages, id 320321 (device 32 rev 03 derivative 21)",
    .uart = 0, .reset = 0, .init = init, .lock = do_lock, .stat = stat, .counts = counts,
    .mem = k_mem, .nmem = 1, .edge = edge, .drive = drive,
};
