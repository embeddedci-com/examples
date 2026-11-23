/*
 * test_progemu_host.c - the chip emulators (progemu_<proto>.c) against the pod's real protocol
 * code, on the host, no hardware: benchpod-firmware's prog_<proto>.c runs over its prog_sim pin
 * layer, and prog_sim's chip model is replaced by the emulator under test. This checks that the
 * ported chip models behave as the host tests of the pod expect, before the NUCLEO runs them.
 *
 * Needs a checkout of benchpod-firmware with the prog_flash sources (branch prog-flash or later):
 *   make test-host-progemu BENCHPOD_FW=../../benchpod-firmware
 *
 * Time is simulated: the pod's delay_us advances the cycle count the models read (pe_now), and
 * every pin change adds 5 cycles, so the waits the models check (erase, program, reset) are real
 * while the 1 us edge-to-edge rules are only informational here (printed, not failed): the pod's
 * bit-bang timing is for the bench to measure.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "prog_flash.h"
#include "prog_pic.h"
#include "prog_sim.h"
#include "progemu.h"

uint32_t pe_now;
uint32_t pe_cyc_us = 16U;
uint32_t pe_pool[PE_POOL_BYTES / 4U];


static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)

/* ---- the adapter: prog_sim's model hooks over a pe_chip_t ---- */

static const pe_chip_t *g_chip;
static prog_sim_t *g_sim;
static uint32_t g_edges;

void pe_updi_tx(const uint8_t *b, unsigned n)
{
    prog_sim_uart_reply(g_sim, b, n);
}

static void a_on_pins(prog_sim_t *s, int clk, int pod)
{
    (void)s;
    if (g_chip == NULL) return;
    pe_now += 5;
    g_edges++;
    g_chip->edge(clk, pod);
}

static int a_chip_data(prog_sim_t *s)
{
    (void)s;
    return g_chip != NULL && !g_chip->uart ? g_chip->drive() : -1;
}

static void a_on_nrst(prog_sim_t *s, bool low)
{
    (void)s;
    if (g_chip == NULL || g_chip->nrst == NULL) return;
    pe_now += 5;
    g_chip->nrst(low);
}

static void a_on_time(prog_sim_t *s, uint32_t us)
{
    (void)s;
    pe_now += us * pe_cyc_us;
}

static void a_uart_rx(prog_sim_t *s, const uint8_t *b, size_t n, uint32_t baud, bool pe, bool s2)
{
    (void)s; (void)pe; (void)s2;
    if (g_chip == NULL) return;
    for (size_t i = 0; i < n; i++)
    {
        if (baud == 300U)
        {
            if (b[i] == 0) g_chip->brk();
            continue;
        }
        if (g_chip->want_sync())
        {
            if (b[i] == 0x55) g_chip->sync();
            continue;
        }
        g_chip->rx(b[i]);
    }
}

static const prog_sim_model_t k_model = {
    .on_pins = a_on_pins, .chip_data = a_chip_data, .on_nrst = a_on_nrst, .on_time = a_on_time,
    .on_uart_rx = a_uart_rx,
};

/* ---- the pod's side ---- */

#define IMG 32768u
typedef struct {
    prog_sim_t sim;
    prog_wire_ops_t ops;
    prog_ctx_t ctx;
    uint8_t img[IMG], out[IMG];
    uint32_t out_n;
} rig_t;

static int rig_src(void *job, uint32_t off, uint8_t *buf, uint32_t n)
{
    rig_t *r = job;
    if (off + n > IMG) return -1;
    memcpy(buf, r->img + off, n);
    return 0;
}

static int rig_dst(void *job, uint32_t off, const uint8_t *buf, uint32_t n)
{
    rig_t *r = job;
    if (off + n > IMG) return -1;
    memcpy(r->out + off, buf, n);
    if (off + n > r->out_n) r->out_n = off + n;
    return 0;
}

typedef struct {
    const char *name;
    const pe_chip_t *chip;
    const prog_proto_t *proto;
    uint16_t page;
    uint32_t flags, baud;
    uint32_t size;               /* image bytes to write to the flash */
    int mask14;                  /* odd bytes keep 6 bits (14-bit words) */
    int last_ff;                 /* C2: the flash's last byte is the lock byte, keep it erased */
} tcase_t;

static uint32_t crc32(const uint8_t *b, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFU;
    for (uint32_t i = 0; i < n; i++)
    {
        c ^= b[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320U & (0U - (c & 1U)));
    }
    return ~c;
}

static uint32_t mem_crc(const pe_mem_t *m)
{
    uint8_t *b = malloc(m->size);
    m->read(0, b, m->size);
    uint32_t c = crc32(b, m->size);
    free(b);
    return c;
}

static rig_t *rig_new(const tcase_t *tc)
{
    rig_t *r = calloc(1, sizeof *r);
    pe_cyc_us = 16U;
    pe_now = 1000;
    g_chip = tc->chip;
    g_sim = &r->sim;
    g_edges = 0;
    tc->chip->init();
    prog_sim_init(&r->sim, &r->ops, &k_model, NULL);
    r->ctx.w = &r->ops;
    r->ctx.page = tc->page;
    r->ctx.flags = tc->flags;
    r->ctx.baud = tc->baud;
    r->ctx.src = rig_src;
    r->ctx.dst = rig_dst;
    r->ctx.job = r;
    return r;
}

typedef int (*step_fn)(prog_ctx_t *);

static int run(rig_t *r, step_fn f)
{
    int rc = PROG_RUNNING;
    for (int i = 0; i < 200000 && rc == PROG_RUNNING; i++) rc = f(&r->ctx);
    return rc;
}

static void fill(rig_t *r, const tcase_t *tc, uint32_t n, unsigned seed)
{
    for (uint32_t i = 0; i < n; i++)
    {
        uint32_t x = (i + seed) * 2654435761U;
        uint8_t b = (uint8_t)(x >> 24);
        r->img[i] = (tc->mask14 && (i & 1)) ? b & 0x3F : b;
    }
    if (tc->last_ff && n >= tc->size) r->img[tc->size - 1U] = 0xFF;
}

static int op(rig_t *r, step_fn f, int mem, int erase, uint32_t addr, uint32_t len)
{
    r->ctx.mem = (uint8_t)mem;
    r->ctx.erase = (uint8_t)erase;
    r->ctx.addr = addr;
    r->ctx.len = len;
    return run(r, f);
}

static void report(const char *what, const tcase_t *tc, rig_t *r)
{
    const pe_counts_t *k = tc->chip->counts ? tc->chip->counts() : NULL;
    printf("  %-6s %-22s frames=%lu errors=%lu timing=%lu edges=%lu contention=%u\n", tc->name, what,
           k ? (unsigned long)k->frames : 0UL, k ? (unsigned long)k->errors : 0UL,
           k ? (unsigned long)k->timing : 0UL, (unsigned long)g_edges, r->sim.contention);
}

static void test_flow(const tcase_t *tc)
{
    const prog_proto_t *p = tc->proto;
    rig_t *r = rig_new(tc);
    int rc = p->enter(&r->ctx);
    if (rc != PROG_OK) printf("  %s enter rc=%d\n", tc->name, rc);
    CHECK(rc == PROG_OK);
    CHECK(r->ctx.id_len >= 2);
    printf("  %-6s id=", tc->name);
    for (unsigned i = 0; i < r->ctx.id_len; i++) printf("%02x", r->ctx.id[i]);
    printf("\n");

    CHECK(op(r, p->erase, PROG_MEM_FLASH, PROG_ERASE_CHIP, 0, 0) == PROG_OK);
    fill(r, tc, tc->size, 1);
    rc = op(r, p->write, PROG_MEM_FLASH, PROG_ERASE_NONE, 0, tc->size);
    if (rc != PROG_OK) printf("  %s write rc=%d at=%lu\n", tc->name, rc, (unsigned long)r->ctx.at_addr);
    CHECK(rc == PROG_OK);
    r->out_n = 0;
    memset(r->out, 0xAA, sizeof r->out);
    CHECK(op(r, p->read, PROG_MEM_FLASH, PROG_ERASE_NONE, 0, tc->size) == PROG_OK);
    CHECK(r->out_n == tc->size && memcmp(r->out, r->img, tc->size) == 0);
    p->leave(&r->ctx, true);

    /* the emulated flash holds the image: its CRC32 is the image's, the rest still erased */
    const pe_mem_t *fm = &tc->chip->mem[0];
    CHECK(strcmp(fm->name, "flash") == 0 && fm->size >= tc->size);
    uint8_t *mb = malloc(fm->size);
    fm->read(0, mb, fm->size);
    CHECK(memcmp(mb, r->img, tc->size) == 0);
    if (fm->size == tc->size)
    {
        CHECK(mem_crc(fm) == crc32(r->img, tc->size));
    }
    free(mb);
    CHECK(r->sim.contention == 0);
    report("flow", tc, r);
    free(r);
}

static void test_locked(const tcase_t *tc)
{
    const prog_proto_t *p = tc->proto;
    rig_t *r = rig_new(tc);
    tc->chip->lock();
    int rc = p->enter(&r->ctx);
    if (rc == PROG_OK)
    {
        CHECK(op(r, p->read, PROG_MEM_FLASH, PROG_ERASE_NONE, 0, 64) == PROG_E_LOCKED);
    }
    else
    {
        CHECK(rc == PROG_E_LOCKED);
    }
    p->leave(&r->ctx, true);
    free(r);

    /* a chip erase unlocks it: with the erase asked for, enter (UPDI) or erase does it */
    r = rig_new(tc);
    tc->chip->lock();
    r->ctx.erase = PROG_ERASE_CHIP;
    rc = p->enter(&r->ctx);
    CHECK(rc == PROG_OK);
    if (rc == PROG_OK)
    {
        CHECK(op(r, p->erase, PROG_MEM_FLASH, PROG_ERASE_CHIP, 0, 0) == PROG_OK);
        fill(r, tc, 256, 2);
        CHECK(op(r, p->write, PROG_MEM_FLASH, PROG_ERASE_NONE, 0, 256) == PROG_OK);
    }
    p->leave(&r->ctx, true);
    report("locked, then erased", tc, r);
    free(r);
}

static void test_stopped(const tcase_t *tc)
{
    rig_t *r = rig_new(tc);
    g_chip = NULL;                    /* the emulator is stopped: nothing answers */
    int rc = tc->proto->enter(&r->ctx);
    CHECK(rc == PROG_E_NO_ANSWER || rc == PROG_E_PROTOCOL);
    tc->proto->leave(&r->ctx, false);
    printf("  %-6s stopped emulator: rc=%d\n", tc->name, rc);
    free(r);
}

int main(void)
{
    const tcase_t cases[] = {
        { "pic",   &pe_pic,   &prog_proto_pic, 16, PIC_F_CP_BIT(5) | PIC_F_HAS_EE, 0x1000E, 8192, 1, 0 },
        { "pic8",  &pe_pic8,  &prog_proto_pic, 32, PIC_F_SET8 | PIC_F_CP_BIT(1), 0x10016, 8192, 1, 0 },
        { "pic18", &pe_pic18, &prog_proto_pic, 32, PIC_F_SET8 | PIC_F_BYTEPC | PIC_F_CP_BIT(3), 0x300004, 8192, 0, 0 },
#ifdef PE_HAVE_c2
        { "c2",    &pe_c2,    &prog_proto_c2, 512, 0, 0, 8192, 0, 1 },
#endif
#ifdef PE_HAVE_tpi
        { "tpi",   &pe_tpi,   &prog_proto_tpi, 16, 0, 0, 1024, 0, 0 },
#endif
#ifdef PE_HAVE_pdi
        { "pdi",   &pe_pdi,   &prog_proto_pdi, 128, 0, 0, 20480, 0, 0 },
#endif
#ifdef PE_HAVE_updi
        { "updi",  &pe_updi,  &prog_proto_updi, 64, 0, 0, 16384, 0, 0 },
        { "updi2", &pe_updi2, &prog_proto_updi, 512, 0, 0, 16384, 0, 0 },
#endif
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; i++)
    {
        test_flow(&cases[i]);
        test_locked(&cases[i]);
        test_stopped(&cases[i]);
    }
    printf("test_progemu_host: %s (%d checks)\n", failures ? "FAILED" : "ok", checks);
    return failures ? 1 : 0;
}
