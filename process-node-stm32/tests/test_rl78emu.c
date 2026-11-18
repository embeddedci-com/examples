/* Host unit test for rl78emu.c, the RL78 Protocol A boot ROM emulator: frames, checksums and
 * every command flow, byte for byte against R01AN0815 Rev. 1.10. Run: make test-host */
#include "rl78emu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

/* ---- the port: frames out are queued for the test to read ---- */
static uint8_t out[4096];
static size_t nout, rpos;
static uint32_t last_baud;
static void port_send(void *ctx, const uint8_t *b, size_t n) { (void)ctx; memcpy(out + nout, b, n); nout += n; }
static void port_baud(void *ctx, uint32_t baud) { (void)ctx; last_baud = baud; }

static rl78emu_t E;   /* 68 KB of flash: static */

static void fresh(void)
{
    const rl78emu_port_t p = {port_send, port_baud, NULL};
    rl78emu_init(&E, &p);
    nout = rpos = 0;
    last_baud = 0;
}

static void feed(const uint8_t *b, size_t n) { for (size_t i = 0; i < n; i++) rl78emu_rx(&E, b[i]); }

/* SOH LEN COM info SUM ETX */
static void cmd(uint8_t com, const uint8_t *info, size_t n)
{
    uint8_t f[300];
    f[0] = RL78_SOH; f[1] = (uint8_t)(n + 1); f[2] = com;
    memcpy(&f[3], info, n);
    f[3 + n] = rl78_sum(&f[1], n + 2);
    f[4 + n] = RL78_ETX;
    feed(f, n + 5);
}

static void data(const uint8_t *d, size_t n, int last)
{
    uint8_t f[300];
    f[0] = RL78_STX; f[1] = (uint8_t)n;
    memcpy(&f[2], d, n);
    f[2 + n] = rl78_sum(&f[1], n + 1);
    f[3 + n] = last ? RL78_ETX : RL78_ETB;
    feed(f, n + 4);
}

/* Next frame from the emulator: its data in d, returns its length (-1 none / malformed). */
static int next(uint8_t *d)
{
    if (rpos + 4 > nout || out[rpos] != RL78_STX) return -1;
    size_t n = out[rpos + 1] ? out[rpos + 1] : 256;
    if (rpos + n + 4 > nout) return -1;
    memcpy(d, &out[rpos + 2], n);
    uint8_t sum = out[rpos + 2 + n], ftr = out[rpos + 3 + n];
    if (sum != rl78_sum(&out[rpos + 1], n + 1) || ftr != RL78_ETX) return -1;
    rpos += n + 4;
    return (int)n;
}

static int st1(void) { uint8_t d[256]; return next(d) == 1 ? d[0] : -1; }
static int st2(int *b) { uint8_t d[256]; if (next(d) != 2) return -1; *b = d[1]; return d[0]; }
static int pending(void) { return (int)(nout - rpos); }

static void addr(uint8_t *p, uint32_t a) { p[0] = (uint8_t)a; p[1] = (uint8_t)(a >> 8); p[2] = (uint8_t)(a >> 16); }
static void range(uint8_t *p, uint32_t sa, uint32_t ea) { addr(p, sa); addr(p + 3, ea); }

static void synced(void)
{
    fresh();
    const uint8_t mode = 0x3A;      /* single-wire mode byte: no reply */
    feed(&mode, 1);
    const uint8_t b[2] = {0x00, 33};
    cmd(RL78_CMD_BAUD, b, 2);
    uint8_t d[256];
    (void)next(d);
    cmd(RL78_CMD_RESET, NULL, 0);
    (void)st1();
}

/* Programming of [sa, ea] with pattern(i); returns the final internal-verify status. */
static uint8_t pat(uint32_t a, int salt) { return (uint8_t)(a * 7u + (uint32_t)salt); }
static int program(uint32_t sa, uint32_t ea, int salt, int *frame_err)
{
    uint8_t r[6]; range(r, sa, ea);
    cmd(RL78_CMD_PROGRAM, r, 6);
    int s = st1();
    if (s != RL78_ST_ACK) return s;
    *frame_err = 0;
    for (uint32_t a = sa; a <= ea; a += 256) {
        uint8_t d[256];
        for (int i = 0; i < 256; i++) d[i] = pat(a + (uint32_t)i, salt);
        data(d, 256, a + 256 > ea);
        int b = 0, a1 = st2(&b);
        if (a1 != RL78_ST_ACK || b != RL78_ST_ACK) { *frame_err = b; return -2; }
    }
    return st1();
}

static void test_checksum_examples(void)
{
    /* the PDF's worked examples: 01 01 A1 5E 03 and 02 04 FF 80 40 22 1B 03 */
    const uint8_t c[] = {0x01, 0xA1}, d[] = {0x04, 0xFF, 0x80, 0x40, 0x22};
    CHECK(rl78_sum(c, 2) == 0x5E, "command example sum %02x", rl78_sum(c, 2));
    CHECK(rl78_sum(d, 5) == 0x1B, "data example sum %02x", rl78_sum(d, 5));
}

static void test_baud_first(void)
{
    fresh();
    const uint8_t mode = 0x3A;
    feed(&mode, 1);
    CHECK(pending() == 0, "answered the mode byte");
    cmd(RL78_CMD_SIGNATURE, NULL, 0);
    CHECK(st1() == RL78_ST_CMD_ERR, "a command before Baud Rate Set is not 04H");
    const uint8_t bad1[2] = {0x04, 33}, bad2[2] = {0x00, 17};
    cmd(RL78_CMD_BAUD, bad1, 2);
    CHECK(st1() == RL78_ST_PARAM_ERR, "baud code 4 not refused");
    cmd(RL78_CMD_BAUD, bad2, 2);
    CHECK(st1() == RL78_ST_PARAM_ERR, "1.7 V not refused");
    CHECK(last_baud == 0, "a refused Baud Rate Set changed the rate");
    const uint8_t ok[2] = {0x03, 33};
    cmd(RL78_CMD_BAUD, ok, 2);
    /* the exact reply: 02 03 06 20 00 SUM 03, then the new rate */
    const uint8_t want[] = {0x02, 0x03, 0x06, 0x20, 0x00, (uint8_t)(0 - 3 - 6 - 0x20), 0x03};
    CHECK(nout - rpos == sizeof(want) && memcmp(out + rpos, want, sizeof(want)) == 0, "baud reply bytes");
    rpos = nout;
    CHECK(last_baud == 1000000u, "rate %u", (unsigned)last_baud);
    cmd(RL78_CMD_RESET, NULL, 0);
    CHECK(st1() == RL78_ST_ACK, "reset not acked");
}

static void test_signature(void)
{
    synced();
    cmd(RL78_CMD_SIGNATURE, NULL, 0);
    CHECK(st1() == RL78_ST_ACK, "signature status");
    uint8_t d[256];
    const uint8_t want[22] = {0x10, 0x00, 0x06, 'R', '5', 'F', '1', '0', '0', 'L', 'E', ' ', ' ',
                              0xFF, 0xFF, 0x00, 0xFF, 0x1F, 0x0F, 0x01, 0x02, 0x03};
    CHECK(next(d) == 22 && memcmp(d, want, 22) == 0, "signature data");
}

static void test_erase_blank_program_verify(void)
{
    synced();
    uint8_t r[7], a[3];
    int fe;
    /* fresh part: blank */
    range(r, 0x000000, 0x00FFFF); r[6] = 0;
    cmd(RL78_CMD_BLANK, r, 7);
    CHECK(st1() == RL78_ST_ACK, "fresh code flash not blank");
    /* program two blocks: 8 frames of 256, double ACK each, then the internal verify ACK */
    CHECK(program(0x000400, 0x000BFF, 1, &fe) == RL78_ST_ACK, "program");
    CHECK(*rl78emu_flash(&E, 0x000400) == pat(0x400, 1) && *rl78emu_flash(&E, 0x000BFF) == pat(0xBFF, 1) &&
          *rl78emu_flash(&E, 0x0003FF) == 0xFF && *rl78emu_flash(&E, 0x000C00) == 0xFF, "flash contents");
    range(r, 0x000400, 0x0007FF); r[6] = 0;
    cmd(RL78_CMD_BLANK, r, 7);
    CHECK(st1() == RL78_ST_BLANK_ERR, "programmed block reads blank");
    range(r, 0x000C00, 0x000FFF);
    cmd(RL78_CMD_BLANK, r, 7);
    CHECK(st1() == RL78_ST_ACK, "untouched block not blank");
    /* writing over programmed bytes without an erase fails (1CH) */
    CHECK(program(0x000400, 0x0007FF, 2, &fe) == -2 && fe == RL78_ST_WRITE_ERR, "overwrite allowed (%d)", fe);
    /* verify: good, then one byte off (reported in the LAST frame's ST2 only) */
    for (int bad = 0; bad < 2; bad++) {
        range(r, 0x000400, 0x000BFF);
        cmd(RL78_CMD_VERIFY, r, 6);
        CHECK(st1() == RL78_ST_ACK, "verify status");
        for (uint32_t x = 0x400; x <= 0xBFF; x += 256) {
            uint8_t d[256];
            for (int i = 0; i < 256; i++) d[i] = pat(x + (uint32_t)i, 1);
            if (bad && x == 0x400) d[5] ^= 1;
            int last = x + 256 > 0xBFF, b = 0;
            data(d, 256, last);
            CHECK(st2(&b) == RL78_ST_ACK, "verify ST1");
            int want = (bad && last) ? RL78_ST_VERIFY_ERR : RL78_ST_ACK;
            CHECK(b == want, "verify ST2 %02x want %02x at %x", b, want, (unsigned)x);
        }
        CHECK(pending() == 0, "verify sent a final status");
    }
    /* checksum: 0000H minus every byte, 16 bits, low byte first */
    range(r, 0x000400, 0x0007FF);
    cmd(RL78_CMD_CHECKSUM, r, 6);
    uint16_t ck = 0;
    for (uint32_t x = 0x400; x <= 0x7FF; x++) ck = (uint16_t)(ck - pat(x, 1));
    uint8_t d[256];
    CHECK(st1() == RL78_ST_ACK && next(d) == 2 && d[0] == (uint8_t)ck && d[1] == (uint8_t)(ck >> 8), "checksum");
    /* erase one block, then it programs again */
    addr(a, 0x000400);
    cmd(RL78_CMD_ERASE, a, 3);
    CHECK(st1() == RL78_ST_ACK, "erase");
    CHECK(program(0x000400, 0x0007FF, 2, &fe) == RL78_ST_ACK, "program after erase");
    /* data flash works the same way */
    CHECK(program(0x0F1000, 0x0F13FF, 3, &fe) == RL78_ST_ACK, "data flash program");
}

static void test_parameters(void)
{
    synced();
    uint8_t r[7], a[3];
    addr(a, 0x000401);
    cmd(RL78_CMD_ERASE, a, 3);
    CHECK(st1() == RL78_ST_PARAM_ERR, "misaligned erase");
    addr(a, 0x010000);
    cmd(RL78_CMD_ERASE, a, 3);
    CHECK(st1() == RL78_ST_PARAM_ERR, "erase past code flash");
    const uint32_t bad[][2] = {
        {0x000000, 0x0003FE},   /* end not a block end */
        {0x000400, 0x0003FF},   /* start > end */
        {0x00FC00, 0x0F13FF},   /* crosses code / data flash */
        {0x00FC00, 0x0103FF},   /* past code flash */
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        range(r, bad[i][0], bad[i][1]);
        cmd(RL78_CMD_PROGRAM, r, 6);
        CHECK(st1() == RL78_ST_PARAM_ERR, "range %zu not refused", i);
    }
    range(r, 0, 0x3FF); r[6] = 2;
    cmd(RL78_CMD_BLANK, r, 7);
    CHECK(st1() == RL78_ST_PARAM_ERR, "blank check D01=2");
    cmd(0x55, NULL, 0);
    CHECK(st1() == RL78_ST_CMD_ERR, "unknown command");
    /* a short Programming range: ETX before the end -> the internal verify fails */
    range(r, 0, 0x3FF);
    cmd(RL78_CMD_PROGRAM, r, 6);
    CHECK(st1() == RL78_ST_ACK, "program status");
    uint8_t d[256]; memset(d, 0x11, sizeof(d));
    int b = 0;
    data(d, 256, 1);
    CHECK(st2(&b) == RL78_ST_ACK && b == RL78_ST_ACK, "short frame");
    CHECK(st1() == RL78_ST_BLANK_ERR, "short range passed the internal verify");
}

static void test_bad_frames(void)
{
    synced();
    /* bad checksum: 07H */
    const uint8_t f1[] = {RL78_SOH, 0x01, RL78_CMD_RESET, 0x00, RL78_ETX};
    feed(f1, sizeof(f1));
    CHECK(st1() == RL78_ST_SUM_ERR, "bad sum");
    /* bad footer: 15H */
    const uint8_t f2[] = {RL78_SOH, 0x01, RL78_CMD_RESET, 0xFF, RL78_ETB};
    feed(f2, sizeof(f2));
    CHECK(st1() == RL78_ST_NACK, "bad footer");
    /* a partial frame dropped by the idle timeout, then a good one */
    const uint8_t f3[] = {RL78_SOH, 0x05, RL78_CMD_RESET};
    feed(f3, sizeof(f3));
    rl78emu_idle(&E);
    cmd(RL78_CMD_RESET, NULL, 0);
    CHECK(st1() == RL78_ST_ACK, "no resync after idle");
    /* a bad data frame during Programming gets the two-byte status */
    uint8_t r[6]; range(r, 0x2000, 0x23FF);
    cmd(RL78_CMD_PROGRAM, r, 6);
    CHECK(st1() == RL78_ST_ACK, "program status");
    uint8_t f4[260]; f4[0] = RL78_STX; f4[1] = 0; memset(&f4[2], 0x22, 256); f4[258] = 0x55; f4[259] = RL78_ETB;   /* the right sum is 00H */
    feed(f4, sizeof(f4));
    int b = 0;
    CHECK(st2(&b) == RL78_ST_SUM_ERR, "bad data frame sum");
}

static void test_security(void)
{
    synced();
    uint8_t d[256], r[7];
    int fe;
    cmd(RL78_CMD_SEC_GET, NULL, 0);
    CHECK(st1() == RL78_ST_ACK && next(d) == 8 && d[0] == 0xFE && d[1] == 0x03 && d[2] == 0 && d[3] == 0 &&
          d[4] == 63 && d[5] == 0 && d[6] == 0xFF && d[7] == 0xFF, "default security");
    /* disable programming */
    cmd(RL78_CMD_SEC_SET, NULL, 0);
    CHECK(st1() == RL78_ST_ACK, "security set status");
    const uint8_t no_prog[8] = {0xEE, 0x03, 0x00, 0x00, 63, 0x00, 0xFF, 0xFF};
    data(no_prog, 8, 1);
    CHECK(st1() == RL78_ST_ACK, "security set data");
    CHECK(program(0, 0x3FF, 1, &fe) == RL78_ST_PROTECT, "program while disabled");
    range(r, 0, 0xFFFF); r[6] = 1;
    cmd(RL78_CMD_BLANK, r, 7);
    CHECK(st1() == RL78_ST_BLANK_ERR, "flash option reads blank with a flag disabled");
    /* re-enabling needs a Security Release */
    cmd(RL78_CMD_SEC_SET, NULL, 0);
    (void)st1();
    const uint8_t all[8] = {0xFE, 0x03, 0x00, 0x00, 63, 0x00, 0xFF, 0xFF};
    data(all, 8, 1);
    CHECK(st1() == RL78_ST_PROTECT, "re-enabled a disabled flag");
    /* a wrong boot cluster size is a parameter error */
    cmd(RL78_CMD_SEC_SET, NULL, 0);
    (void)st1();
    const uint8_t bot[8] = {0xEE, 0x07, 0x00, 0x00, 63, 0x00, 0xFF, 0xFF};
    data(bot, 8, 1);
    CHECK(st1() == RL78_ST_PARAM_ERR, "BOT 07H accepted");
    /* release: refused while something is written, accepted on a blank part */
    synced();
    CHECK(program(0x1000, 0x13FF, 1, &fe) == RL78_ST_ACK, "program");
    cmd(RL78_CMD_SEC_RELEASE, NULL, 0);
    CHECK(st1() == RL78_ST_BLANK_ERR, "release with data written");
    uint8_t a[3]; addr(a, 0x1000);
    cmd(RL78_CMD_ERASE, a, 3);
    (void)st1();
    cmd(RL78_CMD_SEC_RELEASE, NULL, 0);
    CHECK(st1() == RL78_ST_ACK, "release on a blank part");
    /* a flash shield window protects the blocks outside it */
    cmd(RL78_CMD_SEC_SET, NULL, 0);
    (void)st1();
    const uint8_t fsw[8] = {0xFE, 0x03, 8, 0x00, 15, 0x00, 0xFF, 0xFF};   /* blocks 8..15 */
    data(fsw, 8, 1);
    CHECK(st1() == RL78_ST_ACK, "set FSW");
    CHECK(program(0x2000, 0x3FFF, 1, &fe) == RL78_ST_ACK, "program inside the window");
    CHECK(program(0x4000, 0x43FF, 1, &fe) == RL78_ST_PROTECT, "program outside the window");
    addr(a, 0x0000);
    cmd(RL78_CMD_ERASE, a, 3);
    CHECK(st1() == RL78_ST_PROTECT, "erase outside the window");
}

int main(void)
{
    test_checksum_examples();
    test_baud_first();
    test_signature();
    test_erase_blank_program_verify();
    test_parameters();
    test_bad_frames();
    test_security();
    if (failures) { printf("test_rl78emu: %d FAILED\n", failures); return 1; }
    printf("test_rl78emu: all passed\n");
    return 0;
}
