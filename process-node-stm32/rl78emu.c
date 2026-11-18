/*
 * rl78emu.c - the RL78 Protocol A boot ROM emulator (see rl78emu.h). Byte-exact to R01AN0815
 * Rev. 1.10, chapters 2-4: frames, checksums, status codes, the command flows below.
 *
 *   Reset 00H, Baud Rate Set 9AH, Silicon Signature C0H, Block Erase 22H, Block Blank Check 32H,
 *   Programming 40H (data frames, a double status per frame, a final internal-verify status),
 *   Verify 13H (the mismatch is reported in the last frame's ST2), Checksum B0H,
 *   Security Set A0H / Get A1H / Release A2H.
 *
 * Like the real part it refuses everything but Baud Rate Set first (04H), writes only to blank
 * bytes (1CH otherwise, as flash cannot turn a 0 back into a 1 without an erase), and honors the
 * security flags (10H). Pure C: no HAL, host-tested in tests/test_rl78emu.c.
 */
#include "rl78emu.h"

#include <stddef.h>
#include <string.h>

enum { P_HDR, P_LEN, P_BODY, P_SUM, P_FTR };

uint8_t rl78_sum(const uint8_t *p, size_t n)
{
    uint8_t s = 0;
    for (size_t i = 0; i < n; i++)
        s = (uint8_t)(s - p[i]);
    return s;
}

uint8_t *rl78emu_flash(rl78emu_t *e, uint32_t addr)
{
    if (addr <= RL78_CODE_END)
        return &e->code[addr - RL78_CODE_START];
    if (addr >= RL78_DATA_START && addr <= RL78_DATA_END)
        return &e->data[addr - RL78_DATA_START];
    return NULL;
}

/* newlib-nano's memset goes a byte at a time: ~30 ms for the 68 KB at 16 MHz, longer than a
   programmer leaves between the TOOL0 release and its first frame */
typedef uint32_t __attribute__((may_alias)) rl78_word_t;
/* no-tree-loop-distribute-patterns: GCC turns this loop back into that memset call otherwise */
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((optimize("no-tree-loop-distribute-patterns")))
#endif
static void fill_ff(uint8_t *p, size_t n)
{
    rl78_word_t *w = (rl78_word_t *)(void *)p;
    for (size_t i = 0; i < n / 4u; i++)
        w[i] = 0xFFFFFFFFu;
}

void rl78emu_init(rl78emu_t *e, const rl78emu_port_t *port)
{
    memset(e, 0, offsetof(rl78emu_t, code));   /* the state, not the flash */
    e->port = *port;
    e->baud = 115200u;
    e->phase = RL78_WAIT_CMD;
    fill_ff(e->code, sizeof(e->code));
    fill_ff(e->data, sizeof(e->data));
    e->sec_flg = RL78_FLG_FIXED | RL78_FLG_PROG_EN | RL78_FLG_ERASE_EN | RL78_FLG_BOOT_EN;
    e->sec_bot = RL78_BOOT_BLOCKS;
    e->sec_ss = 0;
    e->sec_se = (uint16_t)RL78_LAST_BLOCK;
}

void rl78emu_idle(rl78emu_t *e)
{
    e->pstate = P_HDR;
}

/* ---- replies ---- */

static void send_data(rl78emu_t *e, const uint8_t *d, size_t n, bool last)
{
    uint8_t f[260];
    f[0] = RL78_STX;
    f[1] = (uint8_t)n;          /* 256 is sent as 00H */
    memcpy(&f[2], d, n);
    f[2 + n] = rl78_sum(&f[1], n + 1);
    f[3 + n] = last ? RL78_ETX : RL78_ETB;
    e->port.send(e->port.ctx, f, n + 4);
}

static void status1(rl78emu_t *e, uint8_t st)
{
    send_data(e, &st, 1, true);
}

static void status2(rl78emu_t *e, uint8_t st1, uint8_t st2)
{
    const uint8_t d[2] = {st1, st2};
    send_data(e, d, 2, true);
}

/* ---- the flash ---- */

static uint32_t addr24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static bool in_code(uint32_t a) { return a <= RL78_CODE_END; }
static bool in_data(uint32_t a) { return a >= RL78_DATA_START && a <= RL78_DATA_END; }

/* A range of whole blocks inside one flash (code or data): start on a block start, end on a
   block end, start <= end. */
static bool range_ok(uint32_t sa, uint32_t ea)
{
    if ((sa & (RL78_BLOCK - 1u)) != 0u || (ea & (RL78_BLOCK - 1u)) != RL78_BLOCK - 1u || sa > ea)
        return false;
    return (in_code(sa) && in_code(ea)) || (in_data(sa) && in_data(ea));
}

static bool blank(rl78emu_t *e, uint32_t sa, uint32_t ea)
{
    for (uint32_t a = sa; a <= ea; a++)
        if (*rl78emu_flash(e, a) != 0xFFu)
            return false;
    return true;
}

static bool all_blank(rl78emu_t *e)
{
    return blank(e, RL78_CODE_START, RL78_CODE_END) && blank(e, RL78_DATA_START, RL78_DATA_END);
}

/* Code flash blocks the security settings shield from rewriting: the boot cluster (when its
   rewrite is disabled) and everything outside the flash shield window. Data flash is never
   shielded. */
static bool shielded(rl78emu_t *e, uint32_t sa, uint32_t ea)
{
    if (!in_code(sa))
        return false;
    for (uint32_t b = sa / RL78_BLOCK; b <= ea / RL78_BLOCK; b++) {
        if (!(e->sec_flg & RL78_FLG_BOOT_EN) && b <= e->sec_bot)
            return true;
        if (b < e->sec_ss || b > e->sec_se)
            return true;
    }
    return false;
}

/* ---- commands ---- */

static void cmd_baud(rl78emu_t *e, const uint8_t *info, size_t n)
{
    static const uint32_t rates[4] = {115200u, 250000u, 500000u, 1000000u};
    if (n != 2 || info[0] > 3u || info[1] < 18u) {    /* D02 is volts x10: 1.8 V minimum */
        status1(e, RL78_ST_PARAM_ERR);
        return;
    }
    const uint8_t d[3] = {RL78_ST_ACK, RL78_FREQ_MHZ, 0x00u};   /* full-speed mode */
    send_data(e, d, 3, true);                                  /* at the old rate */
    e->baud_set = true;
    e->baud = rates[info[0]];
    if (e->port.set_baud)
        e->port.set_baud(e->port.ctx, e->baud);
}

static void cmd_signature(rl78emu_t *e)
{
    static const uint8_t sig[22] = {
        0x10, 0x00, 0x06,                                       /* DEC: RL78/G13 */
        'R', '5', 'F', '1', '0', '0', 'L', 'E', ' ', ' ',       /* DEV */
        (uint8_t)RL78_CODE_END, (uint8_t)(RL78_CODE_END >> 8), (uint8_t)(RL78_CODE_END >> 16),
        (uint8_t)RL78_DATA_END, (uint8_t)(RL78_DATA_END >> 8), (uint8_t)(RL78_DATA_END >> 16),
        0x01, 0x02, 0x03,                                       /* VER: V1.23 */
    };
    status1(e, RL78_ST_ACK);
    send_data(e, sig, sizeof(sig), true);
}

static void cmd_erase(rl78emu_t *e, const uint8_t *info, size_t n)
{
    if (n != 3) { status1(e, RL78_ST_PARAM_ERR); return; }
    uint32_t sa = addr24(info);
    if (!range_ok(sa, sa + RL78_BLOCK - 1u)) { status1(e, RL78_ST_PARAM_ERR); return; }
    if (!(e->sec_flg & RL78_FLG_ERASE_EN) || shielded(e, sa, sa + RL78_BLOCK - 1u)) {
        status1(e, RL78_ST_PROTECT);
        return;
    }
    memset(rl78emu_flash(e, sa), 0xFF, RL78_BLOCK);
    status1(e, RL78_ST_ACK);
}

static void cmd_blank(rl78emu_t *e, const uint8_t *info, size_t n)
{
    if (n != 7 || info[6] > 1u) { status1(e, RL78_ST_PARAM_ERR); return; }
    uint32_t sa = addr24(info), ea = addr24(info + 3);
    if (!range_ok(sa, ea)) { status1(e, RL78_ST_PARAM_ERR); return; }
    /* D01 = 01H also checks the flash option (security data): blank when nothing is disabled */
    bool ok = blank(e, sa, ea);
    if (info[6] == 1u && (e->sec_flg & (RL78_FLG_PROG_EN | RL78_FLG_ERASE_EN | RL78_FLG_BOOT_EN)) !=
                             (RL78_FLG_PROG_EN | RL78_FLG_ERASE_EN | RL78_FLG_BOOT_EN))
        ok = false;
    status1(e, ok ? RL78_ST_ACK : RL78_ST_BLANK_ERR);
}

static void cmd_range(rl78emu_t *e, uint8_t com, const uint8_t *info, size_t n)
{
    if (n != 6) { status1(e, RL78_ST_PARAM_ERR); return; }
    uint32_t sa = addr24(info), ea = addr24(info + 3);
    if (!range_ok(sa, ea)) { status1(e, RL78_ST_PARAM_ERR); return; }
    if (com == RL78_CMD_PROGRAM && (!(e->sec_flg & RL78_FLG_PROG_EN) || shielded(e, sa, ea))) {
        status1(e, RL78_ST_PROTECT);
        return;
    }
    if (com == RL78_CMD_CHECKSUM) {
        uint16_t ck = 0;
        for (uint32_t a = sa; a <= ea; a++)
            ck = (uint16_t)(ck - *rl78emu_flash(e, a));
        const uint8_t d[2] = {(uint8_t)ck, (uint8_t)(ck >> 8)};
        status1(e, RL78_ST_ACK);
        send_data(e, d, 2, true);
        return;
    }
    e->cur = sa;
    e->end = ea;
    e->mismatch = false;
    e->write_failed = false;
    e->phase = (com == RL78_CMD_PROGRAM) ? RL78_WAIT_PROG : RL78_WAIT_VERIFY;
    status1(e, RL78_ST_ACK);
}

/* One data frame of a Programming or Verify range. */
static void data_frame(rl78emu_t *e, const uint8_t *d, size_t n, bool last)
{
    bool prog = (e->phase == RL78_WAIT_PROG);
    uint8_t st2 = RL78_ST_ACK;
    if (e->cur + n - 1u > e->end) {                  /* more data than the range holds */
        status2(e, RL78_ST_ACK, prog ? RL78_ST_WRITE_ERR : RL78_ST_VERIFY_ERR);
        e->phase = RL78_WAIT_CMD;
        return;
    }
    uint8_t *f = rl78emu_flash(e, e->cur);
    if (prog) {
        /* a frame is written whole or not at all: any non-blank byte that would change needs an
           erase first */
        for (size_t i = 0; i < n; i++)
            if (f[i] != 0xFFu && f[i] != d[i]) e->write_failed = true;
        if (!e->write_failed) memcpy(f, d, n);
    } else if (memcmp(f, d, n) != 0) {
        e->mismatch = true;
    }
    e->cur += (uint32_t)n;
    if (prog) {
        st2 = e->write_failed ? RL78_ST_WRITE_ERR : RL78_ST_ACK;
    } else if (last) {
        /* Verify reports a mismatch (or a short range) only in the last frame's ST2 */
        st2 = (e->mismatch || e->cur != e->end + 1u) ? RL78_ST_VERIFY_ERR : RL78_ST_ACK;
    }
    status2(e, RL78_ST_ACK, st2);
    if (prog && e->write_failed) {
        e->phase = RL78_WAIT_CMD;
        return;
    }
    if (!last)
        return;
    if (prog)   /* the internal verify after the last frame: the whole range must have arrived */
        status1(e, e->cur == e->end + 1u ? RL78_ST_ACK : RL78_ST_BLANK_ERR);
    e->phase = RL78_WAIT_CMD;
}

static void security_frame(rl78emu_t *e, const uint8_t *d, size_t n)
{
    e->phase = RL78_WAIT_CMD;
    if (n != 8) { status1(e, RL78_ST_PARAM_ERR); return; }
    uint8_t flg = d[0], bot = d[1];
    uint16_t ss = (uint16_t)(d[2] | (d[3] << 8)), se = (uint16_t)(d[4] | (d[5] << 8));
    if (bot != RL78_BOOT_BLOCKS || ss > se || se > RL78_LAST_BLOCK) {
        status1(e, RL78_ST_PARAM_ERR);
        return;
    }
    /* a disabled permission can only come back through Security Release */
    const uint8_t perm = RL78_FLG_PROG_EN | RL78_FLG_ERASE_EN | RL78_FLG_BOOT_EN;
    if ((flg & perm) & ~(e->sec_flg & perm)) {
        status1(e, RL78_ST_PROTECT);
        return;
    }
    e->sec_flg = (uint8_t)(RL78_FLG_FIXED | (flg & perm));
    e->sec_ss = ss;
    e->sec_se = se;
    status1(e, RL78_ST_ACK);
}

static void cmd_sec_get(rl78emu_t *e)
{
    /* Get reports bit 0 as the boot area exchange flag: none on this part */
    const uint8_t d[8] = {e->sec_flg, e->sec_bot, (uint8_t)e->sec_ss, (uint8_t)(e->sec_ss >> 8),
                          (uint8_t)e->sec_se, (uint8_t)(e->sec_se >> 8), 0xFF, 0xFF};
    status1(e, RL78_ST_ACK);
    send_data(e, d, sizeof(d), true);
}

static void cmd_sec_release(rl78emu_t *e)
{
    if (!(e->sec_flg & RL78_FLG_ERASE_EN) || !(e->sec_flg & RL78_FLG_BOOT_EN)) {
        status1(e, RL78_ST_PROTECT);
        return;
    }
    if (!all_blank(e)) {
        status1(e, RL78_ST_BLANK_ERR);
        return;
    }
    e->sec_flg = RL78_FLG_FIXED | RL78_FLG_PROG_EN | RL78_FLG_ERASE_EN | RL78_FLG_BOOT_EN;
    e->sec_ss = 0;
    e->sec_se = (uint16_t)RL78_LAST_BLOCK;
    status1(e, RL78_ST_ACK);
}

static void command(rl78emu_t *e, const uint8_t *p, size_t n)
{
    uint8_t com = p[0];
    const uint8_t *info = p + 1;
    size_t ni = n - 1;
    if (!e->baud_set && com != RL78_CMD_BAUD) {   /* Baud Rate Set must come first */
        status1(e, RL78_ST_CMD_ERR);
        return;
    }
    switch (com) {
    case RL78_CMD_BAUD:      cmd_baud(e, info, ni); break;
    case RL78_CMD_RESET:     status1(e, RL78_ST_ACK); break;
    case RL78_CMD_SIGNATURE: cmd_signature(e); break;
    case RL78_CMD_ERASE:     cmd_erase(e, info, ni); break;
    case RL78_CMD_BLANK:     cmd_blank(e, info, ni); break;
    case RL78_CMD_PROGRAM:
    case RL78_CMD_VERIFY:
    case RL78_CMD_CHECKSUM:  cmd_range(e, com, info, ni); break;
    case RL78_CMD_SEC_SET:   e->phase = RL78_WAIT_SEC; status1(e, RL78_ST_ACK); break;
    case RL78_CMD_SEC_GET:   cmd_sec_get(e); break;
    case RL78_CMD_SEC_RELEASE: cmd_sec_release(e); break;
    default:                 status1(e, RL78_ST_CMD_ERR); break;
    }
}

/* A whole frame arrived with a good checksum and footer. */
static void frame(rl78emu_t *e, uint8_t hdr, const uint8_t *p, size_t n, uint8_t ftr)
{
    e->frames++;
    if (hdr == RL78_SOH) {
        if (e->phase != RL78_WAIT_CMD) {   /* a command in the middle of a data phase ends it */
            e->phase = RL78_WAIT_CMD;
        }
        command(e, p, n);
        return;
    }
    /* a data frame */
    switch (e->phase) {
    case RL78_WAIT_PROG:
    case RL78_WAIT_VERIFY: data_frame(e, p, n, ftr == RL78_ETX); break;
    case RL78_WAIT_SEC:    security_frame(e, p, n); break;
    default:               e->errors++; status1(e, RL78_ST_NACK); break;
    }
}

/* A frame failed its checksum or footer: answer in the shape the programmer waits for. */
static void bad_frame(rl78emu_t *e, uint8_t hdr, uint8_t st)
{
    e->errors++;
    if (hdr == RL78_STX && (e->phase == RL78_WAIT_PROG || e->phase == RL78_WAIT_VERIFY)) {
        status2(e, st, st);
        e->phase = RL78_WAIT_CMD;
        return;
    }
    e->phase = RL78_WAIT_CMD;
    status1(e, st);
}

void rl78emu_rx(rl78emu_t *e, uint8_t b)
{
    switch (e->pstate) {
    case P_HDR:
        /* anything else (the 3AH/00H mode byte, line noise) is not a frame start */
        if (b == RL78_SOH || b == RL78_STX) {
            e->hdr = b;
            e->pstate = P_LEN;
        }
        break;
    case P_LEN:
        if (e->hdr == RL78_SOH && b == 0u) {   /* a command frame has at least COM */
            e->pstate = P_HDR;
            bad_frame(e, e->hdr, RL78_ST_NACK);
            break;
        }
        e->len_byte = b;
        e->want = b ? b : 256u;
        e->got = 0;
        e->pstate = P_BODY;
        break;
    case P_BODY:
        e->buf[e->got++] = b;
        if (e->got == e->want)
            e->pstate = P_SUM;
        break;
    case P_SUM:
        e->sum = b;
        e->pstate = P_FTR;
        break;
    case P_FTR: {
        e->pstate = P_HDR;
        bool ftr_ok = (b == RL78_ETX) || (e->hdr == RL78_STX && b == RL78_ETB);
        if (!ftr_ok) {
            bad_frame(e, e->hdr, RL78_ST_NACK);
            break;
        }
        uint8_t s = (uint8_t)(rl78_sum(e->buf, e->want) - e->len_byte);
        if (s != e->sum) {
            bad_frame(e, e->hdr, RL78_ST_SUM_ERR);
            break;
        }
        frame(e, e->hdr, e->buf, e->want, b);
        break;
    }
    }
}
