/* Serial bootloaders on the CDC port, for testing a host's flashers over USB serial:
 *
 *   optiboot    STK500v1 at 115200 (an Uno: ATmega328P)
 *   atmegaboot  STK500v1 at 57600 (an old Nano, Pro Mini: ATmega328P)
 *   stk500v2    STK500v2 at 115200 (a Mega 2560)
 *   leonardo    a 32u4 sketch: the 1200 baud touch resets it into caterina
 *   caterina    AVR109 (Caterina, ATmega32U4); E starts the sketch (leonardo) again
 *   zero        a SAMD21 sketch: the 1200 baud touch resets it into samba
 *   samba       SAM-BA with Arduino's X/Y commands (SAMD21G18, 256 KB, 8 KB bootloader); a write to
 *               AIRCR starts the sketch (zero) again
 *
 * Like the real chips, an AVR bootloader only understands bytes at its baud rate: bytes at another
 * rate are dropped, so a host has to find the rate. The flash is emulated in the RAM drive's memory
 * (480 KB, erased at a mode change from the console, kept across the touch and leave resets) and
 * `image <addr> <len>` reports its CRC-32. */
#include "loader.h"

#include <string.h>

#include "board.h"
#include "bridge.h"
#include "disk.h"
#include "tusb.h"

#define IMAGE_SIZE (DISK_SECTORS * DISK_SECTOR)

/* AVR signatures (in byte order 0,1,2). */
static const uint8_t SIG_328P[3] = {0x1E, 0x95, 0x0F};
static const uint8_t SIG_2560[3] = {0x1E, 0x98, 0x01};
static const uint8_t SIG_32U4[3] = {0x1E, 0x95, 0x87};

/* SAMD21G18A: DSU DID (Cortex-M0+), NVMCTRL PARAM (64-byte pages, 4096 of them = 256 KB). */
#define SAMD_DID       0x41002018u
#define SAMD_DID_VAL   0x10010005u
#define SAMD_PARAM     0x41004008u
#define SAMD_PARAM_VAL 0x00031000u
#define SAMD_FLASH     (256u * 1024u)
#define SAMBA_RAM_BASE 0x20000000u
#define AIRCR          0xE000ED0Cu

static uint8_t s_rx[1200];  /* the command being collected */
static uint32_t s_n;
static uint32_t s_addr;     /* AVR: byte address; SAM-BA: the Y source */
static uint8_t s_samba_ram[32 * 1024];
static uint32_t s_samba_ram_at; /* SAM-BA: an S in progress (bytes still to come) */
static uint32_t s_samba_left;
static bool s_binary;
static uint32_t s_reset_at;    /* when to reset (0: none) */
static node_mode_t s_reset_to;
static loader_stats_t s_st;
bool s_debug;

static uint32_t mode_baud(void)
{
    switch (g_mode) {
    case MODE_OPTIBOOT:
    case MODE_STK500V2: return 115200;
    case MODE_ATMEGABOOT: return 57600;
    default: return 0; /* native USB: any rate */
    }
}

static uint8_t *image(void) { return disk_sector(0); }

void loader_erase_image(void) { memset(image(), 0xFF, IMAGE_SIZE); }

const loader_stats_t *loader_stats(void) { return &s_st; }

static void send(const uint8_t *p, uint32_t n) { serial_out(p, n); }
static void send1(uint8_t b) { send(&b, 1); }

static void schedule_reset(node_mode_t to)
{
    s_reset_to = to;
    s_reset_at = millis() + 30u;  /* after the reply has gone out */
    if (!s_reset_at) s_reset_at = 1;
}

static bool in_image(uint32_t addr, uint32_t n) { return addr <= IMAGE_SIZE && n <= IMAGE_SIZE - addr; }

static void write_image(uint32_t addr, const uint8_t *p, uint32_t n)
{
    if (!in_image(addr, n)) {
        s_st.errors++;
        return;
    }
    memcpy(image() + addr, p, n);
    s_st.bytes_written += n;
    if (addr + n > s_st.hi) s_st.hi = addr + n;
}

/* ---- STK500v1 */
#define STK_OK 0x10
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20

static uint32_t v1_need(void)
{
    switch (s_rx[0]) {
    case '0': case '1': case 'P': case 'Q': case 'R': case 'u': return 2;
    case 'A': return 3;
    case 'U': return 4;
    case 't': return 5;
    case 'V': return 6;
    case 'E': return 7;
    case 'B': return 22;
    case 'd': return s_n >= 3 ? 5u + (((uint32_t)s_rx[1] << 8) | s_rx[2]) : 0;
    default: return 1;
    }
}

static void v1_handle(void)
{
    uint32_t len = s_n;
    if (s_rx[len - 1] != CRC_EOP) {
        send1(STK_NOSYNC);
        return;
    }
    send1(STK_INSYNC);
    switch (s_rx[0]) {
    case '0': s_st.syncs++; break;
    case 'u': send(SIG_328P, 3); break;
    case 'A': send1(0x03); break;
    case 'V': send1(0x00); break;
    case 'U': s_addr = (((uint32_t)s_rx[2] << 8) | s_rx[1]) * 2u; break;
    case 'd': {
        uint32_t n = ((uint32_t)s_rx[1] << 8) | s_rx[2];
        write_image(s_addr, s_rx + 4, n);
        s_st.pages++;
        break;
    }
    case 't': {
        uint32_t n = ((uint32_t)s_rx[1] << 8) | s_rx[2];
        if (in_image(s_addr, n)) send(image() + s_addr, n);
        break;
    }
    case 'Q': s_st.leaves++; con_write("EV loader leave\n"); break;
    default: break;
    }
    send1(STK_OK);
}

/* ---- STK500v2 */
static void v2_reply(uint8_t seq, const uint8_t *body, uint32_t n)
{
    uint8_t h[5] = {0x1B, seq, (uint8_t)(n >> 8), (uint8_t)n, 0x0E};
    uint8_t ck = 0;
    for (int i = 0; i < 5; i++) ck ^= h[i];
    for (uint32_t i = 0; i < n; i++) ck ^= body[i];
    send(h, 5);
    send(body, n);
    send1(ck);
}

static void v2_handle(void)
{
    uint32_t n = ((uint32_t)s_rx[2] << 8) | s_rx[3];
    const uint8_t *b = s_rx + 5;
    uint8_t ck = 0;
    for (uint32_t i = 0; i < 5 + n; i++) ck ^= s_rx[i];
    if (ck != s_rx[5 + n]) {
        s_st.errors++;
        return;
    }
    static uint8_t a[300];
    a[0] = b[0];
    a[1] = 0x00; /* STATUS_CMD_OK */
    uint32_t an = 2;
    switch (b[0]) {
    case 0x01: /* SIGN_ON */
        a[2] = 8;
        memcpy(a + 3, "AVRISP_2", 8);
        an = 11;
        s_st.syncs++;
        break;
    case 0x1B: /* READ_SIGNATURE_ISP: cmd, 0x04, 0x30, 0x00, index, 0x00 */
        a[2] = SIG_2560[b[4] % 3];
        a[3] = 0x00;
        an = 4;
        break;
    case 0x06: /* LOAD_ADDRESS (words, bit 31 = extended) */
        s_addr = ((((uint32_t)b[1] << 24) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 8) | b[4]) & 0x7FFFFFFFu) * 2u;
        break;
    case 0x13: { /* PROGRAM_FLASH_ISP: len, mode, delay, 3 cmds, poll1, poll2, data */
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        write_image(s_addr, b + 10, len);
        s_addr += len;
        s_st.pages++;
        break;
    }
    case 0x14: { /* READ_FLASH_ISP: len, cmd */
        uint32_t len = ((uint32_t)b[1] << 8) | b[2];
        if (len > sizeof(a) - 3 || !in_image(s_addr, len)) {
            a[1] = 0xC0;
            break;
        }
        memcpy(a + 2, image() + s_addr, len);
        a[2 + len] = 0x00;
        an = 3 + len;
        s_addr += len;
        break;
    }
    case 0x11: s_st.leaves++; con_write("EV loader leave\n"); break;
    default: break; /* ENTER_PROGMODE_ISP and others: ok */
    }
    v2_reply(s_rx[1], a, an);
}

/* ---- AVR109 (Caterina) */
static uint32_t a109_need(void)
{
    switch (s_rx[0]) {
    case 'A': return 3;
    case 'g': return 4;
    case 'B': return s_n >= 3 ? 4u + (((uint32_t)s_rx[1] << 8) | s_rx[2]) : 0;
    default: return 1;
    }
}

static void a109_handle(void)
{
    switch (s_rx[0]) {
    case 'S': send((const uint8_t *)"CATERIN", 7); s_st.syncs++; break;
    case 's': { const uint8_t r[3] = {SIG_32U4[2], SIG_32U4[1], SIG_32U4[0]}; send(r, 3); break; }
    case 'b': { const uint8_t r[3] = {'Y', 0x00, 0x80}; send(r, 3); break; }
    case 'P': case 'L': case 'T': send1('\r'); break;
    case 'e': memset(image(), 0xFF, 32768u - 4096u); send1('\r'); break;
    case 'A': s_addr = (((uint32_t)s_rx[1] << 8) | s_rx[2]) * 2u; send1('\r'); break;
    case 'B': {
        uint32_t n = ((uint32_t)s_rx[1] << 8) | s_rx[2];
        write_image(s_addr, s_rx + 4, n);
        s_addr += n;
        s_st.pages++;
        send1('\r');
        break;
    }
    case 'g': {
        uint32_t n = ((uint32_t)s_rx[1] << 8) | s_rx[2];
        if (in_image(s_addr, n)) send(image() + s_addr, n);
        s_addr += n;
        break;
    }
    case 'E':
        send1('\r');
        s_st.leaves++;
        con_write("EV loader leave\n");
        schedule_reset(MODE_LEONARDO);
        break;
    default: send1('?'); break;
    }
}

/* ---- SAM-BA */
static uint32_t hexval(const uint8_t *p, uint32_t n)
{
    uint32_t v = 0;
    for (uint32_t i = 0; i < n; i++) {
        uint8_t c = p[i];
        v = (v << 4) | (uint32_t)(c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10);
    }
    return v;
}

static void samba_ack(char op)
{
    if (!s_binary) return;
    const uint8_t r[3] = {(uint8_t)op, '\n', '\r'};
    send(r, 3);
}

static uint8_t *samba_ram(uint32_t a, uint32_t n)
{
    if (a < SAMBA_RAM_BASE || a - SAMBA_RAM_BASE > sizeof(s_samba_ram) ||
        n > sizeof(s_samba_ram) - (a - SAMBA_RAM_BASE)) return NULL;
    return s_samba_ram + (a - SAMBA_RAM_BASE);
}

static void samba_handle(void)
{
    /* s_rx: op, then hex a, optionally ',' hex b, ended by '#' */
    char op = (char)s_rx[0];
    const uint8_t *p = s_rx + 1, *end = s_rx + s_n - 1;
    const uint8_t *comma = memchr(p, ',', (size_t)(end - p));
    uint32_t a = hexval(p, (uint32_t)((comma ? comma : end) - p));
    uint32_t b = comma ? hexval(comma + 1, (uint32_t)(end - comma - 1)) : 0;
    if (s_debug) con_printf("EV samba %c %08lx %lx\n", op, (unsigned long)a, (unsigned long)b);
    switch (op) {
    case 'N': s_binary = true; send((const uint8_t *)"\n\r", 2); s_st.syncs++; break;
    case 'T': s_binary = false; send((const uint8_t *)"\n\r", 2); break;
    case 'V': send((const uint8_t *)"v2.0 [Arduino:XYZ] Nov 22 2026 00:00:00\n\r", 41); break;
    case 'w': {
        uint32_t v = a == SAMD_DID ? SAMD_DID_VAL : a == SAMD_PARAM ? SAMD_PARAM_VAL : 0;
        const uint8_t r[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
        send(r, 4);
        break;
    }
    case 'X': /* erase from a to the end of flash */
        if (a < SAMD_FLASH) memset(image() + a, 0xFF, SAMD_FLASH - a);
        s_st.erases++;
        samba_ack('X');
        break;
    case 'S': /* b bytes to RAM at a follow */
        s_samba_ram_at = a;
        s_samba_left = b;
        break;
    case 'Y':
        if (b == 0) {
            s_addr = a; /* the source in RAM */
        } else {
            uint8_t *src = samba_ram(s_addr, b);
            if (src && a + b <= SAMD_FLASH) {
                write_image(a, src, b);
                s_st.pages++;
            } else {
                s_st.errors++;
            }
        }
        samba_ack('Y');
        break;
    case 'R':
        if (in_image(a, b)) send(image() + a, b);
        if (s_debug) con_printf("EV samba R %08lx %lu crc %08lx\n", (unsigned long)a, (unsigned long)b,
                                (unsigned long)(in_image(a, b) ? crc32_update(0, image() + a, b) : 0));
        break;
    case 'W':
        if (a == AIRCR && (b & 0x4u)) {
            s_st.leaves++;
            con_write("EV loader leave\n");
            schedule_reset(MODE_ZERO);
        }
        break;
    default: break;
    }
}

static void samba_rx(const uint8_t *p, uint32_t n)
{
    while (n) {
        if (s_samba_left) { /* the data of an S */
            uint32_t k = n < s_samba_left ? n : s_samba_left;
            uint8_t *dst = samba_ram(s_samba_ram_at, k);
            if (dst) memcpy(dst, p, k);
            else s_st.errors++;
            s_samba_ram_at += k;
            s_samba_left -= k;
            p += k;
            n -= k;
            continue;
        }
        uint8_t c = *p++;
        n--;
        if (s_n < sizeof(s_rx)) s_rx[s_n++] = c;
        if (c == '#') {
            if (s_n >= 2) samba_handle();
            s_n = 0;
        }
    }
}

/* ---- dispatch */
void loader_rx(const uint8_t *p, uint32_t n)
{
    if (g_mode == MODE_SAMBA) {
        samba_rx(p, n);
        return;
    }
    uint32_t want = mode_baud();
    if (want && st_cdc_baud() != want) {
        s_st.dropped += n; /* the wrong rate: the chip sees garbage */
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        if (s_n < sizeof(s_rx)) s_rx[s_n++] = p[i];
        else s_n = 0;
        for (;;) {
            uint32_t need;
            if (g_mode == MODE_STK500V2) {
                if (s_rx[0] != 0x1B) { s_n = 0; break; }
                need = s_n >= 4 ? 6u + (((uint32_t)s_rx[2] << 8) | s_rx[3]) : 0;
            } else if (g_mode == MODE_CATERINA) {
                need = a109_need();
            } else {
                need = v1_need();
            }
            if (!need || s_n < need) break;
            if (g_mode == MODE_STK500V2) v2_handle();
            else if (g_mode == MODE_CATERINA) a109_handle();
            else if (need > 1) v1_handle();  /* a stray byte: dropped, as optiboot does */
            s_n = 0;
            break;
        }
    }
}

void loader_line_state(bool dtr, bool was_dtr, uint32_t baud)
{
    /* The 1200 baud touch: the sketch resets into its bootloader. */
    if (was_dtr && !dtr && baud == 1200 && (g_mode == MODE_LEONARDO || g_mode == MODE_ZERO)) {
        con_write("EV cdc touch1200\n");
        schedule_reset(g_mode == MODE_LEONARDO ? MODE_CATERINA : MODE_SAMBA);
    }
    if (dtr && !was_dtr) { /* a DTR reset of an AVR board restarts its bootloader */
        s_n = 0;
        s_binary = false;
    }
}

bool loader_reset_due(node_mode_t *to)
{
    if (!s_reset_at || (int32_t)(millis() - s_reset_at) < 0) return false;
    s_reset_at = 0;
    *to = s_reset_to;
    return true;
}

void loader_print_image(uint32_t addr, uint32_t len)
{
    if (!in_image(addr, len)) {
        con_printf("ERR image %lu bytes at 0x%lx is outside the %u KB image\n", (unsigned long)len,
                   (unsigned long)addr, IMAGE_SIZE / 1024u);
        return;
    }
    con_printf("IMAGE addr 0x%lx len %lu crc %08lx\n", (unsigned long)addr, (unsigned long)len,
               (unsigned long)crc32_update(0, image() + addr, len));
}
