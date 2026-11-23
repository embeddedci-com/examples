/* An ESP32-S3 on its own USB port (the USB-Serial/JTAG peripheral, 303A:1001), for testing a host's
 * ESP32 flasher: mode esp32s3.
 *
 * The chip runs its "sketch" (bytes are ignored) until the host resets it into download mode the
 * way esptool does over USB-Serial/JTAG: RTS (the chip's reset) goes low while DTR (GPIO0 low) is
 * held, so the reset is released with GPIO0 low. A reset released with DTR low boots the sketch
 * again. In download mode it answers the ROM loader's SLIP commands: SYNC, READ_REG (the chip
 * magic), SPI_ATTACH, SPI_SET_PARAMS, CHANGE_BAUDRATE, FLASH_BEGIN (erases), FLASH_DATA (checks
 * each block's checksum and sequence), SPI_FLASH_MD5 and FLASH_END, into the emulated flash. */
#include "esp_rom.h"

#include <string.h>

#include "board.h"
#include "disk.h"
#include "md5.h"
#include "tusb.h"

#define IMAGE_SIZE (DISK_SECTORS * DISK_SECTOR)
#define CHIP_MAGIC_REG 0x40001000u
#define S3_MAGIC 0x00000009u

#define SLIP_END 0xC0
#define SLIP_ESC 0xDB

static uint8_t s_frame[1100]; /* a decoded command: 8-byte header + up to 16 + 1024 */
static uint32_t s_n;
static bool s_in, s_esc;
static bool s_download;
static bool s_dtr, s_rts;
static bool s_io0_low_at_release;
static uint32_t s_begin_off, s_begin_size, s_blocks, s_block_size, s_next_seq;
static esp_rom_stats_t s_st;

const esp_rom_stats_t *esp_rom_stats(void) { return &s_st; }

static uint8_t *image(void) { return disk_sector(0); }

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint32_t get32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

static void send_slip(const uint8_t *p, uint32_t n)
{
    uint8_t out[160];
    uint32_t k = 0;
    out[k++] = SLIP_END;
    for (uint32_t i = 0; i < n; i++) {
        if (p[i] == SLIP_END) { out[k++] = SLIP_ESC; out[k++] = 0xDC; }
        else if (p[i] == SLIP_ESC) { out[k++] = SLIP_ESC; out[k++] = 0xDD; }
        else out[k++] = p[i];
    }
    out[k++] = SLIP_END;
    const uint8_t *q = out;
    while (k) {
        uint32_t w = tud_cdc_write(q, k);
        tud_cdc_write_flush();
        q += w;
        k -= w;
        if (k) tud_task();
    }
}

/* A response: 0x01, op, size, value, body, then the 4 status bytes of the S3 ROM. */
static void reply(uint8_t op, uint32_t value, const uint8_t *body, uint16_t blen, uint8_t err)
{
    uint8_t r[8 + 64 + 4];
    uint16_t size = (uint16_t)(blen + 4);
    r[0] = 0x01; r[1] = op; r[2] = (uint8_t)size; r[3] = (uint8_t)(size >> 8);
    put32(r + 4, value);
    if (blen) memcpy(r + 8, body, blen);
    r[8 + blen] = err ? 1 : 0;
    r[9 + blen] = err;
    r[10 + blen] = 0;
    r[11 + blen] = 0;
    send_slip(r, 8u + size);
}

static void handle(void)
{
    if (s_n < 8 || s_frame[0] != 0x00) return;
    uint8_t op = s_frame[1];
    uint16_t len = (uint16_t)(s_frame[2] | (s_frame[3] << 8));
    uint32_t chk = get32(s_frame + 4);
    const uint8_t *d = s_frame + 8;
    if (8u + len > s_n) return;
    switch (op) {
    case 0x08: /* SYNC: the ROM answers it several times */
        s_st.syncs++;
        for (int i = 0; i < 4; i++) reply(op, 0x20120707, NULL, 0, 0);
        break;
    case 0x0A: /* READ_REG */
        reply(op, len >= 4 && get32(d) == CHIP_MAGIC_REG ? S3_MAGIC : 0, NULL, 0, 0);
        break;
    case 0x0D: case 0x0B: /* SPI_ATTACH, SPI_SET_PARAMS */
        reply(op, 0, NULL, 0, 0);
        break;
    case 0x0F: /* CHANGE_BAUDRATE: native USB has no rate */
        s_st.baud_changes++;
        reply(op, 0, NULL, 0, 0);
        break;
    case 0x02: { /* FLASH_BEGIN: erase size, blocks, block size, offset [, encrypted] */
        if (len < 16) { reply(op, 0, NULL, 0, 0x05); break; }
        s_begin_size = get32(d); s_blocks = get32(d + 4); s_block_size = get32(d + 8); s_begin_off = get32(d + 12);
        if (s_begin_off > IMAGE_SIZE || s_begin_size > IMAGE_SIZE - s_begin_off || s_block_size == 0 || s_block_size > 1024) {
            reply(op, 0, NULL, 0, 0x05);
            break;
        }
        memset(image() + s_begin_off, 0xFF, s_begin_size);
        s_next_seq = 0;
        s_st.begins++;
        reply(op, 0, NULL, 0, 0);
        break;
    }
    case 0x03: { /* FLASH_DATA: size, seq, 0, 0, data */
        if (len < 16) { reply(op, 0, NULL, 0, 0x05); break; }
        uint32_t n = get32(d), seq = get32(d + 4);
        uint8_t x = 0xEF;
        for (uint32_t i = 0; i < n && 16u + i < len; i++) x ^= d[16 + i];
        if (n + 16u > len || x != (uint8_t)chk) { s_st.bad_checksums++; reply(op, 0, NULL, 0, 0x07); break; }
        if (seq != s_next_seq && seq + 1 != s_next_seq) { s_st.errors++; reply(op, 0, NULL, 0, 0x08); break; }
        uint32_t at = s_begin_off + seq * s_block_size;
        if (at > IMAGE_SIZE || n > IMAGE_SIZE - at) { s_st.errors++; reply(op, 0, NULL, 0, 0x05); break; }
        memcpy(image() + at, d + 16, n);
        if (seq == s_next_seq) { s_next_seq++; s_st.blocks++; } else { s_st.resends++; }
        reply(op, 0, NULL, 0, 0);
        break;
    }
    case 0x13: { /* SPI_FLASH_MD5: address, size -> 32 hex characters */
        uint32_t a = len >= 8 ? get32(d) : 0, n = len >= 8 ? get32(d + 4) : 0;
        if (a > IMAGE_SIZE || n > IMAGE_SIZE - a) { reply(op, 0, NULL, 0, 0x05); break; }
        uint8_t h[16];
        char hex[32];
        md5(image() + a, n, h);
        for (int i = 0; i < 16; i++) {
            hex[i * 2] = "0123456789abcdef"[h[i] >> 4];
            hex[i * 2 + 1] = "0123456789abcdef"[h[i] & 15];
        }
        s_st.md5s++;
        reply(op, 0, (const uint8_t *)hex, 32, 0);
        break;
    }
    case 0x04: /* FLASH_END */
        s_st.ends++;
        reply(op, 0, NULL, 0, 0);
        break;
    default:
        reply(op, 0, NULL, 0, 0x05);
        break;
    }
}

void esp_rom_rx(const uint8_t *p, uint32_t n)
{
    if (!s_download) {
        s_st.ignored += n; /* the sketch does not speak the ROM protocol */
        return;
    }
    for (uint32_t i = 0; i < n; i++) {
        uint8_t c = p[i];
        if (c == SLIP_END) {
            if (s_in && s_n) handle();
            s_in = true;
            s_n = 0;
            s_esc = false;
            continue;
        }
        if (!s_in) continue;
        if (s_esc) {
            c = c == 0xDC ? SLIP_END : c == 0xDD ? SLIP_ESC : c;
            s_esc = false;
        } else if (c == SLIP_ESC) {
            s_esc = true;
            continue;
        }
        if (s_n < sizeof(s_frame)) s_frame[s_n++] = c;
    }
}

/* USB-Serial/JTAG, as esptool drives it: DTR=1 with RTS=0 arms GPIO0 low, DTR=0 with RTS=1 holds
   the chip in reset, and the reset's release boots it: into download mode when GPIO0 was armed
   (esptool goes (0,0) (1,0) (1,1) (0,1) (0,0)), else the sketch (a hard reset: (0,1) (0,0)). */
void esp_rom_line_state(bool dtr, bool rts)
{
    bool was_rts = s_rts;
    if (dtr && !rts) s_io0_low_at_release = true;
    if (was_rts && !rts && !dtr) {
        s_download = s_io0_low_at_release;
        s_io0_low_at_release = false;
        s_in = false;
        s_n = 0;
        s_st.resets++;
        con_write(s_download ? "EV esp download\n" : "EV esp boot\n");
    }
    s_dtr = dtr;
    s_rts = rts;
}

void esp_rom_boot(bool download) { s_download = download; }
bool esp_rom_in_download(void) { return s_download; }
