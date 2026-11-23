/* USB-serial bridge chips: instead of CDC, the serial modes enumerate as an FTDI FT232R, a Silicon
 * Labs CP2102 or a WCH CH340, the chips on boards without native USB (Nano clones, Pro Minis, ESP32
 * dev kits). The chip's vendor requests set the baud rate (decoded from each chip's divisor
 * encoding), the frame and DTR/RTS; data goes over its two bulk endpoints, FTDI with its two
 * modem-status bytes in front of every IN packet. What is behind the chip is the mode's bootloader
 * (loader.c, esp_rom.c), the same as on the CDC port.
 *
 * serial_out() is the one way the bootloaders send, for CDC and the bridges alike. */
#include "bridge.h"

#include <string.h>

#include "board.h"
#include "device/usbd_pvt.h"
#include "tusb.h"

#define EP_IN   0x82
#define EP_OUT  0x02
#define EP_INT  0x83
#define EP_SIZE 64u

bridge_kind_t g_bridge;

static uint8_t s_out_buf[EP_SIZE];
static uint8_t s_in_buf[EP_SIZE];
static uint8_t s_tx[4096];
static volatile uint32_t s_tx_head, s_tx_tail;
static bool s_open, s_in_busy;
static uint8_t s_ctrl[16];
static uint32_t s_baud;
static bool s_dtr, s_rts;

const char *bridge_name(bridge_kind_t k)
{
    static const char *const n[BRIDGE_COUNT] = {"none", "ftdi", "cp210x", "ch34x"};
    return k < BRIDGE_COUNT ? n[k] : "?";
}

uint32_t bridge_baud(void) { return s_baud; }

/* Standard rates within 3 %: a divisor rarely gives the exact rate. */
static uint32_t snap(uint32_t baud)
{
    static const uint32_t std[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 74880, 115200, 230400, 460800, 921600};
    for (unsigned i = 0; i < sizeof(std) / sizeof(std[0]); i++) {
        uint32_t d = baud > std[i] ? baud - std[i] : std[i] - baud;
        if (d * 100u <= std[i] * 3u) return std[i];
    }
    return baud;
}

static void set_baud(uint32_t baud)
{
    s_baud = snap(baud);
    serial_line_coding(s_baud);
}

static void set_lines(bool dtr, bool rts)
{
    if (dtr == s_dtr && rts == s_rts) return;
    s_dtr = dtr;
    s_rts = rts;
    serial_line_state(dtr, rts);
}

/* FT232R (bcdDevice 0x0600): a 14-bit divisor and a 3-bit fraction code, of a 3 MHz clock. */
static uint32_t ftdi_baud(uint16_t value, uint16_t index)
{
    static const uint8_t eighths[8] = {0, 4, 2, 1, 3, 5, 6, 7};  /* fraction code -> eighths */
    uint32_t div = value | ((uint32_t)(index & 0xFF) << 16);
    if (div == 0) return 3000000;
    if (div == 1) return 2000000;
    uint32_t div8 = (div & 0x3FFF) * 8u + eighths[(div >> 14) & 7];
    return div8 ? (uint32_t)((24000000ull + div8 / 2) / div8) : 0;
}

/* CH340: (divisor << 8) | 0x80 | prescaler, as the Linux ch341 driver encodes it. */
static uint32_t ch34x_baud(uint16_t div_ps)
{
    uint8_t a = (uint8_t)(div_ps >> 8), b = div_ps & 7;
    if (b == 7) return a == 0xF3 ? 921600 : a == 0xD9 ? 307200 : 0;
    static const uint32_t c[4] = {11719, 93750, 750000, 6000000};
    return a == 0 ? 0 : c[b & 3] / (256u - a);
}

/* ---- vendor control requests (all three chips; TinyUSB sends every vendor request here) */
static void act(tusb_control_request_t const *req)
{
    uint16_t v = req->wValue, ix = req->wIndex;
    switch (g_bridge) {
    case BRIDGE_FTDI:
        if (req->bRequest == 3) set_baud(ftdi_baud(v, ix));                       /* SET_BAUDRATE */
        else if (req->bRequest == 1) {                                             /* MODEM_CTRL */
            bool dtr = (v & 0x0100) ? (v & 1) != 0 : s_dtr, rts = (v & 0x0200) ? (v & 2) != 0 : s_rts;
            set_lines(dtr, rts);
        }
        break;
    case BRIDGE_CP210X:
        if (req->bRequest == 0x1E && req->wLength >= 4)                            /* SET_BAUDRATE */
            set_baud(s_ctrl[0] | (s_ctrl[1] << 8) | (s_ctrl[2] << 16) | ((uint32_t)s_ctrl[3] << 24));
        else if (req->bRequest == 0x07) {                                          /* SET_MHS */
            bool dtr = (v & 0x0100) ? (v & 1) != 0 : s_dtr, rts = (v & 0x0200) ? (v & 2) != 0 : s_rts;
            set_lines(dtr, rts);
        }
        break;
    case BRIDGE_CH34X:
        if (req->bRequest == 0xA1 && v != 0) set_baud(ch34x_baud(ix));             /* SERIAL_INIT */
        else if (req->bRequest == 0x9A && v == 0x1312) set_baud(ch34x_baud(ix));   /* divisor/prescaler */
        else if (req->bRequest == 0xA4) set_lines(!(v & (1u << 5)), !(v & (1u << 6))); /* inverted */
        break;
    default:
        break;
    }
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    if (g_bridge == BRIDGE_NONE) return false;
    bool in = req->bmRequestType_bit.direction == TUSB_DIR_IN;
    uint16_t n = req->wLength < sizeof(s_ctrl) ? req->wLength : (uint16_t)sizeof(s_ctrl);
    if (stage == CONTROL_STAGE_SETUP) {
        if (in) {
            memset(s_ctrl, 0, sizeof(s_ctrl));
            if (g_bridge == BRIDGE_CH34X && req->bRequest == 0x5F) s_ctrl[0] = 0x31;  /* chip version */
            if (g_bridge == BRIDGE_CP210X && req->bRequest == 0x08) s_ctrl[0] = (s_dtr ? 1 : 0) | (s_rts ? 2 : 0);
            return tud_control_xfer(rhport, req, s_ctrl, n);
        }
        if (req->wLength) return tud_control_xfer(rhport, req, s_ctrl, n);  /* act once the data is in */
        act(req);
        return tud_control_status(rhport, req);
    }
    if (stage == CONTROL_STAGE_DATA && !in) act(req);
    return true;
}

/* ---- the class driver: the chip's vendor interface and its endpoints */
static void drv_init(void) {}
static bool drv_deinit(void) { return true; }
static void drv_reset(uint8_t rhport)
{
    (void)rhport;
    s_open = false;
    s_in_busy = false;
    s_tx_head = s_tx_tail = 0;
}

static uint16_t drv_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
    if (g_bridge == BRIDGE_NONE || itf->bInterfaceClass != 0xFF) return 0;
    uint16_t len = (uint16_t)(sizeof(tusb_desc_interface_t) + itf->bNumEndpoints * sizeof(tusb_desc_endpoint_t));
    if (len > max_len) return 0;
    const uint8_t *p = tu_desc_next(itf);
    for (unsigned i = 0; i < itf->bNumEndpoints; i++, p = tu_desc_next(p)) {
        if (!usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *)p)) return 0;
    }
    s_open = true;
    usbd_edpt_xfer(rhport, EP_OUT, s_out_buf, EP_SIZE, false);
    return len;
}

static bool drv_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    (void)rhport; (void)stage; (void)req;
    return false;  /* the chips only use vendor requests (tud_vendor_control_xfer_cb) */
}

static void kick(void)
{
    if (!s_open || s_in_busy || !tud_ready()) return;
    uint32_t head = s_tx_head, tail = s_tx_tail;
    uint32_t hdr = g_bridge == BRIDGE_FTDI ? 2u : 0u;  /* FTDI: modem and line status first */
    if (head == tail) return;
    uint32_t n = 0;
    if (hdr) { s_in_buf[0] = 0x01; s_in_buf[1] = 0x60; } /* modem status, line status: idle */
    while (tail != head && hdr + n < EP_SIZE) {
        s_in_buf[hdr + n++] = s_tx[tail];
        tail = (tail + 1u) % sizeof(s_tx);
    }
    s_tx_tail = tail;
    s_in_busy = true;
    usbd_edpt_xfer(0, EP_IN, s_in_buf, (uint16_t)(hdr + n), false);
}

static bool drv_xfer(uint8_t rhport, uint8_t ep, xfer_result_t r, uint32_t n)
{
    (void)r;
    if (ep == EP_OUT) {
        if (n) serial_in(s_out_buf, n);
        usbd_edpt_xfer(rhport, EP_OUT, s_out_buf, EP_SIZE, false);
    } else if (ep == EP_IN) {
        s_in_busy = false;
        kick();
    }
    return true;
}

static usbd_class_driver_t const s_bridge_drv = {
    .name = "bridge",
    .init = drv_init,
    .deinit = drv_deinit,
    .reset = drv_reset,
    .open = drv_open,
    .control_xfer_cb = drv_control,
    .xfer_cb = drv_xfer,
    .sof = NULL,
};

usbd_class_driver_t const *bridge_driver(void) { return &s_bridge_drv; }

void bridge_task(void) { kick(); }

/* ---- descriptors */
uint8_t const *bridge_configuration(void)
{
    static uint8_t cfg[9 + 9 + 3 * 7];
    uint8_t eps = g_bridge == BRIDGE_CH34X ? 3 : 2;
    uint8_t sub = g_bridge == BRIDGE_FTDI ? 0xFF : g_bridge == BRIDGE_CH34X ? 0x01 : 0x00;
    uint8_t proto = g_bridge == BRIDGE_FTDI ? 0xFF : g_bridge == BRIDGE_CH34X ? 0x02 : 0x00;
    unsigned n = 9;
    const uint8_t itf[9] = {9, TUSB_DESC_INTERFACE, 0, 0, eps, 0xFF, sub, proto, 0};
    memcpy(cfg + n, itf, 9);
    n += 9;
    const uint8_t in[7] = {7, TUSB_DESC_ENDPOINT, EP_IN, TUSB_XFER_BULK, EP_SIZE, 0, 0};
    const uint8_t out[7] = {7, TUSB_DESC_ENDPOINT, EP_OUT, TUSB_XFER_BULK, EP_SIZE, 0, 0};
    memcpy(cfg + n, in, 7); n += 7;
    memcpy(cfg + n, out, 7); n += 7;
    if (eps == 3) {
        const uint8_t intr[7] = {7, TUSB_DESC_ENDPOINT, EP_INT, TUSB_XFER_INTERRUPT, 8, 0, 1};
        memcpy(cfg + n, intr, 7);
        n += 7;
    }
    const uint8_t hdr[9] = {9, TUSB_DESC_CONFIGURATION, (uint8_t)n, 0, 1, 1, 0, 0x80, 50};
    memcpy(cfg, hdr, 9);
    return cfg;
}

void bridge_ids(uint16_t *vid, uint16_t *pid, uint16_t *bcd)
{
    switch (g_bridge) {
    case BRIDGE_FTDI: *vid = 0x0403; *pid = 0x6001; *bcd = 0x0600; break;  /* FT232R */
    case BRIDGE_CP210X: *vid = 0x10C4; *pid = 0xEA60; *bcd = 0x0100; break; /* CP2102 */
    case BRIDGE_CH34X: *vid = 0x1A86; *pid = 0x7523; *bcd = 0x0264; break;  /* CH340 */
    default: break;
    }
}

/* ---- the bootloaders' output */
void serial_out(const uint8_t *p, uint32_t n)
{
    if (g_bridge == BRIDGE_NONE) {
        while (n) {
            uint32_t k = tud_cdc_write(p, n);
            tud_cdc_write_flush();
            p += k;
            n -= k;
            if (n) tud_task();
        }
        return;
    }
    while (n) {
        uint32_t next = (s_tx_head + 1u) % sizeof(s_tx);
        if (next == s_tx_tail) { /* full: let the host take some */
            kick();
            tud_task();
            if (!tud_ready()) return;
            continue;
        }
        s_tx[s_tx_head] = *p++;
        s_tx_head = next;
        n--;
    }
    kick();
}
