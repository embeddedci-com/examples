/* Hostile devices for testing a USB host's descriptor handling (modes many, big, badlen, overlong,
 * strings, nostr). Each configuration starts with a vendor interface 0 that the null driver
 * below claims together with everything after it, so TinyUSB's own SET_CONFIGURATION walk accepts
 * descriptors a host must survive. Nothing here may contain bLength 0: TinyUSB's walk would never
 * advance past it (the host side is what we test, not this stack). */
#include <string.h>

#include "device/usbd_pvt.h"
#include "tusb.h"
#include "usbnode.h"

#define HOSTILE_MAX 1280u
static uint8_t s_cfg[HOSTILE_MAX];

static unsigned put_itf(uint8_t *p, uint8_t number)
{
    const uint8_t itf[9] = {9, TUSB_DESC_INTERFACE, number, 0, 0, 0xFF, 0x00, 0x00, 0};
    memcpy(p, itf, 9);
    return 9;
}

uint8_t const *hostile_configuration(void)
{
    unsigned n = 9, itfs = 1;
    memset(s_cfg, 0, sizeof(s_cfg));
    n += put_itf(s_cfg + n, 0);
    switch (g_mode) {
    case MODE_MANY: /* 12 interfaces: more than a host lists */
        for (uint8_t i = 1; i < 12; i++) n += put_itf(s_cfg + n, i);
        itfs = 12;
        break;
    case MODE_BIG:  /* 700 bytes, MODE_HUGE 1100: class-specific descriptors of up to 255 bytes */
    case MODE_HUGE: {
        unsigned want = g_mode == MODE_BIG ? 700u : 1100u;
        while (n < want) {
            uint8_t *d = s_cfg + n;
            unsigned len = want - n > 255u ? 255u : want - n;
            if (len < 3) len = 3;
            d[0] = (uint8_t)len;
            d[1] = 0x24;
            for (unsigned i = 2; i < len; i++) d[i] = (uint8_t)(i & 0x7F) | 0x01;
            n += len;
        }
        break;
    }
    case MODE_BADLEN: /* bLength 1, then bytes that read as more 1-byte descriptors */
        for (unsigned i = 0; i < 8; i++) s_cfg[n++] = 0x01;
        break;
    case MODE_OVERLONG: /* a 9-byte descriptor that claims 200 */
        s_cfg[n] = 200; s_cfg[n + 1] = 0x24;
        for (unsigned i = 2; i < 9; i++) s_cfg[n + i] = 0x11;
        n += 9;
        break;
    default:
        break;
    }
    const uint8_t hdr[9] = {9, TUSB_DESC_CONFIGURATION, (uint8_t)n, (uint8_t)(n >> 8), (uint8_t)itfs, 1, 0, 0x80, 50};
    memcpy(s_cfg, hdr, 9);
    return s_cfg;
}

/* The null driver: takes interface 0 of a hostile configuration and everything after it. */
static void null_init(void) {}
static bool null_deinit(void) { return true; }
static void null_reset(uint8_t rhport) { (void)rhport; }
static uint16_t null_open(uint8_t rhport, tusb_desc_interface_t const *itf, uint16_t max_len)
{
    (void)rhport;
    if (!mode_is_hostile(g_mode) || itf->bInterfaceNumber != 0 || itf->bInterfaceClass != 0xFF) return 0;
    return max_len;
}
static bool null_control(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    (void)rhport; (void)stage; (void)req;
    return false;
}
static bool null_xfer(uint8_t rhport, uint8_t ep, xfer_result_t r, uint32_t n)
{
    (void)rhport; (void)ep; (void)r; (void)n;
    return true;
}

static usbd_class_driver_t const s_null = {
    .name = "null",
    .init = null_init,
    .deinit = null_deinit,
    .reset = null_reset,
    .open = null_open,
    .control_xfer_cb = null_control,
    .xfer_cb = null_xfer,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *count)
{
    *count = 1;
    return &s_null;
}
