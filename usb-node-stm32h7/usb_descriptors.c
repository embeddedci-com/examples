/* USB descriptors, one set per mode. The mode is chosen at boot (see main.c). */
#include <string.h>

#include "stm32h7xx.h"
#include "tusb.h"
#include "usbnode.h"

#define VID_TEST   0xCAFEu /* TinyUSB's test VID: bench use only */
#define TEENSY_VID 0x16C0u
#define TEENSY_PID 0x0478u /* PJRC HalfKay bootloader */

enum { STR_LANG, STR_MANUFACTURER, STR_PRODUCT, STR_SERIAL, STR_CDC, STR_MSC };

static tusb_desc_device_t s_dev = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bcdDevice = 0x0100,
    .iManufacturer = STR_MANUFACTURER,
    .iProduct = STR_PRODUCT,
    .iSerialNumber = STR_SERIAL,
    .bNumConfigurations = 1,
};

uint8_t const *tud_descriptor_device_cb(void)
{
    s_dev.bDeviceClass = 0;
    s_dev.bDeviceSubClass = 0;
    s_dev.bDeviceProtocol = 0;
    s_dev.idVendor = VID_TEST;
    s_dev.iManufacturer = STR_MANUFACTURER;
    s_dev.iProduct = STR_PRODUCT;
    s_dev.iSerialNumber = STR_SERIAL;
    switch (g_mode) {
    case MODE_CDC: /* CDC uses an interface association */
        s_dev.bDeviceClass = TUSB_CLASS_MISC;
        s_dev.bDeviceSubClass = MISC_SUBCLASS_COMMON;
        s_dev.bDeviceProtocol = MISC_PROTOCOL_IAD;
        s_dev.idProduct = 0x4001;
        break;
    case MODE_MSC: s_dev.idProduct = 0x4002; break;
    case MODE_UF2: s_dev.idProduct = 0x4003; break;
    case MODE_TEENSY: /* HalfKay has no strings */
        s_dev.idVendor = TEENSY_VID;
        s_dev.idProduct = TEENSY_PID;
        s_dev.bcdDevice = 0x0280;
        s_dev.iManufacturer = s_dev.iProduct = s_dev.iSerialNumber = 0;
        break;
    default: break;
    }
    return (uint8_t const *)&s_dev;
}

/* ---- HID report descriptor of the Teensy loader: one output report of a block plus its header */
static uint8_t const s_teensy_report[] = {
    0x06, 0x9C, 0xFF,       /* usage page 0xFF9C (PJRC) */
    0x09, 0x24,             /* usage 0x24 */
    0xA1, 0x01,             /* collection (application) */
    0x0A, 0x19, 0x00,       /*   usage 0x19 */
    0x75, 0x08,             /*   report size 8 */
    0x15, 0x00,             /*   logical minimum 0 */
    0x26, 0xFF, 0x00,       /*   logical maximum 255 */
    0x96, (uint8_t)(TEENSY_REPORT_LEN & 0xFF), (uint8_t)(TEENSY_REPORT_LEN >> 8), /* report count */
    0x91, 0x02,             /*   output (data, variable, absolute) */
    0xC0,                   /* end collection */
};

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return s_teensy_report;
}

/* ---- configuration descriptors */
#define CFG_CDC_LEN    (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
#define CFG_MSC_LEN    (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)
#define CFG_TEENSY_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static uint8_t const s_cfg_cdc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, CFG_CDC_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(0, STR_CDC, 0x81, 8, 0x02, 0x82, 64),
};
static uint8_t const s_cfg_msc[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CFG_MSC_LEN, 0x00, 100),
    TUD_MSC_DESCRIPTOR(0, STR_MSC, 0x01, 0x81, 64),
};
static uint8_t const s_cfg_teensy[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, CFG_TEENSY_LEN, 0x00, 100),
    TUD_HID_DESCRIPTOR(0, 0, HID_ITF_PROTOCOL_NONE, sizeof(s_teensy_report), 0x81, 64, 1),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    switch (g_mode) {
    case MODE_CDC: return s_cfg_cdc;
    case MODE_MSC:
    case MODE_UF2: return s_cfg_msc;
    case MODE_TEENSY: return s_cfg_teensy;
    default: return s_cfg_cdc;
    }
}

/* ---- strings */
static uint16_t s_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    char buf[32];
    const char *s = NULL;
    switch (index) {
    case STR_LANG:
        s_str[1] = 0x0409;
        s_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | 4);
        return s_str;
    case STR_MANUFACTURER: s = "EmbeddedCI"; break;
    case STR_PRODUCT:
        strcpy(buf, "usb-node ");
        strcat(buf, mode_name(g_mode));
        s = buf;
        break;
    case STR_SERIAL: {
        static const char hex[] = "0123456789ABCDEF";
        const uint32_t *uid = (const uint32_t *)UID_BASE;
        for (unsigned w = 0; w < 3; w++) {
            for (unsigned k = 0; k < 8; k++) buf[w * 8 + k] = hex[(uid[w] >> (28 - 4 * k)) & 0xF];
        }
        buf[24] = 0;
        s = buf;
        break;
    }
    case STR_CDC: s = "usb-node serial"; break;
    case STR_MSC: s = "usb-node drive"; break;
    default: return NULL;
    }
    size_t n = strlen(s);
    if (n > 32) n = 32;
    for (size_t i = 0; i < n; i++) s_str[1 + i] = (uint8_t)s[i];
    s_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * n + 2));
    return s_str;
}
