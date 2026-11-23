#ifndef USBNODE_H
#define USBNODE_H

typedef enum {
    MODE_CDC, MODE_MSC, MODE_UF2, MODE_TEENSY,
    /* Serial bootloaders on the CDC port (loader.c): */
    MODE_OPTIBOOT, MODE_ATMEGABOOT, MODE_STK500V2, MODE_LEONARDO, MODE_CATERINA, MODE_ZERO, MODE_SAMBA,
    /* Hostile devices for the host's descriptor handling (usb_descriptors.c): */
    MODE_MANY,     /* 12 interfaces */
    MODE_BIG,      /* a configuration of about 700 bytes */
    MODE_HUGE,     /* about 1100 bytes: more than the pod reads (1024) */
    MODE_BADLEN,   /* a descriptor with bLength 1 */
    MODE_OVERLONG, /* a descriptor whose bLength runs past the end */
    MODE_STRINGS,  /* strings longer than 47 characters, not ASCII */
    MODE_NOSTR,    /* string requests stall */
    MODE_COUNT
} node_mode_t;

static inline int mode_is_hostile(node_mode_t m) { return m >= MODE_MANY; }
static inline int mode_is_loader(node_mode_t m) { return m >= MODE_OPTIBOOT && m <= MODE_SAMBA; }
static inline int mode_is_cdc(node_mode_t m) { return m == MODE_CDC || mode_is_loader(m); }

extern node_mode_t g_mode;
const char *mode_name(node_mode_t m);

#define TEENSY_REPORT_LEN 1088u /* 64-byte header + a 1024-byte block (Teensy 3.x/4.x) */

#endif
