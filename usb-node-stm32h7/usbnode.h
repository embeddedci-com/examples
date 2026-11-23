#ifndef USBNODE_H
#define USBNODE_H

typedef enum { MODE_CDC, MODE_MSC, MODE_UF2, MODE_TEENSY, MODE_COUNT } node_mode_t;

extern node_mode_t g_mode;
const char *mode_name(node_mode_t m);

#define TEENSY_REPORT_LEN 1088u /* 64-byte header + a 1024-byte block (Teensy 3.x/4.x) */

#endif
