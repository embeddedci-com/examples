/* USB-serial bridge chips in front of the serial modes (bridge.c). */
#ifndef BRIDGE_H
#define BRIDGE_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { BRIDGE_NONE = 0, BRIDGE_FTDI, BRIDGE_CP210X, BRIDGE_CH34X, BRIDGE_COUNT } bridge_kind_t;

extern bridge_kind_t g_bridge;
const char *bridge_name(bridge_kind_t k);
uint32_t bridge_baud(void);
uint8_t const *bridge_configuration(void);
void bridge_ids(uint16_t *vid, uint16_t *pid, uint16_t *bcd);
void bridge_task(void);

/* The serial path the bootloaders use, CDC or a bridge (main.c implements the in-direction). */
void serial_out(const uint8_t *p, uint32_t n);
void serial_in(const uint8_t *p, uint32_t n);
void serial_line_coding(uint32_t baud);
void serial_line_state(bool dtr, bool rts);

#endif
