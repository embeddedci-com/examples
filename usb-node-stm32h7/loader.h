/* Serial bootloaders on the CDC port (loader.c). */
#ifndef LOADER_H
#define LOADER_H
#include <stdbool.h>
#include <stdint.h>

#include "usbnode.h"

typedef struct {
    uint32_t syncs, pages, bytes_written, hi, erases, leaves, errors, dropped;
} loader_stats_t;

void loader_rx(const uint8_t *p, uint32_t n);
void loader_line_state(bool dtr, bool was_dtr, uint32_t baud);
bool loader_reset_due(node_mode_t *to);  /* a touch or a leave wants the board reset into `to` */
void loader_erase_image(void);
void loader_print_image(uint32_t addr, uint32_t len);
const loader_stats_t *loader_stats(void);

uint32_t st_cdc_baud(void);  /* main.c: the host's line coding */

#endif
