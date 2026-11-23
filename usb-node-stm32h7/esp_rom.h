/* An ESP32-S3's ROM loader on its USB-Serial/JTAG port (esp_rom.c). */
#ifndef ESP_ROM_H
#define ESP_ROM_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t resets, syncs, begins, blocks, resends, bad_checksums, md5s, ends, baud_changes, errors, ignored, wrong_baud;
} esp_rom_stats_t;

void esp_rom_rx(const uint8_t *p, uint32_t n);
void esp_rom_line_state(bool dtr, bool rts);
void esp_rom_boot(bool download);  /* at boot: the sketch (false) or download mode */
bool esp_rom_in_download(void);
const esp_rom_stats_t *esp_rom_stats(void);

#endif
