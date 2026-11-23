/* A FAT12 RAM drive in AXI SRAM, for the MSC modes. */
#ifndef DISK_H
#define DISK_H
#include <stdbool.h>
#include <stdint.h>

#define DISK_SECTOR  512u
#define DISK_SECTORS 960u /* 480 KB */

void disk_format(bool uf2);
uint8_t *disk_sector(uint32_t lba);
void disk_wrote(uint32_t lba); /* after the host wrote this sector */
uint32_t disk_writes(void);
bool disk_uf2_complete(void);  /* every block of a UF2 file arrived since the last format */
void disk_print_files(void);
void disk_print_uf2(void);
uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n);

#endif
