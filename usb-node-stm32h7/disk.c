/* FAT12 RAM drive: 960 sectors, 1 sector per cluster, 2 FATs of 3 sectors, 64 root entries.
 *
 * `files` lists the root directory with each file's size and CRC-32 (zlib's), so a test can check
 * what the pod copied. In UF2 mode the drive also holds INFO_UF2.TXT, like a UF2 bootloader, and
 * every sector the host writes is checked for a UF2 block: `uf2` reports the blocks found
 * anywhere on the drive, in block order, with the CRC-32 of their payloads. */
#include "disk.h"

#include <string.h>

#include "board.h"

#define RESERVED   1u
#define FATS       2u
#define FAT_SECS   3u
#define ROOT_ENTS  64u
#define ROOT_SECS  (ROOT_ENTS * 32u / DISK_SECTOR)
#define FAT_START  RESERVED
#define ROOT_START (FAT_START + FATS * FAT_SECS)
#define DATA_START (ROOT_START + ROOT_SECS)
#define CLUSTERS   (DISK_SECTORS - DATA_START)

#define UF2_MAGIC0 0x0A324655u
#define UF2_MAGIC1 0x9E5D5157u
#define UF2_MAGIC2 0x0AB16F30u
#define UF2_FLAG_FAMILY 0x00002000u
#define UF2_MAX_BLOCKS 1024u

static uint8_t s_disk[DISK_SECTORS][DISK_SECTOR] __attribute__((section(".disk"), aligned(32)));
static uint32_t s_writes;
static uint8_t s_uf2_seen[UF2_MAX_BLOCKS / 8];
static uint32_t s_uf2_seen_count, s_uf2_total;

static const char README[] =
    "usb-node-stm32h7: a RAM drive on a NUCLEO-H753ZI, for BenchPod USB tests.\r\n"
    "Its contents are lost on reset or a mode change.\r\n";
static const char INFO_UF2[] =
    "UF2 Bootloader v0.0.0 usb-node (emulated)\r\n"
    "Model: usb-node-stm32h7\r\n"
    "Board-ID: usb-node-stm32h7\r\n";

static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v) { wr16(p, (uint16_t)v); wr16(p + 2, (uint16_t)(v >> 16)); }

uint32_t crc32_update(uint32_t crc, const uint8_t *p, uint32_t n)
{
    crc = ~crc;
    while (n--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

uint8_t *disk_sector(uint32_t lba) { return lba < DISK_SECTORS ? s_disk[lba] : NULL; }
uint32_t disk_writes(void) { return s_writes; }

static uint16_t fat_get(uint32_t cl)
{
    const uint8_t *fat = s_disk[FAT_START];
    uint32_t off = cl + cl / 2u;
    uint16_t v = (uint16_t)(fat[off] | (fat[off + 1] << 8));
    return (cl & 1u) ? (uint16_t)(v >> 4) : (uint16_t)(v & 0x0FFFu);
}

static void fat_set(uint32_t cl, uint16_t val)
{
    for (unsigned f = 0; f < FATS; f++) {
        uint8_t *fat = s_disk[FAT_START + f * FAT_SECS];
        uint32_t off = cl + cl / 2u;
        if (cl & 1u) {
            fat[off] = (uint8_t)((fat[off] & 0x0F) | ((val << 4) & 0xF0));
            fat[off + 1] = (uint8_t)(val >> 4);
        } else {
            fat[off] = (uint8_t)val;
            fat[off + 1] = (uint8_t)((fat[off + 1] & 0xF0) | ((val >> 8) & 0x0F));
        }
    }
}

static void dir_entry(unsigned i, const char name[11], uint8_t attr, uint16_t cluster, uint32_t size)
{
    uint8_t *e = s_disk[ROOT_START] + i * 32u;
    memcpy(e, name, 11);
    e[11] = attr;
    wr16(e + 22, 0x0000);                                  /* 00:00:00 */
    wr16(e + 24, (uint16_t)(((2026 - 1980) << 9) | (11 << 5) | 22)); /* 2026-11-22 */
    wr16(e + 26, cluster);
    wr32(e + 28, size);
}

static void add_file(unsigned entry, const char name[11], uint16_t cluster, const char *text)
{
    uint32_t n = (uint32_t)strlen(text);
    memcpy(s_disk[DATA_START + cluster - 2u], text, n);
    fat_set(cluster, 0xFFF);
    dir_entry(entry, name, 0x01 /* read-only */, cluster, n);
}

void disk_format(bool uf2)
{
    memset(s_disk, 0, sizeof(s_disk));
    uint8_t *b = s_disk[0];
    b[0] = 0xEB; b[1] = 0x3C; b[2] = 0x90;
    memcpy(b + 3, "MSWIN4.1", 8);
    wr16(b + 11, DISK_SECTOR);
    b[13] = 1; /* sectors per cluster */
    wr16(b + 14, RESERVED);
    b[16] = FATS;
    wr16(b + 17, ROOT_ENTS);
    wr16(b + 19, DISK_SECTORS);
    b[21] = 0xF8;
    wr16(b + 22, FAT_SECS);
    wr16(b + 24, 32);
    wr16(b + 26, 2);
    b[36] = 0x80;
    b[38] = 0x29;
    wr32(b + 39, 0x55B0DE00u);
    memcpy(b + 43, uf2 ? "USBNODEUF2 " : "USBNODE    ", 11);
    memcpy(b + 54, "FAT12   ", 8);
    b[510] = 0x55; b[511] = 0xAA;

    fat_set(0, 0xFF8);
    fat_set(1, 0xFFF);
    dir_entry(0, uf2 ? "USBNODEUF2 " : "USBNODE    ", 0x08, 0, 0);
    add_file(1, "README  TXT", 2, README);
    if (uf2) {
        add_file(2, "INFO_UF2TXT", 3, INFO_UF2);
    }
    s_writes = 0;
    memset(s_uf2_seen, 0, sizeof(s_uf2_seen));
    s_uf2_seen_count = 0;
    s_uf2_total = 0;
}

static bool uf2_block(const uint8_t *s)
{
    return rd32(s) == UF2_MAGIC0 && rd32(s + 4) == UF2_MAGIC1 && rd32(s + 508) == UF2_MAGIC2;
}

void disk_wrote(uint32_t lba)
{
    s_writes++;
    const uint8_t *s = disk_sector(lba);
    if (!s || !uf2_block(s)) {
        return;
    }
    uint32_t no = rd32(s + 20), total = rd32(s + 24);
    if (no >= UF2_MAX_BLOCKS || total == 0 || total > UF2_MAX_BLOCKS) {
        return;
    }
    if (total != s_uf2_total) { /* a different file: start counting again */
        memset(s_uf2_seen, 0, sizeof(s_uf2_seen));
        s_uf2_seen_count = 0;
        s_uf2_total = total;
    }
    if (!(s_uf2_seen[no / 8] & (1u << (no % 8)))) {
        s_uf2_seen[no / 8] |= (uint8_t)(1u << (no % 8));
        s_uf2_seen_count++;
    }
}

bool disk_uf2_complete(void) { return s_uf2_total != 0 && s_uf2_seen_count == s_uf2_total; }

void disk_print_files(void)
{
    unsigned count = 0;
    for (unsigned i = 0; i < ROOT_ENTS; i++) {
        const uint8_t *e = s_disk[ROOT_START] + i * 32u;
        if (e[0] == 0x00) {
            break;
        }
        if (e[0] == 0xE5 || e[11] == 0x0F || (e[11] & 0x18)) {
            continue; /* deleted, long-name part, volume label or directory */
        }
        char name[13];
        unsigned n = 0;
        for (unsigned k = 0; k < 8 && e[k] != ' '; k++) name[n++] = (char)e[k];
        if (e[8] != ' ') {
            name[n++] = '.';
            for (unsigned k = 8; k < 11 && e[k] != ' '; k++) name[n++] = (char)e[k];
        }
        name[n] = 0;
        uint32_t size = rd32(e + 28), left = size, crc = 0;
        uint32_t cl = rd16(e + 26);
        bool bad = false;
        while (left > 0) {
            if (cl < 2 || cl >= CLUSTERS + 2u) {
                bad = true;
                break;
            }
            uint32_t take = left < DISK_SECTOR ? left : DISK_SECTOR;
            crc = crc32_update(crc, s_disk[DATA_START + cl - 2u], take);
            left -= take;
            cl = fat_get(cl);
        }
        con_printf("FILE %s %lu %08lx%s\n", name, (unsigned long)size, (unsigned long)crc, bad ? " broken-chain" : "");
        count++;
    }
    con_printf("FILES %u\n", count);
}

void disk_print_uf2(void)
{
    static uint16_t lba_of[UF2_MAX_BLOCKS];
    memset(lba_of, 0, sizeof(lba_of));
    uint32_t total = 0, found = 0, family = 0, lo = 0xFFFFFFFFu, hi = 0, crc = 0, payload = 0;
    for (uint32_t lba = DATA_START; lba < DISK_SECTORS; lba++) {
        const uint8_t *s = s_disk[lba];
        if (!uf2_block(s)) continue;
        uint32_t no = rd32(s + 20);
        if (no >= UF2_MAX_BLOCKS || lba_of[no]) continue;
        lba_of[no] = (uint16_t)lba;
        total = rd32(s + 24);
        if (rd32(s + 8) & UF2_FLAG_FAMILY) family = rd32(s + 28);
        found++;
    }
    bool complete = total != 0 && total <= UF2_MAX_BLOCKS;
    for (uint32_t no = 0; complete && no < total; no++) {
        if (!lba_of[no]) {
            complete = false;
            break;
        }
        const uint8_t *s = s_disk[lba_of[no]];
        uint32_t addr = rd32(s + 12), size = rd32(s + 16);
        if (size > 476) size = 476;
        if (addr < lo) lo = addr;
        if (addr + size > hi) hi = addr + size;
        crc = crc32_update(crc, s + 32, size);
        payload += size;
    }
    con_printf("UF2 blocks %lu of %lu complete %d family %08lx addr %08lx-%08lx payload %lu crc %08lx\n",
               (unsigned long)found, (unsigned long)total, complete ? 1 : 0, (unsigned long)family,
               (unsigned long)(complete ? lo : 0), (unsigned long)(complete ? hi : 0), (unsigned long)payload,
               (unsigned long)(complete ? crc : 0));
}
