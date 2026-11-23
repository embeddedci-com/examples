/* Host check: the BenchPod's own MSC flasher (benchpod-firmware xip_mod_msc_flash.c, file and uf2 modes)
   against the usb-node RAM drive (disk.c), on this computer: `make host-check`. Exits 1 on a mismatch. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "disk.h"
#include "msc_flash.h"

static const uint8_t *g_file; static uint32_t g_len;
static int rd(void *io, uint32_t lba, uint8_t *buf, uint32_t n) { (void)io;
    for (uint32_t i = 0; i < n; i++) { if (!disk_sector(lba + i)) return -1; memcpy(buf + i * 512, disk_sector(lba + i), 512); } return 0; }
static int wr(void *io, uint32_t lba, const uint8_t *buf, uint32_t n) { (void)io;
    for (uint32_t i = 0; i < n; i++) { if (!disk_sector(lba + i)) return -1; memcpy(disk_sector(lba + i), buf + i * 512, 512); disk_wrote(lba + i); } return 0; }
static int src(void *io, uint32_t off, uint8_t *buf, uint32_t n) { (void)io; if (off + n > g_len) return -1; memcpy(buf, g_file + off, n); return 0; }

static int fails;
#define EXPECT(c) do { if (!(c)) { printf("FAIL %s\n", #c); fails++; } } while (0)

static int run(int mode, const char *name, const uint8_t *f, uint32_t len) {
    static msc_flash_ctx_t c; memset(&c, 0, sizeof c);
    c.ops = (msc_flash_ops_t){ .read = rd, .write = wr, .src = src, .block_count = DISK_SECTORS, .block_size = 512 };
    c.mode = (uint8_t)mode; c.len = len; g_file = f; g_len = len;
    if (name) msc_flash_name83(name, c.name83);
    int rc; while ((rc = msc_flash_step(&c)) == MSC_FLASH_RUNNING) {}
    printf("-> %s (%d)\n", msc_flash_rc_str(rc), rc); return rc;
}
int main(void) {
    static uint8_t f[600 * 1024];
    srand(1); for (unsigned i = 0; i < sizeof f; i++) f[i] = (uint8_t)rand();
    disk_format(false);
    EXPECT(run(MSC_FLASH_FILE, "FIRMWARE.BIN", f, 61440) == MSC_FLASH_DONE);
    printf("expect FIRMWARE.BIN 61440 %08x\n", crc32_update(0, f, 61440));
    EXPECT(run(MSC_FLASH_FILE, "APP.BIN", f, 10000) == MSC_FLASH_DONE);
    EXPECT(run(MSC_FLASH_FILE, "APP.BIN", f + 5, 30000) == MSC_FLASH_DONE);
    printf("expect APP.BIN 30000 %08x\n", crc32_update(0, f + 5, 30000));
    EXPECT(run(MSC_FLASH_FILE, "BIG.BIN", f, 600 * 1024) == MSC_FLASH_E_FULL);
    disk_print_files();
    /* uf2 */
    disk_format(true);
    static uint8_t u[200 * 512]; uint32_t chunk = 256;
    for (uint32_t n = 0; n < 200; n++) { uint32_t *h = (uint32_t *)(u + n * 512); memset(h, 0, 512);
        h[0]=0x0A324655; h[1]=0x9E5D5157; h[2]=0x2000; h[3]=0x10000000+n*chunk; h[4]=chunk; h[5]=n; h[6]=200; h[7]=0xE48BFF56;
        memcpy(u + n * 512 + 32, f + n * chunk, chunk); h[127] = 0x0AB16F30; }
    EXPECT(run(MSC_FLASH_UF2, NULL, u, sizeof u) == MSC_FLASH_DONE);
    EXPECT(disk_uf2_complete());
    printf("complete %d expect crc %08x\n", disk_uf2_complete(), crc32_update(0, f, 200 * chunk));
    disk_print_uf2();
    disk_print_files();
    printf(fails ? "host-check FAILED\n" : "host-check ok\n");
    return fails ? 1 : 0;
}
