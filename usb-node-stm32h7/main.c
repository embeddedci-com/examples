/* usb-node-stm32h7: a USB device under test for the BenchPod's USB host, on a NUCLEO-H753ZI.
 *
 * The board plays one USB target at a time on CN13 (OTG_FS, full speed):
 *   cdc     a USB serial port (CDC ACM) that echoes what it receives
 *   msc     a FAT12 RAM drive, like a drag-and-drop loader (DAPLink)
 *   uf2     the same drive with INFO_UF2.TXT, taking UF2 blocks like a UF2 bootloader
 *   teensy  the Teensy loader (HalfKay, 16C0:0478) into a RAM image
 * and the `dfu` command starts ST's ROM bootloader (USB DfuSe), as BOOT0 would.
 *
 * The console (ST-LINK virtual COM port, 115200 8N1) sets the mode and reports what the host did,
 * one line per answer; lines starting with EV are events. The mode survives a reset, not a power
 * cycle (default cdc). Holding B1 at reset starts the ROM bootloader. */
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "disk.h"
#include "loader.h"
#include "esp_rom.h"
#include "tusb.h"
#include "usbnode.h"

#ifndef USBNODE_VERSION
#define USBNODE_VERSION "0.1.0"
#endif

#define MODE_MAGIC 0x55B0DE00u
static uint32_t s_mode_magic __attribute__((section(".noinit")));
static uint32_t s_mode_saved __attribute__((section(".noinit")));
static uint32_t s_keep_image __attribute__((section(".noinit"))); /* MODE_MAGIC: a touch or leave reset */
node_mode_t g_mode;

static const char *const k_mode_names[MODE_COUNT] = {"cdc", "msc", "uf2", "teensy", "optiboot", "atmegaboot",
                                                      "stk500v2", "leonardo", "caterina", "zero", "samba", "esp32s3",
                                                      "many", "big", "huge",
                                                      "badlen", "overlong", "strings", "nostr"};
const char *mode_name(node_mode_t m) { return m < MODE_COUNT ? k_mode_names[m] : "?"; }

static struct {
    uint32_t mounts, unmounts, suspends, resumes;
    uint32_t cdc_rx, cdc_baud;
    uint8_t cdc_bits, cdc_parity, cdc_stop;
    bool cdc_dtr, cdc_rts;
    uint32_t teensy_blocks, teensy_hi, teensy_reboots;
    bool uf2_reported, msc_ejected;
    uint32_t last_usb_ms;
} st;

/* ---- TinyUSB device callbacks */
void tud_mount_cb(void)
{
    st.mounts++;
    led_green(true);
    con_printf("EV mount %lu\n", (unsigned long)st.mounts);
}
void tud_umount_cb(void)
{
    st.unmounts++;
    led_green(false);
    con_write("EV unmount\n");
}
void tud_suspend_cb(bool remote_wakeup_en)
{
    (void)remote_wakeup_en;
    st.suspends++;
    con_write("EV suspend\n");
}
void tud_resume_cb(void)
{
    st.resumes++;
    con_write("EV resume\n");
}

/* CDC: echo, and report line coding and control lines (the 1200 baud touch of Arduino tools). */
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *lc)
{
    (void)itf;
    st.cdc_baud = lc->bit_rate;
    st.cdc_bits = lc->data_bits;
    st.cdc_parity = lc->parity;
    st.cdc_stop = lc->stop_bits;
    con_printf("EV cdc line %lu %u%c%s\n", (unsigned long)lc->bit_rate, lc->data_bits, "NOEMS"[lc->parity % 5],
               lc->stop_bits == 2 ? "2" : (lc->stop_bits == 1 ? "1.5" : "1"));
}
uint32_t st_cdc_baud(void) { return st.cdc_baud; }

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    (void)itf;
    if (g_mode == MODE_ESP32S3) esp_rom_line_state(dtr, rts);
    else if (mode_is_loader(g_mode)) loader_line_state(dtr, st.cdc_dtr, st.cdc_baud);
    if (st.cdc_dtr && !dtr && st.cdc_baud == 1200 && !mode_is_loader(g_mode)) {
        con_write("EV cdc touch1200\n");
    }
    st.cdc_dtr = dtr;
    st.cdc_rts = rts;
    con_printf("EV cdc dtr %d rts %d\n", dtr, rts);
}
static void cdc_task(void)
{
    if (!mode_is_cdc(g_mode) || !tud_cdc_available()) return;
    uint8_t buf[64];
    uint32_t n = tud_cdc_read(buf, sizeof(buf));
    st.cdc_rx += n;
    st.last_usb_ms = millis();
    if (g_mode == MODE_ESP32S3) {
        esp_rom_rx(buf, n);
        return;
    }
    if (mode_is_loader(g_mode)) {
        loader_rx(buf, n);
        return;
    }
    tud_cdc_write(buf, n);
    tud_cdc_write_flush();
}

/* MSC: the RAM drive. */
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id, "EmbedCI ", 8);
    memcpy(product_id, g_mode == MODE_UF2 ? "usb-node UF2    " : "usb-node drive  ", 16);
    memcpy(product_rev, "0.1 ", 4);
}
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return !st.msc_ejected;
}
void tud_msc_capacity_cb(uint8_t lun, uint32_t *count, uint16_t *size)
{
    (void)lun;
    *count = DISK_SECTORS;
    *size = DISK_SECTOR;
}
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, bool start, bool load_eject)
{
    (void)lun;
    (void)power_condition;
    if (load_eject && !start) {
        st.msc_ejected = true;
        con_write("EV msc eject\n");
    }
    return true;
}
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (lba >= DISK_SECTORS || (uint64_t)lba * DISK_SECTOR + offset + bufsize > (uint64_t)DISK_SECTORS * DISK_SECTOR)
        return -1;
    memcpy(buffer, disk_sector(lba) + offset, bufsize);
    st.last_usb_ms = millis();
    return (int32_t)bufsize;
}
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun;
    if (lba >= DISK_SECTORS || (uint64_t)lba * DISK_SECTOR + offset + bufsize > (uint64_t)DISK_SECTORS * DISK_SECTOR)
        return -1;
    memcpy(disk_sector(lba) + offset, buffer, bufsize);
    uint32_t end = offset + bufsize;
    for (uint32_t done = DISK_SECTOR; done <= end; done += DISK_SECTOR) {
        if (done > offset) disk_wrote(lba + done / DISK_SECTOR - 1u);
    }
    st.last_usb_ms = millis();
    return (int32_t)bufsize;
}
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    (void)buffer;
    (void)bufsize;
    (void)scsi_cmd;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}

/* HID: the Teensy loader. Each SET_REPORT is a 3-byte address, 61 bytes of header and a block;
   address 0xFFFFFF is the reboot request. The image lives in the RAM drive's memory. */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t *buffer,
                               uint16_t reqlen)
{
    (void)instance;
    (void)report_id;
    (void)type;
    (void)buffer;
    (void)reqlen;
    return 0;
}
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t type, uint8_t const *buf,
                           uint16_t len)
{
    (void)instance;
    (void)report_id;
    (void)type;
    if (g_mode != MODE_TEENSY || len < 64u) return;
    uint32_t addr = buf[0] | (buf[1] << 8) | ((uint32_t)buf[2] << 16);
    uint32_t blk = len - 64u;
    st.last_usb_ms = millis();
    if (addr == 0xFFFFFFu) {
        st.teensy_reboots++;
        con_printf("EV teensy reboot blocks %lu\n", (unsigned long)st.teensy_blocks);
        return;
    }
    uint8_t *img = disk_sector(0);
    if (addr == 0) { /* HalfKay erases the whole flash on the first block */
        memset(img, 0xFF, DISK_SECTORS * DISK_SECTOR);
        st.teensy_blocks = 0;
        st.teensy_hi = 0;
    }
    if (addr + blk > DISK_SECTORS * DISK_SECTOR) {
        con_printf("EV teensy block 0x%06lx beyond the %u KB image\n", (unsigned long)addr, DISK_SECTORS / 2u);
        return;
    }
    memcpy(img + addr, buf + 64, blk);
    st.teensy_blocks++;
    if (addr + blk > st.teensy_hi) st.teensy_hi = addr + blk;
}

/* ---- console */
static void print_status(void)
{
    unsigned trim;
    bool synced;
    crs_state(&trim, &synced);
    con_printf("STATUS mode %s vbus %d mounted %d suspended %d mounts %lu unmounts %lu crs_sync %d trim %u\n",
               mode_name(g_mode), vbus_present(), tud_mounted(), tud_suspended(), (unsigned long)st.mounts,
               (unsigned long)st.unmounts, synced, trim);
    if (mode_is_loader(g_mode)) {
        const loader_stats_t *l = loader_stats();
        con_printf("LOADER syncs %lu pages %lu bytes %lu hi 0x%lx erases %lu leaves %lu errors %lu dropped %lu\n",
                   (unsigned long)l->syncs, (unsigned long)l->pages, (unsigned long)l->bytes_written,
                   (unsigned long)l->hi, (unsigned long)l->erases, (unsigned long)l->leaves,
                   (unsigned long)l->errors, (unsigned long)l->dropped);
    }
    if (g_mode == MODE_ESP32S3) {
        const esp_rom_stats_t *e = esp_rom_stats();
        con_printf("ESP download %d resets %lu syncs %lu begins %lu blocks %lu resends %lu bad_checksums %lu md5s %lu "
                   "ends %lu errors %lu ignored %lu\n",
                   esp_rom_in_download(), (unsigned long)e->resets, (unsigned long)e->syncs,
                   (unsigned long)e->begins, (unsigned long)e->blocks, (unsigned long)e->resends,
                   (unsigned long)e->bad_checksums, (unsigned long)e->md5s, (unsigned long)e->ends,
                   (unsigned long)e->errors, (unsigned long)e->ignored);
    }
    if (mode_is_cdc(g_mode))
        con_printf("CDC rx %lu baud %lu dtr %d rts %d\n", (unsigned long)st.cdc_rx, (unsigned long)st.cdc_baud,
                   st.cdc_dtr, st.cdc_rts);
    if (g_mode == MODE_MSC || g_mode == MODE_UF2)
        con_printf("MSC writes %lu ejected %d\n", (unsigned long)disk_writes(), st.msc_ejected);
    if (g_mode == MODE_TEENSY)
        con_printf("TEENSY blocks %lu hi 0x%06lx reboots %lu\n", (unsigned long)st.teensy_blocks,
                   (unsigned long)st.teensy_hi, (unsigned long)st.teensy_reboots);
}

static void help(void)
{
    con_write("commands: status | mode cdc|msc|uf2|teensy|optiboot|atmegaboot|stk500v2|leonardo|caterina|zero|samba|esp32s3|many|big|huge|badlen|overlong|strings|nostr | detach | attach | replug [ms] | files | uf2 |\n"
              "          teensy [len] | image <addr> <len> | format | dfu | reset | version\n");
}

static void handle(char *line)
{
    char *arg = strchr(line, ' ');
    if (arg) {
        *arg++ = 0;
        while (*arg == ' ') arg++;
    }
    if (!strcmp(line, "status")) {
        print_status();
    } else if (!strcmp(line, "version")) {
        con_printf("VERSION usb-node-stm32h7 %s\n", USBNODE_VERSION);
    } else if (!strcmp(line, "mode")) {
        if (!arg || !*arg) {
            con_printf("MODE %s\n", mode_name(g_mode));
            return;
        }
        for (int m = 0; m < MODE_COUNT; m++) {
            if (!strcmp(arg, k_mode_names[m])) {
                s_mode_magic = MODE_MAGIC;
                s_mode_saved = (uint32_t)m;
                s_keep_image = 0; /* a mode set here starts with erased flash */
                con_printf("OK mode %s, resetting\n", k_mode_names[m]);
                tud_disconnect();
                board_reset();
            }
        }
        con_printf("ERR unknown mode %s\n", arg);
    } else if (!strcmp(line, "detach")) {
        tud_disconnect();
        con_write("OK detached\n");
    } else if (!strcmp(line, "attach")) {
        tud_connect();
        con_write("OK attached\n");
    } else if (!strcmp(line, "replug")) {
        uint32_t ms = arg && *arg ? (uint32_t)strtoul(arg, NULL, 0) : 500u;
        tud_disconnect();
        delay_ms(ms);
        tud_connect();
        con_printf("OK replugged after %lu ms\n", (unsigned long)ms);
    } else if (!strcmp(line, "files")) {
        disk_print_files();
    } else if (!strcmp(line, "uf2")) {
        disk_print_uf2();
    } else if (!strcmp(line, "teensy")) {
        uint32_t len = arg && *arg ? (uint32_t)strtoul(arg, NULL, 0) : st.teensy_hi;
        if (len > DISK_SECTORS * DISK_SECTOR) len = DISK_SECTORS * DISK_SECTOR;
        con_printf("TEENSY blocks %lu hi 0x%06lx reboots %lu len %lu crc %08lx\n", (unsigned long)st.teensy_blocks,
                   (unsigned long)st.teensy_hi, (unsigned long)st.teensy_reboots, (unsigned long)len,
                   (unsigned long)crc32_update(0, disk_sector(0), len));
    } else if (!strcmp(line, "debug")) {
        extern bool s_debug;
        s_debug = arg && *arg == '1';
        con_printf("OK debug %d\n", s_debug);
    } else if (!strcmp(line, "image")) {
        char *len = arg ? strchr(arg, ' ') : NULL;
        if (!arg || !len) {
            con_write("ERR image <addr> <len>\n");
            return;
        }
        loader_print_image((uint32_t)strtoul(arg, NULL, 0), (uint32_t)strtoul(len + 1, NULL, 0));
    } else if (!strcmp(line, "format")) {
        disk_format(g_mode == MODE_UF2);
        st.uf2_reported = false;
        st.msc_ejected = false;
        con_write("OK formatted\n");
    } else if (!strcmp(line, "dfu")) {
        con_write("OK ROM bootloader\n");
        board_enter_rom_dfu();
    } else if (!strcmp(line, "reset")) {
        con_write("OK resetting\n");
        board_reset();
    } else if (!strcmp(line, "help") || !strcmp(line, "?")) {
        help();
    } else if (*line) {
        con_printf("ERR unknown command %s\n", line);
    }
}

static void console_task(void)
{
    static char line[96];
    static unsigned n;
    int c;
    while ((c = con_getc()) >= 0) {
        if (c == '\r' || c == '\n') {
            line[n] = 0;
            n = 0;
            handle(line);
        } else if (n + 1 < sizeof(line)) {
            line[n++] = (char)c;
        }
    }
}

static void event_task(void)
{
    if (g_mode == MODE_UF2 && !st.uf2_reported && disk_uf2_complete()) {
        st.uf2_reported = true;
        con_write("EV uf2 complete\n");
    }
    led_yellow(millis() - st.last_usb_ms < 50u);
    node_mode_t to;
    if (loader_reset_due(&to)) { /* the touch, or a bootloader starting its sketch */
        con_printf("EV reset into %s\n", mode_name(to));
        s_mode_magic = MODE_MAGIC;
        s_mode_saved = (uint32_t)to;
        s_keep_image = MODE_MAGIC;
        tud_disconnect();
        board_reset();
    }
}

int main(void)
{
    board_init();
    g_mode = (s_mode_magic == MODE_MAGIC && s_mode_saved < MODE_COUNT) ? (node_mode_t)s_mode_saved : MODE_CDC;
    con_printf("\nusb-node-stm32h7 %s mode %s\n", USBNODE_VERSION, mode_name(g_mode));
    if (button_pressed()) {
        con_write("B1 held: ROM bootloader\n");
        board_enter_rom_dfu();
    }
    if (g_mode == MODE_MSC || g_mode == MODE_UF2) disk_format(g_mode == MODE_UF2);
    if (mode_is_loader(g_mode) && s_keep_image != MODE_MAGIC) loader_erase_image();
    s_keep_image = 0;

    usb_hw_init();
    tud_configure_dwc2_t cfg = CFG_TUD_CONFIGURE_DWC2_DEFAULT;
    /* No VBUS sensing: the pull-up goes on at boot, whatever PA9 reads (status still reports it).
       A host that is not there yet simply sees the device when it arrives. */
    cfg.vbus_sensing = 0;
    tud_configure(0, TUD_CFGID_DWC2, &cfg);
    tusb_rhport_init_t init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL};
    if (!tusb_rhport_init(0, &init)) {
        led_red(true);
        con_write("ERR USB init failed\n");
    }
    con_write("APP_OK\n");

    for (;;) {
        tud_task();
        cdc_task();
        console_task();
        event_task();
    }
}
