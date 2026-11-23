# usb-node-stm32h7

A USB device under test for the BenchPod's USB host, on a NUCLEO-H753ZI. The board plays one USB
target at a time on its user USB port (CN13, full speed), so one board covers the pod's USB
features end to end:

| Mode | Plays | Pod feature it tests |
|---|---|---|
| `cdc` | a USB serial port (CDC ACM) that echoes what it receives, and reports baud and DTR/RTS | `usb_host` enumeration, serial ports |
| `msc` | a FAT12 RAM drive (480 KB), like a drag-and-drop loader (DAPLink) | `usb_flash` `file` |
| `uf2` | the same drive with `INFO_UF2.TXT`, checking UF2 blocks like a UF2 bootloader | `usb_flash` `uf2` |
| `teensy` | the Teensy loader (HalfKay, VID/PID 16C0:0478) into a RAM image | `usb_flash` `teensy` |
| `dfu` command | ST's ROM bootloader (USB DfuSe), as BOOT0 high at reset would | `usb_flash` `dfu` |
| `optiboot` | an Uno's bootloader: STK500v1 at 115200 (2341:0043) | `usb_flash` `avr` |
| `atmegaboot` | an old Nano's: STK500v1 at 57600, found by probing | `usb_flash` `avr` |
| `stk500v2` | a Mega 2560's: STK500v2 at 115200, past 128 KB (2341:0042) | `usb_flash` `avr` |
| `leonardo` | a 32u4 sketch (2341:8036): the 1200 baud touch resets it into `caterina` | `usb_flash` `avr`, the touch |
| `caterina` | Caterina, AVR109 (2341:0036); `E` starts the sketch again | `usb_flash` `avr` |
| `zero` | a SAMD21 sketch (2341:804D): the touch resets it into `samba` | `usb_flash` `samd`, the touch |
| `samba` | SAM-BA with Arduino's X/Y commands, SAMD21G18 (2341:004D) | `usb_flash` `samd` |
| `esp32s3` | an ESP32-S3 on its USB-Serial/JTAG port (303A:1001): reset into the ROM loader by DTR/RTS | `usb_flash` `esp` |
| `esp32` | the original ESP32, for use behind a bridge chip (a dev kit) | `usb_flash` `esp`, baud change |
| `many`, `big`, `huge`, `badlen`, `overlong`, `strings`, `nostr` | hostile devices: 12 interfaces, 700- and 1100-byte configurations, a 1-byte and an over-long descriptor, 60-character non-ASCII strings, stalling string requests | `usb_host` robustness |

`bridge ftdi|cp210x|ch34x` puts a USB-serial chip in front of the serial modes (an FT232R
0403:6001, a CP2102 10C4:EA60, a CH340 1A86:7523), the way boards without native USB have one:
the chip's vendor requests set the baud rate and DTR/RTS, and an AVR bootloader or the ESP32 ROM
only understands bytes at its own rate. `bridge none` is CDC again.

The serial modes use the USB IDs of the boards they stand for (Arduino's VID 0x2341, Espressif's
0x303A, the chip makers' for the bridges), because a host picks its flashing steps by them. They
are meant for a test bench, not to be shipped. The other modes use TinyUSB's test VID 0xCAFE.

The console reports what the host did (files on the drive with their CRC-32, UF2 blocks, Teensy
blocks, the bootloaders' flash), so a test compares it with what it sent.

## Bench

```
pod USB-C ── USB-C male breakout ──wires── USB-A female breakout ── USB-A to micro-USB cable ── CN13
                  │ VBUS/GND
               5 V supply (powers the pod)
```

| USB-C male (pod) | USB-A female | Also |
|---|---|---|
| D+ | D+ | |
| D- | D- | |
| GND | GND | 5 V supply GND |
| VBUS | not connected | 5 V supply + (powers the pod only) |
| CC1, CC2 | not connected | the pod has its own 5.1 kOhm pull-downs |

Keep D+/D- short. A USB-C charger as the 5 V supply only turns on when it sees the CC pull-downs:
pass CC through to the pod, or use a USB-A charger or a bench supply. On a pod with a USB-A host
port, plug the cable into that port instead and set `USB_HOST_PORT=usb_a`.

The Nucleo itself stays on its ST-LINK USB (CN1) to the computer: power (the power jumper on
5V_STLK, the default), flashing and the console. CN13 needs only D+, D- and GND: the firmware does
not sense VBUS (it does not reach PA9 on this board), it connects at boot.

While the pod hosts, its USB-C port belongs to the board: no USB console or USB network to the
computer. Reach the pod over Ethernet (or the cloud); the pod refuses `usb_host` over its own USB.

The pod needs a firmware built with `make USB_HOST=1` (development builds only for now); on a pod
whose W25Q cannot be memory-mapped (before digital v3r4) also `XIP_RAM=1`, which the flash modes
need.

## Build and flash

```bash
make deps    # once: ST's CMSIS headers and TinyUSB, pinned
make
make flash   # over the Nucleo's ST-LINK; `make flash-openocd` if st-flash fails with error 255
```

Bare metal on CMSIS and TinyUSB. The CPU runs on HSI (64 MHz); USB on HSI48, trimmed by the CRS
from the host's start-of-frame packets (`status` shows `crs_sync 1` once it locked), so no crystal
is involved. Holding B1 at reset starts the ROM bootloader.

## Console

ST-LINK virtual COM port, 115200 8N1, one command per line. Events start with `EV`.

| Command | Answer |
|---|---|
| `status` | `STATUS mode … vbus … mounted … suspended … mounts … unmounts … crs_sync … trim …`, then a line for the mode |
| `mode <name>` | resets into that mode (kept across resets, not power cycles; default `cdc`) |
| `detach`, `attach` | the pull-up off and on: the host sees the device leave and come back |
| `replug [ms]` | detach, wait (500 ms), attach |
| `files` | `FILE <name> <size> <crc32>` per root-directory file, then `FILES <n>` |
| `uf2` | `UF2 blocks <found> of <total> complete <0\|1> family … addr <lo>-<hi> payload <bytes> crc <crc32>` |
| `teensy [len]` | `TEENSY blocks … hi … reboots … len … crc …` (CRC-32 of the first `len` bytes of the image) |
| `image <addr> <len>` | `IMAGE addr … len … crc …`: CRC-32 of the serial bootloaders' emulated flash (480 KB, erased when the mode is set here, kept across the touch and leave resets) |
| `bridge none\|ftdi\|cp210x\|ch34x` | resets with that chip in front (kept across resets, like the mode) |
| `format` | an empty drive again |
| `dfu` | the ROM bootloader |
| `reset`, `version`, `help` | |

Events: `EV mount <n>`, `EV unmount`, `EV suspend`, `EV resume`, `EV cdc line <baud> <frame>`,
`EV cdc dtr <0|1> rts <0|1>`, `EV cdc touch1200`, `EV msc eject`, `EV uf2 complete`,
`EV teensy reboot blocks <n>`, `EV loader leave`, `EV reset into <mode>`, `EV esp download`,
`EV esp boot`. `status` adds a `LOADER`, `ESP` and `BRIDGE` line in those modes.

The UF2 mode keeps the blocks the host writes as they are (a real UF2 bootloader flashes them and
reboots), so the blocks may overwrite the drive's two text files: `uf2` reads them back from
wherever they landed.

## Tests

```bash
make host-check    # the pod's own MSC flasher (benchpod-firmware) against the RAM drive, no hardware

pytest tests -v --benchpod-connection=<pod host on the LAN>
```

| File | Covers |
|---|---|
| `test_enumerate.py` | every mode enumerates with its IDs, strings and interfaces; replugs count; a detach is seen |
| `test_flash.py` | `file` (copied, replaced, refused when too big), UF2, Teensy with erased blocks the pod skips, ROM DfuSe taking this firmware's own image |
| `test_robust.py` | the hostile devices, unplugging during a flash, abort, the host-mode timeout, start/stop cycles (`USBNODE_CYCLES`, default 20) |
| `test_serial_flash.py` | the AVR bootloaders (probing, the touch, refusals), SAM-BA (the touch, the bootloader kept, staying in it), the ESP32-S3 ROM (reset, MD5, app offset, a port that only echoes) |
| `test_bridges.py` | AVR bootloaders behind each chip, an ESP32 dev kit behind each chip switching to 460800 |
| `test_throughput.py` | KB/s per mode with floors (`USBNODE_THROUGHPUT_JSON` writes the table) |

The suite runs the same over the cloud (`--benchpod-connection=embeddedci:<pod>`, with
`pip install 'embeddedci[cloud]'`). A test that leaves the node in the ROM bootloader resets it
through its ST-LINK (`st-flash reset`).

Throughput a digital v3r3 pod measured (the node keeps everything in RAM, so these are the pod's
limits per mode, not a real board's; staging the file is not counted):

| Mode | KB/s | |
|---|---|---|
| `file`, `uf2`, `teensy` | 145-150 | |
| `esp` (native USB) | 65 | 1 KB blocks, one reply each, then the MD5 |
| `samd` | 50 | write, then read back |
| `avr` | 6 | a page at a time, read back, as avrdude does |
| `dfu` | 10 | the H753's ROM erasing and writing its flash: a real board's number |

`USBNODE_CONSOLE` names the node's console when the computer has more than one ST-LINK.
`USB_HOST_PORT` is `usb_c` (default) or `usb_a`. Take the pod's lock first on a shared bench.
