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

The console reports what the host did (files on the drive with their CRC-32, UF2 blocks, Teensy
blocks), so a test compares it with what it sent.

## Bench

```
pod USB-C ── USB-C male breakout ──wires── USB-A female breakout ── USB-A to micro-USB cable ── CN13
                  │ VBUS/GND
               5 V supply (powers the pod, and is the VBUS on CN13)
```

| USB-C male (pod) | USB-A female | Also |
|---|---|---|
| D+ | D+ | |
| D- | D- | |
| GND | GND | 5 V supply GND |
| VBUS | VBUS | 5 V supply + |
| CC1, CC2 | not connected | the pod has its own 5.1 kOhm pull-downs |

Keep D+/D- short. A USB-C charger as the 5 V supply only turns on when it sees the CC pull-downs:
pass CC through to the pod, or use a USB-A charger or a bench supply. On a pod with a USB-A host
port, plug the cable into that port instead and set `USB_HOST_PORT=usb_a`.

The Nucleo itself stays on its ST-LINK USB (CN1) to the computer: power (the power jumper on
5V_STLK, the default), flashing and the console. VBUS on CN13 only tells the MCU a host is there.

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
| `mode cdc\|msc\|uf2\|teensy` | resets into that mode (kept across resets, not power cycles; default `cdc`) |
| `detach`, `attach` | the pull-up off and on: the host sees the device leave and come back |
| `replug [ms]` | detach, wait (500 ms), attach |
| `files` | `FILE <name> <size> <crc32>` per root-directory file, then `FILES <n>` |
| `uf2` | `UF2 blocks <found> of <total> complete <0\|1> family … addr <lo>-<hi> payload <bytes> crc <crc32>` |
| `teensy [len]` | `TEENSY blocks … hi … reboots … len … crc …` (CRC-32 of the first `len` bytes of the image) |
| `format` | an empty drive again |
| `dfu` | the ROM bootloader |
| `reset`, `version`, `help` | |

Events: `EV mount <n>`, `EV unmount`, `EV suspend`, `EV resume`, `EV cdc line <baud> <frame>`,
`EV cdc dtr <0|1> rts <0|1>`, `EV cdc touch1200`, `EV msc eject`, `EV uf2 complete`,
`EV teensy reboot blocks <n>`.

The UF2 mode keeps the blocks the host writes as they are (a real UF2 bootloader flashes them and
reboots), so the blocks may overwrite the drive's two text files: `uf2` reads them back from
wherever they landed.

## Tests

```bash
make host-check    # the pod's own MSC flasher (benchpod-firmware) against the RAM drive, no hardware

pytest tests -v --benchpod-connection=<pod host on the LAN>
```

`tests/test_enumerate.py`: every mode enumerates with its IDs, strings and interfaces; replugs
count; a detach is seen. `tests/test_flash.py`: a file copied in `file` mode (and replaced, and
refused when too big), a UF2 file, a Teensy image with erased blocks the pod skips, and the ROM
DfuSe loader taking this firmware's own image, after which the node boots again.

`USBNODE_CONSOLE` names the node's console when the computer has more than one ST-LINK.
`USB_HOST_PORT` is `usb_c` (default) or `usb_a`. Take the pod's lock first on a shared bench.
