#!/usr/bin/env python3
"""End-to-end local test: prove the on-pod CMSIS-DAP probe works.

Run against a BenchPod on your LAN (TCP, not the cloud). It:

  1. flashes ``build/selftest.elf`` to the wired STM32 over the **CMSIS-DAP** path
     (OpenOCD's cmsis-dap TCP backend -> the pod's dap_start), and
  2. opens an **event-based UART session**, then does a *non-delayed* eFuse
     power-on and waits for the firmware's ``APP_OK`` boot banner, and
  3. drives the DUT console (``ping`` -> ``pong``) on the same live stream.

Seeing ``APP_OK`` after a DAP flash means the pod actually wrote firmware over the
SWD wire and the target booted it — i.e. the DAP probe is fully working.

Prereqs: embeddedci>=2.0, an OpenOCD with the ``cmsis_dap_tcp`` backend (newer than
0.12.0, e.g. xPack OpenOCD), the pod runs the current firmware (DAP + uart proxy) and
is on the network, and the STM32 is wired (SWCLK/SWDIO + UART + power eFuse on LA
channels, NRST on the pod's reset pin, DUT header J1 pin 22) per the options below.

    python e2e_local.py [ip]            # default ip: 192.168.1.214
    python e2e_local.py 192.168.1.50 --swclk 11 --swdio 12 \
        --uart-rx 5 --uart-tx 4 --efuse 1
    python e2e_local.py 192.168.1.50 --no-nreset   # NRST not wired to the pod

The LA bank voltage is the ``LA_VOLTAGE`` constant at the top of this file — change it for a
1V8 board.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

from embeddedci.benchpod import BenchPod
from embeddedci.benchpod.errors import BenchPodError

# LA I/O-bank voltage selected right after connecting — the pod refuses flash and UART until
# one is set. It must match the DUT's I/O voltage.
LA_VOLTAGE = 3.3  # board I/O voltage — change to 1.8 for a 1V8 board

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ELF = os.path.join(HERE, "build", "selftest.elf")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ip", nargs="?", default="192.168.1.214",
                    help="pod address (host or host:port; default 192.168.1.214)")
    ap.add_argument("--firmware", default=DEFAULT_ELF, help="DUT firmware .elf")
    ap.add_argument("--target", default="target/stm32f4x.cfg",
                    help="OpenOCD target config (selects the flash algorithm)")
    ap.add_argument("--swclk", type=int, default=11)
    ap.add_argument("--swdio", type=int, default=12)
    ap.add_argument("--no-nreset", dest="nreset", action="store_false",
                    help="NRST is NOT wired to the pod's reset pin (J1 pin 22): skip "
                         "connect-under-reset")
    ap.add_argument("--uart-rx", type=int, default=5, help="LA pin the pod samples (DUT TX)")
    ap.add_argument("--uart-tx", type=int, default=4, help="LA pin the pod drives (DUT RX)")
    ap.add_argument("--efuse", type=int, default=1, help="target-power eFuse (1=internal, 2=external)")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    if not os.path.exists(args.firmware):
        print(f"FAIL: firmware not found: {args.firmware}\n"
              f"      build it first:  make CUBE_F4=/path/to/STM32CubeF4")
        return 2

    print(f"== BenchPod end-to-end DAP test @ {args.ip} ==")
    try:
        bp = BenchPod(args.ip, la_voltage=LA_VOLTAGE, timeout=20.0)
    except BenchPodError as exc:
        print(f"FAIL: cannot reach the pod ({exc}).\n"
              f"      Is it powered, on the network, and at {args.ip}?")
        return 2
    with bp:
        return _run(bp, args)


def _run(bp: BenchPod, args: argparse.Namespace) -> int:
    # 0) Reachability / firmware sanity. status() is a dict on every transport.
    try:
        st = bp.status()
    except BenchPodError as exc:
        print(f"FAIL: cannot reach the pod ({exc}).\n"
              f"      Is it powered, on the network, and at {args.ip}?")
        return 2
    print(f"   pod status: version={st.get('version')} caps={st.get('caps')}")
    print(f"   LA bank: {bp.get_la_voltage().voltage} V")

    # 1) Flash over DAP.
    print(f"-- flashing {os.path.basename(args.firmware)} via CMSIS-DAP "
          f"(swclk={args.swclk} swdio={args.swdio} nreset={args.nreset}) ...")
    t0 = time.time()
    try:
        res = bp.flash(
            file=args.firmware, target=args.target,
            swclk=args.swclk, swdio=args.swdio, nreset=args.nreset,
            target_power=args.efuse, check=False,
        )
        # If the wire never answered under reset, retry a plain connect: selftest.c
        # does not remap the SWD pins, so connect-under-reset isn't required and a
        # mis-wired/floating NRST would otherwise block the read.
        if not res.ok and res.target_unreachable and args.nreset:
            print("   under-reset connect failed; retrying without NRST ...")
            res = bp.flash(
                file=args.firmware, target=args.target,
                swclk=args.swclk, swdio=args.swdio, nreset=False,
                target_power=args.efuse, check=False,
            )
    except BenchPodError as exc:
        print(f"FAIL: flash raised: {exc}")
        return 1
    dt = time.time() - t0
    if not res.ok:
        print(f"FAIL: DAP flash failed in {dt:.1f}s "
              f"(target_unreachable={res.target_unreachable}).")
        print("----- openocd output -----")
        print((res.stderr or res.stdout).strip()[-2000:])
        if res.target_unreachable:
            print("\nThe pod's CMSIS-DAP probe ran, but the target never answered on SWD: "
                  "check target power, SWCLK/SWDIO/NRST wiring, and a common ground.")
        return 1
    print(f"   DAP flash OK in {dt:.1f}s")

    # 2) Event-based UART. Over the local wifi/AT link the pod can't service a
    #    second command while a UART session is open, so we *schedule* the power-on
    #    pod-side (returns immediately) BEFORE opening the session — it then fires
    #    while the event-based reader is already listening, catching the banner with
    #    no fixed capture window. (Over the cloud the command channel lets you do a
    #    truly non-delayed power_on() during the session; see docs/event-uart-design.md.)
    print("-- scheduling power-on, opening UART; waiting for APP_OK ...")
    bp.power_off(args.efuse)
    bp.power_on(args.efuse, delay=1.5)  # pod-side timer; fires after the UART opens
    with bp.open_uart(rx=args.uart_rx, tx=args.uart_tx, baud=args.baud) as uart:
        if not uart.read_until("APP_OK", timeout=12):
            print("FAIL: did not see APP_OK after power-on.")
            print("----- captured UART -----")
            print(uart.text.strip()[-2000:] or "(nothing received)")
            return 1
        print("   APP_OK seen ✓")

        # 3) Interactive console check on the same live stream.
        uart.read()
        uart.write("ping\r\n")
        if uart.expect("pong", timeout=4):
            print("   console ping -> pong ✓")

    print("\nPASS: DAP flashing + UART boot verified — the on-pod CMSIS-DAP probe is fully working.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
