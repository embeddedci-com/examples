"""Shared pytest config for the process-node HIL tests: LA bank voltage, bench wiring, flashing and
the console-driven ``node`` fixture every test file uses.

The node is flashed (when --benchpod-firmware is given) and booted ONCE per session, so the CAN,
analog and lifecycle files share one boot.
"""

import os
import re
from types import SimpleNamespace

import pytest

TARGET_CFG = "target/stm32f4x.cfg"
FLASH_ATTEMPTS = 3
OSC_MHZ = int(os.environ.get("CAN_NODE_OSC_MHZ", "8"))
BITRATE = 500_000


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    """LA I/O-bank voltage the ``benchpod`` fixture selects on connect (must match the DUT)."""
    return 3.3  # NUCLEO-F446RE I/O voltage


@pytest.fixture(scope="session")
def wiring(pins):
    """How THIS bench is wired: DUT signal -> BenchPod LA channel."""
    return SimpleNamespace(
        swclk=pins.pin_11,
        swdio=pins.pin_12,
        nreset=True,
        uart_rx=pins.pin_3,   # pod samples the DUT's TX here
        uart_tx=pins.pin_4,   # pod drives the DUT's RX here
        efuse=pins.efuse,
        # Analog in: pod 3.3 V DAC SMA -> 10 kOhm -> PA1 (Nucleo A1). PROCESS_NODE_ANALOG=0 on a
        # bench without that lead skips the analog tests instead of failing them.
        ain_path="3v3",
        analog=os.environ.get("PROCESS_NODE_ANALOG", "1") == "1",
    )


def _flash(bp, wiring, firmware):
    result = None
    for attempt in range(FLASH_ATTEMPTS):
        if attempt > 0:
            bp.power_off(wiring.efuse)
        result = bp.flash(
            file=firmware, target=TARGET_CFG,
            swclk=wiring.swclk, swdio=wiring.swdio, nreset=wiring.nreset,
            target_power=wiring.efuse, check=False,
        )
        if not result.ok and result.target_unreachable and wiring.nreset:
            result = bp.flash(
                file=firmware, target=TARGET_CFG,
                swclk=wiring.swclk, swdio=wiring.swdio, nreset=False,
                target_power=wiring.efuse, check=False,
            )
        if result.ok:
            break
    return result


class Node:
    """The Nucleo process node, driven through its UART console."""

    def __init__(self, uart):
        self.uart = uart

    def cmd(self, line, pattern, timeout=3.0):
        """Send a console line and wait for ``pattern`` (regex) in the reply."""
        self.uart.read()  # forget earlier output so we only match this command's reply
        self.uart.write(line + "\r\n")
        return self.uart.expect(re.compile(pattern), timeout=timeout)

    def info(self):
        """``info`` as a dict: fw, version, build, uptime_ms, reset."""
        m = self.cmd("info", r"INFO ([^\r\n]*)\r\n")
        return {k: v.strip('"') for k, v in re.findall(r'(\w+)=("[^"]*"|\S+)', m.group(1))}

    def ain(self):
        """One fresh analog-input reading: mv, raw, filt_mv, vdda_mv, samples (all ints)."""
        m = self.cmd("ain", r"AIN ([^\r\n]*)\r\n")
        return {k: int(v) for k, v in (kv.split("=", 1) for kv in m.group(1).split())}

    def status(self):
        m = self.cmd("can status", r"CAN status: ([^\r\n]*)\r\n")
        return dict(kv.split("=", 1) for kv in m.group(1).split())

    def init(self, bitrate=BITRATE):
        self.cmd(f"can init {bitrate}", r"CAN init ok bitrate=%d" % bitrate)
        self.cmd("can echo off", r"CAN echo off")
        self.cmd("can print on", r"CAN print on")
        self.cmd("can clear", r"CAN counters cleared")

    def expect_rx(self, can_id, data=b"", ext=False, timeout=3.0):
        """Wait for the node to print a received frame."""
        pat = r"CAN rx id=0x%X ext=%d rtr=0 dlc=%d data=%s\r\n" % (
            can_id, int(ext), len(data), bytes(data).hex().upper())
        return self.uart.expect(re.compile(pat), timeout=timeout)


@pytest.fixture(scope="session")
def node(benchpod, wiring, pytestconfig):
    """Flash (optional), power-cycle and boot the node; one UART session for the whole run."""
    firmware = pytestconfig.getoption("benchpod_firmware")
    if firmware:
        result = _flash(benchpod, wiring, firmware)
        assert result.ok, f"flash failed; openocd output:\n{result.stderr}"
    benchpod.power_off(wiring.efuse)
    benchpod.power_on(wiring.efuse, delay=1.5)
    try:
        with benchpod.open_uart(rx=wiring.uart_rx, tx=wiring.uart_tx) as uart:
            n = Node(uart)
            # The boot banner can be clipped on a slow link, so check the result by command.
            uart.read_until("APP_OK", timeout=5)
            if OSC_MHZ != 8:
                n.cmd(f"can osc {OSC_MHZ}", r"CAN init (ok|fail)")
            st = n.status()
            assert st["chip"] == "ok", f"MCP2515 not answering on SPI: {st}"
            yield n
    finally:
        benchpod.power_off(wiring.efuse)


