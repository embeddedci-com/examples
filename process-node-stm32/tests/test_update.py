"""Firmware-update HIL test: flash the node again and again, it must come back every time.

A product gets updated in the field; the bench equivalent is SWD flashing through the pod. Each
cycle flashes the image under test, boots it and checks it identifies as the expected firmware and
version. PROCESS_NODE_FLASH_CYCLES sets the count (default 2, 0 skips); it needs
--benchpod-firmware.
"""

import os
import pathlib
import re
import time

import pytest

from conftest import _flash

CYCLES = int(os.environ.get("PROCESS_NODE_FLASH_CYCLES", "2"))
MAIN_C = pathlib.Path(__file__).resolve().parent.parent / "main.c"


def _expected_version():
    m = re.search(r'#define FW_VERSION "([^"]+)"', MAIN_C.read_text())
    assert m, f"no FW_VERSION in {MAIN_C}"
    return m.group(1)


@pytest.mark.hardware
@pytest.mark.parametrize("cycle", range(CYCLES) if CYCLES > 0 else [pytest.param(0, marks=pytest.mark.skip(reason="PROCESS_NODE_FLASH_CYCLES=0"))])
def test_flash_and_boot(benchpod, wiring, node, pytestconfig, cycle):
    firmware = pytestconfig.getoption("benchpod_firmware")
    if not firmware:
        pytest.skip("needs --benchpod-firmware")
    with node.paused():
        start = time.monotonic()
        result = _flash(benchpod, wiring, firmware)
        took = time.monotonic() - start
        assert result.ok, f"flash cycle {cycle} failed after {took:.1f} s:\n{result.stderr}"
        benchpod.power_off(wiring.efuse)
        benchpod.power_on(wiring.efuse, delay=0.5)
    node.uart.read_until("APP_OK", timeout=6)
    info = node.info()
    assert (info["fw"], info["version"]) == ("process-node", _expected_version()), info
    assert node.status()["chip"] == "ok"
