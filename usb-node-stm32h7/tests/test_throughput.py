"""How fast the pod flashes in each mode, with floors that catch a slowdown.

The node keeps what it receives in RAM, so these are the pod's own limits per mode (full-speed USB
and the protocol, staging the file not counted), not how fast a real board writes its flash; the
ROM DfuSe case is the exception (the H753's ROM writes its flash). USBNODE_THROUGHPUT_JSON names a
file to write the table to."""

import json
import os
import random
import time
from pathlib import Path

import pytest

from test_flash import FIRMWARE_BIN, make_uf2

pytestmark = pytest.mark.hardware

# mode, node mode, size in KB, usb_flash arguments, floor in KB/s (about half of what a
# digital v3r3 pod measured on 2026-11-22: file 140, uf2 146, teensy 145, samd 49, esp 68, avr 6)
CASES = [
    ("file", "msc", 256, {"name": "T.BIN"}, 70),
    ("uf2", "uf2", 200, {}, 70),
    ("teensy", "teensy", 256, {"address": 0}, 70),
    ("samd", "samba", 200, {"address": "0x2000"}, 24),
    ("esp", "esp32s3", 256, {"address": 0}, 32),
    ("avr", "optiboot", 31, {}, 3),
]

_RESULTS = []


@pytest.fixture(scope="module", autouse=True)
def report():
    yield
    if not _RESULTS:
        return
    print("\n  mode     KB    ms     KB/s")
    for r in _RESULTS:
        print(f"  {r['mode']:7s} {r['kb']:4d} {r['ms']:6d} {r['kb_s']:7.1f}")
    out = os.environ.get("USBNODE_THROUGHPUT_JSON")
    if out:
        Path(out).write_text(json.dumps(_RESULTS, indent=1))


def _record(mode, nbytes, ms):
    kb_s = nbytes / 1024 / (ms / 1000)
    _RESULTS.append({"mode": mode, "kb": nbytes // 1024, "ms": ms, "kb_s": round(kb_s, 1)})
    return kb_s


@pytest.mark.parametrize("mode,node_mode,kb,args,floor", CASES, ids=[c[0] for c in CASES])
def test_throughput(host, node, usb_flash, mode, node_mode, kb, args, floor):
    before = host.state().get("enumerations", 0)
    node.set_mode(node_mode)
    host.wait_device(after=before, timeout=10)  # the node in its new mode, not the old one
    data = random.Random(kb).randbytes(kb * 1024)
    payload = make_uf2(data) if mode == "uf2" else data
    st = usb_flash(payload, mode=mode, timeout=300, **args)
    assert st["ok"], st
    kb_s = _record(mode, len(payload), st["ms"])
    assert kb_s >= floor, f"{mode}: {kb_s:.1f} KB/s, the floor is {floor}"


@pytest.mark.skipif(not FIRMWARE_BIN.exists(), reason="build the firmware first (make)")
def test_throughput_rom_dfu(host, node, usb_flash):
    """ST's ROM loader writing the H753's flash: erases and writes for real."""
    node.set_mode("cdc")
    host.wait_device(vid=0xCAFE, pid=0x4001)
    before = host.state()["enumerations"]
    node.cmd("dfu", r"^OK ROM bootloader")
    host.wait_device(vid=0x0483, pid=0xDF11, after=before, timeout=15)
    image = FIRMWARE_BIN.read_bytes()
    try:
        st = usb_flash(image, mode="dfu", address="0x08000000", timeout=120)
        assert st["ok"], st
        node.wait(r"^APP_OK$", timeout=15)
    except Exception:
        node.hard_reset()
        raise
    kb_s = _record("dfu", len(image), st["ms"])
    assert kb_s >= 5, f"dfu: {kb_s:.1f} KB/s"
    time.sleep(0.5)
