"""The pod flashes the usb-node over USB in every mode it plays, and the node checks what arrived."""

import os
import random
import struct
import time
import zlib
from pathlib import Path

import pytest

pytestmark = pytest.mark.hardware

FIRMWARE_BIN = Path(__file__).resolve().parents[1] / "build" / "usb-node.bin"


def make_uf2(payload: bytes, addr: int = 0x10000000, family: int = 0xE48BFF56, chunk: int = 256) -> bytes:
    """A UF2 file (the RP2040 family by default), ``chunk`` bytes of payload per 512-byte block."""
    blocks = [payload[i:i + chunk] for i in range(0, len(payload), chunk)]
    out = bytearray()
    for n, data in enumerate(blocks):
        head = struct.pack("<8I", 0x0A324655, 0x9E5D5157, 0x00002000, addr + n * chunk, len(data), n,
                           len(blocks), family)
        out += head + data.ljust(476, b"\0") + struct.pack("<I", 0x0AB16F30)
    return bytes(out)


def test_file_copy(host, node, usb_flash):
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    data = random.Random(1).randbytes(61440)
    st = usb_flash(data, mode="file", name="FIRMWARE.BIN")
    assert st["ok"], st
    files = node.files()
    assert files["FIRMWARE.BIN"][:2] == (len(data), zlib.crc32(data))
    assert files["FIRMWARE.BIN"][2] == ""  # the cluster chain is whole
    assert "README.TXT" in files  # what was on the drive is still there


def test_file_replaced(host, node, usb_flash):
    """A file of the same name is replaced, not added twice."""
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    assert usb_flash(b"A" * 10000, mode="file", name="APP.BIN")["ok"]
    data = random.Random(2).randbytes(30000)
    assert usb_flash(data, mode="file", name="APP.BIN")["ok"]
    files = node.files()
    assert files["APP.BIN"][:2] == (len(data), zlib.crc32(data))
    assert list(files).count("APP.BIN") == 1


def test_file_too_big(host, node, usb_flash):
    """A file larger than the drive is refused, not half-written."""
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    st = usb_flash(bytes(600 * 1024), mode="file", name="BIG.BIN")
    assert not st["ok"]
    assert "BIG.BIN" not in node.files()


def test_uf2(host, node, usb_flash):
    node.set_mode("uf2")
    host.wait_device(vid=0xCAFE, pid=0x4003)
    payload = random.Random(3).randbytes(200 * 256)
    node.mark()
    st = usb_flash(make_uf2(payload), mode="uf2")
    assert st["ok"], st
    node.wait(r"^EV uf2 complete$", timeout=5)
    m = node.cmd("uf2", r"^UF2 blocks (\d+) of (\d+) complete (\d) family (\w+) addr (\w+)-(\w+) payload (\d+) crc (\w+)")
    blocks, total, complete, family, lo, hi, size, crc = m.groups()
    assert (int(blocks), int(total), complete) == (200, 200, "1")
    assert int(family, 16) == 0xE48BFF56
    assert (int(lo, 16), int(hi, 16)) == (0x10000000, 0x10000000 + len(payload))
    assert (int(size), int(crc, 16)) == (len(payload), zlib.crc32(payload))


def test_uf2_rejects_other_files(host, node, usb_flash):
    node.set_mode("uf2")
    host.wait_device(vid=0xCAFE, pid=0x4003)
    st = usb_flash(bytes(4096), mode="uf2")
    assert not st["ok"]


def test_teensy(host, node, usb_flash):
    node.set_mode("teensy")
    host.wait_device(vid=0x16C0, pid=0x0478)
    rnd = random.Random(4)
    # 120 blocks, some of them erased flash (0xFF): the pod skips those, the image must still match.
    blocks = [b"\xff" * 1024 if i % 7 == 3 else rnd.randbytes(1024) for i in range(120)]
    data = b"".join(blocks)[:-100]  # a last block that is not full
    node.mark()
    st = usb_flash(data, mode="teensy", address=0)
    assert st["ok"], st
    node.wait(r"^EV teensy reboot", timeout=5)
    m = node.cmd(f"teensy {len(data)}", r"^TEENSY blocks (\d+) hi (\w+) reboots (\d+) len (\d+) crc (\w+)")
    sent, _, reboots, _, crc = m.groups()
    assert int(sent) == 120 - len([i for i in range(120) if i % 7 == 3 and i != 0])
    assert int(reboots) == 1
    assert int(crc, 16) == zlib.crc32(data)


def test_teensy_keep_in_loader(host, node, usb_flash):
    node.set_mode("teensy")
    host.wait_device(vid=0x16C0, pid=0x0478)
    assert usb_flash(bytes(range(256)) * 8, mode="teensy", address=0, leave=False)["ok"]
    assert node.cmd("teensy", r"^TEENSY .* reboots (\d+)").group(1) == "0"


@pytest.mark.skipif(not FIRMWARE_BIN.exists(), reason="build the firmware first (make)")
def test_rom_dfu_reflashes_itself(host, node, usb_flash):
    """ST's ROM bootloader (DfuSe) takes the usb-node's own image, then the node boots it again."""
    node.set_mode("cdc")
    host.wait_device(vid=0xCAFE, pid=0x4001)
    before = host.state()["enumerations"]
    node.cmd("dfu", r"^OK ROM bootloader")
    st = host.wait_device(vid=0x0483, pid=0xDF11, after=before, timeout=15)
    assert "dfu" in [i["class_name"] for i in st["device"]["config"]["interfaces"]] or \
        st["device"]["config"]["interfaces"][0]["class"] == 0xFE
    node.mark()
    image = FIRMWARE_BIN.read_bytes()
    st = usb_flash(image, mode="dfu", address="0x08000000", timeout=120)
    assert st["ok"], st
    node.wait(r"usb-node-stm32h7 \S+ mode \w+$", timeout=15)
    node.wait(r"^APP_OK$", timeout=5)
    host.wait_device(vid=0xCAFE, timeout=15)
