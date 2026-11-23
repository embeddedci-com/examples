"""The pod's serial-port flashers (usb_flash avr and samd) against the bootloaders the node plays on
its CDC port: optiboot, the old Nano bootloader, the Mega 2560's STK500v2, Caterina after the 1200
baud touch, and SAM-BA after the touch."""

import random
import time
import zlib

import pytest

pytestmark = pytest.mark.hardware

ARDUINO = 0x2341


def flash_image(node, addr, length):
    m = node.cmd(f"image {addr} {length}", r"^(IMAGE addr \S+ len \d+ crc ([0-9a-f]{8})|ERR.*)$")
    assert m.group(2), m.group(1)
    return int(m.group(2), 16)


def loader(node):
    return dict(zip(*[iter(node.cmd("status", r"^LOADER (.*)").group(1).split())] * 2))


@pytest.mark.parametrize("mode,vid,pid,proto,size", [
    ("optiboot", ARDUINO, 0x0043, "stk500v1", 20000),      # Uno: the first probe
    ("atmegaboot", 0xCAFE, 0x4021, "stk500v1", 20000),     # old Nano: found at 57600 after two misses
    ("stk500v2", ARDUINO, 0x0042, "stk500v2", 150000),     # Mega 2560, past 128 KB (extended address)
])
def test_avr_probed(host, node, usb_flash, mode, vid, pid, proto, size):
    node.set_mode(mode)
    host.wait_device(vid=vid, pid=pid)
    data = random.Random(size).randbytes(size)
    node.mark()
    st = usb_flash(data, mode="avr", timeout=180)
    assert st["ok"], st
    assert st["protocol"] == proto
    node.wait(r"^EV loader leave$", timeout=5)
    assert flash_image(node, 0, len(data)) == zlib.crc32(data)
    if mode == "atmegaboot":
        assert int(loader(node)["dropped"]) > 0  # the probes at 115200 went unanswered


def test_avr_leonardo_touch(host, node, usb_flash):
    """A 32u4 running its sketch: the 1200 baud touch, Caterina comes up, then the sketch again."""
    node.set_mode("leonardo")
    host.wait_device(vid=ARDUINO, pid=0x8036)
    data = random.Random(8).randbytes(12000)
    node.mark()
    st = usb_flash(data, mode="avr", timeout=60)
    assert st["ok"], st
    assert st["protocol"] == "avr109"
    node.wait(r"^EV cdc touch1200$", timeout=5)
    node.wait(r"^EV reset into caterina$", timeout=5)
    node.wait(r"^EV reset into leonardo$", timeout=15)  # E: the bootloader starts the sketch
    node.wait(r"^APP_OK$", timeout=5)
    assert flash_image(node, 0, len(data)) == zlib.crc32(data)
    host.wait_device(vid=ARDUINO, pid=0x8036)


def test_avr_caterina_direct(host, node, usb_flash):
    """A board already in Caterina (IDs of the bootloader): no touch."""
    node.set_mode("caterina")
    host.wait_device(vid=ARDUINO, pid=0x0036)
    data = bytes(range(256)) * 20
    node.mark()
    st = usb_flash(data, mode="avr", leave=False, timeout=60)
    assert st["ok"] and st["protocol"] == "avr109", st
    assert flash_image(node, 0, len(data)) == zlib.crc32(data)
    assert loader(node)["leaves"] == "0"  # leave=false: still in the bootloader


def test_avr_too_big(host, node, usb_flash):
    """An ATmega328P has 32 KB less a 512-byte bootloader."""
    node.set_mode("optiboot")
    host.wait_device(vid=ARDUINO, pid=0x0043)
    st = usb_flash(bytes(32 * 1024), mode="avr", timeout=60)
    assert not st["ok"]
    assert "bootloader" in st["result"] or "fit" in st["result"], st


def test_samd_zero_touch(host, node, usb_flash):
    """A SAMD21 running its sketch: the touch, SAM-BA erases from 0x2000, writes, reads back, resets."""
    node.set_mode("zero")
    host.wait_device(vid=ARDUINO, pid=0x804D)
    data = random.Random(9).randbytes(50000)
    node.mark()
    st = usb_flash(data, mode="samd", address="0x2000", timeout=120)
    assert st["ok"], st
    assert st["protocol"] == "sam-ba"
    node.wait(r"^EV reset into samba$", timeout=5)
    node.wait(r"^EV reset into zero$", timeout=30)  # the AIRCR write
    node.wait(r"^APP_OK$", timeout=5)
    assert flash_image(node, 0x2000, len(data)) == zlib.crc32(data)


def test_samd_keeps_bootloader(host, node, usb_flash):
    node.set_mode("samba")
    host.wait_device(vid=ARDUINO, pid=0x004D)
    st = usb_flash(bytes(4096), mode="samd", address="0x0", timeout=60)
    assert not st["ok"]
    assert "bootloader" in st["result"], st
    assert loader(node)["erases"] == "0"  # refused before anything was erased


def test_samd_stay_in_loader(host, node, usb_flash):
    node.set_mode("samba")
    host.wait_device(vid=ARDUINO, pid=0x004D)
    data = random.Random(10).randbytes(9000)  # not a whole number of 4 KB chunks
    st = usb_flash(data, mode="samd", address="0x2000", leave=False, timeout=60)
    assert st["ok"], st
    time.sleep(0.5)
    assert node.status()["mode"] == "samba"
    assert flash_image(node, 0x2000, len(data)) == zlib.crc32(data)
