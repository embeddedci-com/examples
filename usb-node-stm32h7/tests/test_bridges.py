"""The pod's serial flashers through the USB-serial chips of boards without native USB: the node
enumerates as an FTDI FT232R, a CP2102 or a CH340 and decodes the chip's vendor requests (baud rate,
DTR/RTS), with the bootloader behind it."""

import random
import zlib

import pytest

pytestmark = pytest.mark.hardware

CHIPS = {"ftdi": (0x0403, 0x6001), "cp210x": (0x10C4, 0xEA60), "ch34x": (0x1A86, 0x7523)}


@pytest.fixture
def bridged(host, node):
    """``bridged(chip, mode)``: the node in ``mode`` behind ``chip``; no chip again afterwards."""

    def setup(chip, mode):
        node.set_mode(mode)
        node.set_bridge(chip)
        vid, pid = CHIPS[chip]
        host.wait_device(vid=vid, pid=pid)

    yield setup
    node.set_bridge("none")


def status_line(node, tag):
    """A status line's key/value pairs; BRIDGE starts with the chip's name (key "chip")."""
    words = node.cmd("status", rf"^{tag} (.*)").group(1).split()
    if tag == "BRIDGE":
        words = ["chip"] + words
    return dict(zip(*[iter(words)] * 2))


def image_crc(node, addr, length):
    return int(node.cmd(f"image {addr} {length}", r"^IMAGE .* crc ([0-9a-f]{8})").group(1), 16)


@pytest.mark.parametrize("chip,mode,baud", [
    ("ch34x", "optiboot", 115200),    # a Nano clone
    ("ftdi", "atmegaboot", 57600),    # a Pro Mini on an FTDI cable: found at 57600
    ("cp210x", "stk500v2", 115200),   # a Mega clone
])
def test_avr_behind_chip(bridged, node, usb_flash, chip, mode, baud):
    bridged(chip, mode)
    data = random.Random(hash(chip) & 0xFFFF).randbytes(16000)
    st = usb_flash(data, mode="avr", timeout=180)
    assert st["ok"], st
    assert image_crc(node, 0, len(data)) == zlib.crc32(data)
    assert int(status_line(node, "BRIDGE")["baud"]) == baud  # the rate the chip was set to


@pytest.mark.parametrize("chip", ["cp210x", "ch34x", "ftdi"])
def test_esp32_devkit(bridged, node, usb_flash, chip):
    """An ESP32 dev kit: the auto-reset circuit, then the ROM and the chip both at 460800."""
    bridged(chip, "esp32")
    data = random.Random(13).randbytes(120 * 1024)
    node.mark()
    st = usb_flash(data, mode="esp", address=0, timeout=180)
    assert st["ok"], st
    assert st["protocol"] == "ESP32"
    node.wait(r"^EV esp download$", timeout=5)
    node.wait(r"^EV esp boot$", timeout=5)
    assert image_crc(node, 0, len(data)) == zlib.crc32(data)
    e = status_line(node, "ESP")
    assert e["baud_changes"] == "1" and e["wrong_baud"] == "0", e
    assert status_line(node, "BRIDGE")["baud"] == "460800"
