"""The pod's USB host against hostile devices and things going wrong mid-way: it reports what it
can, never hangs, and the next device works."""

import os
import random
import time
import zlib

import pytest

pytestmark = pytest.mark.hardware

CYCLES = int(os.environ.get("USBNODE_CYCLES", "20"))


#: The node's product IDs (usb_descriptors.c): 0x4001-0x4003 for cdc, msc, uf2, 0x4010 + n for the
#: hostile modes in their order in usbnode.h.
PIDS = {"cdc": 0x4001, "msc": 0x4002, "uf2": 0x4003}
PIDS.update({m: 0x4010 + i for i, m in enumerate(
    ["many", "big", "huge", "badlen", "overlong", "strings", "nostr"])})


def enumerate_mode(host, node, mode, timeout=6.0):
    """Switch the node to ``mode`` and return the pod's state once that device settled: mounted,
    or stuck (the pod keeps reporting a stuck device until the next one sends its descriptor, so
    the PID tells them apart)."""
    pid = PIDS[mode]
    before = host.state().get("enumerations", 0)
    node.set_mode(mode)
    end = time.monotonic() + timeout
    while True:
        st = host.state()
        dev = st.get("device") or {}
        if dev.get("pid") == pid and ((st.get("attached") and st["enumerations"] > before)
                                      or st.get("enumeration") == "stuck"):
            return st
        if time.monotonic() > end:
            raise TimeoutError(f"{mode}: the pod never settled on 0x{pid:04x}: {st}")
        time.sleep(0.2)


def test_many_interfaces(host, node):
    st = enumerate_mode(host, node, "many")
    c = st["device"]["config"]
    assert st["attached"] and st["enumeration"] == "done"
    assert c["interface_count"] == 12
    assert len(c["interfaces"]) == 8  # the pod lists 8, counts the rest


def test_big_configuration(host, node):
    """700 bytes: more than a simple device, within what the pod reads."""
    st = enumerate_mode(host, node, "big")
    assert st["attached"] and st["device"]["pid"] == 0x4011
    assert not st["device"]["config"]["truncated"]


def test_configuration_too_long_is_reported(host, node):
    """1100 bytes, more than the pod reads: it says the enumeration is stuck instead of nothing,
    and the next device enumerates."""
    st = enumerate_mode(host, node, "huge")
    assert not st["attached"]
    assert st["enumeration"] == "stuck"
    assert st["device"]["pid"] == PIDS["huge"]  # the device descriptor did arrive
    st = enumerate_mode(host, node, "cdc")
    assert st["attached"] and st["device"]["pid"] == 0x4001


@pytest.mark.parametrize("mode", ["badlen", "overlong"])
def test_malformed_descriptor(host, node, mode):
    st = enumerate_mode(host, node, mode)
    assert st["attached"]
    assert st["device"]["config"]["malformed"]


def test_long_strings_not_ascii(host, node):
    st = enumerate_mode(host, node, "strings")
    dev = st["device"]
    assert len(dev["product"]) == 47
    assert dev["product"].startswith("abc?efghij?lm")  # e-acute and the euro sign as '?'
    assert dev["manufacturer"].startswith("Mbc?")


def test_strings_stall(host, node):
    st = enumerate_mode(host, node, "nostr")
    assert st["attached"]
    assert (st["device"]["manufacturer"], st["device"]["product"], st["device"]["serial"]) == ("", "", "")


def _start_flash(benchpod, stage, data, **args):
    stage(data)
    benchpod.command({"cmd": "usb_flash", "len": len(data), **args})


def _wait_flash(benchpod, cond, timeout=60.0):
    end = time.monotonic() + timeout
    while True:
        st = benchpod.command({"cmd": "usb_flash"})
        if cond(st):
            return st
        assert time.monotonic() < end, f"usb_flash state never matched: {st}"
        time.sleep(0.05)


def test_unplug_during_flash(host, node, benchpod, stage, usb_flash):
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    _start_flash(benchpod, stage, random.Random(5).randbytes(400 * 1024), mode="file", name="BIG.BIN")
    _wait_flash(benchpod, lambda s: s.get("running") and s.get("phase") == "data" and s.get("done", 0) > 20)
    node.cmd("detach", r"^OK detached")
    st = _wait_flash(benchpod, lambda s: not s.get("running") and s.get("have_result"))
    assert not st["ok"] and st["result"] not in ("", "ok")
    node.cmd("attach", r"^OK attached")
    host.wait_device(vid=0xCAFE, pid=0x4002, after=host.state()["enumerations"])
    data = b"after the unplug\n" * 100
    assert usb_flash(data, mode="file", name="AFTER.TXT")["ok"]
    assert node.files()["AFTER.TXT"][:2] == (len(data), zlib.crc32(data))


def test_abort(host, node, benchpod, stage, usb_flash):
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    _start_flash(benchpod, stage, random.Random(6).randbytes(400 * 1024), mode="file", name="ABORT.BIN")
    _wait_flash(benchpod, lambda s: s.get("running") and s.get("done", 0) > 10)
    benchpod.command({"cmd": "usb_flash", "abort": True})
    st = _wait_flash(benchpod, lambda s: not s.get("running") and s.get("have_result"))
    assert not st["ok"] and st["result"] == "aborted"
    assert "ABORT.BIN" not in node.files()  # the directory entry is written last
    assert usb_flash(b"x" * 5000, mode="file", name="NEXT.BIN")["ok"]


def test_timeout_returns_to_device_mode(host, node):
    """When host mode times out the pod goes back to being a USB device by itself."""
    node.set_mode("cdc")
    host.bp.command({"cmd": "usb_host", "enabled": True, "port": host.port, "timeout_s": 3})
    host.wait_device(vid=0xCAFE, pid=0x4001)
    end = time.monotonic() + 10
    while host.state()["mode"] != "device":
        assert time.monotonic() < end, f"still hosting after its timeout: {host.state()}"
        time.sleep(0.25)
    assert host.state()["port"] == "none"
    # The node sees the host go (a bus reset clears its configuration; TinyUSB reports no unmount).
    end = time.monotonic() + 5
    while node.status()["mounted"] != "0":
        assert time.monotonic() < end, "the node still thinks it is configured"
        time.sleep(0.2)
    host.start()  # back for the tests after this one
    host.wait_device(vid=0xCAFE, pid=0x4001)


def test_start_stop_cycles(host, node):
    node.set_mode("cdc")
    host.wait_device(vid=0xCAFE, pid=0x4001)  # its first mount after the reset, before counting
    mounts0 = int(node.status()["mounts"])
    for i in range(CYCLES):
        host.stop()
        end = time.monotonic() + 10
        while host.state()["mode"] != "device":
            assert time.monotonic() < end, f"cycle {i}: host mode did not stop"
            time.sleep(0.1)
        st = host.start()
        assert st["enumerations"] <= 1  # counted from this start (the node may already be in)
        st = host.wait_device(vid=0xCAFE, pid=0x4001)
        assert st["enumerations"] == 1, f"cycle {i}: {st}"
    assert int(node.status()["mounts"]) == mounts0 + CYCLES
