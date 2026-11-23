"""The pod enumerates every device the usb-node plays, and sees it come and go."""

import pytest

pytestmark = pytest.mark.hardware

# mode: (vid, pid, product, interface classes)
MODES = {
    "cdc": (0xCAFE, 0x4001, "usb-node cdc", ["cdc", "cdc_data"]),
    "msc": (0xCAFE, 0x4002, "usb-node msc", ["msc"]),
    "uf2": (0xCAFE, 0x4003, "usb-node uf2", ["msc"]),
    "teensy": (0x16C0, 0x0478, "", ["hid"]),
}


@pytest.mark.parametrize("mode", list(MODES))
def test_enumerates(host, node, mode):
    vid, pid, product, classes = MODES[mode]
    before = host.state().get("enumerations", 0)
    node.mark()
    node.set_mode(mode)
    st = host.wait_device(vid=vid, pid=pid, after=before)
    node.wait(r"^EV mount", timeout=5)
    dev = st["device"]
    assert dev["speed"] == "full"
    assert dev["product"] == product
    if mode != "teensy":
        assert dev["manufacturer"] == "EmbeddedCI"
        assert len(dev["serial"]) == 24
    assert [i["class_name"] for i in dev["config"]["interfaces"]] == classes
    assert not dev["config"]["truncated"] and not dev["config"]["malformed"]
    assert dev["mounted_ms"] >= dev["descriptor_ms"]


def test_replug_counts(host, node):
    node.set_mode("cdc")
    first = host.wait_device(vid=0xCAFE, pid=0x4001)["enumerations"]
    for i in range(3):
        node.cmd("replug 300", r"^OK replugged")
        st = host.wait_device(vid=0xCAFE, pid=0x4001, after=first + i)
    assert st["enumerations"] == first + 3


def test_detach_is_seen(host, node):
    node.set_mode("msc")
    host.wait_device(vid=0xCAFE, pid=0x4002)
    node.cmd("detach", r"^OK detached")
    import time
    end = time.monotonic() + 5
    while host.state().get("attached") and time.monotonic() < end:
        time.sleep(0.2)
    assert not host.state()["attached"]
    assert host.state()["device"]["pid"] == 0x4002  # kept after the detach
    node.cmd("attach", r"^OK attached")
    host.wait_device(vid=0xCAFE, pid=0x4002)
