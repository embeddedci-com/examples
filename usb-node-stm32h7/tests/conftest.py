"""Fixtures for the usb-node HIL tests: the pod hosts the NUCLEO-H753ZI's USB port (CN13).

The pod is reached over the network (LAN or cloud), never its own USB: while it hosts, its USB-C
port belongs to the board, so its USB console and USB network are gone. The node's console is the
Nucleo's ST-LINK virtual COM port on this computer (USBNODE_CONSOLE, else found by VID).

USB_HOST_PORT picks the pod's host port: ``usb_c`` (default: the pod's own USB-C connector through
an adapter rig, on any pod) or ``usb_a`` (pods with a USB-A host port). The pod must run a firmware
built with ``make USB_HOST=1`` until USB host mode ships in releases.
"""

from __future__ import annotations

import os
import time
from typing import Any, Dict, Optional

import pytest

from embeddedci.benchpod import FpgaImage

from usbnode import UsbNode

USB_PORT = os.environ.get("USB_HOST_PORT", "usb_c")


def pytest_configure(config):
    config.addinivalue_line("markers", "hardware: needs the bench (pod + usb-node)")


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    return 3.3  # nothing here uses the LA, but the pod wants an I/O voltage set


@pytest.fixture(scope="session")
def node():
    n = UsbNode()
    yield n
    n.close()


class Host:
    """The pod's USB host mode on USB_PORT."""

    def __init__(self, bp):
        self.bp = bp

    def state(self) -> Dict[str, Any]:
        return self.bp.command({"cmd": "usb_host"})

    def start(self, timeout_s: int = 300) -> Dict[str, Any]:
        self.bp.command({"cmd": "usb_host", "enabled": True, "port": USB_PORT, "timeout_s": timeout_s})
        end = time.monotonic() + 10
        while True:
            st = self.state()
            if st.get("mode") == "host":
                return st
            if time.monotonic() > end:
                raise TimeoutError(f"usb_host did not reach host mode: {st}")
            time.sleep(0.2)

    def stop(self) -> None:
        self.bp.command({"cmd": "usb_host", "enabled": False})

    def wait_device(self, *, vid: Optional[int] = None, pid: Optional[int] = None,
                    after: int = 0, timeout: float = 10.0) -> Dict[str, Any]:
        """Wait until a device (with this VID/PID) is attached and enumerated more than ``after`` times."""
        end = time.monotonic() + timeout
        st: Dict[str, Any] = {}
        while time.monotonic() < end:
            st = self.state()
            dev = st.get("device") or {}
            if st.get("attached") and st.get("enumerations", 0) > after \
                    and (vid is None or dev.get("vid") == vid) and (pid is None or dev.get("pid") == pid):
                return st
            time.sleep(0.2)
        raise TimeoutError(f"no device {vid and hex(vid)}:{pid and hex(pid)} enumerated on {USB_PORT}: {st}")


@pytest.fixture(scope="session")
def host(benchpod):
    h = Host(benchpod)
    h.start()
    yield h
    h.stop()


@pytest.fixture(scope="session")
def stage(benchpod):
    """Upload bytes into the pod's PSRAM for ``usb_flash`` (needs the deep-replay gateware image)."""
    benchpod.fpga_image(FpgaImage.DEEP_REPLAY)
    put = getattr(benchpod._transport, "stage_psram", None)
    if put is None:
        pytest.skip("staging a file needs the pod over the LAN (TCP)")
    return put


@pytest.fixture
def usb_flash(benchpod, stage):
    """Stage ``data`` and run one ``usb_flash`` to its end; returns the final state."""

    def run(data: bytes, timeout: float = 60.0, **args) -> Dict[str, Any]:
        stage(data)
        benchpod.command({"cmd": "usb_flash", "len": len(data), **args})
        end = time.monotonic() + timeout
        while True:
            st = benchpod.command({"cmd": "usb_flash"})
            if not st.get("running") and st.get("have_result"):
                return st
            if time.monotonic() > end:
                benchpod.command({"cmd": "usb_flash", "abort": True})
                raise TimeoutError(f"usb_flash did not finish in {timeout} s: {st}")
            time.sleep(0.25)

    return run
