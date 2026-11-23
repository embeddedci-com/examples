"""Console client for the usb-node firmware (the Nucleo's ST-LINK virtual COM port, 115200 8N1)."""

from __future__ import annotations

import glob
import os
import re
import time
from typing import List, Optional

import serial

#: ST-LINK V3 on the NUCLEO-H753ZI (STMicroelectronics VID).
STLINK_VID = 0x0483


def find_console() -> str:
    """The node's console: USBNODE_CONSOLE, else the one ST-LINK virtual COM port on this computer."""
    port = os.environ.get("USBNODE_CONSOLE")
    if port:
        return port
    from serial.tools import list_ports

    found = [p.device for p in list_ports.comports() if p.vid == STLINK_VID]
    if not found:  # pyserial cannot always read the VID on macOS: fall back to the device names
        found = sorted(glob.glob("/dev/cu.usbmodem*"))
    if len(found) != 1:
        raise RuntimeError(f"set USBNODE_CONSOLE to the NUCLEO-H753ZI's ST-LINK port (found {found or 'none'})")
    return found[0]


class UsbNode:
    """One open console. Every line is kept, so a test can wait for an event (``EV ...``) that
    arrived while it was doing something else."""

    def __init__(self, port: Optional[str] = None, baud: int = 115200):
        self.port = port or find_console()
        self.ser = serial.Serial(self.port, baud, timeout=0.05)
        self.lines: List[str] = []
        self._buf = b""
        self._seen = 0
        time.sleep(0.5)  # the ST-LINK VCP drops the first bytes right after opening
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def _pump(self) -> None:
        data = self.ser.read(4096)
        if not data:
            return
        self._buf += data
        *done, self._buf = self._buf.split(b"\n")
        self.lines += [d.decode(errors="replace").rstrip("\r") for d in done]

    def mark(self) -> None:
        """Forget earlier lines for :meth:`wait`."""
        self._pump()
        self._seen = len(self.lines)

    def wait(self, pattern: str, timeout: float = 5.0) -> re.Match:
        """The first line since the last :meth:`mark` (or match) that matches ``pattern``."""
        rx = re.compile(pattern)
        end = time.monotonic() + timeout
        while True:
            while self._seen < len(self.lines):
                line = self.lines[self._seen]
                self._seen += 1
                m = rx.search(line)
                if m:
                    return m
            if time.monotonic() > end:
                tail = "\n".join(self.lines[-15:])
                raise TimeoutError(f"no line matching {pattern!r} from the usb-node in {timeout} s; last lines:\n{tail}")
            self._pump()

    def cmd(self, line: str, expect: str, timeout: float = 5.0) -> re.Match:
        self.mark()
        self.ser.write(line.encode() + b"\n")
        return self.wait(expect, timeout)

    def status(self) -> dict:
        m = self.cmd("status", r"^STATUS (.*)")
        words = m.group(1).split()
        return dict(zip(words[0::2], words[1::2]))

    def set_mode(self, mode: str, timeout: float = 5.0) -> None:
        """Switch the node to ``mode`` (it resets) and wait for its boot."""
        self.cmd(f"mode {mode}", rf"^(OK mode {mode}|ERR)")
        self.wait(rf"usb-node-stm32h7 \S+ mode {mode}( bridge \S+)?$", timeout)
        self.wait(r"^APP_OK$", timeout)

    def set_bridge(self, kind: str, timeout: float = 5.0) -> None:
        """Put a USB-serial chip (ftdi, cp210x, ch34x) in front of the serial modes, or none."""
        self.cmd(f"bridge {kind}", rf"^(OK bridge {kind}|ERR)")
        self.wait(rf"usb-node-stm32h7 \S+ mode \S+ bridge {kind}$", timeout)
        self.wait(r"^APP_OK$", timeout)

    def ensure_mode(self, mode: str) -> None:
        """Make sure the node is in ``mode``, freshly booted (an empty drive, counters at zero)."""
        self.set_mode(mode)

    def hard_reset(self) -> None:
        """Reset the board through its ST-LINK (st-flash), e.g. out of the ROM bootloader."""
        import subprocess
        self.mark()
        subprocess.run(["st-flash", "reset"], check=False, capture_output=True, timeout=30)
        self.wait(r"^APP_OK$", 10)

    def files(self) -> dict:
        """The drive's root directory: {name: (size, crc32)}."""
        self.cmd("files", r"^FILE|^FILES")
        self._seen -= 1
        out = {}
        while True:
            m = self.wait(r"^(FILE (\S+) (\d+) ([0-9a-f]{8})(.*)|FILES \d+)$")
            if m.group(1).startswith("FILES"):
                return out
            out[m.group(2)] = (int(m.group(3)), int(m.group(4), 16), m.group(5).strip())
