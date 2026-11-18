"""A minimal RL78 Protocol A programmer over the BenchPod (Renesas R01AN0815): enough to drive the
node's RL78 boot ROM emulator (rl78emu.c) from the HIL tests, and an example of the pod's one-wire
UART (``half_duplex``) and pod-timed boot-mode entry (``UartEntry``).

TOOL0 is one LA channel. The pod holds it low through the target's reset, releases it, sends the
3AH single-wire mode byte, and then talks to the boot ROM on the same wire: the programmer sends
two stop bits, the target answers with one. In half duplex the pod does not read back its own
bytes, so unlike a USB-serial programmer there is no echo to strip.
"""

from __future__ import annotations

import time
from typing import List, Optional, Tuple

from embeddedci.benchpod import UartEntry

SOH, STX, ETX, ETB = 0x01, 0x02, 0x03, 0x17
ACK = 0x06
BLOCK = 1024
BAUDS = {115200: 0, 250000: 1, 500000: 2, 1000000: 3}

# The entry, timed on the pod: TOOL0 low through a 5 ms reset, low 10 ms more (the guide's tRT is
# at least 723 us + tHD; Baud Rate Set must follow within 100 ms of the reset), released, then
# the single-wire mode byte (tTM >= 16 us after release).
ENTRY = UartEntry(reset_ms=5, tx_low=True, hold_ms=10, idle_ms=1, data=b"\x3a")


def checksum(data: bytes) -> int:
    """0 minus every byte, low 8 bits (covers LEN through the last data byte)."""
    return (-sum(data)) & 0xFF


class Rl78Error(Exception):
    def __init__(self, what: str, status: int) -> None:
        super().__init__(f"{what}: status {status:02X}H")
        self.status = status


class Rl78Programmer:
    def __init__(self, bp, tool0, timeout: float = 2.0) -> None:
        self.bp = bp
        self.tool0 = tool0
        self.timeout = timeout
        self.uart = None
        self.baud = 115200
        self._rx = b""

    # -- link --------------------------------------------------------------------------------

    def _open(self, baud: int, entry: Optional[UartEntry]) -> None:
        self.uart = self.bp.open_uart(rx=self.tool0, half_duplex=True, baud=baud, stop_bits=2,
                                      entry=entry, encoding="latin-1")
        self.baud = baud
        self._rx = b""

    def enter(self, baud: int = 115200, volts: float = 3.3) -> Tuple[int, int]:
        """Reset into the boot ROM, send the mode byte, Baud Rate Set, then Reset at the new rate.
        Returns (frequency MHz, mode) from the Baud Rate Set reply."""
        self.close()
        self._open(115200, ENTRY)
        time.sleep(0.002)                       # tMB: >= 62 us after the mode byte
        freq, mode = self.baud_set(baud, volts)
        self.reset()
        return freq, mode

    def close(self) -> None:
        if self.uart is not None:
            self.uart.close()
            self.uart = None

    def _write(self, frame: bytes) -> None:
        self.uart.write(frame.decode("latin-1"))

    def _read(self, n: int, deadline: float) -> bytes:
        while len(self._rx) < n:
            left = deadline - time.monotonic()
            if left <= 0:
                raise TimeoutError(f"RL78: timed out waiting for {n} bytes, got {self._rx.hex()}")
            self._rx += self.uart.read(timeout=min(left, 0.05)).encode("latin-1")
        out, self._rx = self._rx[:n], self._rx[n:]
        return out

    def frame(self, timeout: Optional[float] = None) -> Tuple[bytes, bool]:
        """One data frame from the target: (data, last)."""
        deadline = time.monotonic() + (timeout or self.timeout)
        hdr = self._read(2, deadline)
        if hdr[0] != STX:
            raise ValueError(f"RL78: frame starts with {hdr[0]:02X}H, not STX")
        n = hdr[1] or 256
        rest = self._read(n + 2, deadline)
        data, s, ftr = rest[:n], rest[n], rest[n + 1]
        if s != checksum(hdr[1:2] + data):
            raise ValueError("RL78: bad checksum from the target")
        if ftr not in (ETX, ETB):
            raise ValueError(f"RL78: frame ends with {ftr:02X}H")
        return data, ftr == ETX

    def status(self, what: str, n: int = 1) -> bytes:
        data, _ = self.frame()
        if len(data) != n:
            raise ValueError(f"{what}: {len(data)}-byte status, expected {n}")
        return data

    def command(self, com: int, info: bytes = b"") -> None:
        body = bytes([len(info) + 1, com]) + info
        self._write(bytes([SOH]) + body + bytes([checksum(body), ETX]))

    def data_frame(self, data: bytes, last: bool) -> None:
        body = bytes([len(data) & 0xFF]) + data
        self._write(bytes([STX]) + body + bytes([checksum(body), ETX if last else ETB]))

    def _ack(self, what: str) -> None:
        st = self.status(what)[0]
        if st != ACK:
            raise Rl78Error(what, st)

    # -- commands ----------------------------------------------------------------------------

    def baud_set(self, baud: int, volts: float = 3.3) -> Tuple[int, int]:
        self.command(0x9A, bytes([BAUDS[baud], int(volts * 10)]))
        st = self.status("Baud Rate Set", 3)
        if st[0] != ACK:
            raise Rl78Error("Baud Rate Set", st[0])
        if baud != self.baud:                   # the reply came at the old rate: switch now
            self.close()
            self._open(baud, None)
        return st[1], st[2]

    def reset(self) -> None:
        self.command(0x00)
        self._ack("Reset")

    def signature(self) -> dict:
        self.command(0xC0)
        self._ack("Silicon Signature")
        d, _ = self.frame()
        le = lambda b: b[0] | b[1] << 8 | b[2] << 16
        return {"device_code": d[0:3].hex(), "name": d[3:13].decode("ascii").rstrip(),
                "code_end": le(d[13:16]), "data_end": le(d[16:19]),
                "version": f"V{d[19]}.{d[20]}{d[21]}"}

    @staticmethod
    def _range(sa: int, ea: int) -> bytes:
        return sa.to_bytes(3, "little") + ea.to_bytes(3, "little")

    def erase(self, addr: int) -> None:
        self.command(0x22, addr.to_bytes(3, "little"))
        self._ack(f"Block Erase {addr:06X}H")

    def blank_check(self, sa: int, ea: int) -> bool:
        self.command(0x32, self._range(sa, ea) + b"\x00")
        st = self.status("Block Blank Check")[0]
        if st == ACK:
            return True
        if st == 0x1B:
            return False
        raise Rl78Error("Block Blank Check", st)

    def _stream(self, what: str, data: bytes) -> List[int]:
        """Send data in 256-byte frames; return each frame's ST2."""
        st2 = []
        for off in range(0, len(data), 256):
            chunk = data[off:off + 256]
            self.data_frame(chunk, off + 256 >= len(data))
            s1, s2 = self.status(what, 2)
            if s1 != ACK:
                raise Rl78Error(what, s1)
            st2.append(s2)
            if s2 != ACK and off + 256 < len(data):
                raise Rl78Error(what, s2)
        return st2

    def program(self, sa: int, data: bytes) -> None:
        ea = sa + len(data) - 1
        self.command(0x40, self._range(sa, ea))
        self._ack("Programming")
        st2 = self._stream("Programming", data)
        if st2[-1] != ACK:
            raise Rl78Error("Programming", st2[-1])
        self._ack("Programming internal verify")

    def verify(self, sa: int, data: bytes) -> bool:
        self.command(0x13, self._range(sa, sa + len(data) - 1))
        self._ack("Verify")
        last = self._stream("Verify", data)[-1]
        if last not in (ACK, 0x0F):
            raise Rl78Error("Verify", last)
        return last == ACK

    def checksum(self, sa: int, ea: int) -> int:
        self.command(0xB0, self._range(sa, ea))
        self._ack("Checksum")
        d, _ = self.frame()
        return d[0] | d[1] << 8

    def security_get(self) -> bytes:
        self.command(0xA1)
        self._ack("Security Get")
        return self.frame()[0]

    def security_set(self, flg: int, bot: int, ss: int, se: int) -> None:
        self.command(0xA0)
        self._ack("Security Set")
        self.data_frame(bytes([flg, bot]) + ss.to_bytes(2, "little") + se.to_bytes(2, "little")
                        + b"\xff\xff", True)
        self._ack("Security Set data")
