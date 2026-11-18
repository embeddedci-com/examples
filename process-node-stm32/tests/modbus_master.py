"""A Modbus RTU master over the BenchPod and a half-duplex RS-485 transceiver, the way
www.embeddedci.com/docs/benchpod-targets/5v-rs485 wires it: the pod's UART TX to the
transceiver's DI, its RX to RO, and a GPIO to DE + /RE (high: the pod drives the bus; low: it
listens). On the bench the node's RS-485 mock (rs485mock.c) stands in for transceiver, bus and
device, and enforces the same direction rules.
"""

from __future__ import annotations

import struct
import time
from typing import List, Optional


def crc16(data: bytes) -> bytes:
    """Modbus RTU CRC, low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)


class ModbusException(Exception):
    def __init__(self, fn: int, code: int) -> None:
        super().__init__(f"function {fn:02X}H: exception {code:02X}H")
        self.fn = fn
        self.code = code


class NoReply(Exception):
    pass


class ModbusMaster:
    def __init__(self, bp, *, rx, tx, de, baud: int = 19200, parity: str = "even",
                 stop_bits: int = 1, release_margin: float = 0.002) -> None:
        self.bp = bp
        self.rx, self.tx, self.de_pin = rx, tx, de
        self.baud, self.parity, self.stop_bits = baud, parity, stop_bits
        self.release_margin = release_margin
        self.uart = None
        self.de = None

    @property
    def char_s(self) -> float:
        return (1 + 8 + (self.parity != "none") + self.stop_bits) / self.baud

    def open(self) -> "ModbusMaster":
        if self.de is None and self.de_pin is not None:
            self.de = self.bp.gpio(self.de_pin, "output", level=0)   # listen until we send
        if self.uart is None:
            self.uart = self.bp.open_uart(rx=self.rx, tx=self.tx, baud=self.baud,
                                          parity=self.parity, stop_bits=self.stop_bits,
                                          encoding="latin-1")
        return self

    def close_uart(self) -> None:
        """Leave the UART (the pod runs one UART session at a time); DE stays low."""
        if self.uart is not None:
            self.uart.close()
            self.uart = None

    def close(self) -> None:
        self.close_uart()
        if self.de is not None:
            self.bp.release_gpio(self.de_pin)
            self.de = None

    # -- the wire ----------------------------------------------------------------------------

    def send(self, frame: bytes, *, drive: bool = True, hold: Optional[float] = None) -> None:
        """Drive DE, send, wait until the last byte is out, release DE. ``hold`` overrides how
        long DE stays high after the write (the tests use it to get it wrong on purpose);
        ``drive=False`` never raises DE. Without a DE pin (an automatic-direction transceiver) it
        only sends."""
        self.uart.read()                       # forget what came before
        if self.de is None:
            self.uart.write(frame.decode("latin-1"))
            return
        if drive:
            self.de.high()
        self.uart.write(frame.decode("latin-1"))
        time.sleep(len(frame) * self.char_s + self.release_margin if hold is None else hold)
        if drive:
            self.de.low()

    def receive(self, want: Optional[int] = None, timeout: float = 0.5) -> bytes:
        """Read a reply: ``want`` bytes, or as many as the reply's header says."""
        data = b""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data += self.uart.read(timeout=0.02).encode("latin-1")
            n = want if want is not None else self._reply_len(data)
            if n is not None and len(data) >= n:
                return data[:n]
        return data

    @staticmethod
    def _reply_len(d: bytes) -> Optional[int]:
        if len(d) < 3:
            return None
        fn = d[1]
        if fn & 0x80:
            return 5
        if fn in (0x01, 0x02, 0x03, 0x04):
            return 5 + d[2]
        return 8

    def transact(self, slave: int, pdu: bytes, *, want: Optional[int] = None, **send_kw) -> bytes:
        """One request and its reply PDU (function code onwards)."""
        req = bytes([slave]) + pdu
        self.send(req + crc16(req), **send_kw)
        r = self.receive(want)
        if not r:
            raise NoReply(f"no answer from slave {slave}")
        n = want if want is not None else self._reply_len(r)
        if n is None or len(r) < n:
            raise NoReply(f"short answer from slave {slave}: {r.hex()}")
        if crc16(r[:-2]) != r[-2:]:
            raise ValueError(f"bad CRC: {r.hex()}")
        if r[0] != slave:
            raise ValueError(f"answer from slave {r[0]}, asked {slave}")
        if r[1] & 0x80:
            raise ModbusException(r[1] & 0x7F, r[2])
        return r[1:-2]

    # -- functions ---------------------------------------------------------------------------

    def read_holding(self, slave: int, first: int, count: int = 1) -> List[int]:
        p = self.transact(slave, struct.pack(">BHH", 0x03, first, count))
        return list(struct.unpack(f">{count}H", p[2:2 + 2 * count]))

    def read_input(self, slave: int, first: int, count: int = 1) -> List[int]:
        p = self.transact(slave, struct.pack(">BHH", 0x04, first, count))
        return list(struct.unpack(f">{count}H", p[2:2 + 2 * count]))

    def write_register(self, slave: int, addr: int, value: int) -> None:
        self.transact(slave, struct.pack(">BHH", 0x06, addr, value))

    def write_registers(self, slave: int, first: int, values: List[int], **send_kw) -> None:
        self.transact(slave, struct.pack(">BHHB", 0x10, first, len(values), 2 * len(values))
                      + struct.pack(f">{len(values)}H", *values), **send_kw)

    def read_coils(self, slave: int, first: int, count: int) -> List[bool]:
        p = self.transact(slave, struct.pack(">BHH", 0x01, first, count))
        return [bool(p[2 + i // 8] >> (i % 8) & 1) for i in range(count)]

    def write_coil(self, slave: int, addr: int, on: bool) -> None:
        self.transact(slave, struct.pack(">BHH", 0x05, addr, 0xFF00 if on else 0))

    def write_coils(self, slave: int, first: int, values: List[bool]) -> None:
        packed = bytearray((len(values) + 7) // 8)
        for i, v in enumerate(values):
            if v:
                packed[i // 8] |= 1 << (i % 8)
        self.transact(slave, struct.pack(">BHHB", 0x0F, first, len(values), len(packed)) + bytes(packed))

    def echo(self, slave: int, data: int) -> int:
        p = self.transact(slave, struct.pack(">BHH", 0x08, 0, data))
        return struct.unpack(">H", p[3:5])[0]
