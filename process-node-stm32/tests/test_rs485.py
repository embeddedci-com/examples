"""Modbus RTU over RS-485, against the node's RS-485 mock (rs485mock.c, ``modbus start``).

The pod is wired as the docs wire it to a half-duplex transceiver, and the node plays
transceiver, bus and device in one: LA8 (pod TX, the transceiver's DI) -> PC11, LA7 (pod RX, RO)
<- PC10, LA6 (a pod GPIO on DE + /RE) -> PA8 through 1 kOhm. With DE honored (``de on``) a byte the
pod sends with DE low never reaches the device, and a reply byte while DE is still high never
reaches the pod, as on a real bus. The protocol tests run with ``de off``, an automatic-direction
transceiver, so they do not depend on the host's DE timing.

Measured on the bench (LAN): a GPIO command from the host takes about 27 ms, so DE drops about
25 ms after the request's last byte. A device that answers sooner loses its reply to a master
that switches DE from the host; the DE tests use a device that answers after 100 ms, and
test_turnaround reports where it breaks. Needs the USART3 wires and the LA6 resistor
(BENCH_NEW_WIRING=1 BENCH_LA6_RESISTOR=1).
"""

from __future__ import annotations

import re
import struct
import time

import pytest

from modbus_master import ModbusException, ModbusMaster, NoReply, crc16

pytestmark = [pytest.mark.hardware, pytest.mark.rs485, pytest.mark.needs_new_wiring,
              pytest.mark.needs_la6_resistor]

SLAVE = 1


class Bus:
    """The master and the mock's console take turns: the pod runs one UART session at a time, and
    the node's console (LA3/LA4) and the master (LA7/LA8) are two. Switching closes one and opens
    the other; neither the node nor the mock is reset by it."""

    def __init__(self, benchpod, wiring, node):
        self.bp, self.wiring, self.node = benchpod, wiring, node
        self.master = None

    def cmd(self, line, pattern):
        if self.master is not None:
            self.master.close_uart()
        if self.node.uart is None:
            self.node.open()
        return self.node.cmd(line, pattern)

    def stat(self):
        m = self.cmd("modbus stat", r"MODBUS stat ([^\r\n]*)\r\n")
        return {k: v for k, v in re.findall(r"(\w+)=(\S+)", m.group(1))}

    def start(self, baud=19200, fmt="8E1", delay_us=5000, addr=SLAVE, parity=None, stop_bits=None,
              de=False, pod_de=False):
        """Start the mock and set the master up for the same line (or for parity / stop_bits).
        ``de``: the master switches DE on LA6 and the mock enforces it; else neither does."""
        if self.master is not None:
            self.master.close()
        self.cmd(f"modbus start {addr} {baud} {fmt} delay {delay_us} de {'on' if de else 'off'}",
                 r"MODBUS ok on")
        par = {"N": "none", "E": "even", "O": "odd"}[fmt[1]] if parity is None else parity
        self.master = ModbusMaster(self.bp, rx=self.wiring.sensor_uart_rx,
                                   tx=self.wiring.sensor_uart_tx, de=self.wiring.drdy if de else None,
                                   baud=baud, pod_de=pod_de,
                                   parity=par, stop_bits=int(fmt[2]) if stop_bits is None else stop_bits)
        return self.master

    @property
    def m(self):
        """The master, its UART open (the console closed)."""
        self.node.close()
        return self.master.open()

    def stop(self):
        if self.master is not None:
            self.master.close()
            self.master = None
        self.cmd("modbus stop", r"MODBUS ok off")


DE_DELAY_US = 100000   # a device slow enough for a master that switches DE from the host


@pytest.fixture
def bus(benchpod, wiring, node):
    """The mock at 19200 8E1 (Modbus RTU's default), answering 5 ms after a request, through an
    automatic-direction transceiver (no DE)."""
    b = Bus(benchpod, wiring, node)
    b.start()
    try:
        yield b
    finally:
        b.stop()


def test_registers(bus):
    m = bus.m
    assert m.read_holding(SLAVE, 0, 4) == [0, 0, 0, 0]
    m.write_register(SLAVE, 3, 0xBEEF)
    m.write_registers(SLAVE, 10, [1, 2, 3, 0xFFFF])
    assert m.read_holding(SLAVE, 0, 14)[3] == 0xBEEF
    assert m.read_holding(SLAVE, 10, 4) == [1, 2, 3, 0xFFFF]
    assert m.read_input(SLAVE, 0, 16) == [0x1000 + i for i in range(16)]
    # the node's own view of the same table, both ways
    assert bus.cmd("modbus reg 3", r"MODBUS reg i=3 v=(\d+)\r\n").group(1) == str(0xBEEF)
    bus.cmd("modbus reg 20 4660", r"MODBUS reg i=20 v=4660\r\n")
    assert bus.m.read_holding(SLAVE, 20) == [4660]


def test_coils_and_echo(bus):
    m = bus.m
    m.write_coil(SLAVE, 2, True)
    m.write_coils(SLAVE, 8, [True, False, True, True, False, False, False, False, True])
    got = m.read_coils(SLAVE, 0, 17)
    assert [i for i, v in enumerate(got) if v] == [2, 8, 10, 11, 16]
    assert m.echo(SLAVE, 0xA55A) == 0xA55A


@pytest.mark.parametrize("pdu,code", [
    (struct.pack(">BHH", 0x2B, 0, 0), 0x01),        # no such function
    (struct.pack(">BHH", 0x03, 63, 2), 0x02),       # past the last holding register
    (struct.pack(">BHH", 0x03, 0, 0), 0x03),        # a quantity of 0
    (struct.pack(">BHH", 0x05, 0, 0x1234), 0x03),   # a coil value other than FF00H / 0000H
], ids=["function", "address", "quantity", "coil-value"])
def test_exceptions(bus, pdu, code):
    with pytest.raises(ModbusException) as err:
        bus.m.transact(SLAVE, pdu)
    assert err.value.code == code


def test_silent_cases(bus):
    """No answer to a bad CRC, to another slave, or to a broadcast (which is still carried out)."""
    m = bus.m
    req = bytes([SLAVE]) + struct.pack(">BHH", 0x03, 0, 1)
    m.send(req + bytes([crc16(req)[0], crc16(req)[1] ^ 1]))
    assert m.receive(timeout=0.2) == b""
    with pytest.raises(NoReply):
        m.read_holding(2, 0, 1)
    req = bytes([0]) + struct.pack(">BHH", 0x06, 5, 777)
    m.send(req + crc16(req))
    assert m.receive(timeout=0.2) == b""
    assert m.read_holding(SLAVE, 5) == [777]
    s = bus.stat()
    assert (s["crc"], s["other"], s["broadcast"], s["frame"]) == ("1", "1", "1", "0")


def test_de_never_raised(bus):
    """The pod sends with its driver off: nothing reaches the device, nothing comes back."""
    bus.start(delay_us=DE_DELAY_US, de=True)
    m = bus.m
    req = bytes([SLAVE]) + struct.pack(">BHH", 0x03, 0, 1)
    m.send(req + crc16(req), drive=False)
    assert m.receive(timeout=0.3) == b""
    s = bus.stat()
    assert int(s["undriven"]) == len(req) + 2 and s["requests"] == "0"


def test_de_driven_right(bus):
    """The master switches DE around every request, the device answers after DE_DELAY_US."""
    bus.start(delay_us=DE_DELAY_US, de=True)
    m = bus.m
    m.write_registers(SLAVE, 0, [11, 22, 33])
    assert m.read_holding(SLAVE, 0, 3) == [11, 22, 33]
    s = bus.stat()
    assert (s["undriven"], s["collisions"], s["frame"]) == ("0", "0", "0")


def test_de_released_too_early(bus):
    """DE drops while the request is still going out: the tail never reaches the device."""
    bus.start(delay_us=DE_DELAY_US, de=True)
    with pytest.raises(NoReply):
        bus.m.write_registers(SLAVE, 0, list(range(60)), hold=0.02)   # 129 bytes, ~74 ms
    s = bus.stat()
    assert int(s["undriven"]) > 0 and s["replies"] == "0"


def test_de_released_too_late(bus):
    """DE stays high after the device has started to answer: the pod's receiver is off, the reply
    is lost (and on a real bus it would collide with the pod's driver)."""
    bus.start(delay_us=20000, de=True)
    m = bus.m
    req = bytes([SLAVE]) + struct.pack(">BHH", 0x03, 0, 1)
    m.send(req + crc16(req), hold=0.15)            # the device answers after 20 ms
    assert m.receive(timeout=0.2) == b""
    assert bus.stat()["collisions"] == "7"


@pytest.mark.parametrize("baud,fmt", [(9600, "8N2"), (19200, "8E1"), (38400, "8O1"), (115200, "8N1")])
def test_frame_formats(bus, baud, fmt):
    bus.start(baud=baud, fmt=fmt)
    m = bus.m
    m.write_registers(SLAVE, 0, [baud & 0xFFFF, 0x1234])
    assert m.read_holding(SLAVE, 0, 2) == [baud & 0xFFFF, 0x1234]
    assert bus.stat()["frame"] == "0"


def test_wrong_parity_is_dropped(bus):
    """A master at 8N2 against a device at 8E1: the device drops every frame."""
    bus.start(fmt="8E1", parity="none", stop_bits=2)
    with pytest.raises(NoReply):
        bus.m.read_holding(SLAVE, 0, 1)
    s = bus.stat()
    assert s["replies"] == "0" and int(s["frame"]) >= 1


@pytest.mark.parametrize("delay_us", [100000, 50000, 30000, 20000, 5000])
def test_turnaround(bus, delay_us):
    """How soon a device may answer before this master, which switches DE through the pod's
    network API, loses the reply. Only the 50 ms and slower cases must pass; the rest report."""
    bus.start(delay_us=delay_us, de=True)
    m = bus.m
    ok = 0
    for _ in range(10):
        try:
            m.read_holding(SLAVE, 0, 1)
            ok += 1
        except (NoReply, ValueError):   # nothing, or the tail of a reply whose start was lost
            pass
    print(f"device answering after {delay_us} us: {ok}/10 replies, collisions {bus.stat()['collisions']}")
    if delay_us >= 50000:
        assert ok == 10


def test_docs_example(bus):
    """The read_holding_registers helper from the 5 V and RS-485 docs page, unchanged, against a
    device slow enough for host-switched DE (DE_DELAY_US)."""
    BAUD = 19200

    def read_holding_registers(uart, de, slave, first, count=1, timeout=1.0):
        request = struct.pack(">BBHH", slave, 0x03, first, count)
        request += crc16(request)
        de.high()                                   # drive the bus
        uart.write(request)
        time.sleep(len(request) * 11 / BAUD + 0.002)  # 11 bits a byte at 8E1
        de.low()                                    # release it: the device answers
        want = 5 + 2 * count                        # address, function, length, data, CRC
        reply = b""
        deadline = time.monotonic() + timeout
        while len(reply) < want and time.monotonic() < deadline:
            reply += uart.read(timeout=0.05).encode("latin-1")
        assert len(reply) >= want, f"no answer from slave {slave}: {reply.hex()}"
        reply = reply[:want]
        assert crc16(reply[:-2]) == reply[-2:], f"bad CRC: {reply.hex()}"
        assert reply[1] == 0x03, f"exception {reply[2]} from slave {slave}"
        return list(struct.unpack(f">{count}H", reply[3:3 + 2 * count]))

    bus.start(baud=BAUD, delay_us=DE_DELAY_US, de=True)
    bus.cmd("modbus reg 0 4242", r"MODBUS reg i=0 v=4242\r\n")
    m = bus.m                                       # the console closed, DE claimed and low
    m.close_uart()
    with bus.bp.open_uart(rx=bus.wiring.sensor_uart_rx, tx=bus.wiring.sensor_uart_tx, baud=BAUD,
                          parity="even", encoding="latin-1") as uart:
        assert read_holding_registers(uart, m.de, SLAVE, 0) == [4242]
        assert not bus.bp.status().get("uart_rx_parity_error")


@pytest.mark.parametrize("baud,fmt", [(9600, "8N2"), (19200, "8E1"), (115200, "8N1")])
@pytest.mark.parametrize("delay_us", [0, 1000, 5000])
def test_pod_de(bus, baud, fmt, delay_us):
    """The pod switches DE itself (``open_uart(de=...)``, gateware v58): the transceiver rules on,
    and the device answering as soon as the spec allows (3.5 characters after the request, with
    ``delay 0``), after 1 ms and after 5 ms. No reply byte may be lost and nothing undriven."""
    if not bus.bp.supports("uart_de"):
        pytest.skip("the pod's firmware does not drive DE (uart_de, gateware v58)")
    bus.start(baud=baud, fmt=fmt, delay_us=delay_us, de=True, pod_de=True)
    m = bus.m
    for i in range(10):
        m.write_registers(SLAVE, 0, [i, 0x5A5A, baud & 0xFFFF])
        assert m.read_holding(SLAVE, 0, 3) == [i, 0x5A5A, baud & 0xFFFF]
    s = bus.stat()
    assert (s["undriven"], s["collisions"], s["frame"], s["replies"]) == ("0", "0", "0", "20")

