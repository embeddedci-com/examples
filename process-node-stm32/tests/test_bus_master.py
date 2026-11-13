"""The pod as a bus master (firmware with gateware v55+): the pod's I2C master on LA1/LA2 talks to
the node, which answers as an I2C target (``i2c target <addr>``: a 256-byte register file), and the
node reports what it received. Pin-direction safety (``la_dir``) is checked on the same wires.
The SPI and gpio sends drive LA6 into PA8, so they need BENCH_LA6_RESISTOR=1 like the data-ready
tests.
"""

import base64
import contextlib

import pytest

pytestmark = pytest.mark.hardware

TARGET = 0x42


def la(p):
    return int(getattr(p, "la", getattr(p, "number", p)))


def b64(data):
    return base64.urlsafe_b64encode(bytes(data)).decode().rstrip("=")


def unb64(s):
    return base64.urlsafe_b64decode(s + "=" * (-len(s) % 4)) if s else b""


def require_master(benchpod):
    if (benchpod.command({"cmd": "status"}).get("gateware") or 0) < 55:
        pytest.skip("the I2C master needs gateware v55+")


def target_mem(node, reg, n):
    m = node.cmd(f"i2c target mem {reg:02x} {n}", r"I2C (target mem=([0-9a-f ]*)|err[^\r\n]*)\r\n")
    assert m.group(2) is not None, m.group(0)
    return bytes.fromhex(m.group(2).replace(" ", ""))


def target_state(node):
    m = node.cmd("i2c target", r"I2C target on=(\d) addr=(\w+) ptr=(\w+) matches=(\d+) rx=(\d+) "
                 r"tx=(\d+) errors=(\d+)\r\n")
    return dict(on=int(m.group(1)), ptr=int(m.group(3), 16), matches=int(m.group(4)),
                rx=int(m.group(5)), tx=int(m.group(6)), errors=int(m.group(7)))


@pytest.fixture
def i2c_bus(benchpod, wiring, node):
    """The node answering at TARGET on I2C1, the pod's pull-ups on LA1/LA2. Afterwards the pod's
    session is stopped, pull-ups off, the node back to its env driver."""
    require_master(benchpod)
    node.cmd(f"i2c target {TARGET:02x}", r"I2C target on=1[^\r\n]*\r\n")
    benchpod.enable_pullup(wiring.i2c_scl, wiring.i2c_sda)
    try:
        yield SimpleBus(benchpod, wiring)
    finally:
        with contextlib.suppress(Exception):
            benchpod.command({"cmd": "i2c_stop"})
        benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)
        node.cmd("i2c target off", r"I2C target on=0[^\r\n]*\r\n")
        node.env("on")


class SimpleBus:
    def __init__(self, benchpod, wiring):
        self.bp = benchpod
        self.sda, self.scl = la(wiring.i2c_sda), la(wiring.i2c_scl)

    def start(self, hz, pullups="pod"):
        return self.bp.command({"cmd": "i2c_start", "sda": self.sda, "scl": self.scl, "hz": hz,
                                "pullups": pullups})

    def xfer(self, addr, write=b"", read=0):
        req = {"cmd": "i2c_xfer", "addr": addr}
        if write:
            req["write"] = b64(write)
        if read:
            req["read"] = read
        return self.bp.command(req)


@pytest.mark.parametrize("hz", [100_000, 400_000, 1_000_000])
def test_i2c_master_write_then_read(i2c_bus, node, hz):
    r = i2c_bus.start(hz)
    assert r["sda"] == i2c_bus.sda and r["scl"] == i2c_bus.scl
    data = bytes((0x10 + i) * 3 & 0xFF for i in range(16))
    w = i2c_bus.xfer(TARGET, bytes([0x20]) + data)
    assert w["ack"], w
    assert target_mem(node, 0x20, len(data)) == data
    rd = i2c_bus.xfer(TARGET, bytes([0x20]), read=len(data))
    assert rd["ack"], rd
    assert unb64(rd["rx"]) == data
    # untouched memory reads back mem[i] = i
    rd = i2c_bus.xfer(TARGET, bytes([0x80]), read=4)
    assert unb64(rd["rx"]) == bytes([0x80, 0x81, 0x82, 0x83])
    assert target_state(node)["errors"] == 0


def test_i2c_master_long_read_and_write(i2c_bus, node):
    i2c_bus.start(400_000)
    data = bytes((i * 7 + 1) & 0xFF for i in range(128))
    assert i2c_bus.xfer(TARGET, bytes([0x00]) + data[:127])["ack"]
    rd = i2c_bus.xfer(TARGET, bytes([0x00]), read=248)
    assert rd["ack"]
    got = unb64(rd["rx"])
    assert len(got) == 248
    assert got[:127] == data[:127]
    assert got[127:] == bytes(range(127, 248))


def test_i2c_master_nack_and_scan(i2c_bus, node):
    i2c_bus.start(100_000)
    r = i2c_bus.xfer(0x50, b"\x00", read=2)
    assert r["ack"] is False and r["nack_at"] == 0 and not r.get("rx")
    probe = i2c_bus.xfer(TARGET)
    assert probe["ack"] is True
    scan = i2c_bus.bp.command({"cmd": "i2c_scan"})
    assert scan["found"] == [TARGET]


def test_i2c_master_needs_pullups(benchpod, wiring, node):
    require_master(benchpod)
    benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)
    with pytest.raises(Exception, match=r"pull-up"):
        benchpod.command({"cmd": "i2c_start", "sda": la(wiring.i2c_sda), "scl": la(wiring.i2c_scl),
                          "pullups": "pod"})


def test_la_dir_refuses_push_pull_on_dut_output(benchpod, wiring, node):
    """LA5 is the node's alarm output (PB0, push-pull): a gpio output there is refused once the
    wiring says so; open drain and input are fine."""
    require_master(benchpod)
    alarm = la(wiring.alarm)
    try:
        benchpod.command({"cmd": "la_dir", "la": alarm, "dir": "dut_output"})
        with pytest.raises(Exception, match=r"direction conflict"):
            benchpod.command({"cmd": "gpio", "la": alarm, "mode": "output", "level": 0})
        benchpod.command({"cmd": "gpio", "la": alarm, "mode": "input"})
        benchpod.command({"cmd": "gpio", "la": alarm, "mode": "off"})
    finally:
        with contextlib.suppress(Exception):
            benchpod.command({"cmd": "gpio", "la": alarm, "mode": "off"})
        benchpod.command({"cmd": "la_dir", "la": alarm, "dir": "unknown"})


def capture_with_send(benchpod, send, *, samples, rate_hz, trigger_la, edge="falling"):
    """la_capture with a ``send`` (the SDK has no field for it yet): (words, rate_hz, send result)."""
    from embeddedci.benchpod import capture as cap
    req = {"cmd": "la_capture", "samples": samples, "sample_rate_mhz": rate_hz / 1e6,
           "trigger": {"la": trigger_la, "edge": edge, "pre_samples": 64}, "trigger_timeout_ms": 2000, "send": send}
    edges, upto, rate, result = [], 0, rate_hz, None
    for chunk in cap._stream_or_samples(benchpod._transport, req):
        if chunk.get("la_rate_hz"):
            rate = float(chunk["la_rate_hz"])
        if "send" in chunk:
            result = chunk["send"]
        if chunk.get("la"):
            edges.extend([int(e[0]), int(e[1])] for e in (chunk.get("la_edges") or []))
            upto = max(upto, int(chunk.get("la_upto", 0)))
    return cap._expand_la_edges(edges, upto or samples), rate, result


def scl_hz(words, scl, rate):
    bit = 1 << (scl - 1)
    rises = [i for i in range(1, len(words)) if (words[i] & bit) and not (words[i - 1] & bit)]
    assert len(rises) >= 18, f"only {len(rises)} SCL rising edges"
    periods = sorted(b - a for a, b in zip(rises, rises[1:]))
    return rate / periods[len(periods) // 2]   # the median: a byte's ACK/stretch gaps are rarer


@pytest.mark.parametrize("hz,rate,want", [(100_000, 12e6, 100_000), (400_000, 12e6, 363_636),
                                          (1_000_000, 48e6, 923_077)])
def test_send_and_capture_i2c(i2c_bus, benchpod, node, hz, rate, want):
    i2c_bus.start(hz)
    assert i2c_bus.xfer(TARGET, bytes([0x10, 0xA5, 0x5A]))["ack"]
    words, got_rate, res = capture_with_send(
        benchpod, {"bus": "i2c", "addr": TARGET, "write": b64([0x10]), "read": 2},
        samples=8192, rate_hz=rate, trigger_la=i2c_bus.sda)
    assert res is not None, "no send result in the capture"
    assert res.get("ack") is True and unb64(res["rx"]) == bytes([0xA5, 0x5A]), res
    tx = benchpod.decode(words, "i2c", sda=i2c_bus.sda, scl=i2c_bus.scl)
    msgs = [m for t in tx for m in t.messages]
    assert [(m.address, m.read, m.values) for m in msgs[:2]] == [
        (TARGET, False, [0x10]), (TARGET, True, [0xA5, 0x5A])], msgs
    # The master counts SCL high only once SCL reads high (that is what lets a target stretch the
    # clock), so the bus's rise time adds to every period: never faster than the rate the pod
    # reports, somewhat slower on a real bus (this bench: 333 kHz at 400k, 774 kHz at 1M).
    f = scl_hz(words, i2c_bus.scl, got_rate)
    assert want * 0.75 < f < want * 1.04, f"SCL {f:.0f} Hz, want at most about {want}"


def test_send_refused_without_session(benchpod, wiring, node):
    require_master(benchpod)
    with pytest.raises(Exception, match=r"i2c_start"):
        capture_with_send(benchpod, {"bus": "i2c", "addr": TARGET, "read": 1}, samples=1024,
                          rate_hz=12e6, trigger_la=la(wiring.i2c_sda))


@pytest.fixture
def free_spi(benchpod, wiring, node):
    """Four pins the node does not drive, for the pod's SPI master: SCK on LA1 (PB8) and MOSI on
    LA2 (PB9) with the node's I2C1 idle, MISO on LA5 (the node's alarm output: an input for the
    pod, so only read), CS on LA6 (PA8, an input while the strobe is off)."""
    require_master(benchpod)
    node.env("off")
    pins = dict(sck=la(wiring.i2c_scl), mosi=la(wiring.i2c_sda), miso=la(wiring.alarm),
                cs=la(wiring.strobe))
    benchpod.command({"cmd": "la_dir", "la": pins["miso"], "dir": "dut_output"})
    try:
        yield pins
    finally:
        with contextlib.suppress(Exception):
            benchpod.command({"cmd": "spi_stop"})
        benchpod.command({"cmd": "la_dir", "la": pins["miso"], "dir": "unknown"})
        node.env("on")


@pytest.mark.needs_la6_resistor
@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_spi_modes_send_and_capture(benchpod, free_spi, mode):
    r = benchpod.command({"cmd": "spi_start", **free_spi, "hz": 1_000_000, "mode": mode})
    assert r["mode"] == mode
    tx = bytes([0x9F, 0x5A, 0xC3, 0x01])
    words, rate, res = capture_with_send(benchpod, {"bus": "spi", "tx": b64(tx)}, samples=4096,
                                         rate_hz=12e6, trigger_la=free_spi["cs"])
    assert res is not None and "rx" in res, res
    sck_bit = 1 << (free_spi["sck"] - 1)
    assert bool(words[0] & sck_bit) == bool(mode >> 1), "SCK idle level does not match CPOL"
    frames = benchpod.decode(words, "spi", sclk=free_spi["sck"], mosi=free_spi["mosi"],
                             cs=free_spi["cs"], mode=mode, sample_rate_hz=rate)
    got = bytes(f.mosi for f in frames)
    assert got == tx, frames


@pytest.mark.needs_la6_resistor
def test_send_gpio(benchpod, wiring, node):
    """A gpio send drives the pin right after the arm: LA6 into PA8 (an input, strobe off)."""
    require_master(benchpod)
    pin = la(wiring.strobe)
    benchpod.command({"cmd": "gpio", "la": pin, "mode": "output", "level": 1})
    try:
        words, rate, res = capture_with_send(benchpod, {"bus": "gpio", "la": pin, "level": 0},
                                             samples=1024, rate_hz=12e6, trigger_la=pin)
        assert res == {"bus": "gpio", "la": pin, "level": 0}
        bit = 1 << (pin - 1)
        assert words[0] & bit and not words[-1] & bit
    finally:
        benchpod.command({"cmd": "gpio", "la": pin, "mode": "off"})
