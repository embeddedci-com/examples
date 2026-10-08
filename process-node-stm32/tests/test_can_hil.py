"""Hardware-in-the-loop CAN tests: BenchPod <-> NUCLEO-F446RE + MCP2515 node.

The pod and the Nucleo are two real nodes on one bus, so this exercises what the pod's
loopback tests cannot: ACKs from another controller, both directions, periodic traffic,
bursts, error counters and bitrate changes. The Nucleo runs ``build/process-node.elf`` and is
driven over its UART console; the pod side uses the SDK's CAN API.

Bench wiring (edit ``wiring`` below to match yours):

    Pod CAN+ / CAN-  <->  MCP2515 module H / L (module 120R jumper removed: the pod's switchable
                          120R is the only termination, so the termination test can see it)
    GND              <->  common ground between pod, Nucleo and module
    SWCLK -> LA11, SWDIO -> LA12, NRST -> the pod's reset pin
    UART (USART1): pod samples the DUT's TX on LA3, drives the DUT's RX on LA4
    Target power: internal-5V eFuse, which also feeds the module through the Nucleo's 5V pin.
    The Nucleo's ST-LINK holds the F446 in reset for ~2.2 s after power-up.

Run (flashes first when --benchpod-firmware is given, otherwise uses what is on the board):

    pytest process-node-stm32/tests -v --benchpod-connection=<host or embeddedci:name> \\
        --benchpod-firmware=process-node-stm32/build/process-node.elf

Set ``CAN_NODE_OSC_MHZ=16`` for a module with a 16 MHz crystal (also enables the 1 Mbit/s case),
``CAN_NODE_TERM=1`` if the module's 120R jumper is fitted.
"""

import os
import time

import pytest

from conftest import BITRATE, OSC_MHZ

# The module's own 120R jumper fitted? Off on this bench, so the pod's switch is the only termination.
MODULE_TERM = os.environ.get("CAN_NODE_TERM", "0") == "1"
POD_RX_RING = 31  # frames the pod buffers between reads (32-slot ring, one kept empty)


@pytest.fixture
def can(benchpod, node):
    """The pod's CAN in normal mode at BITRATE, termination on; node reset to defaults."""
    node.init(BITRATE)
    with benchpod.open_can(bitrate=BITRATE, mode="normal", term=True) as bus:
        yield bus


@pytest.mark.hardware
def test_node_selftest(node):
    """MCU <-> MCP2515 SPI path works on its own (MCP2515 internal loopback, nothing on the bus)."""
    node.init(BITRATE)
    node.cmd("can selftest", r"CAN selftest ok")


@pytest.mark.hardware
@pytest.mark.parametrize("can_id,data,ext", [
    (0x123, b"\x01\x02", False),
    (0x000, b"", False),
    (0x7FF, bytes(range(8)), False),
    (0x1ABCDE0, b"\xAA\x55", True),
    (0x1FFFFFFF, bytes(range(0xF8, 0x100)), True),
])
def test_pod_to_node(can, node, can_id, data, ext):
    """Pod transmits, the node receives and ACKs (pod TEC stays 0)."""
    can.write(can_id, data, ext=ext)
    node.expect_rx(can_id, data, ext=ext)
    st = can.status()
    assert st["tec"] == 0 and not st["bus_off"], f"pod saw TX errors: {st}"


@pytest.mark.hardware
@pytest.mark.parametrize("line,can_id,data,ext", [
    ("can send 321 11 22 33", 0x321, b"\x11\x22\x33", False),
    ("can send 7FF", 0x7FF, b"", False),
    ("can sendx 5 01", 0x5, b"\x01", True),
    ("can send 1ABCDE0 DE AD BE EF 00 11 22 33", 0x1ABCDE0, bytes.fromhex("DEADBEEF00112233"), True),
])
def test_node_to_pod(can, node, line, can_id, data, ext):
    """Node transmits, the pod receives and ACKs (node reports tx ok)."""
    node.cmd(line, r"CAN tx ok")
    f = can.expect(can_id=can_id, timeout=2.0)
    assert (f.data, f.ext) == (data, ext), str(f)


@pytest.mark.hardware
def test_echo_round_trip(can, node):
    """Request/response: the node answers every frame with id+1 and the same data."""
    node.cmd("can echo on", r"CAN echo on")
    for i in range(5):
        payload = bytes([i, 0xA5, i ^ 0xFF])
        can.write(0x200, payload)
        reply = can.expect(can_id=0x201, timeout=2.0)
        assert reply.data == payload, str(reply)


@pytest.mark.hardware
def test_node_periodic(can, node):
    """The node broadcasts every 100 ms; the pod's ISR timestamps confirm the period."""
    node.cmd("can periodic 300 100", r"CAN periodic on")
    try:
        frames = can.assert_periodic(0x300, 0.100, tol=0.2, min_count=8, duration=1.5)
    finally:
        node.cmd("can periodic off", r"CAN periodic off")
    counters = [f.data[0] | (f.data[1] << 8) for f in frames]
    assert counters == list(range(counters[0], counters[0] + len(counters))), \
        f"periodic frames lost or reordered: {counters}"


@pytest.mark.hardware
def test_burst_fits_pod_ring(can, node):
    """A back-to-back burst smaller than the pod's RX ring arrives complete and in order."""
    n = 20
    node.cmd(f"can burst {n} 400", r"CAN burst done sent=%d failed=0" % n)
    frames = can.collect(1.5, match=0x400)
    counters = [f.data[0] | (f.data[1] << 8) for f in frames]
    assert counters == list(range(n)), f"burst frames: {counters}"


@pytest.mark.hardware
def test_burst_overflow_is_counted(benchpod, can, node):
    """A burst bigger than the pod's RX ring: every frame is either delivered or counted as dropped."""
    n = 60
    node.cmd(f"can burst {n} 401", r"CAN burst done sent=%d failed=0" % n)
    got, dropped = 0, 0
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        r = benchpod.can_read(max_frames=8)
        got += sum(1 for f in r.frames if f.id == 0x401)
        dropped = r.overflow  # cumulative since can_config (the fixture just configured)
        if not r.frames and got + dropped >= n:
            break
    assert dropped > 0, f"expected overflow with {n} frames > ring {POD_RX_RING}; got {got}"
    assert got + dropped == n, f"delivered {got} + dropped {dropped} != sent {n}"


@pytest.mark.hardware
def test_pod_listen_mode_does_not_ack(benchpod, node):
    """Pod in listen mode sniffs the node's frame but never ACKs it, so the node's TX fails."""
    node.init(BITRATE)
    with benchpod.open_can(bitrate=BITRATE, mode="listen", term=True) as bus:
        node.cmd("can send 123 01", r"CAN tx fail: timeout", timeout=3.0)
        assert int(node.status()["tec"]) > 0
        f = bus.expect(can_id=0x123, timeout=2.0)  # retransmissions were still seen
        assert f.data == b"\x01"


@pytest.mark.hardware
def test_pod_tx_without_ack_raises_errors(can, node):
    """Node in listen-only: the pod's frame gets no ACK, its TEC climbs. Back to normal: delivered."""
    node.cmd("can mode listen", r"CAN mode listen ok")
    can.write(0x456, [0x42])
    node.expect_rx(0x456, b"\x42")  # a listen-only node still receives
    time.sleep(0.3)
    st = can.status()
    assert st["tec"] > 0 or st["error_passive"], f"pod TEC did not rise without an ACK: {st}"
    node.cmd("can mode normal", r"CAN mode normal ok")
    time.sleep(0.3)  # the pod's pending retransmission now gets ACKed
    assert not can.status()["bus_off"]


@pytest.mark.hardware
@pytest.mark.parametrize("on", [True, False, True, False])
def test_termination_switch(benchpod, node, on):
    """The pod's switchable 120R is the bus's only termination (module jumper removed):
    on, traffic flows both ways without errors; off, the unterminated bus carries nothing.

    With the module's 120R fitted (CAN_NODE_TERM=1) the bus works either way, so the test then
    only checks that switching doesn't disturb it.
    """
    node.init(BITRATE)
    with benchpod.open_can(bitrate=BITRATE, mode="normal", term=on) as bus:
        assert bus.status()["term"] is on
        bus.write(0x150, [int(on)])
        if on or MODULE_TERM:
            node.expect_rx(0x150, bytes([int(on)]))
            node.cmd("can send 151 5A", r"CAN tx ok")
            assert bus.expect(can_id=0x151, timeout=2.0).data == b"\x5A"
            st = bus.status()
            assert st["tec"] == 0 and st["rec"] == 0, f"errors with term={on}: {st}"
        else:
            node.cmd("can send 151 5A", r"CAN tx fail")
            assert bus.read_until(can_id=0x151, timeout=0.5) is None
            # The unACKed frame retransmits until bus-off; the pod then recovers and zeroes TEC,
            # which a slow (cloud) status read can land after.
            st = bus.status()
            assert st["tec"] > 0 or st["bus_off"] or st["bus_off_recoveries"] > 0, \
                f"pod frame got through an unterminated bus: {st}"


BITRATES = [125_000, 250_000, 500_000] + ([1_000_000] if OSC_MHZ == 16 else [])


@pytest.mark.hardware
@pytest.mark.parametrize("bitrate", BITRATES)
def test_bitrates(benchpod, node, bitrate):
    """Both directions at each bitrate both controllers can hit exactly."""
    node.init(bitrate)
    with benchpod.open_can(bitrate=bitrate, mode="normal", term=True) as bus:
        bus.write(0x111, [bitrate >> 16 & 0xFF])
        node.expect_rx(0x111, bytes([bitrate >> 16 & 0xFF]))
        node.cmd("can send 222 5A", r"CAN tx ok")
        assert bus.expect(can_id=0x222, timeout=2.0).data == b"\x5A"


@pytest.mark.hardware
def test_bitrate_mismatch_fails(benchpod, node):
    """Node at 250k, pod at 500k: nothing gets through and the node records TX errors."""
    node.init(250_000)
    with benchpod.open_can(bitrate=BITRATE, mode="normal", term=True) as bus:
        node.cmd("can send 333 01", r"CAN tx fail", timeout=3.0)
        assert int(node.status()["txfail"]) >= 1
        assert bus.read_until(can_id=0x333, timeout=0.5) is None
