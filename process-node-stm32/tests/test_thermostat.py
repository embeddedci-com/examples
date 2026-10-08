"""Thermostat HIL tests: the node closes a PI loop around a plant the pod emulates in its FPGA.

The use case: a node holds a temperature at a setpoint by driving a heater. Here the heater drive
is the node's PA4 output and the "temperature sensor" is its PA1 input. The pod sits between them as
the process: its in-fabric control loop reads the heater drive on the ADC SMA, looks up the steady
temperature the heater would reach (a curve), lags toward it with a ~0.3 s time constant, and drives
the sensor voltage on its 3.3 V DAC SMA. No host is in that path, so the loop runs in real time.

    node PA4 (heater mV) -> pod ADC SMA -> curve + first-order lag (iCE40) -> pod 3.3 V DAC SMA
         -> 5-10 kOhm -> node PA1 (0.5 V + 20 mV/degC) -> PI (50 Hz) -> PA4

Plant: ambient 20 degC, +0.03 degC per mV of heater drive (3.0 V -> 110 degC). The same numbers drive
the host simulation in tests/sim_thermostat.c, which the controller gains were picked against.

Checked: the open-loop plant itself (heater at minimum -> ambient), settling to a setpoint without
much overshoot, holding it, a setpoint change, and rejecting a disturbance: the ambient drops 10 degC
while it runs. Loading the new curve re-arms the pod's loop, which restarts its output at vmin, and
vmin is the ambient temperature (the process cannot get colder), so the disturbance is the whole
process dropping to the new ambient at once, like a cold load put in the oven.
"""

import statistics
import time

import pytest

from conftest import AMBIENT_C, C_PER_MV

SETTLE_S = 6.0              # host sim: ~1.1 s; margin for console round trips over the cloud
IN_BAND_MS = 1000           # "settled" = 1 s continuously within 1 degC (the node counts it)
MAX_OVERSHOOT_C = 3.0
HOLD_TOL_C = 1.0


def _wait_settled(node, setpoint_c, timeout=SETTLE_S, disturbed=False):
    """Poll the node until it reports IN_BAND_MS in band; returns (seconds, peak pv, trace).

    ``disturbed``: the node was in band before a disturbance, so its in-band time must first
    drop (it saw the disturbance) before a long in-band time counts as settling again."""
    start = time.monotonic()
    trace = []
    seen = not disturbed
    while time.monotonic() - start < timeout:
        st = node.ctl()
        trace.append((round(time.monotonic() - start, 2), st["pv_c"], st["out_mv"]))
        seen = seen or st["in_band_ms"] < IN_BAND_MS
        if seen and st["in_band_ms"] >= IN_BAND_MS:
            return time.monotonic() - start, max(p for _, p, _ in trace), trace
        time.sleep(0.1)
    pytest.fail(f"not settled at {setpoint_c} degC within {timeout} s; (t, pv, out): {trace}")


def _hold(node, n=10, period=0.2):
    pvs, outs = [], []
    for _ in range(n):
        st = node.ctl()
        pvs.append(st["pv_c"])
        outs.append(st["out_mv"])
        time.sleep(period)
    return statistics.mean(pvs), statistics.mean(outs), pvs


@pytest.mark.hardware
def test_open_loop_plant_sits_at_ambient(node, plant):
    """Heater at its minimum: the emulated process settles near ambient (checks the plant alone)."""
    node.aout("200")
    plant(AMBIENT_C)
    time.sleep(1.5)
    pv = statistics.mean(node.ctl()["pv_c"] for _ in range(5))
    expected = AMBIENT_C + C_PER_MV * 200
    assert abs(pv - expected) <= 2.0, f"plant at {pv:.1f} degC with the heater at 200 mV, expected {expected:.1f}"


@pytest.mark.hardware
def test_reaches_and_holds_setpoint(node, plant):
    node.aout("200")
    plant(AMBIENT_C)
    time.sleep(1.0)
    node.ctl("on 60")
    took, peak, trace = _wait_settled(node, 60.0)
    assert peak <= 60.0 + MAX_OVERSHOOT_C, f"overshoot to {peak} degC; trace {trace}"
    pv, out, pvs = _hold(node)
    assert abs(pv - 60.0) <= HOLD_TOL_C, f"holding {pv:.2f} degC, readings {pvs}"
    expected_out = (60.0 - AMBIENT_C) / C_PER_MV
    assert abs(out - expected_out) <= 0.15 * expected_out, \
        f"heater at {out:.0f} mV to hold 60 degC; the plant needs ~{expected_out:.0f} mV"


@pytest.mark.hardware
def test_setpoint_change(node, plant):
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    _wait_settled(node, 60.0)
    node.ctl("on 40")
    _wait_settled(node, 40.0)
    pv, _, pvs = _hold(node)
    assert abs(pv - 40.0) <= HOLD_TOL_C, f"holding {pv:.2f} degC after 60 -> 40, readings {pvs}"


@pytest.mark.hardware
def test_rejects_ambient_drop(node, plant):
    """The ambient drops 10 degC while regulating: the node must add ~333 mV of heat and get back."""
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    _wait_settled(node, 60.0)
    _, out_before, _ = _hold(node, n=5)
    assert node.ctl()["in_band_ms"] >= IN_BAND_MS
    plant(AMBIENT_C - 10.0)  # reload the curve while the node keeps regulating
    _, _, trace = _wait_settled(node, 60.0, disturbed=True)
    assert min(p for _, p, _ in trace) < 60.0 - HOLD_TOL_C, f"the node never saw the drop: {trace}"
    pv, out_after, pvs = _hold(node)
    assert abs(pv - 60.0) <= HOLD_TOL_C, f"holding {pv:.2f} degC after the drop, readings {pvs}"
    extra = out_after - out_before
    assert abs(extra - 10.0 / C_PER_MV) <= 100, f"heater went up {extra:.0f} mV, expected ~333 mV"


@pytest.mark.hardware
@pytest.mark.parametrize("args", ["on 101", "on -1", "on 60.55", "on abc", "maybe"])
def test_bad_setpoint_is_refused(node, args):
    node.uart.read()
    node.uart.write(f"ctl {args}\r\n")
    assert node.uart.expect("CTL error", timeout=3.0)
