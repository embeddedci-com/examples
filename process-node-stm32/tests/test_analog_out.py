"""Analog-output HIL tests: the node drives an actuator signal and the pod measures it.

PA4 (the F446's buffered 12-bit DAC) is the node's actuator output, the way a product drives a
valve or heater setpoint. The pod's ADC on its front SMA is the instrument: DC levels, a sine from
the node's DDS (frequency, amplitude, offset), the range limits, and that the output is really off
(high-Z) when not in use, because other pod tests share that SMA and expect it quiet.

Wiring: PA4 (Nucleo A2) -> pod ADC SMA centre, shell to ground.
"""

import math
import time

import pytest

DC_MV = [300, 1000, 1650, 2500, 3000]
# F446 DAC (+-~10 mV offset, buffered) + pod front-SMA path (/12 divider, calibrated) + noise.
DC_TOL_MV = 50
DC_TOL_REL = 0.02
SETTLE_S = 0.05
OFF_MAX_V = 0.1
# The node's clock is the HSI (+-1 %), its DDS adds nothing; the pod times the capture.
FREQ_TOL_REL = 0.02
AMP_TOL_REL = 0.05
OFFSET_TOL_MV = 60


@pytest.fixture
def actuator(benchpod, wiring, node):
    """The node's analog output, measured on the pod's ADC SMA; switched off afterwards."""
    if not wiring.analog:
        pytest.skip("PROCESS_NODE_ANALOG=0: no PA4 -> pod ADC lead on this bench")
    node.aout("off")
    try:
        yield node
    finally:
        node.aout("off")
        benchpod.analog_path("off")


def _read_v(benchpod, wiring):
    return benchpod.adc_read(wiring.aout_source).voltage


@pytest.mark.hardware
def test_output_off_is_high_z(benchpod, wiring, actuator):
    """Off means off: the SMA reads ~0 V, so the pod's own analog tests keep a quiet input."""
    v = _read_v(benchpod, wiring)
    assert abs(v) < OFF_MAX_V, f"PA4 off but the pod ADC SMA reads {v:.3f} V"


@pytest.mark.hardware
def test_dc_levels(benchpod, wiring, actuator):
    bad, seen = [], []
    for mv in DC_MV:
        actuator.aout(str(mv))
        time.sleep(SETTLE_S)
        got = _read_v(benchpod, wiring) * 1000.0
        seen.append((mv, round(got)))
        if abs(got - mv) > max(DC_TOL_MV, DC_TOL_REL * mv):
            bad.append((mv, round(got)))
    assert not bad, f"(set mV, pod mV) out of tolerance: {bad}; all: {seen}"


@pytest.mark.hardware
@pytest.mark.parametrize("hz", [10, 50, 200])
def test_sine(benchpod, wiring, actuator, hz):
    amp_mv, offset_mv = 1000, 1650
    actuator.aout(f"sine {hz} {amp_mv} {offset_mv}")
    time.sleep(0.1)
    # >= 20 cycles, >= 100 samples per cycle
    rate = max(2000, 100 * hz)
    samples = max(4096, int(20 * rate / hz))
    cap = benchpod.capture_adc(samples, sample_rate_hz=rate, source=wiring.aout_source)

    mid = cap.mean()
    edges = cap.crossing_times(mid, "rising", hysteresis=0.2)
    assert len(edges) >= 5, f"{len(edges)} rising edges in {cap.duration:.2f} s: no sine on the SMA"
    freq = (len(edges) - 1) / (edges[-1] - edges[0])
    amp = cap.rms_ac() * math.sqrt(2) * 1000.0

    assert abs(freq - hz) <= FREQ_TOL_REL * hz, f"frequency {freq:.2f} Hz, set {hz} Hz"
    assert abs(amp - amp_mv) <= AMP_TOL_REL * amp_mv, f"amplitude {amp:.0f} mV, set {amp_mv} mV"
    assert abs(mid * 1000.0 - offset_mv) <= OFFSET_TOL_MV, \
        f"offset {mid * 1000:.0f} mV, set {offset_mv} mV"


@pytest.mark.hardware
@pytest.mark.parametrize("args", ["3300", "100", "sine 50 2000 1650", "sine 0 500 1650",
                                  "sine 5000 500 1650", "nonsense"])
def test_out_of_range_is_refused(actuator, args):
    """A request the output cannot do is refused and leaves the output off."""
    actuator.uart.read()
    actuator.uart.write(f"aout {args}\r\n")
    assert actuator.uart.expect("AOUT error", timeout=3.0)
    assert actuator.aout()["mode"] == "off"
