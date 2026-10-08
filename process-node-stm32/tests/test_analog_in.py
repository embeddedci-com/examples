"""Analog-input HIL tests: the node measures a process value the pod sets.

The pod's 3.3 V DAC output stands in for a sensor (a 0-3 V transmitter, a potentiometer). The node
reads it on PA1 with 16x oversampling and VDDA correction, the way a product would. Checked: every
point of a sweep within tolerance, the overall gain and offset, the low-pass filtered process value
after a step, and that the background sampler keeps running.

Wiring: pod 3.3 V DAC SMA -> 5-10 kOhm -> PA1 (Nucleo A1), grounds common. The resistor limits the
current into an unpowered F446 (the pod keeps driving while the DUT rail is off).
"""

import time

import pytest

# Stay under the 3.3 V rail: the top of the F446 ADC is VDDA itself.
SWEEP_V = [0.10, 0.50, 1.00, 1.65, 2.20, 2.80, 3.00]
# Per point: pod DAC calibration (a few mV) + F446 ADC (+-2 LSB after averaging) + VREFINT_CAL
# (+-10 mV at 3.3 V, so ~0.3 %), with margin.
TOL_MV = 25
TOL_REL = 0.01
SETTLE_S = 0.05      # DAC + 5-10 kOhm into the sample cap: microseconds, plus one 10 ms sample period
FILTER_SETTLE_S = 3.0  # ~0.32 s time constant: 1 % after ~1.5 s


@pytest.fixture
def source(benchpod, wiring, node):
    """Set the 'sensor' voltage on PA1 through the pod's 3.3 V DAC; returns the volts achieved."""
    if not wiring.analog:
        pytest.skip("PROCESS_NODE_ANALOG=0: no pod DAC -> PA1 lead on this bench")

    def drive(volts):
        return benchpod.dac_output(wiring.ain_path, volts=volts).voltage

    try:
        yield drive
    finally:
        benchpod.dac_output("off")


def _within(measured_mv, expected_mv):
    return abs(measured_mv - expected_mv) <= max(TOL_MV, TOL_REL * expected_mv)


@pytest.mark.hardware
def test_vdda_is_sane(node, source):
    vdda = node.ain()["vdda_mv"]
    assert 3200 <= vdda <= 3400, f"VDDA from VREFINT is {vdda} mV (Nucleo runs 3.3 V)"


@pytest.mark.hardware
def test_sweep_tracks_the_source(node, source):
    """Every sweep point within tolerance, and a straight line through them: gain ~1, offset ~0."""
    points = []
    for v in SWEEP_V:
        expected_mv = source(v) * 1000.0
        time.sleep(SETTLE_S)
        points.append((expected_mv, node.ain()["mv"]))

    bad = [(round(e), m) for e, m in points if not _within(m, e)]
    assert not bad, f"(expected mV, node mV) out of tolerance: {bad}; all: {points}"

    n = len(points)
    mx = sum(e for e, _ in points) / n
    my = sum(m for _, m in points) / n
    gain = sum((e - mx) * (m - my) for e, m in points) / sum((e - mx) ** 2 for e, _ in points)
    offset = my - gain * mx
    assert 0.985 <= gain <= 1.015, f"gain {gain:.4f}, points {points}"
    assert abs(offset) <= 20, f"offset {offset:.1f} mV, points {points}"


@pytest.mark.hardware
def test_filtered_value_follows_a_step(node, source):
    """The low-passed process value lags a step, then settles on it (a few time constants)."""
    low = source(0.5) * 1000.0
    time.sleep(FILTER_SETTLE_S)
    assert _within(node.ain()["filt_mv"], low)

    high = source(2.5) * 1000.0
    start = time.monotonic()
    first = node.ain()["filt_mv"]
    assert first < high - 100, f"filter did not lag the step: {first} mV right after -> {high:.0f}"
    while time.monotonic() - start < FILTER_SETTLE_S:
        if _within(node.ain()["filt_mv"], high):
            break
    else:
        pytest.fail(f"filtered value not at {high:.0f} mV after {FILTER_SETTLE_S} s")


@pytest.mark.hardware
def test_background_sampler_runs(node, source):
    """100 Hz sampling keeps going between commands.

    The interval is taken between the midpoints of the two console round trips, so a slow link
    (the cloud adds a few hundred ms per command) does not count as extra samples."""
    t0 = time.monotonic()
    first = node.ain()["samples"]
    t1 = time.monotonic()
    time.sleep(3.0)
    t2 = time.monotonic()
    count = node.ain()["samples"] - first
    t3 = time.monotonic()
    rate = count / (((t2 + t3) - (t0 + t1)) / 2)
    assert 85 <= rate <= 115, f"{count} background samples in ~{(t2 + t3 - t0 - t1) / 2:.2f} s = {rate:.1f} Hz (expected ~100)"
