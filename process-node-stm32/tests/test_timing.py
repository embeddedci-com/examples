"""Timing HIL tests: the node's real-time behaviour, measured on its pins by the pod's logic analyzer.

PA8 (on LA6) marks a task the node selects with ``strobe``: high while an ADC sample or an env read
runs, toggled on every control step. From that the pod measures what a datasheet would promise:
the sampling and control rates, their jitter, the time a sample costs, and how fast the alarm output
(PB0 on LA5) follows the reading that confirms an alarm.
"""

import statistics

import pytest

from conftest import AMBIENT_C

AIN_PERIOD_S = 0.010
CTL_PERIOD_S = 0.020
ENV_PERIOD_S = 0.200
RATE_TOL = 0.02            # the node runs on its HSI: +-1 % over temperature, plus margin
AIN_JITTER_S = 0.0015      # 1 ms SysTick scheduling + the occasional VREFINT/env work in the loop
CTL_JITTER_S = 0.0025      # a control step can wait behind a 1 ms ADC burst
AIN_BUSY_S = (0.0007, 0.0014)  # 16 conversions x 61.5 us = 0.98 ms
ALARM_AFTER_READ_S = 0.0005


@pytest.fixture
def strobe(node):
    def select(mode):
        node.cmd(f"strobe {mode}", r"STROBE mode=%s" % mode)

    try:
        yield select
    finally:
        select("off")


def _periods(times):
    return [b - a for a, b in zip(times, times[1:])]


def _check_rate(periods, nominal, jitter, what):
    assert len(periods) >= 5, f"only {len(periods) + 1} {what} marks captured"
    mean = statistics.mean(periods)
    worst = max(abs(p - mean) for p in periods)
    assert abs(mean - nominal) <= RATE_TOL * nominal, \
        f"{what}: mean period {mean * 1e3:.3f} ms, nominal {nominal * 1e3:.1f} ms"
    assert worst <= jitter, \
        f"{what}: a period {worst * 1e3:.3f} ms off the mean (limit {jitter * 1e3:.1f} ms)"
    return mean, worst


@pytest.mark.hardware
def test_adc_sampling_rate_and_cost(benchpod, wiring, node, strobe):
    """100 Hz analog sampling, steady, and each oversampled reading costs about 1 ms of CPU."""
    strobe("ain")
    cap = benchpod.capture_la(65536, sample_rate_hz=50_000)  # 1.3 s, 20 us resolution
    rises = cap.edge_times(wiring.strobe, "rising")
    _check_rate(_periods(rises), AIN_PERIOD_S, AIN_JITTER_S, "ADC sample")
    busy = statistics.median(cap.pulse_widths(wiring.strobe, 1))
    lo, hi = AIN_BUSY_S
    assert lo <= busy <= hi, f"an oversampled reading takes {busy * 1e3:.3f} ms"


@pytest.mark.hardware
def test_control_rate_while_regulating(benchpod, wiring, node, strobe, plant):
    """The thermostat steps at 50 Hz while it regulates, ADC bursts and console included."""
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    strobe("ctl")
    cap = benchpod.capture_la(65536, sample_rate_hz=50_000)
    edges = cap.edge_times(wiring.strobe, "both")
    _check_rate(_periods(edges), CTL_PERIOD_S, CTL_JITTER_S, "control step")


@pytest.mark.hardware
def test_env_reading_rate(benchpod, wiring, node, env, strobe):
    strobe("env")
    cap = benchpod.capture_la(65536, sample_rate_hz=25_000)  # 2.6 s
    rises = cap.edge_times(wiring.strobe, "rising")
    _check_rate(_periods(rises), ENV_PERIOD_S, 0.003, "env read")


@pytest.mark.hardware
def test_alarm_follows_its_reading(benchpod, wiring, node, env, strobe):
    """PB0 rises right after the env read that confirms over-temperature (debounce: 2 readings)."""
    strobe("env")
    env.set_i2c_sensor(temperature_c=60.0)
    cap = benchpod.capture_la(65536, sample_rate_hz=50_000)  # 1.3 s: the alarm sets within ~0.4 s
    rise = cap.first_edge(wiring.alarm, "rising")
    assert rise is not None, "PB0 did not rise within the capture"
    reads = [t for t in cap.edge_times(wiring.strobe, "falling") if t <= rise]
    assert reads, "no env read before the alarm in the capture"
    latency = rise - reads[-1]
    assert latency <= ALARM_AFTER_READ_S, \
        f"PB0 rose {latency * 1e3:.3f} ms after the confirming read (limit {ALARM_AFTER_READ_S * 1e3} ms)"
