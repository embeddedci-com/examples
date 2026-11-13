"""Values over time: a stimulus file the pod plays into an emulated sensor while the node reads it.

sensor_defs/ramp_temperature.yaml holds 20 degC for 1 s, ramps to 40 degC over 4 s and holds; the
node's env driver reads the BME280 every 200 ms and must follow. sensor_defs/fault_window.yaml
switches a no_response fault on from 1 s to 2.5 s, so the node's reads fail exactly there.
"""

import time

import pytest

from conftest import i2c_pins, local_def, wait_for

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_i2c, pytest.mark.sensors_stimulus]


def test_ramp_read_back_over_time(node, wiring, sensors):
    sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76,
                  values={"temperature_c": 20.0})
    node.env("on")
    wait_for(node.env, lambda e: e["status"] == "ok" and e["reads"] > 0, timeout=4.0,
             what="the env driver to find the BME280")
    sensors.load_stimulus(local_def("ramp_temperature"))
    sensors.start_stimulus()
    t0 = time.monotonic()
    samples = []
    while time.monotonic() - t0 < 6.5:
        samples.append((time.monotonic() - t0, node.env()["temp_c"]))
        time.sleep(0.2)
    st = sensors.status()["stimulus"]
    assert st["loaded"], st

    temps = [t for _, t in samples]
    assert abs(temps[0] - 20.0) <= 0.3, samples[:3]
    assert abs(temps[-1] - 40.0) <= 0.2, samples[-3:]
    # Rises with the ramp (a reading lags by up to one 200 ms period), never back down.
    assert all(b >= a - 0.2 for a, b in zip(temps, temps[1:])), samples
    mid = [t for s, t in samples if 2.8 <= s <= 3.2]
    assert mid and all(25.0 <= t <= 35.0 for t in mid), f"half way up the ramp at 3 s: {mid}"
    assert sum(1 for t in temps if 21.0 < t < 39.0) >= 10, "the ramp is seen in steps, not a jump"


@pytest.mark.sensors_faults
def test_fault_window_from_the_stimulus(node, wiring, sensors):
    sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76)
    sensors.load_stimulus(local_def("fault_window"))
    sensors.start_stimulus()
    t0 = time.monotonic()
    results = []
    while time.monotonic() - t0 < 4.0:
        data, err = node.i2c_try("rd 76 d0 1")
        results.append((time.monotonic() - t0, err or data.hex()))
    early = [r for s, r in results if s < 0.8]
    window = [r for s, r in results if 1.3 < s < 2.2]
    late = [r for s, r in results if s > 2.9]
    assert early and all(r == "60" for r in early), early
    assert window and all(r == "nack_addr phase=w" for r in window), window
    assert late and all(r == "60" for r in late), late
