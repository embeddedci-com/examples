"""Alarm HIL tests: fault conditions the pod injects reach every output the node has.

The node watches an environment sensor (a BMP280 on I2C, emulated by the pod on LA1/LA2), its
process sensor (PA1) and the sensor's presence. When an alarm sets or clears it drives PB0 (wired to
LA5), prints an ``EVT alarm`` line and sends CAN frame 0x0A0 ``[active, changed, temp lo, temp hi]``;
over-temperature and a process-sensor fault also trip the thermostat (heater off, latched).

Checked: the BMP280 readings, over-temperature on all three outputs with its hysteresis, a short
spike that the debounce ignores, the heater trip and the refusal to restart while hot, a shorted
process sensor while regulating, and the sensor going missing and coming back.
"""

import re
import time

import pytest

from conftest import AMBIENT_C, BITRATE, ENV_ADDR, ENV_START_C, wait_alarm

ALARM_CAN_ID = 0x0A0
# Env read every 200 ms, debounce 2 evaluations: <= ~0.6 s plus command latency.
ALARM_LATENCY_S = 2.0
# Lost: 3 failed reads (0.6 s) + debounce; back: probe every 1 s + first read + debounce.
LOST_LATENCY_S = 3.0


def _alarm_pin(benchpod, wiring):
    return benchpod.read_gpio(wiring.alarm)


def _expect_evt(node, text, timeout=ALARM_LATENCY_S):
    """Wait for an asynchronous ``EVT ...`` line from the node."""
    return node.uart.expect(re.compile(re.escape(text)), timeout=timeout)


@pytest.mark.hardware
def test_env_sensor_readings(node, env):
    a = node.alarm()
    assert abs(a["press_pa"] - 101325) <= 50, a
    env.set_i2c_sensor(temperature_c=31.5, pressure_pa=95000.0)
    a = wait_alarm(node, lambda a: abs(a["env_c"] - 31.5) <= 0.2, timeout=2.0, what="31.5 degC")
    assert abs(a["press_pa"] - 95000) <= 50, a
    assert a["active"] == 0, a


@pytest.mark.hardware
def test_overtemp_reaches_every_output(benchpod, wiring, node, env):
    node.init(BITRATE)
    with benchpod.open_can(bitrate=BITRATE, mode="normal", term=True) as bus:
        assert _alarm_pin(benchpod, wiring) == 0
        node.uart.read()
        env.set_i2c_sensor(temperature_c=60.0)

        assert benchpod.wait_for_level(wiring.alarm, 1, timeout=ALARM_LATENCY_S), "PB0 never went high"
        _expect_evt(node, "EVT alarm set=overtemp active=0x01")
        f = bus.expect(can_id=ALARM_CAN_ID, timeout=ALARM_LATENCY_S)
        temp_dc = int.from_bytes(f.data[2:4], "little", signed=True)
        assert (f.data[0], f.data[1]) == (0x01, 0x01) and abs(temp_dc - 600) <= 2, str(f)

        # Inside the 2 degC hysteresis band: stays set.
        env.set_i2c_sensor(temperature_c=49.0)
        time.sleep(1.0)
        assert node.alarm()["overtemp"] == 1
        assert _alarm_pin(benchpod, wiring) == 1

        node.uart.read()
        env.set_i2c_sensor(temperature_c=47.0)
        assert benchpod.wait_for_level(wiring.alarm, 0, timeout=ALARM_LATENCY_S), "PB0 stayed high"
        _expect_evt(node, "EVT alarm clear=overtemp active=0x00")
        f = bus.expect(can_id=ALARM_CAN_ID, timeout=ALARM_LATENCY_S)
        assert (f.data[0], f.data[1]) == (0x00, 0x01), str(f)


@pytest.mark.hardware
def test_short_spike_is_debounced(benchpod, wiring, node, env):
    """One hot reading is not an alarm: two consecutive ones are."""
    start = time.monotonic()
    env.set_i2c_sensor(temperature_c=60.0)
    env.set_i2c_sensor(temperature_c=ENV_START_C)
    spike = time.monotonic() - start
    if spike > 0.15:
        pytest.skip(f"the spike lasted {spike:.2f} s on this link; the node reads every 0.2 s")
    time.sleep(1.0)
    a = node.alarm()
    assert a["active"] == 0 and a["events"] == 0, a
    assert _alarm_pin(benchpod, wiring) == 0


@pytest.mark.hardware
def test_overtemp_trips_the_heater(node, env, plant):
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    time.sleep(1.0)
    assert node.ctl()["mode"] == "on"

    node.uart.read()
    env.set_i2c_sensor(temperature_c=70.0)
    _expect_evt(node, "EVT ctl trip=overtemp heater=off")
    st = node.ctl()
    assert (st["mode"], st["trip"], st["out_mv"]) == ("off", "overtemp", 0), st
    assert node.aout()["mode"] == "off"

    # No restart while it is still hot...
    node.uart.read()
    node.uart.write("ctl on 60\r\n")
    assert node.uart.expect("CTL error: over-temperature", timeout=3.0)
    # ...and a clean one once it has cooled.
    env.set_i2c_sensor(temperature_c=ENV_START_C)
    wait_alarm(node, lambda a: a["active"] == 0, timeout=ALARM_LATENCY_S, what="overtemp to clear")
    st = node.ctl("on 60")
    assert (st["mode"], st["trip"]) == ("on", "none"), st


@pytest.mark.hardware
def test_shorted_sensor_trips_the_heater(benchpod, wiring, node, env, plant):
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    time.sleep(1.0)

    node.uart.read()
    plant.short_sensor()  # PA1 to 0 V while regulating
    _expect_evt(node, "EVT alarm set=pv_fault")
    _expect_evt(node, "EVT ctl trip=pv_fault heater=off")
    st = node.ctl()
    assert (st["mode"], st["trip"]) == ("off", "pv_fault"), st
    # Not regulating any more, so the fault is no longer an alarm (the trip stays latched).
    _expect_evt(node, "EVT alarm clear=pv_fault")
    assert benchpod.wait_for_level(wiring.alarm, 0, timeout=ALARM_LATENCY_S)
    assert node.ctl()["trip"] == "pv_fault"


@pytest.mark.hardware
def test_env_sensor_lost_and_back(benchpod, wiring, node, env):
    node.uart.read()
    benchpod.disable_i2c_sensor()
    _expect_evt(node, "EVT alarm set=env_lost active=0x04", timeout=LOST_LATENCY_S)
    assert _alarm_pin(benchpod, wiring) == 1
    assert node.alarm()["env"] == "lost"

    node.uart.read()
    benchpod.enable_i2c_sensor("bmp280", sda=wiring.i2c_sda, scl=wiring.i2c_scl,
                               address=ENV_ADDR, temperature_c=ENV_START_C)
    _expect_evt(node, "EVT alarm clear=env_lost active=0x00", timeout=LOST_LATENCY_S)
    assert _alarm_pin(benchpod, wiring) == 0
    assert node.alarm()["env"] == "ok"


@pytest.mark.hardware
@pytest.mark.parametrize("args", ["limit 90", "limit -41", "limit 50.25", "limit", "bogus"])
def test_bad_limit_is_refused(node, args):
    node.uart.read()
    node.uart.write(f"alarm {args}\r\n")
    assert node.uart.expect("ALARM error", timeout=3.0)
