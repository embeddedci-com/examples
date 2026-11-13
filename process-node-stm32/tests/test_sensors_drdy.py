"""A data-ready line: the pod drives an emulated sensor's data-ready output on LA6, which reaches
PA8 through a 1 kOhm resistor, and the node times it with ``drdy`` while its strobe is off (PA8 is
then an input). The sensor is sensor_defs/drdy_counter.yaml on I2C: RATE (0x10) sets the sample
rate, each sample raises the line, in pulse mode for 50 us, in level mode until the node reads DATA
(0x20). Tests where the pod drives LA6 need the resistor (BENCH_LA6_RESISTOR=1).
"""

import pytest

from conftest import i2c_pins, local_def
from embeddedci.benchpod.sensors import load_definition

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_drdy]

ADDR = 0x3C


def start(sensors, wiring, mode="pulse"):
    doc = load_definition(local_def("drdy_counter"))
    doc["data_ready"]["mode"] = mode
    return sensors.start(doc, instance="drdy_counter", pins=i2c_pins(wiring, drdy=wiring.drdy),
                         values={"reading": -1234})


@pytest.mark.needs_la6_resistor
def test_pulse_mode(node, wiring, sensors):
    start(sensors, wiring, "pulse")
    assert node.i2c_rd(ADDR, 0x0F, 1) == b"\xd7"
    assert node.drdy()["level"] == 0, "no samples yet: the line rests low"
    idle = node.drdy("count 300")
    assert idle["rises"] == 0, idle

    node.i2c_wr(ADDR, 0x10, [0x02])               # 50 samples per second
    c = node.drdy("count 1000")
    assert 45 <= c["rises"] <= 55 and c["falls"] >= c["rises"] - 1, c
    assert 20 <= c["high_min_us"] and c["high_max_us"] <= 200, f"pulse width (50 us): {c}"
    assert 15000 <= c["period_min_us"] and c["period_max_us"] <= 25000, f"period (20 ms): {c}"

    node.i2c_wr(ADDR, 0x10, [0x03])               # 100 per second
    c = node.drdy("count 1000")
    assert 92 <= c["rises"] <= 108, c
    w = node.drdy("wait rise 100")
    assert w["result"] == "ok" and w["us"] <= 12000, w

    node.i2c_wr(ADDR, 0x10, [0x00])               # off again: no more pulses
    assert node.drdy("count 300")["rises"] == 0


@pytest.mark.needs_la6_resistor
def test_level_mode_cleared_on_read(node, wiring, sensors):
    start(sensors, wiring, "level")
    node.i2c_wr(ADDR, 0x10, [0x01])               # 10 samples per second
    w = node.drdy("wait high 500")
    assert w["result"] == "ok" and w["level"] == 1, w
    node.i2c_wr(ADDR, 0x10, [0x00])               # stop sampling: the level must hold by itself
    assert node.drdy()["level"] == 1, "level mode: high until the data is read"
    assert node.drdy("count 300")["falls"] == 0

    assert node.i2c_rd(ADDR, 0x20, 2) == (-1234).to_bytes(2, "big", signed=True)
    assert node.drdy()["level"] == 0, "reading DATA (clear_on_read) released the line"
    assert node.drdy("count 300")["rises"] == 0


def test_drdy_refused_while_strobe_drives_pa8(node, sensors):
    """While a strobe mode drives PA8 the node will not read it as an input. Nothing drives LA6
    from the pod here, so this runs without the resistor."""
    node.cmd("strobe ain", r"STROBE mode=ain\b[^\r\n]*\r\n")
    m = node.cmd("drdy", r"DRDY ([^\r\n]*)\r\n")
    assert m.group(1).startswith("err strobe is on"), m.group(1)
    node.cmd("strobe off", r"STROBE mode=off\b[^\r\n]*\r\n")
    assert node.drdy()["result"] == "level"
