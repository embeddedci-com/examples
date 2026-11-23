"""Injected faults: what a node sees when its sensor fails, and that it recovers once the fault ends.

I2C (on LA1/LA2): no_response (the address is not acknowledged), corrupt (a broken SHT4x CRC),
stuck (readings freeze), bus_stuck (SDA held low, the node frees the bus afterwards). SPI: MISO
released. UART: silence, garbage instead of sentences, and dropped sentences. Each fault is
cleared (or times out) inside its test, and the ``sensors`` fixture stops everything after it.
"""

import time

import pytest

from conftest import crc8_sensirion, i2c_pins, local_def, spi_pins, uart_pins, wait_for

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_faults]


@pytest.mark.sensors_i2c
def test_i2c_no_response(node, wiring, sensors):
    bme = sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76)
    assert node.i2c_rd(0x76, 0xD0, 1) == b"\x60"
    bme.fault("no_response")
    data, err = node.i2c_try("rd 76 d0 1")
    assert data is None and err == "nack_addr phase=w", err
    assert 0x76 not in node.i2c_scan()
    assert any(f.get("type") == "no_response" for f in bme.status()["faults"]), bme.status()
    bme.clear_fault()
    assert node.i2c_rd(0x76, 0xD0, 1) == b"\x60"


@pytest.mark.sensors_i2c
def test_i2c_corrupt_crc(node, wiring, sensors):
    """The SHT4x answer to 0xFD sits at 0xFD..0x02 (it wraps); its temperature CRC is at 0xFF."""
    sht = sensors.start("sht4x", pins=i2c_pins(wiring))
    sht.fault("corrupt", at=0xFF, mask=0x01)
    r = node.i2c("44 w fd d 10 r 6")
    assert crc8_sensirion(r[0:2]) != r[2], f"temperature CRC should be broken: {r.hex()}"
    assert crc8_sensirion(r[3:5]) == r[5], f"humidity CRC untouched: {r.hex()}"
    sht.clear_fault()
    r = node.i2c("44 w fd d 10 r 6")
    assert crc8_sensirion(r[0:2]) == r[2] and crc8_sensirion(r[3:5]) == r[5], r.hex()


@pytest.mark.sensors_i2c
def test_stuck_readings(node, wiring, sensors):
    bme = sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76,
                        values={"temperature_c": 25.0})
    node.env("on")
    wait_for(node.env, lambda e: e["status"] == "ok" and abs(e["temp_c"] - 25.0) <= 0.1,
             timeout=4.0, what="25 degC")
    bme.fault("stuck")
    bme.set(temperature_c=35.0)
    time.sleep(1.0)
    assert abs(node.env()["temp_c"] - 25.0) <= 0.1, "stuck: the old reading stays"
    bme.clear_fault()
    wait_for(node.env, lambda e: abs(e["temp_c"] - 35.0) <= 0.1, timeout=2.0, what="35 degC")


@pytest.mark.sensors_i2c
def test_bus_stuck_recovers(node, wiring, sensors):
    """SDA held low for 0.5 s: transfers fail (busy bus or timeout, never a wrong answer), and
    afterwards the node's driver resets its peripheral and the bus works again."""
    bme = sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76)
    bme.fault("bus_stuck", duration_s=0.5)
    data, err = node.i2c_try("rd 76 d0 1")
    assert data is None and err.split()[0] in ("bus_busy", "timeout", "arb_lost", "bus_error"), \
        (data, err)
    time.sleep(0.8)
    m = node.cmd("i2c recover", r"I2C recover sda_before=(\d) (ok|stuck)\r\n")
    assert m.group(2) == "ok", m.group(0)
    wait_for(lambda: node.i2c_try("rd 76 d0 1")[0], lambda d: d == b"\x60", timeout=2.0,
             what="the bus to work again")


@pytest.mark.sensors_spi
@pytest.mark.sensors_spi   # skips when the probe finds no SPI2 wires (conftest spi2_wired)
def test_spi_no_response(node, wiring, sensors):
    accel = sensors.start(local_def("spi_accel"), pins=spi_pins(wiring))
    assert node.spi([0x8F], read=1)[1:] == b"\x33"
    accel.fault("no_response")
    released = node.spi([0x8F], read=2)[1:]
    assert released in (b"\xff\xff", b"\x00\x00"), f"MISO released reads the bias, got {released.hex()}"
    accel.clear_fault()
    assert node.spi([0x8F], read=1)[1:] == b"\x33"


@pytest.fixture
def gps(node, wiring, sensors):
    g = sensors.start("gps_nmea", pins=uart_pins(wiring), values={"lat_deg": 50.85, "lon_deg": 4.35})
    node.uart3("open 9600")
    yield g
    node.nmea("off")


@pytest.mark.sensors_uart
@pytest.mark.needs_new_wiring
def test_uart_no_response(node, gps):
    n, _ = node.uart3_count(1500)
    assert n > 300, f"{n} bytes in 1.5 s from a 1 Hz GPS"
    gps.fault("no_response")
    time.sleep(0.6)                               # let a sentence in flight finish
    n, _ = node.uart3_count(2000)
    assert n == 0, f"silent, yet {n} bytes arrived"
    gps.clear_fault()
    n, _ = node.uart3_count(2000)
    assert n > 300, n


@pytest.mark.sensors_uart
@pytest.mark.needs_new_wiring
def test_uart_garbage(node, gps):
    node.nmea("on")
    gps.fault("garbage", rate_hz=200)
    time.sleep(0.5)
    node.nmea("on")                               # counters from here
    lines = node.nmea_lines(2.5)
    st = node.nmea()
    assert not [l for ok, l in lines if ok], f"valid sentences during garbage: {lines}"
    assert st["junk"] + st["bad"] > 0 or node.uart3_stat()["rx"] > 300, st
    gps.clear_fault()
    time.sleep(1.2)
    lines = node.nmea_lines(2.5)
    assert [l for ok, l in lines if ok], "sentences again after the fault"


def _epochs(lines):
    """GPS epochs among the node's NMEA lines: one $GPRMC per 1 Hz epoch."""
    return sum(1 for ok, line in lines if ok and line.startswith("$GPRMC"))


def _bus(gps):
    bus = gps.status()["bus"]
    return bus["messages"], bus["dropped"]


@pytest.mark.sensors_uart
@pytest.mark.needs_new_wiring
def test_uart_drop(node, gps):
    """drop 0.5: the pod skips whole 1 Hz epochs (about 7 sentences each), each with chance 0.5.

    Over 12 epochs some are kept and some dropped: none kept, or as many as the 12 s without the
    fault, has a chance of 0.05% to 0.2% (a 5 s window with a 0.2-0.8 sentence ratio failed
    about 6% of the time). What the node received is then checked against
    the pod's own counters (bus.messages: epochs sent, bus.dropped: epochs skipped), not against
    the 0.5 the coin should give."""
    window = 12.0
    node.nmea("on")
    time.sleep(1.2)
    total = _epochs(node.nmea_lines(window))
    assert total >= 10, f"{total} epochs in {window:.0f} s from a 1 Hz GPS"

    gps.fault("drop", value=0.5)
    time.sleep(1.0)
    sent0, dropped0 = _bus(gps)
    lines = node.nmea_lines(window)
    sent1, dropped1 = _bus(gps)
    gps.clear_fault()

    assert all(ok for ok, _ in lines), "a dropped sentence is dropped whole, never cut"
    kept = _epochs(lines)
    sent, dropped = sent1 - sent0, dropped1 - dropped0
    assert 0 < kept < total, f"{kept} of {total} epochs kept with drop 0.5 (pod: sent {sent}, dropped {dropped})"
    assert sent > 0 and dropped > 0, f"pod counters over the window: sent {sent}, dropped {dropped}"
    # the counters are read just before and after the window, so an epoch at either edge may
    # count on one side only
    assert abs(kept - sent) <= 2, f"node saw {kept} epochs, the pod sent {sent}"
    assert sent + dropped >= 10, f"pod: {sent} sent + {dropped} dropped epochs in {window:.0f} s"
