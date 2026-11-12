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
    m = node.cmd("i2c recover", r"I2C recover sda_before=(\d) (ok|stuck)")
    assert m.group(2) == "ok", m.group(0)
    wait_for(lambda: node.i2c_try("rd 76 d0 1")[0], lambda d: d == b"\x60", timeout=2.0,
             what="the bus to work again")


@pytest.mark.sensors_spi
@pytest.mark.needs_new_wiring
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


@pytest.mark.sensors_uart
@pytest.mark.needs_new_wiring
def test_uart_drop(node, gps):
    node.nmea("on")
    time.sleep(1.2)
    base = sum(1 for ok, _ in node.nmea_lines(5.0) if ok)
    gps.fault("drop", value=0.5)
    time.sleep(1.0)
    kept = node.nmea_lines(5.0)
    assert all(ok for ok, _ in kept), "a dropped sentence is dropped whole, never cut"
    ratio = len(kept) / base
    assert 0.2 <= ratio <= 0.8, f"{len(kept)} of about {base} sentences with drop 0.5"
    gps.clear_fault()
