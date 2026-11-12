"""Emulated UART sensors on LA13 (pod to PC11, USART3 RX) and LA14 (PC10, USART3 TX, to the pod).

The GPS receiver is the sensor library's ``gps_nmea``: the node prints every NMEA sentence it
receives with its checksum verdict (``uart3 nmea on``), and the tests parse the position, check
it moves with the speed and course, and that fix and the sentence mask take effect. The CO2
sensor (sensor_defs/co2_uart.yaml, MH-Z19-like) answers requests the node sends: bytes with a
checksum, and a text line. Needs the USART3 wires (BENCH_NEW_WIRING=1).
"""

import math
import time

import pytest

from conftest import local_def, uart_pins

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_uart, pytest.mark.needs_new_wiring]

BRUSSELS = {"lat_deg": 50.8466, "lon_deg": 4.3528, "sats": 9, "utc_start": 1791462896}


def nmea_deg(value, hemi):
    """ddmm.mmmmm / dddmm.mmmmm + N/S/E/W -> signed degrees."""
    dot = value.index(".")
    deg = int(value[:dot - 2]) + float(value[dot - 2:]) / 60
    return -deg if hemi in "SW" else deg


def hhmmss_s(v):
    return int(v[0:2]) * 3600 + int(v[2:4]) * 60 + float(v[4:])


def rmc_fixes(lines):
    """(t_s, status, lat, lon) of each RMC sentence."""
    out = []
    for _, line in lines:
        f = line.split("*")[0].split(",")
        if f[0] == "$GPRMC":
            lat = nmea_deg(f[3], f[4]) if f[3] else None
            lon = nmea_deg(f[5], f[6]) if f[5] else None
            out.append((hhmmss_s(f[1]), f[2], lat, lon))
    return out


@pytest.fixture
def gps(node, wiring, sensors):
    g = sensors.start("gps_nmea", pins=uart_pins(wiring), values=BRUSSELS)
    node.uart3("open 9600")
    node.nmea("on")
    yield g
    node.nmea("off")


def test_gps_sentences_and_checksums(node, gps):
    lines = node.nmea_lines(3.5)
    assert len(lines) >= 16, f"only {len(lines)} sentences in 3.5 s at 1 Hz"
    assert all(ok for ok, _ in lines), [l for ok, l in lines if not ok]
    kinds = {line.split(",")[0] for _, line in lines}
    assert {"$GPRMC", "$GPVTG", "$GPGGA", "$GPGSA", "$GPGSV", "$GPGLL"} <= kinds, kinds
    fixes = rmc_fixes(lines)
    _, status, lat, lon = fixes[-1]
    assert status == "A" and abs(lat - 50.8466) < 1e-5 and abs(lon - 4.3528) < 1e-5, fixes[-1]
    gga = [l.split(",") for _, l in lines if l.startswith("$GPGGA")][-1]
    assert gga[6] == "1" and gga[7] == "09", gga
    st = gps.status()
    assert st["bus"]["messages"] > 0 and st["bus"]["sent_bytes"] > 0, st
    assert node.nmea()["bad"] == 0


def test_gps_position_moves_with_speed_and_course(node, gps):
    gps.set(speed_kmh=36, course_deg=90)          # 10 m/s due east
    time.sleep(1.2)                               # the next fix uses the new values
    fixes = [f for f in rmc_fixes(node.nmea_lines(4.5)) if f[1] == "A"]
    assert len(fixes) >= 4, fixes
    (t0, _, lat0, lon0), (t1, _, lat1, lon1) = fixes[0], fixes[-1]
    east_m = (lon1 - lon0) * 111320 * math.cos(math.radians(lat0))
    north_m = (lat1 - lat0) * 111320
    speed = east_m / (t1 - t0)
    assert abs(speed - 10.0) < 1.0 and abs(north_m) < 1.0, (speed, north_m, fixes)
    vtg = [l for _, l in node.nmea_lines(1.5) if l.startswith("$GPVTG")]
    assert vtg and vtg[-1].split(",")[7] == "36.000", vtg


def test_gps_fix_lost_and_sentence_mask(node, gps):
    gps.set(fix=0)
    time.sleep(1.2)
    fixes = rmc_fixes(node.nmea_lines(2.5))
    assert fixes and all(status == "V" and lat is None for _, status, lat, _ in fixes), fixes
    gps.set(fix=1, sentences=1)                   # RMC only
    time.sleep(1.2)
    lines = node.nmea_lines(2.5)
    assert lines and {l.split(",")[0] for _, l in lines} == {"$GPRMC"}, lines


# ---- a request/answer sensor ----


def _co2_answer(ppm, temp_c):
    b = [0xFF, 0x86, ppm >> 8, ppm & 0xFF, temp_c + 40, 0, 0, 0]
    return bytes(b + [(0xFF - (sum(b[1:8]) & 0xFF) + 1) & 0xFF])


READ_CO2 = "ff 01 86 00 00 00 00 00 79"


def test_co2_request_answer(node, wiring, sensors):
    co2 = sensors.start(local_def("co2_uart"), pins=uart_pins(wiring),
                        values={"co2_ppm": 1234, "temperature_c": 25})
    node.uart3("open 9600")
    node.uart3("flush")
    node.uart3(f"send {READ_CO2}")
    data, matched = node.uart3_read(t_ms=500, n=9)
    assert matched and data == _co2_answer(1234, 25), data.hex()

    co2.set(co2_ppm=415)
    node.uart3(f"send {READ_CO2}")
    data, _ = node.uart3_read(t_ms=500, n=9)
    assert data == _co2_answer(415, 25), data.hex()

    # A request with the wrong checksum byte matches nothing: no answer.
    node.uart3("send ff 01 86 00 00 00 00 00 78")
    data, _ = node.uart3_read(t_ms=300)
    assert data == b"", data.hex()
    assert co2.status()["bus"]["received_bytes"] >= 27


def test_co2_text_request(node, wiring, sensors):
    sensors.start(local_def("co2_uart"), pins=uart_pins(wiring), values={"co2_ppm": 980})
    node.uart3("open 9600")
    node.uart3("flush")
    node.uart3("line READ?")                      # sends READ?\r\n
    data, matched = node.uart3_read(t_ms=500, until="CO2=980")
    assert matched, data
    data += node.uart3_read(t_ms=200)[0]
    assert data == b"CO2=980\r\n", data
