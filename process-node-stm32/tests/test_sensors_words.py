"""Word-register and 16-bit-command I2C sensors from the library, read by the node over I2C1.

TI parts (TMP117, INA219) address 16-bit registers: a pointer byte, then MSB and LSB. Sensirion
parts (SHT3x, SCD4x) take 16-bit commands whose high bytes differ. The pod serves both from its
emulation image (word registers and the command map, firmware 3.10.2). The node reads them the
way their drivers do and decodes them with each datasheet's formulas.
"""

import time

import pytest

from conftest import crc8_sensirion, i2c_pins

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_i2c]


def _word(b):
    return (b[0] << 8) | b[1]


def _s16(v):
    return v - 0x10000 if v & 0x8000 else v


def _sensirion_words(data, n):
    words = []
    for i in range(n):
        chunk = data[3 * i:3 * i + 3]
        assert crc8_sensirion(chunk[:2]) == chunk[2], f"bad CRC in word {i}: {data.hex()}"
        words.append(_word(chunk))
    return words


def test_tmp117_word_registers(node, wiring, sensors):
    """Each register is two bytes; neighboring registers no longer overlap."""
    t = sensors.start("tmp117", pins=i2c_pins(wiring), values={"temperature_c": 31.25})
    assert _word(node.i2c_rd(0x48, 0x0F, 2)) == 0x0117, "DEVICE_ID"
    assert _s16(_word(node.i2c_rd(0x48, 0x00, 2))) * 0.0078125 == pytest.approx(31.25, abs=0.008)
    cfg = _word(node.i2c_rd(0x48, 0x01, 2))
    assert cfg & 0x2000, f"Data_Ready in CONFIGURATION 0x{cfg:04x}"
    # a burst read runs on into the next register, word by word
    burst = node.i2c_rd(0x48, 0x00, 4)
    assert _word(burst[2:4]) == cfg, burst.hex()
    # an offset the DUT writes (TEMP_OFFSET, 0x07) is added to the result, as on the part
    node.i2c_wr(0x48, 0x07, (int(2.0 / 0.0078125)).to_bytes(2, "big"))
    time.sleep(0.02)
    assert _s16(_word(node.i2c_rd(0x48, 0x00, 2))) * 0.0078125 == pytest.approx(33.25, abs=0.008)
    t.set(temperature_c=-12.5)
    time.sleep(0.02)
    assert _s16(_word(node.i2c_rd(0x48, 0x00, 2))) * 0.0078125 == pytest.approx(-10.5, abs=0.008)


def test_ina219_current_from_the_calibration_the_dut_writes(node, wiring, sensors):
    """CURRENT follows the CALIBRATION word the DUT writes (datasheet 8.5.1)."""
    sensors.start("ina219", pins=i2c_pins(wiring), values={"current_a": 0.75, "bus_v": 12.0, "shunt_ohm": 0.1})
    assert _word(node.i2c_rd(0x40, 0x00, 2)) == 0x399F, "CONFIGURATION power-on value"
    bus = _word(node.i2c_rd(0x40, 0x02, 2))
    assert (bus >> 3) * 0.004 == pytest.approx(12.0, abs=0.004)
    assert _s16(_word(node.i2c_rd(0x40, 0x01, 2))) * 0.00001 == pytest.approx(0.075, abs=0.00001)
    assert _word(node.i2c_rd(0x40, 0x04, 2)) == 0, "CURRENT is 0 until the DUT calibrates"
    cal = 4096                                   # current LSB = 0.04096 / (cal * Rshunt) = 100 uA
    node.i2c_wr(0x40, 0x05, cal.to_bytes(2, "big"))
    time.sleep(0.02)
    assert _word(node.i2c_rd(0x40, 0x05, 2)) == cal
    current = _s16(_word(node.i2c_rd(0x40, 0x04, 2))) * 0.0001
    assert current == pytest.approx(0.75, abs=0.0002), current


def test_sht3x_commands_in_any_high_byte(node, wiring, sensors):
    """Single shot (0x2400, NACK while measuring), clock-stretched single shot (0x2C06), status
    (0xF32D) and serial number (0x3780): four high bytes in one sensor."""
    sensors.start("sht3x", pins=i2c_pins(wiring), values={"temperature_c": 23.5, "humidity_pct": 61.0})
    data, err = node.i2c_try("44 w 24 00 r 6")    # no wait: still measuring
    assert data is None and err.startswith("nack_addr"), (data, err)
    st, srh = _sensirion_words(node.i2c("44 w 24 00 d 20 r 6"), 2)
    assert -45 + 175 * st / 65535 == pytest.approx(23.5, abs=0.01)
    assert 100 * srh / 65535 == pytest.approx(61.0, abs=0.01)
    st, srh = _sensirion_words(node.i2c("44 w 2c 06 r 6"), 2)
    assert -45 + 175 * st / 65535 == pytest.approx(23.5, abs=0.01)
    assert _sensirion_words(node.i2c("44 w f3 2d r 3"), 1) == [0x8010]
    assert _sensirion_words(node.i2c("44 w 37 80 r 6"), 2) == [0x1A2B, 0x3C4D]


def test_scd4x_measurement_ready_flag_and_serial(node, wiring, sensors):
    """read_measurement (0xEC05), get_data_ready_status (0xE4B8) and get_serial_number (0x3682)."""
    s = sensors.start("scd4x", pins=i2c_pins(wiring),
                      values={"co2_ppm": 1234, "temperature_c": 22.0, "humidity_pct": 45.0})
    ready, = _sensirion_words(node.i2c("62 w e4 b8 d 2 r 3"), 1)
    assert ready & 0x07FF, f"data ready 0x{ready:04x}"
    co2, st, srh = _sensirion_words(node.i2c("62 w ec 05 d 2 r 9"), 3)
    assert co2 == 1234
    assert -45 + 175 * st / 65536 == pytest.approx(22.0, abs=0.01)
    assert 100 * srh / 65536 == pytest.approx(45.0, abs=0.01)
    assert _sensirion_words(node.i2c("62 w 36 82 d 2 r 9"), 3) == [0xF896, 0x9F07, 0x3BB2]
    s.set(co2_ppm=800)
    time.sleep(0.02)
    assert _sensirion_words(node.i2c("62 w ec 05 d 2 r 9"), 3)[0] == 800
