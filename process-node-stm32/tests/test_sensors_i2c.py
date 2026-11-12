"""Emulated I2C sensors on LA1 (SCL, PB8) and LA2 (SDA, PB9), read by the node over I2C1.

The pod plays sensor definitions: the sensor library's BME280, BMP280, SHT4x and MPU-6050, and
small test-only parts in sensor_defs/ (16-bit register addresses, a FIFO, clock stretching, a part
that NACKs while it converts). The node reads the Bosch parts with its own env driver and the
rest with the generic ``i2c`` commands, the way their drivers would, and decodes them with each
vendor's math. Up to four of them share the bus at different addresses.
"""

import time

import pytest

from conftest import crc8_sensirion, i2c_pins, local_def, wait_for

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_i2c]


def _env_ok(node, chip, addr, timeout=4.0):
    return wait_for(node.env, lambda e: e["status"] == "ok" and e["chip"] == chip and e["addr"] == addr,
                    timeout=timeout, what=f"the env driver to find the {chip} at 0x{addr:02x}")


# ---- BME280 / BMP280 through the node's own env driver ----


def test_bme280_through_env_driver(node, wiring, sensors):
    bme = sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76,
                        values={"temperature_c": 21.5, "pressure_pa": 95000, "humidity_pct": 48})
    node.env("on")
    e = _env_ok(node, "bme280", 0x76)
    e = wait_for(node.env, lambda e: e["reads"] > 0, timeout=2.0, what="a first reading")
    assert abs(e["temp_c"] - 21.5) <= 0.1 and abs(e["press_pa"] - 95000) <= 3 \
        and abs(e["hum_pct"] - 48.0) <= 0.3, e
    # The driver's setup reached the part: normal mode, humidity oversampling x1.
    assert bme.regs(0xF4, 1) == [0x27] and bme.regs(0xF2, 1) == [0x01]

    bme.set(temperature_c=-5.0, humidity_pct=82.5)
    e = wait_for(node.env, lambda e: abs(e["temp_c"] + 5.0) <= 0.1, timeout=2.0, what="-5 degC")
    assert abs(e["hum_pct"] - 82.5) <= 0.3, e
    assert bme.regs(0xF2, 1) == [0x01], "ctrl_hum the driver wrote survives a new reading"
    st = bme.status()
    assert st["bus"]["writes"] > 0 and st["values"]["humidity_pct"] == 82.5, st


def test_bmp280_at_its_alternate_address(node, wiring, sensors):
    sensors.start("bmp280", instance="env", pins=i2c_pins(wiring), address=0x77,
                  values={"temperature_c": 30.0, "pressure_pa": 100000})
    node.env("on")
    _env_ok(node, "bmp280", 0x77)
    e = wait_for(node.env, lambda e: e["reads"] > 0, timeout=2.0, what="a first reading")
    assert abs(e["temp_c"] - 30.0) <= 0.1 and abs(e["press_pa"] - 100000) <= 3, e


def test_bme280_soft_reset_rule(node, wiring, sensors):
    """Only 0xB6 written to 0xE0 resets the part (on_write: reset), and the chip id is read-only."""
    bme = sensors.start("bme280", instance="env", pins=i2c_pins(wiring), address=0x76)
    node.env("on")
    _env_ok(node, "bme280", 0x76)
    node.env("off")
    assert bme.regs(0xF4, 1) == [0x27]

    node.i2c_wr(0x76, 0xE0, [0x00])               # not the reset value: nothing happens
    assert node.i2c_rd(0x76, 0xF4, 1) == b"\x27"
    node.i2c_wr(0x76, 0xD0, [0x00])               # id is writable: false, the pod restores it
    assert node.i2c_rd(0x76, 0xD0, 1) == b"\x60"

    node.i2c_wr(0x76, 0xE0, [0xB6])               # soft reset: back to the power-on image
    time.sleep(0.01)
    assert node.i2c_rd(0x76, 0xF4, 1) == b"\x00" and node.i2c_rd(0x76, 0xF2, 1) == b"\x00"
    assert node.i2c_rd(0x76, 0x88, 2) == bytes([27504 & 0xFF, 27504 >> 8]), "trim survives a reset"

    node.env("on")                                # the driver starts over: its setup again
    _env_ok(node, "bme280", 0x76)
    wait_for(lambda: bme.regs(0xF4, 1), lambda r: r == [0x27], timeout=2.0, what="ctrl_meas 0x27")


# ---- SHT4x: a command sensor (Sensirion datasheet 4.4-4.6) ----


def _sht4x(node, cmd="fd", delay_ms=10):
    r = node.i2c(f"44 w {cmd} d {delay_ms} r 6")
    assert crc8_sensirion(r[0:2]) == r[2] and crc8_sensirion(r[3:5]) == r[5], f"CRC: {r.hex()}"
    return -45 + 175 * (r[0] << 8 | r[1]) / 65535, -6 + 125 * (r[3] << 8 | r[4]) / 65535


def test_sht4x_raw_commands(node, wiring, sensors):
    sht = sensors.start("sht4x", pins=i2c_pins(wiring), values={"temperature_c": 23.4, "humidity_pct": 37.0})
    for cmd in ("fd", "f6", "e0"):                # high, medium, low repeatability
        t, rh = _sht4x(node, cmd)
        assert abs(t - 23.4) < 0.01 and abs(rh - 37.0) < 0.01, (cmd, t, rh)
    serial = node.i2c("44 w 89 d 1 r 6")
    assert crc8_sensirion(serial[0:2]) == serial[2] and crc8_sensirion(serial[3:5]) == serial[5]
    assert serial[0:2] + serial[3:5] == bytes.fromhex("0b1e5ea4"), serial.hex()

    sht.set(temperature_c=-12.25, humidity_pct=99.0)
    t, rh = _sht4x(node)
    assert abs(t + 12.25) < 0.01 and abs(rh - 99.0) < 0.01, (t, rh)
    with pytest.raises(Exception, match="humidity_pct"):
        sht.set(humidity_pct=120)


def test_sht4x_nacks_while_measuring(node, wiring, sensors):
    """A read during the 8.3 ms measurement is not acknowledged; after it, the answer is there."""
    sensors.start("sht4x", pins=i2c_pins(wiring))
    data, err = node.i2c_try("44 w fd r 6")       # no wait between the command and the read
    assert data is None and err.startswith("nack_addr") and "phase=r" in err, (data, err)
    time.sleep(0.02)
    r = node.i2c("44 r 6")
    assert crc8_sensirion(r[0:2]) == r[2], r.hex()


# ---- MPU-6050 (register map rev 4.2) ----


def _mpu(node):
    r = node.i2c_rd(0x68, 0x3B, 14)
    v = [int.from_bytes(r[i:i + 2], "big", signed=True) for i in range(0, 14, 2)]
    return v[0:3], v[3], v[4:7]


def test_mpu6050_range_sleep_reset(node, wiring, sensors):
    mpu = sensors.start("mpu6050", pins=i2c_pins(wiring), values={
        "accel_x_g": 0.5, "accel_y_g": -0.25, "accel_z_g": 1.0,
        "gyro_x_dps": 100, "gyro_y_dps": -50, "gyro_z_dps": 10, "temperature_c": 30})
    assert node.i2c_rd(0x68, 0x75, 1) == b"\x68", "WHO_AM_I"
    assert node.i2c_rd(0x68, 0x6B, 1) == b"\x40", "powers up asleep"
    assert _mpu(node)[0] == [0, 0, 0], "asleep: measurements read 0"

    node.i2c_wr(0x68, 0x6B, [0x01])               # wake, PLL on gyro X
    acc, temp, gyro = _mpu(node)
    assert acc == [8192, -4096, 16384], acc       # +-2 g: 16384 LSB/g
    assert gyro == [13100, -6550, 1310], gyro     # +-250 dps: 131 LSB/dps
    assert abs(temp / 340 + 36.53 - 30) < 0.01, temp

    node.i2c_wr(0x68, 0x1C, [0x18])               # accel +-16 g
    node.i2c_wr(0x68, 0x1B, [0x10])               # gyro +-1000 dps
    acc, _, gyro = _mpu(node)
    assert acc == [1024, -512, 2048] and gyro == [3280, -1640, 328], (acc, gyro)

    mpu.set(accel_x_g=0.0, accel_y_g=0.0, accel_z_g=-1.0)   # upside down
    assert _mpu(node)[0] == [0, 0, -2048]

    node.i2c_wr(0x68, 0x6B, [0x80])               # DEVICE_RESET: self-clearing
    wait_for(lambda: node.i2c_rd(0x68, 0x6B, 1), lambda r: not r[0] & 0x80, timeout=1.0,
             what="DEVICE_RESET to clear")
    assert node.i2c_rd(0x68, 0x6B, 1) == b"\x40" and node.i2c_rd(0x68, 0x1C, 1) == b"\x00"


# ---- several sensors, other bus features ----


def test_four_sensors_on_one_bus(node, wiring, sensors):
    p = i2c_pins(wiring)
    sensors.start("bme280", instance="env", pins=p, address=0x76)
    sensors.start("sht4x", pins=p)
    sensors.start("mpu6050", pins=p)
    sensors.start(local_def("eeprom16"), pins=p)
    assert node.i2c_scan() == [0x44, 0x50, 0x68, 0x76]
    assert node.i2c_rd(0x76, 0xD0, 1) == b"\x60"
    assert node.i2c_rd(0x68, 0x75, 1) == b"\x68"
    assert node.i2c_rd(0x50, 0x0100, 8, reg16=True) == b"BENCHPOD"
    t, _ = _sht4x(node)
    assert abs(t - 25.0) < 0.01, t
    st = sensors.status()
    running = {i["instance"]: i for i in st["instances"]}
    assert set(running) == {"env", "sht4x", "mpu6050", "eeprom16"}, running.keys()
    assert len({i["slot"] for i in running.values()}) == 4, "each on its own I2C target"
    node.env("on")                                # the env driver shares the bus with the rest
    _env_ok(node, "bme280", 0x76)
    assert node.i2c_rd(0x68, 0x75, 1) == b"\x68"


def test_16bit_register_address(node, wiring, sensors):
    part = sensors.start(local_def("eeprom16"), pins=i2c_pins(wiring), values={"serial": 0x1234})
    assert node.i2c_rd(0x50, 0x0100, 8, reg16=True) == b"BENCHPOD"
    assert node.i2c_rd(0x50, 0x0110, 2, reg16=True) == b"\x12\x34"
    part.set(serial=0xBEEF)
    assert node.i2c_rd(0x50, 0x0110, 2, reg16=True) == b"\xbe\xef"

    node.i2c_wr(0x50, 0x0180, b"\xde\xad\xbe\xef", reg16=True)
    assert node.i2c_rd(0x50, 0x0180, 4, reg16=True) == b"\xde\xad\xbe\xef"
    # Outside the 0x0100..0x01FF window: reads 0x00, writes are ignored.
    node.i2c_wr(0x50, 0x0080, b"\x55", reg16=True)
    assert node.i2c_rd(0x50, 0x0080, 2, reg16=True) == b"\x00\x00"


def test_fifo_reads(node, wiring, sensors):
    sensors.start(local_def("fifo_accel"), pins=i2c_pins(wiring),
                  values={"accel_x_g": 0.5, "accel_y_g": -1.25, "accel_z_g": 1.0})
    assert node.i2c_rd(0x1D, 0x0F, 1) == b"\x5f"
    assert node.i2c_rd(0x1D, 0x11, 1) == b"\x00", "FIFO off: empty"
    node.i2c_wr(0x1D, 0x10, [0x01])               # FIFO_EN: 100 frames/s
    time.sleep(0.15)
    level = node.i2c_rd(0x1D, 0x11, 1)[0]
    assert 8 <= level <= 20, f"{level} frames after 150 ms at 100 Hz"

    frames = node.i2c_rd(0x1D, 0x12, 60)          # 10 frames, the pointer stays on FIFO_DATA
    for k in range(10):
        f = frames[6 * k:6 * k + 6]
        xyz = [int.from_bytes(f[i:i + 2], "big", signed=True) for i in (0, 2, 4)]
        assert xyz == [500, -1250, 1000], (k, f.hex())
    assert node.i2c_rd(0x1D, 0x11, 1)[0] < level + 5, "popped frames leave the FIFO"

    time.sleep(0.5)                               # 50 more frames into a 32-frame FIFO
    assert node.i2c_rd(0x1D, 0x11, 1)[0] == 32, "full, drop_oldest keeps it at its depth"
    node.i2c_wr(0x1D, 0x10, [0x00])


def test_clock_stretching(node, wiring, sensors):
    """While it converts (20 ms) the ADC holds SCL low after the read address. One console
    command writes CONFIG (starting the conversion) and reads RESULT right after, so the read
    lands inside the conversion. With a 50 ms timeout the node waits it out and gets the result;
    with 2 ms it gives up."""
    adc = sensors.start(local_def("adc_stretch"), pins=i2c_pins(wiring), values={"input_mv": 1234})
    assert node.i2c_rd(0x48, 0x00, 1) == b"\xa5"
    node.i2c_cfg("timeout 50")
    assert node.i2c("48 w 01 01 r 2") == (1234).to_bytes(2, "big")

    adc.set(input_mv=3210)
    node.i2c_cfg("timeout 2")
    data, err = node.i2c_try("48 w 01 01 r 2")
    assert data is None and err == "timeout phase=r", (data, err)
    time.sleep(0.05)                              # the conversion ends, the target lets go
    node.i2c_cfg("timeout 25")
    assert node.i2c_rd(0x48, 0x02, 2) == (3210).to_bytes(2, "big")


def test_nack_while_busy(node, wiring, sensors):
    """A part with while_busy: nack refuses its read address during the conversion."""
    sensors.start(local_def("adc_nack"), pins=i2c_pins(wiring), values={"input_mv": 777})
    data, err = node.i2c_try("49 w 01 01 r 2")
    assert data is None and err == "nack_addr phase=r", (data, err)
    time.sleep(0.03)
    assert node.i2c_rd(0x49, 0x02, 2) == (777).to_bytes(2, "big")
    assert node.i2c("49 w 01 01 d 30 r 2") == (777).to_bytes(2, "big"), "after the conversion"


def test_absent_address_nacks(node, sensors):
    """Nothing at 0x2A: the address byte is refused (and reported as such, not as a timeout)."""
    data, err = node.i2c_try("rd 2a 00 1")
    assert data is None and err == "nack_addr phase=w", err
    m = node.cmd("i2c probe 2a", r"I2C probe addr=2a (\w+)")
    assert m.group(1) == "nack_addr"
