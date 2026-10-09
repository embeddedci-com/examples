"""The pod's other emulated I2C sensors, read over the real bus by the node.

The node's `i2c` console command does raw transfers on its sensor bus (I2C1, PB8/PB9 on LA1/LA2),
so each test reads an emulated part the way its driver does and decodes it with the vendor's
reference math here: a BME280 (temperature, pressure, humidity), an SHT4x (temperature,
humidity with CRCs) and an MPU-6050 (accelerometer, gyroscope, die temperature, with the range
the node selects, sleep and soft reset). Needs pod firmware that lists "sensor_types" in its caps.
"""
import re
import time

import pytest


def i2c(node, args):
    """Run `i2c <args>` on the node; the bytes it read, or AssertionError on a NACK/timeout."""
    m = node.cmd(f"i2c {args}", r"I2C (ok r=([0-9a-f ]*)|err[^\r\n]*)\r\n")
    assert m.group(1).startswith("ok"), f"i2c {args}: {m.group(1)}"
    return bytes.fromhex(m.group(2)) if m.group(2) else b""


@pytest.fixture
def emulate(benchpod, wiring, node):
    """``emulate(sensor, **values)`` arms one emulated part on the node's bus; removed afterwards."""
    if "sensor_types" not in (benchpod.status().get("caps") or []):
        pytest.skip("pod firmware has no BME280/SHT4x/MPU-6050 models (needs sensor_types)")
    benchpod.enable_pullup(wiring.i2c_scl, wiring.i2c_sda)

    def arm(sensor, **values):
        return benchpod.enable_i2c_sensor(sensor, sda=wiring.i2c_sda, scl=wiring.i2c_scl, **values)

    try:
        yield arm
    finally:
        benchpod.disable_i2c_sensor()
        benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)


# ---- BME280 (Bosch datasheet 4.2.3, double-precision compensation) ----

def _u16(b, i):
    return b[i] | b[i + 1] << 8


def _s16(b, i):
    v = _u16(b, i)
    return v - 65536 if v > 32767 else v


def bme280_read(node, addr=0x76):
    """Forced-mode measurement the way a BME280 driver does it: trim, ctrl_hum, ctrl_meas, poll, burst."""
    cal = i2c(node, f"{addr:x} w 88 r 26")
    h = i2c(node, f"{addr:x} w e1 r 7")
    i2c(node, f"{addr:x} w f2 01")          # humidity oversampling x1 (latched by the next ctrl_meas)
    i2c(node, f"{addr:x} w f4 25")          # temperature x1, pressure x1, forced mode
    for _ in range(20):                     # status bit 3: measuring (~5.5 ms here)
        if not i2c(node, f"{addr:x} w f3 r 1")[0] & 0x08:
            break
    d = i2c(node, f"{addr:x} w f7 r 8")
    T1, T2, T3 = _u16(cal, 0), _s16(cal, 2), _s16(cal, 4)
    P = [_u16(cal, 6)] + [_s16(cal, 6 + 2 * k) for k in range(1, 9)]
    H1, H2, H3 = cal[25], _s16(h, 0), h[2]
    H4 = (((h[3] - 256 if h[3] > 127 else h[3]) << 4) | (h[4] & 0x0F))
    H5 = (((h[5] - 256 if h[5] > 127 else h[5]) << 4) | (h[4] >> 4))
    H6 = h[6] - 256 if h[6] > 127 else h[6]
    adc_p = d[0] << 12 | d[1] << 4 | d[2] >> 4
    adc_t = d[3] << 12 | d[4] << 4 | d[5] >> 4
    adc_h = d[6] << 8 | d[7]
    v1 = (adc_t / 16384.0 - T1 / 1024.0) * T2
    v2 = ((adc_t / 131072.0 - T1 / 8192.0) ** 2) * T3
    t_fine = v1 + v2
    temp = t_fine / 5120.0
    v1 = t_fine / 2.0 - 64000.0
    v2 = v1 * v1 * P[5] / 32768.0 + v1 * P[4] * 2.0
    v2 = v2 / 4.0 + P[3] * 65536.0
    v1 = (P[2] * v1 * v1 / 524288.0 + P[1] * v1) / 524288.0
    v1 = (1.0 + v1 / 32768.0) * P[0]
    p = (1048576.0 - adc_p - v2 / 4096.0) * 6250.0 / v1
    press = p + (P[8] * p * p / 2147483648.0 + p * P[7] / 32768.0 + P[6]) / 16.0
    x = t_fine - 76800.0
    x = (adc_h - (H4 * 64.0 + H5 / 16384.0 * x)) * (
        H2 / 65536.0 * (1.0 + H6 / 67108864.0 * x * (1.0 + H3 / 67108864.0 * x)))
    hum = min(100.0, max(0.0, x * (1.0 - H1 * x / 524288.0)))
    return temp, press, hum


def test_bme280(benchpod, node, emulate):
    emulate("bme280", temperature_c=21.5, pressure_pa=95000, humidity_pct=48)
    assert i2c(node, "76 w d0 r 1") == b"\x60", "BME280 chip id"
    t, p, h = bme280_read(node)
    assert abs(t - 21.5) < 0.05 and abs(p - 95000) < 5 and abs(h - 48) < 0.2, (t, p, h)

    # a new reading reaches the bus, and the node's ctrl_hum/ctrl_meas survive the reload
    benchpod.set_i2c_sensor(humidity_pct=82.5, temperature_c=-5)
    assert i2c(node, "76 w f2 r 1") == b"\x01", "ctrl_hum the node wrote is kept"
    t, p, h = bme280_read(node)
    assert abs(t + 5) < 0.05 and abs(h - 82.5) < 0.2, (t, h)
    st = benchpod.i2c_sensor_status()
    assert st["writes"] > 0 and st["values"]["humidity_pct"] == 82.5, st


# ---- SHT4x (Sensirion datasheet 4.4-4.6) ----

def crc8(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def sht4x_measure(node, cmd="fd", addr=0x44):
    r = i2c(node, f"{addr:x} w {cmd} d 10 r 6")
    assert crc8(r[0:2]) == r[2] and crc8(r[3:5]) == r[5], f"CRC: {r.hex()}"
    t = -45 + 175 * (r[0] << 8 | r[1]) / 65535
    rh = -6 + 125 * (r[3] << 8 | r[4]) / 65535
    return t, rh


def test_sht4x(benchpod, node, emulate):
    emulate("sht4x", temperature_c=23.4, humidity_pct=37.0)
    for cmd in ("fd", "f6", "e0"):      # high, medium, low precision
        t, rh = sht4x_measure(node, cmd)
        assert abs(t - 23.4) < 0.01 and abs(rh - 37.0) < 0.01, (cmd, t, rh)
    serial = i2c(node, "44 w 89 d 1 r 6")
    assert crc8(serial[0:2]) == serial[2] and crc8(serial[3:5]) == serial[5], serial.hex()

    benchpod.set_i2c_sensor(temperature_c=-12.25, humidity_pct=99.0)
    t, rh = sht4x_measure(node)
    assert abs(t + 12.25) < 0.01 and abs(rh - 99.0) < 0.01, (t, rh)
    with pytest.raises(Exception, match="humidity_pct"):
        benchpod.set_i2c_sensor(humidity_pct=120)


# ---- MPU-6050 (register map rev 4.2) ----

def mpu_read(node, addr=0x68):
    r = i2c(node, f"{addr:x} w 3b r 14")
    v = [int.from_bytes(r[i:i + 2], "big", signed=True) for i in range(0, 14, 2)]
    return v[0:3], v[3], v[4:7]


def test_mpu6050(benchpod, node, emulate):
    emulate("mpu6050", accel_x_g=0.5, accel_y_g=-0.25, accel_z_g=1.0,
            gyro_x_dps=100, gyro_y_dps=-50, gyro_z_dps=10, temperature_c=30)
    assert i2c(node, "68 w 75 r 1") == b"\x68", "WHO_AM_I"
    assert i2c(node, "68 w 6b r 1") == b"\x40", "powers up asleep"
    acc, _, _ = mpu_read(node)
    assert acc == [0, 0, 0], "asleep: measurements read 0"

    i2c(node, "68 w 6b 01")             # wake, PLL on gyro X (what drivers write)
    time.sleep(0.02)                    # the pod rebuilds the data a few ms after a write
    acc, temp, gyro = mpu_read(node)
    assert acc == [8192, -4096, 16384], acc                  # +-2 g: 16384 LSB/g
    assert gyro == [13100, -6550, 1310], gyro                # +-250 dps: 131 LSB/dps
    assert abs(temp / 340 + 36.53 - 30) < 0.01, temp
    assert i2c(node, "68 w 3a r 1")[0] & 0x01, "data ready"

    i2c(node, "68 w 1c 18")             # accel +-16 g
    i2c(node, "68 w 1b 10")             # gyro +-1000 dps
    time.sleep(0.02)
    acc, _, gyro = mpu_read(node)
    assert acc == [1024, -512, 2048] and gyro == [3280, -1640, 328], (acc, gyro)

    benchpod.set_i2c_sensor(accel_x_g=0.0, accel_y_g=0.0, accel_z_g=-1.0)   # turned upside down
    acc, _, _ = mpu_read(node)
    assert acc == [0, 0, -2048], acc

    # DEVICE_RESET: the driver waits for bit 7 to clear, then finds power-on registers
    i2c(node, "68 w 6b 80")
    for _ in range(50):
        if not i2c(node, "68 w 6b r 1")[0] & 0x80:
            break
        time.sleep(0.005)
    assert i2c(node, "68 w 6b r 1") == b"\x40" and i2c(node, "68 w 1c r 1") == b"\x00"


def test_sensor_types_lists_the_models(benchpod, emulate):
    types = {t["type"]: t for t in benchpod.i2c_sensor_types()}
    assert {"bmp280", "bme280", "sht4x", "mpu6050"} <= set(types), types.keys()
    assert types["sht4x"]["addr"] == 0x44 and types["mpu6050"]["addr"] == 0x68
    keys = {p["key"] for p in types["mpu6050"]["params"]}
    assert {"accel_x_g", "gyro_z_dps", "temperature_c"} <= keys, keys
