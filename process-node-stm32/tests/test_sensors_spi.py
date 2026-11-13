"""The emulated SPI sensor on LA7-LA10, read by the node over SPI2 (PB13 SCK, PB15 MOSI,
PB14 MISO, PB12 CS).

The part is sensor_defs/spi_accel.yaml, a LIS3DH-like accelerometer: bit 7 of the first byte
selects a read, bit 6 auto increment, WHO_AM_I 0x33, s16le data registers that follow the full
scale the node selects and read 0 while powered down, and a 32-frame FIFO. Needs the SPI wires
(BENCH_NEW_WIRING=1).
"""

import time

import pytest

from conftest import local_def, spi_pins
from embeddedci.benchpod.sensors import load_definition

pytestmark = [pytest.mark.hardware, pytest.mark.sensors_spi, pytest.mark.needs_new_wiring]

READ = 0x80
INC = 0x40


def rd(node, reg, n=1, inc=True):
    return node.spi([READ | (INC if inc else 0) | reg], read=n)[1:]


def wr(node, reg, data):
    node.spi([reg & 0x3F, *data])


def xyz(raw):
    return [int.from_bytes(raw[i:i + 2], "little", signed=True) for i in (0, 2, 4)]


@pytest.fixture
def accel(wiring, node, sensors):
    node.spi_cfg(0, 500_000)
    return sensors.start(local_def("spi_accel"), pins=spi_pins(wiring),
                         values={"accel_x_g": 0.5, "accel_y_g": -0.25, "accel_z_g": 1.0})


def test_who_am_i_and_address_bits(node, accel):
    assert rd(node, 0x0F) == b"\x33"
    # Without the increment bit the address stays put: WHO_AM_I three times.
    assert rd(node, 0x0F, 3, inc=False) == b"\x33\x33\x33"
    # With it, the read runs on: WHO_AM_I, then 0x10, 0x11 (not covered: 0x00).
    assert rd(node, 0x0F, 3) == b"\x33\x00\x00"
    # The address byte itself clocks in nothing (MISO is 0 during the address).
    assert node.spi([READ | 0x0F], read=1)[0] == 0x00


def test_data_registers_follow_odr_and_range(node, accel):
    assert rd(node, 0x20) == b"\x07", "CTRL_REG1 power-on value: ODR 0, power down"
    assert xyz(rd(node, 0x28, 6)) == [0, 0, 0], "powered down: outputs read 0"
    assert rd(node, 0x27) == b"\x00"

    wr(node, 0x20, [0x57])                        # ODR 100 Hz, X/Y/Z on
    assert rd(node, 0x20) == b"\x57" and accel.regs(0x20, 1) == [0x57], "a write is stored"
    assert rd(node, 0x27) == b"\x0f"
    assert xyz(rd(node, 0x28, 6)) == [8192, -4096, 16384]   # +-2 g: 16384 LSB/g
    wr(node, 0x23, [0x30])                        # +-16 g
    assert xyz(rd(node, 0x28, 6)) == [683, -341, 1365]   # 0.5 * 1365 rounds half away from zero

    accel.set(accel_x_g=-2.0, accel_y_g=0.0, accel_z_g=-1.0)
    assert xyz(rd(node, 0x28, 6)) == [-2730, 0, -1365]
    wr(node, 0x0F, [0x00])                        # WHO_AM_I is read-only
    assert rd(node, 0x0F) == b"\x33"


def test_fifo(node, accel):
    wr(node, 0x20, [0x57])                        # 100 Hz
    wr(node, 0x24, [0x40])                        # FIFO_EN
    time.sleep(0.12)
    src = rd(node, 0x2F)[0]
    assert 6 <= (src & 0x1F) <= 20 and not src & 0x40, f"FIFO_SRC 0x{src:02x} after 120 ms"
    frames = rd(node, 0x3E, 6 * 5)                # 5 frames from the FIFO data register
    for k in range(5):
        assert xyz(frames[6 * k:6 * k + 6]) == [8192, -4096, 16384], (k, frames.hex())
    time.sleep(0.5)                               # far past 32 frames
    src = rd(node, 0x2F)[0]
    assert src & 0x40 and (src & 0x1F) == 31, f"FIFO_SRC 0x{src:02x}: full, overrun set"
    wr(node, 0x24, [0x00])


def test_no_data_ready_pin(wiring, sensors):
    """The emulated SPI target has no data-ready pin: a start that asks for one is refused."""
    with pytest.raises(Exception, match="no data-ready pin"):
        sensors.start(local_def("spi_accel"), pins=spi_pins(wiring, drdy=wiring.drdy))


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_spi_modes(node, wiring, sensors, mode):
    """The part in each SPI mode, the node in the same one."""
    doc = load_definition(local_def("spi_accel"))
    doc["spi"]["mode"] = mode
    sensors.start(doc, instance="spi_accel", pins=spi_pins(wiring))
    node.spi_cfg(mode, 500_000)
    assert rd(node, 0x0F) == b"\x33"
    wr(node, 0x20, [0x57])
    assert xyz(rd(node, 0x28, 6)) == [0, 0, 16384]


@pytest.mark.parametrize("hz", [125_000, 500_000, 1_000_000])
def test_clock_rates(node, accel, hz):
    _, actual = node.spi_cfg(0, hz)
    assert actual == hz
    wr(node, 0x20, [0x57])
    for _ in range(5):
        assert rd(node, 0x0F) == b"\x33"
        assert xyz(rd(node, 0x28, 6)) == [8192, -4096, 16384]
