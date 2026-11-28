"""RL78 programming over TOOL0, against the node's RL78 boot ROM emulator (rl78emu.c).

The pod holds LA3 (the node's PA9, its console TX) low through reset: the node then answers as an
RL78/G13 boot ROM (an R5F100LE, RL78 Protocol A) on that one wire, instead of booting. These tests
drive it with the minimal programmer in rl78_programmer.py: the same path a programmer for a real
RL78 takes through the pod (one-wire UART, pod-timed entry). Needs firmware 3.10.11+ on the pod
(gateware v57, capability ``uart_hd``).
"""

from __future__ import annotations

import os
import time

import pytest

from rl78_programmer import ACK, BLOCK, Rl78Error, Rl78Programmer, checksum

pytestmark = [pytest.mark.hardware, pytest.mark.rl78]

# After a cold power-on the NUCLEO's on-board ST-Link holds the target in reset for 2-4 s
# (measured), and a reset from the pod in that window is lost.
POWER_SETTLE_S = 5.0


@pytest.fixture(scope="module")
def rl78(benchpod, wiring, node):
    """The programmer on LA3. The node's console (the session-wide ``node`` fixture, which also
    flashes the firmware) is closed meanwhile, as LA3 is its TX; afterwards the node is
    power-cycled back into a normal boot."""
    if not benchpod.supports("uart_hd"):
        pytest.skip("the pod's firmware has no one-wire UART (uart_hd, gateware v57): update it")
    with node.paused():
        benchpod.power_off(wiring.supply)
        time.sleep(0.5)
        benchpod.power_on(wiring.supply)
        time.sleep(POWER_SETTLE_S)
        prog = Rl78Programmer(benchpod, wiring.uart_rx)
        try:
            yield prog
        finally:
            prog.close()
            benchpod.power_off(wiring.supply)
            time.sleep(0.5)
    benchpod.power_on(wiring.supply)
    node.uart.read_until("APP_OK", timeout=POWER_SETTLE_S + 5)


def test_checksum_examples():
    # R01AN0815's worked examples: 01 01 A1 5E 03 and 02 04 FF 80 40 22 1B 03
    assert checksum(bytes([0x01, 0xA1])) == 0x5E
    assert checksum(bytes([0x04, 0xFF, 0x80, 0x40, 0x22])) == 0x1B


def test_entry_and_signature(rl78):
    freq, mode = rl78.enter()
    assert (freq, mode) == (32, 0)
    sig = rl78.signature()
    assert sig == {"device_code": "100006", "name": "R5F100LE", "code_end": 0x00FFFF,
                   "data_end": 0x0F1FFF, "version": "V1.23"}


def test_program_verify_checksum(rl78):
    rl78.enter()
    image = os.urandom(2 * BLOCK)
    assert rl78.blank_check(0x000800, 0x000FFF)
    rl78.program(0x000800, image)
    assert not rl78.blank_check(0x000800, 0x000BFF)
    assert rl78.verify(0x000800, image)
    bad = bytearray(image)
    bad[1500] ^= 0x40
    assert not rl78.verify(0x000800, bytes(bad))
    assert rl78.checksum(0x000800, 0x000FFF) == (-sum(image)) & 0xFFFF
    # flash needs an erase before it is written again
    with pytest.raises(Rl78Error) as err:
        rl78.program(0x000800, bytes(BLOCK))
    assert err.value.status == 0x1C
    rl78.erase(0x000800)
    assert rl78.blank_check(0x000800, 0x000BFF)
    rl78.program(0x000800, bytes(BLOCK))
    assert rl78.verify(0x000800, bytes(BLOCK))


def test_data_flash(rl78):
    rl78.enter()
    image = os.urandom(BLOCK)
    rl78.erase(0x0F1000)
    rl78.program(0x0F1000, image)
    assert rl78.verify(0x0F1000, image)


def test_parameter_errors(rl78):
    rl78.enter()
    with pytest.raises(Rl78Error) as err:
        rl78.erase(0x000401)                    # not a block start
    assert err.value.status == 0x05
    with pytest.raises(Rl78Error) as err:
        rl78.program(0x00FC00, bytes(2 * BLOCK))   # past the end of code flash
    assert err.value.status == 0x05


@pytest.mark.parametrize("baud", [250000, 500000, 1000000])
def test_baud_rates(rl78, baud):
    rl78.enter(baud=baud)
    assert rl78.baud == baud
    image = os.urandom(4 * BLOCK)
    rl78.erase(0x004000)
    rl78.erase(0x004400)
    rl78.erase(0x004800)
    rl78.erase(0x004C00)
    t0 = time.monotonic()
    rl78.program(0x004000, image)
    took = time.monotonic() - t0
    assert rl78.verify(0x004000, image)
    print(f"{baud} baud: 4 KB programmed in {took:.2f} s")


def test_security(rl78):
    rl78.enter()
    flg = rl78.security_get()
    assert flg[:6] == bytes([0xFE, 0x03, 0x00, 0x00, 63, 0x00])
    rl78.security_set(0xEE, 0x03, 0, 63)       # programming disabled
    with pytest.raises(Rl78Error) as err:
        rl78.program(0x001000, bytes(BLOCK))
    assert err.value.status == 0x10


def test_normal_boot_is_unchanged(benchpod, wiring, rl78):
    """Without the strap the node boots as before (the emulator never starts by accident)."""
    rl78.close()
    benchpod.power_off(wiring.supply)
    time.sleep(0.5)
    with benchpod.open_uart(rx=wiring.uart_rx, tx=wiring.uart_tx, baud=115200) as uart:
        benchpod.power_on(wiring.supply)
        uart.expect("APP_OK", timeout=POWER_SETTLE_S + 5)
