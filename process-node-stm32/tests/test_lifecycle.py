"""Lifecycle HIL tests: identity, uptime and why the node last booted.

A real product has to tell a field engineer what it runs and why it restarted. These tests cause
each kind of restart the node can tell apart (power-on, software reset, watchdog) and check the
node reports it after the reboot.

Run with the rest of the suite (see conftest.py for wiring and flashing):

    pytest process-node-stm32/tests -v -k lifecycle --benchpod-connection=<host or embeddedci:name>
"""

import time

import pytest

FW_NAME = "process-node"
# The Nucleo's ST-LINK holds the F446 in reset ~2.2 s after the rail comes up.
POWER_ON_BOOT_S = 5.0
SOFT_BOOT_S = 3.0
IWDG_TIMEOUT_S = 2.0


def _await_boot(node, timeout):
    """Wait for the boot marker after a restart, then let the console settle."""
    node.uart.read_until("APP_OK", timeout=timeout)
    time.sleep(0.2)


@pytest.mark.hardware
def test_info_identifies_firmware(node):
    info = node.info()
    assert info["fw"] == FW_NAME, info
    assert info["version"].count(".") == 2, info
    assert info["reset"] != "unknown", info


@pytest.mark.hardware
def test_uptime_advances(node):
    first = int(node.info()["uptime_ms"])
    time.sleep(1.0)
    second = int(node.info()["uptime_ms"])
    # 1 s of wall clock, the HSI is +-1 % and the console round trip adds some
    assert 800 <= second - first <= 2500, (first, second)


@pytest.mark.hardware
def test_power_cycle_reports_power_on(benchpod, wiring, node):
    benchpod.power_off(wiring.efuse)
    time.sleep(0.5)
    node.uart.read()
    benchpod.power_on(wiring.efuse)
    _await_boot(node, POWER_ON_BOOT_S)
    info = node.info()
    assert info["reset"] == "power-on", info
    assert int(info["uptime_ms"]) < 5000, info


@pytest.mark.hardware
def test_software_reset_reports_software(node):
    node.cmd("reset", r"RESET: rebooting")
    _await_boot(node, SOFT_BOOT_S)
    assert node.info()["reset"] == "software"


@pytest.mark.hardware
def test_watchdog_recovers_a_hung_node(node):
    """The firmware stops kicking the watchdog; the IWDG must reset it and the node must say so."""
    node.cmd("wdt stall", r"WDT stall")
    start = time.monotonic()
    _await_boot(node, IWDG_TIMEOUT_S + SOFT_BOOT_S)
    took = time.monotonic() - start
    assert node.info()["reset"] == "iwdg"
    # LSI is 17..47 kHz on the F446, so the nominal 2 s can be ~1.4..3.8 s
    assert 1.0 <= took <= IWDG_TIMEOUT_S * 2 + SOFT_BOOT_S, took
