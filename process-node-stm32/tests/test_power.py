"""Power and boot HIL tests: what the node costs and how it comes up.

Checked: the firmware's own boot time and the wall-clock time from power-on to ready, the supply
current while idle and while regulating against a budget, and that a short supply dropout leaves
the node running or cleanly rebooted, never hung.

The current is the whole NUCLEO board on the pod's internal 5 V rail, ST-LINK and MCP2515 module
included, so the budget is wide; set PROCESS_NODE_IDLE_MA_MAX to tighten it for your board. A
firmware that stops sleeping in its main loop (``__WFI``) shows up here first.
"""

import os
import time

import pytest

from conftest import AMBIENT_C

BOOT_MS_MAX = 400           # 200 ms SWD attach window + peripherals + MCP2515 + BMP280 probe
POWER_ON_READY_S = 5.0      # ST-LINK holds the F446 in reset ~2.2 s after the rail comes up
IDLE_MA = (10.0, float(os.environ.get("PROCESS_NODE_IDLE_MA_MAX", "200")))
PEAK_MA_MAX = 400.0
REGULATING_EXTRA_MA = 30.0  # heater drive into 1 MOhm + ADC/DAC busier: a few mA at most
DROPOUT_S = [0.005, 0.05, 0.5]


@pytest.mark.hardware
def test_boot_time(benchpod, wiring, node):
    benchpod.power_off(wiring.efuse)
    time.sleep(0.5)
    node.uart.read()
    start = time.monotonic()
    benchpod.power_on(wiring.efuse)
    node.uart.read_until("APP_OK", timeout=POWER_ON_READY_S)
    ready_s = time.monotonic() - start
    info = node.info()
    assert info["reset"] == "power-on", info
    assert int(info["boot_ms"]) <= BOOT_MS_MAX, f"firmware boot took {info['boot_ms']} ms"
    assert ready_s <= POWER_ON_READY_S, f"power-on to ready took {ready_s:.2f} s"


@pytest.mark.hardware
def test_idle_current(benchpod, wiring, node):
    node.ctl("off")
    node.aout("off")
    prof = benchpod.measure_power(1.0, efuse=wiring.efuse)
    ma, peak = prof.avg_current * 1000, prof.peak_current * 1000
    lo, hi = IDLE_MA
    assert not prof.fault, prof
    assert lo <= ma <= hi, f"idle current {ma:.1f} mA (budget {lo:.0f}..{hi:.0f} mA)"
    assert peak <= PEAK_MA_MAX, f"idle peak {peak:.1f} mA"


@pytest.mark.hardware
def test_regulating_current(benchpod, wiring, node, plant):
    node.ctl("off")
    node.aout("off")
    idle = benchpod.measure_power(1.0, efuse=wiring.efuse).avg_current * 1000
    node.aout("200")
    plant(AMBIENT_C)
    node.ctl("on 60")
    time.sleep(1.0)
    busy = benchpod.measure_power(1.0, efuse=wiring.efuse).avg_current * 1000
    assert busy - idle <= REGULATING_EXTRA_MA, \
        f"regulating draws {busy:.1f} mA vs {idle:.1f} mA idle"


@pytest.mark.hardware
@pytest.mark.parametrize("dropout", DROPOUT_S)
def test_supply_dropout(benchpod, wiring, node, dropout):
    """A dropout of ``dropout`` seconds: the node either rode it out or rebooted cleanly."""
    before = int(node.info()["uptime_ms"])
    t_off = time.monotonic()
    benchpod.power_off(wiring.efuse)
    benchpod.power_on(wiring.efuse, delay=dropout)
    off_for = time.monotonic() - t_off
    # Whichever happened, the console must answer within the reboot time.
    deadline = time.monotonic() + POWER_ON_READY_S
    info = None
    while time.monotonic() < deadline:
        try:
            info = node.info()
            break
        except Exception:  # still booting: no reply yet
            time.sleep(0.2)
    assert info is not None, f"node silent after a {dropout} s dropout"
    if int(info["uptime_ms"]) > before:
        return  # rode it out on the board's bulk capacitance
    assert info["reset"] in ("power-on", "brownout"), \
        f"after a {dropout} s dropout (host saw {off_for:.3f} s) the node reports {info}"
