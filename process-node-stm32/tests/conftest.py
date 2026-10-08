"""Shared pytest config for the process-node HIL tests: LA bank voltage, bench wiring, flashing and
the console-driven ``node`` fixture every test file uses.

The node is flashed (when --benchpod-firmware is given) and booted ONCE per session, so the CAN,
analog and lifecycle files share one boot.
"""

import os
import re
import statistics
import time
from types import SimpleNamespace

import pytest

from embeddedci.benchpod import LoopInputMap

TARGET_CFG = "target/stm32f4x.cfg"
FLASH_ATTEMPTS = 3
OSC_MHZ = int(os.environ.get("CAN_NODE_OSC_MHZ", "8"))
BITRATE = 500_000

# The emulated BMP280 (test_alarms.py, test_timing.py).
ENV_ADDR = 0x76
ENV_START_C = 25.0
LIMIT_C = 50.0

# The thermal plant the pod emulates for the thermostat (test_thermostat.py, test_alarms.py).
AMBIENT_C = 20.0
C_PER_MV = 0.03
SENSOR_MV_AT_0C = 500.0
SENSOR_MV_PER_C = 20.0
HEATER_MAX_MV = 3000.0
SENSOR_MAX_MV = 3000.0      # the plant never drives PA1 above this
POINTS = 256
# Fabric lag: tick = 65535 / 48 MHz = 1.37 ms, k = 150/32768 per tick -> tau ~ 0.3 s.
TICK_DIV = 65535
K = 150


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    """LA I/O-bank voltage the ``benchpod`` fixture selects on connect (must match the DUT)."""
    return 3.3  # NUCLEO-F446RE I/O voltage


@pytest.fixture(scope="session")
def wiring(pins):
    """How THIS bench is wired: DUT signal -> BenchPod LA channel."""
    return SimpleNamespace(
        swclk=pins.pin_11,
        swdio=pins.pin_12,
        nreset=True,
        uart_rx=pins.pin_3,   # pod samples the DUT's TX here
        uart_tx=pins.pin_4,   # pod drives the DUT's RX here
        efuse=pins.efuse,
        i2c_scl=pins.pin_1,   # PB8, the pod emulates the BMP280 here
        i2c_sda=pins.pin_2,   # PB9
        alarm=pins.pin_5,     # PB0, high while any alarm is active
        strobe=pins.pin_6,    # PA8, marks ADC samples / control steps / env reads
        # Analog in: pod 3.3 V DAC SMA -> 5-10 kOhm -> PA1 (Nucleo A1). PROCESS_NODE_ANALOG=0 on a
        # bench without that lead skips the analog tests instead of failing them.
        ain_path="3v3",
        # Analog out: PA4 (Nucleo A2) -> pod ADC SMA (adc_ext). Same PROCESS_NODE_ANALOG switch.
        aout_source="ext",
        analog=os.environ.get("PROCESS_NODE_ANALOG", "1") == "1",
    )


def _flash(bp, wiring, firmware):
    result = None
    for attempt in range(FLASH_ATTEMPTS):
        if attempt > 0:
            bp.power_off(wiring.efuse)
        result = bp.flash(
            file=firmware, target=TARGET_CFG,
            swclk=wiring.swclk, swdio=wiring.swdio, nreset=wiring.nreset,
            target_power=wiring.efuse, check=False,
        )
        if not result.ok and result.target_unreachable and wiring.nreset:
            result = bp.flash(
                file=firmware, target=TARGET_CFG,
                swclk=wiring.swclk, swdio=wiring.swdio, nreset=False,
                target_power=wiring.efuse, check=False,
            )
        if result.ok:
            break
    return result


class Node:
    """The Nucleo process node, driven through its UART console."""

    def __init__(self, uart):
        self.uart = uart

    def cmd(self, line, pattern, timeout=3.0):
        """Send a console line and wait for ``pattern`` (regex) in the reply."""
        self.uart.read()  # forget earlier output so we only match this command's reply
        self.uart.write(line + "\r\n")
        return self.uart.expect(re.compile(pattern), timeout=timeout)

    def info(self):
        """``info`` as a dict: fw, version, build, uptime_ms, reset."""
        m = self.cmd("info", r"INFO ([^\r\n]*)\r\n")
        return {k: v.strip('"') for k, v in re.findall(r'(\w+)=("[^"]*"|\S+)', m.group(1))}

    def ain(self):
        """One fresh analog-input reading: mv, raw, filt_mv, vdda_mv, samples (all ints)."""
        m = self.cmd("ain", r"AIN ([^\r\n]*)\r\n")
        return {k: int(v) for k, v in (kv.split("=", 1) for kv in m.group(1).split())}

    def aout(self, args=""):
        """Run ``aout <args>`` and return its reply as a dict (``mode`` plus numbers), or raise
        AssertionError with the node's error line."""
        m = self.cmd(f"aout {args}".strip(), r"AOUT ([^\r\n]*)\r\n")
        body = m.group(1)
        assert not body.startswith("error"), f"aout {args}: {body}"
        return {k: (v if k == "mode" else int(v)) for k, v in (kv.split("=", 1) for kv in body.split())}

    def ctl(self, args=""):
        """Run ``ctl <args>``; returns mode (str), sp_c/pv_c (float degC), out_mv/in_band_ms/steps."""
        m = self.cmd(f"ctl {args}".strip(), r"CTL ([^\r\n]*)\r\n")
        body = m.group(1)
        assert not body.startswith("error"), f"ctl {args}: {body}"
        out = {}
        for k, v in (kv.split("=", 1) for kv in body.split()):
            out[k] = v if k in ("mode", "trip") else float(v) if k.endswith("_c") else int(v)
        return out

    def alarm(self, args=""):
        """Run ``alarm <args>``; returns active (int), flags, limit_c/env_c (float), env (str)..."""
        m = self.cmd(f"alarm {args}".strip(), r"ALARM ([^\r\n]*)\r\n")
        body = m.group(1)
        assert not body.startswith("error"), f"alarm {args}: {body}"
        out = {}
        for k, v in (kv.split("=", 1) for kv in body.split()):
            out[k] = (v if k == "env" else int(v, 16) if k == "active"
                      else float(v) if k.endswith("_c") else int(v))
        return out

    def reboot(self, timeout=3.0):
        """Software reset and wait for the boot marker: back to power-on defaults."""
        self.cmd("reset", r"RESET: rebooting")
        self.uart.read_until("APP_OK", timeout=timeout)
        time.sleep(0.2)

    def status(self):
        m = self.cmd("can status", r"CAN status: ([^\r\n]*)\r\n")
        return dict(kv.split("=", 1) for kv in m.group(1).split())

    def init(self, bitrate=BITRATE):
        self.cmd(f"can init {bitrate}", r"CAN init ok bitrate=%d" % bitrate)
        self.cmd("can echo off", r"CAN echo off")
        self.cmd("can print on", r"CAN print on")
        self.cmd("can clear", r"CAN counters cleared")

    def expect_rx(self, can_id, data=b"", ext=False, timeout=3.0):
        """Wait for the node to print a received frame."""
        pat = r"CAN rx id=0x%X ext=%d rtr=0 dlc=%d data=%s\r\n" % (
            can_id, int(ext), len(data), bytes(data).hex().upper())
        return self.uart.expect(re.compile(pat), timeout=timeout)


@pytest.fixture(scope="session")
def node(benchpod, wiring, pytestconfig):
    """Flash (optional), power-cycle and boot the node; one UART session for the whole run."""
    firmware = pytestconfig.getoption("benchpod_firmware")
    if firmware:
        result = _flash(benchpod, wiring, firmware)
        assert result.ok, f"flash failed; openocd output:\n{result.stderr}"
    benchpod.power_off(wiring.efuse)
    benchpod.power_on(wiring.efuse, delay=1.5)
    try:
        with benchpod.open_uart(rx=wiring.uart_rx, tx=wiring.uart_tx) as uart:
            n = Node(uart)
            # The boot banner can be clipped on a slow link, so check the result by command.
            uart.read_until("APP_OK", timeout=5)
            if OSC_MHZ != 8:
                n.cmd(f"can osc {OSC_MHZ}", r"CAN init (ok|fail)")
            st = n.status()
            assert st["chip"] == "ok", f"MCP2515 not answering on SPI: {st}"
            yield n
    finally:
        benchpod.power_off(wiring.efuse)


def _sensor_mv(temp_c):
    return SENSOR_MV_AT_0C + SENSOR_MV_PER_C * temp_c


@pytest.fixture
def plant(benchpod, wiring, node):
    """The thermal process the thermostat regulates, emulated by the pod's in-fabric loop (see
    test_thermostat.py). Calibrates the pod's 3.3 V output against the node's ADC, then returns a
    callable: ``plant(ambient_c)`` (re)loads the plant curve; ``plant.short_sensor()`` replaces the
    process with a sensor shorted to ground (0 V on PA1). Everything is stopped afterwards."""
    if not wiring.analog:
        pytest.skip("PROCESS_NODE_ANALOG=0: the thermostat needs both analog leads")
    caps = benchpod.capabilities
    if not (caps.dac_control_loop and caps.dac_loop_input_map):
        pytest.skip("pod gateware has no control loop with an input map")
    node.ctl("off")

    def mv_at_pa1(code):
        benchpod.dac_output(wiring.ain_path)
        with benchpod.control_loop(curve=[code] * POINTS, source="fixed", input_code=0,
                                   k=32767, tick_div=64):
            time.sleep(0.2)
            return statistics.mean(node.ain()["mv"] for _ in range(3))

    # Two-point fit of the 3.3 V output path, code -> mV at PA1 (through the 5-10 kOhm).
    c1, c2 = 10000, 30000
    m1, m2 = mv_at_pa1(c1), mv_at_pa1(c2)
    assert m2 - m1 > 300, f"pod 3.3 V output does not reach PA1: {c1}->{m1:.0f} mV, {c2}->{m2:.0f} mV"
    mv_per_code = (m2 - m1) / (c2 - c1)

    def code_for(mv):
        return max(0, min(65535, round(c1 + (mv - m1) / mv_per_code)))

    vmax = code_for(SENSOR_MAX_MV)
    state = {}

    def arm(ambient_c):
        curve = []
        for i in range(POINTS):
            heater_mv = HEATER_MAX_MV * 1.1 * i / (POINTS - 1)  # input axis 0..3.3 V
            temp = ambient_c + C_PER_MV * heater_mv
            curve.append(min(vmax, code_for(_sensor_mv(temp))))
        # Arming starts the output at vmin: make that the ambient, never below it.
        vmin = min(vmax, code_for(_sensor_mv(ambient_c)))
        benchpod.dac_output(wiring.ain_path)  # a stopped loop may have parked the output
        state["loop"] = benchpod.control_loop(
            curve=curve, source="adc", k=K, tick_div=TICK_DIV, vmin=vmin, vmax=vmax,
            input_map=LoopInputMap(mv_per_unit=1.0, range_min=0.0, range_max=HEATER_MAX_MV * 1.1))
        return state["loop"]

    def short_sensor():
        benchpod.dac_output(wiring.ain_path)
        state["loop"] = benchpod.control_loop(curve=[0] * POINTS, source="fixed", input_code=0,
                                              k=32767, tick_div=64, vmin=0, vmax=0)

    arm.short_sensor = short_sensor
    try:
        yield arm
    finally:
        node.ctl("off")
        benchpod.dac_stop()
        benchpod.dac_output("off")


@pytest.fixture
def env(benchpod, wiring, node):
    """Emulated BMP280 at 25 degC, the node seeing it, limit 50 degC. Afterwards the sensor is
    removed and the node rebooted, so no alarm is left active for the next test."""
    benchpod.enable_pullup(wiring.i2c_scl, wiring.i2c_sda)
    benchpod.enable_i2c_sensor("bmp280", sda=wiring.i2c_sda, scl=wiring.i2c_scl,
                               address=ENV_ADDR, temperature_c=ENV_START_C, pressure_pa=101325.0)
    try:
        node.alarm(f"limit {LIMIT_C}")
        wait_alarm(node, lambda a: a["env"] == "ok" and abs(a["env_c"] - ENV_START_C) <= 0.2,
                    timeout=3.0, what="the node to find the BMP280")
        yield benchpod
    finally:
        benchpod.disable_i2c_sensor()
        benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)
        node.reboot()


def wait_alarm(node, cond, *, timeout, what):
    """Poll ``alarm`` until ``cond(reply)`` holds; fails with the last reply after ``timeout``."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = node.alarm()
        if cond(last):
            return last
        time.sleep(0.1)
    pytest.fail(f"timed out after {timeout} s waiting for {what}; last: {last}")
