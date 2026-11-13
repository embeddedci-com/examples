"""Shared pytest config for the process-node HIL tests: LA bank voltage, bench wiring, flashing and
the console-driven ``node`` fixture every test file uses.

The node is flashed (when --benchpod-firmware is given) and booted ONCE per session, so the CAN,
analog and lifecycle files share one boot.

Emulated-sensor tests (test_sensors_*.py) are marked per group (``sensors_i2c``, ``sensors_spi``,
``sensors_uart``, ``sensors_drdy``, ``sensors_stimulus``, ``sensors_faults``), so ``-m sensors_spi``
runs one group. Wires that are not on every bench yet gate their tests with an environment
variable: BENCH_NEW_WIRING=1 (SPI2 on LA7-LA10, USART3 on LA13/LA14) and BENCH_LA6_RESISTOR=1 (the
1 kOhm between PA8 and LA6 that lets the pod drive a data-ready line into PA8).
"""

import contextlib
import os
import re
import statistics
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from embeddedci.benchpod import FpgaImage, LoopInputMap

TARGET_CFG = "target/stm32f4x.cfg"
FLASH_ATTEMPTS = 3
OSC_MHZ = int(os.environ.get("CAN_NODE_OSC_MHZ", "8"))
BITRATE = 500_000

# Test-only sensor definitions and stimuli (the library's own come from the SDK).
SENSOR_DEFS = Path(__file__).parent / "sensor_defs"

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


SENSOR_MARKERS = {
    "hardware": "needs the bench (pod + node)",
    "sensors_i2c": "emulated I2C sensors on LA1/LA2, read by the node over I2C1",
    "sensors_spi": "the emulated SPI sensor on LA7-LA10 (BENCH_NEW_WIRING=1)",
    "sensors_uart": "emulated UART sensors on LA13/LA14 (BENCH_NEW_WIRING=1)",
    "sensors_drdy": "a data-ready line the pod drives on LA6 into PA8 (BENCH_LA6_RESISTOR=1)",
    "sensors_stimulus": "values over time from a stimulus",
    "sensors_faults": "injected sensor faults",
    "needs_new_wiring": "skips unless BENCH_NEW_WIRING=1 (SPI2 on LA7-LA10, USART3 on LA13/LA14)",
    "needs_la6_resistor": "skips unless BENCH_LA6_RESISTOR=1 (1 kOhm between PA8 and LA6)",
}


def pytest_configure(config):
    for name, text in SENSOR_MARKERS.items():
        config.addinivalue_line("markers", f"{name}: {text}")


def pytest_collection_modifyitems(config, items):
    """Gate tests on wires that are not on every bench yet (see the module docstring)."""
    new_wiring = os.environ.get("BENCH_NEW_WIRING") == "1"
    la6 = os.environ.get("BENCH_LA6_RESISTOR") == "1"
    for item in items:
        if item.get_closest_marker("needs_new_wiring") and not new_wiring:
            item.add_marker(pytest.mark.skip(
                reason="the SPI2 (LA7-LA10) and USART3 (LA13/LA14) wires are not in place: wire "
                       "them as in the README, then set BENCH_NEW_WIRING=1"))
        if item.get_closest_marker("needs_la6_resistor") and not la6:
            item.add_marker(pytest.mark.skip(
                reason="install the 1 kOhm resistor on the LA6-PA8 wire first (the pod drives "
                       "LA6 in this test), then set BENCH_LA6_RESISTOR=1"))


def pytest_addoption(parser):
    parser.addoption("--soak", type=int, default=0, metavar="N",
                     help="run every hardware test N times and print a per-test pass-rate table "
                          "(flakiness hunting; the update test is excluded, see PROCESS_NODE_FLASH_CYCLES)")


def pytest_generate_tests(metafunc):
    rounds = metafunc.config.getoption("soak")
    if rounds > 1 and metafunc.definition.get_closest_marker("hardware") \
            and metafunc.module.__name__ != "test_update":
        metafunc.fixturenames.append("soak_round")
        metafunc.parametrize("soak_round", range(1, rounds + 1), ids=lambda r: f"soak{r}")


_SOAK = {}


def pytest_runtest_logreport(report):
    if not re.search(r"[\[-]soak\d+[\]-]", report.nodeid):
        return
    if report.when == "call" or (report.when == "setup" and report.outcome == "failed"):
        base = re.sub(r"soak\d+-|-?soak\d+", "", report.nodeid).replace("[]", "")
        passed, total = _SOAK.get(base, (0, 0))
        _SOAK[base] = (passed + (report.outcome == "passed"), total + 1)


def pytest_terminal_summary(terminalreporter):
    if not _SOAK:
        return
    tr = terminalreporter
    tr.section("soak pass rate")
    for base, (passed, total) in sorted(_SOAK.items(), key=lambda kv: kv[1][0] / kv[1][1]):
        mark = "" if passed == total else "   <-- flaky"
        tr.write_line(f"{passed:4d}/{total:<4d} {base}{mark}")


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
        # The same LA6 wire, through 1 kOhm: an emulated sensor's data-ready output into PA8
        # (PA8 is an input while the strobe is off). Only the data-ready tests drive it.
        drdy=pins.pin_6,
        # SPI2 to the emulated SPI sensor; the pod drives only MISO (and only while CS is low).
        spi_sclk=pins.pin_7,  # PB13 SCK, DUT drives
        spi_mosi=pins.pin_8,  # PB15 MOSI, DUT drives
        spi_miso=pins.pin_9,  # PB14 MISO, pod drives
        spi_cs=pins.pin_10,   # PB12 CS, DUT drives (software CS)
        # USART3 to the emulated UART sensor.
        sensor_uart_tx=pins.pin_13,  # pod -> PC11 USART3 RX
        sensor_uart_rx=pins.pin_14,  # PC10 USART3 TX -> pod
        # Analog in: pod 3.3 V DAC SMA -> 5-10 kOhm -> PA1 (Nucleo A1). PROCESS_NODE_ANALOG=0 on a
        # bench without that lead skips the analog tests instead of failing them.
        ain_path="3v3",
        # Analog out: PA4 (Nucleo A2) -> pod ADC SMA (adc_ext). Same PROCESS_NODE_ANALOG switch.
        aout_source="ext",
        analog=os.environ.get("PROCESS_NODE_ANALOG", "1") == "1",
    )


def require_analog(benchpod, wiring, missing_lead):
    """Skip a test that needs the pod's DAC/ADC: on the digital-only BenchPod (no analog front end)
    and on a bench without the analog leads (PROCESS_NODE_ANALOG=0, ``missing_lead`` says which)."""
    if not benchpod.can_analog:
        pytest.skip("digital-only BenchPod: no DAC or ADC to drive PA1 / measure PA4")
    if not wiring.analog:
        pytest.skip(f"PROCESS_NODE_ANALOG=0: {missing_lead}")


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

    def __init__(self, benchpod, wiring):
        self._bp = benchpod
        self._wiring = wiring
        self._session = None
        self.uart = None

    def open(self):
        self._session = self._bp.open_uart(rx=self._wiring.uart_rx, tx=self._wiring.uart_tx)
        self.uart = self._session.__enter__()

    def close(self):
        if self._session is not None:
            self._session.__exit__(None, None, None)
            self._session = self.uart = None

    @contextlib.contextmanager
    def paused(self):
        """Close the console for a block that needs the pod to itself (flashing), then reopen it."""
        self.close()
        try:
            yield
        finally:
            self.open()

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

    # -- the generic bus commands (emulated sensors) ------------------------------------------

    def i2c_try(self, args, timeout=3.0):
        """Run ``i2c <args>``; ``(bytes, None)`` on success, ``(None, "nack_addr phase=r")`` style
        error text otherwise."""
        m = self.cmd(f"i2c {args}", r"I2C (ok r=([0-9a-f ]*)|err ([^\r\n]*))\r\n", timeout=timeout)
        if m.group(1).startswith("ok"):
            return (bytes.fromhex(m.group(2)) if m.group(2) else b""), None
        return None, m.group(3)

    def i2c(self, args, timeout=3.0):
        """``i2c <args>``: the bytes read; AssertionError with the node's error otherwise."""
        data, err = self.i2c_try(args, timeout=timeout)
        assert err is None, f"i2c {args}: {err}"
        return data

    def i2c_rd(self, addr, reg, n, reg16=False):
        """Register read with a repeated START (8- or 16-bit register address)."""
        return self.i2c(f"rd {addr:02x} {reg:04x} {n}" if reg16 else f"rd {addr:02x} {reg:02x} {n}")

    def i2c_wr(self, addr, reg, data, reg16=False):
        r = f"{reg:04x}" if reg16 else f"{reg:02x}"
        return self.i2c(f"wr {addr:02x} {r} {bytes(data).hex()}")

    def i2c_scan(self):
        m = self.cmd("i2c scan", r"I2C scan found=([0-9a-f ]*)\r\n", timeout=5.0)
        return [int(a, 16) for a in m.group(1).split()]

    def i2c_cfg(self, args=""):
        m = self.cmd(f"i2c cfg {args}".strip(), r"I2C (cfg hz=(\d+) timeout_ms=(\d+)|err[^\r\n]*)\r\n")
        assert m.group(2), m.group(1)
        return int(m.group(2)), int(m.group(3))

    def env(self, args=""):
        """``env [on|off]``: driver, status, chip (str), addr (int), temp_c/hum_pct (float)..."""
        m = self.cmd(f"env {args}".strip(), r"ENV (driver=[^\r\n]*)\r\n")
        out = {}
        for k, v in (kv.split("=", 1) for kv in m.group(1).split()):
            out[k] = (v if k in ("driver", "status", "chip") else int(v, 16) if k == "addr"
                      else float(v) if k.endswith(("_c", "_pct")) else int(v))
        return out

    def spi_cfg(self, mode=0, hz=500_000):
        m = self.cmd(f"spi cfg mode {mode} hz {hz}", r"SPI (cfg mode=(\d) hz=(\d+)|err[^\r\n]*)\r\n")
        assert m.group(2), m.group(1)
        return int(m.group(2)), int(m.group(3))

    def spi(self, data, read=0):
        """One SPI2 transfer: ``data`` then ``read`` zero bytes, CS low throughout. Returns every
        byte clocked in (the first ``len(data)`` are what came back during the command)."""
        args = bytes(data).hex() + (f" z {read}" if read else "")
        m = self.cmd(f"spi x {args}", r"SPI (ok n=\d+ r=([0-9a-f ]*)|err[^\r\n]*)\r\n")
        assert m.group(1).startswith("ok"), m.group(1)
        return bytes.fromhex(m.group(2))

    def uart3(self, args, pattern=r"UART3 [^\r\n]*\r\n", timeout=3.0):
        m = self.cmd(f"uart3 {args}", pattern, timeout=timeout)
        assert "UART3 err" not in m.group(0), m.group(0)
        return m

    def uart3_read(self, t_ms=200, n=None, until=None):
        """``uart3 read``: (bytes, matched)."""
        args = f"read t {t_ms}" + (f" n {n}" if n else "") + (f" until {until}" if until else "")
        m = self.uart3(args, r"UART3 rx n=(\d+) match=(\d) hex=([0-9a-f]*) text=[^\r\n]*\r\n",
                       timeout=t_ms / 1000 + 5.0)
        return bytes.fromhex(m.group(3)), m.group(2) == "1"

    def uart3_stat(self):
        m = self.uart3("stat", r"UART3 (open=[^\r\n]*)\r\n")
        return {k: int(v) for k, v in (kv.split("=", 1) for kv in m.group(1).split())}

    def uart3_count(self, ms):
        m = self.uart3(f"count {ms}", r"UART3 count ms=\d+ n=(\d+) framing=(\d+)\r\n",
                       timeout=ms / 1000 + 5.0)
        return int(m.group(1)), int(m.group(2))

    def nmea(self, args=""):
        m = self.cmd(f"uart3 nmea {args}".strip(), r"NMEA (mode=[^\r\n]*)\r\n")
        return {k: (v if k == "mode" else int(v)) for k, v in (kv.split("=", 1) for kv in m.group(1).split())}

    def nmea_lines(self, seconds):
        """Every ``NMEA ok|bad <sentence>`` line the node prints within ``seconds``:
        a list of (ok, sentence)."""
        deadline = time.monotonic() + seconds
        text = ""
        while time.monotonic() < deadline:
            text += self.uart.read() or ""
            time.sleep(0.05)
        return [(ok == "ok", line) for ok, line in re.findall(r"NMEA (ok|bad) (\S+)\r\n", text)]

    def drdy(self, args="", timeout=8.0):
        """``drdy [...]``: the reply's fields (ints, plus ``edge``/``kind``), or AssertionError."""
        m = self.cmd(f"drdy {args}".strip(), r"DRDY ([^\r\n]*)\r\n", timeout=timeout)
        body = m.group(1)
        assert not body.startswith("err"), f"drdy {args}: {body}"
        words = body.split()
        out = {"result": words[0] if "=" not in words[0] else "level"}
        for kv in words:
            if "=" in kv:
                k, v = kv.split("=", 1)
                out[k] = int(v) if v.lstrip("-").isdigit() else v
        return out

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
    n = Node(benchpod, wiring)
    try:
        n.open()
        # The boot banner can be clipped on a slow link, so check the result by command.
        n.uart.read_until("APP_OK", timeout=5)
        if OSC_MHZ != 8:
            n.cmd(f"can osc {OSC_MHZ}", r"CAN init (ok|fail)")
        st = n.status()
        assert st["chip"] == "ok", f"MCP2515 not answering on SPI: {st}"
        yield n
    finally:
        n.close()
        benchpod.power_off(wiring.efuse)


def _sensor_mv(temp_c):
    return SENSOR_MV_AT_0C + SENSOR_MV_PER_C * temp_c


@pytest.fixture
def plant(request, benchpod, wiring, node):
    """The thermal process the thermostat regulates, emulated by the pod's in-fabric loop (see
    test_thermostat.py). Calibrates the pod's 3.3 V output against the node's ADC, then returns a
    callable: ``plant(ambient_c)`` (re)loads the plant curve; ``plant.short_sensor()`` replaces the
    process with a sensor shorted to ground (0 V on PA1). Everything is stopped afterwards."""
    require_analog(benchpod, wiring, "the thermostat needs both analog leads")
    if "env" in request.fixturenames:
        sensor = request.getfixturevalue("env")
        if "dac_control_loop" in (sensor.started.get("unavailable") or []):
            pytest.skip("the emulated env sensor runs on the emulation FPGA image, which has no "
                        "DAC control loop: the thermal plant and the env sensor cannot run together")
    caps = benchpod.capabilities
    if not caps.dac_control_loop and benchpod.supports("sensors"):
        # An earlier sensor test left the emulation image loaded: back to the loop image.
        benchpod.sensors.stop()
        benchpod.fpga_image(FpgaImage.LOOP)
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


def require_sensors(benchpod):
    if not benchpod.supports("sensors"):
        pytest.skip("the pod's firmware has no sensor emulation (the 'sensors' capability)")


def start_env(benchpod, wiring, **values):
    """The emulated BMP280 the node's env driver reads, as instance ``env``."""
    return benchpod.sensors.start(
        "bmp280", instance="env", pins={"sda": wiring.i2c_sda, "scl": wiring.i2c_scl},
        address=ENV_ADDR, values={"temperature_c": ENV_START_C, "pressure_pa": 101325.0, **values})


@pytest.fixture
def env(benchpod, wiring, node):
    """Emulated BMP280 (the sensor library's definition) at 25 degC, the node seeing it, limit
    50 degC. Yields its :class:`Sensor` (``env.set(temperature_c=...)``). Afterwards every emulated
    sensor is stopped, the pull-ups are off and the node rebooted, so no alarm is left active."""
    require_sensors(benchpod)
    benchpod.enable_pullup(wiring.i2c_scl, wiring.i2c_sda)
    try:
        sensor = start_env(benchpod, wiring)
        node.alarm(f"limit {LIMIT_C}")
        wait_alarm(node, lambda a: a["env"] == "ok" and abs(a["env_c"] - ENV_START_C) <= 0.2,
                   timeout=3.0, what="the node to find the BMP280")
        yield sensor
    finally:
        with contextlib.suppress(Exception):
            benchpod.sensors.stop()
        benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)
        node.reboot()


@pytest.fixture
def sensors(benchpod, wiring, node):
    """``benchpod.sensors`` with the bench set up for the generic bus commands: the node's env
    driver off (the test owns I2C1), pull-ups on LA1/LA2. Afterwards (bench rules): the
    stimulus and every emulated sensor stopped, pull-ups off, and the node's buses back to their
    defaults (USART3 closed, I2C 100 kHz / 25 ms, SPI mode 0 / 500 kHz, env driver on)."""
    require_sensors(benchpod)
    node.env("off")
    benchpod.enable_pullup(wiring.i2c_scl, wiring.i2c_sda)
    try:
        yield benchpod.sensors
    finally:
        for stop in (benchpod.sensors.stop_stimulus, benchpod.sensors.stop):
            with contextlib.suppress(Exception):
                stop()
        benchpod.disable_pullup(wiring.i2c_scl, wiring.i2c_sda)
        node.cmd("uart3 close", r"UART3 closed\b[^\r\n]*\r\n")
        node.cmd("uart3 nmea off", r"NMEA mode=off\b[^\r\n]*\r\n")
        node.i2c_cfg("hz 100000 timeout 25")
        node.spi_cfg(0, 500_000)
        node.cmd("strobe off", r"STROBE mode=off\b[^\r\n]*\r\n")
        node.env("on")


def i2c_pins(wiring, **extra):
    return {"sda": wiring.i2c_sda, "scl": wiring.i2c_scl, **extra}


def spi_pins(wiring, **extra):
    return {"sclk": wiring.spi_sclk, "mosi": wiring.spi_mosi, "miso": wiring.spi_miso,
            "cs": wiring.spi_cs, **extra}


def uart_pins(wiring):
    return {"tx": wiring.sensor_uart_tx, "rx": wiring.sensor_uart_rx}


def local_def(name):
    """A test-only definition or stimulus from tests/sensor_defs/ (a path the SDK reads)."""
    return str(SENSOR_DEFS / f"{name}.yaml")


def wait_for(fn, cond, *, timeout, what, interval=0.1):
    """Poll ``fn()`` until ``cond(value)``; fails with the last value after ``timeout``."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = fn()
        if cond(last):
            return last
        time.sleep(interval)
    pytest.fail(f"timed out after {timeout} s waiting for {what}; last: {last}")


def crc8_sensirion(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


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
