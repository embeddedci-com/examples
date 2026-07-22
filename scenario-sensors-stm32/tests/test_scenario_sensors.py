"""Cloud hardware-in-the-loop test for the drone-sensors scenario firmware.

Companion to ``selftest-stm32`` but for the richer ``scenario-sensors-stm32`` firmware (BMP280 +
VL53L0X + rotor control, ``main.c``). It runs in GitHub Actions (see
``.github/workflows/scenario-cloud.yml``): the workflow builds ``build/scenario-sensors.elf``, then
pytest drives a physical BenchPod **through embeddedci.com** to flash it and verify it boots and its
console answers — exactly as you would locally.

This file is the *external* (GitHub-sourced) build path. The same project also has an internal
``embeddedci.yaml`` pipeline; the two are deliberately separate ways to build the same firmware, and
both surface on the Builds page (tagged "embeddedci" vs "github").

Wiring (BenchPod logic-analyzer channels → DUT) is THIS bench's map — edit it to match your board.
The pod has no dedicated SWD/UART pins; any DUT signal can be on any of the 12 LA channels.

    SWCLK → LA11   SWDIO → LA12   NRESET → LA3
    UART (USART1): pod samples the DUT's TX on LA5, drives the DUT's RX on LA4
    Target power: internal-5V eFuse (``pins.efuse``, set with ``--benchpod-efuse``)
"""

import os
import re
from types import SimpleNamespace

import pytest

# OpenOCD target config for the DUT (STM32F446, an STM32F4). Used to flash and recorded as the
# flash default for the web UI, so the two can't drift apart.
TARGET_CFG = "target/stm32f4x.cfg"


def _firmware_artifacts(firmware: str):
    """The firmware plus its sibling build outputs (``.elf`` / ``.bin`` / ``.hex``) that exist,
    so a reported build carries every format the UI might flash."""
    stem, _ = os.path.splitext(firmware)
    paths = [firmware] if os.path.exists(firmware) else []
    for ext in (".elf", ".bin", ".hex"):
        sibling = stem + ext
        if os.path.exists(sibling) and sibling not in paths:
            paths.append(sibling)
    return paths


# Flashing over the cloud goes OpenOCD → pod → SWD; a flaky link can drop a transaction. The SDK's
# flash() already retries transient *connect* failures, but a drop *mid-write* ("Failed to write
# memory") is treated as a hard error, so we retry the whole flash a few times here.
FLASH_ATTEMPTS = 3


def _flash_firmware(dut, wiring, firmware):
    """Flash with a connect-under-reset → plain-connect fallback, retried up to FLASH_ATTEMPTS.

    Returns the final FlashResult (inspect ``.ok`` / ``.stderr``).
    """
    result = None
    for attempt in range(FLASH_ATTEMPTS):
        if attempt > 0:
            # Re-flash from a cold boot: a prior failed flash can leave a half-written/sleeping
            # image running that blocks re-attaching, which is why a manual reset "fixes" it. A
            # power-cycle is the software equivalent (flash() powers the target back on).
            dut.power_off(wiring.efuse)
        result = dut.flash(
            file=firmware, target=TARGET_CFG,
            swclk=wiring.swclk, swdio=wiring.swdio, nreset=wiring.nreset,
            target_power=wiring.efuse, check=False,
        )
        if not result.ok and result.target_unreachable and wiring.nreset:
            result = dut.flash(
                file=firmware, target=TARGET_CFG,
                swclk=wiring.swclk, swdio=wiring.swdio, nreset=None,
                target_power=wiring.efuse, check=False,
            )
        if result.ok:
            break
    return result


@pytest.fixture
def wiring(pins):
    """How THIS bench is physically wired: DUT signal → BenchPod LA channel (edit for your board)."""
    return SimpleNamespace(
        swclk=pins.pin_11,    # SWD clock
        swdio=pins.pin_12,    # SWD data
        nreset=pins.pin_3,    # target NRST (None to skip connect-under-reset)
        uart_rx=pins.pin_5,   # pod samples the DUT's TX (USART1 TX) here
        uart_tx=pins.pin_4,   # pod drives the DUT's RX (USART1 RX) here
        efuse=pins.efuse,     # target-power rail
    )


@pytest.fixture
def dut(benchpod, wiring):
    """The BenchPod for the test, with target power guaranteed OFF at teardown."""
    yield benchpod
    benchpod.power_off(wiring.efuse)


@pytest.mark.artifacts_only
def test_report_build(firmware, wiring, build_report):
    """Record the built firmware as a GitHub-sourced build on embeddedci.com — WITHOUT a BenchPod.

    Marked ``artifacts_only`` (not ``hardware``): it needs no device, so it runs on every push (the
    workflow selects it with ``-m artifacts_only``). It uploads the firmware artifacts and records
    the default flash wiring, so the freshly built firmware is available to flash from the web UI even
    when the hardware HIL test below is not run. Outside GitHub Actions (no OIDC token) ``build_report``
    is an inert no-op, so this still passes locally as a plain "the firmware got built" check.
    """
    assert os.path.exists(firmware), f"firmware artifact missing: {firmware}"
    build_report.record_wiring(
        target=TARGET_CFG, swclk=wiring.swclk, swdio=wiring.swdio,
        nreset=wiring.nreset, efuse=wiring.efuse,
    )
    build_report.upload_artifacts(_firmware_artifacts(firmware))


@pytest.mark.hardware
def test_scenario_boots_over_cloud(dut, wiring, firmware, build_report):
    """Flash the scenario firmware over the cloud, assert it boots and its console answers.

    ``build_report`` opts this run into being recorded as a GitHub-sourced build on embeddedci.com
    (firmware upload + pass/fail), but only inside GitHub Actions with an OIDC token; locally it is a
    no-op so this test runs unchanged.
    """
    # 0) Report this build (no-op outside GitHub Actions): upload firmware + record flash wiring so
    #    the web UI flash modal pre-fills these defaults. pytest pass/fail is captured at teardown.
    build_report.record_wiring(
        target=TARGET_CFG, swclk=wiring.swclk, swdio=wiring.swdio,
        nreset=wiring.nreset, efuse=wiring.efuse,
    )
    build_report.upload_artifacts(_firmware_artifacts(firmware))

    # 1) Flash, with the same connect-under-reset → plain-connect fallback + retries as selftest.
    result = _flash_firmware(dut, wiring, firmware)
    # OpenOCD writes its log to stderr, so surface stderr (not stdout) on failure.
    assert result.ok, f"flash failed after {FLASH_ATTEMPTS} attempts; openocd output:\n{result.stderr}"

    # 2) Power-cycle and read the boot banner; main.c prints "APP_OK" once initialised.
    dut.power_off(wiring.efuse)
    dut.power_on(wiring.efuse, delay=1.5)
    with dut.open_uart(rx=wiring.uart_rx, tx=wiring.uart_tx) as uart:
        assert uart.read_until(r"APP_OK", timeout=12), \
            f"scenario firmware did not report APP_OK; captured:\n{uart.text}"

        # 3) Interactive console check on the live stream: "status" prints a status block.
        uart.drain()
        uart.write("status\r\n")
        assert uart.expect("status:", timeout=4), \
            f"DUT console did not answer status; captured:\n{uart.text}"

        # 4) Stepper: "stepmotor" pulses PC10 (STEP)/PC11 (DIR). Self-contained (no
        #    external wiring needed) — just assert the move runs to completion.
        uart.drain()
        uart.write("stepmotor 8\r\n")
        assert uart.expect("STEP done", timeout=4), \
            f"DUT did not complete stepper move; captured:\n{uart.text}"

        # 5) PWM health monitor: with nothing driving PA0 (TIM2_CH1, internal pull-down)
        #    the PWM is never seen, so the monitor treats it as "not started yet" and keeps
        #    waiting rather than faulting on a slow start — it must NOT report "failure
        #    detected" (that fires only after a PWM has been seen and then lost).
        uart.drain()
        uart.write("monitor pwm 1\r\n")
        assert uart.expect("waiting: no PWM on PA0 yet", timeout=6), \
            f"PWM monitor did not wait on an idle pin; captured:\n{uart.text}"
        assert "failure detected" not in uart.text, \
            f"PWM monitor wrongly faulted on a never-started signal; captured:\n{uart.text}"
        uart.write("\r\n")  # any key stops the monitor
        assert uart.expect("PWM monitor stopped", timeout=4), \
            f"PWM monitor did not stop on keypress; captured:\n{uart.text}"

        # 6) Stepper coil-current verify is silent by default: with PA0 idle the coil never
        #    swings, but a flat reading is only a fault once the motor has been seen running,
        #    so the move must run to completion (STEP done) without faulting AND without any
        #    per-cycle status chatter (that only appears with the "log" option).
        uart.drain()
        uart.write("stepmotor 1200 verify\r\n")
        assert uart.expect("STEP done", timeout=8), \
            f"stepper verify wrongly faulted on a never-running motor; captured:\n{uart.text}"
        seg = uart.drain()
        assert "verify cycle=" not in seg, \
            f"per-cycle status must be silent without 'log'; captured:\n{seg}"
        assert "STEP FAULT" not in seg, \
            f"must not fault on a motor that never spun up; captured:\n{seg}"

        # 7) Same move with "log": now the per-cycle status appears, and the firmware's timing
        #    summary lets us prove the in-line coil-current check never stole time from the STEP
        #    pulse train. The firmware holds a strict step period and absorbs the per-step ADC
        #    sample + bookkeeping into the low phase; any step whose verify work spills past the
        #    low-phase budget is counted as an "overrun". Run the most aggressive valid interval
        #    (50 ms floor -> bookkeeping every ~25 steps, plus an ADC sample on *every* step) and
        #    assert zero overruns, i.e. the pulses stayed strictly timed while verifying a lot.
        uart.drain()
        uart.write("stepmotor 1500 fwd verify 50 log\r\n")
        assert uart.expect("waiting: coil current still flat", timeout=8), \
            f"'log' run did not emit per-cycle status; captured:\n{uart.text}"
        timing_re = re.compile(r"STEP timing:.*overruns=(\d+),\s*status-print stalls=(\d+)")
        m = uart.expect(timing_re, timeout=10)  # raises UartTimeout if never printed
        overruns, stalls = int(m.group(1)), int(m.group(2))
        assert overruns == 0, (
            f"stepper verify disturbed the pulse timing: {overruns} step(s) overran the "
            f"low-phase budget (stalls={stalls}); captured:\n{uart.text}"
        )
        assert uart.expect("STEP done", timeout=8), \
            f"stepmotor timing run did not complete; captured:\n{uart.text}"
