"""Cloud hardware-in-the-loop self-test for the device registered as ``benchpod-v1.0.0``.

Runs in GitHub Actions (see ``.github/workflows/selftest-cloud.yml``): the workflow builds
``selftest.c`` into ``build/selftest.elf``, then pytest drives the device **through
embeddedci.com** — there is no BenchPod on the runner. The connection
``embeddedci:benchpod-v1.0.0`` authenticates with the workflow's GitHub OIDC token, the
server bridges a tunnel to the physical BenchPod, and we flash + UART-verify the firmware
over the cloud exactly as we would locally.

Wiring (BenchPod logic-analyzer channels → DUT). The pod has no dedicated SWD/UART
pins — it exposes 12 generic LA channels (``pins.pin_1`` .. ``pins.pin_12``) and any
DUT signal can be on any of them. The ``wiring`` fixture below is THIS bench's map;
edit it to match how your board is wired.

    SWCLK → LA11        SWDIO → LA12        NRESET → LA3
    UART:  the pod samples the DUT's TX on LA5 and drives the DUT's RX on LA4
    Target power: internal-5V eFuse (``pins.efuse``, set with ``--benchpod-efuse``)
"""

import os
from types import SimpleNamespace

import pytest

# OpenOCD target config for the DUT (an STM32F4). Used both to flash and as the flash default
# recorded for the web UI, so they can't drift apart.
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


@pytest.fixture
def wiring(pins):
    """How THIS bench is physically wired: DUT signal → BenchPod LA channel.

    These are bench-specific; any signal can be on any of the 12 LA channels.
    (Pull-ups, if you needed them, exist only on LA1-8 — see ``pins.has_pullup``.)
    """
    return SimpleNamespace(
        swclk=pins.pin_11,    # SWD clock
        swdio=pins.pin_12,    # SWD data
        nreset=pins.pin_3,    # target NRST (None to skip connect-under-reset)
        uart_rx=pins.pin_5,   # pod samples the DUT's TX here
        uart_tx=pins.pin_4,   # pod drives the DUT's RX here
        efuse=pins.efuse,     # target-power rail
    )


@pytest.fixture
def dut(benchpod, wiring):
    """The BenchPod for the test, with a guaranteed clean shutdown: the target-power eFuse is
    switched OFF at teardown whether the test passes, fails, or errors — so we never leave the
    DUT powered on. (Fixture teardown also lets pytest report a power-off problem separately
    instead of masking the real test failure.)"""
    yield benchpod
    benchpod.power_off(wiring.efuse)


@pytest.mark.hardware
def test_selftest_boots_over_cloud(dut, wiring, firmware, build_report):
    """Flash selftest.elf to benchpod-v1.0.0 over the cloud and assert it boots cleanly.

    The ``build_report`` fixture is the opt-in that records this run as a GitHub-sourced build on
    embeddedci.com (uploading the firmware and capturing pass/fail) — but only inside GitHub Actions
    with an OIDC token. Locally it's a no-op, so this test runs the same with or without it.
    """
    # 0) Report this build to embeddedci.com (no-op outside GitHub Actions): upload the firmware
    #    artifacts and record the flash wiring so the web UI's flash modal can pre-fill them. The
    #    pytest pass/fail is captured automatically at teardown.
    build_report.record_wiring(
        target=TARGET_CFG, swclk=wiring.swclk, swdio=wiring.swdio,
        nreset=wiring.nreset, efuse=wiring.efuse,
    )
    build_report.upload_artifacts(_firmware_artifacts(firmware))

    # 1) Flash the freshly built firmware to the DUT over the cloud. flash() drives
    #    OpenOCD's CMSIS-DAP backend through the pod; ``target=`` is a normal OpenOCD
    #    config, so swapping it (stm32f4x.cfg, stm32h7x.cfg, nrf52.cfg, ...) is all it
    #    takes to support a different board.
    # check=False lets us inspect result.ok and fall back to a plain (no-NRST)
    # connect if connect-under-reset doesn't answer: selftest.c doesn't remap the
    # SWD pins, so NRST isn't required, and a mis-wired/floating NRST would
    # otherwise block the read. (Same flow as examples/selftest-stm32/e2e_local.py,
    # which passes on the bench.)
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
    assert result.ok, f"flash failed; openocd output:\n{result.stdout}"

    # 2) Read the boot banner with an event-based UART session. Schedule a pod-side
    #    power-on (returns immediately), then open the session BEFORE it fires so the
    #    banner lands in the already-listening buffer — no fixed capture window, and
    #    no command is needed *during* the session (which the wifi/AT link can't do).
    dut.power_off(wiring.efuse)
    dut.power_on(wiring.efuse, delay=1.5)
    with dut.open_uart(rx=wiring.uart_rx, tx=wiring.uart_tx) as uart:
        # selftest.c prints "APP_OK" once it has booted and initialised.
        assert uart.read_until(r"APP_OK", timeout=12), \
            f"selftest did not report APP_OK; captured:\n{uart.text}"

        # 3) Interactive check on the same live stream: drive the DUT's console and
        #    read its reply (all on the open UART link — something a one-shot
        #    capture cannot do). The pod sends "ping\r\n" back-to-back at line rate;
        #    selftest.c's RX is interrupt-driven so it takes the whole burst without
        #    overrunning (a polled RX would drop all but the first byte).
        uart.drain()
        uart.write("ping\r\n")
        assert uart.expect("pong", timeout=4), \
            f"DUT console did not answer ping; captured:\n{uart.text}"
