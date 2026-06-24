"""Cloud hardware-in-the-loop self-test for the device registered as ``benchpod-v1.0.0``.

Runs in GitHub Actions (see ``.github/workflows/selftest-cloud.yml``): the workflow builds
``selftest.c`` into ``build/selftest.elf``, then pytest drives the device **through
embeddedci.com** — there is no BenchPod on the runner. The connection
``embeddedci:benchpod-v1.0.0`` authenticates with the workflow's GitHub OIDC token, the
server bridges a tunnel to the physical BenchPod, and we flash + UART-verify the firmware
over the cloud exactly as we would locally.

Wiring (BenchPod logic-analyzer channels → DUT) — these come from the plugin's pin-map
options and default sensibly; override per your bench with ``--benchpod-swclk`` /
``--benchpod-swdio`` / ``--benchpod-uart-rx`` / ``--benchpod-uart-tx`` / ``--benchpod-efuse``:

    SWCLK → pins.swclk      SWDIO → pins.swdio
    UART:  the pod samples the DUT's TX on pins.uart_rx and drives the DUT's RX on pins.uart_tx
    Target power: eFuse pins.efuse
"""

import pytest


@pytest.fixture
def dut(benchpod, pins):
    """The BenchPod for the test, with a guaranteed clean shutdown: the target-power eFuse is
    switched OFF at teardown whether the test passes, fails, or errors — so we never leave the
    DUT powered on. (Fixture teardown also lets pytest report a power-off problem separately
    instead of masking the real test failure.)"""
    yield benchpod
    benchpod.power_off(pins.efuse)


@pytest.mark.hardware
def test_selftest_boots_over_cloud(dut, pins, firmware):
    """Flash selftest.elf to benchpod-v1.0.0 over the cloud and assert it boots cleanly."""
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
        file=firmware, target="target/stm32f4x.cfg",
        swclk=pins.swclk, swdio=pins.swdio, nreset=pins.nreset,
        target_power=pins.efuse, check=False,
    )
    if not result.ok and result.target_unreachable and pins.nreset:
        result = dut.flash(
            file=firmware, target="target/stm32f4x.cfg",
            swclk=pins.swclk, swdio=pins.swdio, nreset=None,
            target_power=pins.efuse, check=False,
        )
    assert result.ok, f"flash failed; openocd output:\n{result.stdout}"

    # 2) Read the boot banner with an event-based UART session. Schedule a pod-side
    #    power-on (returns immediately), then open the session BEFORE it fires so the
    #    banner lands in the already-listening buffer — no fixed capture window, and
    #    no command is needed *during* the session (which the wifi/AT link can't do).
    dut.power_off(pins.efuse)
    dut.power_on(pins.efuse, delay=1.5)
    with dut.open_uart(rx=pins.uart_rx, tx=pins.uart_tx) as uart:
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
