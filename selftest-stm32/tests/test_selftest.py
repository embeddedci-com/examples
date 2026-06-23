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


@pytest.mark.hardware
def test_selftest_boots_over_cloud(benchpod, pins, firmware):
    """Flash selftest.elf to benchpod-v1.0.0 over the cloud and assert it boots cleanly."""
    # 1) Flash the freshly built firmware to the DUT over the cloud (SWD via the BenchPod).
    result = benchpod.flash(
        file=firmware,
        target="target/stm32f4x.cfg",
        swclk=pins.swclk,
        swdio=pins.swdio,
        nreset=pins.nreset,
        target_power=pins.efuse,
    )
    assert result.ok, f"flash failed; openocd output:\n{result.stdout}"

    # 2) Power-cycle the target and capture its UART boot output. We reboot via the
    #    target-power eFuse (NRST is unreliable on this bench), scheduling the power-on
    #    inside the capture window so the boot banner lands in it.
    capture = benchpod.power_cycle_and_capture(
        rx=pins.uart_rx,
        tx=pins.uart_tx,
        efuse=pins.efuse,
        delay=1.0,
        duration=6.0,
        until=r"APP_OK",
    )

    # 3) selftest.c prints "APP_OK" once it has booted and initialised — the self-test marker.
    assert capture.match(r"APP_OK"), f"selftest did not report APP_OK; captured:\n{capture.text}"
