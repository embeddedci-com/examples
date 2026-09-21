"""Local pytest config for the scenario-sensors tests.

Registers the ``artifacts_only`` marker used to select the device-free build-reporting test
(``test_report_build``) so it can run on every push without touching a BenchPod. The ``hardware``
marker itself is registered by the embeddedci pytest plugin (embeddedci>=2.0).

Also sets the BenchPod LA I/O-bank voltage for the whole session (see ``benchpod_la_voltage``).
"""

import pytest


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "artifacts_only: build-reporting test that needs no BenchPod; uploads the firmware "
        "to embeddedci as a GitHub-sourced build. Select with -m artifacts_only.",
    )


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    """LA I/O-bank voltage the ``benchpod`` fixture selects on connect.

    The pod refuses flashing, UART, LA capture, pull resistors and I2C-sensor emulation until one
    is selected. It must match the DUT's I/O voltage.
    """
    return 3.3  # the STM32F446 board's I/O voltage — change to 1.8 for a 1V8 board
