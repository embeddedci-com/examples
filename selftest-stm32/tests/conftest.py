"""Local pytest config for the selftest tests.

Sets the BenchPod LA I/O-bank voltage for the whole session (see ``benchpod_la_voltage``).
"""

import pytest


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    """LA I/O-bank voltage the ``benchpod`` fixture selects on connect.

    The pod refuses flashing, UART, LA capture, pull resistors and I2C-sensor emulation until one
    is selected. It must match the DUT's I/O voltage.
    """
    return 3.3  # the STM32 board's I/O voltage — change to 1.8 for a 1V8 board
