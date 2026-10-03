"""Local pytest config for the CAN node tests: the BenchPod LA I/O-bank voltage."""

import pytest


@pytest.fixture(scope="session")
def benchpod_la_voltage():
    """LA I/O-bank voltage the ``benchpod`` fixture selects on connect (must match the DUT)."""
    return 3.3  # NUCLEO-F446RE I/O voltage
