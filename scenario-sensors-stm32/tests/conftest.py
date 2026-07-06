"""Local pytest config for the scenario-sensors tests.

Registers the ``artifacts_only`` marker used to select the device-free build-reporting test
(``test_report_build``) so it can run on every push without touching a BenchPod. The ``hardware``
marker itself is registered by the embeddedci pytest plugin.
"""


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "artifacts_only: build-reporting test that needs no BenchPod; uploads the firmware "
        "to embeddedci as a GitHub-sourced build. Select with -m artifacts_only.",
    )
