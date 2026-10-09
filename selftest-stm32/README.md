# STM32 selftest: a minimal hardware-in-the-loop test

`selftest.c` is a tiny STM32F446 firmware that boots, prints a UART banner ending in
`APP_OK`, and exposes a `ping`/`status` console. It's the "hello world" of an EmbeddedCI
hardware-in-the-loop test driven entirely from CI.

## Run it over the cloud

The pod does not have to be on your network: the SDK reaches it **through embeddedci.com**,
flashes the firmware over the cloud, power-cycles the target and asserts the `APP_OK` UART
marker. Use the device name from the **BenchPod** page in the EmbeddedCI web app and an API key
(or a `benchpod login` session):

```bash
pip install "embeddedci[pytest]>=2.0,<3"
make CUBE_F4=/path/to/STM32CubeF4
BENCHPOD_API_KEY=eci_... pytest selftest-stm32/tests -v \
  --benchpod-connection=embeddedci:<device-name> \
  --benchpod-firmware=selftest-stm32/build/selftest.elf
```

To run this from GitHub Actions instead, see [`scenario-sensors-stm32`](../scenario-sensors-stm32)
and its workflow: it authenticates with the job's GitHub OIDC token, so no secret is stored.

## Run it locally (against a pod on your LAN)

```bash
pip install "embeddedci[pytest]>=2.0,<3"
make CUBE_F4=/path/to/STM32CubeF4
pytest selftest-stm32/tests -v \
  --benchpod-connection=192.168.1.50 \
  --benchpod-firmware=selftest-stm32/build/selftest.elf
```

The logic-analyzer bank voltage (3.3 V for this STM32) is set once in
[`tests/conftest.py`](tests/conftest.py) and in the `LA_VOLTAGE` constant of `e2e_local.py` —
change both to 1.8 for a 1V8 board.

Flashing needs an OpenOCD with the CMSIS-DAP TCP backend (`cmsis_dap_tcp`), which is newer than
OpenOCD 0.12.0 — the stock `apt`/`brew` packages lack it; use e.g. [xPack OpenOCD](https://github.com/xpack-dev-tools/openocd-xpack/releases).

The pod exposes 12 generic logic-analyzer channels (`pins.pin_1`…`pins.pin_12`); any DUT
signal can be on any channel. This bench's wiring (SWCLK→LA11, SWDIO→LA12, UART rx/tx→LA5/LA4,
NRESET→the pod's reset pin on DUT header J1 pin 22) lives in the `wiring` fixture in
[`tests/test_selftest.py`](tests/test_selftest.py) — edit it to match your board (`nreset=False` if
NRST isn't wired). The target-power rail is set with `--benchpod-efuse` (1=internal 5V).

[`e2e_local.py`](e2e_local.py) runs the same flash + UART check as a plain script against a pod on
your LAN: `python selftest-stm32/e2e_local.py 192.168.1.50`.
