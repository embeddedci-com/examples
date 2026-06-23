# STM32 selftest — cloud HIL via GitHub Actions

`selftest.c` is a tiny STM32F446 firmware that boots, prints a UART banner ending in
`APP_OK`, and exposes a `ping`/`status` console. It's the "hello world" of an EmbeddedCI
hardware-in-the-loop test driven entirely from CI.

## What the workflow does

[`.github/workflows/selftest-cloud.yml`](../.github/workflows/selftest-cloud.yml) runs on every
change to `selftest.c` (or the workflow itself):

1. builds `selftest.c` → `build/selftest.elf` (arm-none-eabi + STM32CubeF4),
2. installs the `embeddedci` SDK and runs `pytest selftest-stm32/tests`,
3. the test connects to the device **through embeddedci.com** with the connection
   `embeddedci:benchpod-v1.0.0`, flashes the firmware over the cloud, power-cycles the
   target, and asserts the `APP_OK` UART marker.

There is **no BenchPod on the GitHub runner** — the physical pod lives wherever it's plugged in
and connects out to embeddedci.com. Auth is the job's **GitHub OIDC token** (`permissions:
id-token: write`); no API key or secret is stored.

## One-time setup

1. **Register the device** from the machine wired to the pod:
   ```bash
   benchpod register --connection <pod-ip>
   ```
   then rename it to `benchpod-v1.0.0` on the **BenchPod** page in the EmbeddedCI web app
   (names are URL-safe and unique per org).
2. **Trust this repo**: BenchPod → **GitHub Actions** → add `OWNER/REPO` (click *Look up* to fill
   the numeric ids) and allow it to drive **benchpod-v1.0.0** (Any device, or this specific one).

## Run it locally (against a pod on your LAN)

```bash
pip install "embeddedci"
make CUBE_F4=/path/to/STM32CubeF4
pytest selftest-stm32/tests -v \
  --benchpod-connection=192.168.1.50 \
  --benchpod-firmware=selftest-stm32/build/selftest.elf
```

Pin wiring (BenchPod LA channels → DUT) is set with `--benchpod-swclk/-swdio/-uart-rx/-uart-tx/-efuse`;
see the defaults in [`tests/test_selftest.py`](tests/test_selftest.py).
