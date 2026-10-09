# process-node-stm32

A process-control node on a NUCLEO-F446RE with an MCP2515/TJA1050 module. It behaves like
a small real product (CAN, analog in/out, a thermostat loop, alarms, a watchdog) so the
BenchPod HIL tests cover real use cases, not just single pod features. The Nucleo runs a
UART console; the tests drive it over that console and check what it does from the pod
(CAN API, ADC/DAC, LA, power).

## Wiring

The module runs at 5 V from the Nucleo's 5V pin and is wired straight to the Nucleo.
PA6 and PC7 are 5 V tolerant, so the module's 5 V outputs are fine. The Nucleo's 3.3 V
outputs into the MCP2515 sit just under its 0.7 x VDD input threshold, which works in
practice; `can selftest` checks it. If it is ever flaky, put a TXS0108E in between
(OE tied to VCCA).

| Nucleo | MCP2515 module |
|---|---|
| D13 / PA5 (SPI1 SCK) | SCK |
| D12 / PA6 (SPI1 MISO) | SO |
| D11 / PA7 (SPI1 MOSI) | SI |
| D10 / PB6 (CS) | CS |
| D9 / PC7 (INT) | INT |
| 5V | VCC |
| GND | GND |

- Module **H to pod CAN+, L to pod CAN-**, plus a common ground.
- Leave the module's **120R jumper off**. The pod's switchable 120R is then the bus's only
  termination, so the termination test can prove the switch: on, traffic flows; off, the
  unterminated bus carries nothing. With the jumper fitted, set `CAN_NODE_TERM=1`.
- The Nucleo's ST-LINK holds the F446 in reset for about 2.2 s after power-up.
- Analog in: pod **3.3 V DAC SMA -> 5-10 kOhm -> PA1** (Nucleo A1). The resistor limits the
  current into the F446 when its rail is off and the pod still drives. Use the 3.3 V SMA
  only: the 5 V and +-12 V SMAs would damage the pin. On a bench without this lead set
  `PROCESS_NODE_ANALOG=0` to skip the analog tests.
- Analog out: **PA4 (Nucleo A2) -> pod ADC SMA**. The node keeps PA4 off (high-Z) unless a
  test turns it on, because the pod's own analog tests expect that SMA quiet.
- Environment sensor: BMP280 on I2C1, **PB8 (SCL) -> LA1, PB9 (SDA) -> LA2**; on the bench the
  pod emulates it. Probed at 0x76/0x77, read every 200 ms, re-probed every second while absent.
- Alarm output: **PB0 (Nucleo A3) -> LA5**, high while any alarm is active. Every alarm
  change also prints `EVT alarm set=<name>|clear=<name> active=0x..` and sends CAN frame
  0x0A0 `[active, changed, temp lo, temp hi]` (deci-degC). Alarms (debounced, 2 readings):
  `overtemp` (env above the limit, default 50 degC, clears 2 degC below), `pv_fault` (PA1
  out of 0.3-3.1 V while regulating), `env_lost`. Over-temperature and `pv_fault` trip the
  thermostat: heater off, latched until the next `ctl on` (refused while over-temperature).
- Strobe: **PA8 (Nucleo D7) -> LA6**, low unless `strobe` selects a task.
- Console: USART1 PA9 (TX) to pod LA3, PA10 (RX) to pod LA4, 115200 8N1.
  SWD: SWCLK LA11, SWDIO LA12.

## Build

```bash
make                         # build/process-node.elf, 8 MHz module crystal
make MCP2515_OSC_HZ=16000000 # module with a 16 MHz crystal
make test-host               # bit-timing unit test, no hardware
```

With 8 MHz the fastest bitrate is 500 kbit/s; 1 Mbit/s needs a 16 MHz crystal.

## Console

| Command | Does |
|---|---|
| `info` | `INFO fw=process-node version=... uptime_ms=... boot_ms=... reset=power-on\|software\|iwdg\|pin\|brownout` |
| `ain` | `AIN mv=... raw=... filt_mv=... vdda_mv=... samples=...`: analog input PA1 |
| `aout [<mV> \| sine <Hz> <amp-mV> <offset-mV> \| off]` | analog output PA4 (DAC, 0.2 V .. VDDA-0.2 V, sine by DDS at 10 kHz); off = high-Z |
| `ctl [on <degC> \| off]` | thermostat: PA1 sensor (0.5 V + 20 mV/degC) -> PI at 50 Hz -> PA4 heater; `CTL mode=... sp_c=... pv_c=... out_mv=... in_band_ms=...` |
| `alarm [limit <degC>]` | `ALARM active=0x.. overtemp= pv_fault= env_lost= limit_c= env=ok\|absent\|lost env_c= press_pa= events=` |
| `strobe [off\|ain\|ctl\|env]` | PA8 marks a task for the logic analyzer: high during an ADC sample / env read, toggles per control step |
| `i2c <addr> [w <b..>] [d <ms>] [r <n>]` | raw transfer on the sensor bus (hex addr and bytes): write, wait, read; `I2C ok r=...` or `I2C err` |
| `wdt stall` | stop kicking the watchdog (~2 s timeout) so the IWDG resets the node |
| `can status` | mode, bitrate, TEC/REC, error flags, counters |
| `can init [bitrate]` | reset and configure the MCP2515 (default 500000) |
| `can osc <MHz>` | set the module crystal, then re-init |
| `can mode normal\|listen\|loopback` | operating mode |
| `can send <id> [b0..b7]` | send one frame (hex); id above 7FF is extended |
| `can sendx <id> [b0..b7]` | force an extended id |
| `can burst <n> <id>` | n back-to-back frames, data = 16-bit counter |
| `can periodic <id> <ms> [count]` | send every ms; `can periodic off` stops |
| `can echo on\|off` | answer every frame with id+1, same data |
| `can print on\|off` | print received frames (default on) |
| `can selftest` | loop a frame inside the MCP2515, nothing on the bus |
| `can regs` | dump MCP2515 registers |

Received frames print as `CAN rx id=0x123 ext=0 rtr=0 dlc=2 data=0102`.

## HIL tests

```bash
pytest process-node-stm32/tests -v --benchpod-connection=<host or embeddedci:name> \
    --benchpod-firmware=process-node-stm32/build/process-node.elf
```

Without `--benchpod-firmware` the tests use the firmware already on the board.
`--soak N` runs every hardware test N times and ends with a per-test pass-rate table
(flaky tests at the top). Set
`CAN_NODE_OSC_MHZ=16` for a 16 MHz module (adds the 1 Mbit/s case).

`tests/test_lifecycle.py`: firmware identity, uptime, and the reset cause after a power
cycle, a software reset and a watchdog reset.

`tests/test_analog_in.py`: the pod's 3.3 V DAC is the sensor; the node's reading over a
0.1-3.0 V sweep (per point, gain, offset), VDDA from VREFINT, the filtered process value
after a step, and the 100 Hz background sampler.

`tests/test_analog_out.py`: the node's DAC measured by the pod's ADC: off is high-Z, DC
levels, 10/50/200 Hz sine (frequency, amplitude, offset), and refused out-of-range requests.

`tests/test_thermostat.py`: the node regulates a temperature against a thermal plant the
pod emulates in its FPGA (control loop: ADC SMA -> curve -> ~0.3 s lag -> 3.3 V DAC SMA,
no host in the path). Open-loop plant check, settle and hold at a setpoint, a setpoint
change, an ambient drop it must reject, and refused setpoints. The gains come from the host
simulation in `tests/sim_thermostat.c` (part of `make test-host`), which models the fabric's
integer damping.

`tests/test_alarms.py`: the pod's emulated BMP280 drives the alarms: readings,
over-temperature on PB0 + UART + CAN with hysteresis, a spike the debounce ignores, the
heater trip and no restart while hot, a shorted process sensor while regulating, and the
sensor going missing and coming back.

`tests/test_sensor_models.py`: the pod's other emulated sensors, read by the node over its
sensor bus with the `i2c` command and decoded with each vendor's reference math: a BME280
(temperature, pressure, humidity; the node's `ctrl_hum` survives a new reading), an SHT4x
(three precisions, serial number, CRCs) and an MPU-6050 (asleep at power-up, the range the
node selects, an upside-down reading, `DEVICE_RESET` clearing itself). Needs pod firmware
with the `sensor_types` capability and SDK support for it.

`tests/test_timing.py`: the pod's logic analyzer times the node on PA8/PB0: 100 Hz ADC
sampling (rate, jitter, ~1 ms cost per reading), the 50 Hz control rate while regulating,
the 200 ms env reads, and PB0 rising within 0.5 ms of the reading that confirms an alarm.

`tests/test_power.py`: firmware boot time and power-on-to-ready, idle and regulating
supply current against a budget (`PROCESS_NODE_IDLE_MA_MAX`, whole Nucleo board), and
5 ms / 50 ms / 500 ms supply dropouts (rides through or reboots cleanly, never hangs).

`tests/test_update.py`: flash, boot and identify the image N times
(`PROCESS_NODE_FLASH_CYCLES`, default 2; needs `--benchpod-firmware`).

`tests/test_can_hil.py`: both directions with standard and extended ids and 0 to 8 bytes, request/response
(echo), periodic timing from the pod's timestamps, bursts within and beyond the pod's RX
ring (overflow accounting), listen mode on either side (no ACK, error counters),
125k/250k/500k(/1M), a bitrate mismatch, and the pod's termination switch.
