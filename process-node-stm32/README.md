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
  `PROCESS_NODE_ANALOG=0` to skip the analog tests. On a digital-only BenchPod (no DAC or ADC)
  the analog in/out, thermostat and the alarm, power and timing tests that use the emulated
  thermal plant skip on their own; CAN, the environment sensor, lifecycle, update and the rest
  still run.
- Analog out: **PA4 (Nucleo A2) -> pod ADC SMA**. The node keeps PA4 off (high-Z) unless a
  test turns it on, because the pod's own analog tests expect that SMA quiet.
- Environment sensor: BMP280 or BME280 on I2C1, **PB8 (SCL) -> LA1, PB9 (SDA) -> LA2**; on the
  bench the pod emulates it from the sensor library's definitions. Probed at 0x76/0x77 (chip id
  0x58 or 0x60), soft reset, read every 200 ms (a BME280 also gives humidity), re-probed every
  second while absent. `env off` stops the driver so a test owns the bus.
- Alarm output: **PB0 (Nucleo A3) -> LA5**, high while any alarm is active. Every alarm
  change also prints `EVT alarm set=<name>|clear=<name> active=0x..` and sends CAN frame
  0x0A0 `[active, changed, temp lo, temp hi]` (deci-degC). Alarms (debounced, 2 readings):
  `overtemp` (env above the limit, default 50 degC, clears 2 degC below), `pv_fault` (PA1
  out of 0.3-3.1 V while regulating), `env_lost`. Over-temperature and `pv_fault` trip the
  thermostat: heater off, latched until the next `ctl on` (refused while over-temperature).
- Strobe and data ready: **PA8 (Nucleo D7) -> 1 kOhm -> LA6**. PA8 is an input with a pull-down at
  boot and whenever the strobe is off; only `strobe ain|ctl|env` makes it an output, `strobe off`
  returns it to an input. The pod drives LA6 only in the data-ready tests (an emulated sensor's
  data-ready line, read with `drdy`); the resistor limits the current should both ever drive.
- Console: USART1 PA9 (TX) to pod LA3, PA10 (RX) to pod LA4, 115200 8N1.
  SWD: SWCLK LA11, SWDIO LA12.
- SPI sensor (emulated by the pod) on SPI2, CS in software: **PB13 SCK -> LA9, PB15 MOSI -> LA10,
  LA13 -> PB14 MISO, PB12 CS -> LA14**. The node drives SCK, MOSI and CS; the pod drives MISO only
  while CS is low (PB14 has a pull-up, so a released MISO reads 0xFF). SPI1 stays the MCP2515's.
- UART sensor (emulated by the pod) on USART3: **LA8 -> PC11 (RX), PC10 (TX) -> LA7**. Closed
  (both pins inputs) until `uart3 open`.

All LA wiring at a glance (the `wiring` fixture in `tests/conftest.py` is the same table):

| LA | Nucleo | Signal | Driven by |
|---|---|---|---|
| LA1 | PB8 | I2C1 SCL | node (open drain), pod pull-up 4.7k |
| LA2 | PB9 | I2C1 SDA | both (open drain), pod pull-up 4.7k |
| LA3 | PA9 | USART1 TX, console | node |
| LA4 | PA10 | USART1 RX, console | pod |
| LA5 | PB0 | alarm | node |
| LA6 | PA8 via 1 kOhm | strobe / data ready | node (strobe on) or pod (data-ready tests) |
| LA7 | PC10 | USART3 TX | node |
| LA8 | PC11 | USART3 RX | pod |
| LA9 | PB13 | SPI2 SCK | node |
| LA10 | PB15 | SPI2 MOSI | node |
| LA11 | SWCLK | SWD | pod |
| LA12 | SWDIO | SWD | both |
| LA13 | PB14 | SPI2 MISO | pod |
| LA14 | PB12 | SPI2 CS | node |

## Build

```bash
make                         # build/process-node.elf, 8 MHz module crystal
make MCP2515_OSC_HZ=16000000 # module with a 16 MHz crystal
make test-host               # bit-timing, process logic and compensation unit tests, no hardware
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
| `strobe off` | also makes PA8 an input again (the pod may drive LA6) |
| `i2c <addr> [w <b..>] [d <ms>] [r <n>]` | raw transfer on I2C1 (hex addr and bytes): write, wait, read, each phase its own START..STOP (command sensors such as the SHT4x); `I2C ok r=...` |
| `i2c rd <addr> <reg> <n>` | register read: write the register address, repeated START, read n (up to 64); `reg` with 2 hex digits is 8-bit, with 3-4 digits 16-bit (high byte first) |
| `i2c wr <addr> <reg> <b..>` | register write, same register address rule |
| `i2c scan` / `i2c probe <addr>` | `I2C scan found=44 68 76`; `I2C probe addr=44 ack\|nack_addr` |
| `i2c cfg [hz <n>] [timeout <ms>]` | bus clock 10-400 kHz (default 100 kHz) and the per-step timeout 1-500 ms (default 25, bounds clock stretching) |
| `i2c recover` | up to 9 SCL pulses by hand and a STOP, then a peripheral reset; `I2C recover sda_before=0\|1 ok\|stuck` |
| | every `i2c` error is `I2C err nack_addr phase=w\|r`, `I2C err nack_data byte=<n>` (0 = first data byte), or `I2C err bus_busy\|timeout\|arb_lost\|bus_error phase=w\|r` |
| `env [on\|off]` | `ENV driver=on status=ok\|absent\|lost chip=bmp280\|bme280 addr=76 temp_c= press_pa= hum_pct= reads= fails=`; `off` stops the driver, `on` starts over (probe, soft reset, setup) |
| `spi x <bytes..> [z <n>]` | one SPI2 transfer with CS low: the bytes, then n zero bytes; `SPI ok n=<n> r=<every byte clocked in>` |
| `spi cfg [mode <0-3>] [hz <n>]` | mode and the fastest clock at or below hz (16 MHz / 2..256; default mode 0, 500 kHz); `SPI cfg mode= hz=` |
| `uart3 open <baud>` / `uart3 close` | USART3 8N1, 1200-1000000 baud |
| `uart3 send <hex..>` / `uart3 line <text>` | send bytes, or text plus CR LF; `UART3 sent n=<n>` |
| `uart3 read [t <ms>] [n <count>] [until <text>]` | collect for t ms (default 200), stop early after count bytes or when the text (rest of the line) arrives; `UART3 rx n= match=0\|1 hex=<bytes> text=<escaped>` |
| `uart3 count <ms>` / `uart3 flush` / `uart3 stat` | bytes in a window (discarded); drop what is buffered; `UART3 open= baud= rx= tx= pending= framing= noise= overrun= dropped=` |
| `uart3 nmea [on\|off]` | prints each received sentence as `NMEA ok <sentence>` or `NMEA bad <sentence>` (checksum); `NMEA mode= lines= ok= bad= junk=` |
| `drdy` | `DRDY level=0\|1` of PA8 (only while the strobe is off, else `DRDY err strobe is on`) |
| `drdy wait rise\|fall\|high\|low [ms]` | `DRDY ok edge=rise us=<waited> level=1` or `DRDY timeout ms= level=` (default 1000 ms, at most 5000) |
| `drdy count <ms>` | `DRDY count ms= rises= falls= high_min_us= high_max_us= period_min_us= period_max_us= level=` (polled, about 1 us resolution) |
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

`tests/test_alarms.py`: the pod's emulated BMP280 (the sensor library's definition) drives the alarms: readings,
over-temperature on PB0 + UART + CAN with hysteresis, a spike the debounce ignores, the
heater trip and no restart while hot, a shorted process sensor while regulating, and the
sensor going missing and coming back.

Emulated sensors (`tests/test_sensors_*.py`): the pod plays sensor definitions, the sensor
library's (BME280, BMP280, SHT4x, MPU-6050, GPS) and small test-only ones in `tests/sensor_defs/`,
and the node reads them with its env driver or the generic `i2c`, `spi`, `uart3` and `drdy`
commands. Each test stops every emulated sensor and the stimulus, turns the pull-ups off and puts
the node's buses back to their defaults. Needs pod firmware with the `sensors` capability and the
SDK with `bp.sensors`. Run one group with `-m`:

| Marker | File | Checks |
|---|---|---|
| `sensors_i2c` | `test_sensors_i2c.py` | BME280 and BMP280 through the env driver (values, humidity, the alternate address, the soft-reset rule, read-only chip id), SHT4x raw commands (three precisions, serial, CRCs, NACK while measuring, heater commands answered after the pod's 65.535 ms limit), instance names of at most 16 characters, MPU-6050 (sleep, range, upside down, DEVICE_RESET), four sensors at different addresses on one bus, a 16-bit register address part, FIFO reads and overflow, clock stretching (and a timeout shorter than it), NACK while busy, an absent address |
| `sensors_spi` | `test_sensors_spi.py` | a LIS3DH-like part on LA9, LA10, LA13 and LA14: read and increment bits, WHO_AM_I, data registers that follow ODR and full scale, writes stored, read-only WHO_AM_I, the FIFO and its overrun, no data-ready pin (refused), SPI modes 0-3, 125 kHz to 1 MHz |
| `sensors_uart` | `test_sensors_uart.py` | the GPS: every sentence type with a good checksum, the position, the position moving with speed and course, no fix, the sentence mask; an MH-Z19-like part: byte request with checksum, a wrong request ignored, a text request |
| `sensors_drdy` | `test_sensors_drdy.py` | data ready on LA6 into PA8: pulse mode (rate, width, off), level mode held until DATA is read, `drdy` refused while the strobe drives PA8 |
| `sensors_stimulus` | `test_sensors_stimulus.py` | a temperature ramp the env driver follows over 6 s, a no_response window from a stimulus file |
| `sensors_faults` | `test_sensors_faults.py` | I2C no_response, corrupt (a broken SHT4x CRC), stuck readings, bus_stuck and the recovery after it; SPI MISO released; UART silent, garbage and dropped sentences |

Two environment variables gate tests on wires that are not on every bench yet:

- `BENCH_NEW_WIRING=1`: the SPI2 (LA9, LA10, LA13, LA14) and USART3 (LA7/LA8) wires are in place; without
  it the SPI and UART sensor tests skip.
- `BENCH_LA6_RESISTOR=1`: the 1 kOhm resistor is in the PA8-LA6 wire; without it every test
  where the pod drives LA6 (data ready) skips. Install the resistor first: without it the pod and
  PA8 would drive the wire against each other if the firmware ever got it wrong.

The emulated sensors run on the pod's emulation FPGA image, which has no DAC control loop: the
two heater tests in `test_alarms.py` that need the thermal plant and the emulated env sensor at
once skip, and the thermostat tests switch the pod back to the loop image.

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
