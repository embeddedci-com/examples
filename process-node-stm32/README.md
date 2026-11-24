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

All LA wiring at a glance. [`wiring/process-node-stm32.json`](wiring/process-node-stm32.json) is the
same table as a wiring profile (see [Wiring profile](#wiring-profile)):

| LA | Nucleo | Signal | Driven by |
|---|---|---|---|
| LA1 | PB8 | I2C1 SCL | node (open drain), pod pull-up 4.7k |
| LA2 | PB9 | I2C1 SDA | both (open drain), pod pull-up 4.7k |
| LA3 | PA9 | USART1 TX, console | node |
| LA4 | PA10 | USART1 RX, console | pod |
| LA5 | PB0 | alarm; reset of the PIC and TPI emulators | node (alarm), pod (emulators, open drain) |
| LA6 | PA8 via 1 kOhm | strobe / data ready | node (strobe on) or pod (data-ready tests) |
| LA7 | PC10 | USART3 TX; UPDI of the UPDI emulators | node (USART3), both (UPDI, open drain) |
| LA8 | PC11 | USART3 RX | pod |
| LA9 | PB13 | SPI2 SCK | node |
| LA10 | PB15 | SPI2 MOSI | node |
| LA11 | SWCLK | SWD | pod |
| LA12 | SWDIO | SWD | both |
| LA13 | PB14 | SPI2 MISO; clock of the chip emulators | pod |
| LA14 | PB12 | SPI2 CS; data of the chip emulators | node (SPI CS), both (emulators) |

**JTAG wiring (in place on the bench since 2026-11-23).** For the pod's JTAG tests LA9 and LA10 carry
the node's JTAG pins instead of SPI2 SCK and MOSI: **PA15 (JTDI, morpho CN7 pin 17) -> LA9** and
**PB3 (JTDO, Arduino D3 / CN10 pin 31) -> LA10**. TCK and TMS are SWCLK (LA11) and SWDIO (LA12). The
SPI sensor tests skip on this wiring (see above); move the two wires back to PB13/PB15 to run them.

### Wiring profile

This bench is not wired like the pod's default pins (UART RX on LA5, I2C SDA on LA1 and SCL on LA2):
here the console is on LA3/LA4, I2C SCL on LA1 and SDA on LA2, and LA5 is the alarm. The bench has
its own wiring profile, [`wiring/process-node-stm32.json`](wiring/process-node-stm32.json), with every
pin in the table above: the roles (UART, I2C, SWD with NRST), the named signals (`ALARM`, `STROBE`,
`SENSOR_UART_*`, `SENSOR_SPI_*`), 3.3 V I/O and internal 5 V target power.

Store it on the pod once, under its own name, so the web app, BenchPod Desktop, the CLI, the SDK and
an AI agent all find it:

```bash
benchpod wiring save process-node-stm32/wiring/process-node-stm32.json --name process-node-stm32
```

Saving does not change the pod's default profile (unless it is the pod's first profile). The HIL
tests load it by name (`bp.load_wiring("process-node-stm32")`) and stop with that command when the
pod does not have it. In your own scripts, `BenchPod(conn, wiring_profile="process-node-stm32")`, or
in an agent, `connect(wiring_profile="process-node-stm32")`, does the same. A run that names its own
wiring (`--benchpod-wiring`, `--benchpod-wiring-profile`, `BENCHPOD_WIRING`,
`BENCHPOD_WIRING_PROFILE`) uses that instead.

## Build

```bash
make                         # build/process-node.elf, 8 MHz module crystal
make MCP2515_OSC_HZ=16000000 # module with a 16 MHz crystal
make test-host               # bit-timing, process logic and compensation unit tests, no hardware
make test-host-progemu BENCHPOD_FW=../../benchpod-firmware   # chip emulators vs the pod's protocol code, no hardware
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
| `i2c target <addr>` / `i2c target off` / `i2c target mem <reg> <n>` | answer at `addr` as a 256-byte register file (the first written byte sets the pointer), so the pod's I2C master has a target; the env driver goes off. `I2C target on=1 addr=.. ptr=.. matches=.. rx=.. tx=.. errors=..`; `mem` returns what the master wrote |
| | every `i2c` error is `I2C err nack_addr phase=w\|r`, `I2C err nack_data byte=<n>` (0 = first data byte), or `I2C err bus_busy\|timeout\|arb_lost\|bus_error phase=w\|r` |
| `env [on\|off]` | `ENV driver=on status=ok\|absent\|lost chip=bmp280\|bme280 addr=76 temp_c= press_pa= hum_pct= reads= fails=`; `off` stops the driver, `on` starts over (probe, soft reset, setup) |
| `spi x <bytes..> [z <n>]` | one SPI2 transfer with CS low: the bytes, then n zero bytes; `SPI ok n=<n> r=<every byte clocked in>` |
| `spi cfg [mode <0-3>] [hz <n>]` | mode and the fastest clock at or below hz (16 MHz / 2..256; default mode 0, 500 kHz); `SPI cfg mode= hz=` |
| `uart3 open <baud> [format]` / `uart3 close` | USART3, 1200-1000000 baud; format 8N1 (default), 8E1, 8O1, 8N2, 8E2 or 8O2; `uart3 stat` counts parity errors |
| `uart3 echo [on|off]` | echo every byte received on PC11 straight back on PC10 (loopback tests; off again on open) |
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
| `prog_emu <updi\|updimega\|updi2\|pic\|pic8\|pic18\|pic18q10\|c2\|c2early\|tpi\|tpi20\|tpi40\|tpi104\|pdi>` | answer as that chip for the pod's `prog_flash` (see [Chip emulators](#chip-emulators-prog_emu)); `PROG_EMU ok proto=.. chip=".."` |
| `prog_emu stop` / `stat` / `clear` / `lock` / `list` | normal pins back; counters, timing and CRC32 of each memory; zero the counters; lock the chip; the chips |
| `prog_emu dump <mem> <addr> <len>` | hex of an emulated memory (up to 4096 bytes), 32 bytes a line: `0040:<hex>` |

Received frames print as `CAN rx id=0x123 ext=0 rtr=0 dlc=2 data=0102`.

## RL78 boot ROM emulator

Holding the console TX (PA9, the bench's LA3) low through reset starts an emulated RL78 boot
ROM instead of the node, the way an RL78 enters its programming mode when TOOL0 is low at reset.
It speaks RL78 Protocol A (Renesas R01AN0815) on that one wire, so a programmer can be tested on
this board with no RL78 on the bench:

- The part is an R5F100LE (RL78/G13): 64 KB code flash at 000000H, 4 KB data flash at 0F1000H,
  1 KB blocks. Its flash lives in RAM, so every entry starts with an erased part.
- Commands: Reset, Baud Rate Set (115200, 250000, 500000, 1000000), Silicon Signature, Block
  Erase, Block Blank Check, Programming, Verify, Checksum, Security Set, Get and Release, with the
  guide's frames, checksums and status codes. Like real flash, a byte is written only when blank
  (an erase comes first), and the security flags are enforced.
- The line is open drain, single-wire half duplex, 1 stop bit from the emulator (a programmer sends
  2). It answers no sooner than the end of the programmer's second stop bit.
- TOOL0 has to stay low for about 6 ms after the reset release (the emulator sets up its flash in
  that time). A real RL78 needs only 723 us plus tHD, so a programmer written for the real part
  should hold it longer when it runs against this board.
- A normal boot is unchanged: the strap is sampled with the internal pull-up on, so an undriven
  LA3 never starts the emulator.

`tests/rl78_programmer.py` is a minimal Protocol A programmer over the BenchPod's one-wire UART
(`open_uart(half_duplex=True, entry=UartEntry(...))`, pod firmware with gateware v57), and
`tests/test_rl78.py` (marker `rl78`) runs it against the emulator: entry and signature, program,
verify and checksum, data flash, parameter errors, every baud rate, security, and a normal boot
afterwards. The protocol core (`rl78emu.c`) is also covered on the host by
`tests/test_rl78emu.c` (`make test-host`).

## RS-485 mock (Modbus RTU)

`modbus start <addr> <baud> [8E1|8N2|...] [delay <us>] [de on|off]` turns USART3 into an RS-485
bus with one Modbus RTU device on it, as a half-duplex transceiver presents it to the pod. It
uses the wires the UART sensors use, plus the LA6 data-ready wire as the transceiver's DE:

| Transceiver | Pod | Node |
| --- | --- | --- |
| DI (the pod sends) | LA8 | PC11, USART3 RX |
| RO (the pod listens) | LA7 | PC10, USART3 TX |
| DE + /RE | LA6 (GPIO) | PA8, through 1 kOhm (strobe off) |

- The device: holding registers 0-63 (0 at start), input registers 0-15 (1000H + index), coils
  0-63; functions 01, 03, 04, 05, 06, 08 (return query data), 0F and 10; exceptions 01, 02 and
  03; broadcasts (address 0) are carried out and not answered; a bad CRC or another address gets
  no answer. `modbus reg <i> [<v>]` reads or sets a holding register from the console.
- Frames end after 3.5 characters of silence (1750 us above 19200 baud); a gap of more than 1.5
  characters, or a framing or parity error, drops the frame. The device answers `delay` us after
  the request (default 5000), never sooner than 3.5 characters.
- With `de on` the transceiver's rules apply: a byte the pod sends while DE is low never reaches
  the device (`undriven`), and a reply byte while DE is still high never reaches the pod
  (`collisions`), as the pod's receiver is off then. `de off` is a transceiver with automatic
  direction control: no DE wire, only the protocol is tested.
- `modbus stat` reports the requests, replies, exceptions, CRC and frame errors, other addresses,
  broadcasts, `undriven` bytes and `collisions`.

`tests/modbus_master.py` is a Modbus RTU master over the pod, and `tests/test_rs485.py` (marker
`rs485`, BENCH_NEW_WIRING=1 BENCH_LA6_RESISTOR=1) runs it: registers, coils, exceptions, the silent
cases, four frame formats, a parity mismatch, DE never raised, released too early and too late,
the docs page's example unchanged, and the turnaround a host-switched DE allows.

Measured on the bench (LAN): switching DE with a GPIO command from the host takes about 27 ms, so
DE drops about 25 ms after the request's last byte. A device answering after 30 ms or more works
every time, after 20 ms about 2 times in 10, after 5 ms never. A device that answers faster needs
the pod to switch DE itself: `open_uart(de=6)` (gateware v58, feature `uart_de`) drives DE from
before the first start bit to the end of the last stop bit, and `test_pod_de` gets every reply at
9600, 19200 and 115200 baud with the device answering 3.5 characters after the request. A
transceiver with automatic direction control (`de off` here) works too.

DE's falling edge is timed by EXTI line 8 on PA8: a byte counts as driven when DE fell within its
stop bit, as a master that drives DE well drops it at the end of the last stop bit, often before
the byte's receive interrupt has run.

## Chip emulators (prog_emu)

The BenchPod's `prog_flash` programs chips over their own pins: AVR UPDI, TPI and XMEGA PDI, PIC ICSP,
Silicon Labs C2. With no chips on the bench, `prog_emu <chip>` makes the NUCLEO answer as one, so the
pod's protocols run against something other than their own host test. The chip models are the
bit-level models of benchpod-firmware's `stm32h563/test/test_prog_<proto>.c`, ported to this board
(`progemu_<proto>.c`: plain C, no HAL, time from the cycle counter); the pins, interrupts and the
console are `progemu.c`. Emulated memories live in RAM and start erased.

Wires (all existing; the same LA channels as the SPI sensor and the UART sensor, so run these
instead of those, not with them):

| Pod | Node | Used by | Notes |
|---|---|---|---|
| LA13 | PB14 | clock: PIC ICSPCLK, C2CK, TPICLK, PDI_CLK | input with pull-up, EXTI on both edges |
| LA14 | PB12 | data: ICSPDAT, C2D, TPIDATA, PDI_DATA | input; push-pull only while the chip drives it, released on the next clock edge; the pod holds a 10k pull-up |
| LA5 | PB0 | PIC MCLR, TPI RESET (`"reset":5`) | input with pull-up, EXTI on both edges; the pod drives it open drain. The pod's own reset pin is wired to the NRST of the F446 and must not be used. PB0 is the alarm output again after `prog_emu stop` |
| LA7 | PC10 | UPDI (`data` = 7) | USART3 in single-wire half duplex, 8E2, open drain |

| `prog_emu` | Chip (default ID, as the pod reports it) | Memories (`dump`, CRC32) |
|---|---|---|
| `pic` | PIC16F1xxx-like, 6-bit commands, key LSb first with 33 clocks, one ID word with DEV 0A4 and REV 02 packed: `a4000200` | `flash` 8192 B (4096 14-bit words, little endian), `eeprom` 256, `config` 64 (config words and user ID, 16 bits each), `uid` 32 (unused here) |
| `pic8` | PIC16F18xxx-like, 8-bit commands (key MSb first, 32 clocks), device 30B2 rev 2001: `b2300120` | same |
| `pic18` | PIC18 Q43/Q84-like: byte PC, Program Data (0xC0/0xE0) writes one word with no row latches, Bulk Erase 0x18 takes a region payload, config bytes, device 7A31 rev 2003: `317a0320` | `flash` 8192 B (16-bit words), `eeprom` 256, `config` 64 (bytes at 0x300000, one per 16-bit cell), `uid` 32 (16-bit words at 0x200000) |
| `pic18q10` | PIC18 Q10-like: Bulk Erase has no payload (the PC picks the regions), 16-bit config words, TERAB 75 ms, device 7B10 rev 2004: `107b0420` | same layout; `config` holds 16-bit words |
| `c2` | Silicon Labs C2 (EFM8BB2/UB1-like), device 32 rev 03 derivative 21: `320321`; FPDAT 0xB4 | `flash` 8192 B; the lock byte is its last byte (0x1FFF), so an image that ends in a byte other than FF locks the chip |
| `c2early` | the same chip, data driven early (see below) | same |
| `tpi` | ATtiny10: `1e9003`, one word a write | `flash` 1024 B (section erase wipes it all), `config` 2 (the configuration byte, the lock bits) |
| `tpi20`, `tpi40` | ATtiny20 `1e910f` (2 words a write), ATtiny40 `1e920e` (4 words), IDLE between the words | `flash` 2048 and 4096 B |
| `tpi104` | ATtiny104-like `1e900b`: also PAGE_ERASE (0x18) | `flash` 1024 B |
| `pdi` | XMEGA-like, device 1E 94 41: `1e9441` | `flash` 20480 B (128-byte pages), `eeprom` 1024, `user` 128, `config` 8 (fuse bytes 0-6, the lock byte) |
| `updi` | tinyAVR-like (NVM v0), signature 1E 94 21: `1e9421` | `flash` 16384 B at 0x8000 (64-byte pages), `eeprom` 256, `config` 10 (fuses), `user` 32 |
| `updimega` | megaAVR 0-like (NVM v0), family string `megaAVR`, signature 1E 94 26: `1e9426` | same, `flash` at 0x4000 (the pod takes the base from the SIB) |
| `updi2` | AVR DA-like (NVM v2), signature 1E 97 0A: `1e970a` | `flash` 16384 B at 0x800000 (512-byte pages), `eeprom` 512, `config` 16, `user` 32 |

What the pod is told (the node knows no device tables, the pod does not either): `page` is the chip's
write page in bytes (the Memories column), and the PIC chips need `flags` for the command set and the
code-protect bit, with `cp_addr` holding the byte address of the config word that carries it:

| `prog_emu` | `flags` | `cp_addr` | `page` |
|---|---|---|---|
| `pic` | `PIC_F_CP_BIT(5) \| PIC_F_HAS_EE` = `0x504` | `0x1000E` | 16 |
| `pic8` | `PIC_F_SET8 \| PIC_F_CP_BIT(1)` = `0x101` | `0x10016` | 32 |
| `pic18` | `PIC_F_SET8 \| PIC_F_BYTEPC \| PIC_F_CP_BIT(3)` = `0x303` | `0x300004` | 32 |
| `pic18q10` | `PIC_F_SET8 \| PIC_F_BYTEPC \| PIC_F_Q10 \| PIC_F_CP_BIT(3)` = `0x313` | `0x300004` | 32 |
| `tpi20`, `tpi40`, `tpi104` | words per write (2, 4) or `TPI_F_PAGE_ERASE` (8) | 0 | 16 |

**Datasheet behavior.** The models follow the vendors' documents as the pod's fixed protocol code
(benchpod-firmware #241-#244) reads them, not the first reading: PIC key order and 33 clocks on
the 6-bit set, the packed ID word, PIC18 Q Program Data and Bulk Erase payloads, TENTH 1 ms on the
PIC18 Qs; C2 data driven after the rising clock edge, the page erase's 0x00 byte and third status
(40 ms), the device setup writes (VDD monitor, SFR 0xFF/0xEF/0xA9) counted in `unprepared` when an
erase or write comes without them; TPI section erase of the whole flash, the group writes with IDLE
between the words, TPIIR; PDI RESET 0x59 before the key, the enable timing (data high 1 us, 16 idle
clocks, clock within 100 us), the turn-off when the clock is slower than about 10 kHz (`dropouts`),
the bus off during a chip erase, READ_NVM before NVM-space reads, the fuse write started by a PDI
write; UPDI starting disabled (enable pulse, or a BREAK, then SYNC inside its window; `timing_viol`),
the 10 ms v2 EEPROM byte write (a streamed byte meanwhile is lost: `drops`; an ACKed one is held),
NVMPROG appearing 2 ms after the key, the megaAVR 0 flash base.

**What the node cannot do exactly.** All of it runs from interrupts at the F446's 16 MHz core:
- C2 specifies the chip driving C2D within nanoseconds of the rising edge, and the pod's feed reads
  120 ns or more after it. The interrupt needs about a microsecond, so `c2` can hand the pod the
  previous bit if the pod samples early; `c2early` puts each bit on the wire during the preceding
  low phase instead, so a slow interrupt still meets the read (a pod that wrongly samples with the
  clock low is not caught by `c2early`). Try `c2`; if the IDs are off by a bit, read `missed`/`late`
  and the high-time minimum in `stat` and use `c2early`.
- The enable pulse and BREAK of UPDI and PDI's data-high timing are measured with the cycle counter
  (62.5 ns), PDI's data edge by an extra EXTI line on PB12, so they are only as exact as the
  interrupt latency (about 1 us).
- A held UPDI ACK (v2 EEPROM, 10 ms) is a busy wait in the interrupt: the node does nothing else
  meanwhile.
- The 1.8 V/3.3 V levels, the 5 V-only parts (ATtiny4/5/9/10/20/40 need 5 V on a real bench) and the
  bulk-erase supply minimums are the pod's to check; the emulator does not model supply voltage.
- TPI and PDI clock limits (2 MHz, 10 kHz) other than the PDI turn-off are not checked.

`prog_emu lock` locks the chip the way its own lock works (PIC code protection, the C2 lock byte,
the AVR lock bits, UPDI's lock); a chip erase unlocks it again. A real chip does the same, so a
locked chip answers `info` where it can and refuses to be read or written.

`prog_emu stat` prints, after one `PROG_EMU stat` line (`edges rise fall nrst_edges frames errors
timing missed late core_mhz`):

- `high_ns` and `low_ns` (min, max, and the average of the intervals under 100 us): the time the
  clock stayed high (rising to falling edge) and low, as the node saw it, from the cycle counter
  captured at the top of the interrupt. `high_hist_us` and `low_hist_us` count the intervals in the
  bins `<0.5, <1, <2, <4, <8, <16, <64, <256, >=256` us. UPDI reports the SYNC bit time it measured
  and the baud rate it set instead (`PROG_EMU updi ... sync_bit_ns=`).
- `missed` and `late`: an edge that was gone before the handler read the pins (both edges came
  while the previous one was handled), and an edge that arrived while the previous one was still
  being handled (it is taken right after). Both mean the pod's bit-bang is faster than the node can
  follow at its 16 MHz core clock; the handler does a model step and a pin write per edge and
  nothing else (no printf, only counters), at the top interrupt priority.
- `frames` and `errors`: commands or frames the chip took, and what a real chip would not accept
  (bad frames, parity errors, a command while the NVM is busy, a wrong key, and the protocol's own
  timing rules, which `timing` counts separately: PIC's 1 us command delays and 250 us entry hold,
  the C2 enable wait). A protocol line follows (`PROG_EMU pic entries=.. bad_cmds=.. ...`).
- `crc`: the CRC32 (zlib's) of every memory, in the byte layout of the table above, to compare with
  the image's.

`PROG_EMU updi_gap_us` (UPDI only): for the enable pulse and for the BREAK, how many there were and the
gap in us from the end of the low (its rising edge) to the first edge of the next SYNC (min, max, last), plus the
length of the last and the longest low. It is measured whether or not the chip accepted the SYNC, so the bench
sees the pod's real timing against the 200 us to 14 ms window (pulse) and the 5 ms after the BREAK's frame.

UPDI details: the receiver is off while the chip waits for a frame. EXTI line 10 times the five
falling edges of the SYNC byte (0x55), the baud rate is derived from them (the pod's UPDI links run at 115200 to 225000 baud), the receiver is switched on at the end of the
SYNC and takes the rest of the frame. A low of more than 1 ms, or a framing error with 0x00, is a
BREAK (the chip leaves its state and waits for a SYNC). A frame with a parity or framing error is
dropped and counted. Replies leave a ring through the USART3 interrupt two bit times after the
request's last stop bit, with the receiver off (no echo to take in). While an emulator runs, `uart3 open`,
`modbus start` and `spi x` are refused (`prog_emu stop` first). The pod's UART engine and a UART session exclude
each other, so the console must be closed while an UPDI job runs (the tests do that).

`make test-host-progemu BENCHPOD_FW=<benchpod-firmware checkout>` runs all of this on the host: the
pod's own `prog_<proto>.c` (branch `prog-flash` and later) against these models over `prog_sim`,
erase, write, read back, the lock and a stopped chip. It checks the models, not the interrupt code.

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

`tests/test_bus_master.py`: the pod as a bus master (gateware v55+; skips on older pods). Its
I2C master on LA1/LA2 writes and reads the node's `i2c target` at 100 kHz, 400 kHz and 1 MHz,
long transfers, a NACK, a scan, and the pull-up check. A capture with a `send` gets the
transaction and its decode, and SCL never runs faster than asked. SPI modes 0-3 run on
LA1/LA2/LA5/LA6, and a gpio send drives LA6 (both need `BENCH_LA6_RESISTOR=1`). `la_dir`
refuses a push-pull output on the node's alarm output.

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

- `BENCH_NEW_WIRING=1`: the USART3 (LA7/LA8) wires are in place; without it the UART sensor tests skip.
  The SPI2 wires need no variable: the tests probe them (the node switches SPI2 between modes 0 and 3
  and the pod reads SCK's idle level on its channel) and the SPI sensor tests skip when SCK is not
  there, as with the JTAG wiring below.
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

`tests/test_prog_emu.py` (marker `prog`; skips without the pod's `prog_flash`): for each emulated chip, a random
image through `load_bin` into PSRAM, `info` (ID), `start` with a chip erase, poll `status`, and the CRC32 of the
emulated flash (`prog_emu stat`) against the image's; a wrong `expect_id` (the flash is untouched), a locked chip
(refused without an erase, unlocked and flashed with one) and a stopped emulator (no answer). The edge timing
each run measured is printed (`-s`) and appended to `$PROG_EMU_TIMING_LOG`. See
[Chip emulators](#chip-emulators-prog_emu).

`tests/test_update.py`: flash, boot and identify the image N times
(`PROCESS_NODE_FLASH_CYCLES`, default 2; needs `--benchpod-firmware`).

`tests/test_can_hil.py`: both directions with standard and extended ids and 0 to 8 bytes, request/response
(echo), periodic timing from the pod's timestamps, bursts within and beyond the pod's RX
ring (overflow accounting), listen mode on either side (no ACK, error counters),
125k/250k/500k(/1M), a bitrate mismatch, and the pod's termination switch.
