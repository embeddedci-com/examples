# can-stm32

A CAN node on a NUCLEO-F446RE with an MCP2515/TJA1050 module, used as the second node
when testing the BenchPod's CAN port. The Nucleo runs a UART console; the tests drive it
over that console and talk to it from the pod's CAN API.

## Wiring

The module's TJA1050 needs 5 V, which puts its SPI outputs at 5 V. A TXS0108E shifts
them to the Nucleo's 3.3 V.

| Nucleo (A side, 3.3 V) | TXS0108E | MCP2515 module (B side, 5 V) |
|---|---|---|
| D13 / PA5 (SPI1 SCK) | A1 / B1 | SCK |
| D12 / PA6 (SPI1 MISO) | A2 / B2 | SO |
| D11 / PA7 (SPI1 MOSI) | A3 / B3 | SI |
| D10 / PB6 (CS) | A4 / B4 | CS |
| D9 / PC7 (INT) | A5 / B5 | INT |
| 3V3 | VCCA, OE | |
| 5V | VCCB | VCC |
| GND | GND | GND |

- Tie **OE to VCCA**, or every channel stays off.
- Module **H to pod CAN+, L to pod CAN-**, plus a common ground.
- Fit the module's **120R jumper**. The tests switch on the pod's termination, so the
  bus reads about 60 ohm between CAN+ and CAN- with both powered.
- Console: USART1 PA9 (TX) / PA10 (RX), 115200 8N1. SWD and UART go to the pod as in
  `scenario-sensors-stm32`.

## Build

```bash
make                         # build/can-node.elf, 8 MHz module crystal
make MCP2515_OSC_HZ=16000000 # module with a 16 MHz crystal
make test-host               # bit-timing unit test, no hardware
```

With 8 MHz the fastest bitrate is 500 kbit/s; 1 Mbit/s needs a 16 MHz crystal.

## Console

| Command | Does |
|---|---|
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
pytest can-stm32/tests -v --benchpod-connection=<host or embeddedci:name> \
    --benchpod-firmware=can-stm32/build/can-node.elf
```

Without `--benchpod-firmware` the tests use the firmware already on the board. Set
`CAN_NODE_OSC_MHZ=16` for a 16 MHz module (adds the 1 Mbit/s case).

Covered: both directions with standard and extended ids and 0 to 8 bytes, request/response
(echo), periodic timing from the pod's timestamps, bursts within and beyond the pod's RX
ring (overflow accounting), listen mode on either side (no ACK, error counters),
125k/250k/500k(/1M), and a bitrate mismatch.
