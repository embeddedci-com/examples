# process-node-stm32

A CAN node on a NUCLEO-F446RE with an MCP2515/TJA1050 module, used as the second node
when testing the BenchPod's CAN port. The Nucleo runs a UART console; the tests drive it
over that console and talk to it from the pod's CAN API.

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

Without `--benchpod-firmware` the tests use the firmware already on the board. Set
`CAN_NODE_OSC_MHZ=16` for a 16 MHz module (adds the 1 Mbit/s case).

Covered: both directions with standard and extended ids and 0 to 8 bytes, request/response
(echo), periodic timing from the pod's timestamps, bursts within and beyond the pod's RX
ring (overflow accounting), listen mode on either side (no ACK, error counters),
125k/250k/500k(/1M), a bitrate mismatch, and the pod's termination switch.
