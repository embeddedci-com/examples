/*
 * rs485mock.h - an RS-485 bus with one Modbus RTU device on it, as a half-duplex transceiver
 * would present it to the pod, on the bench's TTL wires:
 *
 *   transceiver DI (the pod sends)   LA8 -> PC11, USART3 RX
 *   transceiver RO (the pod listens) LA7 <- PC10, USART3 TX
 *   transceiver DE + /RE             LA6 -> PA8 (through 1 kOhm; pulled down, so it listens)
 *
 * The rules a real transceiver enforces, so a master's direction handling is tested too:
 *  - a byte from the pod reaches the bus only while DE is high (sampled as the byte completes);
 *    others are counted as `undriven` and never reach the device;
 *  - the device's reply reaches the pod only while DE is low (DE high also turns the pod's
 *    receiver off); a reply byte sent while DE is high is lost and counted as a `collision`.
 *
 * With `de off` the DE rules are not applied: a transceiver with automatic direction control (it
 * turns its driver on while its DI input is active), so only the protocol is tested.
 *
 * Frames are delimited by 3.5 characters of silence (1750 us above 19200 baud); a gap of more than
 * 1.5 characters inside a frame, a framing or parity error drops it (Modbus over Serial Line
 * V1.02). The device (modbus.c) answers `delay_us` after the request, never sooner than 3.5
 * characters. Console: `modbus start|stop|stat|reg|delay` (bench_cmds.c help).
 */
#ifndef RS485MOCK_H
#define RS485MOCK_H

#include <stdint.h>

#include "modbus.h"

typedef struct {
    uint8_t  on;
    uint32_t baud;
    char     format[4];
    uint32_t delay_us;
    uint8_t  de;            /* 1: DE on PA8 is honored; 0: automatic direction (no DE wire) */
    uint32_t undriven;      /* bytes from the pod while DE was low: not on the bus */
    uint32_t collisions;    /* reply bytes while DE was high: the pod could not hear them */
    uint32_t frame_errors;  /* a gap over 1.5 characters, a framing / parity error, too long */
} rs485_stats_t;

int  rs485_start(uint8_t addr, uint32_t baud, char parity, uint8_t stop_bits, uint32_t delay_us,
                 uint8_t de);
void rs485_stop(void);
void rs485_set_delay(uint32_t delay_us);
/* Main loop: end of frame detection and the reply. */
void rs485_service(void);
const rs485_stats_t *rs485_stats(void);
mb_slave_t *rs485_slave(void);

#endif
