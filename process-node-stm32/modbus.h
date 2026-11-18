/*
 * modbus.h - a Modbus RTU slave (server), the device end of the node's RS-485 mock (rs485mock.c).
 *
 * The protocol only: a whole frame in (address, PDU, CRC), the reply out. Pure C, no HAL, so
 * tests/test_modbus.c runs it on the host. Function codes:
 *
 *   01 Read Coils, 03 Read Holding Registers, 04 Read Input Registers, 05 Write Single Coil,
 *   06 Write Single Register, 08 Diagnostics (sub-function 0000, return query data),
 *   0F Write Multiple Coils, 10 Write Multiple Registers.
 *
 * Anything else is exception 01, an address outside the tables 02, a bad quantity, byte count or
 * coil value 03. Address 0 is a broadcast: writes are done, nothing is answered. A frame for
 * another address, or with a bad CRC, gets no answer either (Modbus over Serial Line V1.02).
 */
#ifndef MODBUS_H
#define MODBUS_H

#include <stddef.h>
#include <stdint.h>

#define MB_HOLD_REGS   64u     /* holding registers 0..63, 0 at start */
#define MB_INPUT_REGS  16u     /* input registers 0..15, 1000H + index */
#define MB_COILS       64u     /* coils 0..63, off at start */
#define MB_FRAME_MAX   256u    /* RTU ADU limit */

#define MB_EX_FUNCTION 0x01u
#define MB_EX_ADDRESS  0x02u
#define MB_EX_VALUE    0x03u

typedef struct {
    uint8_t  addr;
    uint16_t hold[MB_HOLD_REGS];
    uint16_t input[MB_INPUT_REGS];
    uint8_t  coil[MB_COILS / 8u];
    /* counters */
    uint32_t requests;     /* good frames for this address (or broadcast) */
    uint32_t replies;
    uint32_t exceptions;
    uint32_t crc_errors;   /* also frames shorter than 4 bytes */
    uint32_t other_addr;   /* good frames for another slave */
    uint32_t broadcasts;
    uint8_t  last_fn;
} mb_slave_t;

void mb_init(mb_slave_t *s, uint8_t addr);
/* Modbus CRC-16 (polynomial A001H reflected, start FFFFH); sent low byte first. */
uint16_t mb_crc16(const uint8_t *p, size_t n);
/* One frame. Returns the reply's length in out (MB_FRAME_MAX bytes), 0 for no reply. */
size_t mb_frame(mb_slave_t *s, const uint8_t *f, size_t n, uint8_t *out);

#endif
