/* Host unit test for modbus.c, the Modbus RTU slave of the RS-485 mock: CRC, every function
 * code, the exceptions, broadcast and other addresses. Run: make test-host */
#include "modbus.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL %s:%d: ", __func__, __LINE__); \
        printf(__VA_ARGS__); printf("\n"); failures++; } } while (0)

static mb_slave_t S;
static uint8_t out[MB_FRAME_MAX];

/* Send the bytes given (address + PDU), with the CRC appended; returns the reply length. */
static size_t req(int n, ...)
{
    uint8_t f[MB_FRAME_MAX];
    va_list ap;
    va_start(ap, n);
    for (int i = 0; i < n; i++) f[i] = (uint8_t)va_arg(ap, int);
    va_end(ap);
    uint16_t c = mb_crc16(f, (size_t)n);
    f[n] = (uint8_t)c;
    f[n + 1] = (uint8_t)(c >> 8);
    return mb_frame(&S, f, (size_t)n + 2u, out);
}

static int crc_ok(size_t n)
{
    uint16_t c = mb_crc16(out, n - 2u);
    return out[n - 2u] == (uint8_t)c && out[n - 1u] == (uint8_t)(c >> 8);
}

static int exception(size_t n, int fn, int code)
{
    return n == 5 && out[0] == S.addr && out[1] == (fn | 0x80) && out[2] == code && crc_ok(n);
}

static void test_crc(void)
{
    /* the spec's example: 02 07 -> CRC 1241H, sent 41 12 */
    const uint8_t a[] = {0x02, 0x07};
    CHECK(mb_crc16(a, 2) == 0x1241, "crc %04x", mb_crc16(a, 2));
    /* a well-known read request: 01 03 00 00 00 01 -> 84 0A */
    const uint8_t b[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    CHECK(mb_crc16(b, 6) == 0x0A84, "crc %04x", mb_crc16(b, 6));
}

static void test_registers(void)
{
    mb_init(&S, 1);
    size_t n = req(6, 1, 0x03, 0, 0, 0, 2);
    CHECK(n == 9 && out[2] == 4 && out[3] == 0 && out[6] == 0 && crc_ok(n), "read holding");
    n = req(6, 1, 0x06, 0, 5, 0x12, 0x34);
    CHECK(n == 8 && out[1] == 0x06 && out[4] == 0x12 && out[5] == 0x34 && S.hold[5] == 0x1234, "write single");
    n = req(11, 1, 0x10, 0, 10, 0, 2, 4, 0xAB, 0xCD, 0x00, 0x01);
    CHECK(n == 8 && out[1] == 0x10 && out[5] == 2 && S.hold[10] == 0xABCD && S.hold[11] == 1, "write multiple");
    n = req(6, 1, 0x03, 0, 10, 0, 2);
    CHECK(n == 9 && out[3] == 0xAB && out[4] == 0xCD && out[5] == 0 && out[6] == 1, "read back");
    n = req(6, 1, 0x04, 0, 14, 0, 2);
    CHECK(n == 9 && out[3] == 0x10 && out[4] == 0x0E && out[5] == 0x10 && out[6] == 0x0F, "input regs");
    /* the last register is fine, one past it is not */
    CHECK(req(6, 1, 0x03, 0, 63, 0, 1) == 7, "last holding register");
    CHECK(exception(req(6, 1, 0x03, 0, 63, 0, 2), 0x03, MB_EX_ADDRESS), "past the end");
    CHECK(exception(req(6, 1, 0x04, 0, 16, 0, 1), 0x04, MB_EX_ADDRESS), "input past the end");
    CHECK(exception(req(6, 1, 0x03, 0, 0, 0, 0), 0x03, MB_EX_VALUE), "quantity 0");
    CHECK(exception(req(6, 1, 0x03, 0, 0, 0, 126), 0x03, MB_EX_VALUE), "quantity 126");
    CHECK(exception(req(6, 1, 0x06, 0, 64, 0, 1), 0x06, MB_EX_ADDRESS), "write past the end");
    CHECK(exception(req(11, 1, 0x10, 0, 0, 0, 2, 3, 0, 0, 0, 0), 0x10, MB_EX_VALUE), "byte count");
}

static void test_coils(void)
{
    mb_init(&S, 7);
    size_t n = req(6, 7, 0x05, 0, 3, 0xFF, 0x00);
    CHECK(n == 8 && out[4] == 0xFF, "coil 3 on");
    n = req(9, 7, 0x0F, 0, 8, 0, 10, 2, 0x05, 0x02);   /* coils 8..17: 8, 10 and 17 on */
    CHECK(n == 8 && out[1] == 0x0F, "write coils");
    n = req(6, 7, 0x01, 0, 0, 0, 20);
    CHECK(n == 8 && out[2] == 3 && out[3] == 0x08 && out[4] == 0x05 && out[5] == 0x02 && crc_ok(n),
          "read coils %02x %02x %02x", out[3], out[4], out[5]);
    CHECK(exception(req(6, 7, 0x05, 0, 0, 0x12, 0x34), 0x05, MB_EX_VALUE), "coil value 1234H");
    CHECK(exception(req(6, 7, 0x01, 0, 60, 0, 5), 0x01, MB_EX_ADDRESS), "coils past the end");
}

static void test_diagnostics_and_unknown(void)
{
    mb_init(&S, 1);
    size_t n = req(6, 1, 0x08, 0, 0, 0xA5, 0x37);
    CHECK(n == 8 && out[1] == 0x08 && out[4] == 0xA5 && out[5] == 0x37, "echo");
    CHECK(exception(req(6, 1, 0x08, 0, 1, 0, 0), 0x08, MB_EX_FUNCTION), "sub-function 1");
    CHECK(exception(req(2, 1, 0x2B), 0x2B, MB_EX_FUNCTION), "function 2BH");
    CHECK(S.exceptions == 2 && S.replies == 3, "counters %u %u", (unsigned)S.exceptions, (unsigned)S.replies);
}

static void test_addressing_and_crc(void)
{
    mb_init(&S, 1);
    CHECK(req(6, 2, 0x03, 0, 0, 0, 1) == 0 && S.other_addr == 1, "another slave answered");
    CHECK(req(6, 0, 0x06, 0, 1, 0, 9) == 0 && S.hold[1] == 9 && S.broadcasts == 1, "broadcast write");
    uint8_t f[] = {1, 0x03, 0, 0, 0, 1, 0x84, 0x0B};   /* last CRC byte off by one */
    CHECK(mb_frame(&S, f, sizeof(f), out) == 0 && S.crc_errors == 1, "bad CRC answered");
    CHECK(mb_frame(&S, f, 3, out) == 0 && S.crc_errors == 2, "a 3-byte frame");
    f[7] = 0x0A;
    CHECK(mb_frame(&S, f, sizeof(f), out) == 7, "good CRC refused");
}

int main(void)
{
    test_crc();
    test_registers();
    test_coils();
    test_diagnostics_and_unknown();
    test_addressing_and_crc();
    if (failures) { printf("test_modbus: %d FAILED\n", failures); return 1; }
    printf("test_modbus: all passed\n");
    return 0;
}
