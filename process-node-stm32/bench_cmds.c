#include "bench_cmds.h"
#include "dwt.h"
#include "env.h"
#include "i2cbus.h"
#include "i2ctarget.h"
#include "spi2.h"
#include "stm32f4xx_hal.h"
#include "uart3.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DRDY_PORT GPIOA
#define DRDY_PIN GPIO_PIN_8
#define MAX_WAIT_MS 5000U
#define UART3_READ_MAX 512U
#define NMEA_LINE_MAX 120U

/* ---------------------------------------------------------------- parsing */

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
    {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f')
    {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F')
    {
        return c - 'A' + 10;
    }
    return -1;
}

/* A hex token: one or two digits is one byte; a longer run of an even number of digits is
 * several bytes ("8f000000"). An optional 0x prefix. Appends to buf; -1 on bad input. */
static int parse_hex_bytes(const char *tok, uint8_t *buf, uint16_t *n, uint16_t max)
{
    if (tok[0] == '0' && (tok[1] == 'x' || tok[1] == 'X'))
    {
        tok += 2;
    }
    size_t len = strlen(tok);
    if (len == 0U || (len > 2U && (len % 2U) != 0U))
    {
        return -1;
    }
    if (len <= 2U)
    {
        int v = 0;
        for (size_t i = 0; i < len; i++)
        {
            int h = hexval(tok[i]);
            if (h < 0)
            {
                return -1;
            }
            v = v * 16 + h;
        }
        if (*n >= max)
        {
            return -1;
        }
        buf[(*n)++] = (uint8_t)v;
        return 0;
    }
    for (size_t i = 0; i < len; i += 2U)
    {
        int hi = hexval(tok[i]);
        int lo = hexval(tok[i + 1U]);
        if (hi < 0 || lo < 0 || *n >= max)
        {
            return -1;
        }
        buf[(*n)++] = (uint8_t)(hi * 16 + lo);
    }
    return 0;
}

static int parse_hex_u32(const char *s, uint32_t *out, uint32_t max)
{
    char *end;
    if (s == NULL || *s == '\0')
    {
        return -1;
    }
    unsigned long v = strtoul(s, &end, 16);
    if (*end != '\0' || v > max)
    {
        return -1;
    }
    *out = (uint32_t)v;
    return 0;
}

static int parse_dec(const char *s, uint32_t *out, uint32_t max)
{
    char *end;
    if (s == NULL || *s == '\0')
    {
        return -1;
    }
    unsigned long v = strtoul(s, &end, 10);
    if (*end != '\0' || v > max)
    {
        return -1;
    }
    *out = (uint32_t)v;
    return 0;
}

static void print_bytes(const uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0U; i < n; i++)
    {
        printf("%s%02x", i ? " " : "", b[i]);
    }
}

static void print_tenths(const char *key, int32_t v)
{
    int32_t a = v < 0 ? -v : v;
    printf(" %s=%s%ld.%ld", key, v < 0 ? "-" : "", (long)(a / 10), (long)(a % 10));
}

/* ---------------------------------------------------------------- i2c */

static void i2c_reply(i2cb_result_t res, const uint8_t *r, uint16_t nr)
{
    if (res.status == I2CB_OK)
    {
        printf("I2C ok r=");
        print_bytes(r, nr);
        printf("\r\n");
    }
    else if (res.status == I2CB_NACK_DATA)
    {
        printf("I2C err nack_data byte=%u\r\n", res.index);
    }
    else
    {
        printf("I2C err %s phase=%c\r\n", i2cb_status_name(res.status), res.phase);
    }
}

#define I2C_USAGE                                                                                  \
    "I2C err usage: i2c <addr> [w <b..>] [d <ms>] [r <n>] | i2c rd <addr> <reg> <n> | "            \
    "i2c wr <addr> <reg> <b..> | i2c scan | i2c probe <addr> | i2c cfg [hz <n>] [timeout <ms>] | " \
    "i2c recover | i2c target [<addr> | off | mem <reg> <n>]\r\n"

/* The register address: two hex digits are an 8-bit address, three or four a 16-bit one
 * (sent high byte first). */
static int parse_reg(const char *tok, uint8_t *w, uint16_t *nw)
{
    uint32_t reg;
    if (tok == NULL || parse_hex_u32(tok, &reg, 0xFFFFU) != 0)
    {
        return -1;
    }
    const char *digits = (tok[0] == '0' && (tok[1] == 'x' || tok[1] == 'X')) ? tok + 2 : tok;
    if (strlen(digits) > 2U)
    {
        w[(*nw)++] = (uint8_t)(reg >> 8);
    }
    w[(*nw)++] = (uint8_t)reg;
    return 0;
}

static void print_target(void)
{
    const i2ct_state_t *t = i2ct_state();
    printf("I2C target on=%u addr=%02x ptr=%02x matches=%lu rx=%lu tx=%lu errors=%lu\r\n", t->on,
           t->addr, t->ptr, (unsigned long)t->matches, (unsigned long)t->rx, (unsigned long)t->tx,
           (unsigned long)t->errors);
}

/* i2c target <addr>: answer at addr as a 256-byte register file (the env driver goes off);
 * i2c target off; i2c target mem <reg> <n>: what the master wrote; i2c target: counters. */
static void handle_i2c_target(void)
{
    char *a0 = strtok(NULL, " ");
    uint32_t v;
    uint32_t n;
    if (a0 == NULL)
    {
        print_target();
    }
    else if (strcmp(a0, "off") == 0)
    {
        if (i2ct_active())
        {
            i2ct_stop();
        }
        print_target();
    }
    else if (strcmp(a0, "mem") == 0)
    {
        if (parse_hex_u32(strtok(NULL, " "), &v, 0xFFU) != 0 || parse_dec(strtok(NULL, " "), &n, I2CB_MAX_BYTES) != 0 ||
            n < 1U)
        {
            printf("I2C err usage: i2c target mem <reg> <1..%u>\r\n", I2CB_MAX_BYTES);
            return;
        }
        printf("I2C target mem=");
        uint8_t buf[I2CB_MAX_BYTES];
        for (uint32_t i = 0; i < n; i++)
        {
            buf[i] = i2ct_mem()[(v + i) & 0xFFU];
        }
        print_bytes(buf, (uint16_t)n);
        printf("\r\n");
    }
    else if (parse_hex_u32(a0, &v, 0x7FU) == 0 && v >= 0x08U && v <= 0x77U)
    {
        env_set_enabled(0);
        (void)i2ct_start((uint8_t)v);
        print_target();
    }
    else
    {
        printf("I2C err usage: i2c target [<addr 08..77> | off | mem <reg> <n>]\r\n");
    }
}

static void handle_i2c(char *args)
{
    static uint8_t w[I2CB_MAX_BYTES + 2U];
    static uint8_t r[I2CB_MAX_BYTES];
    uint16_t nw = 0U;
    uint16_t nr = 0U;
    uint32_t addr;
    char *tok = strtok(args, " ");
    if (tok == NULL)
    {
        printf(I2C_USAGE);
        return;
    }
    if (strcmp(tok, "target") == 0)
    {
        handle_i2c_target();
        return;
    }
    if (i2ct_active())
    {
        printf("I2C err target mode is on; i2c target off first\r\n");
        return;
    }
    if (strcmp(tok, "scan") == 0)
    {
        printf("I2C scan found=");
        int any = 0;
        for (uint8_t a = 0x08U; a <= 0x77U; a++)
        {
            node_kick();
            if (i2cb_xfer(a, NULL, 0U, NULL, 0U, 0, 0U).status == I2CB_OK)
            {
                printf("%s%02x", any ? " " : "", a);
                any = 1;
            }
        }
        printf("\r\n");
        return;
    }
    if (strcmp(tok, "probe") == 0)
    {
        if (parse_hex_u32(strtok(NULL, " "), &addr, 0x7FU) != 0)
        {
            printf(I2C_USAGE);
            return;
        }
        i2cb_result_t res = i2cb_xfer((uint8_t)addr, NULL, 0U, NULL, 0U, 0, 0U);
        printf("I2C probe addr=%02lx %s\r\n", (unsigned long)addr,
               res.status == I2CB_OK ? "ack" : i2cb_status_name(res.status));
        return;
    }
    if (strcmp(tok, "cfg") == 0)
    {
        while ((tok = strtok(NULL, " ")) != NULL)
        {
            uint32_t v;
            if (strcmp(tok, "hz") == 0 && parse_dec(strtok(NULL, " "), &v, 400000U) == 0 &&
                env_i2c_set_hz(v) == 0)
            {
                continue;
            }
            if (strcmp(tok, "timeout") == 0 && parse_dec(strtok(NULL, " "), &v, 500U) == 0 && v > 0U)
            {
                i2cb_set_timeout_ms(v);
                continue;
            }
            printf("I2C err usage: i2c cfg [hz <10000..400000>] [timeout <1..500 ms>]\r\n");
            return;
        }
        printf("I2C cfg hz=%lu timeout_ms=%lu\r\n", (unsigned long)env_i2c_hz(),
               (unsigned long)i2cb_timeout_ms());
        return;
    }
    if (strcmp(tok, "recover") == 0)
    {
        int before = 0;
        int ok = i2cb_recover(&before);
        printf("I2C recover sda_before=%d %s\r\n", before, ok == 0 ? "ok" : "stuck");
        return;
    }
    if (strcmp(tok, "rd") == 0 || strcmp(tok, "wr") == 0)
    {
        int is_read = (tok[0] == 'r');
        if (parse_hex_u32(strtok(NULL, " "), &addr, 0x7FU) != 0 || parse_reg(strtok(NULL, " "), w, &nw) != 0)
        {
            printf(I2C_USAGE);
            return;
        }
        if (is_read)
        {
            uint32_t n;
            if (parse_dec(strtok(NULL, " "), &n, I2CB_MAX_BYTES) != 0 || n == 0U)
            {
                printf(I2C_USAGE);
                return;
            }
            nr = (uint16_t)n;
            i2c_reply(i2cb_xfer((uint8_t)addr, w, nw, r, nr, 1, 0U), r, nr);
            return;
        }
        while ((tok = strtok(NULL, " ")) != NULL)
        {
            if (parse_hex_bytes(tok, w, &nw, sizeof(w)) != 0)
            {
                printf(I2C_USAGE);
                return;
            }
        }
        i2c_reply(i2cb_xfer((uint8_t)addr, w, nw, NULL, 0U, 0, 0U), r, 0U);
        return;
    }

    /* Raw: i2c <addr> [w <b0> <b1> ...] [d <ms>] [r <n>], each phase its own START..STOP.
     *   i2c 44 w fd d 10 r 6      SHT4x: measure, wait, read T/RH with CRCs
     *   i2c 68 w 6b 00            MPU-6050: wake up */
    if (parse_hex_u32(tok, &addr, 0x7FU) != 0)
    {
        printf(I2C_USAGE);
        return;
    }
    uint32_t delay_ms = 0U;
    char mode = 0;
    while ((tok = strtok(NULL, " ")) != NULL)
    {
        if (strcmp(tok, "w") == 0 || strcmp(tok, "d") == 0 || strcmp(tok, "r") == 0)
        {
            mode = tok[0];
            continue;
        }
        uint32_t v;
        if ((mode == 'w' && parse_hex_bytes(tok, w, &nw, I2CB_MAX_BYTES) != 0) ||
            (mode == 'd' && parse_dec(tok, &delay_ms, 1500U) != 0) ||
            (mode == 'r' && (parse_dec(tok, &v, I2CB_MAX_BYTES) != 0 || (nr = (uint16_t)v, 0))) ||
            mode == 0)
        {
            printf(I2C_USAGE);
            return;
        }
    }
    if (nw == 0U && nr == 0U)
    {
        printf(I2C_USAGE);
        return;
    }
    i2c_reply(i2cb_xfer((uint8_t)addr, w, nw, r, nr, 0, delay_ms), r, nr);
}

/* ---------------------------------------------------------------- spi */

static void handle_spi(char *args)
{
    static uint8_t tx[SPI2_MAX_BYTES];
    static uint8_t rx[SPI2_MAX_BYTES];
    char *tok = strtok(args, " ");
    if (tok != NULL && strcmp(tok, "cfg") == 0)
    {
        uint32_t mode = spi2_mode();
        uint32_t hz = spi2_hz();
        while ((tok = strtok(NULL, " ")) != NULL)
        {
            if ((strcmp(tok, "mode") == 0 && parse_dec(strtok(NULL, " "), &mode, 3U) == 0) ||
                (strcmp(tok, "hz") == 0 && parse_dec(strtok(NULL, " "), &hz, 8000000U) == 0))
            {
                continue;
            }
            printf("SPI err usage: spi cfg [mode <0..3>] [hz <62500..8000000>]\r\n");
            return;
        }
        if (spi2_config((uint8_t)mode, hz) == 0U)
        {
            printf("SPI err usage: spi cfg [mode <0..3>] [hz <62500..8000000>]\r\n");
            return;
        }
        printf("SPI cfg mode=%u hz=%lu\r\n", spi2_mode(), (unsigned long)spi2_hz());
        return;
    }
    if (tok == NULL || strcmp(tok, "x") != 0)
    {
        printf("SPI err usage: spi x <bytes..> [z <n>] | spi cfg [mode <0..3>] [hz <n>]\r\n");
        return;
    }
    uint16_t n = 0U;
    while ((tok = strtok(NULL, " ")) != NULL)
    {
        if (strcmp(tok, "z") == 0)
        {
            uint32_t z;
            if (parse_dec(strtok(NULL, " "), &z, SPI2_MAX_BYTES) != 0 || n + z > SPI2_MAX_BYTES)
            {
                printf("SPI err too long (max %u bytes)\r\n", (unsigned)SPI2_MAX_BYTES);
                return;
            }
            memset(&tx[n], 0, z);
            n = (uint16_t)(n + z);
            continue;
        }
        if (parse_hex_bytes(tok, tx, &n, SPI2_MAX_BYTES) != 0)
        {
            printf("SPI err bad byte %s (hex, max %u bytes)\r\n", tok, (unsigned)SPI2_MAX_BYTES);
            return;
        }
    }
    if (n == 0U)
    {
        printf("SPI err usage: spi x <bytes..> [z <n>]\r\n");
        return;
    }
    spi2_xfer(tx, rx, n);
    printf("SPI ok n=%u r=", n);
    print_bytes(rx, n);
    printf("\r\n");
}

/* ---------------------------------------------------------------- uart3 */

static uint8_t g_nmea_on;
static char g_nmea_line[NMEA_LINE_MAX + 1U];
static uint16_t g_nmea_len;
static uint8_t g_nmea_overflow;
static uint32_t g_nmea_lines, g_nmea_ok, g_nmea_bad, g_nmea_junk;

static void print_uart3_stat(void)
{
    const uart3_stats_t *s = uart3_stats();
    printf("UART3 open=%u baud=%lu rx=%lu tx=%lu pending=%u framing=%lu noise=%lu overrun=%lu "
           "dropped=%lu\r\n",
           s->open, (unsigned long)s->baud, (unsigned long)s->rx_total, (unsigned long)s->tx_total,
           uart3_pending(), (unsigned long)s->framing, (unsigned long)s->noise,
           (unsigned long)s->overrun, (unsigned long)s->dropped);
}

static void print_nmea_stat(void)
{
    printf("NMEA mode=%s lines=%lu ok=%lu bad=%lu junk=%lu\r\n", g_nmea_on ? "on" : "off",
           (unsigned long)g_nmea_lines, (unsigned long)g_nmea_ok, (unsigned long)g_nmea_bad,
           (unsigned long)g_nmea_junk);
}

/* "$...*hh": the XOR of everything between $ (or !) and *. */
static int nmea_checksum_ok(const char *line)
{
    const char *star = strrchr(line, '*');
    if (star == NULL || star[1] == '\0' || star[2] == '\0' || star[3] != '\0')
    {
        return 0;
    }
    uint8_t x = 0U;
    for (const char *p = line + 1; p < star; p++)
    {
        x ^= (uint8_t)*p;
    }
    int hi = hexval(star[1]);
    int lo = hexval(star[2]);
    return hi >= 0 && lo >= 0 && (uint8_t)(hi * 16 + lo) == x;
}

static void nmea_line_done(void)
{
    g_nmea_line[g_nmea_len] = '\0';
    if (g_nmea_overflow || g_nmea_len == 0U || (g_nmea_line[0] != '$' && g_nmea_line[0] != '!'))
    {
        if (g_nmea_len > 0U || g_nmea_overflow)
        {
            g_nmea_junk++;
        }
    }
    else
    {
        int printable = 1;
        for (uint16_t i = 0U; i < g_nmea_len; i++)
        {
            if (g_nmea_line[i] < 32 || g_nmea_line[i] > 126)
            {
                printable = 0;
            }
        }
        if (!printable)
        {
            g_nmea_junk++;
        }
        else
        {
            int ok = nmea_checksum_ok(g_nmea_line);
            g_nmea_lines++;
            if (ok)
            {
                g_nmea_ok++;
            }
            else
            {
                g_nmea_bad++;
            }
            printf("NMEA %s %s\r\n", ok ? "ok" : "bad", g_nmea_line);
        }
    }
    g_nmea_len = 0U;
    g_nmea_overflow = 0U;
}

void bench_service(void)
{
    if (!g_nmea_on)
    {
        return;
    }
    uint8_t b;
    for (int budget = 0; budget < 128 && uart3_getc(&b); budget++)
    {
        if (b == '\n')
        {
            nmea_line_done();
        }
        else if (b == '\r')
        {
            continue;
        }
        else if (b == '$' && g_nmea_len > 0U)
        {
            /* a new sentence without the end of the last one (a dropped LF) */
            g_nmea_junk++;
            g_nmea_len = 0U;
            g_nmea_overflow = 0U;
            g_nmea_line[g_nmea_len++] = (char)b;
        }
        else if (g_nmea_len < NMEA_LINE_MAX)
        {
            g_nmea_line[g_nmea_len++] = (char)b;
        }
        else
        {
            g_nmea_overflow = 1U;
        }
    }
}

static int find(const uint8_t *buf, uint16_t n, const char *pat)
{
    size_t m = strlen(pat);
    if (m == 0U || n < m)
    {
        return 0;
    }
    for (uint16_t i = 0U; i + m <= n; i++)
    {
        if (memcmp(&buf[i], pat, m) == 0)
        {
            return 1;
        }
    }
    return 0;
}

/* uart3 read [t <ms>] [n <count>] [until <text...>]: collect for t ms (default 200), or until
 * count bytes arrived, or until the text did (the rest of the line, spaces included). */
static void uart3_read_cmd(char *rest)
{
    static uint8_t buf[UART3_READ_MAX];
    uint32_t t_ms = 200U;
    uint32_t want = 0U;
    const char *until = NULL;
    char *tok;
    while ((tok = strtok(rest, " ")) != NULL)
    {
        rest = NULL;
        if (strcmp(tok, "until") == 0)
        {
            until = strtok(NULL, ""); /* the rest of the line */
            break;
        }
        if ((strcmp(tok, "t") == 0 && parse_dec(strtok(NULL, " "), &t_ms, MAX_WAIT_MS) == 0) ||
            (strcmp(tok, "n") == 0 && parse_dec(strtok(NULL, " "), &want, UART3_READ_MAX) == 0))
        {
            continue;
        }
        printf("UART3 err usage: uart3 read [t <ms>] [n <count>] [until <text>]\r\n");
        return;
    }
    uint16_t n = 0U;
    int match = 0;
    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < t_ms && n < UART3_READ_MAX)
    {
        node_kick();
        uint8_t b;
        if (!uart3_getc(&b))
        {
            continue;
        }
        buf[n++] = b;
        if (until != NULL && find(buf, n, until))
        {
            match = 1;
            break;
        }
        if (want > 0U && n >= want)
        {
            match = 1;
            break;
        }
    }
    printf("UART3 rx n=%u match=%d hex=", n, match);
    for (uint16_t i = 0U; i < n; i++)
    {
        printf("%02x", buf[i]);
    }
    printf(" text=");
    for (uint16_t i = 0U; i < n; i++)
    {
        uint8_t c = buf[i];
        if (c == '\r')
        {
            printf("\\r");
        }
        else if (c == '\n')
        {
            printf("\\n");
        }
        else if (c == '\\')
        {
            printf("\\\\");
        }
        else if (c >= 32U && c <= 126U)
        {
            printf("%c", c);
        }
        else
        {
            printf("\\x%02x", c);
        }
    }
    printf("\r\n");
}

static void handle_uart3(char *args)
{
    static uint8_t tx[128];
    char *sub = strtok(args, " ");
    const uart3_stats_t *s = uart3_stats();
    if (sub == NULL || strcmp(sub, "stat") == 0)
    {
        print_uart3_stat();
        return;
    }
    if (strcmp(sub, "open") == 0)
    {
        uint32_t baud;
        if (parse_dec(strtok(NULL, " "), &baud, 1000000U) != 0 || uart3_open(baud) != 0)
        {
            printf("UART3 err usage: uart3 open <1200..1000000>\r\n");
            return;
        }
        printf("UART3 open baud=%lu\r\n", (unsigned long)baud);
        return;
    }
    if (strcmp(sub, "close") == 0)
    {
        uart3_close();
        g_nmea_on = 0U;
        printf("UART3 closed\r\n");
        return;
    }
    if (strcmp(sub, "nmea") == 0)
    {
        char *v = strtok(NULL, " ");
        if (v != NULL && strcmp(v, "on") == 0)
        {
            uart3_flush();
            g_nmea_lines = g_nmea_ok = g_nmea_bad = g_nmea_junk = 0U;
            g_nmea_len = 0U;
            g_nmea_overflow = 0U;
            g_nmea_on = 1U;
        }
        else if (v != NULL && strcmp(v, "off") == 0)
        {
            g_nmea_on = 0U;
        }
        else if (v != NULL)
        {
            printf("UART3 err usage: uart3 nmea [on|off]\r\n");
            return;
        }
        print_nmea_stat();
        return;
    }
    if (!s->open)
    {
        printf("UART3 err not open: uart3 open <baud>\r\n");
        return;
    }
    if (strcmp(sub, "send") == 0)
    {
        uint16_t n = 0U;
        char *tok;
        while ((tok = strtok(NULL, " ")) != NULL)
        {
            if (parse_hex_bytes(tok, tx, &n, sizeof(tx)) != 0)
            {
                printf("UART3 err bad byte %s (hex, max %u bytes)\r\n", tok, (unsigned)sizeof(tx));
                return;
            }
        }
        uart3_write(tx, n);
        printf("UART3 sent n=%u\r\n", n);
        return;
    }
    if (strcmp(sub, "line") == 0)
    {
        char *text = strtok(NULL, "");
        uint16_t n = 0U;
        if (text != NULL)
        {
            n = (uint16_t)strlen(text);
            uart3_write((const uint8_t *)text, n);
        }
        uart3_write((const uint8_t *)"\r\n", 2U);
        printf("UART3 sent n=%u\r\n", (unsigned)(n + 2U));
        return;
    }
    if (strcmp(sub, "read") == 0)
    {
        uart3_read_cmd(strtok(NULL, ""));
        return;
    }
    if (strcmp(sub, "flush") == 0)
    {
        uint16_t n = uart3_pending();
        uart3_flush();
        printf("UART3 flushed n=%u\r\n", n);
        return;
    }
    if (strcmp(sub, "count") == 0)
    {
        uint32_t ms;
        if (parse_dec(strtok(NULL, " "), &ms, MAX_WAIT_MS) != 0)
        {
            printf("UART3 err usage: uart3 count <ms>\r\n");
            return;
        }
        uart3_flush();
        uint32_t rx0 = s->rx_total;
        uint32_t fe0 = s->framing;
        uint32_t t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < ms)
        {
            node_kick();
            uart3_flush();
        }
        printf("UART3 count ms=%lu n=%lu framing=%lu\r\n", (unsigned long)ms,
               (unsigned long)(s->rx_total - rx0), (unsigned long)(s->framing - fe0));
        return;
    }
    printf("UART3 err usage: uart3 [stat] | open <baud> | close | send <hex..> | line <text> | "
           "read [t <ms>] [n <count>] [until <text>] | flush | count <ms> | nmea [on|off]\r\n");
}

/* ---------------------------------------------------------------- data ready (PA8) */

static int drdy_level(void)
{
    return (DRDY_PORT->IDR & DRDY_PIN) ? 1 : 0;
}

static void drdy_wait(const char *what, uint32_t ms)
{
    int want_edge;
    int want_level;
    if (strcmp(what, "rise") == 0 || strcmp(what, "high") == 0)
    {
        want_level = 1;
    }
    else if (strcmp(what, "fall") == 0 || strcmp(what, "low") == 0)
    {
        want_level = 0;
    }
    else
    {
        printf("DRDY err usage: drdy wait rise|fall|high|low [ms]\r\n");
        return;
    }
    want_edge = (what[0] == 'r' || what[0] == 'f');
    uint32_t c0 = dwt_cycles();
    uint32_t t0 = HAL_GetTick();
    int prev = drdy_level();
    for (;;)
    {
        int lvl = drdy_level();
        if (lvl == want_level && (!want_edge || prev != lvl))
        {
            printf("DRDY ok %s=%s us=%lu level=%d\r\n", want_edge ? "edge" : "level", what,
                   (unsigned long)dwt_cycles_to_us(dwt_cycles() - c0), lvl);
            return;
        }
        prev = lvl;
        if ((HAL_GetTick() - t0) >= ms)
        {
            printf("DRDY timeout ms=%lu level=%d\r\n", (unsigned long)ms, lvl);
            return;
        }
        node_kick();
    }
}

static void drdy_count(uint32_t ms)
{
    uint32_t rises = 0U, falls = 0U;
    uint32_t hmin = 0xFFFFFFFFU, hmax = 0U, pmin = 0xFFFFFFFFU, pmax = 0U;
    uint32_t last_rise = 0U, cyc_ms = SystemCoreClock / 1000U;
    uint32_t c0 = dwt_cycles();
    int prev = drdy_level();
    while ((dwt_cycles() - c0) < ms * cyc_ms)
    {
        node_kick();
        int lvl = drdy_level();
        if (lvl == prev)
        {
            continue;
        }
        uint32_t now = dwt_cycles();
        if (lvl)
        {
            if (rises > 0U)
            {
                uint32_t p = now - last_rise;
                pmin = p < pmin ? p : pmin;
                pmax = p > pmax ? p : pmax;
            }
            rises++;
            last_rise = now;
        }
        else
        {
            falls++;
            if (rises > 0U)
            {
                uint32_t h = now - last_rise;
                hmin = h < hmin ? h : hmin;
                hmax = h > hmax ? h : hmax;
            }
        }
        prev = lvl;
    }
    printf("DRDY count ms=%lu rises=%lu falls=%lu high_min_us=%lu high_max_us=%lu "
           "period_min_us=%lu period_max_us=%lu level=%d\r\n",
           (unsigned long)ms, (unsigned long)rises, (unsigned long)falls,
           (unsigned long)(hmax ? dwt_cycles_to_us(hmin) : 0U), (unsigned long)dwt_cycles_to_us(hmax),
           (unsigned long)(pmax ? dwt_cycles_to_us(pmin) : 0U), (unsigned long)dwt_cycles_to_us(pmax),
           prev);
}

/* drdy | drdy wait rise|fall|high|low [ms] | drdy count <ms>. Only while the strobe is off:
 * PA8 is then an input, and the pod may drive it through the 1 kOhm on LA6. */
static void handle_drdy(char *args)
{
    if (!strobe_is_off())
    {
        printf("DRDY err strobe is on: run strobe off first\r\n");
        return;
    }
    char *sub = strtok(args, " ");
    if (sub == NULL)
    {
        printf("DRDY level=%d\r\n", drdy_level());
        return;
    }
    uint32_t ms = 1000U;
    char *arg = strtok(NULL, " ");
    if (strcmp(sub, "wait") == 0)
    {
        char *t = strtok(NULL, " ");
        if (arg == NULL || (t != NULL && parse_dec(t, &ms, MAX_WAIT_MS) != 0))
        {
            printf("DRDY err usage: drdy wait rise|fall|high|low [ms]\r\n");
            return;
        }
        drdy_wait(arg, ms);
        return;
    }
    if (strcmp(sub, "count") == 0 && parse_dec(arg, &ms, MAX_WAIT_MS) == 0 && ms > 0U)
    {
        drdy_count(ms);
        return;
    }
    printf("DRDY err usage: drdy | drdy wait rise|fall|high|low [ms] | drdy count <ms>\r\n");
}

/* ---------------------------------------------------------------- env driver */

static void handle_env(char *args)
{
    static const char *const status_names[] = {"absent", "ok", "lost"};
    static const char *const chip_names[] = {"none", "bmp280", "bme280"};
    char *v = strtok(args, " ");
    if (v != NULL && strcmp(v, "on") == 0 && i2ct_active())
    {
        printf("ENV err the I2C target mode is on; i2c target off first\r\n");
        return;
    }
    if (v != NULL && strcmp(v, "on") == 0)
    {
        env_set_enabled(1);
    }
    else if (v != NULL && strcmp(v, "off") == 0)
    {
        env_set_enabled(0);
    }
    else if (v != NULL)
    {
        printf("ENV err usage: env [on|off]\r\n");
        return;
    }
    const env_state_t *e = env_state();
    printf("ENV driver=%s status=%s chip=%s addr=%02x", env_enabled() ? "on" : "off",
           status_names[e->status], chip_names[e->chip], e->addr);
    print_tenths("temp_c", e->temp_dc);
    printf(" press_pa=%lu", (unsigned long)e->press_pa);
    print_tenths("hum_pct", e->hum_dpct);
    printf(" reads=%lu fails=%lu\r\n", (unsigned long)e->reads, (unsigned long)e->fails);
}

/* ---------------------------------------------------------------- entry points */

void bench_init(void)
{
    dwt_init();
    spi2_init();
    uart3_close(); /* PC10/PC11 inputs until a test opens it */
}

static int is_cmd(const char *cmd, const char *name, char **args)
{
    size_t n = strlen(name);
    if (strncmp(cmd, name, n) != 0 || (cmd[n] != '\0' && cmd[n] != ' '))
    {
        return 0;
    }
    *args = (char *)cmd + n;
    return 1;
}

int bench_command(char *cmd)
{
    char *args;
    if (is_cmd(cmd, "i2c", &args))
    {
        handle_i2c(args);
    }
    else if (is_cmd(cmd, "spi", &args))
    {
        handle_spi(args);
    }
    else if (is_cmd(cmd, "uart3", &args))
    {
        handle_uart3(args);
    }
    else if (is_cmd(cmd, "drdy", &args))
    {
        handle_drdy(args);
    }
    else if (is_cmd(cmd, "env", &args))
    {
        handle_env(args);
    }
    else
    {
        return 0;
    }
    return 1;
}
