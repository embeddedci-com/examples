/*
 * Process-control node: NUCLEO-F446RE + MCP2515/TJA1050 module, driven from a
 * UART console so a BenchPod (or a person) can exercise it like a real product.
 *
 * Wiring (Nucleo Arduino header -> MCP2515 module, direct):
 *   D13 PA5  SPI1_SCK  -> SCK
 *   D12 PA6  SPI1_MISO <- SO
 *   D11 PA7  SPI1_MOSI -> SI
 *   D10 PB6  CS        -> CS
 *   D9  PC7  INT       <- INT (active low)
 *   5V -> module VCC, GND common
 * The module's TJA1050 needs 5 V, which makes its SPI outputs 5 V too; PA6 and
 * PC7 are 5 V tolerant. A TXS0108E in between is optional (see README).
 *
 * Analog in: PA1 (ADC1_IN1, Nucleo A1) <- pod 3.3 V DAC SMA via 10 kOhm.
 *
 * Console: USART1 PA9 (TX) / PA10 (RX), 115200 8N1. Type "help".
 */

#include "stm32f4xx_hal.h"
#include "analog.h"
#include "mcp2515.h"
#include "mcp2515_timing.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FW_NAME "process-node"
#define FW_VERSION "0.2.0"

#define CMD_BUF_SIZE 96

/* Independent watchdog: LSI ~32 kHz / 64 = 500 Hz, reload 1000 -> ~2 s. The main
 * loop kicks it every pass; long blocking work (CAN bursts) kicks it as it goes. */
#define IWDG_RELOAD 1000U

/* Boot attach window (ms): hold before touching peripherals so a flasher that
 * just reset us can connect SWD during a known-quiet window. */
#define FLASH_ATTACH_WINDOW_MS 200U

#ifndef MCP2515_OSC_HZ
#define MCP2515_OSC_HZ 8000000U /* crystal on the module; most ship with 8 MHz */
#endif
#define CAN_DEFAULT_BITRATE 500000U

#define CAN_TX_TIMEOUT_MS 50U    /* no ACK within this -> abort + report */
#define CAN_BURST_MAX 1000U
#define CAN_SERVICE_MAX_FRAMES 8U /* per main-loop pass, so the console stays responsive */

#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_6
#define INT_PORT GPIOC
#define INT_PIN GPIO_PIN_7

static volatile char cmd_buffer[CMD_BUF_SIZE];
static volatile uint8_t cmd_index = 0;
static volatile uint8_t cmd_ready = 0;
static uint8_t rx_last_was_cr = 0U;

#define RX_RING_SIZE 256u
static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_ring_head = 0;
static volatile uint16_t rx_ring_tail = 0;

typedef struct
{
    uint8_t chip_ok;
    uint8_t init_ok;
    uint32_t osc_hz;
    uint32_t bitrate;
    uint8_t echo;
    uint8_t print;
    uint32_t rx;
    uint32_t tx;
    uint32_t tx_fail;
    uint32_t rx_overflow;
    uint8_t last_eflg;
} can_state_t;

typedef struct
{
    uint8_t on;
    uint32_t id;
    uint8_t ext;
    uint32_t period_ms;
    uint32_t next_ms;
    uint32_t sent;
    uint32_t limit; /* 0 = forever */
} periodic_t;

static can_state_t g_can = {
    .osc_hz = MCP2515_OSC_HZ,
    .bitrate = CAN_DEFAULT_BITRATE,
    .print = 1U,
};
static periodic_t g_periodic;
static const char *g_reset_cause = "unknown";

void SystemClock_Config(void);
static void USART1_Init(void);
static void SPI1_Init(void);
static int usart1_read_byte_nonblocking(uint8_t *byte);
static void uart_process_rx_byte(uint8_t byte);
static void print_prompt(void);
static void process_command(char *cmd);
static void print_help(void);
static void can_init(uint32_t bitrate);
static void can_service(void);
static void can_periodic_tick(void);
static void can_print_status(void);
static void handle_can(char *args);
static void reset_cause_capture(void);
static void iwdg_start(void);
static void iwdg_kick(void);
static void print_info(void);

int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        while ((USART1->SR & USART_SR_TXE) == 0U)
        {
        }
        USART1->DR = (uint8_t)ptr[i];
    }
    return len;
}

int main(void)
{
    reset_cause_capture();
    HAL_Init();
    SystemClock_Config();

    HAL_DBGMCU_EnableDBGSleepMode();
    HAL_DBGMCU_EnableDBGStopMode();
    HAL_DBGMCU_EnableDBGStandbyMode();
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP; /* a halted core must not be reset */
    HAL_Delay(FLASH_ATTACH_WINDOW_MS);
    iwdg_start();

    USART1_Init();
    SPI1_Init();
    (void)setvbuf(stdout, NULL, _IONBF, 0);

    printf("\r\nPROCESS-NODE: stm32f446 + mcp2515\r\n");
    print_info();
    printf("PROCESS-NODE: uart=USART1(115200), spi=SPI1 PA5/PA6/PA7, cs=PB6, int=PC7\r\n");
    can_init(g_can.bitrate);
    ain_init();
    printf("APP_OK\r\n");
    printf("type a command and press Enter (e.g. help)\r\n");
    print_prompt();

    while (1)
    {
        iwdg_kick();
        uint8_t b;
        while (usart1_read_byte_nonblocking(&b))
        {
            uart_process_rx_byte(b);
        }

        if (cmd_ready)
        {
            cmd_ready = 0;
            process_command((char *)cmd_buffer);
            memset((void *)cmd_buffer, 0, CMD_BUF_SIZE);
            cmd_index = 0;
            print_prompt();
        }

        ain_tick(HAL_GetTick());

        if (g_can.init_ok)
        {
            can_service();
            can_periodic_tick();
        }

        /* Woken by SysTick (1 ms), USART1 RX or the MCP2515 INT edge. */
        __WFI();
    }
}

/* ---------------------------------------------------------------- MCP2515 board hooks */

void mcp_board_select(int active)
{
    if (active)
    {
        CS_PORT->BSRR = (uint32_t)CS_PIN << 16U;
    }
    else
    {
        while (SPI1->SR & SPI_SR_BSY)
        {
        }
        CS_PORT->BSRR = CS_PIN;
    }
}

uint8_t mcp_board_xfer(uint8_t out)
{
    while ((SPI1->SR & SPI_SR_TXE) == 0U)
    {
    }
    *(volatile uint8_t *)&SPI1->DR = out;
    while ((SPI1->SR & SPI_SR_RXNE) == 0U)
    {
    }
    return *(volatile uint8_t *)&SPI1->DR;
}

uint32_t mcp_board_millis(void)
{
    return HAL_GetTick();
}

/* ---------------------------------------------------------------- CAN application */

static const char *mode_name(mcp_mode_t m)
{
    switch (m)
    {
    case MCP_MODE_NORMAL: return "normal";
    case MCP_MODE_SLEEP: return "sleep";
    case MCP_MODE_LOOPBACK: return "loopback";
    case MCP_MODE_LISTEN: return "listen";
    case MCP_MODE_CONFIG: return "config";
    default: return "?";
    }
}

static void can_init(uint32_t bitrate)
{
    g_can.init_ok = 0U;
    g_periodic.on = 0U;
    g_can.chip_ok = (mcp_reset() == 0) ? 1U : 0U;
    if (!g_can.chip_ok)
    {
        printf("CAN init fail: mcp2515 not found (check SPI wiring, TXS0108E OE, 5V)\r\n");
        return;
    }
    mcp2515_timing_t t;
    if (mcp2515_calc_timing(g_can.osc_hz, bitrate, &t) != 0)
    {
        printf("CAN init fail: bitrate %lu not possible with a %lu Hz crystal\r\n",
               (unsigned long)bitrate, (unsigned long)g_can.osc_hz);
        return;
    }
    if (mcp_configure(g_can.osc_hz, bitrate) != 0 || mcp_set_mode(MCP_MODE_NORMAL) != 0)
    {
        printf("CAN init fail: mcp2515 did not take the configuration\r\n");
        return;
    }
    g_can.bitrate = bitrate;
    g_can.init_ok = 1U;
    printf("CAN init ok bitrate=%lu osc=%lu tq=%u sp=%u%% cnf=%02X %02X %02X\r\n",
           (unsigned long)bitrate, (unsigned long)g_can.osc_hz, t.tq,
           (unsigned)((1U + t.prseg + t.ps1) * 100U / t.tq), t.cnf1, t.cnf2, t.cnf3);
}

static void print_frame(const char *tag, const can_frame_t *f)
{
    printf("CAN %s id=0x%lX ext=%u rtr=%u dlc=%u data=", tag, (unsigned long)f->id,
           f->ext, f->rtr, f->dlc);
    for (uint32_t i = 0; i < f->dlc && !f->rtr; i++)
    {
        printf("%02X", f->data[i]);
    }
    printf("\r\n");
}

static mcp_tx_result_t can_send(const can_frame_t *f)
{
    iwdg_kick(); /* bursts send up to CAN_BURST_MAX frames from one command */
    mcp_tx_result_t r = mcp_send(f, CAN_TX_TIMEOUT_MS);
    if (r == MCP_TX_OK)
    {
        g_can.tx++;
    }
    else
    {
        g_can.tx_fail++;
    }
    return r;
}

/* Drain received frames and error flags. Called every main-loop pass; the INT
 * pin (active low) tells us cheaply whether there is anything to do. */
static void can_service(void)
{
    if (HAL_GPIO_ReadPin(INT_PORT, INT_PIN) == GPIO_PIN_SET)
    {
        return;
    }

    /* Bounded: a node that never gets ACKed retransmits back to back, and an
     * unbounded drain would never return to the console. */
    can_frame_t f;
    for (uint32_t n = 0; n < CAN_SERVICE_MAX_FRAMES && mcp_receive(&f); n++)
    {
        g_can.rx++;
        if (g_can.echo)
        {
            /* Reply first so the round-trip latency doesn't include the print. */
            can_frame_t r = f;
            r.id = (f.id + 1U) & (f.ext ? 0x1FFFFFFFU : 0x7FFU);
            (void)can_send(&r);
        }
        if (g_can.print)
        {
            print_frame("rx", &f);
        }
    }

    uint8_t intf = mcp_read_reg(MCP_CANINTF);
    if (intf & MCP_INT_ERR)
    {
        uint8_t eflg = mcp_read_reg(MCP_EFLG);
        g_can.last_eflg = eflg;
        if (eflg & (MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR))
        {
            g_can.rx_overflow++;
            mcp_bit_modify(MCP_EFLG, MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR, 0x00U);
        }
        mcp_bit_modify(MCP_CANINTF, MCP_INT_ERR, 0x00U);
    }
}

static void can_periodic_tick(void)
{
    if (!g_periodic.on)
    {
        return;
    }
    uint32_t now = HAL_GetTick();
    if ((int32_t)(now - g_periodic.next_ms) < 0)
    {
        return;
    }
    /* Keep a fixed schedule (no drift) but never queue up a backlog. */
    g_periodic.next_ms += g_periodic.period_ms;
    if ((int32_t)(now - g_periodic.next_ms) >= 0)
    {
        g_periodic.next_ms = now + g_periodic.period_ms;
    }

    can_frame_t f = {.id = g_periodic.id, .ext = g_periodic.ext, .dlc = 2U};
    f.data[0] = (uint8_t)g_periodic.sent;
    f.data[1] = (uint8_t)(g_periodic.sent >> 8);
    (void)can_send(&f);
    g_periodic.sent++;
    if (g_periodic.limit != 0U && g_periodic.sent >= g_periodic.limit)
    {
        g_periodic.on = 0U;
        printf("CAN periodic done sent=%lu\r\n", (unsigned long)g_periodic.sent);
    }
}

static void can_print_status(void)
{
    uint8_t eflg = g_can.chip_ok ? mcp_read_reg(MCP_EFLG) : 0U;
    printf("CAN status: chip=%s init=%s mode=%s bitrate=%lu osc=%lu tec=%u rec=%u eflg=0x%02X "
           "busoff=%u errpassive=%u rx=%lu tx=%lu txfail=%lu rxovr=%lu echo=%s print=%s "
           "periodic=%s\r\n",
           g_can.chip_ok ? "ok" : "missing", g_can.init_ok ? "ok" : "fail",
           g_can.chip_ok ? mode_name(mcp_get_mode()) : "none",
           (unsigned long)g_can.bitrate, (unsigned long)g_can.osc_hz,
           g_can.chip_ok ? mcp_read_reg(MCP_TEC) : 0U, g_can.chip_ok ? mcp_read_reg(MCP_REC) : 0U,
           eflg, (eflg & MCP_EFLG_TXBO) ? 1U : 0U,
           (eflg & (MCP_EFLG_TXEP | MCP_EFLG_RXEP)) ? 1U : 0U,
           (unsigned long)g_can.rx, (unsigned long)g_can.tx, (unsigned long)g_can.tx_fail,
           (unsigned long)g_can.rx_overflow, g_can.echo ? "on" : "off",
           g_can.print ? "on" : "off", g_periodic.on ? "on" : "off");
}

/* Parse "<id> [b0 .. b7]" (hex) into a frame. Returns 0 on success. */
static int parse_frame(char *rest, int force_ext, can_frame_t *f)
{
    memset(f, 0, sizeof(*f));
    char *tok = strtok(rest, " ");
    if (tok == NULL)
    {
        return -1;
    }
    char *end;
    unsigned long id = strtoul(tok, &end, 16);
    if (*end != '\0' || id > 0x1FFFFFFFUL)
    {
        return -1;
    }
    f->id = (uint32_t)id;
    f->ext = (force_ext || id > 0x7FFUL) ? 1U : 0U;
    while ((tok = strtok(NULL, " ")) != NULL)
    {
        if (f->dlc >= 8U)
        {
            return -1;
        }
        unsigned long v = strtoul(tok, &end, 16);
        if (*end != '\0' || v > 0xFFUL)
        {
            return -1;
        }
        f->data[f->dlc++] = (uint8_t)v;
    }
    return 0;
}

static int parse_on_off(const char *s, uint8_t *out)
{
    if (s != NULL && strcmp(s, "on") == 0)
    {
        *out = 1U;
        return 0;
    }
    if (s != NULL && strcmp(s, "off") == 0)
    {
        *out = 0U;
        return 0;
    }
    return -1;
}

static void handle_can_send(char *rest, int force_ext)
{
    can_frame_t f;
    if (parse_frame(rest, force_ext, &f) != 0)
    {
        printf("usage: can send <id-hex> [byte-hex ...]  (max 8 bytes)\r\n");
        return;
    }
    mcp_tx_result_t r = can_send(&f);
    if (r == MCP_TX_OK)
    {
        printf("CAN tx ok id=0x%lX dlc=%u\r\n", (unsigned long)f.id, f.dlc);
    }
    else
    {
        printf("CAN tx fail: %s tec=%u\r\n", (r == MCP_TX_TIMEOUT) ? "timeout (no ACK?)" : "error",
               mcp_read_reg(MCP_TEC));
    }
}

static void handle_can_burst(char *rest)
{
    char *n_s = strtok(rest, " ");
    char *id_s = strtok(NULL, " ");
    unsigned long n = n_s ? strtoul(n_s, NULL, 10) : 0UL;
    unsigned long id = id_s ? strtoul(id_s, NULL, 16) : 0UL;
    if (n == 0UL || n > CAN_BURST_MAX || id_s == NULL || id > 0x1FFFFFFFUL)
    {
        printf("usage: can burst <count 1..%u> <id-hex>\r\n", (unsigned)CAN_BURST_MAX);
        return;
    }
    can_frame_t f = {.id = (uint32_t)id, .ext = (id > 0x7FFUL) ? 1U : 0U, .dlc = 2U};
    uint32_t ok = 0U;
    uint32_t start = HAL_GetTick();
    for (uint32_t i = 0; i < n; i++)
    {
        f.data[0] = (uint8_t)i;
        f.data[1] = (uint8_t)(i >> 8);
        if (can_send(&f) == MCP_TX_OK)
        {
            ok++;
        }
        else
        {
            break; /* no ACK: don't spend n * timeout */
        }
    }
    printf("CAN burst done sent=%lu failed=%lu ms=%lu\r\n", (unsigned long)ok,
           (unsigned long)(n - ok), (unsigned long)(HAL_GetTick() - start));
}

static void handle_can_periodic(char *rest)
{
    char *id_s = strtok(rest, " ");
    if (id_s != NULL && strcmp(id_s, "off") == 0)
    {
        g_periodic.on = 0U;
        printf("CAN periodic off sent=%lu\r\n", (unsigned long)g_periodic.sent);
        return;
    }
    char *ms_s = strtok(NULL, " ");
    char *n_s = strtok(NULL, " ");
    unsigned long id = id_s ? strtoul(id_s, NULL, 16) : 0UL;
    unsigned long ms = ms_s ? strtoul(ms_s, NULL, 10) : 0UL;
    if (id_s == NULL || ms == 0UL || id > 0x1FFFFFFFUL)
    {
        printf("usage: can periodic <id-hex> <period-ms> [count] | can periodic off\r\n");
        return;
    }
    g_periodic.id = (uint32_t)id;
    g_periodic.ext = (id > 0x7FFUL) ? 1U : 0U;
    g_periodic.period_ms = (uint32_t)ms;
    g_periodic.limit = n_s ? (uint32_t)strtoul(n_s, NULL, 10) : 0U;
    g_periodic.sent = 0U;
    g_periodic.next_ms = HAL_GetTick();
    g_periodic.on = 1U;
    printf("CAN periodic on id=0x%lX period=%lums count=%lu\r\n", id, ms,
           (unsigned long)g_periodic.limit);
}

/* Prove the MCU <-> MCP2515 path on its own: loop a frame inside the MCP2515
 * (nothing goes on the bus), then return to normal mode. */
static void handle_can_selftest(void)
{
    if (!g_can.init_ok)
    {
        printf("CAN selftest fail: not initialized\r\n");
        return;
    }
    if (mcp_set_mode(MCP_MODE_LOOPBACK) != 0)
    {
        printf("CAN selftest fail: loopback mode refused\r\n");
        return;
    }
    can_frame_t tx = {.id = 0x5A5U, .dlc = 4U, .data = {0xDE, 0xAD, 0xBE, 0xEF}};
    can_frame_t rx = {0};
    int got = 0;
    if (mcp_send(&tx, CAN_TX_TIMEOUT_MS) == MCP_TX_OK)
    {
        uint32_t start = HAL_GetTick();
        while (!got && (HAL_GetTick() - start) < 20U)
        {
            got = mcp_receive(&rx);
        }
    }
    (void)mcp_set_mode(MCP_MODE_NORMAL);
    if (got && rx.id == tx.id && rx.dlc == tx.dlc && memcmp(rx.data, tx.data, tx.dlc) == 0)
    {
        printf("CAN selftest ok\r\n");
    }
    else
    {
        printf("CAN selftest fail: frame did not loop back\r\n");
    }
}

static void handle_can(char *args)
{
    char *sub = args;
    char *rest = strchr(args, ' ');
    if (rest != NULL)
    {
        *rest++ = '\0';
        while (*rest == ' ')
        {
            rest++;
        }
    }
    else
    {
        rest = args + strlen(args);
    }

    if (strcmp(sub, "init") == 0)
    {
        unsigned long br = (*rest != '\0') ? strtoul(rest, NULL, 10) : g_can.bitrate;
        can_init((uint32_t)br);
        return;
    }
    if (strcmp(sub, "osc") == 0)
    {
        unsigned long mhz = strtoul(rest, NULL, 10);
        if (mhz == 0UL || mhz > 40UL)
        {
            printf("usage: can osc <crystal-MHz>  (8 or 16 on most modules)\r\n");
            return;
        }
        g_can.osc_hz = (uint32_t)(mhz * 1000000UL);
        can_init(g_can.bitrate);
        return;
    }
    if (strcmp(sub, "status") == 0)
    {
        can_print_status();
        return;
    }
    if (!g_can.init_ok)
    {
        printf("CAN not initialized: run \"can init <bitrate>\"\r\n");
        return;
    }
    if (strcmp(sub, "send") == 0 || strcmp(sub, "sendx") == 0)
    {
        handle_can_send(rest, sub[4] == 'x');
    }
    else if (strcmp(sub, "burst") == 0)
    {
        handle_can_burst(rest);
    }
    else if (strcmp(sub, "periodic") == 0)
    {
        handle_can_periodic(rest);
    }
    else if (strcmp(sub, "echo") == 0 || strcmp(sub, "print") == 0)
    {
        uint8_t v;
        if (parse_on_off(rest, &v) != 0)
        {
            printf("usage: can %s on|off\r\n", sub);
            return;
        }
        if (sub[0] == 'e')
        {
            g_can.echo = v;
        }
        else
        {
            g_can.print = v;
        }
        printf("CAN %s %s\r\n", sub, v ? "on" : "off");
    }
    else if (strcmp(sub, "mode") == 0)
    {
        mcp_mode_t m;
        if (strcmp(rest, "normal") == 0)
        {
            m = MCP_MODE_NORMAL;
        }
        else if (strcmp(rest, "listen") == 0)
        {
            m = MCP_MODE_LISTEN;
        }
        else if (strcmp(rest, "loopback") == 0)
        {
            m = MCP_MODE_LOOPBACK;
        }
        else
        {
            printf("usage: can mode normal|listen|loopback\r\n");
            return;
        }
        printf("CAN mode %s %s\r\n", rest, (mcp_set_mode(m) == 0) ? "ok" : "fail");
    }
    else if (strcmp(sub, "selftest") == 0)
    {
        handle_can_selftest();
    }
    else if (strcmp(sub, "clear") == 0)
    {
        g_can.rx = g_can.tx = g_can.tx_fail = g_can.rx_overflow = 0U;
        printf("CAN counters cleared\r\n");
    }
    else if (strcmp(sub, "regs") == 0)
    {
        printf("CAN regs: canstat=%02X canctrl=%02X cnf1=%02X cnf2=%02X cnf3=%02X "
               "caninte=%02X canintf=%02X eflg=%02X tec=%u rec=%u\r\n",
               mcp_read_reg(MCP_CANSTAT), mcp_read_reg(MCP_CANCTRL), mcp_read_reg(MCP_CNF1),
               mcp_read_reg(MCP_CNF2), mcp_read_reg(MCP_CNF3), mcp_read_reg(MCP_CANINTE),
               mcp_read_reg(MCP_CANINTF), mcp_read_reg(MCP_EFLG), mcp_read_reg(MCP_TEC),
               mcp_read_reg(MCP_REC));
    }
    else
    {
        printf("unknown can command: %s (try help)\r\n", sub);
    }
}

/* ---------------------------------------------------------------- console */

static void print_prompt(void)
{
    printf("> ");
    (void)fflush(stdout);
}

static void process_command(char *cmd)
{
    if (strcmp(cmd, "help") == 0)
    {
        print_help();
    }
    else if (strcmp(cmd, "status") == 0)
    {
        can_print_status();
    }
    else if (strcmp(cmd, "info") == 0)
    {
        print_info();
    }
    else if (strcmp(cmd, "ain") == 0)
    {
        const ain_state_t *a = ain_read_now();
        printf("AIN mv=%lu raw=%lu filt_mv=%lu vdda_mv=%lu samples=%lu\r\n", (unsigned long)a->mv,
               (unsigned long)a->raw, (unsigned long)a->filt_mv, (unsigned long)a->vdda_mv,
               (unsigned long)a->samples);
    }
    else if (strcmp(cmd, "wdt stall") == 0)
    {
        /* Lifecycle test hook: stop kicking the watchdog; it resets us in ~2 s and
         * the next boot reports reset=iwdg. */
        printf("WDT stall: waiting for the watchdog reset\r\n");
        while (1)
        {
        }
    }
    else if (strncmp(cmd, "can ", 4) == 0)
    {
        handle_can(cmd + 4);
    }
    else if (strcmp(cmd, "reset") == 0 || strcmp(cmd, "reboot") == 0)
    {
        printf("RESET: rebooting via NVIC_SystemReset()\r\n");
        while ((USART1->SR & USART_SR_TC) == 0U)
        {
        }
        NVIC_SystemReset();
    }
    else
    {
        printf("unknown command: %s (try help)\r\n", cmd);
    }
}

static void print_help(void)
{
    printf("commands:\r\n");
    printf("  info                      firmware, version, uptime, reset cause\r\n");
    printf("  ain                       analog input PA1: mV, raw, filtered, VDDA\r\n");
    printf("  status | can status       counters, mode, error state\r\n");
    printf("  can init [bitrate]        reset + configure the MCP2515 (default 500000)\r\n");
    printf("  can osc <MHz>             module crystal (8 or 16), then re-init\r\n");
    printf("  can mode normal|listen|loopback\r\n");
    printf("  can send <id> [b0..b7]    hex; id > 7FF sends an extended frame\r\n");
    printf("  can sendx <id> [b0..b7]   force an extended id\r\n");
    printf("  can burst <n> <id>        n back-to-back frames, data = 16-bit counter\r\n");
    printf("  can periodic <id> <ms> [count] | can periodic off\r\n");
    printf("  can echo on|off           reply to every frame with id+1, same data\r\n");
    printf("  can print on|off          print received frames (default on)\r\n");
    printf("  can selftest              loop a frame inside the MCP2515\r\n");
    printf("  can clear                 zero the counters\r\n");
    printf("  can regs                  dump MCP2515 registers\r\n");
    printf("  reset                     reboot the MCU\r\n");
    printf("  wdt stall                 hang until the watchdog resets the MCU\r\n");
}

/* ---------------------------------------------------------------- lifecycle */

/* Why we booted, read once from RCC_CSR before anything else, then cleared so
 * the next reset reports only its own cause. A power-on also sets the pin and
 * brown-out flags, and a software reset also sets the pin flag, hence the order. */
static void reset_cause_capture(void)
{
    uint32_t csr = RCC->CSR;
    if (csr & RCC_CSR_IWDGRSTF)
    {
        g_reset_cause = "iwdg";
    }
    else if (csr & RCC_CSR_WWDGRSTF)
    {
        g_reset_cause = "wwdg";
    }
    else if (csr & RCC_CSR_LPWRRSTF)
    {
        g_reset_cause = "lowpower";
    }
    else if (csr & RCC_CSR_SFTRSTF)
    {
        g_reset_cause = "software";
    }
    else if (csr & RCC_CSR_PORRSTF)
    {
        g_reset_cause = "power-on";
    }
    else if (csr & RCC_CSR_BORRSTF)
    {
        g_reset_cause = "brownout";
    }
    else if (csr & RCC_CSR_PINRSTF)
    {
        g_reset_cause = "pin";
    }
    RCC->CSR |= RCC_CSR_RMVF;
}

static void iwdg_start(void)
{
    IWDG->KR = 0xCCCCU; /* start (LSI comes up on its own) */
    IWDG->KR = 0x5555U; /* unlock PR/RLR */
    IWDG->PR = IWDG_PR_PR_2; /* /64 */
    IWDG->RLR = IWDG_RELOAD;
    while (IWDG->SR != 0U)
    {
    }
    IWDG->KR = 0xAAAAU;
}

static void iwdg_kick(void)
{
    IWDG->KR = 0xAAAAU;
}

static void print_info(void)
{
    printf("INFO fw=%s version=%s build=\"%s %s\" uptime_ms=%lu reset=%s\r\n", FW_NAME,
           FW_VERSION, __DATE__, __TIME__, (unsigned long)HAL_GetTick(), g_reset_cause);
}

void USART1_IRQHandler(void)
{
    while ((USART1->SR & (USART_SR_RXNE | USART_SR_ORE)) != 0U)
    {
        uint8_t byte = (uint8_t)(USART1->DR & 0xFFU);
        uint16_t next = (uint16_t)((rx_ring_head + 1U) % RX_RING_SIZE);
        if (next != rx_ring_tail)
        {
            rx_ring[rx_ring_head] = byte;
            rx_ring_head = next;
        }
    }
}

static int usart1_read_byte_nonblocking(uint8_t *byte)
{
    if (rx_ring_tail == rx_ring_head)
    {
        return 0;
    }
    *byte = rx_ring[rx_ring_tail];
    rx_ring_tail = (uint16_t)((rx_ring_tail + 1U) % RX_RING_SIZE);
    return 1;
}

static void uart_process_rx_byte(uint8_t byte)
{
    if (byte == '\r' || byte == '\n')
    {
        if (byte == '\n' && rx_last_was_cr)
        {
            rx_last_was_cr = 0U;
            return;
        }
        rx_last_was_cr = (byte == '\r') ? 1U : 0U;
        printf("\r\n");
        if (cmd_index > 0U)
        {
            cmd_buffer[cmd_index] = '\0';
            cmd_ready = 1;
        }
        else
        {
            print_prompt();
        }
    }
    else if (byte == '\b' || byte == 0x7FU)
    {
        if (cmd_index > 0U)
        {
            cmd_index--;
            cmd_buffer[cmd_index] = '\0';
            printf("\b \b");
        }
    }
    else if (byte >= 32U && byte <= 126U)
    {
        rx_last_was_cr = 0U;
        if (cmd_index < (CMD_BUF_SIZE - 1U))
        {
            cmd_buffer[cmd_index++] = (char)byte;
            printf("%c", (char)byte);
        }
    }
}

/* ---------------------------------------------------------------- peripherals */

static void USART1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    GPIOA->MODER &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->MODER |= (2U << (9U * 2U)) | (2U << (10U * 2U));
    GPIOA->OTYPER &= ~(1U << 9U);
    GPIOA->OSPEEDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->OSPEEDR |= (2U << (9U * 2U)) | (2U << (10U * 2U));
    GPIOA->PUPDR &= ~((3U << (9U * 2U)) | (3U << (10U * 2U)));
    GPIOA->PUPDR |= (1U << (10U * 2U));
    GPIOA->AFR[1] &= ~((0xFU << ((9U - 8U) * 4U)) | (0xFU << ((10U - 8U) * 4U)));
    GPIOA->AFR[1] |= (7U << ((9U - 8U) * 4U)) | (7U << ((10U - 8U) * 4U));

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 0x008BU; /* 16 MHz / 115200 */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    USART1->CR1 |= USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

/* SPI1 master, mode 0, 16 MHz / 16 = 1 MHz: slow enough for the TXS0108E's
 * auto-direction channels over jumper wires, plenty for the MCP2515. */
static void SPI1_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN | RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_MEDIUM;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &g);

    CS_PORT->BSRR = CS_PIN; /* deselect before it becomes an output */
    g.Pin = CS_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Alternate = 0;
    HAL_GPIO_Init(CS_PORT, &g);

    /* INT: input with pull-up, falling edge wakes the core from __WFI. */
    g.Pin = INT_PIN;
    g.Mode = GPIO_MODE_IT_FALLING;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(INT_PORT, &g);
    NVIC_SetPriority(EXTI9_5_IRQn, 2);
    NVIC_EnableIRQ(EXTI9_5_IRQn);

    SPI1->CR1 = 0U;
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (3U << SPI_CR1_BR_Pos);
    SPI1->CR1 |= SPI_CR1_SPE;
}

/* Only here to wake the main loop; can_service() reads the INT level. */
void EXTI9_5_IRQHandler(void)
{
    EXTI->PR = INT_PIN;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_NONE;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 |
                    RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0);
}
