/*
 * progemu.c - the hardware side of the chip emulators (progemu.h): pins, interrupts, timing
 * statistics and the `prog_emu` console command. The chips themselves are in progemu_<proto>.c.
 *
 * Two-wire protocols (PIC, C2, TPI, PDI):
 *   clock  PB14 (LA13), input with pull-up, EXTI line 14 on both edges
 *   data   PB12 (LA14), input; push-pull output only while the emulated chip drives it
 *   reset  PB0  (LA5), input with pull-up, EXTI line 0 on both edges (PIC MCLR, TPI RESET)
 * Every clock edge runs the chip model inside EXTI15_10_IRQHandler: the cycle count and both port
 * levels are captured first, the model runs, then PB12 follows what the chip drives. Nothing in
 * the handler prints; it only counts.
 *
 * UPDI: USART3 in single-wire half-duplex mode on PC10 (LA7), 8E2, open drain. While the chip
 * waits for a frame the receiver is off and EXTI line 10 times the SYNC byte (0x55) edge to edge,
 * as a real UPDI does: the baud rate is set from it and the receiver then takes the rest of the
 * frame. A long low (over 1 ms) or a framing error with 0x00 is a BREAK. Replies are sent from a
 * ring by the USART3 interrupt, two bit times after the request ends.
 */
#include "progemu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dwt.h"
#include "rs485mock.h"
#include "spi2.h"
#include "stm32f4xx_hal.h"
#include "uart3.h"

uint32_t pe_now;
uint32_t pe_cyc_us = 16U;
uint32_t pe_pool[PE_POOL_BYTES / 4U];

/* ---------------------------------------------------------------- the chips */

static const pe_chip_t *const k_chips[] = {
    &pe_pic, &pe_pic8, &pe_pic18, &pe_c2,
};
#define NCHIPS (sizeof k_chips / sizeof k_chips[0])

static const pe_chip_t *volatile s_chip;   /* NULL: stopped */

/* ---------------------------------------------------------------- pins */

#define CLK_LINE   (1UL << 14)    /* PB14 */
#define RST_LINE   (1UL << 0)     /* PB0 */
#define UPDI_LINE  (1UL << 10)    /* PC10 */

static volatile int s_drv = -1;   /* what PB12 is driven to: 0, 1, or -1 released */

static inline void drive_data(int d)
{
    if (d == s_drv)
    {
        return;
    }
    s_drv = d;
    if (d < 0)
    {
        GPIOB->MODER &= ~(3UL << 24);
    }
    else
    {
        GPIOB->BSRR = d ? (1UL << 12) : (1UL << (12 + 16));
        GPIOB->MODER = (GPIOB->MODER & ~(3UL << 24)) | (1UL << 24);
    }
}

/* ---------------------------------------------------------------- statistics */

#define NBIN 9U
static uint32_t s_thr[NBIN - 1U];      /* bin bounds in cycles: 0.5 1 2 4 8 16 64 256 us */
static const char *const k_thr_us = "0.5,1,2,4,8,16,64,256,inf";

typedef struct {
    uint32_t min, max, n, sum;         /* sum and n: intervals under 100 us */
    uint32_t bin[NBIN];
} gap_t;

static volatile uint32_t s_edges, s_rise, s_fall, s_missed, s_late, s_nrst_edges;
static gap_t s_hi, s_lo;               /* high time (rise to fall) and low time (fall to rise) */
static uint32_t s_last_t, s_burst_cyc;
static int s_prev_clk, s_first;

static void stats_reset(void)
{
    s_edges = s_rise = s_fall = s_missed = s_late = s_nrst_edges = 0U;
    memset(&s_hi, 0, sizeof s_hi);
    memset(&s_lo, 0, sizeof s_lo);
    s_hi.min = s_lo.min = 0xFFFFFFFFU;
    s_first = 1;
}

static inline void gap_add(gap_t *g, uint32_t dt)
{
    if (dt < g->min) g->min = dt;
    if (dt > g->max) g->max = dt;
    unsigned b = 0U;
    while (b < NBIN - 1U && dt >= s_thr[b]) b++;
    g->bin[b]++;
    if (dt < s_burst_cyc)
    {
        g->sum += dt;
        g->n++;
    }
}

/* ---------------------------------------------------------------- UPDI (USART3, PC10) */

static uint8_t s_updi_on;
enum { U_HUNT, U_RX, U_TX };
static volatile uint8_t s_umode;
#define TXR 512U
static uint8_t s_txr[TXR];
static volatile uint16_t s_txh, s_txt;
static uint32_t s_bit_cyc;                      /* cycles per bit at the current baud */
static uint32_t s_ft[5];                        /* the falling edges of the SYNC */
static uint8_t s_nf;
static uint32_t s_fall_t;
static uint32_t s_sync_n, s_bad_sync, s_breaks, s_u_fe, s_u_pe, s_u_ne, s_u_ore, s_u_rx, s_u_tx,
                s_tx_over, s_sync_last, s_sync_min, s_sync_max, s_baud, s_brr_last;

void pe_updi_tx(const uint8_t *b, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
    {
        uint16_t next = (uint16_t)((s_txh + 1U) % TXR);
        if (next == s_txt)
        {
            s_tx_over++;
            return;
        }
        s_txr[s_txh] = b[i];
        s_txh = next;
    }
}

static void updi_hunt(void)
{
    USART3->CR1 &= ~USART_CR1_RE;
    s_nf = 0U;
    EXTI->PR = UPDI_LINE;
    EXTI->IMR |= UPDI_LINE;
    s_umode = U_HUNT;
}

static void updi_rx_on(void)
{
    EXTI->IMR &= ~UPDI_LINE;
    if (USART3->SR & USART_SR_RXNE)
    {
        (void)USART3->DR;
    }
    USART3->CR1 |= USART_CR1_RE;
    s_umode = U_RX;
}

static void updi_next(uint32_t t_rx)
{
    const pe_chip_t *c = s_chip;
    if (s_txh != s_txt)
    {
        /* answer two bit times after the request ended (we are mid first stop bit now): the pod's
           receiver is done with its own second stop bit by then */
        while (DWT->CYCCNT - t_rx < 2U * s_bit_cyc)
        {
        }
        USART3->CR1 = (USART3->CR1 & ~USART_CR1_RE) | USART_CR1_TXEIE;
        s_umode = U_TX;
    }
    else if (c->want_sync())
    {
        updi_hunt();
    }
}

/* An edge of the line while the chip waits for a frame: SYNC timing and BREAK. */
static void updi_edge(uint32_t t, int lvl)
{
    const pe_chip_t *c = s_chip;
    if (!lvl)
    {
        if (s_nf >= 2U)
        {
            uint32_t ref = s_ft[1] - s_ft[0], d = t - s_ft[s_nf - 1U];
            if (d < ref - ref / 4U || d > ref + ref / 4U)
            {
                s_nf = 0U;
            }
        }
        if (s_nf >= 5U)
        {
            s_nf = 0U;
        }
        s_ft[s_nf++] = t;
        s_fall_t = t;
        return;
    }
    uint32_t low = t - s_fall_t;
    if (low > 1000U * pe_cyc_us)
    {
        s_breaks++;
        s_nf = 0U;
        pe_now = t;
        c->brk();
        return;
    }
    if (s_nf != 5U)
    {
        return;
    }
    /* the SYNC: falling edges at 0, 2, 4, 6 and 8 bit times, the last low lasting two (D7 and the
       even parity bit), then the stop bits */
    uint32_t b = (s_ft[4] - s_ft[0]) / 8U;
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    uint32_t brr = (uint32_t)(((uint64_t)b * pclk + SystemCoreClock / 2U) / SystemCoreClock);
    if (b == 0U || low < b + b / 2U || low > 2U * b + b / 2U || brr < 16U || brr > 0xFFFFU)
    {
        s_bad_sync++;
        s_nf = 0U;
        return;
    }
    USART3->BRR = brr;
    s_brr_last = brr;
    s_bit_cyc = b;
    s_baud = pclk / brr;
    s_sync_n++;
    s_sync_last = b;
    if (b < s_sync_min) s_sync_min = b;
    if (b > s_sync_max) s_sync_max = b;
    s_nf = 0U;
    pe_now = t;
    c->sync();
    updi_rx_on();
}

int progemu_usart3_irq(void)
{
    if (!s_updi_on)
    {
        return 0;
    }
    uint32_t t = DWT->CYCCNT;
    const pe_chip_t *c = s_chip;
    uint32_t sr = USART3->SR, cr1 = USART3->CR1;
    if ((cr1 & USART_CR1_RE) && (sr & (USART_SR_RXNE | USART_SR_ORE)))
    {
        uint8_t b = (uint8_t)(USART3->DR & 0xFFU);
        if (sr & USART_SR_ORE) s_u_ore++;
        if (sr & USART_SR_NE) s_u_ne++;
        if (sr & USART_SR_PE) s_u_pe++;
        if (sr & USART_SR_FE) s_u_fe++;
        if (sr & USART_SR_RXNE)
        {
            s_u_rx++;
            pe_now = t;
            if ((sr & USART_SR_FE) && b == 0U)
            {
                s_breaks++;
                c->brk();
                updi_hunt();
            }
            else if (sr & (USART_SR_FE | USART_SR_PE | USART_SR_NE))
            {
                /* a bad frame is not taken; the pod times out and breaks */
            }
            else
            {
                c->rx(b);
                updi_next(t);
            }
        }
        sr = USART3->SR;
        cr1 = USART3->CR1;
    }
    if ((cr1 & USART_CR1_TXEIE) && (sr & USART_SR_TXE))
    {
        if (s_txt != s_txh)
        {
            USART3->DR = s_txr[s_txt];
            s_txt = (uint16_t)((s_txt + 1U) % TXR);
            s_u_tx++;
        }
        else
        {
            USART3->CR1 = (cr1 & ~USART_CR1_TXEIE) | USART_CR1_TCIE;
        }
    }
    else if ((cr1 & USART_CR1_TCIE) && (sr & USART_SR_TC))
    {
        USART3->CR1 = cr1 & ~USART_CR1_TCIE;
        USART3->SR = ~USART_SR_TC;
        if (c->want_sync())
        {
            updi_hunt();
        }
        else
        {
            updi_rx_on();
        }
    }
    return 1;
}

static void updi_start(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    g.Pin = GPIO_PIN_10;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &g);
    memset((void *)&s_ft, 0, sizeof s_ft);
    s_txh = s_txt = 0U;
    s_sync_n = s_bad_sync = s_breaks = s_u_fe = s_u_pe = s_u_ne = s_u_ore = s_u_rx = s_u_tx = 0U;
    s_tx_over = s_sync_last = s_sync_max = s_baud = s_brr_last = 0U;
    s_sync_min = 0xFFFFFFFFU;
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    USART3->CR1 = 0U;
    USART3->CR2 = USART_CR2_STOP_1;                 /* 2 stop bits */
    USART3->CR3 = USART_CR3_HDSEL;                  /* single wire: TX pin is the line */
    USART3->BRR = (pclk + 57600U) / 115200U;
    s_bit_cyc = SystemCoreClock / 115200U;
    /* 8E2: the 9-bit word is 8 data bits and the parity bit */
    USART3->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_UE;
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~(0xFUL << 8)) | (2UL << 8);   /* EXTI10: port C */
    EXTI->RTSR |= UPDI_LINE;
    EXTI->FTSR |= UPDI_LINE;
    NVIC_SetPriority(USART3_IRQn, 0);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(USART3_IRQn);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    s_updi_on = 1U;
    updi_hunt();
}

static void updi_stop(void)
{
    NVIC_DisableIRQ(USART3_IRQn);
    EXTI->IMR &= ~UPDI_LINE;
    EXTI->PR = UPDI_LINE;
    USART3->CR1 = 0U;
    s_updi_on = 0U;
    uart3_close();                                  /* PC10 and PC11 back to inputs */
}

/* ---------------------------------------------------------------- two-wire */

void EXTI15_10_IRQHandler(void)
{
    uint32_t t = DWT->CYCCNT;
    uint32_t idr = GPIOB->IDR;
    uint32_t pr = EXTI->PR & (CLK_LINE | UPDI_LINE);
    EXTI->PR = pr;
    const pe_chip_t *c = s_chip;
    if (c == NULL)
    {
        return;
    }
    if (pr & CLK_LINE)
    {
        int clk = (int)((idr >> 14) & 1U);
        if (clk == s_prev_clk)
        {
            s_missed++;        /* both edges came before we ran: a pulse we could not follow */
        }
        else
        {
            s_prev_clk = clk;
            pe_now = t;
            c->edge(clk, s_drv >= 0 ? -1 : (int)((idr >> 12) & 1U));
            drive_data(c->drive());
            s_edges++;
            if (clk) s_rise++; else s_fall++;
            if (s_first)
            {
                s_first = 0;
            }
            else
            {
                gap_add(clk ? &s_lo : &s_hi, t - s_last_t);
            }
            s_last_t = t;
        }
        if (EXTI->PR & CLK_LINE)
        {
            s_late++;          /* the next edge came while this one was handled */
        }
    }
    if (pr & UPDI_LINE)
    {
        if (s_umode == U_HUNT)
        {
            s_edges++;
            updi_edge(t, (int)((GPIOC->IDR >> 10) & 1U));
        }
    }
}

void EXTI0_IRQHandler(void)
{
    uint32_t t = DWT->CYCCNT;
    int low = (int)((GPIOB->IDR & 1U) == 0U);
    EXTI->PR = RST_LINE;
    const pe_chip_t *c = s_chip;
    if (c != NULL && c->nrst != NULL)
    {
        pe_now = t;
        c->nrst(low);
        drive_data(c->drive());
        s_nrst_edges++;
    }
}

static void wires_start(const pe_chip_t *c)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    g.Pin = GPIO_PIN_14;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_12;
    g.Pull = GPIO_NOPULL;                         /* the pod holds a 10k pull-up on data */
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);
    GPIOB->OSPEEDR |= (3UL << 24);
    s_drv = -1;
    SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~(0xFUL << 8)) | (1UL << 8);    /* EXTI14: port B */
    EXTI->RTSR |= CLK_LINE;
    EXTI->FTSR |= CLK_LINE;
    s_prev_clk = (int)((GPIOB->IDR >> 14) & 1U);
    EXTI->PR = CLK_LINE;
    EXTI->IMR |= CLK_LINE;
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    if (c->reset)
    {
        g.Pin = GPIO_PIN_0;
        g.Pull = GPIO_PULLUP;                     /* the pod pulls it low, open drain */
        HAL_GPIO_Init(GPIOB, &g);
        SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~0xFUL) | 1UL;               /* EXTI0: port B */
        EXTI->RTSR |= RST_LINE;
        EXTI->FTSR |= RST_LINE;
        NVIC_SetPriority(EXTI0_IRQn, 0);
        NVIC_EnableIRQ(EXTI0_IRQn);
        if ((GPIOB->IDR & 1U) == 0U)
        {
            pe_now = DWT->CYCCNT;
            c->nrst(1);
        }
        EXTI->PR = RST_LINE;
        EXTI->IMR |= RST_LINE;
    }
}

static void wires_stop(void)
{
    EXTI->IMR &= ~(CLK_LINE | RST_LINE);
    EXTI->PR = CLK_LINE | RST_LINE;
    drive_data(-1);
    /* the node's own pins again: SPI2 (PB12 CS high, PB14 MISO) and the alarm output PB0 (its
       level was kept in ODR all along) */
    spi2_init();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_0;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
}

/* ---------------------------------------------------------------- start / stop */

int progemu_active(void)
{
    return s_chip != NULL;
}

static void pe_stop(void)
{
    const pe_chip_t *c = s_chip;
    if (c == NULL)
    {
        return;
    }
    if (c->uart)
    {
        updi_stop();
    }
    else
    {
        wires_stop();
    }
    s_chip = NULL;
}

static void pe_start(const pe_chip_t *c)
{
    pe_stop();
    rs485_stop();
    uart3_close();
    dwt_init();
    pe_cyc_us = SystemCoreClock / 1000000U;
    for (unsigned i = 0; i < NBIN - 1U; i++)
    {
        static const uint16_t q[NBIN - 1U] = { 5, 10, 20, 40, 80, 160, 640, 2560 };   /* 0.1 us */
        s_thr[i] = (uint32_t)(((uint64_t)q[i] * pe_cyc_us) / 10U);
    }
    s_burst_cyc = 100U * pe_cyc_us;
    stats_reset();
    c->init();
    s_chip = c;
    if (c->uart)
    {
        updi_start();
    }
    else
    {
        wires_start(c);
    }
}

/* ---------------------------------------------------------------- stat, dump */

static uint32_t crc32_update(uint32_t crc, const uint8_t *b, uint32_t n)
{
    static const uint32_t t[16] = {
        0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC, 0x76DC4190, 0x6B6B51F4, 0x4DB26158,
        0x5005713C, 0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C, 0x9B64C2B0, 0x86D3D2D4,
        0xA00AE278, 0xBDBDF21C };
    for (uint32_t i = 0; i < n; i++)
    {
        crc ^= b[i];
        crc = (crc >> 4) ^ t[crc & 15U];
        crc = (crc >> 4) ^ t[crc & 15U];
    }
    return crc;
}

static uint32_t mem_crc(const pe_mem_t *m)
{
    uint8_t buf[64];
    uint32_t crc = 0xFFFFFFFFU;
    for (uint32_t a = 0; a < m->size; a += sizeof buf)
    {
        uint32_t n = m->size - a < sizeof buf ? m->size - a : (uint32_t)sizeof buf;
        m->read(a, buf, n);
        crc = crc32_update(crc, buf, n);
    }
    return ~crc;
}

static unsigned long ns_of(uint32_t cyc)
{
    return (unsigned long)(((uint64_t)cyc * 1000U) / pe_cyc_us);
}

static void print_gap(const char *name, const gap_t *g)
{
    if (g->min == 0xFFFFFFFFU)
    {
        printf("PROG_EMU %s_ns n=0\r\n", name);
        return;
    }
    printf("PROG_EMU %s_ns min=%lu max=%lu avg_under_100us=%lu n_under_100us=%lu\r\n", name,
           ns_of(g->min), ns_of(g->max), g->n ? ns_of(g->sum / g->n) : 0UL, (unsigned long)g->n);
}

static void print_bins(const char *name, const gap_t *g)
{
    printf("PROG_EMU %s_hist_us bounds=%s counts=", name, k_thr_us);
    for (unsigned i = 0; i < NBIN; i++)
    {
        printf("%s%lu", i ? "," : "", (unsigned long)g->bin[i]);
    }
    printf("\r\n");
}

static void print_stat(void)
{
    const pe_chip_t *c = s_chip;
    if (c == NULL)
    {
        printf("PROG_EMU stat running=0\r\n");
        return;
    }
    const pe_counts_t *k = c->counts();
    printf("PROG_EMU stat proto=%s running=1 edges=%lu rise=%lu fall=%lu nrst_edges=%lu frames=%lu "
           "errors=%lu timing=%lu missed=%lu late=%lu core_mhz=%lu\r\n",
           c->name, (unsigned long)s_edges, (unsigned long)s_rise, (unsigned long)s_fall,
           (unsigned long)s_nrst_edges, (unsigned long)k->frames, (unsigned long)k->errors,
           (unsigned long)k->timing, (unsigned long)s_missed, (unsigned long)s_late,
           (unsigned long)pe_cyc_us);
    if (!c->uart)
    {
        print_gap("high", &s_hi);
        print_gap("low", &s_lo);
        print_bins("high", &s_hi);
        print_bins("low", &s_lo);
    }
    else
    {
        printf("PROG_EMU updi syncs=%lu bad_sync=%lu breaks=%lu framing=%lu parity=%lu noise=%lu "
               "overrun=%lu rx=%lu tx=%lu tx_overflow=%lu baud=%lu brr=%lu sync_bit_ns=%lu "
               "min=%lu max=%lu\r\n",
               (unsigned long)s_sync_n, (unsigned long)s_bad_sync, (unsigned long)s_breaks,
               (unsigned long)s_u_fe, (unsigned long)s_u_pe, (unsigned long)s_u_ne,
               (unsigned long)s_u_ore, (unsigned long)s_u_rx, (unsigned long)s_u_tx,
               (unsigned long)s_tx_over, (unsigned long)s_baud, (unsigned long)s_brr_last,
               ns_of(s_sync_last), s_sync_n ? ns_of(s_sync_min) : 0UL, ns_of(s_sync_max));
    }
    if (c->stat != NULL)
    {
        c->stat();
    }
    printf("PROG_EMU crc");
    for (unsigned i = 0; i < c->nmem; i++)
    {
        printf(" %s=%08lx", c->mem[i].name, (unsigned long)mem_crc(&c->mem[i]));
    }
    printf("\r\n");
    printf("PROG_EMU size");
    for (unsigned i = 0; i < c->nmem; i++)
    {
        printf(" %s=%lu", c->mem[i].name, (unsigned long)c->mem[i].size);
    }
    printf("\r\n");
}

static void print_dump(const pe_chip_t *c, const char *mem, uint32_t addr, uint32_t len)
{
    const pe_mem_t *m = NULL;
    for (unsigned i = 0; i < c->nmem; i++)
    {
        if (strcmp(c->mem[i].name, mem) == 0)
        {
            m = &c->mem[i];
        }
    }
    if (m == NULL || addr > m->size || len > m->size - addr || len > 4096U)
    {
        printf("PROG_EMU err usage: prog_emu dump <mem> <addr> <len up to 4096> (memories:");
        for (unsigned i = 0; i < c->nmem; i++)
        {
            printf(" %s=%lu", c->mem[i].name, (unsigned long)c->mem[i].size);
        }
        printf(")\r\n");
        return;
    }
    printf("PROG_EMU dump mem=%s addr=0x%lx len=%lu\r\n", m->name, (unsigned long)addr,
           (unsigned long)len);
    uint8_t buf[32];
    for (uint32_t a = 0; a < len; a += sizeof buf)
    {
        uint32_t n = len - a < sizeof buf ? len - a : (uint32_t)sizeof buf;
        m->read(addr + a, buf, n);
        printf("%04lx:", (unsigned long)(addr + a));
        for (uint32_t i = 0; i < n; i++)
        {
            printf("%02x", buf[i]);
        }
        printf("\r\n");
    }
}

static void clear_counters(void)
{
    stats_reset();
    s_sync_n = s_bad_sync = s_breaks = s_u_fe = s_u_pe = s_u_ne = s_u_ore = s_u_rx = s_u_tx = 0U;
    s_sync_last = s_sync_max = 0U;
    s_sync_min = 0xFFFFFFFFU;
}

/* ---------------------------------------------------------------- console */

void progemu_help(void)
{
    printf("  prog_emu <updi|updi2|pic|pic8|pic18|c2|tpi|pdi>   the NUCLEO answers as that chip (see README)\r\n");
    printf("  prog_emu stop | stat | clear | lock | dump <mem> <addr> <len> | list\r\n");
}

int progemu_command(char *cmd)
{
    if (strncmp(cmd, "prog_emu", 8) != 0 || (cmd[8] != '\0' && cmd[8] != ' '))
    {
        return 0;
    }
    char *sub = strtok(cmd + 8, " ");
    if (sub == NULL || strcmp(sub, "stat") == 0)
    {
        print_stat();
        return 1;
    }
    if (strcmp(sub, "list") == 0)
    {
        for (unsigned i = 0; i < NCHIPS; i++)
        {
            printf("PROG_EMU chip %s: %s\r\n", k_chips[i]->name, k_chips[i]->what);
        }
        return 1;
    }
    if (strcmp(sub, "stop") == 0)
    {
        pe_stop();
        printf("PROG_EMU ok stopped\r\n");
        return 1;
    }
    if (strcmp(sub, "clear") == 0)
    {
        if (s_chip == NULL)
        {
            printf("PROG_EMU err not running\r\n");
        }
        else
        {
            clear_counters();
            printf("PROG_EMU ok cleared\r\n");
        }
        return 1;
    }
    if (strcmp(sub, "lock") == 0)
    {
        if (s_chip == NULL)
        {
            printf("PROG_EMU err not running\r\n");
        }
        else
        {
            __disable_irq();
            s_chip->lock();
            __enable_irq();
            printf("PROG_EMU ok locked proto=%s\r\n", s_chip->name);
        }
        return 1;
    }
    if (strcmp(sub, "dump") == 0)
    {
        char *mem = strtok(NULL, " ");
        char *a = strtok(NULL, " ");
        char *l = strtok(NULL, " ");
        if (s_chip == NULL || mem == NULL || a == NULL || l == NULL)
        {
            printf("PROG_EMU err usage: prog_emu dump <mem> <addr> <len> (running emulator needed)\r\n");
            return 1;
        }
        print_dump(s_chip, mem, (uint32_t)strtoul(a, NULL, 0), (uint32_t)strtoul(l, NULL, 0));
        return 1;
    }
    for (unsigned i = 0; i < NCHIPS; i++)
    {
        if (strcmp(sub, k_chips[i]->name) == 0)
        {
            pe_start(k_chips[i]);
            printf("PROG_EMU ok proto=%s chip=\"%s\"\r\n", k_chips[i]->name, k_chips[i]->what);
            return 1;
        }
    }
    printf("PROG_EMU err usage: prog_emu <proto>|stop|stat|clear|lock|dump|list (protos:");
    for (unsigned i = 0; i < NCHIPS; i++)
    {
        printf(" %s", k_chips[i]->name);
    }
    printf(")\r\n");
    return 1;
}
