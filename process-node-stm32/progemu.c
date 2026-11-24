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
uint32_t pe_break_frame_us = 6667U;
uint32_t pe_pool[PE_POOL_BYTES / 4U];

/* ---------------------------------------------------------------- core clock */

/* The node runs from the 16 MHz HSI. The pod's bit-bang moves a pin edge every 2 us or so, which a
   16 MHz core (32 cycles an edge) cannot follow. While an emulator runs the core is switched to
   128 MHz from the PLL (HSI / 8 * 128 / 2). APB1 and APB2 are divided by 8, so every peripheral
   bus stays at exactly 16 MHz: USART1's baud rate, I2C, SPI, the ADC and USART3 need no change.
   Only the SysTick (HAL_InitTick does it), the cycle-count users (they read SystemCoreClock) and
   TIM6 (its clock is 2 x PCLK1 while APB1 is divided: ARR is doubled) follow. `prog_emu stop`
   puts the 16 MHz clock back. */
static uint8_t s_fast;

static void tim6_scale(int up)
{
    if (TIM6->CR1 & TIM_CR1_CEN)
    {
        TIM6->ARR = up ? (TIM6->ARR + 1U) * 2U - 1U : (TIM6->ARR + 1U) / 2U - 1U;
    }
}

static void clock_fast(void)
{
    if (s_fast)
    {
        return;
    }
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef k = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    o.PLL.PLLM = 8;                     /* 2 MHz */
    o.PLL.PLLN = 128;                   /* 256 MHz VCO */
    o.PLL.PLLP = RCC_PLLP_DIV2;         /* 128 MHz */
    o.PLL.PLLQ = 4;
    o.PLL.PLLR = 2;
    if (HAL_RCC_OscConfig(&o) != HAL_OK)
    {
        return;
    }
    k.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    k.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    k.AHBCLKDivider = RCC_SYSCLK_DIV1;
    k.APB1CLKDivider = RCC_HCLK_DIV8;
    k.APB2CLKDivider = RCC_HCLK_DIV8;
    if (HAL_RCC_ClockConfig(&k, FLASH_LATENCY_4) != HAL_OK)
    {
        return;
    }
    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;
    tim6_scale(1);
    s_fast = 1U;
}

static void clock_slow(void)
{
    if (!s_fast)
    {
        return;
    }
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef k = {0};
    tim6_scale(0);
    k.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    k.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    k.AHBCLKDivider = RCC_SYSCLK_DIV1;
    k.APB1CLKDivider = RCC_HCLK_DIV1;
    k.APB2CLKDivider = RCC_HCLK_DIV1;
    (void)HAL_RCC_ClockConfig(&k, FLASH_LATENCY_0);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    o.HSIState = RCC_HSI_ON;
    o.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    o.PLL.PLLState = RCC_PLL_OFF;
    (void)HAL_RCC_OscConfig(&o);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
    s_fast = 0U;
}

/* ---------------------------------------------------------------- the chips */

static const pe_chip_t *const k_chips[] = {
    &pe_pic, &pe_pic8, &pe_pic18, &pe_pic18q10, &pe_c2, &pe_c2early, &pe_tpi, &pe_tpi20, &pe_tpi40, &pe_tpi104, &pe_pdi, &pe_updi, &pe_updi2, &pe_updimega,
};
#define NCHIPS (sizeof k_chips / sizeof k_chips[0])

static const pe_chip_t *volatile s_chip;   /* NULL: stopped */

/* ---------------------------------------------------------------- pins */

#define CLK_LINE   (1UL << 14)    /* PB14 */
#define RST_LINE   (1UL << 0)     /* PB0 */
#define DATA_LINE  (1UL << 12)    /* PB12, PDI: the pod forcing data high arms the enable timing */
#define UPDI_LINE  (1UL << 10)    /* PC10 */

static uint8_t s_data_on;
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
static uint8_t s_nf, s_pend;
static uint32_t s_apb_div = 1U;                 /* core clock / USART3 clock */
static uint32_t s_fall_t;
/* the gap from the end of the last enable pulse or BREAK to the first edge of the next SYNC, and the
   lengths of those lows: the pod's real timing, whether or not the chip accepted it */
typedef struct { uint32_t n, min, max, last, low_last, low_max; } gapst_t;
static gapst_t s_gap[2];                         /* 0: enable pulse, 1: BREAK */
static int s_ev_kind = -1;
static uint32_t s_ev_end;
static uint32_t s_rxsync, d_last_sr;
static uint32_t d_hunts, d_gate_rej, d_ival, d_bad_brr, d_bad_low, d_bursts, d_err_drop;
static uint8_t s_lastrx[8];
static uint32_t s_nlast;
static uint32_t s_sync_n, s_bad_sync, s_breaks, s_u_fe, s_u_pe, s_u_ne, s_u_ore, s_u_rx, s_u_tx,
                s_tx_over, s_sync_last, s_sync_min, s_sync_max, s_brr_last;

static uint32_t s_hold;
static uint8_t s_holdv;

void pe_updi_hold(uint32_t until)
{
    s_hold = until;
    s_holdv = 1U;
}

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
    s_pend = 0U;
    d_hunts++;
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

/* After a received byte: start the reply when the chip queued one. */
static void updi_next(uint32_t t_rx)
{
    if (s_txh != s_txt)
    {
        /* answer two bit times after the request ended (we are mid first stop bit now): the pod's
           receiver is done with its own second stop bit by then */
        uint32_t target = t_rx + 2U * s_bit_cyc;
        if (s_holdv)
        {
            if ((int32_t)(s_hold - target) > 0) target = s_hold;   /* an ACK held until a write is done */
            s_holdv = 0U;
        }
        while ((int32_t)(DWT->CYCCNT - target) < 0)
        {
        }
        d_bursts++;
        USART3->CR1 = (USART3->CR1 & ~USART_CR1_RE) | USART_CR1_TXEIE;
        s_umode = U_TX;
    }
}

/* The gap from the end of the last enable pulse or BREAK to the start of a SYNC. */
static void gap_note(uint32_t start)
{
    if (s_ev_kind >= 0)
    {
        gapst_t *g = &s_gap[s_ev_kind];
        uint32_t gap = start - s_ev_end;
        if (g->n == 0U || gap < g->min) g->min = gap;
        if (gap > g->max) g->max = gap;
        g->last = gap;
        g->n++;
        s_ev_kind = -1;
    }
}

/* An edge of the line while the chip waits for a frame: SYNC timing and BREAK.
   The SYNC (0x55 at 8E2: start 0, data 1 0 1 0 1 0 1 0, parity 0, two stop bits) falls at 0, 2, 4, 6
   and 8 bit times and stays low for two (D7 and the parity bit). The baud rate is worked out at the
   fifth falling edge, and the receiver is switched on at the rising edge that ends the SYNC, two
   bit times before the next start bit: the work in that second handler is a few register writes. */
static void updi_edge(uint32_t t, int lvl)
{
    const pe_chip_t *c = s_chip;
    if (!lvl)
    {
        if (s_nf == 0U)
        {
            gap_note(t);
            pe_now = t;
            if (!c->sync_gate())          /* disabled, or a SYNC outside its window: not taken */
            {
                d_gate_rej++;
                s_fall_t = t;
                return;
            }
        }
        if (s_nf >= 2U)
        {
            uint32_t ref = s_ft[1] - s_ft[0], d = t - s_ft[s_nf - 1U];
            if (d < ref - ref / 4U || d > ref + ref / 4U)
            {
                d_ival++;
                s_nf = 0U;
            }
        }
        if (s_nf >= 5U)
        {
            s_nf = 0U;
        }
        s_ft[s_nf++] = t;
        s_fall_t = t;
        s_pend = 0U;
        if (s_nf == 5U)
        {
            uint32_t b = (s_ft[4] - s_ft[0]) >> 3;
            uint32_t brr = (b + s_apb_div / 2U) / s_apb_div;   /* USART3 clock = core clock / s_apb_div */
            if (brr >= 16U && brr <= 0xFFFFU)
            {
                USART3->BRR = brr;
                s_brr_last = brr;
                s_bit_cyc = b;
                s_pend = 1U;
            }
            else
            {
                s_bad_sync++;
                d_bad_brr++;
                s_nf = 0U;
            }
        }
        return;
    }
    uint32_t low = t - s_fall_t;
    if (low > 1000U * pe_cyc_us)
    {
        s_breaks++;
        s_ev_kind = 1;
        s_ev_end = t;
        s_gap[1].low_last = low;
        if (low > s_gap[1].low_max) s_gap[1].low_max = low;
        s_nf = 0U;
        s_pend = 0U;
        pe_now = t;
        c->brk();
        return;
    }
    if (!s_pend)
    {
        if (low >= 20U * pe_cyc_us && s_nf <= 1U)    /* too long for a SYNC bit: the enable pulse */
        {
            s_nf = 0U;
            s_ev_kind = 0;
            s_ev_end = t;
            s_gap[0].low_last = low;
            if (low > s_gap[0].low_max) s_gap[0].low_max = low;
            pe_now = t;
            c->pulse();
        }
        return;
    }
    s_pend = 0U;
    if (low < s_bit_cyc + s_bit_cyc / 2U || low > 2U * s_bit_cyc + s_bit_cyc / 2U)
    {
        s_bad_sync++;                          /* not the SYNC's last two low bits */
        d_bad_low++;
        s_nf = 0U;
        return;
    }
    s_sync_n++;
    s_sync_last = s_bit_cyc;
    if (s_bit_cyc < s_sync_min) s_sync_min = s_bit_cyc;
    if (s_bit_cyc > s_sync_max) s_sync_max = s_bit_cyc;
    s_nf = 0U;
    pe_now = t;
    c->sync();
    EXTI->IMR &= ~UPDI_LINE;
    USART3->CR1 |= USART_CR1_RE;
    s_umode = U_RX;
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
            s_lastrx[s_nlast++ & 7U] = b;
            uint32_t bitc = s_bit_cyc;
            if ((sr & USART_SR_FE) && b == 0U)
            {
                /* BREAK: the low goes on, the rising edge (EXTI) says when it ends */
                s_fall_t = t - 10U * bitc;
                updi_hunt();
            }
            else if (sr & (USART_SR_FE | USART_SR_PE | USART_SR_NE))
            {
                d_err_drop++;                  /* a bad frame is not taken; the pod times out and breaks */
                d_last_sr = sr;
                if (c->want_sync())
                {
                    updi_hunt();               /* the baud rate may have changed: measure the next SYNC */
                }
            }
            else if (c->want_sync())
            {
                /* between frames the receiver stays on at the measured baud rate: a SYNC is a 0x55
                   byte (it must come inside the enable window), the enable pulse a 0x00 */
                if (b == 0x55U)
                {
                    uint32_t start = t - (bitc * 21U) / 2U;       /* its start bit, 10.5 bits ago */
                    gap_note(start);
                    pe_now = start;
                    if (c->sync_gate())
                    {
                        c->sync();
                        s_sync_n++;
                        s_rxsync++;
                    }
                    else
                    {
                        d_gate_rej++;
                    }
                }
                else if (b == 0U)
                {
                    pe_now = t - bitc / 2U;                        /* the low ends where the stop bits start */
                    s_ev_kind = 0;
                    s_ev_end = pe_now;
                    s_gap[0].low_last = 10U * bitc;
                    c->pulse();
                }
                else
                {
                    pe_now = t;
                    c->rx(b);
                }
            }
            else
            {
                pe_now = t;
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
        updi_rx_on();
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
    s_tx_over = s_sync_last = s_sync_max = s_brr_last = 0U;
    s_holdv = 0U;
    d_hunts = d_gate_rej = d_ival = d_bad_brr = d_bad_low = d_bursts = d_err_drop = 0U;
    s_rxsync = d_last_sr = s_nlast = 0U;
    memset(s_gap, 0, sizeof s_gap);
    s_ev_kind = -1;
    s_sync_min = 0xFFFFFFFFU;
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    s_apb_div = SystemCoreClock / pclk;
    s_nf = s_pend = 0U;
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

/* The bit stream is followed by polling, not by one interrupt per edge. The first edge of a burst
   wakes an interrupt (EXTI15_10 or EXTI0, priority 0); this loop then watches the EXTI pending bits
   of the clock, reset and data lines (they are set by every edge, interrupt masked or not, and the
   handler is not re-entered) and stays until the lines have been quiet for 100 us. An edge is
   timestamped about 20 cycles (150 ns at 128 MHz) after it happened, and an edge that arrives while
   the previous one is still handled is counted (`late`), a pending bit whose level did not change
   (two edges between two looks) is `missed`. Other interrupts wait while the loop runs; the
   watchdog is kicked from it. */
static uint32_t s_lines;                 /* the lines the loop watches */
static int s_prev_rst;

static void pe_poll(const pe_chip_t *c)
{
    uint32_t idle_t = DWT->CYCCNT;
    const uint32_t idle_lim = 100U * pe_cyc_us;
    uint32_t spin = 0U, n = 0U;
    for (;;)
    {
        uint32_t pr = EXTI->PR & s_lines;
        if (pr == 0U)
        {
            if (DWT->CYCCNT - idle_t > idle_lim)
            {
                return;
            }
            if ((++spin & 0xFFU) == 0U)
            {
                IWDG->KR = 0xAAAAU;
            }
            continue;
        }
        uint32_t t = DWT->CYCCNT;
        uint32_t idr = GPIOB->IDR;
        EXTI->PR = pr;
        idle_t = t;
        if (pr & RST_LINE)
        {
            int low = (int)((idr & 1U) == 0U);
            if (low != s_prev_rst)
            {
                s_prev_rst = low;
                pe_now = t;
                c->nrst(low);
                drive_data(c->drive());
                s_nrst_edges++;
            }
        }
        if ((pr & DATA_LINE) && c->data != NULL && s_drv < 0)
        {
            pe_now = t;
            c->data((int)((idr >> 12) & 1U));
        }
        if (pr & CLK_LINE)
        {
            int clk = (int)((idr >> 14) & 1U);
            if (clk == s_prev_clk)
            {
                s_missed++;        /* both edges came between two looks: a pulse we could not follow */
            }
            else
            {
                s_prev_clk = clk;
                pe_now = t;
                c->edge(clk, s_drv >= 0 ? -1 : (int)((idr >> 12) & 1U));
                drive_data(c->drive());
                if (c->data_want != NULL && (*c->data_want != 0U) != s_data_on)
                {
                    s_data_on = *c->data_want != 0U;
                    if (s_data_on) { EXTI->PR = DATA_LINE; EXTI->IMR |= DATA_LINE; s_lines |= DATA_LINE; }
                    else { EXTI->IMR &= ~DATA_LINE; s_lines &= ~DATA_LINE; }
                }
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
        if ((++n & 0x3FU) == 0U)
        {
            IWDG->KR = 0xAAAAU;
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    const pe_chip_t *c = s_chip;
    if (c == NULL)
    {
        EXTI->PR = CLK_LINE | UPDI_LINE | DATA_LINE;
        return;
    }
    if (!c->uart)
    {
        pe_poll(c);
        return;
    }
    uint32_t t = DWT->CYCCNT;
    uint32_t pr = EXTI->PR & UPDI_LINE;
    EXTI->PR = pr;
    if (pr && s_umode == U_HUNT)
    {
        s_edges++;
        updi_edge(t, (int)((GPIOC->IDR >> 10) & 1U));
    }
}

void EXTI0_IRQHandler(void)
{
    const pe_chip_t *c = s_chip;
    if (c == NULL || c->uart || c->nrst == NULL)
    {
        EXTI->PR = RST_LINE;
        return;
    }
    pe_poll(c);
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
    if (c->data != NULL)
    {
        SYSCFG->EXTICR[3] = (SYSCFG->EXTICR[3] & ~0xFUL) | 1UL;               /* EXTI12: port B */
        EXTI->RTSR |= DATA_LINE;
        EXTI->FTSR |= DATA_LINE;
        EXTI->PR = DATA_LINE;
        EXTI->IMR |= DATA_LINE;
        s_data_on = 1U;
    }
    s_lines = CLK_LINE | (c->reset ? RST_LINE : 0U) | (c->data != NULL ? DATA_LINE : 0U);
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
        s_prev_rst = 0;
        if ((GPIOB->IDR & 1U) == 0U)
        {
            pe_now = DWT->CYCCNT;
            s_prev_rst = 1;
            c->nrst(1);
        }
        EXTI->PR = RST_LINE;
        EXTI->IMR |= RST_LINE;
    }
}

static void wires_stop(void)
{
    EXTI->IMR &= ~(CLK_LINE | RST_LINE | DATA_LINE);
    EXTI->PR = CLK_LINE | RST_LINE | DATA_LINE;
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
    clock_slow();
}

static void pe_start(const pe_chip_t *c)
{
    pe_stop();
    rs485_stop();
    uart3_close();
    clock_fast();
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

static unsigned long us_of(uint32_t cyc)
{
    return (unsigned long)(cyc / pe_cyc_us);
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
        if (s_missed != 0U || s_late != 0U)
        {
            printf("PROG_EMU WARNING timing_valid=0 missed=%lu late=%lu: the node did not follow every edge "
                   "(edges merged or taken late), so the high/low times and the frames below are NOT the "
                   "pod's timing and the chip saw a damaged stream\r\n",
                   (unsigned long)s_missed, (unsigned long)s_late);
        }
        else
        {
            printf("PROG_EMU timing_valid=1 missed=0 late=0\r\n");
        }
        print_gap("high", &s_hi);
        print_gap("low", &s_lo);
        print_bins("high", &s_hi);
        print_bins("low", &s_lo);
    }
    else
    {
        printf("PROG_EMU updi_wire syncs=%lu bad_sync=%lu breaks=%lu framing=%lu parity=%lu noise=%lu "
               "overrun=%lu rx=%lu tx=%lu tx_overflow=%lu baud=%lu brr=%lu sync_bit_ns=%lu "
               "min=%lu max=%lu\r\n",
               (unsigned long)s_sync_n, (unsigned long)s_bad_sync, (unsigned long)s_breaks,
               (unsigned long)s_u_fe, (unsigned long)s_u_pe, (unsigned long)s_u_ne,
               (unsigned long)s_u_ore, (unsigned long)s_u_rx, (unsigned long)s_u_tx,
               (unsigned long)s_tx_over,
               s_brr_last ? (unsigned long)(HAL_RCC_GetPCLK1Freq() / s_brr_last) : 0UL, (unsigned long)s_brr_last,
               ns_of(s_sync_last), s_sync_n ? ns_of(s_sync_min) : 0UL, ns_of(s_sync_max));
    }
    if (c->uart)
    {
        printf("PROG_EMU updi_dbg hunts=%lu gate_rejects=%lu interval_resets=%lu bad_brr=%lu bad_low=%lu "
               "tx_bursts=%lu bad_frames_dropped=%lu last_bad_sr=%04lx syncs_by_byte=%lu last_rx=",
               (unsigned long)d_hunts, (unsigned long)d_gate_rej, (unsigned long)d_ival,
               (unsigned long)d_bad_brr, (unsigned long)d_bad_low, (unsigned long)d_bursts,
               (unsigned long)d_err_drop, (unsigned long)d_last_sr, (unsigned long)s_rxsync);
        for (uint32_t i = 0; i < 8U && i < s_nlast; i++)
        {
            printf("%02x", s_lastrx[(s_nlast - 1U - i) & 7U]);
        }
        printf("\r\n");
    }
    if (c->uart)
    {
        static const char *const nm[2] = { "pulse", "break" };
        printf("PROG_EMU updi_gap_us");
        for (unsigned i = 0; i < 2; i++)
        {
            const gapst_t *g = &s_gap[i];
            printf(" %s_n=%lu %s_gap_min=%lu %s_gap_max=%lu %s_gap_last=%lu %s_low_last=%lu %s_low_max=%lu",
                   nm[i], (unsigned long)g->n, nm[i], g->n ? us_of(g->min) : 0UL, nm[i], us_of(g->max),
                   nm[i], us_of(g->last), nm[i], us_of(g->low_last), nm[i], us_of(g->low_max));
        }
        printf("\r\n");
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
    memset(s_gap, 0, sizeof s_gap);
    s_ev_kind = -1;
    d_hunts = d_gate_rej = d_ival = d_bad_brr = d_bad_low = d_bursts = d_err_drop = 0U;
    s_nlast = s_rxsync = d_last_sr = 0U;
}

/* ---------------------------------------------------------------- console */

void progemu_help(void)
{
    printf("  prog_emu <updi|updi2|updimega|pic|pic8|pic18|pic18q10|c2|c2early|tpi|tpi20|tpi40|tpi104|pdi>   the NUCLEO answers as that chip (see README)\r\n");
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
