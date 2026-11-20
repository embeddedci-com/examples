/*
 * rs485mock.c - the RS-485 bus and Modbus RTU device of rs485mock.h, on USART3 and PA8 (DE).
 */
#include "rs485mock.h"

#include <string.h>

#include "dwt.h"
#include "stm32f4xx_hal.h"
#include "uart3.h"

#define DE_PORT GPIOA
#define DE_PIN  GPIO_PIN_8
#define DELAY_MAX_US 100000U

static mb_slave_t s_mb;
static rs485_stats_t s_st;

/* the frame being received (USART3 interrupt) and the one waiting for the main loop */
static volatile uint8_t s_rx[MB_FRAME_MAX];
static volatile uint16_t s_rx_len;
static volatile uint8_t s_rx_bad;
static volatile uint32_t s_rx_last;      /* cycle count of the last byte */
static uint32_t s_char_cyc, s_t15_cyc, s_t35_cyc, s_late_cyc;
static volatile uint32_t s_de_fell;      /* cycle count of DE's last falling edge (EXTI line 8) */
#define DE_EXTI (1UL << 8)

static int de_high(void)
{
    return (DE_PORT->IDR & DE_PIN) != 0U;
}

/* From EXTI9_5_IRQHandler (main.c): DE fell. */
void rs485_de_edge(void)
{
    s_de_fell = dwt_cycles();
}

/* Was the pod's driver on for the byte that just completed? RXNE comes late in its (first) stop
   bit, and a master that drives DE well drops it at the end of the last stop bit, often before
   this interrupt runs: a byte still counts when DE fell within its stop bit (or the edge is still
   pending, EXTI9_5 being lower priority than USART3). EXTI9_5_IRQHandler stores the time before
   it clears the pending bit, so a preempted handler is seen by one test or the other. */
static int byte_driven(uint32_t now)
{
    if (de_high())
    {
        return 1;
    }
    if (EXTI->PR & DE_EXTI)
    {
        return 1;          /* it fell just now, the edge not yet taken */
    }
    return now - s_de_fell < s_late_cyc;
}

static void on_byte(uint8_t b, uint32_t sr)
{
    uint32_t now = dwt_cycles();
    if (s_st.de && !byte_driven(now))
    {
        s_st.undriven++;   /* the pod's driver was off: the byte never reached the bus */
        return;
    }
    if (s_rx_len > 0U && now - s_rx_last > s_t15_cyc)
    {
        s_rx_bad = 1U;     /* a gap inside the frame (the main loop ends frames at 3.5) */
    }
    if (sr & (USART_SR_FE | USART_SR_PE | USART_SR_NE | USART_SR_ORE))
    {
        s_rx_bad = 1U;
    }
    if (s_rx_len < MB_FRAME_MAX)
    {
        s_rx[s_rx_len++] = b;
    }
    else
    {
        s_rx_bad = 1U;
    }
    s_rx_last = now;
}

static uint32_t us_to_cyc(uint32_t us)
{
    return us * (SystemCoreClock / 1000000U);
}

int rs485_start(uint8_t addr, uint32_t baud, char parity, uint8_t stop_bits, uint32_t delay_us,
                uint8_t de)
{
    if (addr < 1U || addr > 247U || delay_us > DELAY_MAX_US)
    {
        return -1;
    }
    rs485_stop();
    if (uart3_open_format(baud, parity, stop_bits) != 0)
    {
        return -1;
    }
    dwt_init();
    mb_init(&s_mb, addr);
    memset(&s_st, 0, sizeof(s_st));
    s_st.on = 1U;
    s_st.baud = baud;
    s_st.format[0] = '8';
    s_st.format[1] = parity;
    s_st.format[2] = (char)('0' + stop_bits);
    s_st.delay_us = delay_us;
    s_st.de = de ? 1U : 0U;
    uint32_t bits = 1U + 8U + (parity != 'N' ? 1U : 0U) + stop_bits;
    s_char_cyc = (uint32_t)(((uint64_t)SystemCoreClock * bits) / baud);
    /* above 19200 baud the spec fixes t1.5 = 750 us and t3.5 = 1750 us */
    s_t15_cyc = baud > 19200U ? us_to_cyc(750U) : (s_char_cyc * 3U) / 2U;
    s_t35_cyc = baud > 19200U ? us_to_cyc(1750U) : (s_char_cyc * 7U) / 2U;
    /* a DE that fell up to 2 bit times (the stop bits) plus 20 us of interrupt latency ago */
    s_late_cyc = (s_char_cyc / bits) * 2U + us_to_cyc(20U);
    s_de_fell = dwt_cycles() - s_late_cyc - 1U;
    if (s_st.de)
    {
        /* EXTI line 8 on PA8, falling edge (the pin stays the strobe's input with its pull-down) */
        __HAL_RCC_SYSCFG_CLK_ENABLE();
        SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI8;          /* port A */
        EXTI->FTSR |= DE_EXTI;
        EXTI->RTSR &= ~DE_EXTI;
        EXTI->PR = DE_EXTI;
        EXTI->IMR |= DE_EXTI;
        NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
    s_rx_len = 0U;
    s_rx_bad = 0U;
    uart3_set_rx_hook(on_byte);
    return 0;
}

void rs485_stop(void)
{
    if (!s_st.on)
    {
        return;
    }
    uart3_set_rx_hook(NULL);
    uart3_close();
    EXTI->IMR &= ~DE_EXTI;
    EXTI->PR = DE_EXTI;
    s_st.on = 0U;
}

void rs485_set_delay(uint32_t delay_us)
{
    s_st.delay_us = delay_us > DELAY_MAX_US ? DELAY_MAX_US : delay_us;
}

const rs485_stats_t *rs485_stats(void)
{
    return &s_st;
}

mb_slave_t *rs485_slave(void)
{
    return &s_mb;
}

static void wait_until(uint32_t start, uint32_t cycles)
{
    while (dwt_cycles() - start < cycles)
    {
    }
}

void rs485_service(void)
{
    if (!s_st.on || s_rx_len == 0U)
    {
        return;
    }
    /* the frame ends after 3.5 characters of silence */
    NVIC_DisableIRQ(USART3_IRQn);
    uint32_t last = s_rx_last;
    if (dwt_cycles() - last <= s_t35_cyc)
    {
        NVIC_EnableIRQ(USART3_IRQn);
        return;
    }
    uint8_t f[MB_FRAME_MAX];
    uint16_t n = s_rx_len;
    uint8_t bad = s_rx_bad;
    memcpy(f, (const uint8_t *)s_rx, n);
    s_rx_len = 0U;
    s_rx_bad = 0U;
    NVIC_EnableIRQ(USART3_IRQn);
    if (bad)
    {
        s_st.frame_errors++;
        return;
    }
    uint8_t reply[MB_FRAME_MAX];
    size_t rn = mb_frame(&s_mb, f, n, reply);
    if (rn == 0U)
    {
        return;
    }
    /* answer delay_us after the request ended, never inside the 3.5-character silence */
    uint32_t wait = us_to_cyc(s_st.delay_us);
    wait_until(last, wait > s_t35_cyc ? wait : s_t35_cyc);
    for (size_t i = 0; i < rn; i++)
    {
        uint32_t t = dwt_cycles();
        if (s_st.de && de_high())
        {
            /* the pod is still driving the bus with its receiver off: this byte is lost to it
               (and collides on a real bus), but it takes its time on the wire all the same */
            s_st.collisions++;
            wait_until(t, s_char_cyc);
            continue;
        }
        uart3_write(&reply[i], 1U);
    }
}
