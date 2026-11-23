#include "uart3.h"
#include "stm32f4xx_hal.h"
#include "progemu.h"
#include <string.h>

#define TX_PIN GPIO_PIN_10
#define RX_PIN GPIO_PIN_11

static volatile uint8_t g_ring[UART3_RING_SIZE];
static volatile uint16_t g_head;
static volatile uint16_t g_tail;
static volatile uart3_stats_t g_st;
static volatile uint8_t g_echo;
static void (*volatile g_hook)(uint8_t byte, uint32_t sr);

static void pins_input(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin = TX_PIN | RX_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);
}

int uart3_open(uint32_t baud)
{
    return uart3_open_format(baud, 'N', 1U);
}

int uart3_open_format(uint32_t baud, char parity, uint8_t stop_bits)
{
    if (baud < 1200U || baud > 1000000U || (parity != 'N' && parity != 'E' && parity != 'O') ||
        (stop_bits != 1U && stop_bits != 2U))
    {
        return -1;
    }
    uart3_close();
    g_echo = 0U;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = TX_PIN;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = RX_PIN;
    g.Pull = GPIO_PULLUP; /* an idle (or released) line reads high */
    HAL_GPIO_Init(GPIOC, &g);

    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    USART3->CR1 = 0U;
    USART3->CR2 = 0U;
    USART3->CR3 = 0U;
    USART3->BRR = (pclk + baud / 2U) / baud; /* oversampling 16: BRR = fck / baud, 12.4 fixed point */
    memset((void *)&g_st, 0, sizeof(g_st));
    g_head = g_tail = 0U;
    g_st.baud = baud;
    g_st.open = 1U;
    g_st.format[0] = '8';
    g_st.format[1] = parity;
    g_st.format[2] = (char)('0' + stop_bits);
    g_st.format[3] = '\0';
    /* With parity the word is 9 bits (M): 8 data bits and the parity bit. */
    uint32_t cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    if (parity != 'N')
    {
        cr1 |= USART_CR1_M | USART_CR1_PCE | (parity == 'O' ? USART_CR1_PS : 0U);
    }
    USART3->CR2 = (stop_bits == 2U) ? USART_CR2_STOP_1 : 0U;
    USART3->CR1 = cr1;
    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART3_IRQn);
    return 0;
}

void uart3_close(void)
{
    NVIC_DisableIRQ(USART3_IRQn);
    if (g_st.open)
    {
        while ((USART3->SR & USART_SR_TC) == 0U)
        {
        }
    }
    USART3->CR1 = 0U;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    pins_input();
    g_st.open = 0U;
}

const uart3_stats_t *uart3_stats(void)
{
    return (const uart3_stats_t *)&g_st;
}

void uart3_write(const uint8_t *b, uint16_t n)
{
    if (!g_st.open)
    {
        return;
    }
    for (uint16_t i = 0U; i < n; i++)
    {
        while ((USART3->SR & USART_SR_TXE) == 0U)
        {
        }
        USART3->DR = b[i];
        g_st.tx_total++;
    }
    while ((USART3->SR & USART_SR_TC) == 0U)
    {
    }
}

int uart3_getc(uint8_t *b)
{
    if (g_tail == g_head)
    {
        return 0;
    }
    *b = g_ring[g_tail];
    g_tail = (uint16_t)((g_tail + 1U) % UART3_RING_SIZE);
    return 1;
}

uint16_t uart3_pending(void)
{
    return (uint16_t)((g_head + UART3_RING_SIZE - g_tail) % UART3_RING_SIZE);
}

void uart3_set_rx_hook(void (*hook)(uint8_t byte, uint32_t sr))
{
    g_hook = hook;
}

void uart3_set_echo(uint8_t on)
{
    g_echo = on;
}

void uart3_flush(void)
{
    g_tail = g_head;
}

void USART3_IRQHandler(void)
{
    if (progemu_usart3_irq())
    {
        return; /* the UPDI emulator owns USART3 (progemu.c) */
    }
    uint32_t sr;
    while ((sr = USART3->SR) & (USART_SR_RXNE | USART_SR_ORE))
    {
        uint8_t byte = (uint8_t)(USART3->DR & 0xFFU); /* reading SR then DR clears the error flags */
        if (sr & USART_SR_ORE)
        {
            g_st.overrun++;
        }
        if (sr & USART_SR_FE)
        {
            g_st.framing++;
        }
        if (sr & USART_SR_NE)
        {
            g_st.noise++;
        }
        if (sr & USART_SR_PE)
        {
            g_st.parity++;
        }
        if ((sr & USART_SR_RXNE) == 0U)
        {
            continue;
        }
        g_st.rx_total++;
        if (g_hook != NULL)
        {
            g_hook(byte, sr);
            continue;
        }
        if (g_echo && (USART3->SR & USART_SR_TXE))
        {
            USART3->DR = byte; /* same baud both ways, so TX keeps up byte for byte */
            g_st.tx_total++;
        }
        uint16_t next = (uint16_t)((g_head + 1U) % UART3_RING_SIZE);
        if (next == g_tail)
        {
            g_st.dropped++;
            continue;
        }
        g_ring[g_head] = byte;
        g_head = next;
    }
}
