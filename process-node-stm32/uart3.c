#include "uart3.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define TX_PIN GPIO_PIN_10
#define RX_PIN GPIO_PIN_11

static volatile uint8_t g_ring[UART3_RING_SIZE];
static volatile uint16_t g_head;
static volatile uint16_t g_tail;
static volatile uart3_stats_t g_st;

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
    if (baud < 1200U || baud > 1000000U)
    {
        return -1;
    }
    uart3_close();
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
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
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

void uart3_flush(void)
{
    g_tail = g_head;
}

void USART3_IRQHandler(void)
{
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
        if ((sr & USART_SR_RXNE) == 0U)
        {
            continue;
        }
        g_st.rx_total++;
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
