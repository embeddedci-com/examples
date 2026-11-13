#include "spi2.h"
#include "dwt.h"
#include "stm32f4xx_hal.h"

#define CS_PORT GPIOB
#define CS_PIN GPIO_PIN_12

static uint8_t g_mode;
static uint32_t g_hz;

void spi2_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    CS_PORT->BSRR = CS_PIN; /* deselected before it becomes an output */
    GPIO_InitTypeDef g = {0};
    g.Pin = CS_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_PORT, &g);

    g.Pin = GPIO_PIN_13 | GPIO_PIN_15;
    g.Mode = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &g);
    g.Pin = GPIO_PIN_14;
    g.Pull = GPIO_PULLUP; /* MISO: a target that lets go reads 0xFF, not noise */
    HAL_GPIO_Init(GPIOB, &g);

    (void)spi2_config(0U, SPI2_DEFAULT_HZ);
}

uint32_t spi2_config(uint8_t mode, uint32_t hz)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    if (mode > 3U || hz < pclk / 256U)
    {
        return 0U;
    }
    uint32_t br = 0U;
    while (br < 7U && (pclk >> (br + 1U)) > hz)
    {
        br++;
    }
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | (br << SPI_CR1_BR_Pos) |
                ((mode & 2U) ? SPI_CR1_CPOL : 0U) | ((mode & 1U) ? SPI_CR1_CPHA : 0U);
    SPI2->CR2 = 0U;
    SPI2->CR1 |= SPI_CR1_SPE;
    g_mode = mode;
    g_hz = pclk >> (br + 1U);
    return g_hz;
}

uint8_t spi2_mode(void)
{
    return g_mode;
}

uint32_t spi2_hz(void)
{
    return g_hz;
}

void spi2_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    while (SPI2->SR & SPI_SR_RXNE)
    {
        (void)SPI2->DR; /* nothing stale from before */
    }
    CS_PORT->BSRR = (uint32_t)CS_PIN << 16U;
    dwt_delay_us(2U); /* CS setup: the target sees CS before the first edge */
    for (uint16_t i = 0U; i < n; i++)
    {
        while ((SPI2->SR & SPI_SR_TXE) == 0U)
        {
        }
        *(volatile uint8_t *)&SPI2->DR = tx[i];
        while ((SPI2->SR & SPI_SR_RXNE) == 0U)
        {
        }
        rx[i] = *(volatile uint8_t *)&SPI2->DR;
    }
    while (SPI2->SR & SPI_SR_BSY)
    {
    }
    dwt_delay_us(2U);
    CS_PORT->BSRR = CS_PIN;
}
