#include "analog.h"
#include "proc.h"
#include "stm32f4xx_hal.h"

#define AIN_CHANNEL 1U       /* PA1 */
#define VREFINT_CHANNEL 17U
#define VREFINT_CAL (*(const volatile uint16_t *)0x1FFF7A2AU)

static ain_state_t g_ain;
static uint32_t g_filt_state;
static uint32_t g_next_ms;
static uint32_t g_next_vref_ms;

/* One conversion, polled. 480-cycle sample time at ADCCLK 8 MHz = 60 us: long
 * enough for the 10 kOhm series resistor in front of PA1 and for VREFINT (>= 10 us). */
static uint32_t adc_convert(uint32_t channel)
{
    ADC1->SQR3 = channel;
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while ((ADC1->SR & ADC_SR_EOC) == 0U)
    {
    }
    return ADC1->DR & 0xFFFU;
}

static uint32_t adc_oversample(uint32_t channel)
{
    uint32_t sum = 0U;
    for (uint32_t i = 0; i < AIN_OVERSAMPLE; i++)
    {
        sum += adc_convert(channel);
    }
    return (sum + AIN_OVERSAMPLE / 2U) / AIN_OVERSAMPLE;
}

static void update_vdda(void)
{
    g_ain.vdda_mv = proc_vdda_mv(adc_oversample(VREFINT_CHANNEL), VREFINT_CAL);
}

static void sample(void)
{
    g_ain.raw = adc_oversample(AIN_CHANNEL);
    g_ain.mv = proc_raw_to_mv(g_ain.raw, g_ain.vdda_mv);
}

void ain_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    GPIOA->PUPDR &= ~(3U << (AIN_CHANNEL * 2U));
    GPIOA->MODER |= 3U << (AIN_CHANNEL * 2U); /* analog */

    ADC123_COMMON->CCR = ADC_CCR_TSVREFE; /* ADCPRE = /2 -> 8 MHz, VREFINT on */
    ADC1->CR1 = 0U;                       /* 12-bit, single channel */
    ADC1->CR2 = 0U;
    ADC1->SMPR2 = 7U << (AIN_CHANNEL * 3U);
    ADC1->SMPR1 = 7U << ((VREFINT_CHANNEL - 10U) * 3U);
    ADC1->SQR1 = 0U; /* one conversion */
    ADC1->CR2 = ADC_CR2_ADON;
    HAL_Delay(1); /* tSTAB + VREFINT start-up */

    update_vdda();
    sample();
    g_filt_state = proc_ema_seed(g_ain.mv);
    g_ain.filt_mv = g_ain.mv;
    g_next_ms = HAL_GetTick() + AIN_PERIOD_MS;
    g_next_vref_ms = HAL_GetTick() + AIN_VREF_PERIOD_MS;
}

void ain_tick(uint32_t now_ms)
{
    if ((int32_t)(now_ms - g_next_vref_ms) >= 0)
    {
        g_next_vref_ms = now_ms + AIN_VREF_PERIOD_MS;
        update_vdda();
    }
    if ((int32_t)(now_ms - g_next_ms) < 0)
    {
        return;
    }
    g_next_ms += AIN_PERIOD_MS;
    if ((int32_t)(now_ms - g_next_ms) >= 0)
    {
        g_next_ms = now_ms + AIN_PERIOD_MS; /* fell behind (a long command): no backlog */
    }
    sample();
    g_ain.filt_mv = proc_ema_step(&g_filt_state, g_ain.mv, AIN_FILTER_SHIFT);
    g_ain.samples++;
}

const ain_state_t *ain_read_now(void)
{
    sample();
    return &g_ain;
}

const ain_state_t *ain_state(void)
{
    return &g_ain;
}
