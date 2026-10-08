#include "analog.h"
#include "proc.h"
#include "sine_table.h"
#include "stm32f4xx_hal.h"

#define AIN_CHANNEL 1U       /* PA1 */
#define VREFINT_CHANNEL 17U
#define VREFINT_CAL (*(const volatile uint16_t *)0x1FFF7A2AU)

static ain_state_t g_ain;
static uint32_t g_filt_state;
static uint32_t g_next_ms;
static uint32_t g_next_vref_ms;

/* One conversion, polled. 480-cycle sample time at ADCCLK 8 MHz = 60 us: long
 * enough for the 5-10 kOhm series resistor in front of PA1 and for VREFINT (>= 10 us). */
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

int ain_due(uint32_t now_ms)
{
    return (int32_t)(now_ms - g_next_ms) >= 0;
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

/* ---------------------------------------------------------------- analog out */

#define AOUT_CHANNEL_PIN 4U /* PA4 */

static aout_state_t g_aout;
static volatile uint32_t g_phase;
static volatile uint32_t g_phase_inc;
static volatile int32_t g_amp_code;
static volatile int32_t g_offset_code;

static uint32_t mv_to_code(uint32_t mv)
{
    uint32_t vdda = g_ain.vdda_mv ? g_ain.vdda_mv : PROC_VREFINT_CAL_MV;
    uint32_t code = (mv * PROC_ADC_FULL_SCALE + vdda / 2U) / vdda;
    return code > PROC_ADC_FULL_SCALE ? PROC_ADC_FULL_SCALE : code;
}

static int in_range(uint32_t lo_mv, uint32_t hi_mv)
{
    uint32_t vdda = g_ain.vdda_mv ? g_ain.vdda_mv : PROC_VREFINT_CAL_MV;
    return lo_mv >= AOUT_MIN_MV && hi_mv + AOUT_HEADROOM_MV <= vdda;
}

static void dac_enable(void)
{
    DAC->CR |= DAC_CR_EN1; /* buffered (BOFF1 = 0), no trigger: DHR -> output at once */
}

static void tim6_stop(void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->DIER = 0U;
    NVIC_DisableIRQ(TIM6_DAC_IRQn);
}

void aout_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN | RCC_APB1ENR_TIM6EN;
    (void)RCC->APB1ENR;

    GPIOA->PUPDR &= ~(3U << (AOUT_CHANNEL_PIN * 2U));
    GPIOA->MODER |= 3U << (AOUT_CHANNEL_PIN * 2U); /* analog; high-Z while the DAC is off */
    DAC->CR = 0U;

    /* TIM6 on APB1 (16 MHz timer clock): update at AOUT_RATE_HZ. */
    TIM6->PSC = 0U;
    TIM6->ARR = (16000000U / AOUT_RATE_HZ) - 1U;
    TIM6->CNT = 0U;
    NVIC_SetPriority(TIM6_DAC_IRQn, 3);
    g_aout.mode = AOUT_OFF;
}

int aout_dc(uint32_t mv)
{
    if (!in_range(mv, mv))
    {
        return -1;
    }
    tim6_stop();
    g_aout.code = mv_to_code(mv);
    DAC->DHR12R1 = g_aout.code;
    dac_enable();
    g_aout.mode = AOUT_DC;
    g_aout.mv = mv;
    g_aout.amp_mv = 0U;
    g_aout.hz = 0U;
    return 0;
}

int aout_sine(uint32_t hz, uint32_t amp_mv, uint32_t offset_mv)
{
    if (hz == 0U || hz > AOUT_SINE_MAX_HZ || amp_mv == 0U || amp_mv > offset_mv ||
        !in_range(offset_mv - amp_mv, offset_mv + amp_mv))
    {
        return -1;
    }
    tim6_stop();
    g_offset_code = (int32_t)mv_to_code(offset_mv);
    g_amp_code = (int32_t)mv_to_code(amp_mv);
    g_phase = 0U;
    g_phase_inc = (uint32_t)(((uint64_t)hz << 32) / AOUT_RATE_HZ);
    DAC->DHR12R1 = (uint32_t)g_offset_code;
    dac_enable();

    TIM6->CNT = 0U;
    TIM6->SR = 0U;
    TIM6->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
    TIM6->CR1 = TIM_CR1_CEN;

    g_aout.mode = AOUT_SINE;
    g_aout.mv = offset_mv;
    g_aout.amp_mv = amp_mv;
    g_aout.hz = hz;
    g_aout.code = (uint32_t)g_offset_code;
    return 0;
}

void aout_off(void)
{
    tim6_stop();
    DAC->CR &= ~DAC_CR_EN1; /* PA4 back to high-Z */
    g_aout.mode = AOUT_OFF;
    g_aout.mv = g_aout.amp_mv = g_aout.hz = g_aout.code = 0U;
}

const aout_state_t *aout_state(void)
{
    return &g_aout;
}

/* DDS: a 32-bit phase accumulator, the top 8 bits index the sine table. */
void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR = 0U;
    g_phase += g_phase_inc;
    int32_t v = g_offset_code + ((g_amp_code * SINE_Q15[g_phase >> 24]) >> 15);
    DAC->DHR12R1 = (uint32_t)v;
}
