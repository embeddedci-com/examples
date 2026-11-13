#include "env.h"
#include "i2ctarget.h"
#include "bmp280_comp.h"
#include "stm32f4xx_hal.h"

#define BMP280_CHIP_ID 0x58U
#define BME280_CHIP_ID 0x60U
#define REG_CALIB 0x88U
#define REG_CALIB_H1 0xA1U
#define REG_CALIB_H2 0xE1U
#define REG_CTRL_HUM 0xF2U
#define REG_CHIP_ID 0xD0U
#define REG_RESET 0xE0U
#define REG_CTRL_MEAS 0xF4U
#define REG_CONFIG 0xF5U
#define REG_DATA 0xF7U
#define I2C_TIMEOUT_MS 10U

static I2C_HandleTypeDef g_i2c;
static env_state_t g_env;
static bmp280_calib_t g_calib;
static bme280_hcalib_t g_hcalib;
static uint32_t g_next_ms;
static uint32_t g_fail_run;
static uint32_t g_hz = 100000U;
static int g_enabled = 1;

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1)
    {
        return;
    }
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP; /* the pod adds its own pull-ups; these keep an empty bus idle */
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C1_CLK_ENABLE();
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_I2C1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
    }
}

static void bus_init(void)
{
    g_i2c.Instance = I2C1;
    g_i2c.Init.ClockSpeed = g_hz;
    g_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2c.Init.OwnAddress1 = 0U;
    g_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&g_i2c);
}

/* A failed transfer can leave the peripheral BUSY (a target held SDA low, the F4's I2C does not
 * always notice the STOP that follows); a software reset clears it, then start it over. */
static void bus_recover(void)
{
    (void)HAL_I2C_DeInit(&g_i2c);
    __HAL_RCC_I2C1_CLK_ENABLE();
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;
    bus_init();
}

void env_i2c_reset(void)
{
    bus_recover();
}

int env_i2c_set_hz(uint32_t hz)
{
    if (hz < 10000U || hz > 400000U)
    {
        return -1;
    }
    g_hz = hz;
    bus_recover();
    return 0;
}

uint32_t env_i2c_hz(void)
{
    return g_hz;
}

static int rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (HAL_I2C_Mem_Read(&g_i2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, n,
                         I2C_TIMEOUT_MS) != HAL_OK)
    {
        bus_recover();
        return -1;
    }
    return 0;
}

static int wr(uint8_t addr, uint8_t reg, uint8_t v)
{
    if (HAL_I2C_Mem_Write(&g_i2c, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT, &v, 1U,
                          I2C_TIMEOUT_MS) != HAL_OK)
    {
        bus_recover();
        return -1;
    }
    return 0;
}

static int probe(void)
{
    static const uint8_t addrs[] = {0x76U, 0x77U};
    for (uint32_t i = 0; i < sizeof(addrs); i++)
    {
        uint8_t id = 0U;
        uint8_t cal[24];
        if (rd(addrs[i], REG_CHIP_ID, &id, 1U) != 0 || (id != BMP280_CHIP_ID && id != BME280_CHIP_ID))
        {
            continue;
        }
        if (wr(addrs[i], REG_RESET, 0xB6U) != 0)
        {
            continue;
        }
        HAL_Delay(3); /* start-up after reset: 2 ms */
        if (rd(addrs[i], REG_CALIB, cal, sizeof(cal)) != 0 || wr(addrs[i], REG_CONFIG, 0x00U) != 0)
        {
            continue;
        }
        if (id == BME280_CHIP_ID)
        {
            /* Humidity: its trim, and ctrl_hum, which only takes effect with the next ctrl_meas. */
            uint8_t h1 = 0U;
            uint8_t h[7];
            if (rd(addrs[i], REG_CALIB_H1, &h1, 1U) != 0 || rd(addrs[i], REG_CALIB_H2, h, sizeof(h)) != 0 ||
                wr(addrs[i], REG_CTRL_HUM, 0x01U) != 0)
            {
                continue;
            }
            bme280_hcalib_parse(h1, h, &g_hcalib);
        }
        if (wr(addrs[i], REG_CTRL_MEAS, 0x27U) != 0) /* T x1, P x1, normal mode */
        {
            continue;
        }
        bmp280_calib_parse(cal, &g_calib);
        g_env.addr = addrs[i];
        g_env.chip = (id == BME280_CHIP_ID) ? ENV_CHIP_BME280 : ENV_CHIP_BMP280;
        return 0;
    }
    return -1;
}

static int read_sample(void)
{
    uint8_t d[8];
    uint16_t n = (g_env.chip == ENV_CHIP_BME280) ? 8U : 6U;
    if (rd(g_env.addr, REG_DATA, d, n) != 0)
    {
        return -1;
    }
    int32_t adc_p = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
    if (adc_t == 0x80000) /* "skipped" pattern: no conversion yet */
    {
        return -1;
    }
    int32_t t_fine;
    g_env.temp_dc = bmp280_temp_cdc(&g_calib, adc_t, &t_fine) / 10;
    g_env.press_pa = bmp280_press_pa(&g_calib, adc_p, t_fine);
    if (g_env.chip == ENV_CHIP_BME280)
    {
        int32_t adc_h = (int32_t)(((uint32_t)d[6] << 8) | d[7]);
        /* %RH in Q22.10 -> tenths of a percent */
        g_env.hum_dpct = (int32_t)((bme280_hum_q10(&g_hcalib, adc_h, t_fine) * 10U + 512U) >> 10);
    }
    return 0;
}

void env_init(void)
{
    bus_init();
    g_env.status = ENV_ABSENT;
    if (probe() == 0)
    {
        g_env.status = ENV_OK;
    }
    g_next_ms = HAL_GetTick();
}

int env_due(uint32_t now_ms)
{
    return g_enabled && !i2ct_active() && (int32_t)(now_ms - g_next_ms) >= 0;
}

void env_set_enabled(int on)
{
    if (on && !g_enabled)
    {
        /* Start over as at boot: probe (soft reset + setup) on the next pass. */
        g_env.status = ENV_ABSENT;
        g_env.chip = ENV_CHIP_NONE;
        g_fail_run = 0U;
        g_next_ms = HAL_GetTick();
        g_env.seq++;
    }
    else if (!on && g_enabled)
    {
        g_env.status = ENV_ABSENT;
        g_env.seq++;
    }
    g_enabled = on ? 1 : 0;
}

int env_enabled(void)
{
    return g_enabled;
}

void env_tick(uint32_t now_ms)
{
    if (!g_enabled || (int32_t)(now_ms - g_next_ms) < 0)
    {
        return;
    }
    if (g_env.status != ENV_OK)
    {
        g_next_ms = now_ms + ENV_PROBE_MS;
        if (probe() == 0)
        {
            g_env.status = ENV_OK;
            g_fail_run = 0U;
            g_next_ms = now_ms + 20U; /* first conversion (~6 ms) before the first read */
        }
        return;
    }
    g_next_ms = now_ms + ENV_PERIOD_MS;
    if (read_sample() == 0)
    {
        g_fail_run = 0U;
        g_env.reads++;
        g_env.seq++;
        return;
    }
    g_env.fails++;
    if (++g_fail_run >= ENV_FAILS_TO_LOSE)
    {
        g_env.status = ENV_LOST;
        g_env.seq++;
    }
}

const env_state_t *env_state(void)
{
    return &g_env;
}
