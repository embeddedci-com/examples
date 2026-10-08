#include "env.h"
#include "bmp280_comp.h"
#include "stm32f4xx_hal.h"

#define BMP280_CHIP_ID 0x58U
#define REG_CALIB 0x88U
#define REG_CHIP_ID 0xD0U
#define REG_RESET 0xE0U
#define REG_CTRL_MEAS 0xF4U
#define REG_CONFIG 0xF5U
#define REG_DATA 0xF7U
#define I2C_TIMEOUT_MS 10U

static I2C_HandleTypeDef g_i2c;
static env_state_t g_env;
static bmp280_calib_t g_calib;
static uint32_t g_next_ms;
static uint32_t g_fail_run;

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
    g_i2c.Init.ClockSpeed = 100000U;
    g_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    g_i2c.Init.OwnAddress1 = 0U;
    g_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    g_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    g_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    g_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    (void)HAL_I2C_Init(&g_i2c);
}

/* A failed transfer can leave the peripheral BUSY; start it over. */
static void bus_recover(void)
{
    (void)HAL_I2C_DeInit(&g_i2c);
    bus_init();
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
        if (rd(addrs[i], REG_CHIP_ID, &id, 1U) != 0 || id != BMP280_CHIP_ID)
        {
            continue;
        }
        if (wr(addrs[i], REG_RESET, 0xB6U) != 0)
        {
            continue;
        }
        HAL_Delay(3); /* start-up after reset: 2 ms */
        if (rd(addrs[i], REG_CALIB, cal, sizeof(cal)) != 0 || wr(addrs[i], REG_CONFIG, 0x00U) != 0 ||
            wr(addrs[i], REG_CTRL_MEAS, 0x27U) != 0) /* T x1, P x1, normal mode */
        {
            continue;
        }
        bmp280_calib_parse(cal, &g_calib);
        g_env.addr = addrs[i];
        return 0;
    }
    return -1;
}

static int read_sample(void)
{
    uint8_t d[6];
    if (rd(g_env.addr, REG_DATA, d, sizeof(d)) != 0)
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
    return (int32_t)(now_ms - g_next_ms) >= 0;
}

void env_tick(uint32_t now_ms)
{
    if ((int32_t)(now_ms - g_next_ms) < 0)
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
