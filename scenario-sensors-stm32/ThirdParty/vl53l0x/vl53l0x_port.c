#include "i2c.h"

static I2C_HandleTypeDef *g_vl53_i2c = NULL;
static uint8_t g_pending_reg_valid = 0U;
static uint8_t g_pending_reg = 0U;
static uint8_t g_pending_addr = 0U;

void vl53l0x_port_set_i2c(I2C_HandleTypeDef *hi2c)
{
    g_vl53_i2c = hi2c;
    g_pending_reg_valid = 0U;
    g_pending_reg = 0U;
    g_pending_addr = 0U;
}

uint8_t i2c_write(uint8_t addr_7b, uint8_t *buf, uint8_t len)
{
    if (g_vl53_i2c == NULL || buf == NULL || len == 0U)
    {
        return 1U;
    }

    HAL_StatusTypeDef st;
    if (g_pending_reg_valid != 0U && g_pending_addr == addr_7b)
    {
        st = HAL_I2C_Mem_Write(g_vl53_i2c,
                               (uint16_t)(addr_7b << 1),
                               g_pending_reg,
                               I2C_MEMADD_SIZE_8BIT,
                               buf,
                               len,
                               100U);
        g_pending_reg_valid = 0U;
        return (st == HAL_OK) ? 0U : 1U;
    }

    if (len == 1U)
    {
        g_pending_reg_valid = 1U;
        g_pending_reg = buf[0];
        g_pending_addr = addr_7b;
        return 0U;
    }

    st = HAL_I2C_Master_Transmit(g_vl53_i2c, (uint16_t)(addr_7b << 1), buf, len, 100U);

    return (st == HAL_OK) ? 0U : 1U;
}

uint8_t i2c_read(uint8_t addr_7b, uint8_t *buf, uint8_t len)
{
    if (g_vl53_i2c == NULL || buf == NULL || len == 0U)
    {
        return 1U;
    }

    HAL_StatusTypeDef st;
    if (g_pending_reg_valid != 0U && g_pending_addr == addr_7b)
    {
        st = HAL_I2C_Mem_Read(g_vl53_i2c,
                              (uint16_t)(addr_7b << 1),
                              g_pending_reg,
                              I2C_MEMADD_SIZE_8BIT,
                              buf,
                              len,
                              100U);
        g_pending_reg_valid = 0U;
    }
    else
    {
        st = HAL_I2C_Master_Receive(g_vl53_i2c, (uint16_t)(addr_7b << 1), buf, len, 100U);
    }

    return (st == HAL_OK) ? 0U : 1U;
}
