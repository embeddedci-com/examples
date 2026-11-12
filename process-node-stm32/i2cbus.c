#include "i2cbus.h"
#include "dwt.h"
#include "env.h"
#include "stm32f4xx_hal.h"

#define SCL_PIN GPIO_PIN_8
#define SDA_PIN GPIO_PIN_9

static uint32_t g_timeout_ms = I2CB_DEFAULT_TIMEOUT_MS;
static int g_stop_set; /* the read sequence already asked for its STOP */

void i2cb_set_timeout_ms(uint32_t ms)
{
    g_timeout_ms = ms;
}

uint32_t i2cb_timeout_ms(void)
{
    return g_timeout_ms;
}

const char *i2cb_status_name(i2cb_status_t s)
{
    switch (s)
    {
    case I2CB_OK: return "ok";
    case I2CB_NACK_ADDR: return "nack_addr";
    case I2CB_NACK_DATA: return "nack_data";
    case I2CB_BUS_BUSY: return "bus_busy";
    case I2CB_TIMEOUT: return "timeout";
    case I2CB_ARB_LOST: return "arb_lost";
    default: return "bus_error";
    }
}

/* Wait for any of the SR1 bits in mask. AF (a NACK), ARLO and BERR end the wait at once. */
static i2cb_status_t wait_sr1(uint32_t mask)
{
    uint32_t t0 = HAL_GetTick();
    for (;;)
    {
        uint32_t sr1 = I2C1->SR1;
        if (sr1 & I2C_SR1_AF)
        {
            I2C1->SR1 = ~I2C_SR1_AF; /* rc_w0 */
            return I2CB_NACK_DATA;   /* the caller tells address from data */
        }
        if (sr1 & I2C_SR1_ARLO)
        {
            I2C1->SR1 = ~I2C_SR1_ARLO;
            return I2CB_ARB_LOST;
        }
        if (sr1 & I2C_SR1_BERR)
        {
            I2C1->SR1 = ~I2C_SR1_BERR;
            return I2CB_BUS_ERROR;
        }
        if (sr1 & mask)
        {
            return I2CB_OK;
        }
        if ((HAL_GetTick() - t0) > g_timeout_ms)
        {
            return I2CB_TIMEOUT;
        }
    }
}

static void clear_addr(void)
{
    (void)I2C1->SR1;
    (void)I2C1->SR2;
}

static void stop_and_wait(void)
{
    if (!g_stop_set)
    {
        I2C1->CR1 |= I2C_CR1_STOP;
    }
    g_stop_set = 0;
    uint32_t t0 = HAL_GetTick();
    while ((I2C1->CR1 & I2C_CR1_STOP) && (HAL_GetTick() - t0) <= g_timeout_ms)
    {
    }
}

/* START (or a repeated START) and the address byte. */
static i2cb_status_t address(uint8_t addr_rw)
{
    I2C1->CR1 |= I2C_CR1_START;
    i2cb_status_t s = wait_sr1(I2C_SR1_SB);
    if (s != I2CB_OK)
    {
        return (s == I2CB_NACK_DATA) ? I2CB_BUS_ERROR : s;
    }
    I2C1->DR = addr_rw;
    s = wait_sr1(I2C_SR1_ADDR);
    return (s == I2CB_NACK_DATA) ? I2CB_NACK_ADDR : s;
}

static i2cb_status_t wait_idle(void)
{
    uint32_t t0 = HAL_GetTick();
    while (I2C1->SR2 & I2C_SR2_BUSY)
    {
        if ((HAL_GetTick() - t0) > g_timeout_ms)
        {
            return I2CB_BUS_BUSY;
        }
    }
    return I2CB_OK;
}

static i2cb_result_t write_phase(uint8_t addr, const uint8_t *w, uint16_t nw)
{
    i2cb_result_t res = {.status = I2CB_OK, .index = 0U, .phase = 'w'};
    res.status = address((uint8_t)(addr << 1));
    if (res.status != I2CB_OK)
    {
        return res;
    }
    clear_addr();
    for (uint16_t i = 0U; i < nw; i++)
    {
        res.status = wait_sr1(I2C_SR1_TXE);
        if (res.status != I2CB_OK)
        {
            res.index = (i > 0U) ? (uint16_t)(i - 1U) : 0U;
            return res;
        }
        I2C1->DR = w[i];
    }
    if (nw > 0U)
    {
        res.status = wait_sr1(I2C_SR1_BTF);
        res.index = (uint16_t)(nw - 1U);
    }
    return res;
}

/* The read sequences of RM0390 27.3.3 for 1, 2 and more bytes. The steps between clearing
 * ADDR (or BTF) and setting STOP are timing critical, so interrupts are off for them. */
static i2cb_result_t read_phase(uint8_t addr, uint8_t *r, uint16_t nr)
{
    i2cb_result_t res = {.status = I2CB_OK, .index = 0U, .phase = 'r'};
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR1 &= ~I2C_CR1_POS;
    res.status = address((uint8_t)((addr << 1) | 1U));
    if (res.status != I2CB_OK)
    {
        return res;
    }
    if (nr == 1U)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        __disable_irq();
        clear_addr();
        I2C1->CR1 |= I2C_CR1_STOP;
        g_stop_set = 1;
        __enable_irq();
        if ((res.status = wait_sr1(I2C_SR1_RXNE)) != I2CB_OK)
        {
            return res;
        }
        r[0] = (uint8_t)I2C1->DR;
        return res;
    }
    if (nr == 2U)
    {
        I2C1->CR1 &= ~I2C_CR1_ACK;
        I2C1->CR1 |= I2C_CR1_POS;
        __disable_irq();
        clear_addr();
        __enable_irq();
        if ((res.status = wait_sr1(I2C_SR1_BTF)) != I2CB_OK)
        {
            return res;
        }
        __disable_irq();
        I2C1->CR1 |= I2C_CR1_STOP;
        g_stop_set = 1;
        r[0] = (uint8_t)I2C1->DR;
        __enable_irq();
        r[1] = (uint8_t)I2C1->DR;
        I2C1->CR1 &= ~I2C_CR1_POS;
        return res;
    }
    clear_addr();
    uint16_t i = 0U;
    while ((uint16_t)(nr - i) > 3U)
    {
        if ((res.status = wait_sr1(I2C_SR1_RXNE)) != I2CB_OK)
        {
            return res;
        }
        r[i++] = (uint8_t)I2C1->DR;
    }
    /* Three left: N-2 in DR and N-1 in the shift register once BTF sets. */
    if ((res.status = wait_sr1(I2C_SR1_BTF)) != I2CB_OK)
    {
        return res;
    }
    I2C1->CR1 &= ~I2C_CR1_ACK;
    r[i++] = (uint8_t)I2C1->DR;
    if ((res.status = wait_sr1(I2C_SR1_BTF)) != I2CB_OK)
    {
        return res;
    }
    __disable_irq();
    I2C1->CR1 |= I2C_CR1_STOP;
    g_stop_set = 1;
    r[i++] = (uint8_t)I2C1->DR;
    __enable_irq();
    if ((res.status = wait_sr1(I2C_SR1_RXNE)) != I2CB_OK)
    {
        return res;
    }
    r[i] = (uint8_t)I2C1->DR;
    return res;
}

static void finish(i2cb_result_t *res)
{
    int clean = (res->status == I2CB_OK || res->status == I2CB_NACK_ADDR || res->status == I2CB_NACK_DATA);
    if (clean)
    {
        stop_and_wait(); /* after a read the STOP is already set: this only waits for it */
        I2C1->CR1 &= ~I2C_CR1_POS;
        clean = (I2C1->CR1 & I2C_CR1_STOP) == 0U; /* else the STOP never went out */
    }
    if (!clean)
    {
        /* timeouts, a lost arbitration, a bus error: start the peripheral over */
        g_stop_set = 0;
        env_i2c_reset();
    }
}

i2cb_result_t i2cb_xfer(uint8_t addr, const uint8_t *w, uint16_t nw, uint8_t *r, uint16_t nr,
                        int restart, uint32_t delay_ms)
{
    g_stop_set = 0;
    i2cb_result_t res = {.status = wait_idle(), .index = 0U, .phase = 'w'};
    if (res.status != I2CB_OK)
    {
        env_i2c_reset();
        return res;
    }
    if (nw > 0U || nr == 0U)
    {
        res = write_phase(addr, w, nw);
        if (res.status != I2CB_OK || nr == 0U)
        {
            finish(&res);
            return res;
        }
        if (!restart)
        {
            stop_and_wait();
            if (delay_ms > 0U)
            {
                HAL_Delay(delay_ms);
            }
            if ((res.status = wait_idle()) != I2CB_OK)
            {
                res.phase = 'r';
                env_i2c_reset();
                return res;
            }
        }
    }
    else if (delay_ms > 0U)
    {
        HAL_Delay(delay_ms);
    }
    res = read_phase(addr, r, nr);
    finish(&res);
    return res;
}

static void line_od(uint16_t pins)
{
    GPIO_InitTypeDef g = {0};
    g.Pin = pins;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);
}

int i2cb_recover(int *sda_before)
{
    __HAL_RCC_I2C1_CLK_DISABLE();
    GPIOB->BSRR = SCL_PIN | SDA_PIN; /* released */
    line_od(SCL_PIN | SDA_PIN);
    dwt_delay_us(10U);
    *sda_before = (GPIOB->IDR & SDA_PIN) ? 1 : 0;
    for (int i = 0; i < 9 && (GPIOB->IDR & SDA_PIN) == 0U; i++)
    {
        GPIOB->BSRR = (uint32_t)SCL_PIN << 16U;
        dwt_delay_us(5U);
        GPIOB->BSRR = SCL_PIN;
        dwt_delay_us(5U);
    }
    /* STOP: SDA low then high while SCL is high. */
    GPIOB->BSRR = (uint32_t)SCL_PIN << 16U;
    dwt_delay_us(5U);
    GPIOB->BSRR = (uint32_t)SDA_PIN << 16U;
    dwt_delay_us(5U);
    GPIOB->BSRR = SCL_PIN;
    dwt_delay_us(5U);
    GPIOB->BSRR = SDA_PIN;
    dwt_delay_us(10U);
    int ok = (GPIOB->IDR & SDA_PIN) ? 0 : -1;
    env_i2c_reset(); /* back to the peripheral (alternate function) */
    return ok;
}
