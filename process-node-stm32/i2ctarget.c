#include "i2ctarget.h"
#include "env.h"
#include "stm32f4xx_hal.h"

static i2ct_state_t g_t;
static uint8_t g_mem[I2CT_MEM_SIZE];
static volatile uint8_t g_first; /* the next received byte is the register pointer */

int i2ct_start(uint8_t addr)
{
    if (addr < 0x08U || addr > 0x77U)
    {
        return -1;
    }
    i2ct_stop();
    for (uint32_t i = 0; i < I2CT_MEM_SIZE; i++)
    {
        g_mem[i] = (uint8_t)i;
    }
    g_t = (i2ct_state_t){.on = 1U, .addr = addr};
    /* env_i2c_reset() left the peripheral set up as a master (FREQ, PE); a target only needs
     * its own address and ACK on top of that. OAR1 bit 14 must be kept at 1. */
    I2C1->OAR1 = (1U << 14) | ((uint32_t)addr << 1);
    I2C1->CR1 |= I2C_CR1_ACK;
    I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
    NVIC_SetPriority(I2C1_EV_IRQn, 1);
    NVIC_SetPriority(I2C1_ER_IRQn, 1);
    NVIC_EnableIRQ(I2C1_EV_IRQn);
    NVIC_EnableIRQ(I2C1_ER_IRQn);
    return 0;
}

void i2ct_stop(void)
{
    NVIC_DisableIRQ(I2C1_EV_IRQn);
    NVIC_DisableIRQ(I2C1_ER_IRQn);
    I2C1->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN);
    g_t.on = 0U;
    env_i2c_reset(); /* back to the master setup, own address 0 */
}

int i2ct_active(void)
{
    return g_t.on;
}

const i2ct_state_t *i2ct_state(void)
{
    return &g_t;
}

const uint8_t *i2ct_mem(void)
{
    return g_mem;
}

void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_ADDR)
    {
        (void)I2C1->SR2; /* SR1 then SR2 clears ADDR */
        g_t.matches++;
        g_first = 1U;
        I2C1->CR2 |= I2C_CR2_ITBUFEN; /* byte interrupts only while we are addressed */
        sr1 = I2C1->SR1;
    }
    if (sr1 & I2C_SR1_RXNE)
    {
        uint8_t b = (uint8_t)I2C1->DR;
        g_t.rx++;
        if (g_first)
        {
            g_t.ptr = b;
            g_first = 0U;
        }
        else
        {
            g_mem[g_t.ptr++] = b;
        }
    }
    else if ((sr1 & I2C_SR1_TXE) && (I2C1->SR2 & I2C_SR2_TRA))
    {
        I2C1->DR = g_mem[g_t.ptr++];
        g_t.tx++;
    }
    if (sr1 & I2C_SR1_STOPF)
    {
        I2C1->CR1 |= I2C_CR1_PE; /* SR1 read, then a CR1 write clears STOPF */
        I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
    }
}

void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    if (sr1 & I2C_SR1_AF)
    {
        /* The master NACKed the last byte it read: the end of a read, not an error. The byte
         * already loaded for the next read is dropped, so the pointer runs one past. */
        I2C1->SR1 = ~I2C_SR1_AF;
        I2C1->CR2 &= ~I2C_CR2_ITBUFEN;
    }
    if (sr1 & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR))
    {
        I2C1->SR1 = ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR);
        g_t.errors++;
    }
}
