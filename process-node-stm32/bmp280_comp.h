/*
 * BMP280 compensation, integer, straight from the Bosch datasheet (section 3.11.3,
 * the 32-bit temperature and 64-bit pressure versions). Pure, so tests/test_proc.c
 * checks it against the datasheet's worked example on the host.
 */
#ifndef BMP280_COMP_H
#define BMP280_COMP_H

#include <stdint.h>

typedef struct
{
    uint16_t t1;
    int16_t t2, t3;
    uint16_t p1;
    int16_t p2, p3, p4, p5, p6, p7, p8, p9;
} bmp280_calib_t;

/* The 24 calibration bytes from 0x88, little-endian. */
static inline void bmp280_calib_parse(const uint8_t b[24], bmp280_calib_t *c)
{
#define U16(i) ((uint16_t)(b[i] | (b[(i) + 1] << 8)))
    c->t1 = U16(0);
    c->t2 = (int16_t)U16(2);
    c->t3 = (int16_t)U16(4);
    c->p1 = U16(6);
    c->p2 = (int16_t)U16(8);
    c->p3 = (int16_t)U16(10);
    c->p4 = (int16_t)U16(12);
    c->p5 = (int16_t)U16(14);
    c->p6 = (int16_t)U16(16);
    c->p7 = (int16_t)U16(18);
    c->p8 = (int16_t)U16(20);
    c->p9 = (int16_t)U16(22);
#undef U16
}

/* Temperature in 0.01 degC; t_fine feeds the pressure compensation. */
static inline int32_t bmp280_temp_cdc(const bmp280_calib_t *c, int32_t adc_t, int32_t *t_fine)
{
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)c->t1 << 1))) * ((int32_t)c->t2)) >> 11;
    int32_t d = (adc_t >> 4) - (int32_t)c->t1;
    int32_t var2 = (((d * d) >> 12) * ((int32_t)c->t3)) >> 14;
    *t_fine = var1 + var2;
    return (*t_fine * 5 + 128) >> 8;
}

/* Pressure in Pa (rounded down), 0 when the calibration is unusable. */
static inline uint32_t bmp280_press_pa(const bmp280_calib_t *c, int32_t adc_p, int32_t t_fine)
{
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)c->p6;
    var2 = var2 + ((var1 * (int64_t)c->p5) * 131072);
    var2 = var2 + (((int64_t)c->p4) * 34359738368LL);
    var1 = ((var1 * var1 * (int64_t)c->p3) / 256) + ((var1 * (int64_t)c->p2) * 4096);
    var1 = ((((int64_t)1) * 140737488355328LL) + var1) * ((int64_t)c->p1) / 8589934592LL;
    if (var1 == 0)
    {
        return 0U;
    }
    int64_t p = 1048576 - adc_p;
    p = (((p * 2147483648LL) - var2) * 3125) / var1;
    var1 = (((int64_t)c->p9) * (p / 8192) * (p / 8192)) / 33554432;
    var2 = (((int64_t)c->p8) * p) / 524288;
    p = ((p + var1 + var2) / 256) + (((int64_t)c->p7) * 16);
    return (uint32_t)(p / 256);
}

#endif
