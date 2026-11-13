/*
 * Environment sensor: a BMP280 or BME280 on I2C1 (PB8 SCL, PB9 SDA; on the bench the pod
 * emulates it on LA1/LA2). Probed at 0x76 then 0x77, read every ENV_PERIOD_MS, and
 * re-probed every ENV_PROBE_MS while absent, so it can come and go at runtime.
 */
#ifndef ENV_H
#define ENV_H

#include <stdint.h>

#define ENV_PERIOD_MS 200U
#define ENV_PROBE_MS 1000U
#define ENV_FAILS_TO_LOSE 3U /* consecutive failed reads before the sensor counts as gone */

typedef enum
{
    ENV_ABSENT = 0, /* never seen since boot */
    ENV_OK,
    ENV_LOST,       /* was there, stopped answering */
} env_status_t;

typedef enum
{
    ENV_CHIP_NONE = 0,
    ENV_CHIP_BMP280, /* chip id 0x58 */
    ENV_CHIP_BME280, /* chip id 0x60: also humidity */
} env_chip_t;

typedef struct
{
    env_status_t status;
    env_chip_t chip;
    uint8_t addr;
    int32_t temp_dc;   /* deci-degC */
    uint32_t press_pa;
    int32_t hum_dpct;  /* BME280 only: tenths of a %RH */
    uint32_t reads;    /* successful readings since boot */
    uint32_t fails;
    uint32_t seq;      /* bumps on every new reading, so callers act once per sample */
} env_state_t;

void env_init(void);
/* env_due(): this pass reads (or probes) the sensor. */
int env_due(uint32_t now_ms);
void env_tick(uint32_t now_ms);
const env_state_t *env_state(void);

/* env off stops the driver (no probes, no reads) so a test owns the bus; env on starts it over
 * as at boot (probe, soft reset, setup). */
void env_set_enabled(int on);
int env_enabled(void);

/* The bus itself, shared with the generic transfers in i2cbus.c: reset the peripheral (after a
 * stuck bus or a timeout) and its clock (10 kHz .. 400 kHz; -1 when out of range). */
void env_i2c_reset(void);
int env_i2c_set_hz(uint32_t hz);
uint32_t env_i2c_hz(void);

#endif
