/*
 * Environment sensor: a BMP280 on I2C1 (PB8 SCL, PB9 SDA; on the bench the pod
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

typedef struct
{
    env_status_t status;
    uint8_t addr;
    int32_t temp_dc;   /* deci-degC */
    uint32_t press_pa;
    uint32_t reads;    /* successful readings since boot */
    uint32_t fails;
    uint32_t seq;      /* bumps on every new reading, so callers act once per sample */
} env_state_t;

void env_init(void);
void env_tick(uint32_t now_ms);
const env_state_t *env_state(void);

#endif
