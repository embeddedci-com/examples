/*
 * Analog input of the process node: PA1 (ADC1_IN1, Nucleo A1), the process
 * value. Readings are oversampled and corrected for VDDA through VREFINT.
 */
#ifndef ANALOG_H
#define ANALOG_H

#include <stdint.h>

#define AIN_OVERSAMPLE 16U
#define AIN_PERIOD_MS 10U      /* background sampling rate: 100 Hz */
#define AIN_FILTER_SHIFT 5U    /* ~32 samples = ~320 ms time constant */
#define AIN_VREF_PERIOD_MS 1000U

typedef struct
{
    uint32_t raw;      /* last oversampled count, 0..4095 */
    uint32_t mv;       /* last reading, mV */
    uint32_t filt_mv;  /* low-passed process value, mV */
    uint32_t vdda_mv;  /* VDDA from VREFINT */
    uint32_t samples;  /* background readings since boot */
} ain_state_t;

void ain_init(void);
/* Call every main-loop pass; samples on its own schedule. */
void ain_tick(uint32_t now_ms);
/* Take one fresh oversampled reading now (also updates the state). */
const ain_state_t *ain_read_now(void);
const ain_state_t *ain_state(void);

#endif
