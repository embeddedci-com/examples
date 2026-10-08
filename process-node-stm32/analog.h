/*
 * Analog I/O of the process node.
 *   in:  PA1 (ADC1_IN1, Nucleo A1), the process value. Oversampled and
 *        corrected for VDDA through VREFINT.
 *   out: PA4 (DAC1_OUT1, Nucleo A2), the actuator. Off (high-Z) until asked:
 *        the pod's ADC SMA it drives is shared with other tests.
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

#define AOUT_RATE_HZ 10000U  /* waveform update rate (TIM6) */
#define AOUT_MIN_MV 200U     /* buffered DAC output swing: 0.2 V .. VDDA - 0.2 V */
#define AOUT_HEADROOM_MV 200U
#define AOUT_SINE_MAX_HZ 1000U

typedef enum
{
    AOUT_OFF = 0,
    AOUT_DC,
    AOUT_SINE,
} aout_mode_t;

typedef struct
{
    aout_mode_t mode;
    uint32_t mv;        /* DC level, or the sine offset */
    uint32_t amp_mv;    /* sine amplitude (peak) */
    uint32_t hz;
    uint32_t code;      /* DC code, or the offset code */
} aout_state_t;

void aout_init(void);
/* Each returns 0, or -1 for a request outside the output's range (nothing changes). */
int aout_dc(uint32_t mv);
int aout_sine(uint32_t hz, uint32_t amp_mv, uint32_t offset_mv);
void aout_off(void);
const aout_state_t *aout_state(void);

#endif
