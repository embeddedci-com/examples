/*
 * motor_health.h — pure coil-current fault decision, shared by the firmware
 * ("rotors step ... verify" in main.c) and the host unit test
 * (tests/test_motor_health.c). No HAL / MCU dependencies so it compiles and runs
 * natively, which lets us prove the flat-line detection against real captured data.
 *
 * The signal is the motor coil-current sense on PA0 (shunt -> INA amplifier),
 * read as ADC1_IN0 (12-bit against the 3.3 V VREF+). A running motor makes the
 * reading swing step-to-step; a stalled / open coil (or dead driver) sits flat.
 * We judge health by the peak-to-peak swing over a verify interval, not by an
 * absolute level.
 */
#ifndef MOTOR_HEALTH_H
#define MOTOR_HEALTH_H

#include <stdint.h>

#define ADC_VREF_MV 3300U
#define ADC_FULL_SCALE 4095U
/* Below this peak-to-peak swing (mV) over a verify interval the coil current is
 * "flat" -> motor fault. A healthy step train swings ~700 mV pk-pk on this bench;
 * a stalled/open coil sits within a few tens of mV of noise, so 250 mV separates
 * the two with wide margin. */
#define MOTOR_SWING_MIN_MV 250U

/* 12-bit ADC count -> millivolts at VREF+. */
static inline uint32_t adc_counts_to_mv(uint32_t counts)
{
    return (counts * ADC_VREF_MV) / ADC_FULL_SCALE;
}

/* Peak-to-peak swing (mV) of the coil-current reading over an interval, given the
 * min and max ADC counts seen. Returns 0 if the pair is degenerate (max < min). */
static inline uint32_t motor_swing_vpp_mv(uint16_t adc_min, uint16_t adc_max)
{
    if (adc_max < adc_min)
    {
        return 0U;
    }
    return adc_counts_to_mv(adc_max) - adc_counts_to_mv(adc_min);
}

/* Non-zero when the interval's swing is too flat to be a running motor. */
static inline int motor_swing_is_fault(uint16_t adc_min, uint16_t adc_max)
{
    return motor_swing_vpp_mv(adc_min, adc_max) < MOTOR_SWING_MIN_MV;
}

#endif /* MOTOR_HEALTH_H */
