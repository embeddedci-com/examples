/*
 * Host unit test for the coil-current motor-fault decision (motor_health.h), the
 * logic behind "rotors step ... verify" in main.c. It runs natively (no MCU/HAL)
 * so CI can prove the flat-line detection against the exact waveform we captured.
 *
 * Build & run:  make test-host   (or: cc -I.. tests/test_motor_health.c && ./a.out)
 *
 * Captured data (see the scope grab that motivated this feature):
 *   healthy motor: the coil-current sense on PA0 swings between roughly 1.30 V
 *                  (trough) and 2.05 V (peak) as it steps.
 *   faulted motor: the motor stops and the sense goes FLAT at ~1.70 V.
 * The check must call the first "motor ok" and the second "STEP FAULT".
 */
#include "motor_health.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

/* Volts -> 12-bit ADC counts at 3.3 V VREF+, matching the firmware's ADC mapping. */
static uint16_t v_to_counts(double volts)
{
    double c = (volts / 3.3) * (double)ADC_FULL_SCALE + 0.5;
    if (c < 0.0)
    {
        c = 0.0;
    }
    if (c > (double)ADC_FULL_SCALE)
    {
        c = (double)ADC_FULL_SCALE;
    }
    return (uint16_t)c;
}

int main(void)
{
    int failures = 0;

    /* 1) Healthy interval: ~1.30 V trough .. ~2.05 V peak (the swinging sine). */
    uint16_t healthy_min = v_to_counts(1.30);
    uint16_t healthy_max = v_to_counts(2.05);
    uint32_t healthy_vpp = motor_swing_vpp_mv(healthy_min, healthy_max);
    printf("healthy  min=1.30V max=2.05V -> Vpp=%u mV (threshold %u mV): %s\n",
           (unsigned)healthy_vpp, (unsigned)MOTOR_SWING_MIN_MV,
           motor_swing_is_fault(healthy_min, healthy_max) ? "FAULT" : "ok");
    if (motor_swing_is_fault(healthy_min, healthy_max))
    {
        printf("  FAIL: healthy swing was flagged as a motor fault\n");
        failures++;
    }

    /* 2) Faulted interval: the motor stopped, sense FLAT at ~1.70 V (min == max). */
    uint16_t flat = v_to_counts(1.70);
    uint32_t flat_vpp = motor_swing_vpp_mv(flat, flat);
    printf("faulted  flat 1.70V         -> Vpp=%u mV (threshold %u mV): %s\n",
           (unsigned)flat_vpp, (unsigned)MOTOR_SWING_MIN_MV,
           motor_swing_is_fault(flat, flat) ? "FAULT" : "ok");
    if (!motor_swing_is_fault(flat, flat))
    {
        printf("  FAIL: flat 1.70V line was NOT flagged as a motor fault\n");
        failures++;
    }

    /* 3) Faulted with realistic ADC noise: 1.70 V +/- ~10 mV must still be a fault
     *    (a stalled motor's residual noise must not read as a healthy swing). */
    uint16_t noisy_min = v_to_counts(1.69);
    uint16_t noisy_max = v_to_counts(1.71);
    uint32_t noisy_vpp = motor_swing_vpp_mv(noisy_min, noisy_max);
    printf("faulted  1.70V +/-10mV noise -> Vpp=%u mV (threshold %u mV): %s\n",
           (unsigned)noisy_vpp, (unsigned)MOTOR_SWING_MIN_MV,
           motor_swing_is_fault(noisy_min, noisy_max) ? "FAULT" : "ok");
    if (!motor_swing_is_fault(noisy_min, noisy_max))
    {
        printf("  FAIL: noisy flat line was NOT flagged as a motor fault\n");
        failures++;
    }

    if (failures == 0)
    {
        printf("\nPASS: coil-current fault detection matches the captured data\n");
        return 0;
    }
    printf("\n%d check(s) FAILED\n", failures);
    return 1;
}
