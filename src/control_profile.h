#ifndef CONTROL_PROFILE_H
#define CONTROL_PROFILE_H

/*
 * V18 timing profile
 * ==================
 * The ADC DMA / PWM cadence stays at the stock hoverboard rate (~16 kHz).
 * BOTH sensors are observed from every DMA sample so speed math uses the true
 * 16-kHz sample interval. The STM32F103 cannot reliably execute two complete
 * fixed-point FOC loops inside every 62.5-us slot (hardware V14/V16 logs showed
 * >4000-cycle ISR peaks), therefore current control is deterministically
 * interleaved: LEFT and RIGHT each run at 8 kHz. This keeps communication alive
 * and, unlike the old reactive shed path, gives speed and PI code one explicit
 * frequency contract.
 *
 * The stock hardware uses A2BIT_CONV = 50 ADC counts/A. The fixed-point FOC
 * expands phase current by x16, therefore 1 A = 800 internal units.
 */
#define CONTROL_PWM_FREQUENCY_HZ             16000U
#define CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ   16000U
#define CONTROL_FOC_MOTOR_FREQUENCY_HZ        8000U
#define CONTROL_CURRENT_ADC_COUNTS_PER_A      50U
#define CONTROL_CURRENT_INTERNAL_SHIFT         4U
#define CONTROL_CURRENT_INTERNAL_PER_A \
    (CONTROL_CURRENT_ADC_COUNTS_PER_A << CONTROL_CURRENT_INTERNAL_SHIFT)

#if CONTROL_PWM_FREQUENCY_HZ != CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ
#error "ADC sensor observation must stay synchronous with the 16-kHz PWM/DMA cadence"
#endif
#if CONTROL_FOC_MOTOR_FREQUENCY_HZ * 2U != CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ
#error "V18 dual-motor FOC contract requires deterministic LEFT/RIGHT 8-kHz interleave"
#endif

#endif
