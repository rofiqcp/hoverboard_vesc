#ifndef MOTOR_CURRENT_CAL_H
#define MOTOR_CURRENT_CAL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    MOTOR_CURRENT_CAL_IDLE = 0,
    MOTOR_CURRENT_CAL_SETTLING = 1,
    MOTOR_CURRENT_CAL_COLLECTING = 2,
    MOTOR_CURRENT_CAL_VALID = 3,
    MOTOR_CURRENT_CAL_FAILED = 4
} MotorCurrentCalState;

typedef struct {
    uint8_t state;
    uint8_t valid;
    uint8_t adc1_hw_cal_ok;
    uint8_t adc2_hw_cal_ok;
    uint16_t collected_samples;
    uint16_t target_samples;
    uint16_t raw[6];            /* instantaneous: rlA, rlB, rrB, rrC, dcl, dcr */
    uint16_t candidate_mean[6]; /* zero estimate even if validation fails */
    uint16_t offset[6];         /* applied zero; updated only after validation */
    uint16_t raw_span[6];       /* diagnostic peak-to-peak, NOT a pass/fail gate */
    uint16_t block_span[6];     /* span of 64-sample block means; stability gate */
    int16_t residual[6];        /* applied zero - instantaneous raw (may be released-domain) */
    uint16_t final_active_raw[6];
    int16_t final_active_residual[6]; /* applied zero - last raw before MOE release */
    uint16_t failure_mask;
    uint16_t generation;
    uint8_t sampling_mode;       /* 1 = active LOW-FET zero-vector (V11) */
    uint8_t bridge_active_mask;  /* physical MOE bits while snapshot was taken */
    uint8_t bridge_warmup_left;
    uint8_t bridge_warmup_right;
    uint16_t bridge_transition_left;
    uint16_t bridge_transition_right;
} MotorCurrentOffsetDebug;

typedef struct {
    uint8_t valid;
    uint8_t left;
    uint8_t fault_mask;
    uint8_t streak_at_trip;
    uint16_t generation;
    int16_t target_internal;
    int16_t id_internal;
    int16_t iq_internal;
    int16_t phase_current_1_delta;
    int16_t phase_current_2_delta;
    int16_t dc_delta;
    uint16_t adc_phase_1_raw;
    uint16_t adc_phase_2_raw;
    uint16_t adc_dc_raw;
    uint16_t offset_phase_1;
    uint16_t offset_phase_2;
    uint16_t offset_dc;
    uint16_t forced_phase_q16;
    int16_t duty_a;
    int16_t duty_b;
    int16_t duty_c;
    uint16_t duty_abs_q15;
} MotorCommissioningFaultSnapshot;

/* Safe request: accepted only while neither inverter nor commissioning override
 * is energized. Calibration is performed inside the regular current ADC ISR. */
bool MotorControl_RequestCurrentOffsetCalibration(void);
bool MotorControl_CurrentOffsetsValid(void);
uint8_t MotorControl_CurrentOffsetCalState(void);
void MotorControl_GetCurrentOffsetDebug(MotorCurrentOffsetDebug *out);
void MotorControl_SetAdcHardwareCalibrationResult(bool adc1_ok, bool adc2_ok);

/* Validated DC-link telemetry. The fast over-current protection still uses raw
 * ADC delta counts in motor.c. These getters expose a low-cost filtered current
 * only while the corresponding bridge is actually switching. When MOE is off,
 * VESC input/battery current is defined as 0 A; raw ADC evidence remains in HBTS. */
int16_t MotorControl_GetDcInputCentiAmp(bool left);
int16_t MotorControl_GetDcRawCentiAmp(bool left);
bool MotorControl_CurrentMeasurementValid(bool left);
bool MotorControl_BridgeActive(bool left);
uint8_t MotorControl_GetBridgeWarmupRemaining(bool left);
uint16_t MotorControl_GetBridgeTransitionCount(bool left);
uint16_t MotorControl_GetDcTelemetryRejects(bool left);
void MotorControl_ResetCurrentTelemetry(void);
void MotorControl_GetCommissioningFaultSnapshot(bool left, MotorCommissioningFaultSnapshot *out);
void MotorControl_ClearCommissioningFaultSnapshot(bool left);

#endif
