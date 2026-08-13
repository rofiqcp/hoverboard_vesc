/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-point STM32F103 adaptation of the VESC-style FOC architecture.
 * VESC reference: Copyright 2016-2022 Benjamin Vedder.
 * Board firmware lineage: Copyright 2019-2020 Emanuel FERU.
 * See NOTICE.md and COPYING.
 */

/*
 * Motor current-control ISR
 * =========================
 * Hot-path order is intentionally VESC-like:
 *   ADC current -> selected sensor backend -> electrical phase -> FOC current PI
 *   -> inverse Park -> six-sector SVPWM -> timer compare.
 *
 * Slow functions (speed/position PID, EEPROM, commissioning validation, protocol)
 * never execute here.
 */

#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "runtime_control.h"
#include "motor_sensor.h"
#include "left_encoder.h"
#include "foc_motor.h"
#include "motor_current_cal.h"

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>
#include <stddef.h>

static const uint16_t pwm_res = 64000000U / 2U / PWM_FREQ; /* 2000 @ 16 kHz */
static const int16_t curDC_max = (I_DC_MAX * A2BIT_CONV);
static int16_t pwm_margin = 110;

int16_t curL_phaA = 0;
int16_t curL_phaB = 0;
int16_t curL_DC = 0;
int16_t curR_phaB = 0;
int16_t curR_phaC = 0;
int16_t curR_DC = 0;

/* Kept for protocol/runtime compatibility: command ownership remains in slow loop. */
volatile int pwml = 0;
volatile int pwmr = 0;

extern volatile adc_buf_t adc_buffer;

uint8_t buzzerFreq = 0;
uint8_t buzzerPattern = 0;
uint8_t buzzerCount = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t buzzerPrev = 0;
static uint8_t buzzerIdx = 0;
static uint32_t buzzerWindowTicks = 0U;
static uint16_t buzzerToneTicks = 0U;
static bool buzzerWindowActive = false;
static uint16_t batteryFilterTicks = 1000U;

uint8_t enable = 0U;

volatile uint16_t motorControlIsrOverrunCount = 0U;
volatile uint8_t motorControlIsrOverrunFaultMask = 0U;
volatile uint8_t motorControlOvercurrentFaultMask = 0U;
volatile uint32_t motorControlIsrLastCycles = 0U;
volatile uint32_t motorControlIsrMaxCycles = 0U;
volatile uint32_t motorControlIsrDeadlineCycles = 4000U; /* refreshed from SystemCoreClock */
/* V15: deadline is diagnostic only. Hardware gating is sample-local, V1-style. */
static uint8_t motorIsrOverrunStreak = 0U;
/* V18: both sensors are sampled on every 16-kHz ADC DMA event while FOC is
 * deterministically interleaved LEFT/RIGHT at 8 kHz per motor. The old V16
 * overload shed remains only as an emergency CPU-liveness fallback; even that
 * fallback never skips sensor observation, clears ARM, or latches a fault. */
static bool motorIsrShedNext = false;
static bool motorFocSlotRight = false;
/* Retained only for legacy diagnostic clear API; ISR no longer increments these. */
static uint8_t motorOvercurrentStreakLeft = 0U;
static uint8_t motorOvercurrentStreakRight = 0U;

/* V7 validated DC-link telemetry. Protection keeps using raw curL_DC/curR_DC
 * every 16 kHz sample. Telemetry is filtered only while the bridge is really
 * switching; with MOE off it is forced to 0 A because the DC-link shunt signal
 * is not a valid proxy for battery current during passive back-EMF motion. */
#define DC_TELEM_FILTER_SHIFT 3U
#define DC_TELEM_SANITY_CENTI_AMP 4000
static volatile int32_t dcTelemLeftQ4 = 0;
static volatile int32_t dcTelemRightQ4 = 0;
static volatile int16_t dcRawLeftCentiAmp = 0;
static volatile int16_t dcRawRightCentiAmp = 0;
static volatile uint16_t dcTelemRejectLeft = 0U;
static volatile uint16_t dcTelemRejectRight = 0U;
static volatile bool currentMeasurementValidLeft = false;
static volatile bool currentMeasurementValidRight = false;

/* V15 keeps exactly one V1-compatible domain-prime transition. The ADC sample
 * that triggered an OFF->ON request was captured while MOE was still OFF, so it
 * cannot be used with active-domain offsets. Assert the calibrated low-FET zero
 * vector once; the NEXT DMA sample is already in the valid active domain and can
 * enter the PI. No multi-sample warm-up state is used. */
#define BRIDGE_CURRENT_WARMUP_ADC_SAMPLES 1U
static volatile uint8_t bridgeWarmupLeft = 0U;
static volatile uint8_t bridgeWarmupRight = 0U;
static volatile uint16_t bridgeTransitionsLeft = 0U;
static volatile uint16_t bridgeTransitionsRight = 0U;

/* OPEN phase accumulator: upper 16 bit = one-turn Q16 phase. */
static uint32_t openPhaseLeftQ16 = 0U;
static uint32_t openPhaseRightQ16 = 0U;

static inline int16_t dc_counts_to_centi_amp(int16_t delta_counts)
{
#if CONTROL_CURRENT_ADC_COUNTS_PER_A == 50U
    /* left_dc_curr legacy sign was -(offset-raw)*100/50 = -2*delta. */
    int32_t v = -(int32_t)delta_counts * 2;
#else
    int32_t v = -((int32_t)delta_counts * 100) / (int32_t)CONTROL_CURRENT_ADC_COUNTS_PER_A;
#endif
    if (v > INT16_MAX) v = INT16_MAX;
    if (v < INT16_MIN) v = INT16_MIN;
    return (int16_t)v;
}

static inline void update_dc_telemetry_one(bool active, int16_t sample_centi_amp,
                                           volatile int32_t *filter_q4,
                                           volatile uint16_t *rejects,
                                           volatile bool *valid)
{
    if (!active) {
        *filter_q4 = 0;
        *valid = false;
        return;
    }
    int32_t a = sample_centi_amp;
    if (a < 0) a = -a;
    if (a > DC_TELEM_SANITY_CENTI_AMP) {
        if (*rejects != UINT16_MAX) ++(*rejects);
        *filter_q4 = 0;
        *valid = false;
        return;
    }
    const int32_t target_q4 = (int32_t)sample_centi_amp << 4;
    *filter_q4 += (target_q4 - *filter_q4) >> DC_TELEM_FILTER_SHIFT;
    *valid = true;
}

int16_t MotorControl_GetDcInputCentiAmp(bool left)
{
    const bool valid = left ? currentMeasurementValidLeft : currentMeasurementValidRight;
    if (!valid) return 0;
    const int32_t q4 = left ? dcTelemLeftQ4 : dcTelemRightQ4;
    int32_t v = (q4 >= 0) ? ((q4 + 8) >> 4) : -(((-q4) + 8) >> 4);
    if (v > INT16_MAX) v = INT16_MAX;
    if (v < INT16_MIN) v = INT16_MIN;
    return (int16_t)v;
}

int16_t MotorControl_GetDcRawCentiAmp(bool left)
{
    return left ? dcRawLeftCentiAmp : dcRawRightCentiAmp;
}

bool MotorControl_CurrentMeasurementValid(bool left)
{
    return left ? currentMeasurementValidLeft : currentMeasurementValidRight;
}

bool MotorControl_BridgeActive(bool left)
{
    const TIM_TypeDef *tim = left ? LEFT_TIM : RIGHT_TIM;
    return (tim->BDTR & TIM_BDTR_MOE) != 0U;
}

uint16_t MotorControl_GetDcTelemetryRejects(bool left)
{
    return left ? dcTelemRejectLeft : dcTelemRejectRight;
}

void MotorControl_ResetCurrentTelemetry(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    dcTelemLeftQ4 = 0; dcTelemRightQ4 = 0;
    dcRawLeftCentiAmp = 0; dcRawRightCentiAmp = 0;
    currentMeasurementValidLeft = false; currentMeasurementValidRight = false;
    if (primask == 0U) __enable_irq();
}

static inline void set_current_zero_vector_left(void)
{
    /* Matches the stock hoverboard startup state: PWM1 CCR=0 on all phases,
     * complementary outputs enabled. This is zero line-to-line voltage while
     * providing the LOW-FET sampling domain expected by the shunts. */
    LEFT_TIM->LEFT_TIM_U = 0U;
    LEFT_TIM->LEFT_TIM_V = 0U;
    LEFT_TIM->LEFT_TIM_W = 0U;
}

static inline void set_current_zero_vector_right(void)
{
    RIGHT_TIM->RIGHT_TIM_U = 0U;
    RIGHT_TIM->RIGHT_TIM_V = 0U;
    RIGHT_TIM->RIGHT_TIM_W = 0U;
}

static inline void bridge_release_left(void)
{
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    bridgeWarmupLeft = 0U;
}

static inline void bridge_release_right(void)
{
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    bridgeWarmupRight = 0U;
}

uint8_t MotorControl_GetBridgeWarmupRemaining(bool left)
{
    return left ? bridgeWarmupLeft : bridgeWarmupRight;
}

uint16_t MotorControl_GetBridgeTransitionCount(bool left)
{
    return left ? bridgeTransitionsLeft : bridgeTransitionsRight;
}

/* Current ADC zero calibration. Do not learn offsets until main() explicitly
 * requests it after BOTH ADCs have been initialized and hardware-calibrated.
 * The old recursive (sample+offset)/2 scheme could freeze a startup transient as
 * the zero reference. The robust policy discards a settling window, averages 2048 samples and validates 64-sample block means. */
#define CURRENT_CAL_SETTLE_SAMPLES 512U
#define CURRENT_CAL_COLLECT_SAMPLES 2048U
#define CURRENT_CAL_BLOCK_SAMPLES 64U
#define CURRENT_CAL_MAX_BLOCK_MEAN_SPAN_COUNTS 64U
#define CURRENT_CAL_MIN_MEAN_COUNTS 64U
#define CURRENT_CAL_MAX_MEAN_COUNTS 4031U

/* failure_mask layout:
 * bits 0..5   : candidate mean out of ADC range for rlA,rlB,rrB,rrC,dcl,dcr
 * bits 6..11  : 64-sample block means unstable for the same six channels
 * bit 12      : ADC1 hardware self-calibration failed
 * bit 13      : ADC2 hardware self-calibration failed
 * bit 14      : no complete calibration blocks were collected
 * bit 15      : wheel motion detected while active-zero-vector calibration ran
 *
 * Raw single-sample peak-to-peak is diagnostic only. One switching/EMI spike
 * must not discard an otherwise stable zero-current mean. */

typedef struct {
    volatile uint8_t state;
    volatile uint8_t request_pending;
    volatile uint8_t adc1_hw_cal_ok;
    volatile uint8_t adc2_hw_cal_ok;
    volatile uint16_t settle_left;
    volatile uint16_t sample_count;
    volatile uint16_t generation;
    uint32_t sum[6];
    uint16_t minv[6];
    uint16_t maxv[6];
    uint32_t block_sum[6];
    uint16_t block_min_mean[6];
    uint16_t block_max_mean[6];
    uint16_t candidate_mean[6];
    uint16_t offset[6];
    uint16_t final_active_raw[6];
    uint16_t block_sample_count;
    uint16_t block_count;
    uint16_t failure_mask;
} CurrentOffsetRuntime;

static CurrentOffsetRuntime currentCal = {
    .state = MOTOR_CURRENT_CAL_IDLE,
    .request_pending = 0U,
    .adc1_hw_cal_ok = 0U,
    .adc2_hw_cal_ok = 0U
};

static uint16_t offsetrlA = 2000U;
static uint16_t offsetrlB = 2000U;
static uint16_t offsetrrB = 2000U;
static uint16_t offsetrrC = 2000U;
static uint16_t offsetdcl = 2000U;
static uint16_t offsetdcr = 2000U;

/* V8 commissioning guard: detect-current is allowed to use only a tightly
 * bounded phase-current envelope. A bad polarity/sample window must not be able
 * to drive tens of amps for hundreds of milliseconds before the slow loop sees
 * it. The guard is independent from the runtime DC-link overcurrent latch. */
static uint8_t commissioningFastStreakLeft = 0U;
static uint8_t commissioningFastStreakRight = 0U;
static volatile MotorCommissioningFaultSnapshot commissioningFaultLeft = {0};
static volatile MotorCommissioningFaultSnapshot commissioningFaultRight = {0};

void MotorControl_ClearCommissioningFaultSnapshot(bool left)
{
    volatile MotorCommissioningFaultSnapshot *s = left ? &commissioningFaultLeft : &commissioningFaultRight;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint16_t next_generation = (uint16_t)(s->generation + 1U);
    *s = (MotorCommissioningFaultSnapshot){0};
    s->generation = next_generation;
    if (primask == 0U) __enable_irq();
}

void MotorControl_GetCommissioningFaultSnapshot(bool left, MotorCommissioningFaultSnapshot *out)
{
    if (out == NULL) return;
    const volatile MotorCommissioningFaultSnapshot *s = left ? &commissioningFaultLeft : &commissioningFaultRight;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = *(const MotorCommissioningFaultSnapshot *)s;
    if (primask == 0U) __enable_irq();
}

/* V18: the obsolete commissioning_current_guard was removed. Hardware DC-link
 * chopping remains sample-local; commissioning current is regulated by the same
 * FOC PI and board current limit instead of a second sticky guard. */

int16_t batVoltage = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt =
    (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;

int32_t odom_l = 0;
int32_t odom_r = 0;

uint8_t MotorControl_GetOverrunStreakPct(void)
{
    const uint32_t pct = ((uint32_t)motorIsrOverrunStreak * 100U) / MOTOR_ISR_OVERRUN_LIMIT;
    return pct > 100U ? 100U : (uint8_t)pct;
}

void MotorControl_ClearIsrOverrunFault(uint8_t clear_mask)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motorControlIsrOverrunFaultMask &= (uint8_t)~(clear_mask & 0x03U);
    if (motorControlIsrOverrunFaultMask == 0U) {
        motorControlIsrOverrunCount = 0U;
        motorIsrOverrunStreak = 0U;
        DMA1->IFCR = DMA_IFCR_CTCIF1;
    }
    if (primask == 0U) __enable_irq();
}


void MotorControl_ClearOvercurrentFault(uint8_t clear_mask)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motorControlOvercurrentFaultMask &= (uint8_t)~(clear_mask & 0x03U);
    if ((clear_mask & 0x01U) != 0U) motorOvercurrentStreakLeft = 0U;
    if ((clear_mask & 0x02U) != 0U) motorOvercurrentStreakRight = 0U;
    if (primask == 0U) __enable_irq();
}

static void service_buzzer_isr(void)
{
    ++buzzerTimer;

    if (buzzerFreq == 0U) {
        buzzerWindowTicks = 0U;
        buzzerToneTicks = 0U;
        buzzerWindowActive = false;
        if (buzzerPrev != 0U) {
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            buzzerPrev = 0U;
        }
        return;
    }

    if (buzzerWindowTicks == 0U) {
        buzzerWindowActive = !buzzerWindowActive;
        if (!buzzerWindowActive && buzzerPattern == 0U) buzzerWindowActive = true;

        if (buzzerWindowActive) {
            buzzerWindowTicks = 5000U;
            if (buzzerPrev == 0U) {
                buzzerPrev = 1U;
                if (++buzzerIdx > (uint8_t)(buzzerCount + 2U)) buzzerIdx = 1U;
            }
        } else {
            buzzerWindowTicks = 5000U * (uint32_t)buzzerPattern;
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
            buzzerPrev = 0U;
        }
    }

    if (buzzerWindowActive) {
        if (buzzerToneTicks == 0U) {
            buzzerToneTicks = (uint16_t)(buzzerFreq - 1U);
            if (buzzerIdx <= buzzerCount || buzzerCount == 0U) {
                HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
            }
        } else {
            --buzzerToneTicks;
        }
    }
    --buzzerWindowTicks;
}

static void current_cal_reset_stats(void)
{
    currentCal.sample_count = 0U;
    currentCal.block_sample_count = 0U;
    currentCal.block_count = 0U;
    currentCal.failure_mask = 0U;
    for (uint8_t n = 0U; n < 6U; ++n) {
        currentCal.sum[n] = 0U;
        currentCal.minv[n] = UINT16_MAX;
        currentCal.maxv[n] = 0U;
        currentCal.block_sum[n] = 0U;
        currentCal.block_min_mean[n] = UINT16_MAX;
        currentCal.block_max_mean[n] = 0U;
        currentCal.candidate_mean[n] = 0U;
        currentCal.final_active_raw[n] = 0U;
    }
}

static void current_cal_raw(uint16_t raw[6])
{
    raw[0] = adc_buffer.rlA;
    raw[1] = adc_buffer.rlB;
    raw[2] = adc_buffer.rrB;
    raw[3] = adc_buffer.rrC;
    raw[4] = adc_buffer.dcl;
    raw[5] = adc_buffer.dcr;
}

void MotorControl_SetAdcHardwareCalibrationResult(bool adc1_ok, bool adc2_ok)
{
    currentCal.adc1_hw_cal_ok = adc1_ok ? 1U : 0U;
    currentCal.adc2_hw_cal_ok = adc2_ok ? 1U : 0U;
}

bool MotorControl_RequestCurrentOffsetCalibration(void)
{
    /* V11 calibration intentionally turns on all LOW FETs (zero vector), matching
     * the original hoverboard current-offset domain. Never do that to a wheel that
     * is already rotating. At boot the observation samples are zero-initialized;
     * subsequent ISR observations continuously enforce the same guard. */
    if (runtimeMotorEnableMask != 0U || sensorCalibrationOpenLoopMask != 0U) return false;
    int32_t sl = motorSensorSampleLeft.mechanical_speed_q4; if (sl < 0) sl = -sl;
    int32_t sr = motorSensorSampleRight.mechanical_speed_q4; if (sr < 0) sr = -sr;
    if (sl > (5 * 16) || sr > (5 * 16)) return false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    currentCal.request_pending = 1U;
    if (primask == 0U) __enable_irq();
    return true;
}

bool MotorControl_CurrentOffsetsValid(void)
{
    return currentCal.state == MOTOR_CURRENT_CAL_VALID;
}

uint8_t MotorControl_CurrentOffsetCalState(void)
{
    return currentCal.state;
}

void MotorControl_GetCurrentOffsetDebug(MotorCurrentOffsetDebug *out)
{
    if (out == NULL) return;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t raw[6];
    current_cal_raw(raw);
    out->state = currentCal.state;
    out->valid = currentCal.state == MOTOR_CURRENT_CAL_VALID ? 1U : 0U;
    out->adc1_hw_cal_ok = currentCal.adc1_hw_cal_ok;
    out->adc2_hw_cal_ok = currentCal.adc2_hw_cal_ok;
    out->collected_samples = currentCal.sample_count;
    out->target_samples = CURRENT_CAL_COLLECT_SAMPLES;
    out->failure_mask = currentCal.failure_mask;
    out->generation = currentCal.generation;
    out->sampling_mode = 1U; /* active LOW-FET zero-vector, V11 */
    out->bridge_active_mask =
        (uint8_t)(((LEFT_TIM->BDTR & TIM_BDTR_MOE) ? 0x01U : 0U) |
                  ((RIGHT_TIM->BDTR & TIM_BDTR_MOE) ? 0x02U : 0U));
    out->bridge_warmup_left = bridgeWarmupLeft;
    out->bridge_warmup_right = bridgeWarmupRight;
    out->bridge_transition_left = bridgeTransitionsLeft;
    out->bridge_transition_right = bridgeTransitionsRight;
    for (uint8_t n = 0U; n < 6U; ++n) {
        out->raw[n] = raw[n];
        out->candidate_mean[n] = currentCal.candidate_mean[n];
        out->offset[n] = currentCal.offset[n];
        out->raw_span[n] =
            (currentCal.maxv[n] >= currentCal.minv[n] && currentCal.minv[n] != UINT16_MAX)
                ? (uint16_t)(currentCal.maxv[n] - currentCal.minv[n]) : 0U;
        out->block_span[n] =
            (currentCal.block_max_mean[n] >= currentCal.block_min_mean[n] &&
             currentCal.block_min_mean[n] != UINT16_MAX)
                ? (uint16_t)(currentCal.block_max_mean[n] - currentCal.block_min_mean[n]) : 0U;
        const uint16_t zero = (currentCal.state == MOTOR_CURRENT_CAL_VALID)
            ? currentCal.offset[n] : currentCal.candidate_mean[n];
        out->residual[n] = (int16_t)((int32_t)zero - (int32_t)raw[n]);
        out->final_active_raw[n] = currentCal.final_active_raw[n];
        out->final_active_residual[n] = (int16_t)((int32_t)currentCal.offset[n] -
                                                   (int32_t)currentCal.final_active_raw[n]);
    }
    if (primask == 0U) __enable_irq();
}

static void current_cal_begin_isr(void)
{
    currentCal.request_pending = 0U;
    currentCal.state = MOTOR_CURRENT_CAL_SETTLING;
    currentCal.settle_left = CURRENT_CAL_SETTLE_SAMPLES;
    current_cal_reset_stats();

    /* CRITICAL V11 FIX: the original hoverboard firmware learns phase-current
     * offset after PWM channels are started with Pulse=0. That means MOE is on
     * and all three LOW FETs establish the ADC common-mode used during runtime.
     * V10 learned offset with MOE off, which shifted the phase ADC baseline by
     * ~700 counts on the user's board and created a false ~14 A measurement. */
    set_current_zero_vector_left();
    set_current_zero_vector_right();
    bridgeWarmupLeft = 0U;
    bridgeWarmupRight = 0U;
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
}

static bool current_cal_service_isr(void)
{
    if (currentCal.request_pending != 0U) current_cal_begin_isr();
    if (currentCal.state == MOTOR_CURRENT_CAL_VALID) return true;
    if (currentCal.state == MOTOR_CURRENT_CAL_IDLE || currentCal.state == MOTOR_CURRENT_CAL_FAILED) return false;

    /* Keep the exact electrical domain constant for every calibration sample. */
    set_current_zero_vector_left();
    set_current_zero_vector_right();
    LEFT_TIM->BDTR |= TIM_BDTR_MOE;
    RIGHT_TIM->BDTR |= TIM_BDTR_MOE;

    int32_t sl = motorSensorSampleLeft.mechanical_speed_q4; if (sl < 0) sl = -sl;
    int32_t sr = motorSensorSampleRight.mechanical_speed_q4; if (sr < 0) sr = -sr;
    if (sl > (5 * 16) || sr > (5 * 16)) {
        currentCal.failure_mask |= (uint16_t)(1U << 15);
        currentCal.state = MOTOR_CURRENT_CAL_FAILED;
        bridge_release_left();
        bridge_release_right();
        return false;
    }

    if (currentCal.state == MOTOR_CURRENT_CAL_SETTLING) {
        if (currentCal.settle_left > 0U) --currentCal.settle_left;
        if (currentCal.settle_left == 0U) {
            current_cal_reset_stats();
            currentCal.state = MOTOR_CURRENT_CAL_COLLECTING;
        }
        return false;
    }

    uint16_t raw[6];
    current_cal_raw(raw);
    for (uint8_t n = 0U; n < 6U; ++n) {
        currentCal.sum[n] += raw[n];
        currentCal.block_sum[n] += raw[n];
        if (raw[n] < currentCal.minv[n]) currentCal.minv[n] = raw[n];
        if (raw[n] > currentCal.maxv[n]) currentCal.maxv[n] = raw[n];
    }
    ++currentCal.sample_count;
    ++currentCal.block_sample_count;

    if (currentCal.block_sample_count >= CURRENT_CAL_BLOCK_SAMPLES) {
        for (uint8_t n = 0U; n < 6U; ++n) {
            const uint16_t block_mean =
                (uint16_t)((currentCal.block_sum[n] + (CURRENT_CAL_BLOCK_SAMPLES / 2U)) /
                           CURRENT_CAL_BLOCK_SAMPLES);
            if (block_mean < currentCal.block_min_mean[n]) currentCal.block_min_mean[n] = block_mean;
            if (block_mean > currentCal.block_max_mean[n]) currentCal.block_max_mean[n] = block_mean;
            currentCal.block_sum[n] = 0U;
        }
        currentCal.block_sample_count = 0U;
        if (currentCal.block_count != UINT16_MAX) ++currentCal.block_count;
    }

    if (currentCal.sample_count < CURRENT_CAL_COLLECT_SAMPLES) return false;

    uint16_t fail = 0U;
    for (uint8_t n = 0U; n < 6U; ++n) {
        const uint16_t mean =
            (uint16_t)((currentCal.sum[n] + (CURRENT_CAL_COLLECT_SAMPLES / 2U)) /
                       CURRENT_CAL_COLLECT_SAMPLES);
        currentCal.candidate_mean[n] = mean;
        if (mean < CURRENT_CAL_MIN_MEAN_COUNTS || mean > CURRENT_CAL_MAX_MEAN_COUNTS) {
            fail |= (uint16_t)(1U << n);
        }
        if (currentCal.block_count > 0U &&
            currentCal.block_min_mean[n] != UINT16_MAX &&
            (uint16_t)(currentCal.block_max_mean[n] - currentCal.block_min_mean[n]) >
                CURRENT_CAL_MAX_BLOCK_MEAN_SPAN_COUNTS) {
            fail |= (uint16_t)(1U << (n + 6U));
        }
    }
    if (currentCal.block_count == 0U) fail |= (uint16_t)(1U << 14);
    /* Hardware self-calibration failure is diagnostic and also blocks power. */
    if (currentCal.adc1_hw_cal_ok == 0U) fail |= (uint16_t)(1U << 12);
    if (currentCal.adc2_hw_cal_ok == 0U) fail |= (uint16_t)(1U << 13);
    currentCal.failure_mask = fail;
    if (fail != 0U) {
        currentCal.state = MOTOR_CURRENT_CAL_FAILED;
        bridge_release_left();
        bridge_release_right();
        return false;
    }

    for (uint8_t n = 0U; n < 6U; ++n) currentCal.offset[n] = currentCal.candidate_mean[n];
    offsetrlA = currentCal.candidate_mean[0];
    offsetrlB = currentCal.candidate_mean[1];
    offsetrrB = currentCal.candidate_mean[2];
    offsetrrC = currentCal.candidate_mean[3];
    offsetdcl = currentCal.candidate_mean[4];
    offsetdcr = currentCal.candidate_mean[5];
    /* Preserve the LAST raw sample while MOE is still in the calibration domain.
     * Once the bridge is released the phase ADC common-mode legitimately moves,
     * so instantaneous residual is not evidence of a bad offset anymore. */
    current_cal_raw(currentCal.final_active_raw);
    if (currentCal.generation != UINT16_MAX) ++currentCal.generation;
    currentCal.state = MOTOR_CURRENT_CAL_VALID;
    /* Calibration is complete; release both bridges. The next real command must
     * deliberately traverse the current-domain warm-up before FOC can actuate. */
    bridge_release_left();
    bridge_release_right();
    return true;
}

static uint16_t open_phase_update(uint32_t *accumulator_q16,
                                  int32_t step_q16,
                                  int16_t signed_command,
                                  bool advance)
{
    if (advance && signed_command != 0) {
        const int32_t signed_step = signed_command < 0 ? -step_q16 : step_q16;
        *accumulator_q16 += (uint32_t)signed_step;
    }
    return (uint16_t)(*accumulator_q16 >> 16);
}

static int16_t abs_s16_saturated(int16_t value)
{
    if (value == INT16_MIN) return INT16_MAX;
    return value < 0 ? (int16_t)-value : value;
}

static uint16_t pwm_compare_from_centered_duty(int16_t duty)
{
    const int32_t center = (int32_t)pwm_res / 2;
    const int32_t minimum = (int32_t)pwm_margin;
    const int32_t maximum = (int32_t)pwm_res - (int32_t)pwm_margin;
    int32_t compare = center + (int32_t)duty;
    if (compare < minimum) compare = minimum;
    if (compare > maximum) compare = maximum;
    return (uint16_t)compare;
}

static void set_pwm_left(const mc_foc_output_t *out)
{
    if (out->zero_duty_phase_brake) {
        set_current_zero_vector_left();
        return;
    }
    LEFT_TIM->LEFT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);
    LEFT_TIM->LEFT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);
    LEFT_TIM->LEFT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);
}

static void set_pwm_right(const mc_foc_output_t *out)
{
    if (out->zero_duty_phase_brake) {
        set_current_zero_vector_right();
        return;
    }
    RIGHT_TIM->RIGHT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);
    RIGHT_TIM->RIGHT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);
    RIGHT_TIM->RIGHT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);
}

/* Establish the same shunt common-mode domain used by current calibration
 * before allowing the PI to see phase-current samples. Returns true only after
 * enough complete ADC conversions have occurred with MOE on and zero vector. */
static bool bridge_current_domain_service(bool left, bool request_ok)
{
    TIM_TypeDef *tim = left ? LEFT_TIM : RIGHT_TIM;
    volatile uint8_t *warm = left ? &bridgeWarmupLeft : &bridgeWarmupRight;
    volatile uint16_t *transitions = left ? &bridgeTransitionsLeft : &bridgeTransitionsRight;

    if (!request_ok) {
        tim->BDTR &= ~TIM_BDTR_MOE;
        *warm = 0U;
        return false;
    }

    /* The ADC sample available in THIS call was captured before MOE was raised.
     * Match the active low-FET calibration domain first and discard only that
     * released-domain sample. */
    if ((tim->BDTR & TIM_BDTR_MOE) == 0U) {
        if (left) set_current_zero_vector_left(); else set_current_zero_vector_right();
        tim->BDTR |= TIM_BDTR_MOE;
        *warm = BRIDGE_CURRENT_WARMUP_ADC_SAMPLES;
        if (*transitions != UINT16_MAX) ++(*transitions);
        return false;
    }

    /* One complete DMA period has elapsed with MOE + low-FET zero vector active.
     * The current sample is now in the same electrical domain as the stored
     * offsets and is valid immediately; do not add another six-sample delay. */
    if (*warm != 0U) --(*warm);
    return true;
}

static void configure_phase_override(motor_all_state_t *motor,
                                     bool calibration,
                                     bool open_mode,
                                     uint16_t calibration_phase_q16,
                                     uint32_t *open_accumulator,
                                     int32_t open_step_q16,
                                     int16_t command)
{
    if (calibration) {
        mc_foc_set_phase_override(motor, true, calibration_phase_q16);
        return;
    }
    if (open_mode) {
        const uint16_t phase = open_phase_update(
            open_accumulator, open_step_q16, command, true);
        mc_foc_set_phase_override(motor, true, phase);
        return;
    }
    mc_foc_set_phase_override(motor, false, 0U);
}

static inline uint32_t motor_cycle_count(void)
{
#if defined(DWT)
    return DWT->CYCCNT;
#else
    return 0U;
#endif
}

static inline void motor_isr_record_timing(uint32_t start_cycles)
{
    const uint32_t cycles = motor_cycle_count() - start_cycles;
    motorControlIsrLastCycles = cycles;
    if (cycles > motorControlIsrMaxCycles) motorControlIsrMaxCycles = cycles;

    /* V15: deadline is observation, not a PWM permission gate. The 16-kHz path
     * must not clear runtime commands because one diagnostic sample exceeded the
     * nominal cycle budget. Hard DC-current chopping remains sample-local below. */
    if (motorControlIsrDeadlineCycles != 0U && cycles >= motorControlIsrDeadlineCycles) {
        if (motorControlIsrOverrunCount != UINT16_MAX) ++motorControlIsrOverrunCount;
        if (motorIsrOverrunStreak != UINT8_MAX) ++motorIsrOverrunStreak;
    } else {
        motorIsrOverrunStreak = 0U;
    }
    motorControlIsrOverrunFaultMask = 0U;
}

void DMA1_Channel1_IRQHandler(void)
{
    const uint32_t isr_cycle_start = motor_cycle_count();
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    service_buzzer_isr();
    ADC_Slow_TriggerFromCurrentISR();

    /* Keep the proven V11+ offset calibration, but make its startup ownership as
     * simple as V1: while active, the ISR owns both bridges in the calibrated
     * low-FET zero vector and performs no FOC work. */
    const bool current_offsets_ready = current_cal_service_isr();
    const bool current_cal_active =
        currentCal.state == MOTOR_CURRENT_CAL_SETTLING ||
        currentCal.state == MOTOR_CURRENT_CAL_COLLECTING;
    if (!current_offsets_ready) {
        curL_phaA = 0; curL_phaB = 0; curL_DC = 0;
        curR_phaB = 0; curR_phaC = 0; curR_DC = 0;
        dcRawLeftCentiAmp = 0; dcRawRightCentiAmp = 0;
        dcTelemLeftQ4 = 0; dcTelemRightQ4 = 0;
        currentMeasurementValidLeft = false;
        currentMeasurementValidRight = false;
        if (!current_cal_active) {
            bridge_release_left();
            bridge_release_right();
        }
        motor_isr_record_timing(isr_cycle_start);
        return;
    }

    if (--batteryFilterTicks == 0U) {
        batteryFilterTicks = 1000U;
        filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
        batVoltage = (int16_t)(batVoltageFixdt >> 16);
    }

    /* V1-compatible direct ADC delta path. These six values are the only current
     * samples presented to protection/FOC. */
    curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);
    curL_phaB = (int16_t)(offsetrlB - adc_buffer.rlB);
    curL_DC   = (int16_t)(offsetdcl - adc_buffer.dcl);
    curR_phaB = (int16_t)(offsetrrB - adc_buffer.rrB);
    curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);
    curR_DC   = (int16_t)(offsetdcr - adc_buffer.dcr);

    const uint8_t output_request_mask =
        (uint8_t)(runtimeMotorEnableMask | sensorCalibrationOpenLoopMask);
    const bool left_output_requested =
        enable != 0U && (output_request_mask & 0x01U) != 0U;
    const bool right_output_requested =
        enable != 0U && (output_request_mask & 0x02U) != 0U;

    /* Keep only the stock-board hard current chop in the hot path. There is no
     * overrun latch, telemetry-valid gate or commissioning guard that can clear a
     * command. If DC current exceeds the physical board limit, MOE is removed for
     * this sample immediately, exactly as in the V1/hoverboard lineage. */
    const bool left_current_ok = abs_s16_saturated(curL_DC) <= curDC_max;
    const bool right_current_ok = abs_s16_saturated(curR_DC) <= curDC_max;
    if (!left_output_requested || !left_current_ok) bridge_release_left();
    if (!right_output_requested || !right_current_ok) bridge_release_right();
    motorControlOvercurrentFaultMask = 0U;

    const bool left_calibration = (sensorCalibrationOpenLoopMask & 0x01U) != 0U;
    const bool right_calibration = (sensorCalibrationOpenLoopMask & 0x02U) != 0U;
    const bool left_detect_current = (sensorCalibrationCurrentControlMask & 0x01U) != 0U;
    const bool right_detect_current = (sensorCalibrationCurrentControlMask & 0x02U) != 0U;
    const bool left_open = motorLeft.m_control_mode == CONTROL_MODE_OPENLOOP;
    const bool right_open = motorRight.m_control_mode == CONTROL_MODE_OPENLOOP;

    /* V18 SENSOR CADENCE: sample BOTH feedback paths on EVERY 16-kHz DMA sample.
     * Speed estimators are defined against this cadence and are no longer tied to
     * whether the FOC workload had to be interleaved/shed. */
    const uint32_t left_idr = LEFT_HALL_U_PORT->IDR;
    const uint8_t left_u = (uint8_t)!(left_idr & LEFT_HALL_U_PIN);
    const uint8_t left_v = (uint8_t)!(left_idr & LEFT_HALL_V_PIN);
    const uint8_t left_w = (uint8_t)!(left_idr & LEFT_HALL_W_PIN);
    if (motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB && LeftEncoder_IsEnabled()) {
        const uint8_t encoder_a = left_v; /* PB6 */ /* TIM4_CH1 */
        const uint8_t encoder_b = left_w; /* PB7 */ /* TIM4_CH2 */
        MotorSensor_UpdateHardwareEncoder(&motorConfigLeft, &motorSensorStateLeft,
                       LeftEncoder_GetCount(), encoder_a, encoder_b,
                       motorConfLeft.foc_motor_pole_pairs, &motorSensorSampleLeft);
    } else {
        MotorSensor_Update(&motorConfigLeft, &motorSensorStateLeft,
                       left_u, left_v, left_w, motorConfLeft.foc_motor_pole_pairs,
                       &motorSensorSampleLeft);
    }
    odom_l = motorSensorSampleLeft.position_ticks;

    const uint32_t right_idr = RIGHT_HALL_U_PORT->IDR;
    const uint8_t right_u = (uint8_t)!(right_idr & RIGHT_HALL_U_PIN);
    const uint8_t right_v = (uint8_t)!(right_idr & RIGHT_HALL_V_PIN);
    const uint8_t right_w = (uint8_t)!(right_idr & RIGHT_HALL_W_PIN);
    MotorSensor_Update(&motorConfigRight, &motorSensorStateRight,
                       right_u, right_v, right_w, motorConfRight.foc_motor_pole_pairs,
                       &motorSensorSampleRight);
    odom_r = motorSensorSampleRight.position_ticks;

    /* Prepare phase/current ownership for both motors before choosing this ISR's
     * FOC slot. Current-domain warmup also advances at the true ADC cadence. */
    configure_phase_override(&motorLeft, left_calibration, left_open,
                             sensorCalibrationPhaseLeftQ16,
                             &openPhaseLeftQ16, runtimeOpenPhaseStepLeftQ16,
                             (int16_t)pwml);
    if (left_detect_current) {
        mc_foc_set_voltage_override(&motorLeft, false, 0, 0);
        mc_foc_set_current_commissioning(&motorLeft, true);
        mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_CURRENT);
        motorLeft.m_id_set = sensorCalibrationCurrentLeft;
        motorLeft.m_iq_set = 0;
    } else {
        mc_foc_set_current_commissioning(&motorLeft, false);
        mc_foc_set_voltage_override(&motorLeft, left_calibration,
                                    left_calibration ? sensorCalibrationVoltageLeft : 0, 0);
    }

    configure_phase_override(&motorRight, right_calibration, right_open,
                             sensorCalibrationPhaseRightQ16,
                             &openPhaseRightQ16, runtimeOpenPhaseStepRightQ16,
                             (int16_t)pwmr);
    if (right_detect_current) {
        mc_foc_set_voltage_override(&motorRight, false, 0, 0);
        mc_foc_set_current_commissioning(&motorRight, true);
        mc_foc_set_control_mode(&motorRight, CONTROL_MODE_CURRENT);
        motorRight.m_id_set = sensorCalibrationCurrentRight;
        motorRight.m_iq_set = 0;
    } else {
        mc_foc_set_current_commissioning(&motorRight, false);
        mc_foc_set_voltage_override(&motorRight, right_calibration,
                                    right_calibration ? sensorCalibrationVoltageRight : 0, 0);
    }

    const bool left_phase_valid = left_calibration || left_open ||
        motorLeft.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleLeft.feedback_valid != 0U;
    const bool right_phase_valid = right_calibration || right_open ||
        motorRight.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleRight.feedback_valid != 0U;
    const bool left_request_ok = left_output_requested && left_current_ok && left_phase_valid;
    const bool right_request_ok = right_output_requested && right_current_ok && right_phase_valid;
    const bool left_domain_ready = bridge_current_domain_service(true, left_request_ok);
    const bool right_domain_ready = bridge_current_domain_service(false, right_request_ok);
    const bool left_foc_enabled = left_request_ok && left_domain_ready;
    const bool right_foc_enabled = right_request_ok && right_domain_ready;

    /* During warmup keep the explicit active-domain zero vector. Once ready, the
     * non-selected motor simply holds its previous PWM compare values for one
     * 62.5-us slot, exactly what deterministic 8-kHz current control requires. */
    if (left_request_ok && !left_domain_ready) set_current_zero_vector_left();
    if (right_request_ok && !right_domain_ready) set_current_zero_vector_right();

    /* Emergency liveness fallback is evaluated AFTER both sensors were sampled.
     * It only skips one FOC slot if even the one-motor interleaved path previously
     * exceeded an ADC period. */
    if (motorIsrShedNext) {
        motorIsrShedNext = false;
        motor_isr_record_timing(isr_cycle_start);
        return;
    }

    const bool run_right_slot = motorFocSlotRight;
    motorFocSlotRight = !motorFocSlotRight;
    if (!run_right_slot) {
        const mc_foc_sample_t left_sample = {
            .output_enabled = left_foc_enabled,
            .feedback_valid = motorSensorSampleLeft.feedback_valid != 0U,
            .phase_q16 = motorSensorSampleLeft.electrical_phase_q16,
            .speed_rpm_q4 = motorSensorSampleLeft.mechanical_speed_q4,
            .position_ticks = motorSensorSampleLeft.position_ticks,
            .phase_current_1 = left_domain_ready ? curL_phaA : 0,
            .phase_current_2 = left_domain_ready ? curL_phaB : 0
        };
        mc_foc_run_current_control(&motorLeft, &left_sample, &motorOutputLeft, pwm_res);
        if (left_foc_enabled) set_pwm_left(&motorOutputLeft);
        commissioningFastStreakLeft = 0U;
    } else {
        const mc_foc_sample_t right_sample = {
            .output_enabled = right_foc_enabled,
            .feedback_valid = motorSensorSampleRight.feedback_valid != 0U,
            .phase_q16 = motorSensorSampleRight.electrical_phase_q16,
            .speed_rpm_q4 = motorSensorSampleRight.mechanical_speed_q4,
            .position_ticks = motorSensorSampleRight.position_ticks,
            .phase_current_1 = right_domain_ready ? curR_phaB : 0,
            .phase_current_2 = right_domain_ready ? curR_phaC : 0
        };
        mc_foc_run_current_control(&motorRight, &right_sample, &motorOutputRight, pwm_res);
        if (right_foc_enabled) set_pwm_right(&motorOutputRight);
        commissioningFastStreakRight = 0U;
    }

    /* ISR only publishes raw engineering input current + validity. Averaging is
     * deliberately left to VescProtocol_CurrentTelemetrySample() in the 200-Hz
     * slow loop, matching VESC GET_VALUES read-reset semantics without burdening
     * this timing-sensitive interrupt. */
    dcRawLeftCentiAmp = dc_counts_to_centi_amp(curL_DC);
    dcRawRightCentiAmp = dc_counts_to_centi_amp(curR_DC);
    currentMeasurementValidLeft = left_domain_ready &&
        ((LEFT_TIM->BDTR & TIM_BDTR_MOE) != 0U);
    currentMeasurementValidRight = right_domain_ready &&
        ((RIGHT_TIM->BDTR & TIM_BDTR_MOE) != 0U);
    dcTelemLeftQ4 = currentMeasurementValidLeft ? ((int32_t)dcRawLeftCentiAmp << 4) : 0;
    dcTelemRightQ4 = currentMeasurementValidRight ? ((int32_t)dcRawRightCentiAmp << 4) : 0;

    /* Exact V1 overload detector: if DMA TC is already pending again, this ISR
     * consumed at least one complete 16-kHz sample period. Ask only the NEXT ISR
     * to be short. Do not latch a motor fault, clear ARM, or touch MOE here. */
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U) {
        motorIsrShedNext = true;
    }

    motor_isr_record_timing(isr_cycle_start);
}

