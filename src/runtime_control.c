#include "runtime_control.h"
#include "config.h"
#include "defines.h"
#include "eeprom.h"
#include "esc_protocol.h"
#include "foc_motor.h"
#include "motor_current_cal.h"
#include "vesc_app.h"
#include "left_encoder.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_it.h"
#include <limits.h>
#include <string.h>
#include <math.h>

extern mc_configuration motorConfLeft;
extern mc_configuration motorConfRight;
extern motor_all_state_t motorLeft;
extern motor_all_state_t motorRight;
extern mc_foc_output_t motorOutputLeft;
extern mc_foc_output_t motorOutputRight;
extern volatile adc_buf_t adc_buffer;
extern volatile uint32_t main_loop_counter;
extern volatile uint32_t buzzerTimer;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deci_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t curL_phaA, curL_phaB, curL_DC;
extern int16_t curR_phaB, curR_phaC, curR_DC;
extern volatile int pwml;
extern volatile int pwmr;
extern uint8_t enable;
extern int32_t odom_l;
extern int32_t odom_r;

volatile uint8_t controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
volatile uint8_t controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;
volatile int32_t runtimeSetpointLeft = 0;
volatile int32_t runtimeSetpointRight = 0;
volatile int16_t runtimeCommandLeft = 0;
volatile int16_t runtimeCommandRight = 0;
volatile uint8_t runtimeMotorEnableMask = 0U;

/* Extended OPEN runtime parameter. Default aman 2 electrical Hz, 5 s, tetapi
 * tidak dipakai untuk client legacy sampai ESC_FLAG_OPEN_PARAMS_VALID diterima. */
volatile uint32_t runtimeOpenFrequencyLeftMilliHz = 2000U;
volatile uint32_t runtimeOpenFrequencyRightMilliHz = 2000U;
volatile int32_t runtimeOpenPhaseStepLeftQ16 = 0;
volatile int32_t runtimeOpenPhaseStepRightQ16 = 0;
volatile uint32_t runtimeOpenDurationLeftMs = 5000U;
volatile uint32_t runtimeOpenDurationRightMs = 5000U;
volatile uint8_t runtimeOpenParamsValidMask = 0U;

static int32_t open_phase_step_q16(uint32_t frequency_millihz)
{
    const int64_t denominator = 1000LL * (int64_t)CONTROL_FOC_MOTOR_FREQUENCY_HZ;
    const int64_t numerator = (int64_t)frequency_millihz * 65536LL * 65536LL;
    int64_t step = (numerator + (denominator / 2LL)) / denominator;
    if (step > INT32_MAX) step = INT32_MAX;
    return (int32_t)step;
}

/*
 * Clock runtime hybrid. HAL/SysTick dan heartbeat DMA 16 kHz dihitung sebagai
 * DUA timeline kumulatif independen, kemudian dipilih timeline yang paling maju.
 * Jangan menjumlahkan max(delta) per pemanggilan: bila HAL tick terlihat pada call
 * A dan DMA tick ekuivalen baru terlihat pada call B, metode lama bisa menghitung
 * millisecond fisik yang sama dua kali dan mempercepat OPEN timeout/telemetry.
 */
static bool runtimeClockInitialized = false;
static uint32_t runtimeClockBaseMs = 0U;
static uint32_t runtimeClockMs = 0U;
static uint32_t runtimeClockLastHal = 0U;
static uint32_t runtimeClockLastDma = 0U;
static uint32_t runtimeClockHalElapsedMs = 0U;
static uint32_t runtimeClockDmaElapsedMs = 0U;
static uint32_t runtimeClockDmaRemainder = 0U;

uint32_t RuntimeControl_MonotonicMs(void)
{
    const uint32_t hal_now = HAL_GetTick();
    const uint32_t dma_now = buzzerTimer;

    if (!runtimeClockInitialized) {
        runtimeClockInitialized = true;
        runtimeClockBaseMs = hal_now;
        runtimeClockMs = hal_now;
        runtimeClockLastHal = hal_now;
        runtimeClockLastDma = dma_now;
        runtimeClockHalElapsedMs = 0U;
        runtimeClockDmaElapsedMs = 0U;
        runtimeClockDmaRemainder = 0U;
        return runtimeClockMs;
    }

    const uint32_t hal_delta = hal_now - runtimeClockLastHal;
    const uint32_t dma_delta = dma_now - runtimeClockLastDma;
    runtimeClockLastHal = hal_now;
    runtimeClockLastDma = dma_now;
    runtimeClockHalElapsedMs += hal_delta;

    /* PWM_FREQ=16000 -> 16 control ticks/ms. Remainder mempertahankan pecahan
     * antar-call; unsigned delta tetap aman ketika counter DMA 32-bit wrap. */
    const uint32_t dma_accum = runtimeClockDmaRemainder + dma_delta;
    runtimeClockDmaElapsedMs += dma_accum >> 4;
    runtimeClockDmaRemainder = dma_accum & 0x0FU;

    const uint32_t elapsed = (runtimeClockDmaElapsedMs > runtimeClockHalElapsedMs)
        ? runtimeClockDmaElapsedMs : runtimeClockHalElapsedMs;
    runtimeClockMs = runtimeClockBaseMs + elapsed;
    return runtimeClockMs;
}

#define OPEN_FREQUENCY_MAX_MILLIHZ 50000U
#define OPEN_DURATION_MAX_MS       60000U

typedef struct {
    bool started;
    bool expired;
    uint32_t start_tick;
} OpenLoopRunState;

static OpenLoopRunState openRunLeft;
static OpenLoopRunState openRunRight;

volatile uint8_t sensorCalibrationOpenLoopMask = 0U;
volatile uint8_t sensorCalibrationCurrentControlMask = 0U;
volatile uint16_t sensorCalibrationPhaseLeftQ16 = 0U;
volatile uint16_t sensorCalibrationPhaseRightQ16 = 0U;
volatile int16_t sensorCalibrationCurrentLeft = 0;
volatile int16_t sensorCalibrationCurrentRight = 0;
volatile int16_t sensorCalibrationVoltageLeft = 0;
volatile int16_t sensorCalibrationVoltageRight = 0;
volatile uint8_t sensorCalibrationFastCurrentFaultMask = 0U;

PositionPidConfig positionPidConfigLeft;
PositionPidConfig positionPidConfigRight;
MotorRuntimeConfig motorConfigLeft;
MotorRuntimeConfig motorConfigRight;
HomingMotorConfig homingConfigLeft;
HomingMotorConfig homingConfigRight;
uint16_t homingTimeoutMs = 8000U;
uint16_t homingDebounceMs = 150U;
bool homingOnBootLeft = false;
bool homingOnBootRight = false;
SteeringCalibration steeringCalibrationLeft;
SteeringCalibration steeringCalibrationRight;
/* V20 session position origin. Hall and incremental AB are relative sensors after
 * power-on; before hard-stop homing, define 0..360 as exactly one mechanical
 * revolution starting from the post-detect/post-sync position of this boot. */
static int32_t positionSessionZeroLeft = 0;
static int32_t positionSessionZeroRight = 0;
static bool positionSessionZeroValidLeft = false;
static bool positionSessionZeroValidRight = false;

typedef struct {
    uint8_t state;
    uint8_t operation; /* 1=single-stop home, 2=full 0..360 calibration */
    uint32_t start_tick;
    uint32_t stop_candidate_since_tick;
    int16_t peak_current_centi_amp;
    int32_t last_position_ticks;
    uint32_t last_motion_tick;
    int32_t right_stop_ticks;
    bool stop_candidate;
} HomingRuntimeState;

static HomingRuntimeState homingRuntimeLeft;
static HomingRuntimeState homingRuntimeRight;
static uint8_t homingActiveMask = 0U;
/* Auto-homing EEPROM v8 berjalan tanpa memerlukan koneksi host. Delay boot memberi
 * ADC/FOC waktu stabil sebelum motor mulai mencari hard-stop. */
#define BOOT_HOMING_DELAY_MS 750U
static uint8_t bootHomingPendingMask = 0U;
static bool homingStartedFromBoot = false;
static uint8_t homingPendingOperationLeft = 0U;
static uint8_t homingPendingOperationRight = 0U;

/* -------------------- Sensor commissioning / auto-detect -------------------- */
#define SENSOR_CAL_MANUAL_DEFAULT_MS 6000U
#define SENSOR_CAL_AUTO_ALIGN_MS 1200U
#define SENSOR_CAL_AUTO_TIMEOUT_MS 30000U
/* Match the VESC 6.00 Hall commissioning shape: three forward and three
 * reverse electrical revolutions. Bidirectional acquisition cancels mechanical
 * lag and makes RAW Hall -> commanded electrical-sector voting independent of
 * one-way drag. */
#define SENSOR_CAL_HALL_FORWARD_SWEEPS 3U
#define SENSOR_CAL_HALL_REVERSE_SWEEPS 3U
#define SENSOR_CAL_HALL_TOTAL_SWEEPS (SENSOR_CAL_HALL_FORWARD_SWEEPS + SENSOR_CAL_HALL_REVERSE_SWEEPS)
/* Upstream VESC measures encoder inversion/ratio with repeated +/-120 degree
 * electrical moves. A steering axis can start against either hard stop, so this
 * port repeats +120 -> 0 -> -120 -> 0 and accepts only the probes that actually
 * moved. This state machine runs at 200 Hz; ISR work is unchanged. */
#define SENSOR_CAL_ENCODER_PROBE_Q4            1920
#define SENSOR_CAL_ENCODER_PROBE_SETTLE_MS      180U
#define SENSOR_CAL_ENCODER_PROBE_MIN_VALID        4U
#define SENSOR_CAL_ENCODER_PROBE_EARLY_COUNT      8U
#define SENSOR_CAL_ENCODER_PROBE_MAX_COUNT       16U
#define SENSOR_CAL_NATIVE_ENCODER_FORWARD_SWEEPS 3U
#define SENSOR_CAL_NATIVE_ENCODER_REVERSE_SWEEPS 3U
#define SENSOR_CAL_VESC_POLE_PAIRS_MAX           60U
/* VESC Hall detect advances 1 electrical degree every 5 ms (0.2 deg/ms).
 * Q4=1/16 degree, so 3 Q4/ms = 0.1875 deg/ms is the nearest integer rate. */
#define SENSOR_CAL_PHASE_Q4_PER_MS 3U
/* Physical board scale is locked at 800 internal current units/A (50 ADC
 * count/A, then <<4). VESC detect current therefore maps directly to this domain. */
#define SENSOR_CAL_CURRENT_DEFAULT_INTERNAL 800   /* 1.00 A */
#define SENSOR_CAL_CURRENT_MIN_INTERNAL     200   /* 0.25 A */
#define SENSOR_CAL_CURRENT_MAX_INTERNAL    1600   /* 2.00 A safety cap on stock hoverboard */
#define SENSOR_CAL_ALIGN_START_CURRENT_INTERNAL 400 /* 0.50 A soft-start */
#define SENSOR_CAL_OVERCURRENT_DEBOUNCE_MS 250U
/* Software commissioning limit uses the measured phase-current vector, not the
 * DC-link shunt. Fast 17-A DC-link chopping in motor.c remains the hard guard. */
#define SENSOR_CAL_CURRENT_LIMIT_CENTI_AMP 700U /* 7 A conservative phase limit */

typedef struct {
    uint8_t state;
    uint8_t method;
    uint8_t motor;
    uint8_t sensor_type;
    uint8_t result_code;
    uint8_t previous_raw;
    bool aligned_zero_done;
    uint32_t start_tick;
    uint32_t run_start_tick;
    uint32_t overcurrent_since_tick;
    uint16_t manual_duration_ms;
    uint16_t samples;
    uint16_t invalid_samples;
    uint16_t completed_cycles;
    int8_t sweep_direction;            /* +1 forward, -1 reverse */
    uint8_t forward_cycles;
    uint8_t reverse_cycles;
    int16_t drive_current_internal;
    int16_t phase_q4;
    int32_t encoder_start;
    /* Raw TIM4 count while rotor is locked at commanded electrical phase 0.
     * Used only to report the measured VESC encoder offset for this calibration. */
    int32_t encoder_zero_raw_count;
    int32_t encoder_delta;
    uint32_t encoder_valid_edges_start;
    uint32_t encoder_invalid_transitions_start;
    /* V15 session baselines survive failed sweep windows. V14 reset the edge
     * baseline every six electrical cycles, which could hide a healthy encoder
     * that accumulated 94+ clean edges only across multiple dither windows. */
    uint32_t encoder_session_valid_edges_start;
    uint32_t encoder_session_invalid_transitions_start;
    uint8_t encoder_session_seen_mask;
    /* V14: match VESC 6.00 Hall detect semantics. For every raw Hall code,
     * accumulate the commanded electrical angle as a circular sin/cos vector
     * across 3 forward + 3 reverse sweeps. This is robust to rotor lag near
     * 60-degree boundaries, unlike V12/V13 sector voting. */
    int32_t hall_sin_sum_q14[8];
    int32_t hall_cos_sum_q14[8];
    uint16_t hall_angle_samples[8];
    uint8_t measured_hall_table[8];
    bool measured_hall_table_valid;
    uint16_t encoder_transition_start[16];
    uint16_t hall_transition_start[64];
    uint8_t observed_hall_sequence[6];
    uint8_t observed_hall_count;
    uint8_t observed_encoder_sequence[4];
    uint8_t observed_encoder_count;
    bool motion_detected;
    int32_t last_motion_position;
    uint32_t last_motion_tick;
    uint32_t last_current_step_tick;
    int16_t requested_drive_current_internal;
    bool vesc_wire_detect;
    uint8_t original_sensor_inverted;
    uint8_t detected_pole_pairs;
    uint8_t detected_encoder_inverted;
    /* V17: raw A/B transition direction proof. TIM4 count is authoritative,
     * but the directed quadrature votes observed during a commanded positive
     * electrical sweep tell us whether raw hardware count must be inverted. */
    uint32_t encoder_positive_score;
    uint32_t encoder_negative_score;
    /* V17 VESC-style direction proof separates transitions captured while the
     * commanded electrical field moves forward from those captured in reverse.
     * Aggregating both directions destroys the sign evidence on a loaded wheel. */
    uint16_t encoder_transition_mid[16];
    int32_t encoder_forward_delta;
    uint32_t encoder_direction_normal_score;
    uint32_t encoder_direction_inverted_score;
    bool encoder_direction_proved;
    bool encoder_ratio_fallback_used;
    bool encoder_probe_initialized;
    uint8_t encoder_probe_stage;
    uint8_t encoder_probe_total;
    uint8_t encoder_probe_valid;
    int32_t encoder_probe_start_raw;
    uint32_t encoder_probe_hold_until;
    uint16_t encoder_probe_ratio_sum;
    uint8_t encoder_probe_ratio_min;
    uint8_t encoder_probe_ratio_max;
    uint16_t detected_encoder_offset_deg;
} SensorCalibrationRuntime;

static SensorCalibrationRuntime sensorCal;

/* V13: COMM_DETECT_* terminal result must survive the next motor's detect.
 * V1-V12 kept only one global sensorCal object, so a completed RIGHT Hall
 * transaction could overwrite the LEFT terminal state before the UART facade
 * had delivered its reply. Keep only the compact public result per motor. */
typedef struct {
    bool valid;
    RuntimeVescDetectResult result;
    bool encoder_ratio_fallback_used;
} VescDetectTerminalSnapshot;
static VescDetectTerminalSnapshot vescDetectTerminal[2];
static bool sensorCalTelemetryCandidate = false;
static bool encoderAlignedLeft = false;
static bool encoderAlignedRight = false;

typedef struct {
    bool active;
    uint8_t motor;
    uint8_t stage;
    uint32_t start_tick;
    uint32_t stage_tick;
    int16_t target_current_internal;
    int16_t phase_q4;
    int8_t probe_direction;
    uint8_t probe_attempts;
    int32_t probe_start_count;
    int16_t probe_delta_counts;
    bool direction_proved;
    bool direction_changed;
} EncoderAlignmentRuntime;
static EncoderAlignmentRuntime encoderAlign;

/* Sensor-health monitor berjalan di main loop (bukan ISR). OPEN mode sengaja
 * tidak diblokir oleh fault sensor agar motor dapat diuji dan Auto Detect dapat
 * dijalankan sebelum Hall/Encoder siap. */
#define HALL_INVALID_QUALIFY_MS          80U
#define ENCODER_NO_SIGNAL_QUALIFY_MS    600U
#define ENCODER_INVALID_WINDOW_MS       500U
#define ENCODER_INVALID_LIMIT           6U
#define ENCODER_MOTION_COMMAND_MIN      50

typedef struct {
    uint8_t fault_code;
    uint32_t hall_invalid_since;
    uint32_t motion_since;
    uint32_t invalid_window_start;
    uint32_t last_valid_edges;
    uint32_t last_invalid_transitions;
    uint16_t valid_in_window;
    uint16_t invalid_in_window;
} SensorHealthRuntime;

static SensorHealthRuntime sensorHealthLeft;
static SensorHealthRuntime sensorHealthRight;

/* Command reversal hanya mereset integrator outer/current loop; sensor phase
 * tetap dimiliki backend Hall/encoder dan tidak pernah diprime ulang di runtime. */
static int8_t previousCommandDirectionLeft = 0;
static int8_t previousCommandDirectionRight = 0;

/* Fault event UX:
 * - proteksi internal tetap persistent dan tetap memblokir ARM selama sumber
 *   fault nyata belum sehat;
 * - error yang ditampilkan + buzzer adalah EVENT selama tepat 3 detik;
 * - setelah 3 detik motor tetap DISARM, buzzer berhenti, dan kode event kembali 0;
 * - attempt ARM baru membuka reporting lagi dan fault nyata akan terdeteksi ulang.
 * Ini mencegah fault statis mengulang beep tanpa henti. */
#define FAULT_REPORT_DURATION_MS 3000U
static bool faultReportActiveLeft = false;
static bool faultReportActiveRight = false;
static bool faultReportSuppressLeft = false;
static bool faultReportSuppressRight = false;
static uint8_t faultReportCodeLeft = ESC_MOTOR_ERROR_NONE;
static uint8_t faultReportCodeRight = ESC_MOTOR_ERROR_NONE;
static uint32_t faultReportUntilLeft = 0U;
static uint32_t faultReportUntilRight = 0U;

static void fault_report_reset_all(void);
static void fault_report_allow_new(bool left);
static void fault_report_latch(bool left, uint8_t code, uint32_t now);
static void fault_report_service(uint32_t now);

static uint8_t requestedModeLeft = ESC_MODE_OPEN;
static uint8_t requestedModeRight = ESC_MODE_OPEN;
static bool linkActive = false;
static bool armRequestedLeft = false;
static bool armRequestedRight = false;
/* VESC-style fault stop has no separate ARM-release handshake. A repeated
 * command may be accepted again after the hold window if the fault is healthy. */
static bool armedLeft = false;
static bool armedRight = false;
static uint8_t armRejectLeft = ESC_ARM_REJECT_NONE;
static uint8_t armRejectRight = ESC_ARM_REJECT_NONE;
static uint8_t bootCpuFaultCode = 0U;
static uint8_t bootResetFlags = 0U;
/* Independent watchdog dari thread-context untuk memastikan ADC/current ISR
 * benar-benar masih berjalan. Ini menangkap DMA ADC yang berhenti total, kasus
 * yang tidak mungkin dideteksi dari dalam ISR itu sendiri. */
#define MOTOR_ISR_HEARTBEAT_TIMEOUT_MS 100U
static uint32_t motorIsrHeartbeatLast = 0U;
static uint32_t motorIsrHeartbeatLastChangeMs = 0U;
static bool eepromOk = false;
static bool eepromVerified = false;
static uint16_t eepromStoredCrc = 0U;
static uint16_t eepromGeneration = 0U;
static uint16_t eepromVerifyFailures = 0U;
static bool settingsDirty = false;
static uint32_t lastCommandTick = 0;
static uint32_t reconnectCounter = 0;
static uint16_t feedbackSequence = 0;
static uint16_t lastHostSequence = 0;
static uint8_t telemetryPage = ESC_TELEM_BASIC;
static uint8_t telemetryPidLoop = ESC_PID_SPEED;
static uint8_t configMotor = ESC_MOTOR_LEFT;
/* One-shot config/calibration tidak boleh mengganti halaman stream utama.
 * Request hanya mengisi slot side-channel ini; setelah satu frame sukses dikirim,
 * stream BASIC/FOC/PID/RAW langsung berlanjut tanpa starvation. */
static uint8_t telemetryOneShotPage = ESC_TELEM_BASIC;
static bool configOneShotPending = false;
/* ARM snapshot punya antrian sendiri agar tidak menimpa REQUEST_CONFIG/SENSOR_CAL
 * yang kebetulan pending pada saat yang sama. */
static bool armStatusSnapshotPending = false;
/* Wajib ada minimal satu frame stream normal di antara dua one-shot. Tanpa guard
 * ini, GUI yang meminta SENSOR_CAL berulang dapat mengisi slot one-shot sebelum
 * setiap periode telemetry dan membuat BASIC/FOC/PID terlihat berhenti. */
static bool telemetryLastWasOneShot = false;
static uint16_t telemetryRateHz = TELEMETRY_RATE_DEFAULT;
static uint32_t telemetryMask = 0xFFFFFFFFUL;
static uint32_t lastTelemetryTick = 0;

static void disarm_outputs(void);
static void disarm_motor(uint8_t motor);
static void refresh_master_enable(void);
static void sensor_cal_start_rejected(uint8_t method, uint8_t motor);
static uint8_t to_foc_mode(uint8_t mode);
static void apply_runtime_target_one(bool left, uint8_t mode, int32_t host_setpoint,
                                     uint32_t dt_ms, uint32_t now);
static void sensor_health_reset_one(SensorHealthRuntime *health, const MotorSensorState *state);
static void apply_sensor_backend_to_foc(const MotorRuntimeConfig *cfg, mc_configuration *params);
static bool sensor_cal_finalize_auto_hall(MotorRuntimeConfig *cfg);
static void sensor_cal_record_hall_angle(uint8_t raw_hall);
static bool sensor_cal_finalize_encoder_sequence(MotorRuntimeConfig *cfg, MotorSensorState *state);
static void vesc_detect_hall_table(const MotorRuntimeConfig *cfg, uint8_t table[8]);
static void vesc_detect_store_terminal_snapshot(void);
static void position_session_rezero(bool left);

/* ARM state berada di BASIC payload. Saat user sedang streaming FOC/PID/RAW,
 * kirim satu BASIC side-channel hanya ketika state/reject berubah supaya diagnosis
 * LEFT/RIGHT selalu sampai tanpa mengubah halaman stream utama atau membuat spam. */
static void queue_arm_status_snapshot(void)
{
    armStatusSnapshotPending = true;
}

static void set_arm_reject(bool left, uint8_t reason)
{
    uint8_t *target = left ? &armRejectLeft : &armRejectRight;
    if (*target != reason) {
        *target = reason;
        queue_arm_status_snapshot();
    }
}

/* EEPROM emulation memerlukan tabel seluruh virtual address yang ikut page transfer. */
uint16_t VirtAddVarTab[NB_OF_VAR] = {
    2000,2001,2002,2003,2004,2005,2006,2007,2008,2009,2010,2011,2012,2013,2014,2015,2016,2017,2018,2019,2020,2021,2022,2023,2024,2025,2026,2027,2028,2029,2030,2031,2032,2033,2034,2035,2036,2037,2038,2039,2040,2041,2042,2043,2044,2045,2046,2047,2048,2049,2050,2051,2052,2053,2054,2055,2056,2057,2058,2059,2060,2061,2062,2063,2064,2065,2066,2067,2068,2069,2070,2071,2072,2073,2074,2075,2076,2077,2078,2079,2080,2081,2082,2083,2084,2085,2086,2087,2088,2089,2090,2091,2092,2093,2094,2095,2096,2097,2098,2099,2100,2101,2102,2103,2104,2105,2106,2107,2108,2109,2110,2111,2112,2113,2114,2115,2116,2117,2118,2119,2120,2121,2122,2123,2124,2125,2126,2127,2128,2129,2130,2131,2132,2133,2134,2135,2136,2137,2138,2139,2140,2141,2142,2143,2144,2145,2146,2147,2148,2149,2150,2151,2152,2153,2154,2155,2156,2157,2158,2159,2160,2161
};

#define EEPROM_CONFIG_VERSION 19U
#define EEPROM_CONFIG_VERSION_V18 18U
#define EEPROM_CONFIG_VERSION_V17 17U
#define EEPROM_CONFIG_VERSION_V16 16U
#define EEPROM_CONFIG_VERSION_V15 15U
#define EEPROM_CONFIG_VERSION_V14 14U
#define EEPROM_CONFIG_VERSION_V13 13U
#define EEPROM_CONFIG_VERSION_V12 12U
#define EEPROM_CONFIG_VERSION_V11 11U
#define EEPROM_CONFIG_VERSION_V10 10U
#define EEPROM_CONFIG_VERSION_V9 9U
#define EEPROM_CONFIG_VERSION_V8 8U
#define EEPROM_CONFIG_VERSION_V7 7U
#define EEPROM_CONFIG_VERSION_V6 6U
#define EEPROM_CONFIG_VERSION_V5 5U
#define EEPROM_WORD_KEY       0U
#define EEPROM_WORD_VERSION   1U
#define EEPROM_WORD_MAX_CURRENT 2U
#define EEPROM_WORD_MAX_SPEED   3U
#define EEPROM_LEFT_FOC_BASE    4U
#define EEPROM_RIGHT_FOC_BASE   16U
#define EEPROM_LEFT_POS_BASE    28U
#define EEPROM_RIGHT_POS_BASE   44U
#define EEPROM_LEFT_MOTOR_CONFIG  (EEPROM_LEFT_FOC_BASE + 10U)
#define EEPROM_LEFT_ENCODER_CPR   (EEPROM_LEFT_FOC_BASE + 11U)
#define EEPROM_RIGHT_MOTOR_CONFIG (EEPROM_RIGHT_FOC_BASE + 10U)
#define EEPROM_RIGHT_ENCODER_CPR  (EEPROM_RIGHT_FOC_BASE + 11U)
#define EEPROM_WORD_TELEM_RATE  60U
#define EEPROM_WORD_TELEM_PAGE  61U
#define EEPROM_WORD_GENERATION  62U
#define EEPROM_WORD_CRC         63U
#define EEPROM_LEFT_HOMING_CURRENT   64U
#define EEPROM_RIGHT_HOMING_CURRENT  65U
#define EEPROM_LEFT_HOMING_COMMAND   66U
#define EEPROM_RIGHT_HOMING_COMMAND  67U
#define EEPROM_HOMING_TIMEOUT        68U
#define EEPROM_HOMING_DEBOUNCE       69U
#define EEPROM_LEFT_BOOT_HOMING      70U
#define EEPROM_RIGHT_BOOT_HOMING     71U
#define EEPROM_LEFT_HALL_LUT_LO      72U
#define EEPROM_LEFT_HALL_LUT_HI      73U
#define EEPROM_RIGHT_HALL_LUT_LO     74U
#define EEPROM_RIGHT_HALL_LUT_HI     75U
#define EEPROM_LEFT_ENCODER_SEQUENCE  76U
#define EEPROM_RIGHT_ENCODER_SEQUENCE 77U
#define EEPROM_LEFT_ADV_BASE            78U
#define EEPROM_RIGHT_ADV_BASE           99U
#define EEPROM_ADV_WORDS_PER_MOTOR      21U
#define EEPROM_VESC_APP_BASE              120U
#define EEPROM_VESC_APP_WORDS             24U
#define EEPROM_VESC_APP_PACKED            (EEPROM_VESC_APP_BASE + 0U)
#define EEPROM_VESC_APP_TIMEOUT_LO        (EEPROM_VESC_APP_BASE + 1U)
#define EEPROM_VESC_APP_TIMEOUT_HI        (EEPROM_VESC_APP_BASE + 2U)
#define EEPROM_VESC_APP_BRAKE_CA          (EEPROM_VESC_APP_BASE + 3U)
#define EEPROM_VESC_APP_BAUD_LO           (EEPROM_VESC_APP_BASE + 4U)
#define EEPROM_VESC_APP_BAUD_HI           (EEPROM_VESC_APP_BASE + 5U)
#define EEPROM_VESC_APP_FLAGS             (EEPROM_VESC_APP_BASE + 6U)
#define EEPROM_VESC_APP_HYST_MV           (EEPROM_VESC_APP_BASE + 7U)
#define EEPROM_VESC_APP_VSTART_MV         (EEPROM_VESC_APP_BASE + 8U)
#define EEPROM_VESC_APP_VEND_MV           (EEPROM_VESC_APP_BASE + 9U)
#define EEPROM_VESC_APP_VMIN_MV           (EEPROM_VESC_APP_BASE + 10U)
#define EEPROM_VESC_APP_VMAX_MV           (EEPROM_VESC_APP_BASE + 11U)
#define EEPROM_VESC_APP_VCENTER_MV        (EEPROM_VESC_APP_BASE + 12U)
#define EEPROM_VESC_APP_V2START_MV        (EEPROM_VESC_APP_BASE + 13U)
#define EEPROM_VESC_APP_V2END_MV          (EEPROM_VESC_APP_BASE + 14U)
#define EEPROM_VESC_APP_EXP               (EEPROM_VESC_APP_BASE + 15U)
#define EEPROM_VESC_APP_EXP_BRAKE         (EEPROM_VESC_APP_BASE + 16U)
#define EEPROM_VESC_APP_RAMP_POS_MS       (EEPROM_VESC_APP_BASE + 17U)
#define EEPROM_VESC_APP_RAMP_NEG_MS       (EEPROM_VESC_APP_BASE + 18U)
#define EEPROM_VESC_APP_TC_DIFF           (EEPROM_VESC_APP_BASE + 19U)
#define EEPROM_VESC_APP_UPDATE_HZ         (EEPROM_VESC_APP_BASE + 20U)
/* v16 reserved words promoted without changing image size/version. Old v16
 * images contain zero here; loader then keeps safe compiled pole-pair defaults. */
#define EEPROM_LEFT_POLE_PAIRS             (EEPROM_VESC_APP_BASE + 21U)
#define EEPROM_RIGHT_POLE_PAIRS            (EEPROM_VESC_APP_BASE + 22U)
#define EEPROM_VESC_APP_RESERVED            (EEPROM_VESC_APP_BASE + 23U)
/* v17/V19: keep electrical encoder calibration and mechanical steering calibration
 * independent. Homed is runtime-only and is never restored after power loss. */
#define EEPROM_LEFT_ENCODER_RATIO            144U
#define EEPROM_RIGHT_ENCODER_RATIO           145U
#define EEPROM_LEFT_STEER_ZERO_LO            146U
#define EEPROM_LEFT_STEER_ZERO_HI            147U
#define EEPROM_LEFT_STEER_SPAN_LO            148U
#define EEPROM_LEFT_STEER_SPAN_HI            149U
#define EEPROM_LEFT_STEER_CAL                 150U
#define EEPROM_RIGHT_STEER_ZERO_LO           151U
#define EEPROM_RIGHT_STEER_ZERO_HI           152U
#define EEPROM_RIGHT_STEER_SPAN_LO           153U
#define EEPROM_RIGHT_STEER_SPAN_HI           154U
#define EEPROM_RIGHT_STEER_CAL                155U
#define EEPROM_LEFT_GEAR_RATIO_MILLI          156U
#define EEPROM_RIGHT_GEAR_RATIO_MILLI         157U
#define EEPROM_LEFT_CURRENT_MAX                158U
#define EEPROM_LEFT_CURRENT_MIN                159U
#define EEPROM_RIGHT_CURRENT_MAX               160U
#define EEPROM_RIGHT_CURRENT_MIN               161U

static int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

/* Clamp hasil kalkulasi int64 sebelum dikonversi ke int32. Ini mencegah
 * overflow/implementation-defined cast ketika posisi atau gain mendekati batas. */
static int32_t apply_motor_direction_i32(const MotorRuntimeConfig *config, int32_t value)
{
    if (!config->motor_inverted) return value;
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

static int16_t apply_motor_direction_i16(const MotorRuntimeConfig *config, int16_t value)
{
    if (!config->motor_inverted) return value;
    return (value == INT16_MIN) ? INT16_MAX : (int16_t)-value;
}

static int16_t clamp_i16(int32_t value, int16_t lower, int16_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return (int16_t)value;
}




static MotorRuntimeConfig *sensor_cal_config(uint8_t motor)
{
    return (motor == ESC_MOTOR_RIGHT) ? &motorConfigRight : &motorConfigLeft;
}

static MotorSensorState *sensor_cal_sensor_state(uint8_t motor)
{
    return (motor == ESC_MOTOR_RIGHT) ? &motorSensorStateRight : &motorSensorStateLeft;
}

static MotorSensorSample *sensor_cal_sensor_sample(uint8_t motor)
{
    return (motor == ESC_MOTOR_RIGHT) ? &motorSensorSampleRight : &motorSensorSampleLeft;
}

static mc_configuration *sensor_cal_params(uint8_t motor)
{
    return (motor == ESC_MOTOR_RIGHT) ? &motorConfRight : &motorConfLeft;
}

static uint8_t sensor_cal_target_cycles(uint8_t motor, uint8_t sensor_type)
{
    if (sensor_type == MOTOR_SENSOR_HALL_UVW) return SENSOR_CAL_HALL_TOTAL_SWEEPS;
    if (sensor_type == MOTOR_SENSOR_ENCODER_AB && sensorCal.vesc_wire_detect)
        return SENSOR_CAL_ENCODER_PROBE_MAX_COUNT;
    uint8_t pole_pairs = sensor_cal_params(motor)->foc_motor_pole_pairs;
    if (pole_pairs == 0U) pole_pairs = 1U;
    return pole_pairs;
}

static uint16_t sensor_cal_current_limit_centi_amp(uint8_t motor)
{
    (void)motor;
    /* Commissioning punya batas sendiri. Mengikat ini ke homing membuat Auto
     * Detect v11.8/v11.9-awal berhenti hanya karena current threshold hard-stop
     * disetel rendah. Nilai ini tetap di bawah current chopping 17 A di ISR. */
    return SENSOR_CAL_CURRENT_LIMIT_CENTI_AMP;
}

static int16_t sensor_cal_measured_current_internal(uint8_t motor)
{
    const mc_foc_output_t *out = (motor == ESC_MOTOR_RIGHT) ? &motorOutputRight : &motorOutputLeft;
    int32_t id = out->id; if (id < 0) id = -id;
    int32_t iq = out->iq; if (iq < 0) iq = -iq;
    /* L1 norm is deliberately conservative for commissioning protection and
     * avoids sqrt/floating point in the slow loop. */
    int32_t mag = id + iq;
    if (mag > INT16_MAX) mag = INT16_MAX;
    return (int16_t)mag;
}

static uint16_t sensor_cal_measured_current_centi_amp(uint8_t motor)
{
    const mc_configuration *conf = sensor_cal_params(motor);
    const uint16_t units = conf->foc_current_units_per_amp ? conf->foc_current_units_per_amp : 800U;
    const uint32_t mag = (uint16_t)sensor_cal_measured_current_internal(motor);
    uint32_t ca = (mag * 100U + (units / 2U)) / units;
    if (ca > UINT16_MAX) ca = UINT16_MAX;
    return (uint16_t)ca;
}

/* Motion detector commissioning tidak boleh bergantung pada LUT Hall lama yang
 * justru sedang dipelajari. Hall memakai jumlah edge RAW dari counter ISR 16 kHz;
 * Encoder memakai valid-edge quadrature. Hanya dipanggil slow-loop (~200 Hz). */
static uint32_t sensor_cal_motion_counter(const MotorSensorState *state, uint8_t sensor_type)
{
    if (sensor_type == MOTOR_SENSOR_ENCODER_AB) return state->encoder_valid_edges;
    uint32_t total = 0U;
    for (uint8_t i = 0U; i < 64U; ++i) total += state->hall_transition_counts[i];
    return total;
}


/* Setiap window commissioning harus benar-benar dimulai dari tabel kosong.
 * Tabel directed-transition adalah data pengukuran sesi, bukan konfigurasi aktif,
 * sehingga aman di-zero tanpa mengubah position_ticks/speed telemetry. Reset
 * dilakukan di critical section karena tabel diisi oleh ISR 16 kHz. */
static void sensor_cal_reset_capture_tables(MotorSensorState *state,
                                            const MotorSensorSample *sample,
                                            bool reset_observation)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memset(state->encoder_transition_counts, 0, sizeof(state->encoder_transition_counts));
    memset(state->hall_transition_counts, 0, sizeof(state->hall_transition_counts));
    state->previous_encoder_ab = sample->encoder_ab & 0x03U;
    state->previous_raw_hall_encoding = sample->raw_hall_encoding & 0x07U;
    state->hall_raw_initialized = (state->previous_raw_hall_encoding > 0U &&
                                   state->previous_raw_hall_encoding < 7U);
    if (primask == 0U) __enable_irq();

    memset(sensorCal.encoder_transition_start, 0, sizeof(sensorCal.encoder_transition_start));
    memset(sensorCal.hall_transition_start, 0, sizeof(sensorCal.hall_transition_start));

    if (reset_observation) {
        memset(sensorCal.hall_sin_sum_q14, 0, sizeof(sensorCal.hall_sin_sum_q14));
        memset(sensorCal.hall_cos_sum_q14, 0, sizeof(sensorCal.hall_cos_sum_q14));
        memset(sensorCal.hall_angle_samples, 0, sizeof(sensorCal.hall_angle_samples));
        memset(sensorCal.measured_hall_table, 0xFF, sizeof(sensorCal.measured_hall_table));
        sensorCal.measured_hall_table_valid = false;
        memset(sensorCal.observed_hall_sequence, 0, sizeof(sensorCal.observed_hall_sequence));
        memset(sensorCal.observed_encoder_sequence, 0, sizeof(sensorCal.observed_encoder_sequence));
        sensorCal.observed_hall_count = 0U;
        sensorCal.observed_encoder_count = 0U;
        sensorCal.samples = 0U;
        sensorCal.invalid_samples = 0U;
    }

    sensorCal.previous_raw = sample->raw_hall_encoding & 0x07U;
    sensorCal.completed_cycles = 0U;
    sensorCal.detected_pole_pairs = 0U;
    sensorCal.detected_encoder_inverted = 0U;
    sensorCal.encoder_positive_score = 0U;
    sensorCal.encoder_negative_score = 0U;
    memset(sensorCal.encoder_transition_mid, 0, sizeof(sensorCal.encoder_transition_mid));
    sensorCal.encoder_forward_delta = 0;
    sensorCal.encoder_direction_normal_score = 0U;
    sensorCal.encoder_direction_inverted_score = 0U;
    sensorCal.encoder_direction_proved = false;
    sensorCal.encoder_start = state->position_ticks;
    sensorCal.encoder_valid_edges_start = state->encoder_valid_edges;
    sensorCal.encoder_invalid_transitions_start = state->encoder_invalid_transitions;
    sensorCal.last_motion_position = (int32_t)sensor_cal_motion_counter(
        state, sensorCal.sensor_type);
}

/* Cek kandidat hanya pada boundary satu window sweep. Finalizer memakai salinan
 * config sehingga tidak ada LUT/CPR aktif yang berubah sebelum kandidat lolos. */
static bool sensor_cal_auto_candidate_ready(MotorRuntimeConfig *cfg,
                                            MotorSensorState *state)
{
    MotorRuntimeConfig candidate = *cfg;
    if (!sensorCal.motion_detected) return false;

    if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
        if (sensorCal.observed_hall_count != 6U) return false;
        return sensor_cal_finalize_auto_hall(&candidate);
    }

    if (sensorCal.observed_encoder_count != 4U && sensorCal.encoder_session_seen_mask != 0x0FU)
        return false;
    int64_t delta = (int64_t)state->position_ticks - (int64_t)sensorCal.encoder_start;

    if (sensorCal.vesc_wire_detect) {
        /* VESC-wire encoder commissioning is finalized by the dedicated +/-120
         * degree probe service above. Reaching this generic endpoint is never a
         * valid ratio proof. */
        sensorCal.encoder_ratio_fallback_used = false;
        return false;
    }

    if (delta < 0) delta = -delta;
    return delta >= MOTOR_ENCODER_CPR_MIN && delta <= MOTOR_ENCODER_CPR_MAX;
}

static uint16_t sensor_cal_phase_q16_from_q4(int16_t angle_q4)
{
    int32_t wrapped_q4 = (int32_t)angle_q4;
    while (wrapped_q4 >= 5760) wrapped_q4 -= 5760;
    while (wrapped_q4 < 0) wrapped_q4 += 5760;
    return (uint16_t)(((uint32_t)wrapped_q4 * 65536U + 2880U) / 5760U);
}

static void sensor_cal_set_current_override(uint8_t motor, bool active,
                                            int16_t angle_q4, int16_t current_internal)
{
    const uint8_t bit = (motor == ESC_MOTOR_RIGHT) ? 0x02U : 0x01U;
    const uint16_t phase_q16 = sensor_cal_phase_q16_from_q4(angle_q4);
    if (active) {
        sensorCalibrationOpenLoopMask |= bit;
        sensorCalibrationCurrentControlMask |= bit;
    } else {
        sensorCalibrationOpenLoopMask &= (uint8_t)~bit;
        sensorCalibrationCurrentControlMask &= (uint8_t)~bit;
    }
    if (motor == ESC_MOTOR_RIGHT) {
        sensorCalibrationPhaseRightQ16 = phase_q16;
        sensorCalibrationCurrentRight = active ? current_internal : 0;
        sensorCalibrationVoltageRight = 0;
    } else {
        sensorCalibrationPhaseLeftQ16 = phase_q16;
        sensorCalibrationCurrentLeft = active ? current_internal : 0;
        sensorCalibrationVoltageLeft = 0;
    }
}


static void sensor_cal_stop_output(void)
{
    sensorCalibrationOpenLoopMask = 0U;
    sensorCalibrationCurrentControlMask = 0U;
    /* Preserve the fast-current guard evidence through the terminal diagnostic.
     * It is cleared transactionally at the start of the NEXT calibration. V7/V8
     * initially cleared it here, which made a 1-ms safety trip invisible by the
     * time the Python tester asked for post-detect HBTS data. */
    sensorCalibrationCurrentLeft = 0;
    sensorCalibrationCurrentRight = 0;
    sensorCalibrationVoltageLeft = 0;
    sensorCalibrationVoltageRight = 0;

    /* Commissioning owns both bridges exclusively. Clear every FOC-side target
     * as well as the hardware gate when it ends, so a failed/timed-out detect
     * cannot leave a stale D-axis current target visible for one scheduler slot. */
    mc_foc_set_current_commissioning(&motorLeft, false);
    mc_foc_set_current_commissioning(&motorRight, false);
    mc_foc_set_voltage_override(&motorLeft, false, 0, 0);
    mc_foc_set_voltage_override(&motorRight, false, 0, 0);
    mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_NONE);
    mc_foc_set_control_mode(&motorRight, CONTROL_MODE_NONE);
    motorLeft.m_id_set = 0;
    motorLeft.m_iq_set = 0;
    motorRight.m_id_set = 0;
    motorRight.m_iq_set = 0;
    motorLeft.m_voltage_q_set = 0;
    motorRight.m_voltage_q_set = 0;
    controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
    controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;

    armRequestedLeft = false;
    armRequestedRight = false;
    disarm_outputs();
}

/* V14 fixed-point circular Hall acquisition. focSinTableQ14 is already part of
 * the hot FOC build; reuse it here from the slow commissioning loop without
 * adding libm or any division helper to the ISR. */
static int16_t sensor_cal_trig_q14(uint16_t phase)
{
    const uint8_t index = (uint8_t)(phase >> 8);
    const uint8_t fraction = (uint8_t)phase;
    const int16_t y0 = focSinTableQ14[index];
    const int16_t y1 = focSinTableQ14[(uint8_t)(index + 1U)];
    const int32_t delta = (int32_t)y1 - y0;
    return (int16_t)((int32_t)y0 + ((delta * fraction + 128) >> 8));
}

static void sensor_cal_record_hall_angle(uint8_t raw_hall)
{
    raw_hall &= 0x07U;
    const uint16_t phase = sensor_cal_phase_q16_from_q4(sensorCal.phase_q4);
    const int16_t sv = sensor_cal_trig_q14(phase);
    const int16_t cv = sensor_cal_trig_q14((uint16_t)(phase + 16384U));
    int64_t ss = (int64_t)sensorCal.hall_sin_sum_q14[raw_hall] + sv;
    int64_t cs = (int64_t)sensorCal.hall_cos_sum_q14[raw_hall] + cv;
    if (ss > INT32_MAX) ss = INT32_MAX;
    if (ss < INT32_MIN) ss = INT32_MIN;
    if (cs > INT32_MAX) cs = INT32_MAX;
    if (cs < INT32_MIN) cs = INT32_MIN;
    sensorCal.hall_sin_sum_q14[raw_hall] = (int32_t)ss;
    sensorCal.hall_cos_sum_q14[raw_hall] = (int32_t)cs;
    if (sensorCal.hall_angle_samples[raw_hall] != UINT16_MAX)
        ++sensorCal.hall_angle_samples[raw_hall];
}

/* Return the VESC Hall angle domain 0..199. Instead of atan2f, scan the same
 * circular direction against 200 unit vectors and choose the largest dot
 * product. This only runs six times at detect finalization and keeps firmware
 * deterministic/fixed-point on Cortex-M3. */
static uint8_t sensor_cal_hall_angle200(int32_t sin_sum, int32_t cos_sum)
{
    int64_t best_dot = INT64_MIN;
    uint8_t best = 0U;
    for (uint16_t k = 0U; k < 200U; ++k) {
        const uint16_t phase = (uint16_t)(((uint32_t)k * 65536U + 100U) / 200U);
        const int16_t sv = sensor_cal_trig_q14(phase);
        const int16_t cv = sensor_cal_trig_q14((uint16_t)(phase + 16384U));
        const int64_t dot = (int64_t)sin_sum * sv + (int64_t)cos_sum * cv;
        if (dot > best_dot) {
            best_dot = dot;
            best = (uint8_t)k;
        }
    }
    return best;
}

static bool sensor_cal_finalize_auto_hall(MotorRuntimeConfig *cfg)
{
    if (cfg == NULL) return false;
    if (sensorCal.forward_cycles < SENSOR_CAL_HALL_FORWARD_SWEEPS ||
        sensorCal.reverse_cycles < SENSOR_CAL_HALL_REVERSE_SWEEPS) return false;

    uint8_t fails = 0U;
    for (uint8_t raw = 0U; raw < 8U; ++raw) {
        if (sensorCal.hall_angle_samples[raw] > 30U) {
            sensorCal.measured_hall_table[raw] = sensor_cal_hall_angle200(
                sensorCal.hall_sin_sum_q14[raw], sensorCal.hall_cos_sum_q14[raw]);
        } else {
            sensorCal.measured_hall_table[raw] = 255U;
            ++fails;
        }
    }

    /* Same success invariant as VESC 6.00 is two missing Hall codes. On this
     * three-Hall board those must be illegal 000 and 111; accepting either as a
     * real state would produce an unsafe runtime LUT. */
    if (fails != 2U || sensorCal.measured_hall_table[0] != 255U ||
        sensorCal.measured_hall_table[7] != 255U) return false;
    for (uint8_t raw = 1U; raw <= 6U; ++raw) {
        if (sensorCal.measured_hall_table[raw] == 255U) return false;
    }

    /* Local Hall runtime stores a six-state electrical order. Sort the measured
     * VESC circular angles in ascending electrical phase; the cyclic sequence is
     * exactly the RAW Hall order seen by a positive commanded electrical field. */
    uint8_t sequence[6] = {1U,2U,3U,4U,5U,6U};
    for (uint8_t i = 1U; i < 6U; ++i) {
        const uint8_t key = sequence[i];
        uint8_t j = i;
        while (j > 0U &&
               sensorCal.measured_hall_table[sequence[j - 1U]] > sensorCal.measured_hall_table[key]) {
            sequence[j] = sequence[j - 1U];
            --j;
        }
        sequence[j] = key;
    }
    if (!MotorRuntimeConfig_HallSequenceValid(sequence)) return false;
    if (!MotorRuntimeConfig_SetHallSequence(cfg, sequence)) return false;
    memcpy(sensorCal.observed_hall_sequence, sequence, 6U);
    sensorCal.observed_hall_count = 6U;
    sensorCal.measured_hall_table_valid = true;
    return true;
}

static bool sensor_cal_finalize_manual_hall(MotorRuntimeConfig *cfg, const MotorSensorState *state)
{
    uint16_t transition_delta[64];
    uint8_t detected_sequence[6];
    for (uint8_t i = 0U; i < 64U; ++i) {
        /* uint16 subtraction sengaja menangani satu wrap counter selama window
         * commissioning. Counter ISR bersifat monotonik dan saturating. */
        transition_delta[i] = (uint16_t)(state->hall_transition_counts[i] -
                                         sensorCal.hall_transition_start[i]);
    }

    if (!MotorRuntimeConfig_DetectHallSequenceFromTransitions(
            cfg, transition_delta, detected_sequence)) {
        return false;
    }
    return MotorRuntimeConfig_SetHallSequence(cfg, detected_sequence);
}


static uint16_t sensor_cal_encoder_transition_delta(const MotorSensorState *state, uint8_t from, uint8_t to)
{
    const uint8_t index = (uint8_t)(((from & 0x03U) << 2) | (to & 0x03U));
    /* uint16 subtraction intentionally handles one wrap. Calibration is <=15 s;
     * per directed edge count tetap <65536 pada sampling 16 kHz normal. */
    return (uint16_t)(state->encoder_transition_counts[index] -
                      sensorCal.encoder_transition_start[index]);
}

static uint32_t sensor_cal_encoder_sequence_score(const MotorSensorState *state,
                                                  const uint8_t sequence[4],
                                                  bool *all_edges_seen)
{
    uint32_t score = 0U;
    bool complete = true;
    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint8_t from = sequence[i];
        const uint8_t to = sequence[(uint8_t)((i + 1U) & 0x03U)];
        const uint16_t votes = sensor_cal_encoder_transition_delta(state, from, to);
        if (votes == 0U) complete = false;
        score += votes;
    }
    if (all_edges_seen != NULL) *all_edges_seen = complete;
    return score;
}


static uint32_t sensor_cal_encoder_sequence_score_counts(const uint16_t counts[16],
                                                         const uint8_t sequence[4])
{
    uint32_t score = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint8_t from = sequence[i];
        const uint8_t to = sequence[(uint8_t)((i + 1U) & 0x03U)];
        const uint8_t index = (uint8_t)(((from & 0x03U) << 2) | (to & 0x03U));
        score += counts[index];
    }
    return score;
}

static void sensor_cal_encoder_direction_proof(const MotorSensorState *state,
                                               const uint8_t seq_negative_raw[4],
                                               const uint8_t seq_positive_raw[4])
{
    if (!sensorCal.vesc_wire_detect || sensorCal.sensor_type != MOTOR_SENSOR_ENCODER_AB)
        return;

    uint16_t reverse_counts[16];
    for (uint8_t i = 0U; i < 16U; ++i) {
        reverse_counts[i] = (uint16_t)(state->encoder_transition_counts[i] -
                                       sensorCal.encoder_transition_mid[i]);
    }

    const uint32_t fwd_pos = sensor_cal_encoder_sequence_score_counts(
        sensorCal.encoder_transition_mid, seq_positive_raw);
    const uint32_t fwd_neg = sensor_cal_encoder_sequence_score_counts(
        sensorCal.encoder_transition_mid, seq_negative_raw);
    const uint32_t rev_pos = sensor_cal_encoder_sequence_score_counts(
        reverse_counts, seq_positive_raw);
    const uint32_t rev_neg = sensor_cal_encoder_sequence_score_counts(
        reverse_counts, seq_negative_raw);

    /* If raw encoder direction agrees with commanded electrical direction,
     * forward should be positive-cycle and reverse negative-cycle. If A/B is
     * swapped the two relationships reverse. This is the integer/quadrature
     * analogue of VESC's +120/-120 degree encoder inversion measurement. */
    const uint32_t normal_score = fwd_pos + rev_neg;
    const uint32_t inverted_score = fwd_neg + rev_pos;
    sensorCal.encoder_direction_normal_score = normal_score;
    sensorCal.encoder_direction_inverted_score = inverted_score;

    const uint32_t winner = normal_score > inverted_score ? normal_score : inverted_score;
    const uint32_t loser = normal_score > inverted_score ? inverted_score : normal_score;
    const uint32_t margin = winner > loser ? winner - loser : 0U;
    if (winner >= 8U && margin >= 4U && (loser == 0U || winner * 4U >= loser * 5U)) {
        sensorCal.detected_encoder_inverted = inverted_score > normal_score ? 1U : 0U;
        sensorCal.encoder_direction_proved = true;
        return;
    }

    /* Secondary proof for a heavily loaded wheel: the end point of the three
     * commanded forward electrical revolutions can be only a few encoder counts.
     * Use it only when there is at least a two-count signed displacement. */
    int32_t raw_forward_delta = sensorCal.encoder_forward_delta;
    if (sensorCal.original_sensor_inverted) raw_forward_delta = -raw_forward_delta;
    if (raw_forward_delta >= 2 || raw_forward_delta <= -2) {
        sensorCal.detected_encoder_inverted = raw_forward_delta < 0 ? 1U : 0U;
        sensorCal.encoder_direction_proved = true;
    }
}

static bool sensor_cal_finalize_encoder_sequence(MotorRuntimeConfig *cfg, MotorSensorState *state)
{
    /* The only two legal quadrature cycles when normalized to 00. */
    static const uint8_t seq_negative_raw[4] = {0U, 1U, 3U, 2U};
    static const uint8_t seq_positive_raw[4] = {0U, 2U, 3U, 1U};
    bool complete_negative = false;
    bool complete_positive = false;
    const uint32_t negative_score = sensor_cal_encoder_sequence_score(
        state, seq_negative_raw, &complete_negative);
    const uint32_t positive_score = sensor_cal_encoder_sequence_score(
        state, seq_positive_raw, &complete_positive);
    sensorCal.encoder_negative_score = negative_score;
    sensorCal.encoder_positive_score = positive_score;

    if (!complete_negative && !complete_positive) return false;

    if (sensorCal.vesc_wire_detect && sensorCal.encoder_probe_valid == 0U) {
        sensor_cal_encoder_direction_proof(state, seq_negative_raw, seq_positive_raw);
    }

    if (sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO && !sensorCal.vesc_wire_detect) {
        /* Native software-decoded encoder commissioning still requires a
         * dominant direction because its transition LUT participates directly
         * in runtime counting. */
        if (negative_score == positive_score) return false;
        const uint32_t winner = positive_score > negative_score ? positive_score : negative_score;
        const uint32_t loser = positive_score > negative_score ? negative_score : positive_score;
        if (winner < 8U || (loser != 0U && winner < (loser * 2U))) return false;
    } else if (sensorCal.vesc_wire_detect) {
        /* LEFT hardware encoder count is authoritative from TIM4. The raw A/B
         * transition table is a wiring/noise proof, not the position integrator.
         * A loaded motor can legitimately dither both directions during phase
         * lock, so do not reject a clean quadrature source merely because the
         * forward/reverse vote is balanced. */
        const uint32_t valid_edges = state->encoder_valid_edges - sensorCal.encoder_session_valid_edges_start;
        const uint32_t invalid_edges = state->encoder_invalid_transitions -
            sensorCal.encoder_session_invalid_transitions_start;
        if (valid_edges < 8U || invalid_edges > (2U + valid_edges / 16U)) return false;
    }

    const uint8_t *selected;
    if (positive_score > negative_score) selected = seq_positive_raw;
    else if (negative_score > positive_score) selected = seq_negative_raw;
    else selected = seq_positive_raw; /* balanced dither: TIM4 + sensor_inverted owns direction */
    if (!MotorRuntimeConfig_SetEncoderSequence(cfg, selected)) return false;
    return true;
}

static uint16_t sensor_cal_measured_encoder_offset_deg(const MotorRuntimeConfig *cfg,
                                                        uint8_t ratio,
                                                        uint8_t inverted)
{
    if (cfg == NULL || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN || ratio == 0U)
        return 0U;
    int64_t count = sensorCal.encoder_zero_raw_count;
    const int64_t cpr = cfg->encoder_cpr;
    count %= cpr;
    if (count < 0) count += cpr;
    if (inverted && count != 0) count = cpr - count;
    /* VESC encoder equation is theta_e = theta_m*ratio - offset. At the D-axis
     * lock theta_e=0, so offset is the measured encoder electrical phase. */
    const uint64_t num = (uint64_t)count * (uint64_t)ratio * 360ULL;
    return (uint16_t)(((num + (uint64_t)cpr / 2ULL) / (uint64_t)cpr) % 360ULL);
}

static void sensor_cal_finish(uint8_t terminal_state, uint8_t result_code)
{
    MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
    MotorSensorState *state = sensor_cal_sensor_state(sensorCal.motor);
    mc_configuration *params = sensor_cal_params(sensorCal.motor);
    MotorRuntimeConfig candidate_config = *cfg;

    bool success = false;
    bool apply_candidate = false;
    bool manual_candidate = false;

    if (terminal_state == ESC_SENSOR_CAL_SUCCESS) {
        if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
            success = (sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO)
                ? sensor_cal_finalize_auto_hall(&candidate_config)
                : sensor_cal_finalize_manual_hall(&candidate_config, state);
            if (success && sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO) {
                /* Auto sweep phase listrik meningkat: hasil vote adalah mapping
                 * RAW Hall -> electrical sector FOC-positive yang absolut. */
                candidate_config.sensor_inverted = 0U;
                candidate_config.hall_calibrated = 1U;
                apply_candidate = true;
            } else if (success) {
                /* Manual PWM-OFF hanya diagnostic sequence. Jangan pernah mengganti
                 * active LUT/proof karena arah putaran tangan bukan arah +Q FOC. */
                memcpy(sensorCal.observed_hall_sequence,
                       candidate_config.hall_sequence,
                       sizeof(candidate_config.hall_sequence));
                sensorCal.observed_hall_count = 6U;
                manual_candidate = true;
            }
        } else if (sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB) {
            const bool sequence_ok = sensor_cal_finalize_encoder_sequence(&candidate_config, state);
            const int32_t normalized_delta = state->position_ticks - sensorCal.encoder_start;
            sensorCal.encoder_delta = normalized_delta;

            if (sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO) {
                int64_t mag = normalized_delta;
                if (mag < 0) mag = -mag;
                if (sensorCal.vesc_wire_detect) {
                    /* VESC detect: CPR must already describe the physical A/B
                     * encoder. Auto-detect ratio/pole-pairs and raw direction;
                     * incremental electrical zero is re-established safely at
                     * each power-on/first closed-loop run. */
                    if (sequence_ok &&
                        candidate_config.encoder_cpr >= MOTOR_ENCODER_CPR_MIN &&
                        sensorCal.detected_pole_pairs >= 1U &&
                        sensorCal.detected_pole_pairs <= SENSOR_CAL_VESC_POLE_PAIRS_MAX) {
                        candidate_config.sensor_inverted = sensorCal.detected_encoder_inverted;
                        candidate_config.encoder_ratio = sensorCal.detected_pole_pairs;
                        sensorCal.detected_encoder_offset_deg =
                            sensor_cal_measured_encoder_offset_deg(&candidate_config,
                                sensorCal.detected_pole_pairs, sensorCal.detected_encoder_inverted);
                        candidate_config.encoder_offset_deg = 0U;
                        candidate_config.encoder_calibrated = 1U;
                        success = true;
                        apply_candidate = true;
                    }
                } else if (sequence_ok && mag >= MOTOR_ENCODER_CPR_MIN && mag <= MOTOR_ENCODER_CPR_MAX) {
                    /* Native Auto Detect keeps its historical mode: configured
                     * pole-pair count defines one mechanical revolution and the
                     * measured magnitude becomes CPR. */
                    candidate_config.sensor_inverted = 0U;
                    candidate_config.encoder_cpr = (uint16_t)mag;
                    candidate_config.encoder_offset_deg = 0U;
                    candidate_config.encoder_ratio = params->foc_motor_pole_pairs;
                    candidate_config.encoder_calibrated = 1U;
                    success = true;
                    apply_candidate = true;
                }
            } else {
                int64_t mag = normalized_delta;
                if (mag < 0) mag = -mag;
                const bool configured_cpr_valid =
                    candidate_config.encoder_cpr >= MOTOR_ENCODER_CPR_MIN;
                if (sequence_ok && mag >= 4 && configured_cpr_valid) {
                    /* Sama seperti Hall manual: sequence A/B valid untuk wiring/
                     * noise diagnosis, tetapi tidak membuktikan +Q FOC. */
                    memcpy(sensorCal.observed_encoder_sequence,
                           candidate_config.encoder_sequence,
                           sizeof(candidate_config.encoder_sequence));
                    sensorCal.observed_encoder_count = 4U;
                    success = true;
                    manual_candidate = true;
                }
            }
        }
    }

    sensor_cal_stop_output();

    if (success && apply_candidate) {
        /* Commit hanya hasil AUTO yang memiliki proof arah/phase FOC. */
        settingsDirty = true;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *cfg = candidate_config;
        /* VESC encoder detect returns foc_encoder_ratio. Do NOT overwrite the
         * physical motor pole count (si_motor_poles/2): those are independent
         * configuration fields and forcing one from the other caused VESC Tool
         * pole edits to be truncated back to the detected encoder ratio. */
        apply_sensor_backend_to_foc(cfg, params);
        MotorSensor_Reset(state);
        MotorSensor_PrepareRuntime(cfg, state, params->foc_motor_pole_pairs);
        if (sensorCal.motor == ESC_MOTOR_RIGHT) odom_r = 0; else odom_l = 0;
        if (sensorCal.motor == ESC_MOTOR_RIGHT) {
            mc_foc_init(&motorRight, &motorConfRight);
        } else {
            mc_foc_init(&motorLeft, &motorConfLeft);
        }
        if (primask == 0U) __enable_irq();
        if (sensorCal.motor == ESC_MOTOR_RIGHT) {
            encoderAlignedRight = false;
            positionSessionZeroValidRight = false;
            sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);
            if (cfg->sensor_type == MOTOR_SENSOR_HALL_UVW) position_session_rezero(false);
        } else {
            encoderAlignedLeft = false;
            positionSessionZeroValidLeft = false;
            sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);
        }
        sensorCal.state = ESC_SENSOR_CAL_SUCCESS;
        sensorCal.result_code = 0U;

        /* AUTO Detect adalah transaksi commissioning lengkap: reset -> measure ->
         * validate -> commit RAM -> persist EEPROM -> read-back/CRC verify. Karena
         * output sudah dihentikan dan state bukan RUNNING, Save aman dilakukan di
         * sini. Jika flash verify gagal, RAM tetap berisi hasil baru tetapi status
         * eksplisit PERSIST_FAILED dan settingsDirty tetap true. */
        if (sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO) {
            eepromOk = RuntimeSettings_Save();
            if (!eepromOk) {
                settingsDirty = true;
                sensorCal.state = ESC_SENSOR_CAL_PERSIST_FAILED;
            }
        }
    } else if (success && manual_candidate) {
        /* Diagnostic manual berhasil tetapi ACTIVE RAM tidak disentuh. Ini juga
         * berarti posisi/odometry tidak tiba-tiba reset setelah diputar tangan. */
        sensorCal.state = ESC_SENSOR_CAL_SUCCESS;
        sensorCal.result_code = 0U;
    } else {
        /* Failed calibration tidak boleh menimpa LUT/CPR aktif dengan partial
         * observation. Untuk AUTO, proof sudah dicabut saat start dan tetap OFF;
         * untuk MANUAL, active proof/config sebelumnya tetap utuh. */
        sensorCal.state = (terminal_state == ESC_SENSOR_CAL_SUCCESS)
            ? ESC_SENSOR_CAL_FAILED_SEQUENCE : terminal_state;
        sensorCal.result_code = result_code ? result_code :
            (sensorCal.motion_detected ? 1U : ESC_SENSOR_CAL_RESULT_NO_MOTION);
    }

    sensorCalTelemetryCandidate = manual_candidate || !success;

    /* V14: freeze VESC wire terminal result before another motor can reuse the
     * single live commissioning workspace. */
    if (sensorCal.vesc_wire_detect) vesc_detect_store_terminal_snapshot();

    /* Standard COMM_DETECT_ENCODER result is already frozen above. Board-specific
     * steering commissioning continues asynchronously so VESC Tool is not held
     * without a reply during right-stop -> left-stop -> center travel. */
    if (success && sensorCal.vesc_wire_detect &&
        sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO &&
        sensorCal.motor == ESC_MOTOR_LEFT &&
        sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB &&
        sensorCal.state == ESC_SENSOR_CAL_SUCCESS) {
        (void)RuntimeControl_StartHomingCalibration(true);
    }

    /* Selalu antrekan satu final read-back setelah state terminal. */
    telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;
    configOneShotPending = true;
}

static bool sensor_cal_start(uint8_t method, uint8_t motor, int16_t detect_current_internal, uint16_t manual_ms)
{
    if (motor != ESC_MOTOR_LEFT && motor != ESC_MOTOR_RIGHT) return false;
    if (method != ESC_SENSOR_CAL_METHOD_AUTO && method != ESC_SENSOR_CAL_METHOD_MANUAL) return false;
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || homingActiveMask != 0U || encoderAlign.active) return false;

    armRequestedLeft = false;
    armRequestedRight = false;
    disarm_outputs();
    /* ARM reject lama dapat meninggalkan buzzer latch ~1.2 s. Main loop lama
     * memaksa enable=0 selama latch ini dan secara tidak sengaja mematikan PWM
     * Auto Detect. Commissioning memakai forced electrical phase dengan proteksi
     * arus sendiri; hapus latch lama sebelum mengaktifkan override SVPWM. */
    fault_report_reset_all();
    sensorCalibrationFastCurrentFaultMask = 0U;
    MotorControl_ClearCommissioningFaultSnapshot(motor == ESC_MOTOR_LEFT);
    memset(&sensorCal, 0, sizeof(sensorCal));
    sensorCal.state = ESC_SENSOR_CAL_RUNNING;
    sensorCalTelemetryCandidate = true;
    sensorCal.method = method;
    sensorCal.motor = motor;
    sensorCal.sensor_type = sensor_cal_config(motor)->sensor_type;
    sensorCal.sweep_direction = 1;
    sensorCal.original_sensor_inverted = sensor_cal_config(motor)->sensor_inverted ? 1U : 0U;
    sensorCal.start_tick = RuntimeControl_MonotonicMs();
    sensorCal.run_start_tick = sensorCal.start_tick;
    sensorCal.manual_duration_ms = (manual_ms >= 1000U && manual_ms <= 15000U)
        ? manual_ms : SENSOR_CAL_MANUAL_DEFAULT_MS;
    sensorCal.requested_drive_current_internal = clamp_i16(
        detect_current_internal, SENSOR_CAL_CURRENT_MIN_INTERNAL, SENSOR_CAL_CURRENT_MAX_INTERNAL);
    /* Auto Detect melakukan ramp alignment dari command kecil agar rotor tidak
     * hanya tersentak akibat D-axis step. Manual tidak menyalakan PWM. */
    sensorCal.drive_current_internal = (method == ESC_SENSOR_CAL_METHOD_AUTO)
        ? (int16_t)((sensorCal.requested_drive_current_internal < SENSOR_CAL_ALIGN_START_CURRENT_INTERNAL)
            ? sensorCal.requested_drive_current_internal : SENSOR_CAL_ALIGN_START_CURRENT_INTERNAL)
        : 0;
    MotorSensorState *cal_state = sensor_cal_sensor_state(motor);
    MotorSensorSample *cal_sample = sensor_cal_sensor_sample(motor);
    /* START selalu membuka tabel commissioning baru. Data sequence/counter dari
     * sesi sebelumnya tidak boleh ikut memberi vote pada kalibrasi baru. */
    sensor_cal_reset_capture_tables(cal_state, cal_sample, true);
    sensorCal.encoder_session_valid_edges_start = cal_state->encoder_valid_edges;
    sensorCal.encoder_session_invalid_transitions_start = cal_state->encoder_invalid_transitions;
    sensorCal.encoder_session_seen_mask = (uint8_t)(1U << (cal_sample->encoder_ab & 0x03U));
    sensorCal.last_motion_tick = sensorCal.start_tick;
    sensorCal.last_current_step_tick = sensorCal.start_tick;

    /* AUTO commissioning is transactional: keep the previously proven active
     * Hall/encoder configuration intact while the new candidate is measured.
     * The bridge is exclusively owned by forced-phase commissioning anyway, so
     * the old feedback mapping cannot steer this sweep. Only sensor_cal_finish()
     * commits/replaces the active proof after the new candidate succeeds. A
     * timeout therefore never destroys a previously working calibration. */
    if (method == ESC_SENSOR_CAL_METHOD_AUTO) {
        sensor_cal_set_current_override(motor, true, 0, sensorCal.drive_current_internal);
        if (motor == ESC_MOTOR_RIGHT) controlModeRightFoc = (uint8_t)CONTROL_MODE_CURRENT;
        else controlModeLeftFoc = (uint8_t)CONTROL_MODE_CURRENT;
        refresh_master_enable();
    } else {
        sensor_cal_set_current_override(motor, false, 0, 0);
        refresh_master_enable();
    }
    return true;
}

bool RuntimeControl_VescStartSensorDetect(bool left, uint8_t sensor_type, int16_t detect_current_internal)
{
    if (sensor_type != MOTOR_SENSOR_HALL_UVW && sensor_type != MOTOR_SENSOR_ENCODER_AB) return false;
    if (!left && sensor_type != MOTOR_SENSOR_HALL_UVW) return false; /* RIGHT is Hall-only. */
    const uint8_t motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    vescDetectTerminal[motor == ESC_MOTOR_RIGHT ? 1U : 0U].valid = false;
    /* Sensor commissioning is meaningless when zero-current calibration is not
     * valid. In that condition Id/Iq cannot be trusted and alignment current
     * protection would be operating on a false baseline. */
    if (!MotorControl_CurrentOffsetsValid()) {
        sensor_cal_start_rejected(ESC_SENSOR_CAL_METHOD_AUTO, motor);
        sensorCal.sensor_type = sensor_type;
        sensorCal.vesc_wire_detect = true;
        sensorCal.result_code = ESC_SENSOR_CAL_RESULT_START_REJECTED;
        vesc_detect_store_terminal_snapshot();
        return false;
    }
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    MotorSensorState *state = left ? &motorSensorStateLeft : &motorSensorStateRight;

    /* VESC Tool detection is a commissioning transaction. Select the requested
     * physical backend before starting the sweep, but keep RIGHT hard-locked Hall. */
    if (left) {
        cfg->sensor_type = sensor_type;
        LeftEncoder_SetMode(sensor_type == MOTOR_SENSOR_ENCODER_AB);
    } else {
        cfg->sensor_type = MOTOR_SENSOR_HALL_UVW;
        cfg->sensor_inverted = 0U;
    }
    apply_sensor_backend_to_foc(cfg, conf);
    MotorSensor_Reset(state);
    MotorSensor_PrepareRuntime(cfg, state, conf->foc_motor_pole_pairs);

    if (sensor_type == MOTOR_SENSOR_ENCODER_AB && cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN)
        return false;
    const bool started = sensor_cal_start(ESC_SENSOR_CAL_METHOD_AUTO, motor, detect_current_internal, 0U);
    if (started) sensorCal.vesc_wire_detect = true;
    return started;
}

static void vesc_detect_hall_table(const MotorRuntimeConfig *cfg, uint8_t table[8])
{
    memset(table, 255, 8U);
    if (cfg == NULL || cfg->hall_calibrated == 0U ||
        !MotorRuntimeConfig_HallSequenceValid(cfg->hall_sequence)) return;

    for (uint8_t sector = 0U; sector < 6U; ++sector) {
        const uint8_t raw = cfg->hall_sequence[sector] & 0x07U;
        if (raw > 0U && raw < 7U) {
            /* Same wire semantics as VESC mcpwm_foc_hall_detect(): angle*200/360.
             * This estimator stores calibrated 60-degree sectors, so report the
             * sector centre (30,90,... electrical degrees). */
            table[raw] = (uint8_t)(((uint32_t)(2U * sector + 1U) * 200U) / 12U);
        }
    }
}

static void vesc_detect_store_terminal_snapshot(void)
{
    const uint8_t idx = sensorCal.motor == ESC_MOTOR_RIGHT ? 1U : 0U;
    VescDetectTerminalSnapshot *snap = &vescDetectTerminal[idx];
    const MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
    const mc_configuration *conf = sensor_cal_params(sensorCal.motor);

    memset(snap, 0, sizeof(*snap));
    snap->valid = true;
    snap->encoder_ratio_fallback_used = sensorCal.encoder_ratio_fallback_used;
    snap->result.state = sensorCal.state;
    snap->result.result_code = sensorCal.result_code;
    snap->result.sensor_type = sensorCal.sensor_type;
    snap->result.encoder_cpr = cfg->encoder_cpr;
    snap->result.encoder_offset_deg = 0U;
    snap->result.encoder_inverted = cfg->sensor_inverted ? 1U : 0U;
    snap->result.pole_pairs = (sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB && cfg->encoder_ratio != 0U)
        ? cfg->encoder_ratio : conf->foc_motor_pole_pairs;
    if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW &&
        sensorCal.measured_hall_table_valid) {
        memcpy(snap->result.hall_table, sensorCal.measured_hall_table, 8U);
    } else {
        vesc_detect_hall_table(cfg, snap->result.hall_table);
    }
}

bool RuntimeControl_VescPollSensorDetect(bool left, uint8_t sensor_type, RuntimeVescDetectResult *result)
{
    if (result == NULL) return false;
    const uint8_t idx = left ? 0U : 1U;

    /* While THIS motor owns the live transaction there is intentionally no
     * terminal reply yet. A different motor's live/terminal state must never
     * satisfy this poll. */
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING &&
        sensorCal.motor == (left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT)) return false;

    const VescDetectTerminalSnapshot *snap = &vescDetectTerminal[idx];
    if (!snap->valid || snap->result.sensor_type != sensor_type) return false;
    *result = snap->result;
    return true;
}

bool RuntimeControl_VescGetLastSensorDetect(bool left, RuntimeVescDetectResult *result,
                                            bool *encoder_ratio_fallback_used)
{
    const VescDetectTerminalSnapshot *snap = &vescDetectTerminal[left ? 0U : 1U];
    if (!snap->valid) return false;
    if (result != NULL) *result = snap->result;
    if (encoder_ratio_fallback_used != NULL)
        *encoder_ratio_fallback_used = snap->encoder_ratio_fallback_used;
    return true;
}

static void sensor_cal_start_rejected(uint8_t method, uint8_t motor)
{
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING) return;
    memset(&sensorCal, 0, sizeof(sensorCal));
    sensorCal.state = ESC_SENSOR_CAL_ABORTED;
    sensorCal.method = method;
    sensorCal.motor = motor;
    sensorCal.sensor_type = sensor_cal_config(motor)->sensor_type;
    sensorCal.result_code = ESC_SENSOR_CAL_RESULT_START_REJECTED;
    sensorCalTelemetryCandidate = true;
    telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;
    configOneShotPending = true;
}

static void sensor_cal_record_transition(uint8_t raw)
{
    if (raw == 0U || raw == 7U || raw > 7U) {
        ++sensorCal.invalid_samples;
        return;
    }
    ++sensorCal.samples;
    bool known = false;
    for (uint8_t i = 0U; i < sensorCal.observed_hall_count; ++i) {
        if (sensorCal.observed_hall_sequence[i] == raw) {
            known = true;
            break;
        }
    }
    if (!known && sensorCal.observed_hall_count < 6U)
        sensorCal.observed_hall_sequence[sensorCal.observed_hall_count++] = raw;
    if (sensorCal.previous_raw != 0U && sensorCal.previous_raw != 7U &&
        sensorCal.previous_raw != raw) {
        sensorCal.motion_detected = true;
    }
    sensorCal.previous_raw = raw;
}

static void sensor_cal_encoder_probe_observe(MotorSensorState *state,
                                             const MotorSensorSample *sample)
{
    ++sensorCal.samples;
    const uint8_t ab = sample->encoder_ab & 0x03U;
    sensorCal.encoder_session_seen_mask |= (uint8_t)(1U << ab);
    bool known = false;
    for (uint8_t i = 0U; i < sensorCal.observed_encoder_count; ++i) {
        if (sensorCal.observed_encoder_sequence[i] == ab) { known = true; break; }
    }
    if (!known && sensorCal.observed_encoder_count < 4U)
        sensorCal.observed_encoder_sequence[sensorCal.observed_encoder_count++] = ab;
    if (state->position_ticks != sensorCal.encoder_start) sensorCal.motion_detected = true;
}

static void sensor_cal_encoder_probe_record(int8_t commanded_direction)
{
    const MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
    const int32_t raw_now = LeftEncoder_GetCount();
    const int32_t delta = raw_now - sensorCal.encoder_probe_start_raw;
    sensorCal.encoder_probe_start_raw = raw_now;
    const uint32_t mag = (uint32_t)(delta < 0 ? -delta : delta);
    if (mag < 3U || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN) return;

    const uint32_t den = 3U * mag;
    const uint32_t ratio = ((uint32_t)cfg->encoder_cpr + den / 2U) / den;
    if (ratio < 1U || ratio > SENSOR_CAL_VESC_POLE_PAIRS_MAX) return;

    ++sensorCal.encoder_probe_valid;
    sensorCal.encoder_probe_ratio_sum =
        (uint16_t)(sensorCal.encoder_probe_ratio_sum + ratio);
    if (sensorCal.encoder_probe_ratio_min == 0U || ratio < sensorCal.encoder_probe_ratio_min)
        sensorCal.encoder_probe_ratio_min = (uint8_t)ratio;
    if (ratio > sensorCal.encoder_probe_ratio_max)
        sensorCal.encoder_probe_ratio_max = (uint8_t)ratio;

    const bool same = (delta > 0 && commanded_direction > 0) ||
                      (delta < 0 && commanded_direction < 0);
    if (same) sensorCal.encoder_direction_normal_score += mag;
    else sensorCal.encoder_direction_inverted_score += mag;
}

static bool sensor_cal_service_vesc_encoder_probe(uint32_t now, uint32_t dt_ms,
                                                  MotorSensorState *state,
                                                  const MotorSensorSample *sample)
{
    if (sensorCal.sensor_type != MOTOR_SENSOR_ENCODER_AB ||
        !sensorCal.vesc_wire_detect || sensorCal.motor != ESC_MOTOR_LEFT)
        return false;

    sensor_cal_encoder_probe_observe(state, sample);
    if (!sensorCal.encoder_probe_initialized) {
        sensorCal.encoder_probe_initialized = true;
        sensorCal.encoder_probe_start_raw = LeftEncoder_GetCount();
        sensorCal.encoder_probe_stage = 0U;
        sensorCal.encoder_probe_total = 0U;
        sensorCal.encoder_probe_valid = 0U;
        sensorCal.encoder_probe_ratio_sum = 0U;
        sensorCal.encoder_probe_ratio_min = 0U;
        sensorCal.encoder_probe_ratio_max = 0U;
        sensorCal.encoder_direction_normal_score = 0U;
        sensorCal.encoder_direction_inverted_score = 0U;
        sensorCal.encoder_probe_hold_until = 0U;
        sensorCal.phase_q4 = 0;
    }

    static const int16_t targets[4] = {
        SENSOR_CAL_ENCODER_PROBE_Q4, 0,
        -SENSOR_CAL_ENCODER_PROBE_Q4, 0
    };
    static const int8_t directions[4] = {1, -1, -1, 1};
    const uint8_t stage = sensorCal.encoder_probe_stage & 3U;
    const int16_t target = targets[stage];

    if (sensorCal.encoder_probe_hold_until != 0U) {
        sensor_cal_set_current_override(sensorCal.motor, true, sensorCal.phase_q4,
                                        sensorCal.requested_drive_current_internal);
        refresh_master_enable();
        if ((int32_t)(now - sensorCal.encoder_probe_hold_until) < 0) return true;

        sensor_cal_encoder_probe_record(directions[stage]);
        if (directions[stage] > 0) {
            if (sensorCal.forward_cycles != UINT8_MAX) ++sensorCal.forward_cycles;
        } else {
            if (sensorCal.reverse_cycles != UINT8_MAX) ++sensorCal.reverse_cycles;
        }
        if (sensorCal.completed_cycles != UINT16_MAX) ++sensorCal.completed_cycles;
        if (sensorCal.encoder_probe_total != UINT8_MAX) ++sensorCal.encoder_probe_total;
        sensorCal.encoder_probe_stage = (uint8_t)((stage + 1U) & 3U);
        sensorCal.encoder_probe_hold_until = 0U;

        const bool enough =
            sensorCal.encoder_probe_total >= SENSOR_CAL_ENCODER_PROBE_EARLY_COUNT &&
            sensorCal.encoder_probe_valid >= SENSOR_CAL_ENCODER_PROBE_MIN_VALID;
        const bool exhausted = sensorCal.encoder_probe_total >= SENSOR_CAL_ENCODER_PROBE_MAX_COUNT;
        if (enough || exhausted) {
            MotorRuntimeConfig candidate = *sensor_cal_config(sensorCal.motor);
            const bool sequence_ok = sensor_cal_finalize_encoder_sequence(&candidate, state);
            uint8_t ratio = 0U;
            bool ratio_ok = false;
            if (sensorCal.encoder_probe_valid != 0U) {
                ratio = (uint8_t)((sensorCal.encoder_probe_ratio_sum +
                    sensorCal.encoder_probe_valid / 2U) / sensorCal.encoder_probe_valid);
                uint8_t tolerance = ratio / 4U;
                if (tolerance < 2U) tolerance = 2U;
                ratio_ok = ratio >= 1U && ratio <= SENSOR_CAL_VESC_POLE_PAIRS_MAX &&
                    sensorCal.encoder_probe_ratio_max >= sensorCal.encoder_probe_ratio_min &&
                    (uint8_t)(sensorCal.encoder_probe_ratio_max -
                              sensorCal.encoder_probe_ratio_min) <= tolerance;
            }
            const uint32_t normal = sensorCal.encoder_direction_normal_score;
            const uint32_t inverted = sensorCal.encoder_direction_inverted_score;
            const uint32_t total = normal + inverted;
            const uint32_t diff = normal > inverted ? normal - inverted : inverted - normal;
            const bool direction_ok = total >= 8U && diff * 4U >= total;
            const uint32_t valid_edges = state->encoder_valid_edges -
                sensorCal.encoder_session_valid_edges_start;
            const uint32_t invalid_edges = state->encoder_invalid_transitions -
                sensorCal.encoder_session_invalid_transitions_start;
            const bool quadrature_ok = valid_edges >= 8U &&
                invalid_edges <= (2U + valid_edges / 16U);

            /* ABI direction/quadrature proof is independent from the coarse
             * electrical-ratio estimate. On the captured board the four 120-deg
             * probes produced 0 normal vs 364 inverted counts and zero invalid
             * quadrature transitions, but rounding yielded ratio 14 while the
             * physical motor is configured as 15 pole-pairs. Treat a +/-1 probe
             * estimate as validation of the authoritative motor pole count. */
            if (direction_ok && quadrature_ok) sensorCal.encoder_direction_proved = true;
            const MotorRuntimeConfig *active_cfg = sensor_cal_config(sensorCal.motor);
            uint8_t configured_pp = active_cfg != NULL ? active_cfg->encoder_ratio : 0U;
            if (configured_pp == 0U) configured_pp = sensorCal.motor == ESC_MOTOR_LEFT ? motorConfLeft.foc_motor_pole_pairs : motorConfRight.foc_motor_pole_pairs;
            const uint8_t ratio_err = ratio > configured_pp ? (uint8_t)(ratio - configured_pp) :
                                                        (uint8_t)(configured_pp - ratio);
            const bool ratio_near_config = configured_pp >= 1U && configured_pp <= SENSOR_CAL_VESC_POLE_PAIRS_MAX &&
                                           ratio >= 1U && ratio_err <= 1U;

            if (sequence_ok && direction_ok && quadrature_ok && enough && (ratio_ok || ratio_near_config)) {
                sensorCal.detected_pole_pairs = ratio_near_config ? configured_pp : ratio;
                sensorCal.detected_encoder_inverted = inverted > normal ? 1U : 0U;
                sensorCal.encoder_direction_proved = true;
                sensorCal.encoder_ratio_fallback_used = ratio_near_config && ratio != configured_pp;
                sensorCal.encoder_delta = state->position_ticks - sensorCal.encoder_start;
                sensor_cal_finish(ESC_SENSOR_CAL_SUCCESS, 0U);
            } else if (exhausted) {
                sensor_cal_finish(ESC_SENSOR_CAL_FAILED_SEQUENCE, 1U);
            }
        }
        return true;
    }

    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 50U) dt_ms = 50U;
    int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
    int32_t phase = sensorCal.phase_q4;
    bool reached = false;
    if (phase < target) {
        phase += step;
        if (phase >= target) { phase = target; reached = true; }
    } else if (phase > target) {
        phase -= step;
        if (phase <= target) { phase = target; reached = true; }
    } else {
        reached = true;
    }
    sensorCal.phase_q4 = (int16_t)phase;
    sensor_cal_set_current_override(sensorCal.motor, true, sensorCal.phase_q4,
                                    sensorCal.requested_drive_current_internal);
    refresh_master_enable();
    if (reached)
        sensorCal.encoder_probe_hold_until = now + SENSOR_CAL_ENCODER_PROBE_SETTLE_MS;
    return true;
}

static void sensor_cal_service(uint32_t now, uint32_t dt_ms)
{
    if (sensorCal.state != ESC_SENSOR_CAL_RUNNING) return;
    const uint32_t elapsed = now - sensorCal.start_tick;
    MotorSensorState *state = sensor_cal_sensor_state(sensorCal.motor);
    MotorSensorSample *sample = sensor_cal_sensor_sample(sensorCal.motor);
    const uint16_t calibration_current_limit =
        sensor_cal_current_limit_centi_amp(sensorCal.motor);

    if (sensorCal.method == ESC_SENSOR_CAL_METHOD_MANUAL) {
        if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
            sensor_cal_record_transition(sample->raw_hall_encoding);
        } else {
            /* Encoder manual calibration tidak menafsirkan A/B sebagai Hall 3-bit.
             * U/V dipolling pada ISR kontrol motor yang sama dengan Hall (~16 kHz);
             * manual 5 s mengamati arah dan jumlah transisi quadrature tersebut.
             * Urutan exact 4-state direkonstruksi dari directed transition counter
             * yang diisi ISR 16 kHz, bukan dari polling slow-loop ini. */
            const uint32_t edges =
                state->encoder_valid_edges - sensorCal.encoder_valid_edges_start;
            sensorCal.samples = (uint16_t)(edges > UINT16_MAX ? UINT16_MAX : edges);
            const uint32_t invalid =
                state->encoder_invalid_transitions -
                sensorCal.encoder_invalid_transitions_start;
            sensorCal.invalid_samples = (uint16_t)(invalid > UINT16_MAX ? UINT16_MAX : invalid);
            const uint8_t current_ab = sample->encoder_ab & 0x03U;
            bool known = false;
            for (uint8_t i = 0U; i < sensorCal.observed_encoder_count; ++i) {
                if (sensorCal.observed_encoder_sequence[i] == current_ab) {
                    known = true;
                    break;
                }
            }
            if (!known && sensorCal.observed_encoder_count < 4U)
                sensorCal.observed_encoder_sequence[sensorCal.observed_encoder_count++] = current_ab;
            if (edges != 0U) sensorCal.motion_detected = true;
        }
        if (elapsed >= sensorCal.manual_duration_ms) {
            sensor_cal_finish(ESC_SENSOR_CAL_SUCCESS, 0U);
        }
        return;
    }

    if (elapsed > SENSOR_CAL_AUTO_TIMEOUT_MS) {
        sensor_cal_finish(ESC_SENSOR_CAL_FAILED_TIMEOUT, 2U);
        return;
    }

    const uint8_t cal_bit = (sensorCal.motor == ESC_MOTOR_RIGHT) ? 0x02U : 0x01U;
    if ((sensorCalibrationFastCurrentFaultMask & cal_bit) != 0U) {
        /* The fast current guard already removed MOE because measured phase current
         * diverged grossly from the requested detect current. Treat this as a
         * commissioning failure, not as a runtime VESC fault/buzzer event. */
        sensor_cal_finish(ESC_SENSOR_CAL_OVERCURRENT, 3U);
        return;
    }

    const uint16_t current_centi_amp = sensor_cal_measured_current_centi_amp(sensorCal.motor);
    if (current_centi_amp > calibration_current_limit) {
        /* Current chopping di DMA ISR tetap memotong PWM secara instan. Di level
         * commissioning jangan abort karena satu spike alignment; kwalifikasi
         * over-current selama debounce panjang agar transient alignment tidak false-fail. */
        if (sensorCal.overcurrent_since_tick == 0U) sensorCal.overcurrent_since_tick = now;
        if ((uint32_t)(now - sensorCal.overcurrent_since_tick) >= SENSOR_CAL_OVERCURRENT_DEBOUNCE_MS) {
            sensor_cal_finish(ESC_SENSOR_CAL_OVERCURRENT, 3U);
            return;
        }
    } else {
        sensorCal.overcurrent_since_tick = 0U;
    }

    /* Alignment awal: medan D statis pada electrical angle 0. Untuk encoder
     * incremental, software-zero dilakukan setelah rotor sudah tertarik ke
     * referensi ini, sehingga CPR/offset dapat dipelajari tanpa index Z. */
    if (elapsed < SENSOR_CAL_AUTO_ALIGN_MS) {
        sensorCal.phase_q4 = 0;
        const int16_t start_v = (sensorCal.requested_drive_current_internal < SENSOR_CAL_ALIGN_START_CURRENT_INTERNAL)
            ? sensorCal.requested_drive_current_internal : SENSOR_CAL_ALIGN_START_CURRENT_INTERNAL;
        const int32_t span = (int32_t)sensorCal.requested_drive_current_internal - start_v;
        sensorCal.drive_current_internal = (int16_t)(start_v +
            (span * (int32_t)elapsed + (int32_t)(SENSOR_CAL_AUTO_ALIGN_MS / 2U)) /
            (int32_t)SENSOR_CAL_AUTO_ALIGN_MS);
        sensor_cal_set_current_override(sensorCal.motor, true, 0, sensorCal.drive_current_internal);
        refresh_master_enable();
        return;
    }
    if (!sensorCal.aligned_zero_done) {
        if (sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB) {
            const uint32_t primask = __get_PRIMASK();
            __disable_irq();
            /* Alignment listrik TIDAK boleh meng-zero-kan posisi mekanik. Simpan
             * count saat rotor terkunci pada electrical phase 0 sebagai referensi
             * theta_e terpisah; posisi tetap kontinu untuk POS/odometry/homing. */
            (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);
            sensorCal.encoder_zero_raw_count = LeftEncoder_GetCount();
            sensorCal.encoder_start = state->position_ticks;
            state->encoder_speed_reference_ticks = state->position_ticks;
            state->encoder_speed_window_count = 0U;
            state->encoder_speed_q4 = 0;
            state->encoder_speed_initialized = false;
            if (primask == 0U) __enable_irq();
        }
        /* Alignment dapat menimbulkan twitch/transisi yang bukan bagian sweep.
         * Buang semuanya dan mulai acquisition dari tabel 0 tepat di sini. */
        sensor_cal_reset_capture_tables(state, sample, true);
        sensorCal.aligned_zero_done = true;
        sensorCal.run_start_tick = now;
        sensorCal.phase_q4 = 0;
        sensorCal.sweep_direction = 1;
        sensorCal.forward_cycles = 0U;
        sensorCal.reverse_cycles = 0U;
        sensorCal.completed_cycles = 0U;
        sensorCal.drive_current_internal = sensorCal.requested_drive_current_internal;
        sensorCal.last_motion_tick = now;
    }

    /* Respect the current explicitly requested by VESC Tool. Do not silently
     * increase commissioning current above the user-selected value. Motion is
     * still observed for timeout/result diagnostics. */
    const int32_t motion_position = (int32_t)sensor_cal_motion_counter(state, sensorCal.sensor_type);
    if (motion_position != sensorCal.last_motion_position) {
        sensorCal.last_motion_position = motion_position;
        sensorCal.last_motion_tick = now;
        sensorCal.motion_detected = true;
    }
    sensorCal.drive_current_internal = sensorCal.requested_drive_current_internal;

    if (sensor_cal_service_vesc_encoder_probe(now, dt_ms, state, sample)) return;

    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 50U) dt_ms = 50U;
    const int32_t phase_step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
    int32_t phase = (int32_t)sensorCal.phase_q4 +
        ((sensorCal.sweep_direction < 0) ? -phase_step : phase_step);
    if (sensorCal.sweep_direction >= 0) {
        while (phase >= 5760) {
            phase -= 5760;
            if (sensorCal.completed_cycles < UINT16_MAX) ++sensorCal.completed_cycles;
            if (sensorCal.forward_cycles < UINT8_MAX) ++sensorCal.forward_cycles;
            const bool hall_reverse_now =
                sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW &&
                sensorCal.forward_cycles >= SENSOR_CAL_HALL_FORWARD_SWEEPS;
            const bool encoder_reverse_now =
                sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB && sensorCal.vesc_wire_detect &&
                sensorCal.forward_cycles >= SENSOR_CAL_NATIVE_ENCODER_FORWARD_SWEEPS;
            if (hall_reverse_now || encoder_reverse_now) {
                /* 0 deg and 360 deg are equivalent. Reverse from the same field
                 * direction instead of jumping to an unrelated electrical angle. */
                if (encoder_reverse_now) {
                    sensorCal.encoder_forward_delta = state->position_ticks - sensorCal.encoder_start;
                    const uint32_t primask = __get_PRIMASK();
                    __disable_irq();
                    memcpy(sensorCal.encoder_transition_mid,
                           state->encoder_transition_counts,
                           sizeof(sensorCal.encoder_transition_mid));
                    if (primask == 0U) __enable_irq();
                }
                phase = 5759;
                sensorCal.sweep_direction = -1;
                break;
            }
        }
    } else {
        while (phase < 0) {
            phase += 5760;
            if (sensorCal.completed_cycles < UINT16_MAX) ++sensorCal.completed_cycles;
            if (sensorCal.reverse_cycles < UINT8_MAX) ++sensorCal.reverse_cycles;
        }
    }
    sensorCal.phase_q4 = (int16_t)phase;
    sensor_cal_set_current_override(sensorCal.motor, true, sensorCal.phase_q4, sensorCal.drive_current_internal);
    refresh_master_enable();

    if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
        const uint8_t raw = sample->raw_hall_encoding;
        sensor_cal_record_transition(raw);
        /* Match VESC 6.00: accumulate commanded electrical angle for every
         * raw Hall state after the phase has settled for this slow-loop sample.
         * Include 000/111 so the finalizer can verify that exactly two codes are
         * absent rather than assuming them invalid. */
        sensor_cal_record_hall_angle(raw);
        const uint8_t target_cycles = sensor_cal_target_cycles(
            sensorCal.motor, MOTOR_SENSOR_HALL_UVW);
        if (sensorCal.completed_cycles >= target_cycles) {
            MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
            if (sensor_cal_auto_candidate_ready(cfg, state)) {
                sensor_cal_finish(ESC_SENSOR_CAL_SUCCESS, 0U);
            } else {
                /* Window belum valid: jangan finalize menjadi FAILED_SEQUENCE.
                 * Reset tabel dan ukur ulang sweep berikutnya sampai timeout. */
                const bool motion_seen = sensorCal.motion_detected;
                sensor_cal_reset_capture_tables(state, sample, true);
                sensorCal.motion_detected = motion_seen;
                sensorCal.sweep_direction = 1;
                sensorCal.forward_cycles = 0U;
                sensorCal.reverse_cycles = 0U;
                sensorCal.phase_q4 = 0;
                sensorCal.last_motion_tick = now;
            }
        }
    } else {
        ++sensorCal.samples;
        const uint8_t current_ab = sample->encoder_ab & 0x03U;
        sensorCal.encoder_session_seen_mask |= (uint8_t)(1U << current_ab);
        bool known = false;
        for (uint8_t i = 0U; i < sensorCal.observed_encoder_count; ++i) {
            if (sensorCal.observed_encoder_sequence[i] == current_ab) {
                known = true;
                break;
            }
        }
        if (!known && sensorCal.observed_encoder_count < 4U)
            sensorCal.observed_encoder_sequence[sensorCal.observed_encoder_count++] = current_ab;
        if (state->position_ticks != sensorCal.encoder_start) sensorCal.motion_detected = true;
        const uint8_t target_cycles = sensor_cal_target_cycles(
            sensorCal.motor, MOTOR_SENSOR_ENCODER_AB);
        if (sensorCal.completed_cycles >= target_cycles) {
            MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
            sensorCal.encoder_delta = state->position_ticks - sensorCal.encoder_start;
            if (sensor_cal_auto_candidate_ready(cfg, state)) {
                sensor_cal_finish(ESC_SENSOR_CAL_SUCCESS, 0U);
            } else {
                /* V21: every retry is a complete +3/-3 electrical sweep again.
                 * The 17:49 trace proved V20/V21-pre left sweep_direction=-1 after
                 * the first failed window, creating reverse-only retries. Reset the
                 * per-window transition counters/direction snapshot while keeping
                 * transaction-wide valid-edge and seen-state evidence intact. */
                const bool motion_seen = sensorCal.motion_detected;
                sensor_cal_reset_capture_tables(state, sample, false);
                sensorCal.motion_detected = motion_seen;
                sensorCal.sweep_direction = 1;
                sensorCal.forward_cycles = 0U;
                sensorCal.reverse_cycles = 0U;
                sensorCal.phase_q4 = 0;
                /* encoder_session_* baselines and encoder_session_seen_mask are
                 * intentionally NOT reset by the per-window helper. */
                sensorCal.last_motion_position = (int32_t)sensor_cal_motion_counter(
                    state, MOTOR_SENSOR_ENCODER_AB);
                sensorCal.last_motion_tick = now;
            }
        }
    }
}

static void position_session_rezero(bool left)
{
    const MotorSensorState *state = left ? &motorSensorStateLeft : &motorSensorStateRight;
    if (left) {
        positionSessionZeroLeft = state->position_ticks;
        positionSessionZeroValidLeft = true;
    } else {
        positionSessionZeroRight = state->position_ticks;
        positionSessionZeroValidRight = true;
    }
}

static int32_t position_session_zero(bool left)
{
    SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    if (cal->calibrated != 0U && cal->homed != 0U) return cal->right_zero_ticks;
    bool *valid = left ? &positionSessionZeroValidLeft : &positionSessionZeroValidRight;
    int32_t *zero = left ? &positionSessionZeroLeft : &positionSessionZeroRight;
    if (!*valid) {
        position_session_rezero(left);
    }
    return *zero;
}

static int16_t encoder_alignment_target_current(uint8_t motor)
{
    const mc_configuration *conf = sensor_cal_params(motor);
    int32_t units = conf->foc_current_units_per_amp ?
        (int32_t)conf->foc_current_units_per_amp : 800;
    int32_t target = units; /* 1.00 A D-axis phase-0 lock */
    if (target < SENSOR_CAL_CURRENT_MIN_INTERNAL) target = SENSOR_CAL_CURRENT_MIN_INTERNAL;
    if (target > SENSOR_CAL_CURRENT_MAX_INTERNAL) target = SENSOR_CAL_CURRENT_MAX_INTERNAL;
    if (target > conf->l_current_max) target = conf->l_current_max;
    if (target < 1) target = 1;
    return (int16_t)target;
}

static void encoder_alignment_abort(void)
{
    if (!encoderAlign.active) return;
    const uint8_t motor = encoderAlign.motor;
    sensor_cal_set_current_override(motor, false, 0, 0);
    encoderAlign.active = false;
    refresh_master_enable();
}

static int16_t encoder_alignment_probe_delta_s16(int32_t delta)
{
    if (delta > INT16_MAX) return INT16_MAX;
    if (delta < INT16_MIN) return INT16_MIN;
    return (int16_t)delta;
}

static bool encoder_alignment_probe_matches(const MotorRuntimeConfig *cfg,
                                            int8_t direction, int32_t delta)
{
    if (cfg == NULL || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN ||
        cfg->encoder_ratio == 0U || direction == 0) return false;

    /* 120 electrical degrees = one third electrical revolution. Because the
     * fast runtime phase uses encoder_ratio, the same physical proof must move
     * approximately CPR/(3*ratio) normalized TIM4 counts. position_ticks is
     * already normalized by sensor_inverted in the 16-kHz hardware path. */
    const uint32_t den = 3U * (uint32_t)cfg->encoder_ratio;
    uint32_t expected = ((uint32_t)cfg->encoder_cpr + den / 2U) / den;
    if (expected < 2U) expected = 2U;

    int64_t d = delta;
    const bool sign_ok = direction > 0 ? d > 0 : d < 0;
    if (!sign_ok) return false;
    if (d < 0) d = -d;
    const uint32_t mag = d > UINT32_MAX ? UINT32_MAX : (uint32_t)d;

    /* Detect already proved the ratio. Startup only verifies that the rotor
     * follows that electrical field. Allow load/compliance, but reject a tiny
     * twitch or a count rate incompatible with the persisted ratio. */
    uint32_t lower = (expected * 55U) / 100U;
    if (lower < 2U) lower = 2U;
    const uint32_t upper = (expected * 145U) / 100U + 2U;
    return mag >= lower && mag <= upper;
}

static void encoder_alignment_fail(bool left, MotorSensorState *state)
{
    if (state != NULL) state->encoder_electrical_aligned = false;
    if (left) {
        encoderAlignedLeft = false;
        armRequestedLeft = false;
        armRejectLeft = ESC_ARM_REJECT_SENSOR_OR_FOC;
    } else {
        encoderAlignedRight = false;
        armRequestedRight = false;
        armRejectRight = ESC_ARM_REJECT_SENSOR_OR_FOC;
    }
    encoder_alignment_abort();
}

static void encoder_alignment_start(uint8_t motor)
{
    fault_report_reset_all();
    const uint8_t bit = motor == ESC_MOTOR_RIGHT ? 0x02U : 0x01U;
    sensorCalibrationFastCurrentFaultMask &= (uint8_t)~bit;
    MotorControl_ClearCommissioningFaultSnapshot(motor == ESC_MOTOR_LEFT);
    memset(&encoderAlign, 0, sizeof(encoderAlign));
    encoderAlign.active = true;
    encoderAlign.motor = motor;
    encoderAlign.stage = 0U;
    encoderAlign.start_tick = RuntimeControl_MonotonicMs();
    encoderAlign.stage_tick = encoderAlign.start_tick;
    encoderAlign.target_current_internal = encoder_alignment_target_current(motor);
    encoderAlign.phase_q4 = 0;
    encoderAlign.probe_direction = 1;

    int16_t start_current = (int16_t)(encoderAlign.target_current_internal / 2);
    if (start_current < SENSOR_CAL_CURRENT_MIN_INTERNAL)
        start_current = SENSOR_CAL_CURRENT_MIN_INTERNAL;
    if (start_current > encoderAlign.target_current_internal)
        start_current = encoderAlign.target_current_internal;
    sensor_cal_set_current_override(motor, true, 0, start_current);
    if (motor == ESC_MOTOR_RIGHT) controlModeRightFoc = (uint8_t)CONTROL_MODE_CURRENT;
    else controlModeLeftFoc = (uint8_t)CONTROL_MODE_CURRENT;
    refresh_master_enable();
}

static bool encoder_alignment_service(uint32_t now)
{
    if (!encoderAlign.active) return false;
    const bool left = encoderAlign.motor == ESC_MOTOR_LEFT;
    const uint8_t bit = left ? 0x01U : 0x02U;
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    MotorSensorState *state = left ? &motorSensorStateLeft : &motorSensorStateRight;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;

    if ((sensorCalibrationFastCurrentFaultMask & bit) != 0U ||
        (motorControlOvercurrentFaultMask & bit) != 0U ||
        cfg->sensor_type != MOTOR_SENSOR_ENCODER_AB ||
        cfg->encoder_calibrated == 0U || cfg->encoder_sequence_valid == 0U ||
        cfg->encoder_ratio == 0U || !left) {
        encoder_alignment_fail(left, state);
        return false;
    }

    /* Stage 0: establish a stable D-axis lock at commanded electrical phase 0.
     * Current ramps from about 0.5 A to 1.0 A, but phase is not accepted as the
     * encoder zero yet. A following motion probe must prove the rotor is coupled. */
    if (encoderAlign.stage == 0U) {
        const uint32_t elapsed = now - encoderAlign.start_tick;
        int32_t current = encoderAlign.target_current_internal;
        if (elapsed < 350U) {
            int32_t start = current / 2;
            if (start < SENSOR_CAL_CURRENT_MIN_INTERNAL) start = SENSOR_CAL_CURRENT_MIN_INTERNAL;
            current = start + ((current - start) * (int32_t)elapsed) / 350;
        }
        encoderAlign.phase_q4 = 0;
        sensor_cal_set_current_override(encoderAlign.motor, true, 0, (int16_t)current);
        refresh_master_enable();
        if (elapsed < 700U) return true;
        encoderAlign.probe_start_count = state->position_ticks;
        encoderAlign.stage = 1U;
        encoderAlign.stage_tick = now;
        return true;
    }

    /* Stage 1: slowly move the forced electrical field by +/-120 degrees.
     * This is slow-loop only (~200 Hz); ISR cadence/work remains unchanged. */
    if (encoderAlign.stage == 1U) {
        uint32_t dt_ms = now - encoderAlign.stage_tick;
        encoderAlign.stage_tick = now;
        if (dt_ms == 0U) dt_ms = 1U;
        if (dt_ms > 50U) dt_ms = 50U;
        int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
        const int32_t target = encoderAlign.probe_direction > 0
            ? SENSOR_CAL_ENCODER_PROBE_Q4 : -SENSOR_CAL_ENCODER_PROBE_Q4;
        int32_t phase = encoderAlign.phase_q4;
        if (phase < target) {
            phase += step;
            if (phase > target) phase = target;
        } else if (phase > target) {
            phase -= step;
            if (phase < target) phase = target;
        }
        encoderAlign.phase_q4 = (int16_t)phase;
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if (phase == target) {
            encoderAlign.stage = 2U;
            encoderAlign.stage_tick = now;
        }
        return true;
    }

    /* Stage 2: settle at +/-120, then compare normalized TIM4 displacement with
     * the encoder ratio already proven by COMM_DETECT_ENCODER. */
    if (encoderAlign.stage == 2U) {
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if ((now - encoderAlign.stage_tick) < SENSOR_CAL_ENCODER_PROBE_SETTLE_MS) return true;

        const int32_t delta = state->position_ticks - encoderAlign.probe_start_count;
        encoderAlign.probe_delta_counts = encoder_alignment_probe_delta_s16(delta);
        ++encoderAlign.probe_attempts;
        encoderAlign.direction_proved = encoder_alignment_probe_matches(
            cfg, encoderAlign.probe_direction, delta);
        if (!encoderAlign.direction_proved && encoderAlign.probe_attempts < 2U) {
            /* Positive probe can be blocked when steering already sits on that
             * mechanical stop. Return to phase 0 and retry the opposite direction. */
            encoderAlign.probe_direction = -1;
            encoderAlign.direction_changed = true;
        }
        encoderAlign.stage = 3U;
        encoderAlign.stage_tick = now;
        return true;
    }

    /* Stage 3: always return the stator field to exact electrical phase 0 before
     * either trying the opposite probe or committing the runtime zero. */
    if (encoderAlign.stage == 3U) {
        uint32_t dt_ms = now - encoderAlign.stage_tick;
        encoderAlign.stage_tick = now;
        if (dt_ms == 0U) dt_ms = 1U;
        if (dt_ms > 50U) dt_ms = 50U;
        int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
        int32_t phase = encoderAlign.phase_q4;
        if (phase > 0) {
            phase -= step;
            if (phase < 0) phase = 0;
        } else if (phase < 0) {
            phase += step;
            if (phase > 0) phase = 0;
        }
        encoderAlign.phase_q4 = (int16_t)phase;
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if (phase != 0) return true;

        if (encoderAlign.direction_proved) {
            encoderAlign.stage = 4U;
            encoderAlign.stage_tick = now;
            return true;
        }
        if (encoderAlign.probe_attempts < 2U) {
            encoderAlign.probe_start_count = state->position_ticks;
            encoderAlign.stage = 1U;
            encoderAlign.stage_tick = now;
            return true;
        }

        /* Neither direction produced the expected count/ratio relationship.
         * Never run DUTY/CURRENT/RPM/POS with a guessed electrical zero. */
        encoder_alignment_fail(left, state);
        return false;
    }

    /* Stage 4: phase is proven and back at zero. Hold briefly, then rebuild the
     * incremental phase accumulator at the current TIM4 ISR-owned snapshot and
     * define THIS boot's electrical zero. Nothing electrical is persisted. */
    sensor_cal_set_current_override(encoderAlign.motor, true, 0,
                                    encoderAlign.target_current_internal);
    refresh_master_enable();
    if ((now - encoderAlign.stage_tick) < 350U) return true;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    MotorSensor_Reset(state);
    MotorSensor_PrepareRuntime(cfg, state, conf->foc_motor_pole_pairs);
    const int32_t raw = LeftEncoder_GetCount(); /* passive ISR-owned snapshot */
    state->encoder_hw_last_count = raw;
    state->encoder_hw_count_initialized = true;
    state->position_ticks = 0;
    state->encoder_speed_reference_ticks = 0;
    state->encoder_speed_window_count = 0U;
    state->encoder_speed_q4 = 0;
    state->encoder_speed_initialized = false;
    (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);
    encoderAlignedLeft = true;
    if (primask == 0U) __enable_irq();

    position_session_rezero(true);
    sensor_cal_set_current_override(encoderAlign.motor, false, 0, 0);
    encoderAlign.active = false;
    refresh_master_enable();
    return false;
}

static void Homing_SetDefaults(void)
{
    /* Mechanical hard-stop homing is intentionally low-current. Search torque is
     * 10% of configured motor current (~1.5 A with the 15 A default), while the
     * stop proof requires >=1.2 A + near-zero speed + no encoder movement. */
    homingConfigLeft.current_threshold_centi_amp = 120U;
    homingConfigRight.current_threshold_centi_amp = 120U;
    homingConfigLeft.search_command = -100;
    homingConfigRight.search_command = -100;
    homingTimeoutMs = 10000U;
    homingDebounceMs = 200U;
    homingOnBootLeft = false;
    homingOnBootRight = false;
    bootHomingPendingMask = 0U;
    homingStartedFromBoot = false;
    memset(&homingRuntimeLeft, 0, sizeof(homingRuntimeLeft));
    memset(&homingRuntimeRight, 0, sizeof(homingRuntimeRight));
    homingRuntimeLeft.state = ESC_HOMING_IDLE;
    homingRuntimeRight.state = ESC_HOMING_IDLE;
    homingActiveMask = 0U;
}

static bool HomingConfig_IsValid(const HomingMotorConfig *cfg)
{
    if (cfg == NULL) return false;
    if (cfg->current_threshold_centi_amp < 50U || cfg->current_threshold_centi_amp > 30000U) return false;
    if (cfg->search_command == 0 || cfg->search_command < -500 || cfg->search_command > 500) return false;
    return true;
}

static bool HomingTiming_IsValid(uint16_t timeout_ms, uint16_t debounce_ms)
{
    return timeout_ms >= 500U && timeout_ms <= 30000U &&
           debounce_ms >= 20U && debounce_ms <= 2000U && debounce_ms < timeout_ms;
}


/* Default posisi sengaja konservatif. Pengguna dapat tuning dari GUI lalu Save EEPROM. */
static void PositionPid_SetDefaults(PositionPidConfig *cfg, mc_configuration *conf)
{
    if (cfg == NULL || conf == NULL) return;
    cfg->kp_q16 = 16384;
    cfg->ki_q16 = 0;
    cfg->kd_q16 = 0;
    cfg->position_min = POSITION_MIN_DEFAULT;
    cfg->position_max = POSITION_MAX_DEFAULT;
    cfg->deadband_ticks = POSITION_DEADBAND_DEFAULT;

    conf->p_pid_kp_q16 = cfg->kp_q16;
    conf->p_pid_ki_q16 = cfg->ki_q16;
    conf->p_pid_kd_q16 = cfg->kd_q16;
    conf->p_pid_pos_min = cfg->position_min;
    conf->p_pid_pos_max = cfg->position_max;
    conf->p_pid_deadband_ticks = cfg->deadband_ticks;
}




static void open_run_reset(OpenLoopRunState *state)
{
    memset(state, 0, sizeof(*state));
}

static int16_t open_run_gate(OpenLoopRunState *state, uint32_t now, uint8_t mode,
                             int16_t command, uint32_t duration_ms, bool params_valid)
{
    if (mode != ESC_MODE_OPEN || !params_valid) {
        open_run_reset(state);
        return command;
    }

    /* Durasi 0 = continuous. Timer baru mulai ketika amplitude OPEN pertama kali
     * non-zero; safe-arm frame dengan setpoint 0 tidak memakan waktu test. */
    if (duration_ms == 0U) {
        state->started = (command != 0);
        state->expired = false;
        if (command != 0 && state->start_tick == 0U) state->start_tick = now;
        return command;
    }

    if (state->expired) return 0;
    if (!state->started) {
        if (command == 0) return 0;
        state->started = true;
        state->start_tick = now;
        return command;
    }

    if ((uint32_t)(now - state->start_tick) >= duration_ms) {
        state->expired = true;
        return 0;
    }
    return command;
}

static bool mode_valid(uint8_t mode)
{
    return mode <= ESC_MODE_HANDBRAKE;
}

static void sensor_health_reset_one(SensorHealthRuntime *health, const MotorSensorState *state)
{
    memset(health, 0, sizeof(*health));
    health->last_valid_edges = state->encoder_valid_edges;
    health->last_invalid_transitions = state->encoder_invalid_transitions;
}

static bool command_requests_motion(uint8_t mode, int16_t command)
{
    /* Encoder incremental yang diam tidak dapat dibedakan dari encoder yang
     * terputus. NO_SIGNAL karena itu hanya valid pada mode yang SECARA EKSPLISIT
     * meminta gerak: SPD, atau POS yang outer-loop-nya menghasilkan speed command.
     * VLT/TRQ boleh sah menahan rotor/load pada zero-speed tanpa false fault. */
    if (mode != ESC_MODE_SPD && mode != ESC_MODE_POS) return false;
    return command > ENCODER_MOTION_COMMAND_MIN || command < -ENCODER_MOTION_COMMAND_MIN;
}

static void sensor_health_update_one(uint32_t now, const MotorRuntimeConfig *cfg,
                                     const MotorSensorState *state, const MotorSensorSample *sample,
                                     uint8_t mode, int16_t command, bool motor_is_armed,
                                     SensorHealthRuntime *health)
{
    if (cfg->sensor_type == MOTOR_SENSOR_HALL_UVW) {
        health->motion_since = 0U;
        health->invalid_in_window = 0U;
        health->last_valid_edges = state->encoder_valid_edges;
        health->last_invalid_transitions = state->encoder_invalid_transitions;
        if (cfg->hall_lut_valid == 0U || !MotorRuntimeConfig_HallSequenceValid(cfg->hall_sequence)) {
            health->fault_code = ESC_MOTOR_ERROR_HALL_LUT_INVALID;
            health->hall_invalid_since = 0U;
            return;
        }
        if (cfg->hall_calibrated == 0U) {
            health->fault_code = ESC_MOTOR_ERROR_HALL_NOT_DETECTED;
            health->hall_invalid_since = 0U;
            return;
        }
        if (sample->raw_hall_encoding == 0U || sample->raw_hall_encoding == 7U) {
            if (health->hall_invalid_since == 0U) health->hall_invalid_since = now;
            if ((now - health->hall_invalid_since) >= HALL_INVALID_QUALIFY_MS)
                health->fault_code = ESC_MOTOR_ERROR_HALL_NOT_DETECTED;
        } else {
            health->hall_invalid_since = 0U;
            health->fault_code = ESC_MOTOR_ERROR_NONE;
        }
        return;
    }

    health->hall_invalid_since = 0U;
    if (cfg->encoder_calibrated == 0U || cfg->encoder_sequence_valid == 0U ||
        !MotorRuntimeConfig_EncoderSequenceValid(cfg->encoder_sequence) ||
        cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN) {
        health->fault_code = ESC_MOTOR_ERROR_ENCODER_NOT_CALIBRATED;
        health->motion_since = 0U;
        health->last_valid_edges = state->encoder_valid_edges;
        health->last_invalid_transitions = state->encoder_invalid_transitions;
        return;
    }

    const uint32_t valid_now = state->encoder_valid_edges;
    const uint32_t invalid_now = state->encoder_invalid_transitions;
    const uint32_t valid_delta = valid_now - health->last_valid_edges;
    const uint32_t invalid_delta = invalid_now - health->last_invalid_transitions;
    health->last_valid_edges = valid_now;
    health->last_invalid_transitions = invalid_now;

    if (health->invalid_window_start == 0U ||
        (now - health->invalid_window_start) >= ENCODER_INVALID_WINDOW_MS) {
        /* Window baru juga menjadi de-qualify SIGNAL_INVALID. Fault tidak boleh
         * latch selamanya setelah burst noise lama sudah hilang. */
        if (health->fault_code == ESC_MOTOR_ERROR_ENCODER_SIGNAL_INVALID &&
            health->invalid_in_window < ENCODER_INVALID_LIMIT) {
            health->fault_code = ESC_MOTOR_ERROR_NONE;
        }
        health->invalid_window_start = now;
        health->valid_in_window = 0U;
        health->invalid_in_window = 0U;
    }
    if (valid_delta > 0U) {
        const uint32_t total = (uint32_t)health->valid_in_window + valid_delta;
        health->valid_in_window = (uint16_t)(total > UINT16_MAX ? UINT16_MAX : total);
    }
    if (invalid_delta > 0U) {
        const uint32_t total = (uint32_t)health->invalid_in_window + invalid_delta;
        health->invalid_in_window = (uint16_t)(total > UINT16_MAX ? UINT16_MAX : total);
    }

    /* Polling encoder 16 kHz dapat sesekali kehilangan satu state pada speed
     * tinggi. Nyatakan SIGNAL_INVALID hanya bila burst ilegal bukan sekadar kecil
     * dibanding edge valid yang benar-benar diterima pada window yang sama. */
    const bool invalid_burst = health->invalid_in_window >= ENCODER_INVALID_LIMIT;
    const bool invalid_dominant =
        ((uint32_t)health->invalid_in_window * 4U) >=
        ((uint32_t)health->valid_in_window + (uint32_t)health->invalid_in_window);
    if (invalid_burst && invalid_dominant) {
        health->fault_code = ESC_MOTOR_ERROR_ENCODER_SIGNAL_INVALID;
        health->motion_since = 0U;
        return;
    }
    if (health->fault_code == ESC_MOTOR_ERROR_ENCODER_SIGNAL_INVALID &&
        !invalid_burst) {
        health->fault_code = ESC_MOTOR_ERROR_NONE;
    }

    if (motor_is_armed && command_requests_motion(mode, command)) {
        if (valid_delta > 0U) {
            health->motion_since = now;
            health->fault_code = ESC_MOTOR_ERROR_NONE;
        } else {
            if (health->motion_since == 0U) health->motion_since = now;
            if ((now - health->motion_since) >= ENCODER_NO_SIGNAL_QUALIFY_MS)
                health->fault_code = ESC_MOTOR_ERROR_ENCODER_NO_SIGNAL;
        }
    } else {
        health->motion_since = 0U;
        if (health->fault_code == ESC_MOTOR_ERROR_ENCODER_NO_SIGNAL)
            health->fault_code = ESC_MOTOR_ERROR_NONE;
        if (health->fault_code != ESC_MOTOR_ERROR_ENCODER_SIGNAL_INVALID)
            health->fault_code = ESC_MOTOR_ERROR_NONE;
    }
}

static void apply_sensor_backend_to_foc(const MotorRuntimeConfig *cfg, mc_configuration *conf)
{
    if (cfg == NULL || conf == NULL) return;
    /* Sensor selection has exactly one owner: MotorRuntimeConfig.sensor_type.
     * The FOC core consumes only the calibrated phase/speed sample published by
     * MotorSensor, so there is no duplicate Hall/encoder selector in mc_configuration. */
    (void)cfg;
    mc_foc_conf_prepare(conf);
}


static void sensor_health_update(uint32_t now)
{
    sensor_health_update_one(now, &motorConfigLeft, &motorSensorStateLeft, &motorSensorSampleLeft,
                             requestedModeLeft, runtimeCommandLeft, armedLeft, &sensorHealthLeft);
    sensor_health_update_one(now, &motorConfigRight, &motorSensorStateRight, &motorSensorSampleRight,
                             requestedModeRight, runtimeCommandRight, armedRight, &sensorHealthRight);
}




static uint8_t effective_error_one(uint8_t mode, uint8_t core_error,
                                   const SensorHealthRuntime *health,
                                   const MotorRuntimeConfig *cfg)
{
    (void)cfg;
    if (mode == ESC_MODE_OPEN) return ESC_MOTOR_ERROR_NONE;
    if (health != NULL && health->fault_code != ESC_MOTOR_ERROR_NONE) return health->fault_code;
    return core_error;
}



static void fault_report_reset_all(void)
{
    faultReportActiveLeft = false;
    faultReportActiveRight = false;
    faultReportSuppressLeft = false;
    faultReportSuppressRight = false;
    faultReportCodeLeft = ESC_MOTOR_ERROR_NONE;
    faultReportCodeRight = ESC_MOTOR_ERROR_NONE;
    faultReportUntilLeft = 0U;
    faultReportUntilRight = 0U;
}

static void fault_report_allow_new(bool left)
{
    if (left) faultReportSuppressLeft = false;
    else faultReportSuppressRight = false;
}

static void fault_report_latch(bool left, uint8_t code, uint32_t now)
{
    if (code == ESC_MOTOR_ERROR_NONE) return;
    if (left) {
        /* Do not refresh the 3 s VESC-style fault-stop window every slow-loop. */
        if (faultReportSuppressLeft) return;
        /* Jangan refresh timer dari fault level yang sama setiap slow-loop. */
        if (!faultReportActiveLeft) {
            faultReportActiveLeft = true;
            faultReportCodeLeft = code;
            faultReportUntilLeft = now + FAULT_REPORT_DURATION_MS;
        }
    } else {
        if (faultReportSuppressRight) return;
        if (!faultReportActiveRight) {
            faultReportActiveRight = true;
            faultReportCodeRight = code;
            faultReportUntilRight = now + FAULT_REPORT_DURATION_MS;
        }
    }
}

static void fault_report_service(uint32_t now)
{
    uint8_t expired_mask = 0U;
    if (faultReportActiveLeft && (int32_t)(now - faultReportUntilLeft) >= 0) {
        faultReportActiveLeft = false;
        faultReportSuppressLeft = true;
        faultReportCodeLeft = ESC_MOTOR_ERROR_NONE;
        expired_mask |= 0x01U;
    }
    if (faultReportActiveRight && (int32_t)(now - faultReportUntilRight) >= 0) {
        faultReportActiveRight = false;
        faultReportSuppressRight = true;
        faultReportCodeRight = ESC_MOTOR_ERROR_NONE;
        expired_mask |= 0x02U;
    }

    if (expired_mask != 0U) {
        /* FOC core has no legacy Hall diagnostic latch. Only clear the explicit
         * ISR deadline latch after its reporting window. */
        MotorControl_ClearIsrOverrunFault(expired_mask);
        MotorControl_ClearOvercurrentFault(expired_mask);
    }

    /* Bila sumber fault sudah sehat saat DISARM, reporting boleh normal lagi
     * bahkan sebelum user melakukan ARM berikutnya. */
    if (!armedLeft && !armRequestedLeft &&
        effective_error_one(requestedModeLeft, motorOutputLeft.fault_code,
                            &sensorHealthLeft, &motorConfigLeft) == ESC_MOTOR_ERROR_NONE) {
        faultReportSuppressLeft = false;
    }
    if (!armedRight && !armRequestedRight &&
        effective_error_one(requestedModeRight, motorOutputRight.fault_code,
                            &sensorHealthRight, &motorConfigRight) == ESC_MOTOR_ERROR_NONE) {
        faultReportSuppressRight = false;
    }
}

static bool blocking_error_one(uint8_t mode, uint8_t core_error,
                               const SensorHealthRuntime *health,
                               const MotorRuntimeConfig *cfg)
{
    (void)cfg;
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE) return false;
    if (health != NULL && health->fault_code != ESC_MOTOR_ERROR_NONE) return true;
    return core_error != 0U;
}

static bool homing_target_fault_free(uint8_t motor_select)
{
    if ((motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH) &&
        blocking_error_one(ESC_MODE_SPD, motorOutputLeft.fault_code, &sensorHealthLeft, &motorConfigLeft)) return false;
    if ((motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH) &&
        blocking_error_one(ESC_MODE_SPD, motorOutputRight.fault_code, &sensorHealthRight, &motorConfigRight)) return false;
    return true;
}

/*
 * ARM/DISARM adalah gate output, bukan gate nilai setpoint. Host selalu mengirim
 * target sebenarnya dan runtimeSetpoint* harus tetap merepresentasikan target itu.
 * Karena PWM/MOE belum dibuka sebelum try_arm_motor() sukses, tidak perlu lagi
 * menimpa target POS dengan posisi sensor atau target SPD/TRQ/VLT dengan nol.
 *
 * Safety di sini hanya memvalidasi domain command. Setelah ARMED:
 *   - POS dibatasi soft-limit dan outer position PID menghasilkan Iq target.
 *   - SPD outer PID juga menghasilkan Iq target; TRQ langsung mengisi Iq target.
 *   - VLT mengisi Vq target; OPEN memakai forced electrical phase.
 */
static bool safe_to_arm_one(uint8_t mode, int32_t target, int32_t position,
                            const PositionPidConfig *position_cfg)
{
    if (mode == ESC_MODE_OPEN) return true;

    if (mode == ESC_MODE_POS) {
        if (position < position_cfg->position_min || position > position_cfg->position_max)
            return false;
        return target >= position_cfg->position_min && target <= position_cfg->position_max;
    }

    return target >= -1000 && target <= 1000;
}

static bool safe_to_arm_motor(bool left)
{
    if (left) {
        const int32_t position = apply_motor_direction_i32(&motorConfigLeft, odom_l);
        return safe_to_arm_one(requestedModeLeft, runtimeSetpointLeft, position, &positionPidConfigLeft);
    }
    const int32_t position = apply_motor_direction_i32(&motorConfigRight, odom_r);
    return safe_to_arm_one(requestedModeRight, runtimeSetpointRight, position, &positionPidConfigRight);
}

/* V15 pre-arm permission is intentionally narrow and local. The protocol must
 * not inherit debounced/stale diagnostic health as another PWM gate. Closed-loop
 * only requires the sensor proof actually needed by the selected motor; OPEN is
 * allowed without Hall/encoder calibration. Runtime sensor loss is still handled
 * after ARM by the normal sensor/core fault monitor. */
static bool prearm_feedback_not_ready(uint8_t mode, int32_t target,
                                      const MotorRuntimeConfig *cfg,
                                      const MotorSensorSample *sample, bool encoder_aligned)
{
    /* OPEN and handbrake own their phase independently. SET_DUTY(0) remains
     * the normal VESC DUTY control state at zero modulation; static low-side
     * shorting is only a separate foc_short_ls_on_zero_duty policy upstream and
     * must not bypass this board's proven sensored FOC/ADC path. */
    /* Zero-duty is VESC Tool Full Brake control state. It commands zero
     * modulation and must be enterable even when an ABI phase-alignment job is
     * pending; Stop remains COMM_SET_CURRENT(0) and fully releases the bridge. */
    if (mode == ESC_MODE_DUTY && target == 0) return false;
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE) return false;
    if (cfg == NULL || sample == NULL) return true;
    if (cfg->sensor_type == MOTOR_SENSOR_HALL_UVW) {
        return cfg->hall_calibrated == 0U || cfg->hall_lut_valid == 0U ||
               sample->feedback_valid == 0U;
    }
    if (cfg->sensor_type == MOTOR_SENSOR_ENCODER_AB) {
        return cfg->encoder_calibrated == 0U || cfg->encoder_sequence_valid == 0U ||
               !encoder_aligned || sample->feedback_valid == 0U;
    }
    return true;
}

static int32_t position_default_span_ticks(bool left)
{
    const MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    const mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    int32_t span;
    if (cfg->sensor_type == MOTOR_SENSOR_ENCODER_AB && cfg->encoder_cpr >= MOTOR_ENCODER_CPR_MIN) {
        span = (int32_t)cfg->encoder_cpr;
    } else {
        const uint8_t pp = conf->foc_motor_pole_pairs != 0U ? conf->foc_motor_pole_pairs : 1U;
        span = (int32_t)(6U * pp);
    }

    /* V20 position coordinates are host/logical coordinates. Before hard-stop
     * homing establishes a measured signed span, one logical +360 revolution
     * must follow the same direction as a positive host torque/speed command.
     * motor_inverted reverses that physical direction, therefore the default raw
     * sensor span must be signed too. This fixes RIGHT SET_POS on installations
     * where Duty/Current/RPM are correct only with Motor Direction inverted. */
    return cfg->motor_inverted ? -span : span;
}

int32_t RuntimeControl_PositionSpanTicks(bool left)
{
    const SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    if (cal->calibrated != 0U && cal->span_ticks != 0) return cal->span_ticks;
    return position_default_span_ticks(left);
}

bool RuntimeControl_PositionTargetTicks(bool left, float deg, int32_t *target_ticks)
{
    if (target_ticks == NULL || !isfinite(deg)) return false;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 360.0f) deg = 360.0f;

    const int32_t span = RuntimeControl_PositionSpanTicks(left);
    if (span == 0) return false;

    /* A persisted mechanical span is only absolute after this boot's homing.
     * Without a calibrated span the board intentionally exposes one relative
     * revolution from tick zero, which keeps bench POSITION tests deterministic. */
    const int32_t zero = position_session_zero(left);
    const int64_t num = (int64_t)span * (int64_t)lrintf(deg * 1000.0f);
    *target_ticks = zero + (int32_t)(num / 360000LL);
    return true;
}

float RuntimeControl_PositionDeg(bool left)
{
    const SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    const int32_t span = RuntimeControl_PositionSpanTicks(left);
    if (span == 0) return 0.0f;
    const int32_t zero = position_session_zero(left);
    const int64_t rel = (int64_t)sample->position_ticks - zero;

    if (cal->calibrated != 0U && cal->homed != 0U) {
        float deg = ((float)rel * 360.0f) / (float)span;
        if (deg < 0.0f) deg = 0.0f;
        if (deg > 360.0f) deg = 360.0f;
        return deg;
    }

    /* Unhomed session is a bounded one-revolution coordinate: 0 and 360 are
     * distinct endpoints. This matches the SET_POS contract requested for bench/
     * steering; hard-stop homing later replaces the session origin/span. */
    float deg = ((float)rel * 360.0f) / (float)span;
    if (deg < 0.0f) deg = 0.0f;
    if (deg > 360.0f) deg = 360.0f;
    return deg;
}

float RuntimeControl_PositionErrorDeg(bool left)
{
    const motor_all_state_t *motor = left ? &motorLeft : &motorRight;
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    int32_t span = RuntimeControl_PositionSpanTicks(left);
    if (span == 0) return 0.0f;
    return ((float)(motor->m_pos_pid_set - sample->position_ticks) * 360.0f) / (float)span;
}

static void homing_motor_stop(bool left)
{
    if (left) {
        pwml = 0; runtimeCommandLeft = 0; controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
        mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_NONE);
    } else {
        pwmr = 0; runtimeCommandRight = 0; controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;
        mc_foc_set_control_mode(&motorRight, CONTROL_MODE_NONE);
    }
}

static void homing_abort_all(uint8_t terminal_state)
{
    if ((homingActiveMask & 0x01U) != 0U || homingPendingOperationLeft != 0U)
        homingRuntimeLeft.state = terminal_state;
    if ((homingActiveMask & 0x02U) != 0U || homingPendingOperationRight != 0U)
        homingRuntimeRight.state = terminal_state;
    homingActiveMask = 0U;
    homingPendingOperationLeft = 0U;
    homingPendingOperationRight = 0U;
    homingRuntimeLeft.stop_candidate = false;
    homingRuntimeRight.stop_candidate = false;
    homingStartedFromBoot = false;
}

static void homing_begin_motion(bool left, uint8_t operation, uint32_t now)
{
    HomingRuntimeState *state = left ? &homingRuntimeLeft : &homingRuntimeRight;
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    state->operation = operation;
    state->state = operation == 2U ? ESC_HOMING_CAL_RIGHT : ESC_HOMING_SEARCHING;
    state->start_tick = now;
    state->stop_candidate_since_tick = 0U;
    state->peak_current_centi_amp = 0;
    state->last_position_ticks = sample->position_ticks;
    state->last_motion_tick = now;
    state->right_stop_ticks = sample->position_ticks;
    state->stop_candidate = false;
    homingActiveMask |= left ? 0x01U : 0x02U;
}

bool RuntimeControl_RequestEncoderSync(bool left)
{
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    MotorSensorState *st = left ? &motorSensorStateLeft : &motorSensorStateRight;
    if (cfg->sensor_type != MOTOR_SENSOR_ENCODER_AB || cfg->encoder_calibrated == 0U)
        return false;
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || homingActiveMask != 0U || encoderAlign.active)
        return false;
    st->encoder_electrical_aligned = false;
    if (left) encoderAlignedLeft = false; else encoderAlignedRight = false;
    encoder_alignment_start(left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT);
    return true;
}

static bool homing_start_request(bool left, uint8_t operation)
{
    HomingRuntimeState *state = left ? &homingRuntimeLeft : &homingRuntimeRight;
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    MotorSensorState *sensor = left ? &motorSensorStateLeft : &motorSensorStateRight;
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || homingActiveMask != 0U || encoderAlign.active)
        return false;
    if (!homing_target_fault_free(left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT)) return false;

    if (cfg->sensor_type == MOTOR_SENSOR_ENCODER_AB && !sensor->encoder_electrical_aligned) {
        if (left) homingPendingOperationLeft = operation; else homingPendingOperationRight = operation;
        state->state = ESC_HOMING_SYNC_ELECTRICAL;
        if (!RuntimeControl_RequestEncoderSync(left)) {
            if (left) homingPendingOperationLeft = 0U; else homingPendingOperationRight = 0U;
            state->state = ESC_HOMING_FAULT;
            return false;
        }
        return true;
    }
    homing_begin_motion(left, operation, RuntimeControl_MonotonicMs());
    return true;
}

bool RuntimeControl_StartHomingCalibration(bool left)
{
    return homing_start_request(left, 2U);
}

bool RuntimeControl_StartHomingOne(bool left)
{
    return homing_start_request(left, 1U);
}

bool RuntimeControl_HomingOnBoot(bool left)
{
    return left ? homingOnBootLeft : homingOnBootRight;
}

bool RuntimeControl_SetHomingOnBoot(bool left, bool on)
{
    SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    if (RuntimeControl_Armed() || sensorCal.state == ESC_SENSOR_CAL_RUNNING ||
        encoderAlign.active || homingActiveMask != 0U) return false;

    /* Boot homing is intentionally only a one-stop re-reference. A full span
     * calibration can drive into both hard stops, so it must remain an explicit
     * user action and is never enabled implicitly on power-up. */
    if (on && (cal->calibrated == 0U || cal->span_ticks == 0)) return false;

    if (left) homingOnBootLeft = on; else homingOnBootRight = on;
    settingsDirty = true;
    if (!RuntimeSettings_Save()) return false;
    settingsDirty = false;
    return true;
}

static void homing_service_pending_after_sync(uint32_t now)
{
    if (encoderAlign.active) return;
    if (homingPendingOperationLeft != 0U) {
        const uint8_t op = homingPendingOperationLeft;
        homingPendingOperationLeft = 0U;
        if (motorSensorStateLeft.encoder_electrical_aligned) homing_begin_motion(true, op, now);
        else homingRuntimeLeft.state = ESC_HOMING_FAULT;
    }
    if (homingPendingOperationRight != 0U) {
        const uint8_t op = homingPendingOperationRight;
        homingPendingOperationRight = 0U;
        if (motorSensorStateRight.encoder_electrical_aligned) homing_begin_motion(false, op, now);
        else homingRuntimeRight.state = ESC_HOMING_FAULT;
    }
}

static bool homing_hardstop_detected(bool left, HomingRuntimeState *state,
                                     const HomingMotorConfig *cfg, uint32_t now)
{
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    const uint8_t motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    const uint16_t current_ca = sensor_cal_measured_current_centi_amp(motor);
    if ((int16_t)current_ca > state->peak_current_centi_amp)
        state->peak_current_centi_amp = (int16_t)current_ca;

    const int32_t delta = sample->position_ticks - state->last_position_ticks;
    state->last_position_ticks = sample->position_ticks;
    if (delta > 1 || delta < -1) state->last_motion_tick = now;

    const int16_t speed = sample->mechanical_speed_q4;
    const bool speed_low = speed > -80 && speed < 80; /* <5 mechanical rpm */
    const bool no_motion = (now - state->last_motion_tick) >= homingDebounceMs;
    const bool current_high = current_ca >= cfg->current_threshold_centi_amp;

    if (current_high && speed_low && no_motion && (now - state->start_tick) >= 250U) {
        if (!state->stop_candidate) {
            state->stop_candidate = true;
            state->stop_candidate_since_tick = now;
        }
        return (now - state->stop_candidate_since_tick) >= homingDebounceMs;
    }
    state->stop_candidate = false;
    state->stop_candidate_since_tick = 0U;
    return false;
}

static bool homing_update_one(bool left, uint32_t now)
{
    HomingRuntimeState *state = left ? &homingRuntimeLeft : &homingRuntimeRight;
    HomingMotorConfig *cfg = left ? &homingConfigLeft : &homingConfigRight;
    SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    const uint8_t mask = left ? 0x01U : 0x02U;
    if ((homingActiveMask & mask) == 0U) return false;

    if ((now - state->start_tick) >= homingTimeoutMs) {
        state->state = ESC_HOMING_TIMEOUT;
        homingActiveMask &= (uint8_t)~mask;
        return true;
    }

    if (state->state == ESC_HOMING_RETURN_CENTER) {
        int32_t center = 0;
        if (!RuntimeControl_PositionTargetTicks(left, 180.0f, &center)) {
            state->state = ESC_HOMING_FAULT;
            homingActiveMask &= (uint8_t)~mask;
            return true;
        }
        int32_t err = sample->position_ticks - center;
        if (err < 0) err = -err;
        int32_t tol = RuntimeControl_PositionSpanTicks(left);
        if (tol < 0) tol = -tol;
        tol = tol / 180; /* about 2 degrees */
        if (tol < 1) tol = 1;
        if (err <= tol) {
            state->state = ESC_HOMING_READY;
            homingActiveMask &= (uint8_t)~mask;
            return true;
        }
        return false;
    }

    if (!homing_hardstop_detected(left, state, cfg, now)) return false;

    homing_motor_stop(left);
    if (state->operation == 2U && state->state == ESC_HOMING_CAL_RIGHT) {
        state->right_stop_ticks = sample->position_ticks;
        cal->right_zero_ticks = sample->position_ticks;
        cal->homed = 1U;
        state->state = ESC_HOMING_CAL_LEFT;
        state->start_tick = now;
        state->last_motion_tick = now;
        state->stop_candidate = false;
        return false;
    }

    if (state->operation == 2U && state->state == ESC_HOMING_CAL_LEFT) {
        const int32_t span = sample->position_ticks - state->right_stop_ticks;
        int32_t mag = span < 0 ? -span : span;
        int32_t min_span = position_default_span_ticks(left);
        if (min_span < 0) min_span = -min_span;
        min_span /= 4;
        if (span == 0 || mag < (min_span > 2 ? min_span : 2)) {
            state->state = ESC_HOMING_FAULT;
            homingActiveMask &= (uint8_t)~mask;
            return true;
        }
        cal->right_zero_ticks = state->right_stop_ticks;
        cal->span_ticks = span;
        cal->calibrated = 1U;
        cal->homed = 1U;
        if (left) homingOnBootLeft = true;
        settingsDirty = true;
        state->state = ESC_HOMING_RETURN_CENTER;
        state->start_tick = now;
        return false;
    }

    /* Single-stop home: refresh absolute zero for this boot and reuse saved span. */
    cal->right_zero_ticks = sample->position_ticks;
    cal->homed = 1U;
    if (cal->calibrated != 0U && cal->span_ticks != 0) {
        state->state = ESC_HOMING_RETURN_CENTER;
        state->start_tick = now;
    } else {
        state->state = ESC_HOMING_HOMED;
        homingActiveMask &= (uint8_t)~mask;
    }
    return true;
}

static void homing_start(uint8_t motor_select)
{
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH)
        (void)RuntimeControl_StartHomingOne(true);
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH)
        (void)RuntimeControl_StartHomingOne(false);
}

static void homing_service_active(uint32_t now, uint32_t dt_ms)
{
    const uint8_t active_target = (homingActiveMask == 0x01U) ? ESC_MOTOR_LEFT :
                                  ((homingActiveMask == 0x02U) ? ESC_MOTOR_RIGHT : ESC_MOTOR_BOTH);
    if (!homing_target_fault_free(active_target)) {
        homing_abort_all(ESC_HOMING_FAULT);
        homingStartedFromBoot = false;
        disarm_outputs();
        return;
    }

    armedLeft = false;
    armedRight = false;
    armRequestedLeft = false;
    armRequestedRight = false;
    runtimeMotorEnableMask = homingActiveMask & 0x03U;
    enable = runtimeMotorEnableMask != 0U ? 1U : 0U;

    HomingRuntimeState *states[2] = {&homingRuntimeLeft, &homingRuntimeRight};
    HomingMotorConfig *cfgs[2] = {&homingConfigLeft, &homingConfigRight};
    for (uint8_t i = 0U; i < 2U; ++i) {
        const bool left = i == 0U;
        const uint8_t mask = left ? 0x01U : 0x02U;
        if ((homingActiveMask & mask) == 0U) {
            homing_motor_stop(left);
            continue;
        }
        HomingRuntimeState *st = states[i];
        int16_t search = cfgs[i]->search_command;
        if (st->state == ESC_HOMING_CAL_LEFT) {
            if (search < 0) search = (int16_t)-search;
        } else if (st->state == ESC_HOMING_RETURN_CENTER) {
            int32_t center = 0;
            if (RuntimeControl_PositionTargetTicks(left, 180.0f, &center))
                apply_runtime_target_one(left, ESC_MODE_POS, center, dt_ms, now);
            continue;
        } else {
            if (search > 0) search = (int16_t)-search;
        }
        /* Hard-stop search is current/torque controlled, not speed controlled.
         * This prevents a speed integrator from winding up against the stopper. */
        apply_runtime_target_one(left, ESC_MODE_TRQ, search, dt_ms, now);
    }

    (void)homing_update_one(true, now);
    (void)homing_update_one(false, now);
    if (homingActiveMask == 0U) {
        homingStartedFromBoot = false;
        disarm_outputs();
        if (settingsDirty) {
            if (RuntimeSettings_Save()) settingsDirty = false;
        }
    }
}


static void refresh_master_enable(void)
{
    /* Legacy master enable tetap menjadi kill-switch bersama. Per-motor gating
     * dilakukan runtimeMotorEnableMask di motor.c. Calibration override dan
     * homing juga membutuhkan master ON walau closed-loop ARM false. */
    const uint8_t active = (uint8_t)(runtimeMotorEnableMask | sensorCalibrationOpenLoopMask | homingActiveMask);
    enable = (active != 0U) ? 1U : 0U;
}

static int8_t command_direction_i16(int16_t command)
{
    return command > 0 ? 1 : (command < 0 ? -1 : 0);
}

/* Hanya dipanggil slow-loop setelah runtimeCommand final (termasuk POS PID)
 * selesai dihitung. Reset PID command + stale stall latch dilakukan atomik
 * terhadap ISR dan hanya pada perubahan tanda non-zero. */
static void update_direction_change_guard(bool left, uint32_t now)
{
    (void)now;
    const bool armed = left ? armedLeft : armedRight;
    int8_t *previous = left ? &previousCommandDirectionLeft : &previousCommandDirectionRight;
    const MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    motor_all_state_t *motor = left ? &motorLeft : &motorRight;
    const int16_t command = left ? runtimeCommandLeft : runtimeCommandRight;

    if (!armed) {
        *previous = 0;
        return;
    }

    const int16_t physical_command = clamp_i16(
        apply_motor_direction_i32(cfg, command), -1000, 1000);
    const int8_t direction = command_direction_i16(physical_command);
    const bool reversed = direction != 0 && *previous != 0 && direction != *previous;
    if (reversed) {
        /* Sensor backend owns rotor phase. Reversal must never rewrite Hall/encoder
         * state; only control-loop memory is cleared. */
        mc_foc_reset_outer_loops(motor);
    }
    if (direction != 0) *previous = direction;
}


static void disarm_motor(uint8_t motor)
{
    const bool left = motor == ESC_MOTOR_LEFT;
    const bool state_changed = left
        ? (armedLeft || (runtimeMotorEnableMask & 0x01U) != 0U)
        : (armedRight || (runtimeMotorEnableMask & 0x02U) != 0U);

    if (left) {
        armedLeft = false;
        runtimeMotorEnableMask &= (uint8_t)~0x01U;
        runtimeCommandLeft = 0;
        pwml = 0;
        controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
        previousCommandDirectionLeft = 0;
        open_run_reset(&openRunLeft);
        mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_NONE);
        mc_foc_reset_control(&motorLeft);
    } else {
        armedRight = false;
        runtimeMotorEnableMask &= (uint8_t)~0x02U;
        runtimeCommandRight = 0;
        pwmr = 0;
        controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;
        previousCommandDirectionRight = 0;
        open_run_reset(&openRunRight);
        mc_foc_set_control_mode(&motorRight, CONTROL_MODE_NONE);
        mc_foc_reset_control(&motorRight);
    }
    refresh_master_enable();
    if (state_changed) queue_arm_status_snapshot();
}


static void disarm_outputs(void)
{
    const bool state_changed = armedLeft || armedRight || runtimeMotorEnableMask != 0U;
    armedLeft = false;
    armedRight = false;
    runtimeMotorEnableMask = 0U;
    runtimeCommandLeft = 0;
    runtimeCommandRight = 0;
    pwml = 0;
    pwmr = 0;
    controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
    controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;
    previousCommandDirectionLeft = 0;
    previousCommandDirectionRight = 0;
    open_run_reset(&openRunLeft);
    open_run_reset(&openRunRight);
    mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_NONE);
    mc_foc_set_control_mode(&motorRight, CONTROL_MODE_NONE);
    mc_foc_reset_control(&motorLeft);
    mc_foc_reset_control(&motorRight);
    refresh_master_enable();
    if (state_changed) queue_arm_status_snapshot();
}


/* Satu percobaan ARM hanya menyentuh motor target. Fault/encoder/alignment sisi
 * lain tidak boleh lagi menolak atau mematikan motor ini. */
static void try_arm_motor(bool left, uint32_t now)
{
    (void)now;
    bool *requested = left ? &armRequestedLeft : &armRequestedRight;
    bool *armed = left ? &armedLeft : &armedRight;
    const uint8_t motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    const uint8_t bit = left ? 0x01U : 0x02U;
    const uint8_t mode = left ? requestedModeLeft : requestedModeRight;
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;

    if (!*requested || *armed) return;

    /* V15: a historical GUI fault-report window is diagnostic, not an ARM
     * interlock. Current offset, active commissioning ownership and the selected
     * feedback proof below are the only pre-arm prerequisites. */
    if (!MotorControl_CurrentOffsetsValid()) {
        set_arm_reject(left, ESC_ARM_REJECT_CURRENT_OFFSET_NOT_READY);
        return;
    }
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING) {
        set_arm_reject(left, ESC_ARM_REJECT_CALIBRATION_BUSY);
        return;
    }
    if (homingActiveMask != 0U) {
        set_arm_reject(left, ESC_ARM_REJECT_HOMING_BUSY);
        return;
    }
    if (encoderAlign.active && encoderAlign.motor == motor) {
        set_arm_reject(left, ESC_ARM_REJECT_ALIGNMENT_BUSY);
        return;
    }
    /* Alignment encoder sisi lain tidak boleh menahan motor ini. Jika motor ini
     * sendiri juga Encoder dan belum aligned, ia akan menunggu giliran tanpa
     * menjatuhkan ARM request. */
    if (encoderAlign.active && cfg->sensor_type == MOTOR_SENSOR_ENCODER_AB &&
        mode != ESC_MODE_OPEN && mode != ESC_MODE_HANDBRAKE && !(left ? encoderAlignedLeft : encoderAlignedRight)) {
        set_arm_reject(left, ESC_ARM_REJECT_ALIGNMENT_BUSY);
        return;
    }

    /* Incremental A/B membutuhkan electrical zero sekali per power-on sebelum
     * closed-loop. Alignment ini hanya mengambil alih motor target. */
    const bool encoder_needs_alignment =
        mode != ESC_MODE_OPEN && mode != ESC_MODE_HANDBRAKE &&
        cfg->sensor_type == MOTOR_SENSOR_ENCODER_AB &&
        cfg->encoder_calibrated != 0U &&
        !(left ? encoderAlignedLeft : encoderAlignedRight);
    if (encoder_needs_alignment) {
        set_arm_reject(left, ESC_ARM_REJECT_ALIGNMENT_BUSY);
        encoder_alignment_start(motor);
        return;
    }

    /* ISR deadline is diagnostic in V15; hard DC over-current is chopped
     * sample-by-sample in motor.c and does not create a stale pre-arm latch. */
    if (!safe_to_arm_motor(left)) {
        set_arm_reject(left, ESC_ARM_REJECT_UNSAFE_SETPOINT);
        return;
    }
    const MotorSensorSample *sample = left ? &motorSensorSampleLeft : &motorSensorSampleRight;
    const bool encoder_aligned = left ? encoderAlignedLeft : encoderAlignedRight;
    const int32_t requested_target = left ? runtimeSetpointLeft : runtimeSetpointRight;
    if (prearm_feedback_not_ready(mode, requested_target, cfg, sample, encoder_aligned)) {
        set_arm_reject(left, ESC_ARM_REJECT_SENSOR_OR_FOC);
        return;
    }

    /* Prepare VESC-style controller state before enabling the hardware gate. */
    motor_all_state_t *core = left ? &motorLeft : &motorRight;
    const mc_control_mode core_mode = (mc_control_mode)to_foc_mode(mode);
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    mc_foc_reset_control(core);
    mc_foc_set_control_mode(core, core_mode);
    if (primask == 0U) __enable_irq();

    *armed = true;
    set_arm_reject(left, ESC_ARM_REJECT_NONE);
    runtimeMotorEnableMask |= bit;
    queue_arm_status_snapshot();
    if (left) controlModeLeftFoc = (uint8_t)core_mode;
    else controlModeRightFoc = (uint8_t)core_mode;
    refresh_master_enable();
}

/* Memetakan mode host ke core VESC-style. SPD/POS adalah outer loop -> Iq. */
static uint8_t to_foc_mode(uint8_t mode)
{
    switch (mode) {
        case ESC_MODE_VLT: return (uint8_t)CONTROL_MODE_VOLTAGE;
        case ESC_MODE_SPD: return (uint8_t)CONTROL_MODE_SPEED;
        case ESC_MODE_TRQ: return (uint8_t)CONTROL_MODE_CURRENT;
        case ESC_MODE_BRAKE: return (uint8_t)CONTROL_MODE_CURRENT_BRAKE;
        case ESC_MODE_HANDBRAKE: return (uint8_t)CONTROL_MODE_HANDBRAKE;
        case ESC_MODE_POS: return (uint8_t)CONTROL_MODE_POS;
        case ESC_MODE_DUTY: return (uint8_t)CONTROL_MODE_DUTY;
        case ESC_MODE_OPEN:
        default: return (uint8_t)CONTROL_MODE_OPENLOOP;
    }
}

static int16_t scale_permille_to_limit(int16_t command, int16_t positive_limit)
{
    int32_t value = (int32_t)command * positive_limit;
    if (value >= 0) value += 500;
    else value -= 500;
    return clamp_i16(value / 1000, (int16_t)-positive_limit, positive_limit);
}

/* VESC has independent positive motor-current and negative/braking-current
 * limits. Keep the generic host permille representation, but map each sign to
 * the corresponding physical limit instead of assuming symmetric limits. */
static int16_t scale_permille_to_current_limit(int16_t command,
                                                const mc_configuration *conf)
{
    if (conf == NULL || command == 0) return 0;
    if (command > 0) {
        return scale_permille_to_limit(command, conf->l_current_max);
    }

    const int16_t brake_limit = conf->l_current_min < 0
        ? (int16_t)(-conf->l_current_min) : conf->l_current_max;
    int32_t value = (int32_t)(-command) * brake_limit + 500;
    value /= 1000;
    if (value > brake_limit) value = brake_limit;
    return (int16_t)-value;
}

static int32_t scale_permille_to_erpm(int16_t command, const mc_configuration *conf)
{
    if (conf == NULL || conf->foc_motor_pole_pairs == 0U) return 0;
    const int32_t max_erpm = ((int32_t)conf->l_max_speed_rpm_q4 *
                              (int32_t)conf->foc_motor_pole_pairs) >> 4;
    int64_t value = (int64_t)command * (int64_t)max_erpm;
    if (value >= 0) value += 500;
    else value -= 500;
    value /= 1000;
    if (value > max_erpm) value = max_erpm;
    if (value < -max_erpm) value = -max_erpm;
    return (int32_t)value;
}

/* VESC mc_interface update_override_limits() starts reducing accelerating
 * current at l_erpm_start (default 0.8) and reaches zero at l_max_erpm.
 * Keep this on the slow control path so torque/current commands cannot keep
 * accelerating past the configured electrical speed limit, while braking
 * current in the opposite direction remains available. */
#define VESC_ERPM_LIMIT_START_Q15 26214 /* round(0.8 * 32768) */
static int16_t limit_accel_current_by_erpm(const motor_all_state_t *motor,
                                           const mc_configuration *conf,
                                           int16_t iq)
{
    if (motor == NULL || conf == NULL || conf->foc_motor_pole_pairs == 0U || iq == 0) return iq;

    const int32_t max_erpm = ((int32_t)conf->l_max_speed_rpm_q4 *
                              (int32_t)conf->foc_motor_pole_pairs) >> 4;
    if (max_erpm <= 0) return iq;
    const int32_t start_erpm = (int32_t)(((int64_t)max_erpm * VESC_ERPM_LIMIT_START_Q15) >> 15);
    if (start_erpm <= 0 || start_erpm >= max_erpm) return iq;

    const int32_t erpm = ((int32_t)motor->m_speed_rpm_q4 *
                          (int32_t)conf->foc_motor_pole_pairs) >> 4;
    int32_t abs_erpm = erpm >= 0 ? erpm : -erpm;
    const bool accelerating_same_direction = (erpm > 0 && iq > 0) || (erpm < 0 && iq < 0);
    if (!accelerating_same_direction || abs_erpm <= start_erpm) return iq;
    if (abs_erpm >= max_erpm) return 0;

    const int32_t span = max_erpm - start_erpm;
    const int32_t remaining = max_erpm - abs_erpm;
    int32_t limited = (int32_t)(((int64_t)iq * remaining) / span);
    if (iq > 0 && limited < 0) limited = 0;
    if (iq < 0 && limited > 0) limited = 0;
    return clamp_i16(limited, conf->l_current_min, conf->l_current_max);
}

static int16_t iq_to_host_permille(const MotorRuntimeConfig *cfg,
                                   const mc_configuration *conf,
                                   int16_t iq_internal)
{
    if (conf->l_current_max <= 0) return 0;
    int32_t command = ((int32_t)iq_internal * 1000) / conf->l_current_max;
    command = apply_motor_direction_i32(cfg, command);
    return clamp_i16(command, -1000, 1000);
}

/*
 * Slow-path target mapping. This is the only place that maps host modes to the
 * VESC-style controller variables. ISR only consumes m_id_set/m_iq_set/phase.
 */
static void apply_runtime_target_one(bool left, uint8_t mode, int32_t host_setpoint,
                                     uint32_t dt_ms, uint32_t now)
{
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    motor_all_state_t *motor = left ? &motorLeft : &motorRight;
    volatile int16_t *runtime_command = left ? &runtimeCommandLeft : &runtimeCommandRight;
    volatile uint8_t *mode_status = left ? &controlModeLeftFoc : &controlModeRightFoc;
    volatile int *legacy_open_command = left ? &pwml : &pwmr;
    OpenLoopRunState *open_state = left ? &openRunLeft : &openRunRight;
    const uint32_t open_duration = left ? runtimeOpenDurationLeftMs : runtimeOpenDurationRightMs;
    const bool open_params_valid = (runtimeOpenParamsValidMask & (left ? 0x01U : 0x02U)) != 0U;

    const mc_control_mode core_mode = (mc_control_mode)to_foc_mode(mode);
    mc_foc_set_control_mode(motor, core_mode);
    *mode_status = (uint8_t)core_mode;
    motor->m_id_set = 0;
    *legacy_open_command = 0;

    if (mode == ESC_MODE_POS) {
        const PositionPidConfig *pos = left ? &positionPidConfigLeft : &positionPidConfigRight;
        int32_t target_host = clamp_i32(host_setpoint, pos->position_min, pos->position_max);
        /* RuntimeControl_PositionTargetTicks() already maps logical 0..360 into
         * the signed raw-sensor coordinate (session direction or calibrated
         * hard-stop span). Do NOT invert this absolute tick a second time; doing
         * so also negates a nonzero home origin and was the remaining RIGHT POS
         * error when motor_inverted was enabled. */
        motor->m_pos_pid_set = target_host;
        int16_t iq = mc_foc_run_pid_control_pos(motor, dt_ms);
        /* Bench-safe POSITION torque ceiling for BOTH Hall and ABI. A position
         * command is an outer loop and must never turn a bad phase reference or
         * tuning value into a multi-amp stall. One amp is sufficient for the
         * 15-degree validation step; VESC current limits still remain the outer
         * absolute limit. */
        int16_t pos_cap = (int16_t)CONTROL_CURRENT_INTERNAL_PER_A;
        if (pos_cap > conf->l_current_max) pos_cap = conf->l_current_max;
        if (iq > pos_cap) iq = pos_cap;
        if (iq < -pos_cap) iq = (int16_t)-pos_cap;
        motor->m_iq_set = iq;
        *runtime_command = iq_to_host_permille(cfg, conf, iq);
        return;
    }

    int16_t command = clamp_i16(host_setpoint, -1000, 1000);
    if (mode == ESC_MODE_OPEN) {
        command = open_run_gate(open_state, now, mode, command,
                                open_duration, open_params_valid);
    }
    const int16_t physical_command = clamp_i16(
        apply_motor_direction_i32(cfg, command), -1000, 1000);

    if (mode != ESC_MODE_DUTY) motor->m_duty_cycle_set_q15 = 0;

    switch (mode) {
        case ESC_MODE_SPD: {
            /* VESC COMM_SET_RPM is electrical RPM. The protocol facade stores it
             * as signed permille of l_max_erpm; reconstruct eRPM directly here.
             * Do not round-trip through mechanical RPM because that obscures the
             * 15-pole-pair scaling and makes VESC Tool tuning hard to audit. */
            motor->m_speed_command_rpm = scale_permille_to_erpm(physical_command, conf);
            if (conf->s_pid_ramp_erpms_s <= 0)
                motor->m_speed_pid_set_rpm = motor->m_speed_command_rpm;
            {
                const int16_t iq_pid = mc_foc_run_pid_control_speed(motor, dt_ms);
                motor->m_iq_set = limit_accel_current_by_erpm(motor, conf, iq_pid);
                *runtime_command = iq_to_host_permille(cfg, conf, motor->m_iq_set);
            }
            break;
        }
        case ESC_MODE_TRQ:
            /* VESC current mode is torque control: signed Iq is regulated, not RPM.
             * An unloaded wheel is therefore expected to accelerate until losses or
             * the configured eRPM taper limit balance the commanded torque. */
            motor->m_iq_set = limit_accel_current_by_erpm(
                motor, conf, scale_permille_to_current_limit(physical_command, conf));
            *runtime_command = iq_to_host_permille(cfg, conf, motor->m_iq_set);
            break;
        case ESC_MODE_BRAKE: {
            /* VESC set_brake_current stores a magnitude. The fast current loop
             * chooses the opposing sign from instantaneous measured speed every
             * update, so a direction reversal cannot turn braking into acceleration. */
            int16_t mag_command = command < 0 ? (int16_t)-command : command;
            if (mag_command < 0) mag_command = 1000; /* INT16 defensive only */
            const int16_t brake_limit = conf->l_current_min < 0
                ? (int16_t)(-conf->l_current_min) : conf->l_current_max;
            motor->m_iq_set = scale_permille_to_limit(mag_command, brake_limit);
            if (motor->m_iq_set < 0) motor->m_iq_set = (int16_t)-motor->m_iq_set;
            *runtime_command = mag_command;
            break;
        }
        case ESC_MODE_HANDBRAKE: {
            /* VESC handbrake is a fixed static current vector at electrical phase
             * zero, not current-brake with a sign sampled when the command arrives.
             * Do not apply motor-direction inversion to this static-vector current. */
            int16_t hb = clamp_i16(command, -1000, 1000);
            motor->m_iq_set = scale_permille_to_current_limit(hb, conf);
            *runtime_command = hb;
            break;
        }
        case ESC_MODE_VLT:
            motor->m_voltage_q_set = scale_permille_to_limit(physical_command, conf->l_max_voltage);
            motor->m_iq_set = 0;
            *runtime_command = command;
            break;
        case ESC_MODE_DUTY: {
            /* Match VESC semantics more closely: duty is a modulation target, not
             * an unregulated Vq command. Ask the current loop for available torque
             * in the requested direction and cap its voltage by the duty target. */
            int32_t dq = ((int32_t)physical_command * 32767) / 1000;
            if (dq > 32767) dq = 32767;
            if (dq < -32767) dq = -32767;
            motor->m_duty_cycle_set_q15 = (int16_t)dq;
            motor->m_voltage_q_set = 0;
            motor->m_iq_set = physical_command >= 0 ? conf->l_current_max : conf->l_current_min;
            *runtime_command = command;
            break;
        }
        case ESC_MODE_OPEN:
            /* Match VESC open-loop current semantics: current and electrical
             * rotation speed keep their signs. One host command controls both
             * on this board, so reverse command means negative Iq and reverse phase. */
            motor->m_iq_set = scale_permille_to_current_limit(physical_command, conf);
            *runtime_command = command;
            *legacy_open_command = physical_command; /* phase direction in motor.c */
            break;
        default:
            motor->m_iq_set = 0;
            motor->m_voltage_q_set = 0;
            *runtime_command = 0;
            break;
    }
}


static void apply_pid_to_motor(mc_configuration *conf, uint8_t loop,
                               const EscCommandFrame *frame)
{
    if (conf == NULL || frame == NULL) return;
    const int32_t kp = frame->kp_q16 < 0 ? 0 : frame->kp_q16;
    const int32_t ki = frame->ki_q16 < 0 ? 0 : frame->ki_q16;
    const int32_t kd = frame->kd_q16 < 0 ? 0 : frame->kd_q16;

    switch (loop) {
        case ESC_PID_CURRENT_D:
        case ESC_PID_TORQUE_Q:
            /* VESC uses the same PI gains for Id and Iq. */
            conf->foc_current_kp_q16 = kp;
            conf->foc_current_ki_q16 = ki;
            break;
        case ESC_PID_SPEED:
            conf->s_pid_kp_q16 = kp;
            conf->s_pid_ki_q16 = ki;
            conf->s_pid_kd_q16 = kd;
            break;
        default:
            break;
    }
    mc_foc_conf_prepare(conf);
}


static void apply_position_pid(PositionPidConfig *cfg, mc_configuration *conf,
                               const EscCommandFrame *frame)
{
    if (cfg == NULL || conf == NULL || frame == NULL) return;
    cfg->kp_q16 = frame->kp_q16 < 0 ? 0 : frame->kp_q16;
    cfg->ki_q16 = frame->ki_q16 < 0 ? 0 : frame->ki_q16;
    cfg->kd_q16 = frame->kd_q16 < 0 ? 0 : frame->kd_q16;
    cfg->position_min = frame->position_min;
    cfg->position_max = frame->position_max;
    if (cfg->position_min > cfg->position_max) {
        const int32_t tmp = cfg->position_min;
        cfg->position_min = cfg->position_max;
        cfg->position_max = tmp;
    }
    cfg->deadband_ticks = frame->aux_value;

    conf->p_pid_kp_q16 = cfg->kp_q16;
    conf->p_pid_ki_q16 = cfg->ki_q16;
    conf->p_pid_kd_q16 = cfg->kd_q16;
    conf->p_pid_pos_min = cfg->position_min;
    conf->p_pid_pos_max = cfg->position_max;
    conf->p_pid_deadband_ticks = cfg->deadband_ticks;
    mc_foc_conf_prepare(conf);
}


static bool motor_select_valid(uint8_t motor_select)
{
    return motor_select == ESC_MOTOR_LEFT ||
           motor_select == ESC_MOTOR_RIGHT ||
           motor_select == ESC_MOTOR_BOTH;
}

/* Flag encoder_calibrated adalah bukti hasil Auto Detect firmware, bukan opsi
 * konfigurasi bebas dari host. Host boleh mempertahankan bukti yang sudah ada
 * selama parameter yang menentukan interpretasi A/B tidak berubah, atau
 * menghapusnya. Host tidak boleh membuat encoder menjadi "calibrated" hanya
 * dengan menulis bit konfigurasi. */
static bool encoder_calibration_claim_matches(const MotorRuntimeConfig *existing,
                                              const MotorRuntimeConfig *candidate)
{
    if (candidate->encoder_calibrated == 0U) return true;
    if (existing->encoder_calibrated == 0U) return false;
    return candidate->encoder_cpr == existing->encoder_cpr &&
           candidate->encoder_offset_deg == existing->encoder_offset_deg &&
           candidate->sensor_inverted == existing->sensor_inverted;
}


/* Hall Auto Detect membuktikan mapping RAW->electrical-sector dengan satu
 * orientasi sensor tertentu. Mengubah sensor_inverted membuat proof itu tidak
 * lagi sah. motor_inverted sengaja tidak ikut: itu hanya koordinat host. */
static bool hall_calibration_claim_matches(const MotorRuntimeConfig *existing,
                                           const MotorRuntimeConfig *candidate)
{
    if (candidate->hall_calibrated == 0U) return true;
    if (existing->hall_calibrated == 0U) return false;
    return candidate->sensor_inverted == existing->sensor_inverted;
}

static bool apply_motor_runtime_config(uint8_t motor_select, uint16_t config_word, uint16_t encoder_cpr)
{
    if (!motor_select_valid(motor_select)) {
        return false;
    }

    MotorRuntimeConfig decoded_base;
    if (!MotorRuntimeConfig_UnpackWord(config_word, encoder_cpr, &decoded_base)) {
        return false;
    }

    /* MOTOR_CONFIG tidak membawa LUT Hall 18-bit. Saat user hanya mengganti
     * invert/Hall-vs-Encoder/CPR/offset, LUT hasil kalibrasi yang sudah ada
     * harus tetap dipertahankan per motor. */
    MotorRuntimeConfig decoded_left = decoded_base;
    MotorRuntimeConfig decoded_right = decoded_base;
    memcpy(decoded_left.hall_sequence, motorConfigLeft.hall_sequence, 6U);
    decoded_left.hall_lut_valid = motorConfigLeft.hall_lut_valid;
    decoded_left.hall_calibrated = motorConfigLeft.hall_calibrated;
    memcpy(decoded_left.encoder_sequence, motorConfigLeft.encoder_sequence, 4U);
    decoded_left.encoder_sequence_valid = motorConfigLeft.encoder_sequence_valid;
    memcpy(decoded_right.hall_sequence, motorConfigRight.hall_sequence, 6U);
    decoded_right.hall_lut_valid = motorConfigRight.hall_lut_valid;
    decoded_right.hall_calibrated = motorConfigRight.hall_calibrated;
    memcpy(decoded_right.encoder_sequence, motorConfigRight.encoder_sequence, 4U);
    decoded_right.encoder_sequence_valid = motorConfigRight.encoder_sequence_valid;

    /* Jangan percaya bit calibration yang dibuat host. Jika CPR, offset, atau
     * arah sensor berubah, proof lama otomatis gugur dan Auto Detect harus
     * dijalankan lagi. Switching Hall<->Encoder atau motor invert saja tidak
     * menghapus proof selama parameter encoder tetap identik. */
    if (!encoder_calibration_claim_matches(&motorConfigLeft, &decoded_left))
        decoded_left.encoder_calibrated = 0U;
    if (!encoder_calibration_claim_matches(&motorConfigRight, &decoded_right))
        decoded_right.encoder_calibrated = 0U;
    if (!hall_calibration_claim_matches(&motorConfigLeft, &decoded_left))
        decoded_left.hall_calibrated = 0U;
    if (!hall_calibration_claim_matches(&motorConfigRight, &decoded_right))
        decoded_right.hall_calibrated = 0U;
    if (!MotorRuntimeConfig_IsValid(&decoded_left)) {
        decoded_left.hall_calibrated = 0U;
        decoded_left.hall_lut_valid = 0U;
        decoded_left.encoder_calibrated = 0U;
        decoded_left.encoder_sequence_valid = 0U;
    }
    if (!MotorRuntimeConfig_IsValid(&decoded_right)) {
        decoded_right.hall_calibrated = 0U;
        decoded_right.hall_lut_valid = 0U;
        decoded_right.encoder_calibrated = 0U;
        decoded_right.encoder_sequence_valid = 0U;
    }
    /* Konfigurasi dibaca juga oleh ISR PWM. Terapkan config, mode sudut,
     * reset sensor, dan odometri dalam satu critical section agar ISR tidak
     * pernah melihat struct yang baru terisi sebagian. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH) {
        motorConfigLeft = decoded_left;
        apply_sensor_backend_to_foc(&decoded_left, &motorConfLeft);
        mc_foc_init(&motorLeft, &motorConfLeft);
        
        encoderAlignedLeft = false;
        MotorSensor_Reset(&motorSensorStateLeft);
        MotorSensor_PrepareRuntime(&motorConfigLeft, &motorSensorStateLeft, motorConfLeft.foc_motor_pole_pairs);
        odom_l = 0;
    }
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH) {
        motorConfigRight = decoded_right;
        apply_sensor_backend_to_foc(&decoded_right, &motorConfRight);
        mc_foc_init(&motorRight, &motorConfRight);
        
        encoderAlignedRight = false;
        MotorSensor_Reset(&motorSensorStateRight);
        MotorSensor_PrepareRuntime(&motorConfigRight, &motorSensorStateRight, motorConfRight.foc_motor_pole_pairs);
        odom_r = 0;
    }
    if (primask == 0U) __enable_irq();
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH)
        sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH)
        sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);

    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH)
        mc_foc_reset_outer_loops(&motorLeft);
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH)
        mc_foc_reset_outer_loops(&motorRight);
    return true;
}

/* MSG_RESET_SENSOR_CAL: hapus proof commissioning (encoder_calibrated/
 * hall_calibrated) dan LUT/sequence terkait untuk motor terpilih, kembali ke
 * default pabrik. Parameter wiring (sensor_type, CPR, offset, inverted) TIDAK
 * disentuh -- hanya hasil pembelajaran Auto/Manual Detect yang dibuang. Dipakai
 * host saat wiring sensor diganti secara fisik atau saat proof lama ingin
 * dibuang tanpa mengubah konfigurasi lain. Sama seperti apply_motor_runtime_config,
 * caller wajib men-disarm motor target lebih dulu sebelum memanggil ini. */
static bool reset_sensor_lut(uint8_t motor_select, bool reset_hall, bool reset_encoder)
{
    if (!motor_select_valid(motor_select)) return false;
    if (!reset_hall && !reset_encoder) return true;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH) {
        if (reset_hall) MotorRuntimeConfig_ResetHallCalibration(&motorConfigLeft);
        if (reset_encoder) MotorRuntimeConfig_ResetEncoderCalibration(&motorConfigLeft);
        apply_sensor_backend_to_foc(&motorConfigLeft, &motorConfLeft);
        mc_foc_init(&motorLeft, &motorConfLeft);
        
        encoderAlignedLeft = false;
        MotorSensor_Reset(&motorSensorStateLeft);
        MotorSensor_PrepareRuntime(&motorConfigLeft, &motorSensorStateLeft, motorConfLeft.foc_motor_pole_pairs);
        odom_l = 0;
    }
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH) {
        if (reset_hall) MotorRuntimeConfig_ResetHallCalibration(&motorConfigRight);
        if (reset_encoder) MotorRuntimeConfig_ResetEncoderCalibration(&motorConfigRight);
        apply_sensor_backend_to_foc(&motorConfigRight, &motorConfRight);
        mc_foc_init(&motorRight, &motorConfRight);
        
        encoderAlignedRight = false;
        MotorSensor_Reset(&motorSensorStateRight);
        MotorSensor_PrepareRuntime(&motorConfigRight, &motorSensorStateRight, motorConfRight.foc_motor_pole_pairs);
        odom_r = 0;
    }
    if (primask == 0U) __enable_irq();

    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH)
        sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH)
        sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH)
        mc_foc_reset_outer_loops(&motorLeft);
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH)
        mc_foc_reset_outer_loops(&motorRight);
    return true;
}

static uint8_t capture_and_clear_reset_flags(void)
{
    uint8_t flags = 0U;
    const uint32_t csr = RCC->CSR;
#ifdef RCC_CSR_PINRSTF
    if ((csr & RCC_CSR_PINRSTF) != 0U) flags |= 0x01U;
#endif
#ifdef RCC_CSR_PORRSTF
    if ((csr & RCC_CSR_PORRSTF) != 0U) flags |= 0x02U;
#endif
#ifdef RCC_CSR_SFTRSTF
    if ((csr & RCC_CSR_SFTRSTF) != 0U) flags |= 0x04U;
#endif
#ifdef RCC_CSR_IWDGRSTF
    if ((csr & RCC_CSR_IWDGRSTF) != 0U) flags |= 0x08U;
#endif
#ifdef RCC_CSR_WWDGRSTF
    if ((csr & RCC_CSR_WWDGRSTF) != 0U) flags |= 0x10U;
#endif
#ifdef RCC_CSR_LPWRRSTF
    if ((csr & RCC_CSR_LPWRRSTF) != 0U) flags |= 0x20U;
#endif
#ifdef RCC_CSR_RMVF
    RCC->CSR |= RCC_CSR_RMVF;
#endif
    return flags;
}

void RuntimeControl_Init(void)
{
    /* Capture warm-reset fault record sebelum di-acknowledge. Jika FOC pernah
     * HardFault/UsageFault, reboot berikutnya dapat menjelaskannya lewat BASIC
     * telemetry alih-alih terlihat sebagai UART mati tanpa sebab. */
    bootCpuFaultCode = SystemFault_GetBootCode();
    bootResetFlags = capture_and_clear_reset_flags();
    SystemFault_AcknowledgeBootRecord();
    requestedModeLeft = requestedModeRight = ESC_MODE_OPEN;
    runtimeSetpointLeft = runtimeSetpointRight = 0;
    armRequestedLeft = false;
    armRequestedRight = false;
    armedLeft = false;
    armedRight = false;
    previousCommandDirectionLeft = 0;
    previousCommandDirectionRight = 0;
    armRejectLeft = ESC_ARM_REJECT_NONE;
    armRejectRight = ESC_ARM_REJECT_NONE;
    runtimeMotorEnableMask = 0U;
    runtimeOpenFrequencyLeftMilliHz = runtimeOpenFrequencyRightMilliHz = 2000U;
    runtimeOpenPhaseStepLeftQ16 = runtimeOpenPhaseStepRightQ16 = 0;
    runtimeOpenDurationLeftMs = runtimeOpenDurationRightMs = 5000U;
    runtimeOpenParamsValidMask = 0U;
    open_run_reset(&openRunLeft);
    open_run_reset(&openRunRight);
    telemetryPage = ESC_TELEM_BASIC;
    telemetryPidLoop = ESC_PID_SPEED;
    telemetryRateHz = TELEMETRY_RATE_DEFAULT;
    configOneShotPending = false;
    armStatusSnapshotPending = false;
    telemetryLastWasOneShot = false;
    PositionPid_SetDefaults(&positionPidConfigLeft, &motorConfLeft);
    PositionPid_SetDefaults(&positionPidConfigRight, &motorConfRight);
    MotorRuntimeConfig_SetDefaults(&motorConfigLeft);
    MotorRuntimeConfig_SetDefaults(&motorConfigRight);
    Homing_SetDefaults();
    memset(&sensorCal, 0, sizeof(sensorCal));
    sensorCal.state = ESC_SENSOR_CAL_IDLE;
    memset(&encoderAlign, 0, sizeof(encoderAlign));
    sensorCalibrationOpenLoopMask = 0U;
    sensorCalibrationCurrentControlMask = 0U;
    sensorCalibrationCurrentLeft = 0;
    sensorCalibrationCurrentRight = 0;
    sensorCalibrationVoltageLeft = 0;
    sensorCalibrationVoltageRight = 0;
    encoderAlignedLeft = false;
    encoderAlignedRight = false;
    MotorSensor_Reset(&motorSensorStateLeft);
    MotorSensor_Reset(&motorSensorStateRight);
    MotorSensor_PrepareRuntime(&motorConfigLeft, &motorSensorStateLeft, motorConfLeft.foc_motor_pole_pairs);
    MotorSensor_PrepareRuntime(&motorConfigRight, &motorSensorStateRight, motorConfRight.foc_motor_pole_pairs);
    sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);
    sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);
    fault_report_reset_all();
    motorIsrHeartbeatLast = buzzerTimer;
    motorIsrHeartbeatLastChangeMs = HAL_GetTick();
    disarm_outputs();

    HAL_FLASH_Unlock();
    const uint16_t ee_init_status = EE_Init();
    HAL_FLASH_Lock();

    eepromOk = (ee_init_status == HAL_OK);
    eepromVerified = false;
    if (eepromOk) {
        eepromOk = RuntimeSettings_Load();
        if (!eepromOk) {
            /* First boot / CRC salah: simpan default aman lalu lakukan read-back verification. */
            eepromOk = RuntimeSettings_Save();
        } else if (settingsDirty) {
            /* Image lama dimigrasikan ke layout v16. Sensor proof yang masih
             * aman dipertahankan sesuai aturan versi; gain controller lama tidak
             * dipakai karena semantik outer/current loop sudah berubah. */
            eepromOk = RuntimeSettings_Save();
        }
    }

    /* Flag EEPROM hanya menentukan auto-home power-on. Default keduanya OFF.
     * Eksekusi ditunda ke UpdateSlow agar ADC/DC-current sudah hidup. */
    bootHomingPendingMask = (uint8_t)((homingOnBootLeft ? 0x01U : 0U) |
                                      (homingOnBootRight ? 0x02U : 0U));
    homingStartedFromBoot = false;
}


static int16_t current_centiamp_to_internal(const mc_configuration *conf, int32_t centi_amp)
{
    if (conf == NULL || conf->foc_current_units_per_amp == 0U || centi_amp <= 0) return 0;
    const int64_t value = ((int64_t)centi_amp * conf->foc_current_units_per_amp + 50LL) / 100LL;
    return clamp_i16((int32_t)value, 0, conf->l_current_max);
}

static int16_t current_internal_to_centiamp(const mc_configuration *conf, int16_t current)
{
    if (conf == NULL || conf->foc_current_units_per_amp == 0U) return 0;
    const int64_t value = ((int64_t)current * 100LL) / conf->foc_current_units_per_amp;
    return clamp_i16((int32_t)value, INT16_MIN, INT16_MAX);
}

static uint16_t q16_unit_to_q15(int32_t value_q16)
{
    if (value_q16 <= 0) return 0U;
    if (value_q16 >= 65536) return 32767U;
    return (uint16_t)(((int64_t)value_q16 * 32767LL + 32768LL) >> 16);
}

static bool apply_foc_advanced_config_one(mc_configuration *conf, const EscCommandFrame *frame)
{
    if (conf == NULL || frame == NULL || frame->loop > MTPA_MODE_IQ_MEASURED) return false;
    if (frame->setpoint_left < 0 || frame->setpoint_left > 10000000) return false; /* <=10 Wb */
    if (frame->setpoint_right < -1000000 || frame->setpoint_right > 1000000) return false;
    if (frame->output_min < 0 || frame->output_min > 100000 ||
        frame->output_max < 0 || frame->output_max > 1000) return false;
    if (frame->aux_value > 10000U || frame->position_min < 0 || frame->position_max < 0) return false;
    const uint16_t pos_filter_permille = (uint16_t)((frame->config_word >> 1) & 0x03FFU);
    if (pos_filter_permille > 1000U) return false;

    const int16_t fw_internal = current_centiamp_to_internal(conf, frame->output_min);
    /* Reject rather than silently clip a GUI request above motor current limit. */
    if (frame->output_min > 0 && fw_internal >= conf->l_current_max) {
        const int32_t max_ca = ((int32_t)conf->l_current_max * 100) / conf->foc_current_units_per_amp;
        if (frame->output_min > max_ca) return false;
    }
    if (frame->loop != MTPA_MODE_OFF && (frame->setpoint_left == 0 || frame->setpoint_right == 0)) return false;

    conf->foc_motor_flux_linkage_uwb = (uint32_t)frame->setpoint_left;
    conf->foc_motor_ld_lq_diff_uh = frame->setpoint_right;
    conf->foc_mtpa_mode = (mc_mtpa_mode)frame->loop;
    conf->foc_fw_current_max = fw_internal;
    conf->foc_fw_duty_start_q15 = (uint16_t)(((uint32_t)frame->output_max * 32767U + 500U) / 1000U);
    conf->foc_fw_ramp_time_ms = frame->aux_value;
    conf->foc_fw_q_current_factor_q15 = q16_unit_to_q15(frame->ki_q16);
    conf->foc_fw_backoff_q15 = q16_unit_to_q15(frame->kd_q16);
    conf->s_pid_kd_filter_q15 = q16_unit_to_q15(frame->i_limit_q16);
    conf->s_pid_min_erpm = frame->position_min;
    conf->s_pid_ramp_erpms_s = frame->position_max;
    conf->p_pid_kd_proc_q16 = frame->kp_q16;
    conf->p_pid_kd_filter_q15 = (uint16_t)(((uint32_t)pos_filter_permille * 32767U + 500U) / 1000U);
    conf->p_pid_gain_dec_ticks = frame->telemetry_mask;
    conf->s_pid_allow_braking = (frame->config_word & 0x0001U) != 0U;
    mc_foc_conf_prepare(conf);
    return true;
}


void RuntimeControl_VescAlive(void)
{
    lastCommandTick = RuntimeControl_MonotonicMs();
    if (!linkActive) { linkActive = true; ++reconnectCounter; }
}

void RuntimeControl_VescReleaseAll(void)
{
    armRequestedLeft = false;
    armRequestedRight = false;
    runtimeSetpointLeft = 0;
    runtimeSetpointRight = 0;
    disarm_outputs();
}

void RuntimeControl_VescSetOne(bool left, uint8_t esc_mode, int32_t setpoint, bool arm)
{
    EscCommandFrame f;
    memset(&f, 0, sizeof(f));
    f.start = ESC_FRAME_START;
    f.version = ESC_PROTOCOL_VERSION;
    f.type = ESC_MSG_CONTROL;
    f.mode_left = requestedModeLeft;
    f.mode_right = requestedModeRight;
    f.setpoint_left = runtimeSetpointLeft;
    f.setpoint_right = runtimeSetpointRight;
    f.flags = 0U;
    /* Side-arm format preserves the other motor exactly as it is now. */
    if (left) {
        f.mode_left = esc_mode;
        f.setpoint_left = setpoint;
        if (arm) f.flags |= ESC_FLAG_ARM_LEFT;
        if (armedRight || armRequestedRight) f.flags |= ESC_FLAG_ARM_RIGHT;
    } else {
        f.mode_right = esc_mode;
        f.setpoint_right = setpoint;
        if (armedLeft || armRequestedLeft) f.flags |= ESC_FLAG_ARM_LEFT;
        if (arm) f.flags |= ESC_FLAG_ARM_RIGHT;
    }
    RuntimeControl_OnCommand(&f);
}

void RuntimeControl_OnCommand(const EscCommandFrame *frame)
{
    lastHostSequence = frame->sequence;
    lastCommandTick = RuntimeControl_MonotonicMs();

    if (!linkActive) {
        linkActive = true;
        ++reconnectCounter;
    }

    switch ((EscMessageType)frame->type) {
        case ESC_MSG_HELLO:
            /* HELLO adalah handshake/idempotent keepalive, BUKAN perintah safety.
             * GUI dapat mengirim HELLO ulang ketika RX telemetry tersendat. Pada
             * v11.9 HELLO melakukan global DISARM sehingga motor yang sebenarnya
             * sehat tampak tidak pernah bisa ARM. Safety tetap dimiliki oleh
             * ESC_MSG_DISARM dan watchdog SERIAL_TIMEOUT_MS. */
            queue_arm_status_snapshot();
            break;

        case ESC_MSG_CONTROL:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            /* Auto-home power-on punya prioritas terhadap command kontrol rutin dari
             * ROS/GUI yang baru connect. DISARM/ABORT tetap dapat menghentikannya. */
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U || homingPendingOperationLeft != 0U || homingPendingOperationRight != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            if (mode_valid(frame->mode_left) && mode_valid(frame->mode_right)) {
                const bool left_mode_changed = frame->mode_left != requestedModeLeft;
                const bool right_mode_changed = frame->mode_right != requestedModeRight;
                requestedModeLeft = frame->mode_left;
                requestedModeRight = frame->mode_right;
                runtimeSetpointLeft = frame->setpoint_left;
                runtimeSetpointRight = frame->setpoint_right;

                /* CONTROL extended OPEN memakai field yang tidak dipakai PID pada
                 * message ini: position_min/max = electrical frequency mHz LEFT/RIGHT,
                 * output_min/max = duration ms LEFT/RIGHT. */
                if ((frame->flags & ESC_FLAG_OPEN_PARAMS_VALID) != 0U) {
                    runtimeOpenFrequencyLeftMilliHz = (uint32_t)clamp_i32(
                        frame->position_min, 0, (int32_t)OPEN_FREQUENCY_MAX_MILLIHZ);
                    runtimeOpenFrequencyRightMilliHz = (uint32_t)clamp_i32(
                        frame->position_max, 0, (int32_t)OPEN_FREQUENCY_MAX_MILLIHZ);
                    runtimeOpenPhaseStepLeftQ16 = open_phase_step_q16(runtimeOpenFrequencyLeftMilliHz);
                    runtimeOpenPhaseStepRightQ16 = open_phase_step_q16(runtimeOpenFrequencyRightMilliHz);
                    runtimeOpenDurationLeftMs = (uint32_t)clamp_i32(
                        frame->output_min, 0, (int32_t)OPEN_DURATION_MAX_MS);
                    runtimeOpenDurationRightMs = (uint32_t)clamp_i32(
                        frame->output_max, 0, (int32_t)OPEN_DURATION_MAX_MS);
                    runtimeOpenParamsValidMask = 0x03U;
                } else {
                    runtimeOpenParamsValidMask = 0U;
                }

                if ((frame->flags & ESC_FLAG_OPEN_RESTART) != 0U) {
                    open_run_reset(&openRunLeft);
                    open_run_reset(&openRunRight);
                }
                if (left_mode_changed) open_run_reset(&openRunLeft);
                if (right_mode_changed) open_run_reset(&openRunRight);

                /* v11.9: ARM benar-benar independen. Jika host memakai salah satu
                 * side-arm bit, legacy ARM bit0 diabaikan. Host lama yang hanya
                 * mengenal bit0 tetap meng-ARM kedua motor seperti sebelumnya. */
                const bool side_arm_format =
                    (frame->flags & (ESC_FLAG_ARM_LEFT | ESC_FLAG_ARM_RIGHT)) != 0U;
                const bool host_request_left = side_arm_format
                    ? ((frame->flags & ESC_FLAG_ARM_LEFT) != 0U)
                    : ((frame->flags & ESC_FLAG_ARM) != 0U);
                const bool host_request_right = side_arm_format
                    ? ((frame->flags & ESC_FLAG_ARM_RIGHT) != 0U)
                    : ((frame->flags & ESC_FLAG_ARM) != 0U);

                /* Match VESC input semantics: there is no separate ARM-release
                 * handshake. Fault-stop time is enforced in try_arm_motor(); after
                 * it expires a still-streaming command may run again if healthy. */
                if (!host_request_left) fault_report_allow_new(true);
                if (!host_request_right) fault_report_allow_new(false);
                const bool request_left = host_request_left;
                const bool request_right = host_request_right;

                if (left_mode_changed && armedLeft) disarm_motor(ESC_MOTOR_LEFT);
                if (right_mode_changed && armedRight) disarm_motor(ESC_MOTOR_RIGHT);

                const bool arm_request_changed =
                    (armRequestedLeft != request_left) || (armRequestedRight != request_right);
                armRequestedLeft = request_left;
                armRequestedRight = request_right;
                if (arm_request_changed) queue_arm_status_snapshot();
                if (!request_left) {
                    armRejectLeft = ESC_ARM_REJECT_NONE;
                    disarm_motor(ESC_MOTOR_LEFT);
                }
                if (!request_right) {
                    armRejectRight = ESC_ARM_REJECT_NONE;
                    disarm_motor(ESC_MOTOR_RIGHT);
                }
            }
            break;

        case ESC_MSG_PID_CONFIG:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            if (frame->loop > ESC_PID_POSITION || !motor_select_valid(frame->motor)) break;
            /*
             * Gain dipakai FOC ISR frekuensi tinggi. DISARM hanya motor target
             * agar satu control step tidak membaca campuran Kp/Ki/Kd. Motor
             * sisi lain tetap boleh berjalan. SAVE EEPROM di bawah tetap
             * melakukan global safe-disarm karena operasi flash dapat stall ISR.
             */
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedLeft = false;
                disarm_motor(ESC_MOTOR_LEFT);
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedRight = false;
                disarm_motor(ESC_MOTOR_RIGHT);
            }
            telemetryPidLoop = frame->loop;
            settingsDirty = true;
            if (frame->loop == ESC_PID_POSITION) {
                if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH)
                    apply_position_pid(&positionPidConfigLeft, &motorConfLeft, frame);
                if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH)
                    apply_position_pid(&positionPidConfigRight, &motorConfRight, frame);
            } else {
                if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH)
                    apply_pid_to_motor(&motorConfLeft, frame->loop, frame);
                if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH)
                    apply_pid_to_motor(&motorConfRight, frame->loop, frame);
            }
            /* Gain baru tidak boleh mewarisi integrator/derivative state dari gain
             * lama. Motor target sudah DISARM di atas, jadi reset controller di
             * sini aman dan deterministik. Phase scheduling kiri/kanan dipasang
             * kembali setelah Init. */
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                mc_foc_init(&motorLeft, &motorConfLeft);
                
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                mc_foc_init(&motorRight, &motorConfRight);
                
            }
            if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U) eepromOk = RuntimeSettings_Save();
            break;

        case ESC_MSG_TELEMETRY_CONFIG:
            if (frame->telemetry_page <= ESC_TELEM_RAW) telemetryPage = frame->telemetry_page;
            if (frame->loop <= ESC_PID_POSITION) telemetryPidLoop = frame->loop;
            configOneShotPending = false;
            telemetryLastWasOneShot = false;
            telemetryRateHz = frame->telemetry_rate_hz;
            if (telemetryRateHz < 1U) telemetryRateHz = 1U;
            if (telemetryRateHz > TELEMETRY_RATE_MAX) telemetryRateHz = TELEMETRY_RATE_MAX;
            telemetryMask = frame->telemetry_mask;
            break;

        case ESC_MSG_SAVE_EEPROM:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            eepromOk = RuntimeSettings_Save();
            break;

        case ESC_MSG_LOAD_EEPROM:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            /* Gain/limit dapat berubah besar; load hanya dilakukan dalam kondisi DISARM.
             * PENTING: saat host sudah terhubung, jangan biarkan telemetry rate/page
             * lama dari EEPROM menimpa streaming aktif hasil HELLO. GUI melakukan
             * LOAD otomatis setelah feedback pertama; sebelumnya ini dapat mengubah
             * 50 Hz menjadi rate EEPROM yang sangat lambat dan terlihat seperti link mati. */
            {
                const uint16_t active_rate = telemetryRateHz;
                const uint8_t active_page = telemetryPage;
                const uint8_t active_pid_loop = telemetryPidLoop;
                const uint32_t active_mask = telemetryMask;
                const bool keep_stream = linkActive;
                armRequestedLeft = false;
                armRequestedRight = false;
                disarm_outputs();
                eepromOk = RuntimeSettings_Load();
                if (eepromOk && keep_stream) {
                    telemetryRateHz = active_rate;
                    telemetryPage = active_page;
                    telemetryPidLoop = active_pid_loop;
                    telemetryMask = active_mask;
                }
            }
            break;

        case ESC_MSG_ZERO_POSITION:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (!motor_select_valid(frame->motor)) break;
            /* Mengubah referensi posisi saat motor aktif dapat membuat POS melonjak.
             * Hentikan hanya sisi yang referensinya diubah. */
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedLeft = false;
                disarm_motor(ESC_MOTOR_LEFT);
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedRight = false;
                disarm_motor(ESC_MOTOR_RIGHT);
            }
            MotorOdometry_Zero(frame->motor);
            break;

        case ESC_MSG_DISARM:
            bootHomingPendingMask = 0U;
            /* Explicit operator DISARM adalah acknowledgement untuk fault deadline
             * ISR. Clear latch setelah PWM dimatikan di bawah; request ARM baru
             * kemudian boleh mencoba lagi. */
            /* Commissioning/homing adalah state-machine eksklusif: DISARM apa pun
             * membatalkannya. Pada runtime normal, target motor dihormati. */
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING)
                sensor_cal_finish(ESC_SENSOR_CAL_ABORTED, 4U);
            if (encoderAlign.active) encoder_alignment_abort();
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            if (frame->motor == ESC_MOTOR_LEFT) {
                armRequestedLeft = false;
                fault_report_allow_new(true);
                armRejectLeft = ESC_ARM_REJECT_NONE;
                disarm_motor(ESC_MOTOR_LEFT);
            } else if (frame->motor == ESC_MOTOR_RIGHT) {
                armRequestedRight = false;
                fault_report_allow_new(false);
                armRejectRight = ESC_ARM_REJECT_NONE;
                disarm_motor(ESC_MOTOR_RIGHT);
            } else {
                armRequestedLeft = false;
                armRequestedRight = false;
                fault_report_allow_new(true);
                fault_report_allow_new(false);
                armRejectLeft = ESC_ARM_REJECT_NONE;
                armRejectRight = ESC_ARM_REJECT_NONE;
                disarm_outputs();
            }
            MotorControl_ClearIsrOverrunFault(
                (frame->motor == ESC_MOTOR_LEFT) ? 0x01U :
                ((frame->motor == ESC_MOTOR_RIGHT) ? 0x02U : 0x03U));
            break;

        case ESC_MSG_MOTOR_CONFIG:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            /* Apply konfigurasi RAM hanya menghentikan motor target. Sisi lain
             * tidak boleh jatuh ARM hanya karena dropdown/config motor ini berubah. */
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedLeft = false;
                disarm_motor(ESC_MOTOR_LEFT);
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedRight = false;
                disarm_motor(ESC_MOTOR_RIGHT);
            }
            if (apply_motor_runtime_config(frame->motor, frame->config_word, frame->aux_value)) {
                settingsDirty = true;
                if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U) {
                    eepromOk = RuntimeSettings_Save();
                }
            }
            break;

        case ESC_MSG_HOMING_CONFIG: {
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (!motor_select_valid(frame->motor)) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            armRequestedLeft = false;
            armRequestedRight = false;
            disarm_outputs();
            HomingMotorConfig candidate;
            candidate.current_threshold_centi_amp = frame->aux_value;
            candidate.search_command = clamp_i16(frame->setpoint_left, -500, 500);
            const uint16_t timeout_candidate = (uint16_t)clamp_i32(frame->output_min, 500, 30000);
            const uint16_t debounce_candidate = (uint16_t)clamp_i32(frame->output_max, 20, 2000);
            if (!HomingConfig_IsValid(&candidate) || !HomingTiming_IsValid(timeout_candidate, debounce_candidate)) break;
            const bool auto_home_boot = (frame->config_word & 0x0001U) != 0U;
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                homingConfigLeft = candidate;
                homingOnBootLeft = auto_home_boot;
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                homingConfigRight = candidate;
                homingOnBootRight = auto_home_boot;
            }
            homingTimeoutMs = timeout_candidate;
            homingDebounceMs = debounce_candidate;
            settingsDirty = true;
            if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U) eepromOk = RuntimeSettings_Save();
            break;
        }

        case ESC_MSG_REQUEST_HOMING_CONFIG:
            telemetryOneShotPage = ESC_TELEM_HOMING_CONFIG;
            configOneShotPending = true;
            break;

        case ESC_MSG_START_HOMING:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (!motor_select_valid(frame->motor)) break;
            armRequestedLeft = false;
            armRequestedRight = false;
            disarm_outputs();
            if (HomingConfig_IsValid(&homingConfigLeft) && HomingConfig_IsValid(&homingConfigRight) &&
                HomingTiming_IsValid(homingTimeoutMs, homingDebounceMs) &&
                homing_target_fault_free(frame->motor)) {
                homingStartedFromBoot = false;
                bootHomingPendingMask = 0U;
                homing_start(frame->motor);
            }
            break;

        case ESC_MSG_ABORT_HOMING:
            bootHomingPendingMask = 0U;
            homing_abort_all(ESC_HOMING_ABORTED);
            armRequestedLeft = false;
            armRequestedRight = false;
            disarm_outputs();
            break;

        case ESC_MSG_START_SENSOR_AUTODETECT:
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            bootHomingPendingMask = 0U;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            /* v11.9 atomic commissioning: GUI menaruh config target pada frame
             * START itu sendiri. position_min dipakai sebagai CPR hanya pada
             * command START_SENSOR; client lama punya default negatif sehingga
             * jalur kompatibilitas tetap memakai config EEPROM/RAM saat ini. */
            if (frame->motor != ESC_MOTOR_LEFT && frame->motor != ESC_MOTOR_RIGHT) break;
            if (frame->position_min >= (int32_t)MOTOR_ENCODER_CPR_MIN &&
                frame->position_min <= (int32_t)MOTOR_ENCODER_CPR_MAX) {
                if (!apply_motor_runtime_config(frame->motor, frame->config_word,
                                                (uint16_t)frame->position_min)) {
                    sensor_cal_start_rejected(ESC_SENSOR_CAL_METHOD_AUTO, frame->motor);
                    break;
                }
                settingsDirty = true;
            }
            if (!sensor_cal_start(ESC_SENSOR_CAL_METHOD_AUTO, frame->motor,
                                  clamp_i16(frame->setpoint_left ? frame->setpoint_left : SENSOR_CAL_CURRENT_DEFAULT_INTERNAL,
                                            SENSOR_CAL_CURRENT_MIN_INTERNAL, SENSOR_CAL_CURRENT_MAX_INTERNAL),
                                  SENSOR_CAL_MANUAL_DEFAULT_MS))
                sensor_cal_start_rejected(ESC_SENSOR_CAL_METHOD_AUTO, frame->motor);
            break;

        case ESC_MSG_START_SENSOR_MANUAL_CAL:
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            bootHomingPendingMask = 0U;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            if (frame->motor != ESC_MOTOR_LEFT && frame->motor != ESC_MOTOR_RIGHT) break;
            if (frame->position_min >= (int32_t)MOTOR_ENCODER_CPR_MIN &&
                frame->position_min <= (int32_t)MOTOR_ENCODER_CPR_MAX) {
                if (!apply_motor_runtime_config(frame->motor, frame->config_word,
                                                (uint16_t)frame->position_min)) {
                    sensor_cal_start_rejected(ESC_SENSOR_CAL_METHOD_MANUAL, frame->motor);
                    break;
                }
                settingsDirty = true;
            }
            if (!sensor_cal_start(ESC_SENSOR_CAL_METHOD_MANUAL, frame->motor, 0, frame->aux_value))
                sensor_cal_start_rejected(ESC_SENSOR_CAL_METHOD_MANUAL, frame->motor);
            break;

        case ESC_MSG_ABORT_SENSOR_CAL:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING)
                sensor_cal_finish(ESC_SENSOR_CAL_ABORTED, 4U);
            encoder_alignment_abort();
            break;

        case ESC_MSG_REQUEST_SENSOR_CAL:
            sensorCalTelemetryCandidate = (frame->aux_value == 1U);
            telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;
            configOneShotPending = true;
            break;

        case ESC_MSG_RESET_SENSOR_CAL:
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (homingStartedFromBoot && homingActiveMask != 0U) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
            /* Sama seperti MOTOR_CONFIG: hanya motor target yang di-disarm. */
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedLeft = false;
                disarm_motor(ESC_MOTOR_LEFT);
            }
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH) {
                armRequestedRight = false;
                disarm_motor(ESC_MOTOR_RIGHT);
            }
            if (reset_sensor_lut(frame->motor,
                                 (frame->aux_value & ESC_RESET_SENSOR_HALL) != 0U,
                                 (frame->aux_value & ESC_RESET_SENSOR_ENCODER) != 0U)) {
                settingsDirty = true;
                if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U) {
                    eepromOk = RuntimeSettings_Save();
                }
            }
            sensorCalTelemetryCandidate = false;
            telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;
            configOneShotPending = true;
            break;

        case ESC_MSG_FOC_ADV_CONFIG: {
            if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) break;
            if (!motor_select_valid(frame->motor)) break;
            if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);

            /* Transactional decode first. Never leave LEFT updated if RIGHT fails. */
            mc_configuration left_candidate = motorConfLeft;
            mc_configuration right_candidate = motorConfRight;
            bool ok = true;
            if (frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH)
                ok = apply_foc_advanced_config_one(&left_candidate, frame) && ok;
            if (frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH)
                ok = apply_foc_advanced_config_one(&right_candidate, frame) && ok;
            if (!ok) break;

            const bool update_left = frame->motor == ESC_MOTOR_LEFT || frame->motor == ESC_MOTOR_BOTH;
            const bool update_right = frame->motor == ESC_MOTOR_RIGHT || frame->motor == ESC_MOTOR_BOTH;
            if (update_left) {
                armRequestedLeft = false;
                disarm_motor(ESC_MOTOR_LEFT);
            }
            if (update_right) {
                armRequestedRight = false;
                disarm_motor(ESC_MOTOR_RIGHT);
            }

            /* Even while DISARMED the 16-kHz ISR still snapshots configuration
             * and motor state. Commit the selected side(s) atomically so it can
             * never observe a half-copied mc_configuration or half-zeroed state.
             * The unselected motor is not reinitialized. */
            const uint32_t adv_primask = __get_PRIMASK();
            __disable_irq();
            if (update_left) {
                motorConfLeft = left_candidate;
                mc_foc_init(&motorLeft, &motorConfLeft);
            }
            if (update_right) {
                motorConfRight = right_candidate;
                mc_foc_init(&motorRight, &motorConfRight);
            }
            if (adv_primask == 0U) __enable_irq();
            settingsDirty = true;
            if ((frame->flags & ESC_FLAG_SAVE_AFTER_APPLY) != 0U)
                eepromOk = RuntimeSettings_Save();
            break;
        }

        case ESC_MSG_REQUEST_FOC_ADV_CONFIG:
            configMotor = (frame->motor == ESC_MOTOR_RIGHT) ? ESC_MOTOR_RIGHT : ESC_MOTOR_LEFT;
            telemetryOneShotPage = ESC_TELEM_FOC_ADV_CONFIG;
            configOneShotPending = true;
            break;

        case ESC_MSG_REQUEST_MOTOR_CONFIG:
            telemetryOneShotPage = ESC_TELEM_MOTOR_CONFIG;
            configOneShotPending = true;
            break;

        case ESC_MSG_REQUEST_CONFIG:
            if (frame->loop <= ESC_PID_POSITION) telemetryPidLoop = frame->loop;
            configMotor = (frame->motor == ESC_MOTOR_RIGHT) ? ESC_MOTOR_RIGHT : ESC_MOTOR_LEFT;
            telemetryOneShotPage = ESC_TELEM_CONFIG;
            configOneShotPending = true;
            break;

        default:
            break;
    }
}

void RuntimeControl_UpdateSlow(uint32_t dt_ms)
{
    uint32_t now = RuntimeControl_MonotonicMs();
    uint32_t age = now - lastCommandTick;

    const uint32_t motor_heartbeat = buzzerTimer;
    if (motor_heartbeat != motorIsrHeartbeatLast) {
        motorIsrHeartbeatLast = motor_heartbeat;
        motorIsrHeartbeatLastChangeMs = now;
    } else if ((uint32_t)(now - motorIsrHeartbeatLastChangeMs) > MOTOR_ISR_HEARTBEAT_TIMEOUT_MS) {
        /* ADC/current-control ISR berhenti total: block KEDUA motor. UART/main
         * dibiarkan hidup agar GUI menerima CONTROL_ISR_OVERRUN, bukan RX=0. */
        motorControlIsrOverrunFaultMask |= 0x03U;
        runtimeMotorEnableMask = 0U;
        sensorCalibrationOpenLoopMask = 0U;
        sensorCalibrationCurrentControlMask = 0U;
        sensorCalibrationCurrentLeft = 0; sensorCalibrationCurrentRight = 0;
        sensorCalibrationVoltageLeft = 0; sensorCalibrationVoltageRight = 0;
        mc_foc_set_current_commissioning(&motorLeft, false);
        mc_foc_set_current_commissioning(&motorRight, false);
        motorLeft.m_id_set = 0; motorLeft.m_iq_set = 0;
        motorRight.m_id_set = 0; motorRight.m_iq_set = 0;
        enable = 0U;
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    }

    /* Health sensor diperbarui bahkan sebelum link host aktif agar auto-home boot
     * tidak dapat menjalankan closed-loop dengan sensor EEPROM yang belum siap. */
    sensor_health_update(now);
    fault_report_service(now);

    /* Auto-home power-on harus dapat berjalan walau GUI/ROS belum terhubung.
     * Motor baru mulai setelah delay boot dan hanya jika konfigurasi/fault aman. */
    if (bootHomingPendingMask != 0U && homingActiveMask == 0U &&
        !homingStartedFromBoot && now >= BOOT_HOMING_DELAY_MS) {
        if (HomingConfig_IsValid(&homingConfigLeft) && HomingConfig_IsValid(&homingConfigRight) &&
            HomingTiming_IsValid(homingTimeoutMs, homingDebounceMs) &&
            homing_target_fault_free((bootHomingPendingMask == 0x01U) ? ESC_MOTOR_LEFT :
                                     ((bootHomingPendingMask == 0x02U) ? ESC_MOTOR_RIGHT : ESC_MOTOR_BOTH))) {
            uint8_t target = ESC_MOTOR_BOTH;
            if (bootHomingPendingMask == 0x01U) target = ESC_MOTOR_LEFT;
            else if (bootHomingPendingMask == 0x02U) target = ESC_MOTOR_RIGHT;
            homing_start(target);
            homingStartedFromBoot = true;
        } else {
            homingRuntimeLeft.state = (bootHomingPendingMask & 0x01U) ? ESC_HOMING_FAULT : homingRuntimeLeft.state;
            homingRuntimeRight.state = (bootHomingPendingMask & 0x02U) ? ESC_HOMING_FAULT : homingRuntimeRight.state;
        }
        bootHomingPendingMask = 0U; /* satu attempt per power-on; tidak auto-retry */
    }

    if (homingStartedFromBoot &&
        (homingActiveMask != 0U || homingPendingOperationLeft != 0U ||
         homingPendingOperationRight != 0U || encoderAlign.active)) {
        if (encoderAlign.active) (void)encoder_alignment_service(now);
        homing_service_pending_after_sync(now);
        if (homingActiveMask != 0U) homing_service_active(now, dt_ms);
        if (homingActiveMask != 0U || homingPendingOperationLeft != 0U ||
            homingPendingOperationRight != 0U || encoderAlign.active) return;
        homingStartedFromBoot = false;
    }

    /* VESC App Settings owns the communication watchdog. VESC uses 0 to
     * disable the timeout; keep that semantic at the compatibility boundary. */
    const uint32_t link_timeout_ms = vescAppConfig.timeout_ms;
    if (!linkActive || (link_timeout_ms != 0U && age > link_timeout_ms)) {
        if (linkActive) linkActive = false;
        /* Homing/calibration manual membutuhkan link host; timeout terminal. */
        if (homingActiveMask != 0U) homing_abort_all(ESC_HOMING_ABORTED);
        if (sensorCal.state == ESC_SENSOR_CAL_RUNNING)
            sensor_cal_finish(ESC_SENSOR_CAL_ABORTED, 5U);
        encoder_alignment_abort();
        if (armRequestedLeft || armedLeft) armRejectLeft = ESC_ARM_REJECT_LINK_TIMEOUT;
        if (armRequestedRight || armedRight) armRejectRight = ESC_ARM_REJECT_LINK_TIMEOUT;
        armRequestedLeft = false;
        armRequestedRight = false;
        requestedModeLeft = requestedModeRight = ESC_MODE_OPEN;
        queue_arm_status_snapshot();
        disarm_outputs();
        return;
    }

    /* V15: measured ISR deadline overruns are diagnostics only. The hot path no
     * longer creates a persistent overrun/over-current permission latch. A TOTAL
     * ADC/DMA heartbeat loss is still handled above as a catastrophic condition,
     * while the stock DC-link current chop in motor.c removes MOE sample-by-sample.
     * Commissioning therefore cannot be aborted by stale diagnostic masks. */
    motorControlIsrOverrunFaultMask = 0U;
    motorControlOvercurrentFaultMask = 0U;

    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING) {
        sensor_cal_service(now, dt_ms);
        return;
    }

    /* Alignment encoder hanya mengambil alih motor target. Motor lain yang sudah
     * ARMED tetap mempertahankan command terakhir/terbaru dan PWM gate-nya. */
    if (encoderAlign.active) {
        (void)encoder_alignment_service(now);
    }
    homing_service_pending_after_sync(now);

    if (homingActiveMask != 0U) {
        homing_service_active(now, dt_ms);
        return;
    }

    /* V15: SensorHealth remains visible in HBTS/diagnostics but it is NOT another
     * sticky PWM gate. The ISR already requires feedback_valid in the SAME sample;
     * a bad Hall/encoder sample therefore releases MOE immediately and can recover
     * on the next valid sample without deleting the host SET command. Only an
     * explicit FOC-core fault is terminal here. */
    const uint8_t left_core_code = motorOutputLeft.fault_code;
    const uint8_t right_core_code = motorOutputRight.fault_code;
    if (armedLeft && left_core_code != 0U) {
        armRejectLeft = ESC_ARM_REJECT_SENSOR_OR_FOC;
        armRequestedLeft = false;
        fault_report_latch(true, left_core_code, now);
        disarm_motor(ESC_MOTOR_LEFT);
    }
    if (armedRight && right_core_code != 0U) {
        armRejectRight = ESC_ARM_REJECT_SENSOR_OR_FOC;
        armRequestedRight = false;
        fault_report_latch(false, right_core_code, now);
        disarm_motor(ESC_MOTOR_RIGHT);
    }

    /* Coba kedua sisi setiap siklus. Jika LEFT sedang alignment Encoder, RIGHT
     * Hall/SPD tetap boleh ARM. Hanya motor yang membutuhkan alignment Encoder
     * yang sama/berikutnya akan menunggu giliran. */
    try_arm_motor(true, now);
    try_arm_motor(false, now);

    /* VESC-style outer loops run here at ~200 Hz. V15 restores the V1 timing model: PWM/ADC and BOTH per-motor current loops run at 16 kHz. */
    if (armedLeft) {
        apply_runtime_target_one(true, requestedModeLeft, runtimeSetpointLeft, dt_ms, now);
        update_direction_change_guard(true, now);
    } else if (!(encoderAlign.active && encoderAlign.motor == ESC_MOTOR_LEFT)) {
        runtimeCommandLeft = 0;
        pwml = 0;
        controlModeLeftFoc = (uint8_t)CONTROL_MODE_NONE;
        mc_foc_set_control_mode(&motorLeft, CONTROL_MODE_NONE);
        motorLeft.m_iq_set = 0;
        motorLeft.m_voltage_q_set = 0;
        update_direction_change_guard(true, now);
    }

    if (armedRight) {
        apply_runtime_target_one(false, requestedModeRight, runtimeSetpointRight, dt_ms, now);
        update_direction_change_guard(false, now);
    } else if (!(encoderAlign.active && encoderAlign.motor == ESC_MOTOR_RIGHT)) {
        runtimeCommandRight = 0;
        pwmr = 0;
        controlModeRightFoc = (uint8_t)CONTROL_MODE_NONE;
        mc_foc_set_control_mode(&motorRight, CONTROL_MODE_NONE);
        motorRight.m_iq_set = 0;
        motorRight.m_voltage_q_set = 0;
        update_direction_change_guard(false, now);
    }

    refresh_master_enable();
}

bool RuntimeControl_LinkActive(void) { return linkActive; }
bool RuntimeControl_Armed(void) { return armedLeft || armedRight; }
bool RuntimeControl_ArmedLeft(void) { return armedLeft; }
bool RuntimeControl_ArmedRight(void) { return armedRight; }
uint8_t RuntimeControl_ArmRejectLeft(void) { return armRejectLeft; }
uint8_t RuntimeControl_ArmRejectRight(void) { return armRejectRight; }
uint16_t RuntimeControl_FaultStopRemainingMs(bool left)
{
    const uint32_t now = RuntimeControl_MonotonicMs();
    const bool active = left ? faultReportActiveLeft : faultReportActiveRight;
    const uint32_t until = left ? faultReportUntilLeft : faultReportUntilRight;
    if (!active || (int32_t)(now - until) >= 0) return 0U;
    const uint32_t remaining = until - now;
    return (uint16_t)(remaining > 65535U ? 65535U : remaining);
}
uint8_t RuntimeControl_SensorCalibrationState(void) { return sensorCal.state; }
uint8_t RuntimeControl_SensorCalibrationMotor(void) { return sensorCal.motor; }
uint8_t RuntimeControl_SensorCalibrationType(void) { return sensorCal.sensor_type; }
uint8_t RuntimeControl_SensorCalibrationResultCode(void) { return sensorCal.result_code; }
int16_t RuntimeControl_SensorCalibrationTargetCurrentInternal(void)
{
    return sensorCal.state == ESC_SENSOR_CAL_RUNNING ? sensorCal.drive_current_internal : 0;
}
int16_t RuntimeControl_SensorCalibrationMeasuredCurrentInternal(void)
{
    return sensorCal.state == ESC_SENSOR_CAL_RUNNING ? sensor_cal_measured_current_internal(sensorCal.motor) : 0;
}
uint16_t RuntimeControl_SensorCalibrationElectricalPhaseQ16(void)
{
    if (sensorCal.state != ESC_SENSOR_CAL_RUNNING) return 0U;
    return (sensorCal.motor == ESC_MOTOR_RIGHT) ? sensorCalibrationPhaseRightQ16 : sensorCalibrationPhaseLeftQ16;
}
int8_t RuntimeControl_SensorCalibrationSweepDirection(void) { return sensorCal.sweep_direction; }
uint8_t RuntimeControl_SensorCalibrationForwardCycles(void) { return sensorCal.forward_cycles; }
uint8_t RuntimeControl_SensorCalibrationReverseCycles(void) { return sensorCal.reverse_cycles; }
uint16_t RuntimeControl_SensorCalibrationCompletedCycles(void) { return sensorCal.completed_cycles; }
bool RuntimeControl_SensorCalibrationMotionDetected(void) { return sensorCal.motion_detected; }
uint32_t RuntimeControl_SensorCalibrationMotionCounter(void)
{
    return (uint32_t)sensorCal.last_motion_position;
}
uint16_t RuntimeControl_SensorCalibrationMotionAgeMs(void)
{
    const uint32_t age = RuntimeControl_MonotonicMs() - sensorCal.last_motion_tick;
    return (uint16_t)(age > UINT16_MAX ? UINT16_MAX : age);
}
uint8_t RuntimeControl_SensorCalibrationObservedHallMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t n = 0U; n < sensorCal.observed_hall_count && n < 6U; ++n)
        mask |= (uint8_t)(1U << (sensorCal.observed_hall_sequence[n] & 0x07U));
    return mask;
}
uint8_t RuntimeControl_SensorCalibrationObservedEncoderMask(void)
{
    uint8_t mask = 0U;
    for (uint8_t n = 0U; n < sensorCal.observed_encoder_count && n < 4U; ++n)
        mask |= (uint8_t)(1U << (sensorCal.observed_encoder_sequence[n] & 0x03U));
    return mask;
}
int32_t RuntimeControl_SensorCalibrationEncoderDelta(void) { return sensorCal.encoder_delta; }
int32_t RuntimeControl_SensorCalibrationEncoderForwardDelta(void) { return sensorCal.encoder_forward_delta; }
uint32_t RuntimeControl_SensorCalibrationEncoderDirectionNormalScore(void)
{
    return sensorCal.encoder_direction_normal_score;
}
uint32_t RuntimeControl_SensorCalibrationEncoderDirectionInvertedScore(void)
{
    return sensorCal.encoder_direction_inverted_score;
}
bool RuntimeControl_SensorCalibrationEncoderDirectionProved(void)
{
    return sensorCal.encoder_direction_proved;
}
int16_t RuntimeControl_EncoderAlignmentProbeDelta(bool left)
{
    if (!encoderAlign.active && encoderAlign.motor != (left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT)) return 0;
    return encoderAlign.probe_delta_counts;
}

bool RuntimeControl_EncoderAlignmentDirectionProved(bool left)
{
    return encoderAlign.motor == (left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT) && encoderAlign.direction_proved;
}

bool RuntimeControl_EncoderAlignmentActive(bool left)
{
    if (!encoderAlign.active) return false;
    return encoderAlign.motor == (left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT);
}
bool RuntimeControl_EncoderElectricalReady(bool left)
{
    const MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    const MotorSensorState *st = left ? &motorSensorStateLeft : &motorSensorStateRight;
    if (cfg->sensor_type != MOTOR_SENSOR_ENCODER_AB) return true;
    return cfg->encoder_calibrated != 0U && st->encoder_electrical_aligned;
}
uint16_t RuntimeControl_LinkAgeMs(void)
{
    uint32_t age = RuntimeControl_MonotonicMs() - lastCommandTick;
    return (uint16_t)(age > 65535U ? 65535U : age);
}
uint32_t RuntimeControl_GetReconnectCount(void) { return reconnectCounter; }
uint16_t RuntimeSettings_GetStoredCrc(void) { return eepromStoredCrc; }
uint16_t RuntimeSettings_GetGeneration(void) { return eepromGeneration; }
uint16_t RuntimeSettings_GetVerifyFailures(void) { return eepromVerifyFailures; }
bool RuntimeSettings_Verified(void) { return eepromVerified; }

static uint8_t runtime_core_error_for_report(bool left, uint32_t now)
{
    (void)now;
    return left ? motorOutputLeft.fault_code : motorOutputRight.fault_code;
}

uint8_t RuntimeControl_ErrorLeft(void)
{
    if (faultReportActiveLeft) return faultReportCodeLeft;
    /* Total ISR heartbeat loss adalah system-level fault: jangan sembunyikan. */
    if ((uint32_t)(RuntimeControl_MonotonicMs() - motorIsrHeartbeatLastChangeMs) > MOTOR_ISR_HEARTBEAT_TIMEOUT_MS)
        return ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN;
    if (!armedLeft && !armRequestedLeft && faultReportSuppressLeft) return ESC_MOTOR_ERROR_NONE;
    if ((motorControlIsrOverrunFaultMask & 0x01U) != 0U) return ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN;
    if ((motorControlOvercurrentFaultMask & 0x01U) != 0U) return ESC_MOTOR_ERROR_ABS_OVER_CURRENT;
    const uint32_t now = RuntimeControl_MonotonicMs();
    return effective_error_one(requestedModeLeft, runtime_core_error_for_report(true, now),
                               &sensorHealthLeft, &motorConfigLeft);
}
uint8_t RuntimeControl_ErrorRight(void)
{
    if (faultReportActiveRight) return faultReportCodeRight;
    if ((uint32_t)(RuntimeControl_MonotonicMs() - motorIsrHeartbeatLastChangeMs) > MOTOR_ISR_HEARTBEAT_TIMEOUT_MS)
        return ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN;
    if (!armedRight && !armRequestedRight && faultReportSuppressRight) return ESC_MOTOR_ERROR_NONE;
    if ((motorControlIsrOverrunFaultMask & 0x02U) != 0U) return ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN;
    if ((motorControlOvercurrentFaultMask & 0x02U) != 0U) return ESC_MOTOR_ERROR_ABS_OVER_CURRENT;
    const uint32_t now = RuntimeControl_MonotonicMs();
    return effective_error_one(requestedModeRight, runtime_core_error_for_report(false, now),
                               &sensorHealthRight, &motorConfigRight);
}
bool RuntimeControl_HasBlockingFault(void)
{
    if (motorControlIsrOverrunFaultMask != 0U || motorControlOvercurrentFaultMask != 0U) return true;
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING) return false;
    const uint32_t now = RuntimeControl_MonotonicMs();
    return blocking_error_one(requestedModeLeft, runtime_core_error_for_report(true, now),
                              &sensorHealthLeft, &motorConfigLeft) ||
           blocking_error_one(requestedModeRight, runtime_core_error_for_report(false, now),
                              &sensorHealthRight, &motorConfigRight);
}

bool RuntimeControl_ShouldSoundFaultBuzzer(void)
{
    const uint32_t now = RuntimeControl_MonotonicMs();

    /* Commissioning/alignment memakai buzzer/timing sendiri; jangan wariskan
     * alarm fault dari attempt ARM sebelumnya. */
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active) {
        return false;
    }

    /* fault_report_service() dijalankan pada slow-loop. Guard waktu di sini
     * memastikan buzzer tidak pernah melewati 3 detik walau satu slow tick telat. */
    const bool left_active = faultReportActiveLeft &&
        (int32_t)(faultReportUntilLeft - now) > 0 &&
        (faultReportCodeLeft == ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN ||
         faultReportCodeLeft == ESC_MOTOR_ERROR_ABS_OVER_CURRENT);
    const bool right_active = faultReportActiveRight &&
        (int32_t)(faultReportUntilRight - now) > 0 &&
        (faultReportCodeRight == ESC_MOTOR_ERROR_CONTROL_ISR_OVERRUN ||
         faultReportCodeRight == ESC_MOTOR_ERROR_ABS_OVER_CURRENT);
    /* Sensor-not-calibrated / no-signal is a commissioning/readiness condition.
     * VESC Tool must report it, but simply connecting or changing APP mode must
     * not produce a fault beep. Only hard runtime protection is audible. */
    return left_active || right_active;
}

uint8_t RuntimeControl_HomingStateLeft(void) { return homingRuntimeLeft.state; }
uint8_t RuntimeControl_HomingStateRight(void) { return homingRuntimeRight.state; }
bool RuntimeControl_SteeringReady(bool left)
{
    const SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    const MotorSensorState *sensor = left ? &motorSensorStateLeft : &motorSensorStateRight;
    return cal->calibrated != 0U && cal->homed != 0U && cal->span_ticks != 0 &&
           sensor->encoder_electrical_aligned;
}
uint8_t RuntimeControl_HomingActiveMask(void) { return homingActiveMask; }
int16_t RuntimeControl_HomingPeakCurrentLeft(void) { return homingRuntimeLeft.peak_current_centi_amp; }
int16_t RuntimeControl_HomingPeakCurrentRight(void) { return homingRuntimeRight.peak_current_centi_amp; }

void MotorOdometry_Zero(uint8_t motor_select)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (motor_select == ESC_MOTOR_LEFT || motor_select == ESC_MOTOR_BOTH) {
        MotorSensor_Zero(&motorSensorStateLeft);
        odom_l = 0;
    }
    if (motor_select == ESC_MOTOR_RIGHT || motor_select == ESC_MOTOR_BOTH) {
        MotorSensor_Zero(&motorSensorStateRight);
        odom_r = 0;
    }
    if (primask == 0U) __enable_irq();
}

/* ----------------------------- EEPROM ---------------------------------- */
static uint16_t eeprom_crc_words(const uint16_t *words, uint8_t count)
{
    /* Dipakai hanya untuk kompatibilitas image EEPROM v5/v6 (word 0..62). */
    return EscProtocol_Crc16((const uint8_t *)words, (uint16_t)count * (uint16_t)sizeof(uint16_t));
}

static uint16_t eeprom_crc_image_count(const uint16_t *words, uint8_t total_words)
{
    /* CRC tetap berada di word 63 agar layout telemetry/upgrade lama kompatibel.
     * v7 mencakup 0..69; v8 0..71; v9/v10/v11 0..75 (tanpa word CRC 63). */
    uint16_t packed[NB_OF_VAR - 1U];
    uint8_t out = 0U;
    if (total_words > NB_OF_VAR) total_words = NB_OF_VAR;
    for (uint8_t i = 0U; i < total_words; ++i) {
        if (i == EEPROM_WORD_CRC) continue;
        packed[out++] = words[i];
    }
    return EscProtocol_Crc16((const uint8_t *)packed, (uint16_t)out * (uint16_t)sizeof(uint16_t));
}

static uint16_t eeprom_crc_current_image(const uint16_t *words)
{
    return eeprom_crc_image_count(words, NB_OF_VAR);
}

static void put_i32(uint16_t *words, uint8_t index, int32_t value)
{
    words[index] = (uint16_t)((uint32_t)value & 0xFFFFU);
    words[index + 1U] = (uint16_t)(((uint32_t)value >> 16) & 0xFFFFU);
}
static int32_t get_i32(const uint16_t *words, uint8_t index)
{
    return (int32_t)(((uint32_t)words[index + 1U] << 16) | words[index]);
}
static void put_u32(uint16_t *words, uint8_t index, uint32_t value)
{
    words[index] = (uint16_t)(value & 0xFFFFU);
    words[index + 1U] = (uint16_t)(value >> 16);
}

static uint32_t get_u32(const uint16_t *words, uint8_t index)
{
    return ((uint32_t)words[index + 1U] << 16) | words[index];
}

static void store_hall_lut_words(uint16_t *words, uint8_t lo_index, uint8_t hi_index,
                                 const MotorRuntimeConfig *config)
{
    uint32_t packed = 0U;
    for (uint8_t i = 0U; i < 6U; ++i)
        packed |= ((uint32_t)(config->hall_sequence[i] & 0x07U) << (i * 3U));
    words[lo_index] = (uint16_t)(packed & 0xFFFFU);
    words[hi_index] = (uint16_t)((packed >> 16) & 0x0003U);
    if (config->hall_lut_valid != 0U) words[hi_index] |= 0x8000U;
}

static bool load_hall_lut_words(const uint16_t *words, uint8_t lo_index, uint8_t hi_index,
                                MotorRuntimeConfig *config)
{
    const uint32_t packed = (uint32_t)words[lo_index] |
                            ((uint32_t)(words[hi_index] & 0x0003U) << 16);
    uint8_t sequence[6];
    for (uint8_t i = 0U; i < 6U; ++i)
        sequence[i] = (uint8_t)((packed >> (i * 3U)) & 0x07U);
    const bool valid_flag = (words[hi_index] & 0x8000U) != 0U;
    if (valid_flag && !MotorRuntimeConfig_HallSequenceValid(sequence)) return false;
    memcpy(config->hall_sequence, sequence, 6U);
    config->hall_lut_valid = valid_flag ? 1U : 0U;
    return true;
}


static uint16_t pack_encoder_sequence_word(const MotorRuntimeConfig *config)
{
    uint16_t word = 0U;
    for (uint8_t i = 0U; i < 4U; ++i)
        word |= (uint16_t)((uint16_t)(config->encoder_sequence[i] & 0x03U) << (i * 2U));
    if (config->encoder_sequence_valid != 0U) word |= 0x0100U;
    return word;
}

static bool unpack_encoder_sequence_word(uint16_t word, MotorRuntimeConfig *config)
{
    uint8_t sequence[4];
    for (uint8_t i = 0U; i < 4U; ++i)
        sequence[i] = (uint8_t)((word >> (i * 2U)) & 0x03U);
    if ((word & 0xFE00U) != 0U) return false;
    const bool valid_flag = (word & 0x0100U) != 0U;
    if (valid_flag && !MotorRuntimeConfig_EncoderSequenceValid(sequence)) return false;
    memcpy(config->encoder_sequence, sequence, 4U);
    config->encoder_sequence_valid = valid_flag ? 1U : 0U;
    return true;
}

static void store_foc_words(uint16_t *w, uint8_t base, const mc_configuration *conf)
{
    /* v14: five signed Q16.16 gains occupy the ten tuning words exactly. */
    put_i32(w, base + 0U, conf->foc_current_kp_q16);
    put_i32(w, base + 2U, conf->foc_current_ki_q16);
    put_i32(w, base + 4U, conf->s_pid_kp_q16);
    put_i32(w, base + 6U, conf->s_pid_ki_q16);
    put_i32(w, base + 8U, conf->s_pid_kd_q16);
}

static void load_foc_words(const uint16_t *w, uint8_t base, mc_configuration *conf)
{
    conf->foc_current_kp_q16 = get_i32(w, base + 0U);
    conf->foc_current_ki_q16 = get_i32(w, base + 2U);
    conf->s_pid_kp_q16 = get_i32(w, base + 4U);
    conf->s_pid_ki_q16 = get_i32(w, base + 6U);
    conf->s_pid_kd_q16 = get_i32(w, base + 8U);
}

static void store_pos_words(uint16_t *w, uint8_t base, const PositionPidConfig *cfg)
{
    put_i32(w, base + 0U, cfg->kp_q16);
    put_i32(w, base + 2U, cfg->ki_q16);
    put_i32(w, base + 4U, cfg->kd_q16);
    put_i32(w, base + 6U, cfg->position_min);
    put_i32(w, base + 8U, cfg->position_max);
    w[base + 10U] = cfg->deadband_ticks;
    for (uint8_t i = 11U; i < 16U; ++i) w[base + i] = 0U;
}

static void load_pos_words(const uint16_t *w, uint8_t base, PositionPidConfig *cfg)
{
    cfg->kp_q16 = get_i32(w, base + 0U);
    cfg->ki_q16 = get_i32(w, base + 2U);
    cfg->kd_q16 = get_i32(w, base + 4U);
    cfg->position_min = get_i32(w, base + 6U);
    cfg->position_max = get_i32(w, base + 8U);
    cfg->deadband_ticks = w[base + 10U];
}

static void store_advanced_foc_words(uint16_t *w, uint8_t base, const mc_configuration *conf)
{
    put_u32(w, base + 0U, conf->foc_motor_flux_linkage_uwb);
    put_i32(w, base + 2U, conf->foc_motor_ld_lq_diff_uh);
    w[base + 4U] = (uint16_t)conf->foc_mtpa_mode;
    w[base + 5U] = (uint16_t)conf->foc_fw_current_max;
    w[base + 6U] = conf->foc_fw_duty_start_q15;
    w[base + 7U] = conf->foc_fw_ramp_time_ms;
    w[base + 8U] = conf->foc_fw_q_current_factor_q15;
    w[base + 9U] = conf->foc_fw_backoff_q15;
    w[base + 10U] = conf->s_pid_kd_filter_q15;
    put_i32(w, base + 11U, conf->s_pid_min_erpm);
    put_i32(w, base + 13U, conf->s_pid_ramp_erpms_s);
    put_i32(w, base + 15U, conf->p_pid_kd_proc_q16);
    w[base + 17U] = conf->p_pid_kd_filter_q15;
    put_u32(w, base + 18U, conf->p_pid_gain_dec_ticks);
    w[base + 20U] = conf->s_pid_allow_braking ? 1U : 0U;
}

static bool load_advanced_foc_words(const uint16_t *w, uint8_t base, mc_configuration *conf)
{
    conf->foc_motor_flux_linkage_uwb = get_u32(w, base + 0U);
    conf->foc_motor_ld_lq_diff_uh = get_i32(w, base + 2U);
    if (w[base + 4U] > (uint16_t)MTPA_MODE_IQ_MEASURED) return false;
    conf->foc_mtpa_mode = (mc_mtpa_mode)w[base + 4U];
    if (w[base + 5U] > INT16_MAX || w[base + 6U] > FOC_Q15_ONE ||
        w[base + 8U] > FOC_Q15_ONE || w[base + 9U] > FOC_Q15_ONE ||
        w[base + 10U] > FOC_Q15_ONE || w[base + 17U] > FOC_Q15_ONE ||
        w[base + 20U] > 1U) return false;
    conf->foc_fw_current_max = (int16_t)w[base + 5U];
    conf->foc_fw_duty_start_q15 = w[base + 6U];
    conf->foc_fw_ramp_time_ms = w[base + 7U];
    conf->foc_fw_q_current_factor_q15 = w[base + 8U];
    conf->foc_fw_backoff_q15 = w[base + 9U];
    conf->s_pid_kd_filter_q15 = w[base + 10U];
    conf->s_pid_min_erpm = get_i32(w, base + 11U);
    conf->s_pid_ramp_erpms_s = get_i32(w, base + 13U);
    conf->p_pid_kd_proc_q16 = get_i32(w, base + 15U);
    conf->p_pid_kd_filter_q15 = w[base + 17U];
    conf->p_pid_gain_dec_ticks = get_u32(w, base + 18U);
    conf->s_pid_allow_braking = w[base + 20U] != 0U;
    return true;
}


static bool vesc_app_config_valid(const VescAppConfig *a)
{
    if (a == NULL || a->controller_id > 253U || a->timeout_ms > 600000U) return false;
    if (a->app_to_use > VESC_APP_ADC_UART || !VescApp_AdcControlSupported(a->adc_ctrl_type)) return false;
    if (a->uart_baud < 9600U || a->uart_baud > 921600U) return false;
    if (a->voltage_start_mV > 3300U || a->voltage_end_mV > 3300U ||
        a->voltage_min_mV > 3300U || a->voltage_max_mV > 3300U ||
        a->voltage_center_mV > 3300U || a->voltage2_start_mV > 3300U ||
        a->voltage2_end_mV > 3300U || a->voltage_start_mV >= a->voltage_end_mV ||
        a->voltage2_start_mV >= a->voltage2_end_mV || a->voltage_min_mV > a->voltage_max_mV) return false;
    if (a->use_filter > 1U || a->safe_start > 1U || a->buttons > 1U ||
        a->voltage_inverted > 1U || a->voltage2_inverted > 1U ||
        a->multi_esc > 1U || a->tc > 1U || a->throttle_exp_mode > 2U) return false;
    if (a->update_rate_hz == 0U || a->update_rate_hz > 2000U) return false;
    return true;
}

static void store_vesc_app_words(uint16_t *w, const VescAppConfig *a)
{
    w[EEPROM_VESC_APP_PACKED] = (uint16_t)a->controller_id |
        ((uint16_t)(a->app_to_use & 0x07U) << 8) | ((uint16_t)(a->adc_ctrl_type & 0x0FU) << 11);
    put_u32(w, EEPROM_VESC_APP_TIMEOUT_LO, a->timeout_ms);
    w[EEPROM_VESC_APP_BRAKE_CA] = (uint16_t)a->timeout_brake_cA;
    put_u32(w, EEPROM_VESC_APP_BAUD_LO, a->uart_baud);
    w[EEPROM_VESC_APP_FLAGS] = (uint16_t)((a->use_filter ? 1U : 0U) |
        (a->safe_start ? 2U : 0U) | (a->buttons ? 4U : 0U) |
        (a->voltage_inverted ? 8U : 0U) | (a->voltage2_inverted ? 16U : 0U) |
        (a->multi_esc ? 32U : 0U));
    w[EEPROM_VESC_APP_HYST_MV] = a->adc_hyst_mV;
    w[EEPROM_VESC_APP_VSTART_MV] = a->voltage_start_mV;
    w[EEPROM_VESC_APP_VEND_MV] = a->voltage_end_mV;
    w[EEPROM_VESC_APP_VMIN_MV] = a->voltage_min_mV;
    w[EEPROM_VESC_APP_VMAX_MV] = a->voltage_max_mV;
    w[EEPROM_VESC_APP_VCENTER_MV] = a->voltage_center_mV;
    w[EEPROM_VESC_APP_V2START_MV] = a->voltage2_start_mV;
    w[EEPROM_VESC_APP_V2END_MV] = a->voltage2_end_mV;
    w[EEPROM_VESC_APP_EXP] = 0U;
    w[EEPROM_VESC_APP_EXP_BRAKE] = 0U;
    w[EEPROM_VESC_APP_RAMP_POS_MS] = a->ramp_time_pos_ms;
    w[EEPROM_VESC_APP_RAMP_NEG_MS] = a->ramp_time_neg_ms;
    w[EEPROM_VESC_APP_TC_DIFF] = 0U;
    w[EEPROM_VESC_APP_UPDATE_HZ] = a->update_rate_hz;
}

static bool load_vesc_app_words(const uint16_t *w, VescAppConfig *a)
{
    const uint16_t p = w[EEPROM_VESC_APP_PACKED];
    a->controller_id = (uint8_t)(p & 0xFFU);
    a->app_to_use = (uint8_t)((p >> 8) & 0x07U);
    a->adc_ctrl_type = (uint8_t)((p >> 11) & 0x0FU);
    a->timeout_ms = get_u32(w, EEPROM_VESC_APP_TIMEOUT_LO);
    a->timeout_brake_cA = (int16_t)w[EEPROM_VESC_APP_BRAKE_CA];
    a->uart_baud = get_u32(w, EEPROM_VESC_APP_BAUD_LO);
    const uint16_t f = w[EEPROM_VESC_APP_FLAGS];
    a->use_filter = (f & 1U) != 0U; a->safe_start = (f & 2U) != 0U;
    a->buttons = 0U; a->voltage_inverted = (f & 8U) != 0U;
    a->voltage2_inverted = (f & 16U) != 0U; a->multi_esc = (f & 32U) != 0U;
    a->tc = 0U; a->throttle_exp_mode = 0U;
    a->adc_hyst_mV = w[EEPROM_VESC_APP_HYST_MV]; a->voltage_start_mV = w[EEPROM_VESC_APP_VSTART_MV];
    a->voltage_end_mV = w[EEPROM_VESC_APP_VEND_MV]; a->voltage_min_mV = w[EEPROM_VESC_APP_VMIN_MV];
    a->voltage_max_mV = w[EEPROM_VESC_APP_VMAX_MV]; a->voltage_center_mV = w[EEPROM_VESC_APP_VCENTER_MV];
    a->voltage2_start_mV = w[EEPROM_VESC_APP_V2START_MV]; a->voltage2_end_mV = w[EEPROM_VESC_APP_V2END_MV];
    a->throttle_exp_milli = 0; a->throttle_exp_brake_milli = 0;
    a->ramp_time_pos_ms = w[EEPROM_VESC_APP_RAMP_POS_MS]; a->ramp_time_neg_ms = w[EEPROM_VESC_APP_RAMP_NEG_MS];
    a->tc_max_diff_milli = 0U; a->update_rate_hz = w[EEPROM_VESC_APP_UPDATE_HZ];
    a->uart_baud = USART3_BAUD;
    return vesc_app_config_valid(a);
}

bool RuntimeSettings_Save(void)
{
    /* Flash erase/program menghentikan fetch kode dari flash. Save hanya boleh
     * dilakukan saat seluruh output sudah idle. Auto Detect memanggil fungsi ini
     * hanya setelah open-loop dihentikan dan hasil kalibrasi sudah tervalidasi;
     * command GUI Save tetap memakai jalur yang sama. */
    if (sensorCal.state == ESC_SENSOR_CAL_RUNNING || encoderAlign.active ||
        homingActiveMask != 0U || runtimeMotorEnableMask != 0U ||
        sensorCalibrationOpenLoopMask != 0U || armedLeft || armedRight ||
        armRequestedLeft || armRequestedRight) return false;

    uint16_t w[NB_OF_VAR];
    uint16_t verify[NB_OF_VAR];
    memset(w, 0, sizeof(w));
    memset(verify, 0, sizeof(verify));

    w[EEPROM_WORD_KEY] = FLASH_WRITE_KEY;
    w[EEPROM_WORD_VERSION] = EEPROM_CONFIG_VERSION;
    w[EEPROM_WORD_MAX_CURRENT] = (uint16_t)motorConfLeft.l_current_max;
    w[EEPROM_WORD_MAX_SPEED] = (uint16_t)motorConfLeft.l_max_speed_rpm_q4;
    positionPidConfigLeft.kp_q16 = motorConfLeft.p_pid_kp_q16;
    positionPidConfigLeft.ki_q16 = motorConfLeft.p_pid_ki_q16;
    positionPidConfigLeft.kd_q16 = motorConfLeft.p_pid_kd_q16;
    positionPidConfigRight.kp_q16 = motorConfRight.p_pid_kp_q16;
    positionPidConfigRight.ki_q16 = motorConfRight.p_pid_ki_q16;
    positionPidConfigRight.kd_q16 = motorConfRight.p_pid_kd_q16;
    store_foc_words(w, EEPROM_LEFT_FOC_BASE, &motorConfLeft);
    store_foc_words(w, EEPROM_RIGHT_FOC_BASE, &motorConfRight);
    w[EEPROM_LEFT_MOTOR_CONFIG] = MotorRuntimeConfig_PackWord(&motorConfigLeft);
    w[EEPROM_LEFT_ENCODER_CPR] = motorConfigLeft.encoder_cpr;
    w[EEPROM_RIGHT_MOTOR_CONFIG] = MotorRuntimeConfig_PackWord(&motorConfigRight);
    w[EEPROM_RIGHT_ENCODER_CPR] = motorConfigRight.encoder_cpr;
    store_pos_words(w, EEPROM_LEFT_POS_BASE, &positionPidConfigLeft);
    store_pos_words(w, EEPROM_RIGHT_POS_BASE, &positionPidConfigRight);
    w[EEPROM_WORD_TELEM_RATE] = telemetryRateHz;
    /* Halaman CONFIG/MOTOR_CONFIG bersifat one-shot dan tidak boleh tersimpan
     * sebagai halaman boot. Simpan halaman telemetry normal terakhir saja. */
    const uint8_t persistent_telemetry_page =
        (telemetryPage <= ESC_TELEM_RAW) ? telemetryPage : ESC_TELEM_BASIC;
    w[EEPROM_WORD_TELEM_PAGE] = persistent_telemetry_page;
    w[EEPROM_LEFT_HOMING_CURRENT] = homingConfigLeft.current_threshold_centi_amp;
    w[EEPROM_RIGHT_HOMING_CURRENT] = homingConfigRight.current_threshold_centi_amp;
    w[EEPROM_LEFT_HOMING_COMMAND] = (uint16_t)homingConfigLeft.search_command;
    w[EEPROM_RIGHT_HOMING_COMMAND] = (uint16_t)homingConfigRight.search_command;
    w[EEPROM_HOMING_TIMEOUT] = homingTimeoutMs;
    w[EEPROM_HOMING_DEBOUNCE] = homingDebounceMs;
    w[EEPROM_LEFT_BOOT_HOMING] = homingOnBootLeft ? 1U : 0U;
    w[EEPROM_RIGHT_BOOT_HOMING] = homingOnBootRight ? 1U : 0U;
    store_hall_lut_words(w, EEPROM_LEFT_HALL_LUT_LO, EEPROM_LEFT_HALL_LUT_HI, &motorConfigLeft);
    store_hall_lut_words(w, EEPROM_RIGHT_HALL_LUT_LO, EEPROM_RIGHT_HALL_LUT_HI, &motorConfigRight);
    w[EEPROM_LEFT_ENCODER_SEQUENCE] = pack_encoder_sequence_word(&motorConfigLeft);
    w[EEPROM_RIGHT_ENCODER_SEQUENCE] = pack_encoder_sequence_word(&motorConfigRight);
    store_advanced_foc_words(w, EEPROM_LEFT_ADV_BASE, &motorConfLeft);
    store_advanced_foc_words(w, EEPROM_RIGHT_ADV_BASE, &motorConfRight);
    store_vesc_app_words(w, &vescAppConfig);
    w[EEPROM_LEFT_POLE_PAIRS] = motorConfLeft.foc_motor_pole_pairs;
    w[EEPROM_RIGHT_POLE_PAIRS] = motorConfRight.foc_motor_pole_pairs;
    w[EEPROM_VESC_APP_RESERVED] = 0U;
    w[EEPROM_LEFT_ENCODER_RATIO] = motorConfigLeft.encoder_ratio;
    w[EEPROM_RIGHT_ENCODER_RATIO] = motorConfigRight.encoder_ratio;
    put_i32(w, EEPROM_LEFT_STEER_ZERO_LO, steeringCalibrationLeft.right_zero_ticks);
    put_i32(w, EEPROM_LEFT_STEER_SPAN_LO, steeringCalibrationLeft.span_ticks);
    w[EEPROM_LEFT_STEER_CAL] = steeringCalibrationLeft.calibrated ? 1U : 0U;
    put_i32(w, EEPROM_RIGHT_STEER_ZERO_LO, steeringCalibrationRight.right_zero_ticks);
    put_i32(w, EEPROM_RIGHT_STEER_SPAN_LO, steeringCalibrationRight.span_ticks);
    w[EEPROM_RIGHT_STEER_CAL] = steeringCalibrationRight.calibrated ? 1U : 0U;
    w[EEPROM_LEFT_GEAR_RATIO_MILLI] = motorConfLeft.si_gear_ratio_milli;
    w[EEPROM_RIGHT_GEAR_RATIO_MILLI] = motorConfRight.si_gear_ratio_milli;
    w[EEPROM_LEFT_CURRENT_MAX] = (uint16_t)motorConfLeft.l_current_max;
    w[EEPROM_LEFT_CURRENT_MIN] = (uint16_t)motorConfLeft.l_current_min;
    w[EEPROM_RIGHT_CURRENT_MAX] = (uint16_t)motorConfRight.l_current_max;
    w[EEPROM_RIGHT_CURRENT_MIN] = (uint16_t)motorConfRight.l_current_min;
    w[EEPROM_WORD_GENERATION] = (uint16_t)(eepromGeneration + 1U);
    if (w[EEPROM_WORD_GENERATION] == 0U) w[EEPROM_WORD_GENERATION] = 1U;
    w[EEPROM_WORD_CRC] = eeprom_crc_current_image(w);

    eepromVerified = false;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
    RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
    HAL_FLASH_Unlock();
    bool write_ok = true;
    for (uint8_t i = 0; i < NB_OF_VAR; ++i) {
        /* Kurangi wear flash: tulis hanya bila nilai terakhir berbeda / belum ada. */
        uint16_t current = 0U;
        const uint16_t read_status = EE_ReadVariable(VirtAddVarTab[i], &current);
        if (read_status != 0U || current != w[i]) {
            if (EE_WriteVariable(VirtAddVarTab[i], w[i]) != HAL_OK) {
                write_ok = false;
                break;
            }
        }
    }
    HAL_FLASH_Lock();
    DMA1->IFCR = DMA_IFCR_CGIF1;
    if (primask == 0U) __enable_irq();
    if (!write_ok) {
        ++eepromVerifyFailures;
        return false;
    }

    /* Read-back verification: SAVE baru dianggap sukses bila seluruh word identik. */
    for (uint8_t i = 0; i < NB_OF_VAR; ++i) {
        if (EE_ReadVariable(VirtAddVarTab[i], &verify[i]) != 0U) {
            ++eepromVerifyFailures;
            return false;
        }
    }
    if (memcmp(w, verify, sizeof(w)) != 0 ||
        verify[EEPROM_WORD_CRC] != eeprom_crc_current_image(verify)) {
        ++eepromVerifyFailures;
        return false;
    }

    eepromStoredCrc = verify[EEPROM_WORD_CRC];
    eepromGeneration = verify[EEPROM_WORD_GENERATION];
    eepromVerified = true;
    settingsDirty = false;
    return true;
}

static bool persistent_config_valid(const mc_configuration *left,
                                    const mc_configuration *right,
                                    const PositionPidConfig *pos_left,
                                    const PositionPidConfig *pos_right,
                                    const MotorRuntimeConfig *motor_left,
                                    const MotorRuntimeConfig *motor_right,
                                    uint16_t max_current_word,
                                    uint16_t max_speed_word,
                                    uint16_t telemetry_rate,
                                    uint8_t telemetry_page_value,
                                    const HomingMotorConfig *home_left,
                                    const HomingMotorConfig *home_right,
                                    uint16_t home_timeout,
                                    uint16_t home_debounce)
{
    if (max_current_word == 0U || max_current_word > INT16_MAX ||
        max_speed_word == 0U || max_speed_word > INT16_MAX) return false;

    const mc_configuration *conf[2] = {left, right};
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (conf[i]->l_current_max <= 0 || conf[i]->l_current_max > 12000 ||
            conf[i]->l_current_min >= 0 || conf[i]->l_current_min < -12000) return false;
        if (conf[i]->l_max_speed_rpm_q4 <= 0 || conf[i]->l_max_speed_rpm_q4 != (int16_t)max_speed_word) return false;
        if (conf[i]->l_max_voltage <= 0) return false;
        if (conf[i]->foc_current_kp_q16 < 0 || conf[i]->foc_current_ki_q16 < 0) return false;
        if (conf[i]->s_pid_kp_q16 < 0 || conf[i]->s_pid_ki_q16 < 0 || conf[i]->s_pid_kd_q16 < 0) return false;
        if (conf[i]->p_pid_kd_proc_q16 < 0) return false;
        if (conf[i]->foc_mtpa_mode > MTPA_MODE_IQ_MEASURED) return false;
        if (conf[i]->foc_mtpa_mode != MTPA_MODE_OFF &&
            (conf[i]->foc_motor_flux_linkage_uwb == 0U || conf[i]->foc_motor_ld_lq_diff_uh == 0)) return false;
        if (conf[i]->foc_motor_flux_linkage_uwb > 10000000U ||
            conf[i]->foc_motor_ld_lq_diff_uh < -1000000 || conf[i]->foc_motor_ld_lq_diff_uh > 1000000) return false;
        if (conf[i]->foc_fw_current_max < 0 || conf[i]->foc_fw_current_max > conf[i]->l_current_max ||
            conf[i]->foc_fw_duty_start_q15 > FOC_Q15_ONE ||
            conf[i]->foc_fw_q_current_factor_q15 > FOC_Q15_ONE || conf[i]->foc_fw_backoff_q15 > FOC_Q15_ONE ||
            conf[i]->s_pid_kd_filter_q15 > FOC_Q15_ONE || conf[i]->p_pid_kd_filter_q15 > FOC_Q15_ONE ||
            conf[i]->s_pid_min_erpm < 0 || conf[i]->s_pid_ramp_erpms_s < 0) return false;
        if (conf[i]->foc_motor_pole_pairs == 0U) return false;
    }

    const PositionPidConfig *pos[2] = {pos_left, pos_right};
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (pos[i]->kp_q16 < 0 || pos[i]->ki_q16 < 0 || pos[i]->kd_q16 < 0) return false;
        if (pos[i]->position_min > pos[i]->position_max) return false;
        if (pos[i]->deadband_ticks > 1000U) return false;
    }

    if (!MotorRuntimeConfig_IsValid(motor_left) || !MotorRuntimeConfig_IsValid(motor_right)) return false;
    if (telemetry_rate < 1U || telemetry_rate > TELEMETRY_RATE_MAX) return false;
    if (telemetry_page_value > ESC_TELEM_RAW) return false;
    if (!HomingConfig_IsValid(home_left) || !HomingConfig_IsValid(home_right)) return false;
    if (!HomingTiming_IsValid(home_timeout, home_debounce)) return false;
    return true;
}


bool RuntimeSettings_Load(void)
{
    uint16_t w[NB_OF_VAR];
    memset(w, 0, sizeof(w));
    eepromVerified = false;

    /* Semua versi lama memiliki base image word 0..63. */
    for (uint8_t i = 0U; i <= EEPROM_WORD_CRC; ++i) {
        if (EE_ReadVariable(VirtAddVarTab[i], &w[i]) != 0U) return false;
    }

    const uint16_t version = w[EEPROM_WORD_VERSION];
    const bool version_v5 = (version == EEPROM_CONFIG_VERSION_V5);
    const bool version_v6 = (version == EEPROM_CONFIG_VERSION_V6);
    const bool version_v7 = (version == EEPROM_CONFIG_VERSION_V7);
    const bool version_v8 = (version == EEPROM_CONFIG_VERSION_V8);
    const bool version_v9 = (version == EEPROM_CONFIG_VERSION_V9);
    const bool version_v10 = (version == EEPROM_CONFIG_VERSION_V10);
    const bool version_v11 = (version == EEPROM_CONFIG_VERSION_V11);
    const bool version_v12 = (version == EEPROM_CONFIG_VERSION_V12);
    const bool version_v13 = (version == EEPROM_CONFIG_VERSION_V13);
    const bool version_v18 = (version == EEPROM_CONFIG_VERSION_V18);
    const bool version_v17 = (version == EEPROM_CONFIG_VERSION_V17);
    const bool version_v16 = (version == EEPROM_CONFIG_VERSION_V16);
    const bool version_v15 = (version == EEPROM_CONFIG_VERSION_V15);
    const bool version_v14 = (version == EEPROM_CONFIG_VERSION_V14);
    const bool current_version = (version == EEPROM_CONFIG_VERSION);
    if (w[EEPROM_WORD_KEY] != FLASH_WRITE_KEY ||
        (!version_v5 && !version_v6 && !version_v7 && !version_v8 && !version_v9 && !version_v10 && !version_v11 && !version_v12 && !version_v13 && !version_v14 && !version_v15 && !version_v16 && !version_v17 && !version_v18 && !current_version)) return false;

    /* v7: homing 64..69; v8: auto-home 70..71; v9-v11: Hall LUT 72..75;
     * v12-v14: persistent Encoder 4-state sequence 76..77. v15 adds MTPA/FW and advanced outer-loop configuration 78..119;
     * v16 adds persistent VESC Tool APP/UART/ADC configuration 120..143; v17 adds encoder-ratio and steering calibration 144..155. */
    if (version_v7 || version_v8 || version_v9 || version_v10 || version_v11 || version_v12 || version_v13 || version_v14 || version_v15 || version_v16 || current_version) {
        uint8_t last_word = EEPROM_HOMING_DEBOUNCE;
        if (version_v8) last_word = EEPROM_RIGHT_BOOT_HOMING;
        if (version_v9 || version_v10 || version_v11) last_word = EEPROM_RIGHT_HALL_LUT_HI;
        if (version_v12 || version_v13 || version_v14) last_word = EEPROM_RIGHT_ENCODER_SEQUENCE;
        if (version_v15) last_word = EEPROM_RIGHT_ADV_BASE + EEPROM_ADV_WORDS_PER_MOTOR - 1U;
        if (version_v16) last_word = EEPROM_VESC_APP_RESERVED;
        if (current_version) last_word = NB_OF_VAR - 1U;
        for (uint8_t i = EEPROM_LEFT_HOMING_CURRENT; i <= last_word; ++i) {
            if (EE_ReadVariable(VirtAddVarTab[i], &w[i]) != 0U) return false;
        }
        const uint8_t crc_words = (uint8_t)(last_word + 1U);
        if (w[EEPROM_WORD_CRC] != eeprom_crc_image_count(w, crc_words)) return false;
    } else {
        /* v5/v6: CRC legacy hanya word 0..62. */
        if (w[EEPROM_WORD_CRC] != eeprom_crc_words(w, EEPROM_WORD_CRC)) return false;
    }

    /* Transactional load: decode ke salinan temporary dahulu. Jika satu field
     * tidak valid, parameter aktif di RAM tidak berubah sama sekali. */
    mc_configuration left = motorConfLeft;
    mc_configuration right = motorConfRight;
    PositionPidConfig pos_left = positionPidConfigLeft;
    PositionPidConfig pos_right = positionPidConfigRight;
    MotorRuntimeConfig motor_left = motorConfigLeft;
    MotorRuntimeConfig motor_right = motorConfigRight;
    HomingMotorConfig home_left = homingConfigLeft;
    HomingMotorConfig home_right = homingConfigRight;
    uint16_t home_timeout = homingTimeoutMs;
    uint16_t home_debounce = homingDebounceMs;
    bool boot_home_left = false;
    bool boot_home_right = false;
    VescAppConfig app_cfg = vescAppConfig;
    SteeringCalibration steer_left = {0};
    SteeringCalibration steer_right = {0};

    left.l_current_max = right.l_current_max = (int16_t)w[EEPROM_WORD_MAX_CURRENT];
    left.l_current_min = right.l_current_min = (int16_t)-left.l_current_max;
    left.l_max_speed_rpm_q4 = right.l_max_speed_rpm_q4 = (int16_t)w[EEPROM_WORD_MAX_SPEED];

    if (version_v15 || version_v16 || current_version) {
        load_foc_words(w, EEPROM_LEFT_FOC_BASE, &left);
        load_foc_words(w, EEPROM_RIGHT_FOC_BASE, &right);
        load_pos_words(w, EEPROM_LEFT_POS_BASE, &pos_left);
        load_pos_words(w, EEPROM_RIGHT_POS_BASE, &pos_right);
        if (!load_advanced_foc_words(w, EEPROM_LEFT_ADV_BASE, &left) ||
            !load_advanced_foc_words(w, EEPROM_RIGHT_ADV_BASE, &right)) return false;
    } else {
        /* v5..v14 tuning semantics differ from the VESC-7 PID/MTPA/FW port.
         * Keep safe VESC-style defaults, but migrate the non-controller limits. */
        pos_left.position_min = get_i32(w, EEPROM_LEFT_POS_BASE + 12U);
        pos_left.position_max = get_i32(w, EEPROM_LEFT_POS_BASE + 14U);
        pos_right.position_min = get_i32(w, EEPROM_RIGHT_POS_BASE + 12U);
        pos_right.position_max = get_i32(w, EEPROM_RIGHT_POS_BASE + 14U);
        pos_left.deadband_ticks = w[EEPROM_LEFT_FOC_BASE + 9U];
        pos_right.deadband_ticks = w[EEPROM_RIGHT_FOC_BASE + 9U];
        if (pos_left.position_min > pos_left.position_max) {
            pos_left.position_min = POSITION_MIN_DEFAULT;
            pos_left.position_max = POSITION_MAX_DEFAULT;
        }
        if (pos_right.position_min > pos_right.position_max) {
            pos_right.position_min = POSITION_MIN_DEFAULT;
            pos_right.position_max = POSITION_MAX_DEFAULT;
        }
    }

    if (version_v5) {
        /* v5: word +10/+11 belum didefinisikan sebagai motor/sensor config. */
        MotorRuntimeConfig_SetDefaults(&motor_left);
        MotorRuntimeConfig_SetDefaults(&motor_right);
    } else if (!MotorRuntimeConfig_UnpackWord(w[EEPROM_LEFT_MOTOR_CONFIG], w[EEPROM_LEFT_ENCODER_CPR], &motor_left) ||
               !MotorRuntimeConfig_UnpackWord(w[EEPROM_RIGHT_MOTOR_CONFIG], w[EEPROM_RIGHT_ENCODER_CPR], &motor_right)) {
        return false;
    }
    /* Hardware contract: RIGHT is Hall-only. Legacy images that selected an
     * encoder on the right are migrated safely instead of reconfiguring PC10..12. */
    motor_right.sensor_type = MOTOR_SENSOR_HALL_UVW;
    motor_right.encoder_calibrated = 0U;
    motor_right.encoder_sequence_valid = 0U;

    if (version_v7 || version_v8 || version_v9 || version_v10 || version_v11 || version_v12 || version_v13 || version_v14 || version_v15 || version_v16 || current_version) {
        home_left.current_threshold_centi_amp = w[EEPROM_LEFT_HOMING_CURRENT];
        home_right.current_threshold_centi_amp = w[EEPROM_RIGHT_HOMING_CURRENT];
        home_left.search_command = (int16_t)w[EEPROM_LEFT_HOMING_COMMAND];
        home_right.search_command = (int16_t)w[EEPROM_RIGHT_HOMING_COMMAND];
        home_timeout = w[EEPROM_HOMING_TIMEOUT];
        home_debounce = w[EEPROM_HOMING_DEBOUNCE];
    }
    if (version_v8 || version_v9 || version_v10 || version_v11 || version_v12 || version_v13 || version_v14 || version_v15 || version_v16 || current_version) {
        if (w[EEPROM_LEFT_BOOT_HOMING] > 1U || w[EEPROM_RIGHT_BOOT_HOMING] > 1U) return false;
        boot_home_left = (w[EEPROM_LEFT_BOOT_HOMING] != 0U);
        boot_home_right = (w[EEPROM_RIGHT_BOOT_HOMING] != 0U);
    }
    if (version_v9 || version_v10 || version_v11 || version_v12 || version_v13 || version_v14 || version_v15 || version_v16 || current_version) {
        if (!load_hall_lut_words(w, EEPROM_LEFT_HALL_LUT_LO, EEPROM_LEFT_HALL_LUT_HI, &motor_left) ||
            !load_hall_lut_words(w, EEPROM_RIGHT_HALL_LUT_LO, EEPROM_RIGHT_HALL_LUT_HI, &motor_right)) return false;
        if (version_v9) {
            /* v9 belum menyimpan proof-of-calibration encoder. */
            motor_left.encoder_calibrated = 0U;
            motor_right.encoder_calibrated = 0U;
        }
        if (version_v10) {
            /* v10 menyimpan offset dengan semantik mekanik + kompensasi internal
             * -30deg. v11+ memakai offset listrik eksplisit dan direct theta_e.
             * Pertahankan CPR/arah/LUT, tetapi paksa Auto Detect sekali agar tidak
             * pernah menjalankan FOC dengan referensi sudut legacy yang salah. */
            if (motor_left.sensor_type == MOTOR_SENSOR_ENCODER_AB) {
                motor_left.encoder_offset_deg = 0U;
                motor_left.encoder_calibrated = 0U;
            }
            if (motor_right.sensor_type == MOTOR_SENSOR_ENCODER_AB) {
                motor_right.encoder_offset_deg = 0U;
                motor_right.encoder_calibrated = 0U;
            }
        }
    }

    if (version_v12 || version_v13 || version_v14 || version_v15 || version_v16 || current_version) {
        if (!unpack_encoder_sequence_word(w[EEPROM_LEFT_ENCODER_SEQUENCE], &motor_left) ||
            !unpack_encoder_sequence_word(w[EEPROM_RIGHT_ENCODER_SEQUENCE], &motor_right)) return false;
        if (version_v12) {
            /* v12 tidak dapat membedakan hasil Manual Detect dari Auto Detect.
             * Karena manual lama dapat menjadikan arah arbitrary sebagai arah FOC
             * dan memicu +SP -> -speed / 0x04, proof lama harus dianggap tidak
             * aman satu kali. Sequence/CPR tetap dipertahankan untuk diagnosis. */
            motor_left.hall_calibrated = 0U;
            motor_right.hall_calibrated = 0U;
            motor_left.encoder_calibrated = 0U;
            motor_right.encoder_calibrated = 0U;
        }
    } else {
        /* Legacy image belum memiliki proof urutan quadrature persistent. */
        motor_left.encoder_sequence_valid = 0U;
        motor_right.encoder_sequence_valid = 0U;
    }

    if ((version_v16 || current_version) && !load_vesc_app_words(w, &app_cfg)) return false;
    if (version_v16 || current_version) {
        /* Backward compatible with early v16 images: reserved words were zero. */
        if (w[EEPROM_LEFT_POLE_PAIRS] >= 1U && w[EEPROM_LEFT_POLE_PAIRS] <= 60U)
            left.foc_motor_pole_pairs = (uint8_t)w[EEPROM_LEFT_POLE_PAIRS];
        if (w[EEPROM_RIGHT_POLE_PAIRS] >= 1U && w[EEPROM_RIGHT_POLE_PAIRS] <= 60U)
            right.foc_motor_pole_pairs = (uint8_t)w[EEPROM_RIGHT_POLE_PAIRS];
    }
    if (version_v18 || current_version) {
        if (w[EEPROM_LEFT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_LEFT_GEAR_RATIO_MILLI] > 60000U ||
            w[EEPROM_RIGHT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_RIGHT_GEAR_RATIO_MILLI] > 60000U) return false;
        left.si_gear_ratio_milli = w[EEPROM_LEFT_GEAR_RATIO_MILLI];
        right.si_gear_ratio_milli = w[EEPROM_RIGHT_GEAR_RATIO_MILLI];
        if (w[EEPROM_LEFT_ENCODER_RATIO] <= 60U) motor_left.encoder_ratio = (uint8_t)w[EEPROM_LEFT_ENCODER_RATIO];
        else return false;
        if (w[EEPROM_RIGHT_ENCODER_RATIO] <= 60U) motor_right.encoder_ratio = (uint8_t)w[EEPROM_RIGHT_ENCODER_RATIO];
        else return false;
        if (w[EEPROM_LEFT_STEER_CAL] > 1U || w[EEPROM_RIGHT_STEER_CAL] > 1U) return false;
        steer_left.right_zero_ticks = get_i32(w, EEPROM_LEFT_STEER_ZERO_LO);
        steer_left.span_ticks = get_i32(w, EEPROM_LEFT_STEER_SPAN_LO);
        steer_left.calibrated = (uint8_t)w[EEPROM_LEFT_STEER_CAL];
        steer_left.homed = 0U;
        steer_right.right_zero_ticks = get_i32(w, EEPROM_RIGHT_STEER_ZERO_LO);
        steer_right.span_ticks = get_i32(w, EEPROM_RIGHT_STEER_SPAN_LO);
        steer_right.calibrated = (uint8_t)w[EEPROM_RIGHT_STEER_CAL];
        steer_right.homed = 0U;
        if ((steer_left.calibrated && steer_left.span_ticks == 0) ||
            (steer_right.calibrated && steer_right.span_ticks == 0)) return false;
    } else {
        /* v16 did not persist encoder ratio independently. The old behavior was
         * ratio == physical pole-pairs, so preserve that semantics explicitly. */
        motor_left.encoder_ratio = 0U;
        motor_right.encoder_ratio = 0U;
        steer_left.calibrated = steer_right.calibrated = 0U;
        steer_left.homed = steer_right.homed = 0U;
        if (version_v16) {
            /* Migrate old unsafe homing defaults only when they are untouched. */
            if (home_left.current_threshold_centi_amp == 500U && home_left.search_command == -120) {
                home_left.current_threshold_centi_amp = 120U; home_left.search_command = -100;
            }
            if (home_right.current_threshold_centi_amp == 500U && home_right.search_command == -120) {
                home_right.current_threshold_centi_amp = 120U; home_right.search_command = -100;
            }
        }
    }

    left.p_pid_kp_q16 = pos_left.kp_q16;
    left.p_pid_ki_q16 = pos_left.ki_q16;
    left.p_pid_kd_q16 = pos_left.kd_q16;
    left.p_pid_pos_min = pos_left.position_min;
    left.p_pid_pos_max = pos_left.position_max;
    left.p_pid_deadband_ticks = pos_left.deadband_ticks;
    right.p_pid_kp_q16 = pos_right.kp_q16;
    right.p_pid_ki_q16 = pos_right.ki_q16;
    right.p_pid_kd_q16 = pos_right.kd_q16;
    right.p_pid_pos_min = pos_right.position_min;
    right.p_pid_pos_max = pos_right.position_max;
    right.p_pid_deadband_ticks = pos_right.deadband_ticks;
    if (current_version) {
        left.l_current_max = (int16_t)w[EEPROM_LEFT_CURRENT_MAX];
        left.l_current_min = (int16_t)w[EEPROM_LEFT_CURRENT_MIN];
        right.l_current_max = (int16_t)w[EEPROM_RIGHT_CURRENT_MAX];
        right.l_current_min = (int16_t)w[EEPROM_RIGHT_CURRENT_MIN];
        if (left.l_current_max <= 0 || left.l_current_max > 12000 ||
            left.l_current_min >= 0 || left.l_current_min < -12000 ||
            right.l_current_max <= 0 || right.l_current_max > 12000 ||
            right.l_current_min >= 0 || right.l_current_min < -12000) return false;
    }

    mc_foc_conf_prepare(&left);
    mc_foc_conf_prepare(&right);

    const uint16_t loaded_rate = w[EEPROM_WORD_TELEM_RATE];
    const uint8_t loaded_page = (uint8_t)w[EEPROM_WORD_TELEM_PAGE];
    if (!persistent_config_valid(&left, &right, &pos_left, &pos_right, &motor_left, &motor_right,
                                 w[EEPROM_WORD_MAX_CURRENT], w[EEPROM_WORD_MAX_SPEED],
                                 loaded_rate, loaded_page, &home_left, &home_right, home_timeout, home_debounce)) {
        return false;
    }

    /* Commit atomik: ISR 16 kHz membaca parameter/sensor/state yang sama. LOAD
     * dari GUI dilakukan saat DISARM, tetapi ISR tetap hidup, jadi jangan pernah
     * biarkan ia melihat separuh struct lama dan separuh struct baru. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    motorConfLeft = left;
    motorConfRight = right;
    positionPidConfigLeft = pos_left;
    positionPidConfigRight = pos_right;
    motorConfigLeft = motor_left;
    motorConfigRight = motor_right;
    homingConfigLeft = home_left;
    homingConfigRight = home_right;
    homingTimeoutMs = home_timeout;
    homingDebounceMs = home_debounce;
    homingOnBootLeft = boot_home_left;
    homingOnBootRight = boot_home_right;
    vescAppConfig = app_cfg;
    steeringCalibrationLeft = steer_left;
    steeringCalibrationRight = steer_right;
    apply_sensor_backend_to_foc(&motorConfigLeft, &motorConfLeft);
    apply_sensor_backend_to_foc(&motorConfigRight, &motorConfRight);
    mc_foc_init(&motorLeft, &motorConfLeft);
    mc_foc_init(&motorRight, &motorConfRight);
    
    
    encoderAlignedLeft = false;
    encoderAlignedRight = false;
    MotorSensor_Reset(&motorSensorStateLeft);
    MotorSensor_Reset(&motorSensorStateRight);
    MotorSensor_PrepareRuntime(&motorConfigLeft, &motorSensorStateLeft, motorConfLeft.foc_motor_pole_pairs);
    MotorSensor_PrepareRuntime(&motorConfigRight, &motorSensorStateRight, motorConfRight.foc_motor_pole_pairs);
    odom_l = odom_r = 0;
    if (primask == 0U) __enable_irq();
    sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);
    sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);
    mc_foc_reset_outer_loops(&motorLeft);
    mc_foc_reset_outer_loops(&motorRight);
    telemetryRateHz = loaded_rate;
    telemetryPage = loaded_page;

    eepromStoredCrc = w[EEPROM_WORD_CRC];
    eepromGeneration = w[EEPROM_WORD_GENERATION];
    eepromVerified = true;
    settingsDirty = !current_version;
    return true;
}

/* ---------------------------- TELEMETRY -------------------------------- */
#define TELEM_ENABLED(bit) ((telemetryMask & (1UL << (bit))) != 0UL)



static void fill_basic(uint8_t payload[46])
{
    EscTelemetryBasic t;
    memset(&t, 0, sizeof(t));
    /* Posisi dan speed telemetry selalu memakai koordinat mekanik host yang
     * sama dengan setpoint. Inversi hardware per motor dinormalisasi di sini. */
    if (TELEM_ENABLED(0))  t.position_left = apply_motor_direction_i32(&motorConfigLeft, odom_l);
    if (TELEM_ENABLED(1))  t.position_right = apply_motor_direction_i32(&motorConfigRight, odom_r);
    if (TELEM_ENABLED(2))  t.setpoint_left = runtimeSetpointLeft;
    if (TELEM_ENABLED(3))  t.setpoint_right = runtimeSetpointRight;
    /* BASIC adalah telemetry SENSOR. Jangan ambil speed dari output FOC karena
     * core sengaja men-zero-kan feedback yang belum control-proof. Observation
     * sensor harus tetap terlihat saat DISARM/manual commissioning. */
    if (TELEM_ENABLED(4))  t.speed_left = apply_motor_direction_i16(&motorConfigLeft,
        (int16_t)(motorSensorSampleLeft.mechanical_speed_q4 >> 4));
    if (TELEM_ENABLED(5))  t.speed_right = apply_motor_direction_i16(&motorConfigRight,
        (int16_t)(motorSensorSampleRight.mechanical_speed_q4 >> 4));
    if (TELEM_ENABLED(6))  t.battery_centi_volt = batVoltageCalib;
    if (TELEM_ENABLED(7))  t.temperature_deci_c = board_temp_deci_c;
    if (TELEM_ENABLED(8))  t.dc_current_left_centi_amp = left_dc_curr;
    if (TELEM_ENABLED(9))  t.dc_current_right_centi_amp = right_dc_curr;
    if (TELEM_ENABLED(10)) t.command_left = runtimeCommandLeft;
    if (TELEM_ENABLED(11)) t.command_right = runtimeCommandRight;
    if (TELEM_ENABLED(12)) t.link_age_ms = RuntimeControl_LinkAgeMs();
    if (TELEM_ENABLED(13)) t.telemetry_rate_hz = telemetryRateHz;
    t.arm_state_mask = (armedLeft ? 0x01U : 0U) | (armedRight ? 0x02U : 0U);
    t.arm_request_mask = (armRequestedLeft ? 0x01U : 0U) | (armRequestedRight ? 0x02U : 0U);
    t.output_enable_mask = runtimeMotorEnableMask & 0x03U;
    t.arm_reject_left = armRejectLeft;
    t.arm_reject_right = armRejectRight;
    /* Tidak ada cycle-counter di ISR 16 kHz (lihat motor.c). Field ini sekarang
     * melaporkan seberapa dekat ke fail-safe MOTOR_ISR_OVERRUN_LIMIT, bukan lagi
     * persentase budget CPU literal -- tetap 0 selama sehat. */
    t.motor_isr_peak_load_pct = MotorControl_GetOverrunStreakPct();
    t.motor_isr_overrun_count = (motorControlIsrOverrunCount > 255U) ? 255U : (uint8_t)motorControlIsrOverrunCount;
    t.boot_cpu_fault_code = bootCpuFaultCode;
    t.boot_reset_flags = bootResetFlags;
    t.uart_recovery_flags = (uint8_t)((EscProtocol_GetTxDmaErrorCount() != 0U ? 0x01U : 0U) |
                                      (EscProtocol_GetRxDmaRecoveryCount() != 0U ? 0x02U : 0U));
    memcpy(payload, &t, sizeof(t));
}

static void fill_foc(uint8_t payload[46])
{
    EscTelemetryFoc t;
    memset(&t, 0, sizeof(t));
    if (TELEM_ENABLED(0))  t.id_left = motorOutputLeft.id;
    if (TELEM_ENABLED(1))  t.iq_left = motorOutputLeft.iq;
    if (TELEM_ENABLED(2))  t.id_right = motorOutputRight.id;
    if (TELEM_ENABLED(3))  t.iq_right = motorOutputRight.iq;
    if (TELEM_ENABLED(4))  t.phase_a_left = curL_phaA;
    if (TELEM_ENABLED(5))  t.phase_b_left = curL_phaB;
    if (TELEM_ENABLED(6))  t.phase_b_right = curR_phaB;
    if (TELEM_ENABLED(7))  t.phase_c_right = curR_phaC;
    if (TELEM_ENABLED(8))  t.dc_link_left = curL_DC;
    if (TELEM_ENABLED(9))  t.dc_link_right = curR_DC;
    if (TELEM_ENABLED(10)) t.duty_u_left = motorOutputLeft.duty_a;
    if (TELEM_ENABLED(11)) t.duty_v_left = motorOutputLeft.duty_b;
    if (TELEM_ENABLED(12)) t.duty_w_left = motorOutputLeft.duty_c;
    if (TELEM_ENABLED(13)) t.duty_u_right = motorOutputRight.duty_a;
    if (TELEM_ENABLED(14)) t.duty_v_right = motorOutputRight.duty_b;
    if (TELEM_ENABLED(15)) t.duty_w_right = motorOutputRight.duty_c;
    if (TELEM_ENABLED(16)) t.electrical_angle_left = motorOutputLeft.electrical_angle_deg;
    if (TELEM_ENABLED(17)) t.electrical_angle_right = motorOutputRight.electrical_angle_deg;
    /* FOC page menampilkan measured sensor speed yang sama dengan BASIC. Core
     * speed tetap tersedia terpisah di RAW untuk diagnosis controller. */
    if (TELEM_ENABLED(18)) t.speed_left = apply_motor_direction_i16(&motorConfigLeft,
        (int16_t)(motorSensorSampleLeft.mechanical_speed_q4 >> 4));
    if (TELEM_ENABLED(19)) t.speed_right = apply_motor_direction_i16(&motorConfigRight,
        (int16_t)(motorSensorSampleRight.mechanical_speed_q4 >> 4));
    if (TELEM_ENABLED(20)) {
        /* hall_bits juga membawa mode sensor agar GUI dapat menampilkan channel
         * hall_kiri/hall_kanan secara kontekstual tanpa menambah frame:
         * bit15 LEFT encoder, bit14 RIGHT encoder; bit5..3 LEFT raw field;
         * bit2..0 RIGHT raw field. Hall memakai UVW 3-bit yang masuk FOC,
         * Encoder memakai raw AB (00/01/10/11) pada 2 bit bawah field. */
        const bool left_encoder = (motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB);
        const bool right_encoder = (motorConfigRight.sensor_type == MOTOR_SENSOR_ENCODER_AB);
        const uint8_t left_sensor_bits = left_encoder
            ? (uint8_t)(motorSensorSampleLeft.encoder_ab & 0x03U)
            : (uint8_t)(motorSensorSampleLeft.hall_encoding & 0x07U);
        const uint8_t right_sensor_bits = right_encoder
            ? (uint8_t)(motorSensorSampleRight.encoder_ab & 0x03U)
            : (uint8_t)(motorSensorSampleRight.hall_encoding & 0x07U);
        t.hall_bits = (uint16_t)((left_encoder ? 0x8000U : 0U) |
                                 (right_encoder ? 0x4000U : 0U) |
                                 ((uint16_t)left_sensor_bits << 3) |
                                 (uint16_t)right_sensor_bits);
    }
    if (TELEM_ENABLED(21)) t.dc_current_left_centi_amp = left_dc_curr;
    if (TELEM_ENABLED(22)) t.dc_current_right_centi_amp = right_dc_curr;
    memcpy(payload, &t, sizeof(t));
}

static void pid_values_for_motor(bool left, uint8_t loop,
                                 int32_t *setpoint, int32_t *measured,
                                 int32_t *error, int16_t *p, int16_t *i,
                                 int16_t *d, int16_t *output, bool *antiwindup)
{
    motor_all_state_t *motor = left ? &motorLeft : &motorRight;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    mc_foc_output_t *out = left ? &motorOutputLeft : &motorOutputRight;
    const MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    const bool is_armed = left ? armedLeft : armedRight;
    const uint8_t mode = left ? requestedModeLeft : requestedModeRight;
    const int32_t host_sp = left ? runtimeSetpointLeft : runtimeSetpointRight;

    *setpoint = 0; *measured = 0; *error = 0;
    *p = 0; *i = 0; *d = 0; *output = 0; *antiwindup = false;

    if (loop == ESC_PID_POSITION) {
        *setpoint = host_sp;
        *measured = apply_motor_direction_i32(cfg, motor->m_position_ticks);
        if (is_armed && mode == ESC_MODE_POS) {
            *error = apply_motor_direction_i32(cfg, motor->m_pos_pid.previous_error);
            *p = apply_motor_direction_i16(cfg, motor->m_pos_pid.last_p);
            *i = apply_motor_direction_i16(cfg, motor->m_pos_pid.last_i);
            *d = apply_motor_direction_i16(cfg, motor->m_pos_pid.last_d);
            *output = apply_motor_direction_i16(cfg, motor->m_pos_pid.last_output);
            *antiwindup = motor->m_pos_pid.anti_windup;
        }
        return;
    }

    if (loop == ESC_PID_SPEED) {
        *measured = apply_motor_direction_i16(cfg, out->speed_rpm);
        if (is_armed && mode == ESC_MODE_SPD) {
            const int32_t mech_rpm = conf->foc_motor_pole_pairs > 0U
                ? motor->m_speed_pid_set_rpm / conf->foc_motor_pole_pairs : 0;
            *setpoint = apply_motor_direction_i32(cfg, mech_rpm);
            *error = apply_motor_direction_i32(cfg, motor->m_speed_pid.previous_error);
            *p = apply_motor_direction_i16(cfg, motor->m_speed_pid.last_p);
            *i = apply_motor_direction_i16(cfg, motor->m_speed_pid.last_i);
            *d = apply_motor_direction_i16(cfg, motor->m_speed_pid.last_d);
            *output = apply_motor_direction_i16(cfg, motor->m_speed_pid.last_output);
            *antiwindup = motor->m_speed_pid.anti_windup;
        }
        return;
    }

    if (loop == ESC_PID_CURRENT_D) {
        *setpoint = motor->m_motor_state.id_target;
        *measured = out->id;
        *error = motor->m_motor_state.pid_d.previous_error;
        *p = motor->m_motor_state.pid_d.last_p;
        *i = motor->m_motor_state.pid_d.last_i;
        *d = 0;
        *output = motor->m_motor_state.pid_d.last_output;
        *antiwindup = motor->m_motor_state.pid_d.anti_windup;
        return;
    }

    /* ESC_PID_TORQUE_Q: target and feedback are Iq in host motor direction. */
    *setpoint = apply_motor_direction_i16(cfg, motor->m_motor_state.iq_target);
    *measured = apply_motor_direction_i16(cfg, out->iq);
    *error = apply_motor_direction_i32(cfg, motor->m_motor_state.pid_q.previous_error);
    *p = apply_motor_direction_i16(cfg, motor->m_motor_state.pid_q.last_p);
    *i = apply_motor_direction_i16(cfg, motor->m_motor_state.pid_q.last_i);
    *d = 0;
    *output = apply_motor_direction_i16(cfg, motor->m_motor_state.pid_q.last_output);
    *antiwindup = motor->m_motor_state.pid_q.anti_windup;
}


static void fill_pid(uint8_t payload[46])
{
    EscTelemetryPid t;
    memset(&t, 0, sizeof(t));
    bool aw_l=false, aw_r=false;
    int32_t sp_l=0,sp_r=0,meas_l=0,meas_r=0,err_l=0,err_r=0;
    int16_t p_l=0,i_l=0,d_l=0,out_l=0,p_r=0,i_r=0,d_r=0,out_r=0;
    t.loop=telemetryPidLoop;
    pid_values_for_motor(true,telemetryPidLoop,&sp_l,&meas_l,&err_l,&p_l,&i_l,&d_l,&out_l,&aw_l);
    pid_values_for_motor(false,telemetryPidLoop,&sp_r,&meas_r,&err_r,&p_r,&i_r,&d_r,&out_r,&aw_r);
    if (TELEM_ENABLED(0))  t.setpoint_left=sp_l;
    if (TELEM_ENABLED(1))  t.setpoint_right=sp_r;
    if (TELEM_ENABLED(2))  t.measured_left=meas_l;
    if (TELEM_ENABLED(3))  t.measured_right=meas_r;
    if (TELEM_ENABLED(4))  t.error_left=err_l;
    if (TELEM_ENABLED(5))  t.error_right=err_r;
    if (TELEM_ENABLED(6))  t.p_left=p_l;
    if (TELEM_ENABLED(7))  t.i_left=i_l;
    if (TELEM_ENABLED(8))  t.d_left=d_l;
    if (TELEM_ENABLED(9))  t.output_left=out_l;
    if (TELEM_ENABLED(10)) t.p_right=p_r;
    if (TELEM_ENABLED(11)) t.i_right=i_r;
    if (TELEM_ENABLED(12)) t.d_right=d_r;
    if (TELEM_ENABLED(13)) t.output_right=out_r;
    if (TELEM_ENABLED(14)) t.antiwindup_flags=(aw_l?1U:0U)|(aw_r?2U:0U);
    memcpy(payload,&t,sizeof(t));
}

static void fill_raw(uint8_t payload[46])
{
    EscTelemetryRaw t;
    memset(&t,0,sizeof(t));
    if (TELEM_ENABLED(0))  t.adc_phase_a_left=adc_buffer.rlA;
    if (TELEM_ENABLED(1))  t.adc_phase_b_left=adc_buffer.rlB;
    if (TELEM_ENABLED(2))  t.adc_phase_b_right=adc_buffer.rrB;
    if (TELEM_ENABLED(3))  t.adc_phase_c_right=adc_buffer.rrC;
    if (TELEM_ENABLED(4))  t.adc_dc_left=adc_buffer.dcl;
    if (TELEM_ENABLED(5))  t.adc_dc_right=adc_buffer.dcr;
    if (TELEM_ENABLED(6))  t.adc_battery=adc_buffer.batt1;
    if (TELEM_ENABLED(7))  t.adc_temperature=adc_buffer.temp;
    if (TELEM_ENABLED(8))  t.hall_left = (motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB)
        ? motorSensorSampleLeft.encoder_ab : motorSensorSampleLeft.raw_hall_encoding;
    if (TELEM_ENABLED(9))  t.hall_right = (motorConfigRight.sensor_type == MOTOR_SENSOR_ENCODER_AB)
        ? motorSensorSampleRight.encoder_ab : motorSensorSampleRight.raw_hall_encoding;
    if (TELEM_ENABLED(10)) t.pwm_period=2000U;
    if (TELEM_ENABLED(11)) t.main_loop_counter=main_loop_counter;
    if (TELEM_ENABLED(12)) t.valid_frames=EscProtocol_GetValidFrameCount();
    if (TELEM_ENABLED(13)) t.bad_frames=EscProtocol_GetBadFrameCount();
    if (TELEM_ENABLED(14)) t.reconnect_counter=reconnectCounter;
    t.core_speed_left = motorOutputLeft.speed_rpm;
    t.core_speed_right = motorOutputRight.speed_rpm;
    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    memcpy(payload,&t,sizeof(t));
}

static void fill_config(uint8_t payload[46])
{
    EscTelemetryConfig t;
    memset(&t, 0, sizeof(t));
    const bool left = configMotor != ESC_MOTOR_RIGHT;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    PositionPidConfig *pos = left ? &positionPidConfigLeft : &positionPidConfigRight;

    t.motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    t.loop = telemetryPidLoop;
    t.position_min = pos->position_min;
    t.position_max = pos->position_max;
    t.position_deadband_ticks = pos->deadband_ticks;

    switch (telemetryPidLoop) {
        case ESC_PID_POSITION:
            t.kp_q16 = conf->p_pid_kp_q16;
            t.ki_q16 = conf->p_pid_ki_q16;
            t.kd_q16 = conf->p_pid_kd_q16;
            t.i_limit_q16 = 65536; /* normalized integrator clamp +/-1.0 */
            t.output_min = -conf->l_current_max;
            t.output_max = conf->l_current_max;
            break;
        case ESC_PID_SPEED:
            t.kp_q16 = conf->s_pid_kp_q16;
            t.ki_q16 = conf->s_pid_ki_q16;
            t.kd_q16 = conf->s_pid_kd_q16;
            t.i_limit_q16 = 65536;
            t.output_min = -conf->l_current_max;
            t.output_max = conf->l_current_max;
            break;
        case ESC_PID_CURRENT_D:
        case ESC_PID_TORQUE_Q:
        default:
            t.kp_q16 = conf->foc_current_kp_q16;
            t.ki_q16 = conf->foc_current_ki_q16;
            t.kd_q16 = 0;
            t.output_min = -conf->l_max_voltage;
            t.output_max = conf->l_max_voltage;
            break;
    }

    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    t.eeprom_verified = eepromVerified ? 1U : 0U;
    t.settings_dirty = settingsDirty ? 1U : 0U;
    memcpy(payload, &t, sizeof(t));
}



static uint16_t q15_to_permille(uint16_t value_q15)
{
    return (uint16_t)(((uint32_t)value_q15 * 1000U + 16383U) / 32767U);
}

static void fill_foc_advanced_config(uint8_t payload[46])
{
    EscTelemetryFocAdvanced t;
    memset(&t, 0, sizeof(t));
    const bool left = configMotor != ESC_MOTOR_RIGHT;
    const mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;
    const motor_all_state_t *motor = left ? &motorLeft : &motorRight;

    t.motor = left ? ESC_MOTOR_LEFT : ESC_MOTOR_RIGHT;
    t.mtpa_mode = (uint8_t)conf->foc_mtpa_mode;
    t.flags = conf->s_pid_allow_braking ? 0x0001U : 0U;
    t.flux_linkage_uwb = conf->foc_motor_flux_linkage_uwb;
    t.ld_lq_diff_uh = conf->foc_motor_ld_lq_diff_uh;
    t.fw_current_centi_amp = (uint16_t)current_internal_to_centiamp(conf, conf->foc_fw_current_max);
    t.fw_duty_start_permille = q15_to_permille(conf->foc_fw_duty_start_q15);
    t.fw_ramp_time_ms = conf->foc_fw_ramp_time_ms;
    t.fw_q_current_factor_permille = q15_to_permille(conf->foc_fw_q_current_factor_q15);
    t.fw_backoff_permille = q15_to_permille(conf->foc_fw_backoff_q15);
    t.speed_kd_filter_permille = q15_to_permille(conf->s_pid_kd_filter_q15);
    t.speed_min_erpm = conf->s_pid_min_erpm;
    t.speed_ramp_erpms_s = conf->s_pid_ramp_erpms_s;
    t.pos_kd_proc_q16 = conf->p_pid_kd_proc_q16;
    t.pos_kd_filter_permille = q15_to_permille(conf->p_pid_kd_filter_q15);
    t.pos_gain_dec_ticks = conf->p_pid_gain_dec_ticks;
    t.actual_fw_current_centi_amp = current_internal_to_centiamp(conf, motor->m_i_fw_set);
    t.actual_mtpa_id_centi_amp = current_internal_to_centiamp(conf, motor->m_motor_state.id_target_mtpa);
    memcpy(payload, &t, sizeof(t));
}

static void fill_motor_config(uint8_t payload[46])
{
    EscTelemetryMotorConfig t;
    memset(&t, 0, sizeof(t));
    t.left_config_word = MotorRuntimeConfig_PackWord(&motorConfigLeft);
    t.right_config_word = MotorRuntimeConfig_PackWord(&motorConfigRight);
    t.left_encoder_cpr = motorConfigLeft.encoder_cpr;
    t.right_encoder_cpr = motorConfigRight.encoder_cpr;
    t.left_sensor_position = odom_l;
    t.right_sensor_position = odom_r;
    t.left_sensor_encoding = motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB
        ? motorSensorSampleLeft.encoder_ab : motorSensorSampleLeft.raw_hall_encoding;
    t.right_sensor_encoding = motorConfigRight.sensor_type == MOTOR_SENSOR_ENCODER_AB
        ? motorSensorSampleRight.encoder_ab : motorSensorSampleRight.raw_hall_encoding;
    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    t.eeprom_verified = eepromVerified ? 1U : 0U;
    t.settings_dirty = settingsDirty ? 1U : 0U;
    memcpy(payload, &t, sizeof(t));
}

static void fill_homing_config(uint8_t payload[46])
{
    EscTelemetryHomingConfig t;
    memset(&t, 0, sizeof(t));
    t.left_current_threshold_centi_amp = homingConfigLeft.current_threshold_centi_amp;
    t.right_current_threshold_centi_amp = homingConfigRight.current_threshold_centi_amp;
    t.left_search_command = homingConfigLeft.search_command;
    t.right_search_command = homingConfigRight.search_command;
    t.timeout_ms = homingTimeoutMs;
    t.debounce_ms = homingDebounceMs;
    t.left_state = homingRuntimeLeft.state;
    t.right_state = homingRuntimeRight.state;
    t.active_mask = homingActiveMask;
    t.left_current_centi_amp = left_dc_curr;
    t.right_current_centi_amp = right_dc_curr;
    t.left_peak_current_centi_amp = homingRuntimeLeft.peak_current_centi_amp;
    t.right_peak_current_centi_amp = homingRuntimeRight.peak_current_centi_amp;
    t.left_position = apply_motor_direction_i32(&motorConfigLeft, odom_l);
    t.right_position = apply_motor_direction_i32(&motorConfigRight, odom_r);
    t.eeprom_crc = eepromStoredCrc;
    t.eeprom_generation = eepromGeneration;
    t.eeprom_verify_failures = eepromVerifyFailures;
    t.eeprom_verified = eepromVerified ? 1U : 0U;
    t.settings_dirty = settingsDirty ? 1U : 0U;
    t.auto_home_boot_left = homingOnBootLeft ? 1U : 0U;
    t.auto_home_boot_right = homingOnBootRight ? 1U : 0U;
    memcpy(payload, &t, sizeof(t));
}

/* Telemetry commissioning sensor. Page ini juga membawa LUT/parameter sensor
 * tersimpan sehingga GUI dapat menampilkan hasil kalibrasi walau proses sudah
 * selesai atau sesudah reconnect. */
static void fill_sensor_cal(uint8_t payload[46])
{
    EscTelemetrySensorCalibration t;
    memset(&t, 0, sizeof(t));

    t.state = sensorCal.state;
    t.method = sensorCal.method;
    t.motor = sensorCal.motor;
    t.sensor_type = sensorCal.sensor_type;

    uint32_t elapsed = 0U;
    if (sensorCal.start_tick != 0U) elapsed = RuntimeControl_MonotonicMs() - sensorCal.start_tick;
    if (elapsed > UINT16_MAX) elapsed = UINT16_MAX;
    t.elapsed_ms = (uint16_t)elapsed;
    t.samples = sensorCal.samples;
    t.invalid_samples = sensorCal.invalid_samples;

    uint32_t progress = 0U;
    if (sensorCal.state == ESC_SENSOR_CAL_SUCCESS ||
        sensorCal.state == ESC_SENSOR_CAL_FAILED_SEQUENCE ||
        sensorCal.state == ESC_SENSOR_CAL_FAILED_TIMEOUT ||
        sensorCal.state == ESC_SENSOR_CAL_OVERCURRENT ||
        sensorCal.state == ESC_SENSOR_CAL_ABORTED ||
        sensorCal.state == ESC_SENSOR_CAL_PERSIST_FAILED) {
        progress = (sensorCal.state == ESC_SENSOR_CAL_SUCCESS) ? 1000U : 0U;
    } else if (sensorCal.state == ESC_SENSOR_CAL_RUNNING) {
        if (sensorCal.method == ESC_SENSOR_CAL_METHOD_MANUAL) {
            const uint32_t duration = sensorCal.manual_duration_ms ? sensorCal.manual_duration_ms
                                                                   : SENSOR_CAL_MANUAL_DEFAULT_MS;
            progress = (elapsed >= duration) ? 1000U : ((elapsed * 1000U) / duration);
        } else if (elapsed < SENSOR_CAL_AUTO_ALIGN_MS) {
            /* Alignment adalah 10% awal agar progress tidak terlihat macet. */
            progress = (elapsed * 100U) / SENSOR_CAL_AUTO_ALIGN_MS;
        } else if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
            const uint8_t target_cycles = sensor_cal_target_cycles(
                sensorCal.motor, MOTOR_SENSOR_HALL_UVW);
            progress = 100U + ((uint32_t)sensorCal.completed_cycles * 900U / target_cycles);
        } else {
            const uint8_t pp = sensor_cal_params(sensorCal.motor)->foc_motor_pole_pairs
                ? sensor_cal_params(sensorCal.motor)->foc_motor_pole_pairs : 1U;
            progress = 100U + ((uint32_t)sensorCal.completed_cycles * 900U / pp);
        }
    }
    if (progress > 1000U) progress = 1000U;
    t.progress_permille = (uint16_t)progress;

    memcpy(t.hall_sequence_left, motorConfigLeft.hall_sequence, sizeof(t.hall_sequence_left));
    memcpy(t.hall_sequence_right, motorConfigRight.hall_sequence, sizeof(t.hall_sequence_right));
    memcpy(t.encoder_sequence_left, motorConfigLeft.encoder_sequence, sizeof(t.encoder_sequence_left));
    memcpy(t.encoder_sequence_right, motorConfigRight.encoder_sequence, sizeof(t.encoder_sequence_right));
    t.hall_lut_valid_left = motorConfigLeft.hall_lut_valid ? 1U : 0U;
    t.hall_lut_valid_right = motorConfigRight.hall_lut_valid ? 1U : 0U;
    t.encoder_sequence_valid_left = motorConfigLeft.encoder_sequence_valid ? 1U : 0U;
    t.encoder_sequence_valid_right = motorConfigRight.encoder_sequence_valid ? 1U : 0U;
    t.encoder_cpr_left = motorConfigLeft.encoder_cpr;
    t.encoder_cpr_right = motorConfigRight.encoder_cpr;
    t.encoder_offset_left_deg = motorConfigLeft.encoder_offset_deg;
    t.encoder_offset_right_deg = motorConfigRight.encoder_offset_deg;
    const bool show_observed = sensorCalTelemetryCandidate &&
        (sensorCal.state == ESC_SENSOR_CAL_RUNNING ||
         (sensorCal.state == ESC_SENSOR_CAL_SUCCESS &&
          sensorCal.method == ESC_SENSOR_CAL_METHOD_MANUAL) ||
         sensorCal.state == ESC_SENSOR_CAL_FAILED_SEQUENCE ||
         sensorCal.state == ESC_SENSOR_CAL_FAILED_TIMEOUT ||
         sensorCal.state == ESC_SENSOR_CAL_OVERCURRENT ||
         sensorCal.state == ESC_SENSOR_CAL_ABORTED ||
         sensorCal.state == ESC_SENSOR_CAL_PERSIST_FAILED);
    if (show_observed && sensorCal.motor == ESC_MOTOR_LEFT) {
        if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
            memset(t.hall_sequence_left, 0xFF, sizeof(t.hall_sequence_left));
            memcpy(t.hall_sequence_left, sensorCal.observed_hall_sequence,
                   sensorCal.observed_hall_count);
            t.hall_lut_valid_left = 0U;
        } else {
            memset(t.encoder_sequence_left, 0xFF, sizeof(t.encoder_sequence_left));
            memcpy(t.encoder_sequence_left, sensorCal.observed_encoder_sequence,
                   sensorCal.observed_encoder_count);
            t.encoder_sequence_valid_left = 0U;
        }
    } else if (show_observed && sensorCal.motor == ESC_MOTOR_RIGHT) {
        if (sensorCal.sensor_type == MOTOR_SENSOR_HALL_UVW) {
            memset(t.hall_sequence_right, 0xFF, sizeof(t.hall_sequence_right));
            memcpy(t.hall_sequence_right, sensorCal.observed_hall_sequence,
                   sensorCal.observed_hall_count);
            t.hall_lut_valid_right = 0U;
        } else {
            memset(t.encoder_sequence_right, 0xFF, sizeof(t.encoder_sequence_right));
            memcpy(t.encoder_sequence_right, sensorCal.observed_encoder_sequence,
                   sensorCal.observed_encoder_count);
            t.encoder_sequence_valid_right = 0U;
        }
    }
    t.result_code = sensorCal.result_code;
    t.calibration_flags = (motorConfigLeft.encoder_calibrated ? 0x01U : 0U) |
                          (motorConfigRight.encoder_calibrated ? 0x02U : 0U) |
                          (motorConfigLeft.sensor_inverted ? 0x04U : 0U) |
                          (motorConfigRight.sensor_inverted ? 0x08U : 0U) |
                          (motorConfigLeft.hall_calibrated ? 0x10U : 0U) |
                          (motorConfigRight.hall_calibrated ? 0x20U : 0U) |
                          (show_observed ? 0x40U : 0U);
    memcpy(payload, &t, sizeof(t));
}

void RuntimeControl_ServiceTelemetry(void)
{
    const uint32_t now = RuntimeControl_MonotonicMs();
    const uint32_t period = 1000U / (telemetryRateHz ? telemetryRateHz : 1U);
    if ((now - lastTelemetryTick) < period) return;

    /* Jangan habiskan CPU membangun CRC berkali-kali saat DMA frame sebelumnya
     * masih BUSY. lastTelemetryTick/sequence hanya maju sesudah TX benar-benar
     * diterima HAL, sehingga frame tidak hilang diam-diam saat UART sibuk. */
    if (!EscProtocol_TxReady()) return;
    const uint16_t next_sequence = (uint16_t)(feedbackSequence + 1U);

    EscFeedbackFrame f;
    memset(&f,0,sizeof(f));
    f.start=ESC_FRAME_START; f.version=ESC_PROTOCOL_VERSION; f.type=ESC_MSG_TELEMETRY;
    const bool send_arm_snapshot = armStatusSnapshotPending && !telemetryLastWasOneShot;
    const bool send_config_one_shot = !send_arm_snapshot && configOneShotPending && !telemetryLastWasOneShot;
    const bool send_one_shot = send_arm_snapshot || send_config_one_shot;
    const uint8_t page_to_send = send_arm_snapshot ? ESC_TELEM_BASIC :
        (send_config_one_shot ? telemetryOneShotPage : telemetryPage);
    f.sequence=next_sequence; f.page=page_to_send; f.uptime_ms=now;
    f.mode_left=requestedModeLeft; f.mode_right=requestedModeRight;
    f.error_left=RuntimeControl_ErrorLeft(); f.error_right=RuntimeControl_ErrorRight();
    if (linkActive) f.status|=ESC_STATUS_LINK_OK;
    if (armedLeft || armedRight) f.status|=ESC_STATUS_ARMED;
    if (eepromOk) f.status|=ESC_STATUS_EEPROM_OK;
    if (eepromVerified) f.status|=ESC_STATUS_EEPROM_VERIFIED;
    if (settingsDirty) f.status|=ESC_STATUS_SETTINGS_DIRTY;
    if (homingActiveMask != 0U) f.status|=ESC_STATUS_HOMING_ACTIVE;
    if (homingRuntimeLeft.state == ESC_HOMING_HOMED || homingRuntimeRight.state == ESC_HOMING_HOMED)
        f.status|=ESC_STATUS_HOMING_COMPLETE;
    if (!linkActive) f.status|=ESC_STATUS_TIMEOUT;

    switch (page_to_send) {
        case ESC_TELEM_FOC: fill_foc(f.payload); break;
        case ESC_TELEM_PID: fill_pid(f.payload); break;
        case ESC_TELEM_RAW: fill_raw(f.payload); break;
        case ESC_TELEM_CONFIG: fill_config(f.payload); break;
        case ESC_TELEM_MOTOR_CONFIG: fill_motor_config(f.payload); break;
        case ESC_TELEM_HOMING_CONFIG: fill_homing_config(f.payload); break;
        case ESC_TELEM_SENSOR_CAL: fill_sensor_cal(f.payload); break;
        case ESC_TELEM_FOC_ADV_CONFIG: fill_foc_advanced_config(f.payload); break;
        default: fill_basic(f.payload); break;
    }
    f.checksum=EscProtocol_Crc16((const uint8_t*)&f,sizeof(f)-sizeof(f.checksum));
    if (!EscProtocol_SendFeedback(&f)) return;

    feedbackSequence = next_sequence;
    lastTelemetryTick = now;
    if (send_one_shot) {
        if (send_arm_snapshot) armStatusSnapshotPending = false;
        else configOneShotPending = false;
        telemetryLastWasOneShot = true;
    } else {
        /* Jika request baru masuk segera setelah one-shot, pending dibiarkan.
         * Frame berikutnya baru boleh one-shot lagi setelah stream normal ini. */
        telemetryLastWasOneShot = false;
    }
}

#undef TELEM_ENABLED
