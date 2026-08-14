/*
 * VESC V22 cooperative background services (NO RTOS)
 * ==================================================
 * Semantic replacements for the upstream VESC background threads:
 *
 * adc_thread            -> service_app_adc()
 * packet_process_thread -> service_packet_process()
 * blocking_thread       -> protocol/runtime async commissioning state machines
 * timer_thread          -> service_control_timer()
 * sample_send_thread    -> VescProtocol_ServiceBudget() rotor/sample stream
 * fault_stop_thread     -> RuntimeControl_UpdateSlow() + service_status()
 * stat_thread           -> service_statistics()
 * pid_thread            -> RuntimeControl_UpdateSlow() FOC speed/position outer loop
 * rpm_thread (BLDC)     -> NOT duplicated; FOC RPM uses the same PID owner above
 * periodic_thread       -> service_periodic()
 * led_thread            -> service_status()
 *
 * Fast FOC remains in the ADC/PWM ISR and is intentionally not called here.
 */
#include "vesc_services.h"
#include "vesc_protocol.h"
#include "vesc_app.h"
#include "runtime_control.h"
#include "motor_current_cal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "stm32f1xx_hal.h"
#include <limits.h>
#include <stdbool.h>
#include <string.h>

#define VESC_RX_BUDGET_PER_PASS       160U
#define VESC_CONTROL_PERIOD_MS        DELAY_IN_MAIN_LOOP
#define VESC_CONTROL_DT_MAX_MS        50U
#define VESC_STATUS_PERIOD_MS         20U
#define VESC_STAT_PERIOD_MS           1000U
#define VESC_STARTUP_MELODY_MS        900U
#define VESC_STARTUP_NOTE_MS          100U
#define VESC_NORMAL_FLASH_PERIOD_MS   1000U
#define VESC_NORMAL_FLASH_ON_MS       100U
#define VESC_FAULT_FLASH_ON_MS        100U
#define VESC_FAULT_FLASH_OFF_MS       100U
#define VESC_FAULT_FLASH_GAP_MS       800U

extern volatile adc_buf_t adc_buffer;
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern int16_t batVoltage;
extern volatile uint32_t main_loop_counter;

extern volatile uint8_t runtimeBuzzerReason;
extern volatile uint8_t runtimeBuzzerLastReason;
extern volatile uint16_t runtimeBuzzerEventCount;
extern uint8_t backwardDrive;
extern int16_t batVoltageCalib;
extern int16_t board_temp_deci_c;
extern int16_t left_dc_curr;
extern int16_t right_dc_curr;
extern int16_t dc_curr;

static VescServiceStats s_stats;
static uint32_t s_last_control_ms;
static uint32_t s_last_app_ms;
static uint32_t s_last_status_ms;
static uint32_t s_last_stat_ms;
static uint32_t s_boot_ms;
static uint32_t s_battery_low_since_ms;
static uint32_t s_status_pattern_epoch_ms;
static uint8_t s_last_status_code;
static bool s_startup_melody_active;
static int32_t s_board_temp_adc_fix;
static int16_t s_board_temp_adc_filt;

static uint32_t clamp_app_period_ms(void)
{
    uint32_t hz = vescAppConfig.update_rate_hz;
    if (hz == 0U) hz = 1U;
    if (hz > 500U) hz = 500U; /* physical ADC source limit */

    uint32_t period = 1000U / hz;
    if (period == 0U) period = 1U;
    return period;
}

static void service_packet_process(void)
{
    /* Bounded semantic replacement for packet_process_thread. */
    VescProtocol_ServiceBudget(VESC_RX_BUDGET_PER_PASS);
}

static void service_app_adc(uint32_t now)
{
    const uint32_t period = clamp_app_period_ms();
    uint32_t dt = now - s_last_app_ms;
    if (dt < period) return;

    if (dt > VESC_CONTROL_DT_MAX_MS) dt = VESC_CONTROL_DT_MAX_MS;
    s_last_app_ms = now;
    VescApp_Update(dt);

    if (s_stats.app_updates != UINT32_MAX) ++s_stats.app_updates;
}

static void update_engineering_values(void)
{
    left_dc_curr = MotorControl_GetDcInputCentiAmp(true);
    right_dc_curr = MotorControl_GetDcInputCentiAmp(false);

    int32_t sum = (int32_t)left_dc_curr + (int32_t)right_dc_curr;
    if (sum > INT16_MAX) sum = INT16_MAX;
    if (sum < INT16_MIN) sum = INT16_MIN;
    dc_curr = (int16_t)sum;

    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &s_board_temp_adc_fix);
    s_board_temp_adc_filt = (int16_t)(s_board_temp_adc_fix >> 16);
    board_temp_deci_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                        (s_board_temp_adc_filt - TEMP_CAL_LOW_ADC) /
                        (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
}

static void service_control_timer(uint32_t now)
{
    uint32_t dt = now - s_last_control_ms;
    if (dt < VESC_CONTROL_PERIOD_MS) return;

    if (dt > s_stats.max_control_gap_ms && dt <= UINT16_MAX) {
        s_stats.max_control_gap_ms = (uint16_t)dt;
    }
    if (dt > VESC_CONTROL_DT_MAX_MS) dt = VESC_CONTROL_DT_MAX_MS;
    s_last_control_ms = now;

    /* Fresh engineering current before watchdog/PID/detection transitions. */
    update_engineering_values();

    /* Standard VESC read/reset current-average backing. */
    VescProtocol_CurrentTelemetrySample();

    /*
     * This is the single FOC outer-loop owner. It covers the semantic work of
     * upstream pid_thread/timer_thread/fault_stop_thread and commissioning jobs.
     * Do NOT add a second BLDC-style rpm_thread writer on top of this path.
     */
    RuntimeControl_UpdateSlow(dt);
    calcAvgSpeed();

    if (s_stats.control_updates != UINT32_MAX) ++s_stats.control_updates;
    if (main_loop_counter != UINT32_MAX) ++main_loop_counter;
}

static uint8_t status_code(uint32_t now)
{
    /* 1 = runtime fault */
    if (RuntimeControl_ShouldSoundFaultBuzzer()) return 1U;

    /* 2 = over-temperature warning */
    if (TEMP_WARNING_ENABLE && board_temp_deci_c >= TEMP_WARNING) return 2U;

    /* 4 = battery warning level 2 (more severe, evaluate first). */
    if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) return 4U;

    /* 3 = qualified battery warning level 1 */
    if ((uint32_t)(now - s_boot_ms) >= 2000U &&
        BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
        if (s_battery_low_since_ms == 0U) s_battery_low_since_ms = now;
        if ((uint32_t)(now - s_battery_low_since_ms) >= 500U) return 3U;
    } else {
        s_battery_low_since_ms = 0U;
    }

    /* 5 = reverse warning */
    if (BEEPS_BACKWARD && speedAvg < -50) return 5U;

    return 0U;
}

static void service_led_pattern(uint32_t now, uint8_t code)
{
    bool led_on = false;

    if (code == 0U) {
        /* Normal: one short flash every second. */
        const uint32_t phase = (uint32_t)(now - s_status_pattern_epoch_ms) %
                               VESC_NORMAL_FLASH_PERIOD_MS;
        led_on = phase < VESC_NORMAL_FLASH_ON_MS;
    } else {
        /* Error N: N fast flashes, then a long gap. */
        const uint32_t pulse_span = VESC_FAULT_FLASH_ON_MS + VESC_FAULT_FLASH_OFF_MS;
        const uint32_t active_span = (uint32_t)code * pulse_span;
        const uint32_t cycle = active_span + VESC_FAULT_FLASH_GAP_MS;
        const uint32_t phase = (uint32_t)(now - s_status_pattern_epoch_ms) % cycle;

        if (phase < active_span) {
            led_on = (phase % pulse_span) < VESC_FAULT_FLASH_ON_MS;
        }
    }

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void service_startup_melody(uint32_t now)
{
    if (!s_startup_melody_active) return;

    const uint32_t elapsed = now - s_boot_ms;
    if (elapsed >= VESC_STARTUP_MELODY_MS) {
        s_startup_melody_active = false;
        beepCount(0U, 0U, 0U);
        return;
    }

    const uint8_t step = (uint8_t)(elapsed / VESC_STARTUP_NOTE_MS);
    const uint8_t freq = (step < 9U) ? (uint8_t)(8U - step) : 0U;
    beepCount(1U, freq, 1U);
}

static void service_status(uint32_t now)
{
    if ((uint32_t)(now - s_last_status_ms) < VESC_STATUS_PERIOD_MS) return;
    s_last_status_ms = now;

    const uint8_t code = status_code(now);
    backwardDrive = (code == 5U) ? 1U : 0U;

    if (code != s_last_status_code) {
        s_status_pattern_epoch_ms = now;
        s_last_status_code = code;

        if (code != 0U) {
            runtimeBuzzerLastReason = code;
            if (runtimeBuzzerEventCount != UINT16_MAX) ++runtimeBuzzerEventCount;
        }
    }

    runtimeBuzzerReason = code;
    s_stats.status_code = code;

    if (code != 0U) {
        /* Requirement V22: beep count equals visible error code. */
        s_startup_melody_active = false;
        beepCount(code, 24U, 1U);
    } else if (s_startup_melody_active) {
        service_startup_melody(now);
    } else {
        beepCount(0U, 0U, 0U);
    }

    service_led_pattern(now, code);
    if (s_stats.status_updates != UINT32_MAX) ++s_stats.status_updates;
}

static void service_statistics(uint32_t now)
{
    if ((uint32_t)(now - s_last_stat_ms) < VESC_STAT_PERIOD_MS) return;
    s_last_stat_ms = now;

    /*
     * Semantic stat_thread: counters are already maintained at their sources.
     * Do not printf synchronously here; diagnostics read them on demand via VESC.
     * Touching the getters also guarantees this service remains live/testable.
     */
    (void)VescProtocol_RxPackets();
    (void)VescProtocol_RxCrcErrors();
    (void)VescProtocol_TxDrops();
    (void)VescProtocol_RxBudgetYields();
}

static void service_periodic(uint32_t now)
{
    (void)now;
    /*
     * periodic_thread/sample_send_thread/blocking_thread work is advanced by
     * VescProtocol_ServiceBudget() and RuntimeControl_UpdateSlow(). Do not call
     * RuntimeControl_ServiceTelemetry() here: USART3 is exclusively VESC packet
     * framing and legacy raw telemetry would corrupt the VESC Tool byte stream.
     */
}

void VescServices_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));

    const uint32_t now = RuntimeControl_MonotonicMs();
    s_last_control_ms = now;
    s_last_app_ms = now;
    s_last_status_ms = now;
    s_last_stat_ms = now;
    s_boot_ms = now;
    s_battery_low_since_ms = 0U;
    s_status_pattern_epoch_ms = now;
    s_last_status_code = 0U;
    s_startup_melody_active = true;

    s_board_temp_adc_fix = ((int32_t)adc_buffer.temp) << 16;
    s_board_temp_adc_filt = (int16_t)adc_buffer.temp;

    /* Start dark; first service_status() generates the normal flash/melody. */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

void VescServices_Run(void)
{
    /* Highest-priority background work: VESC Tool transport. */
    service_packet_process();

    const uint32_t now = RuntimeControl_MonotonicMs();
    ADC_Slow_Service(now);

    /* Safety/control precedes application-generated commands. */
    service_control_timer(now);
    service_app_adc(now);
    service_status(now);
    service_statistics(now);
    service_periodic(now);

    if (s_stats.scheduler_passes != UINT32_MAX) ++s_stats.scheduler_passes;
}

void VescServices_GetStats(VescServiceStats *out)
{
    if (out == NULL) return;
    *out = s_stats;
}
