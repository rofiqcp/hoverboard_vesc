/*
 * HOVERBOARD VESC V22 - MAIN-OWNED COOPERATIVE VESC SCHEDULER
 * ===========================================================
 * Target: STM32F103RCT6, bare metal / STM32Cube HAL, NO RTOS.
 *
 * Design rule from upstream VESC architecture:
 * - the deterministic motor/current loop remains in ADC/PWM ISR (motor.c),
 * - every VESC background thread semantic is represented by an explicit task
 *   function below and every task function is called from main_scheduler_run(),
 * - no HAL_Delay(), packet parsing, printf or flash write is used in motor ISR.
 *
 * LEFT  : local controller ID 1, Hall OR ABI encoder (TIM4 PB6/PB7 + Z PB5)
 * RIGHT : virtual-CAN controller ID 2, Hall feedback.
 *
 * IMPORTANT: motor.c and stm32f1xx_it.c are intentionally not changed by this
 * revision. A watchdog-reset boot enters communication-only degraded mode so a
 * CPU/ISR starvation cannot become an endless invisible boot loop.
 */

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "foc_motor.h"
#include "runtime_control.h"
#include "vesc_protocol.h"
#include "vesc_app.h"
#include "left_encoder.h"
#include "motor_current_cal.h"
#include "stm32f1xx_it.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern int16_t batVoltage;
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern volatile uint32_t buzzerTimer;

/* Standard globals retained for VESC/HBTS diagnostics. */
volatile uint8_t runtimeBuzzerReason = 0U;
volatile uint8_t runtimeBuzzerLastReason = 0U;
volatile uint16_t runtimeBuzzerEventCount = 0U;
uint8_t backwardDrive = 0U;
volatile uint32_t main_loop_counter = 0U;
int16_t batVoltageCalib = 0;
int16_t board_temp_deci_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;

/* Main/scheduler diagnostics. These are deliberately simple aligned scalars so
 * the UART diagnostic path may read them without locks on Cortex-M3. */
volatile uint32_t vescMainSchedulerPasses = 0U;
volatile uint32_t vescMainResetFlags = 0U;
volatile uint32_t vescMainLastAliveMs = 0U;
volatile uint8_t vescMainBootFaultCode = 0U;
volatile uint8_t vescMainBootPhase = 0U;
volatile uint8_t vescMainDegradedMode = 0U;
volatile uint32_t vescMainOptionalTaskTicks = 0U;

/* ----------------------------- scheduler policy --------------------------- */
#define VESC_RX_BUDGET_MAIN             128U
#define VESC_RUNTIME_PERIOD_MS          DELAY_IN_MAIN_LOOP /* existing backend = 5 ms */
#define VESC_RUNTIME_DT_MAX_MS          50U
#define VESC_LED_PERIOD_MS              20U
#define VESC_STAT_PERIOD_MS             10U
#define VESC_FLASH_TASK_PERIOD_MS       6U
#define VESC_SHUTDOWN_TASK_PERIOD_MS    10U
#define VESC_STARTUP_MELODY_MS          900U
#define VESC_STARTUP_NOTE_MS            100U
#define VESC_AUTO_CURRENT_CAL_DELAY_MS  1200U
#define VESC_WARNING_ARM_DELAY_MS       2000U
#define VESC_NORMAL_FLASH_PERIOD_MS     1000U
#define VESC_NORMAL_FLASH_ON_MS         100U
#define VESC_FAULT_FLASH_ON_MS          100U
#define VESC_FAULT_FLASH_OFF_MS         100U
#define VESC_FAULT_FLASH_GAP_MS         800U
#define VESC_IWDG_ENABLE                1U
#define VESC_IWDG_RELOAD                2500U /* ~4 s with ~40 kHz LSI / 64 */
#define VESC_IWDG_HEARTBEAT_MAX_AGE_MS  250U

/* Physical capabilities of THIS board/firmware target. All upstream task hooks
 * are still called below. Unsupported hardware hooks return immediately instead
 * of pretending that a driver exists. */
#define VESC_CAP_USB             0U
#define VESC_CAP_PHYSICAL_CAN    0U /* virtual CAN motor 2 is implemented in protocol */
#define VESC_CAP_HFI             0U
#define VESC_CAP_IMU             0U
#define VESC_CAP_PPM             0U
#define VESC_CAP_NUNCHUK         0U
#define VESC_CAP_PAS             0U
#define VESC_CAP_NRF             0U
#define VESC_CAP_LORA            0U
#define VESC_CAP_SI8900          0U
#define VESC_CAP_TS5700          0U
#define VESC_CAP_UAVCAN          0U
#define VESC_CAP_LISPBM          0U
#define VESC_CAP_BOARD_MUX       0U
#define VESC_CAP_BOARD_I2C_TEMP  0U
#define VESC_CAP_FAN             0U
#define VESC_CAP_BOARD_MAG       0U
#define VESC_CAP_CUSTOM_DISPLAY  0U

/* Boot phases are public diagnostics, not motor-state definitions. */
enum {
    VESC_BOOT_STARTUP = 0,
    VESC_BOOT_CURRENT_CAL_WAIT = 1,
    VESC_BOOT_CURRENT_CAL_RUNNING = 2,
    VESC_BOOT_READY = 3,
    VESC_BOOT_CURRENT_CAL_FAILED = 4,
    VESC_BOOT_DEGRADED = 5
};

typedef struct {
    uint32_t last_ms;
    uint32_t runs;
} main_task_clock_t;

static uint32_t s_boot_ms;
static main_task_clock_t s_runtime_clock;
static main_task_clock_t s_app_clock;
static main_task_clock_t s_led_clock;
static main_task_clock_t s_stat_clock;
static main_task_clock_t s_flash_clock;
static main_task_clock_t s_shutdown_clock;
static main_task_clock_t s_optional_clock;
static uint32_t s_status_epoch_ms;
static uint32_t s_battery_low_since_ms;
static uint8_t s_last_status_code;
static bool s_startup_melody_active;
static int32_t s_board_temp_adc_fix;
static int16_t s_board_temp_adc_filt;
static bool s_iwdg_started;
static uint32_t s_last_dma_heartbeat;
static uint32_t s_last_dma_seen_ms;

static bool task_due(main_task_clock_t *clock, uint32_t now, uint32_t period_ms)
{
    if ((uint32_t)(now - clock->last_ms) < period_ms) return false;
    /* Do not catch-up in a burst after a stall. VESC threads sleep until their
     * next wake-up; one cooperative iteration per main pass preserves fairness. */
    clock->last_ms = now;
    if (clock->runs != UINT32_MAX) ++clock->runs;
    return true;
}

static uint32_t app_period_ms(void)
{
    uint32_t hz = vescAppConfig.update_rate_hz;
    if (hz == 0U) hz = 1U;
    if (hz > 500U) hz = 500U;
    uint32_t period = 1000U / hz;
    return period == 0U ? 1U : period;
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

/* RuntimeControl_UpdateSlow currently contains the already-tested hoverboard
 * arming, timeout, homing/detect and FOC outer-loop ownership. Several upstream
 * VESC threads map into that backend. To avoid two writers, every corresponding
 * task wrapper calls this owner, but task_due() lets it execute only once per
 * VESC_RUNTIME_PERIOD_MS. */
static void runtime_slow_owner(uint32_t now)
{
    uint32_t dt = now - s_runtime_clock.last_ms;
    if (dt < VESC_RUNTIME_PERIOD_MS) return;
    if (dt > VESC_RUNTIME_DT_MAX_MS) dt = VESC_RUNTIME_DT_MAX_MS;
    s_runtime_clock.last_ms = now;
    if (s_runtime_clock.runs != UINT32_MAX) ++s_runtime_clock.runs;

    update_engineering_values();
    VescProtocol_CurrentTelemetrySample();
    RuntimeControl_UpdateSlow(dt);
    calcAvgSpeed();
    if (main_loop_counter != UINT32_MAX) ++main_loop_counter;
}

/* ----------------------------- watchdog / boot ---------------------------- */
static void iwdg_start(void)
{
#if VESC_IWDG_ENABLE
    if (s_iwdg_started) return;
    /* STM32F1 IWDG: PR=4 => /64. Long timeout is intentional: it catches a
     * permanently-starved main loop without turning short commissioning work
     * into a reset storm. */
    IWDG->KR = 0x5555U;
    IWDG->PR = 4U;
    IWDG->RLR = VESC_IWDG_RELOAD;
    IWDG->KR = 0xAAAAU;
    IWDG->KR = 0xCCCCU;
    s_iwdg_started = true;
#endif
}

static void iwdg_feed(void)
{
#if VESC_IWDG_ENABLE
    if (s_iwdg_started) IWDG->KR = 0xAAAAU;
#endif
}

static void task_boot_supervisor(uint32_t now)
{
    const uint32_t elapsed = now - s_boot_ms;

    if (vescMainDegradedMode != 0U) {
        vescMainBootPhase = VESC_BOOT_DEGRADED;
        return;
    }

    if (MotorControl_CurrentOffsetsValid()) {
        vescMainBootPhase = VESC_BOOT_READY;
        return;
    }

    const uint8_t cal_state = MotorControl_CurrentOffsetCalState();
    if (cal_state == MOTOR_CURRENT_CAL_FAILED) {
        vescMainBootPhase = VESC_BOOT_CURRENT_CAL_FAILED;
        return;
    }

    if (cal_state == MOTOR_CURRENT_CAL_SETTLING ||
        cal_state == MOTOR_CURRENT_CAL_COLLECTING) {
        vescMainBootPhase = VESC_BOOT_CURRENT_CAL_RUNNING;
        return;
    }

    if (elapsed < VESC_AUTO_CURRENT_CAL_DELAY_MS) {
        vescMainBootPhase = (elapsed < VESC_STARTUP_MELODY_MS) ?
            VESC_BOOT_STARTUP : VESC_BOOT_CURRENT_CAL_WAIT;
        return;
    }

    /* Key hang-isolation change: calibration is NOT started during power melody.
     * UART/VESC Tool and the complete main scheduler are alive first. The fast
     * ISR remains exactly the existing implementation. */
    if (MotorControl_RequestCurrentOffsetCalibration()) {
        vescMainBootPhase = VESC_BOOT_CURRENT_CAL_RUNNING;
    }
}

/* -------------------------- active upstream task map ----------------------- */

/* comm/commands.c::blocking_thread */
static void task_blocking_thread(uint32_t now)
{
    (void)now;
    VescProtocol_ServiceBlocking();
}

/* applications/app_uartcomm.c::packet_process_thread */
static void task_packet_process_thread(uint32_t now)
{
    (void)now;
    VescProtocol_ServiceRxBudget(VESC_RX_BUDGET_MAIN);
}

/* motor/mc_interface.c::fault_stop_thread -- highest cooperative priority. */
static void task_fault_stop_thread(uint32_t now)
{
    runtime_slow_owner(now);
}

/* motor/mc_interface.c::sample_send_thread */
static void task_sample_send_thread(uint32_t now)
{
    (void)now;
    VescProtocol_ServiceSampleSend();
}

/* motor/mc_interface.c::timer_thread */
static void task_mc_interface_timer_thread(uint32_t now)
{
    runtime_slow_owner(now);
}

/* motor/mcpwm_foc.c::timer_thread */
static void task_mcpwm_foc_timer_thread(uint32_t now)
{
    runtime_slow_owner(now);
}

/* motor/mcpwm_foc.c::pid_thread. The PID writer is the same shared backend,
 * never a second competing controller. */
static void task_pid_thread(uint32_t now)
{
    runtime_slow_owner(now);
}

/* timeout.c::timeout_thread + independent watchdog supervisor. */
static void task_timeout_thread(uint32_t now)
{
    runtime_slow_owner(now);

    const uint32_t heartbeat = buzzerTimer;
    if (heartbeat != s_last_dma_heartbeat) {
        s_last_dma_heartbeat = heartbeat;
        s_last_dma_seen_ms = now;
        if (!s_iwdg_started) iwdg_start();
    }

    if ((uint32_t)(now - s_last_dma_seen_ms) <= VESC_IWDG_HEARTBEAT_MAX_AGE_MS) {
        iwdg_feed();
    }
}

/* applications/app_adc.c::adc_thread */
static void task_adc_thread(uint32_t now)
{
    const uint32_t period = app_period_ms();
    uint32_t dt = now - s_app_clock.last_ms;
    if (dt < period) return;
    if (dt > VESC_RUNTIME_DT_MAX_MS) dt = VESC_RUNTIME_DT_MAX_MS;
    s_app_clock.last_ms = now;
    if (s_app_clock.runs != UINT32_MAX) ++s_app_clock.runs;
    VescApp_Update(dt);
}

/* main.c::periodic_thread */
static void task_periodic_thread(uint32_t now)
{
    (void)now;
    VescProtocol_ServicePeriodic();
}

/* encoder/encoder.c::routine_thread.
 * ABI A/B are decoded by TIM4 hardware and consumed by the unchanged 16-kHz
 * ISR; Hall inputs are also sampled there. SPI/BiSS/etc. are not wired. */
static void task_encoder_routine_thread(uint32_t now)
{
    (void)now;
    /* Explicit task call retained; no second encoder reader may touch TIM4's
     * previous-count state because the ISR owns that state. */
}

/* motor/mc_interface.c::stat_thread */
static void task_stat_thread(uint32_t now)
{
    if (!task_due(&s_stat_clock, now, VESC_STAT_PERIOD_MS)) return;
    (void)VescProtocol_RxPackets();
    (void)VescProtocol_RxCrcErrors();
    (void)VescProtocol_TxDrops();
    (void)VescProtocol_RxBudgetYields();
    (void)MotorControl_GetOverrunStreakPct();
}

static uint8_t main_status_code(uint32_t now)
{
    /* Runtime/motor fault remains highest visible priority. */
    if (RuntimeControl_ShouldSoundFaultBuzzer()) return 1U;

    /* Boot fault / watchdog-recovery mode. Communication remains alive, but
     * automatic current calibration and motor arming are intentionally blocked. */
    if (vescMainDegradedMode != 0U) return 6U;
    if (vescMainBootPhase == VESC_BOOT_CURRENT_CAL_FAILED) return 7U;

    /* Sensor readings are allowed to settle before low-priority warnings take
     * buzzer ownership. */
    if ((uint32_t)(now - s_boot_ms) < VESC_WARNING_ARM_DELAY_MS) return 0U;

    if (TEMP_WARNING_ENABLE && board_temp_deci_c >= TEMP_WARNING) return 2U;
    if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) return 4U;

    if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
        if (s_battery_low_since_ms == 0U) s_battery_low_since_ms = now;
        if ((uint32_t)(now - s_battery_low_since_ms) >= 500U) return 3U;
    } else {
        s_battery_low_since_ms = 0U;
    }

    if (BEEPS_BACKWARD && speedAvg < -50) return 5U;
    return 0U;
}

static void led_pattern_write(uint32_t now, uint8_t code)
{
    bool on = false;
    if (code == 0U) {
        const uint32_t phase = (uint32_t)(now - s_status_epoch_ms) %
                               VESC_NORMAL_FLASH_PERIOD_MS;
        on = phase < VESC_NORMAL_FLASH_ON_MS;
    } else {
        const uint32_t span = VESC_FAULT_FLASH_ON_MS + VESC_FAULT_FLASH_OFF_MS;
        const uint32_t active = (uint32_t)code * span;
        const uint32_t cycle = active + VESC_FAULT_FLASH_GAP_MS;
        const uint32_t phase = (uint32_t)(now - s_status_epoch_ms) % cycle;
        if (phase < active) on = (phase % span) < VESC_FAULT_FLASH_ON_MS;
    }
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void startup_melody_service(uint32_t now)
{
    if (!s_startup_melody_active) return;
    const uint32_t elapsed = now - s_boot_ms;
    if (elapsed >= VESC_STARTUP_MELODY_MS) {
        s_startup_melody_active = false;
        beepCount(0U, 0U, 0U);
        return;
    }

    const uint8_t step = (uint8_t)(elapsed / VESC_STARTUP_NOTE_MS);
    const uint8_t freq = (step < 8U) ? (uint8_t)(8U - step) : 1U;
    /* count=0, pattern=0 means continuous tone. Error beep windows must NOT be
     * reused for 100-ms melody notes. */
    beepCount(0U, freq, 0U);
}

/* main.c::led_thread + board switch_color_thread semantic */
static void task_led_thread(uint32_t now)
{
    if (!task_due(&s_led_clock, now, VESC_LED_PERIOD_MS)) return;

    const uint8_t code = main_status_code(now);
    backwardDrive = (code == 5U) ? 1U : 0U;

    if (code != s_last_status_code) {
        s_last_status_code = code;
        s_status_epoch_ms = now;
        if (code != 0U) {
            runtimeBuzzerLastReason = code;
            if (runtimeBuzzerEventCount != UINT16_MAX) ++runtimeBuzzerEventCount;
        }
    }

    runtimeBuzzerReason = code;

    /* Only a real runtime fault is allowed to pre-empt the startup melody.
     * Degraded/calibration warnings become audible immediately afterwards. */
    if (s_startup_melody_active && code != 1U) {
        startup_melody_service(now);
    } else if (code != 0U) {
        s_startup_melody_active = false;
        beepCount(code, 24U, 1U);
    } else {
        beepCount(0U, 0U, 0U);
    }

    led_pattern_write(now, code);
}

/* main.c::flash_integrity_check_thread.
 * This STM32F103 port does not store an upstream flash-helper reference CRC.
 * The task is still called at the upstream cadence, but deliberately performs no
 * fake comparison. Adding a manifest CRC later belongs here, never in ISR. */
static void task_flash_integrity_check_thread(uint32_t now)
{
    if (!task_due(&s_flash_clock, now, VESC_FLASH_TASK_PERIOD_MS)) return;
}

/* util/worker.c::work_thread. There is no queued worker subsystem in this target;
 * all current long operations are explicit asynchronous state machines. */
static void task_work_thread(uint32_t now) { (void)now; }

static void optional_task_touch(void)
{
    if (vescMainOptionalTaskTicks != UINT32_MAX) ++vescMainOptionalTaskTicks;
}

/* hwconf/shutdown.c::shutdown_thread. The current board has a power-hold output
 * but no migrated VESC shutdown state-machine yet. Keep the latch asserted so a
 * background task cannot accidentally power the controller off. */
static void task_shutdown_thread(uint32_t now)
{
    if (!task_due(&s_shutdown_clock, now, VESC_SHUTDOWN_TASK_PERIOD_MS)) return;
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
}

/* ------------------ explicit conditional upstream task hooks -------------- */
/* Every function below is called from main_scheduler_run() on every pass. The
 * compile-time capability is explicit so absent hardware never becomes a fake
 * VESC feature or an accidental register access. */
static void task_usb_serial_read_thread(uint32_t now) { (void)now; (void)VESC_CAP_USB; optional_task_touch(); }
static void task_usb_serial_process_thread(uint32_t now) { (void)now; (void)VESC_CAP_USB; optional_task_touch(); }
static void task_cancom_read_thread(uint32_t now) { (void)now; (void)VESC_CAP_PHYSICAL_CAN; optional_task_touch(); }
static void task_cancom_process_thread(uint32_t now) { (void)now; (void)VESC_CAP_PHYSICAL_CAN; optional_task_touch(); }
static void task_cancom_status_thread(uint32_t now) { (void)now; (void)VESC_CAP_PHYSICAL_CAN; optional_task_touch(); }
static void task_cancom_status_thread_2(uint32_t now) { (void)now; (void)VESC_CAP_PHYSICAL_CAN; optional_task_touch(); }
static void task_cancom_status_internal_thread(uint32_t now) { (void)now; optional_task_touch(); /* virtual ID2 handled by protocol */ }
static void task_mcpwm_timer_thread(uint32_t now) { (void)now; optional_task_touch(); /* BLDC/DC only; target is FOC */ }
static void task_rpm_thread(uint32_t now) { (void)now; optional_task_touch(); /* BLDC/DC only; do not duplicate FOC PID */ }
static void task_hfi_thread(uint32_t now) { (void)now; (void)VESC_CAP_HFI; optional_task_touch(); }
static void task_imu_thread(uint32_t now) { (void)now; (void)VESC_CAP_IMU; optional_task_touch(); }
static void task_ppm_thread(uint32_t now) { (void)now; (void)VESC_CAP_PPM; optional_task_touch(); }
static void task_chuk_thread(uint32_t now) { (void)now; (void)VESC_CAP_NUNCHUK; optional_task_touch(); }
static void task_nunchuk_output_thread(uint32_t now) { (void)now; (void)VESC_CAP_NUNCHUK; optional_task_touch(); }
static void task_pas_thread(uint32_t now) { (void)now; (void)VESC_CAP_PAS; optional_task_touch(); }
static void task_nrf_rx_thread(uint32_t now) { (void)now; (void)VESC_CAP_NRF; optional_task_touch(); }
static void task_nrf_tx_thread(uint32_t now) { (void)now; (void)VESC_CAP_NRF; optional_task_touch(); }
static void task_lora_packet_process_thread(uint32_t now) { (void)now; (void)VESC_CAP_LORA; optional_task_touch(); }
static void task_si_read_thread(uint32_t now) { (void)now; (void)VESC_CAP_SI8900; optional_task_touch(); }
static void task_ts5700n8501_thread(uint32_t now) { (void)now; (void)VESC_CAP_TS5700; optional_task_touch(); }
static void task_canard_thread(uint32_t now) { (void)now; (void)VESC_CAP_UAVCAN; optional_task_touch(); }
static void task_lisp_eval_thread(uint32_t now) { (void)now; (void)VESC_CAP_LISPBM; optional_task_touch(); }
static void task_lisp_lib_thread(uint32_t now) { (void)now; (void)VESC_CAP_LISPBM; optional_task_touch(); }
static void task_dpv_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_sten_uart_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_app_custom_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_finn_control_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_finn_status_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_skypuff_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_erockit_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_smart_switch_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_mux_thread(uint32_t now) { (void)now; (void)VESC_CAP_BOARD_MUX; optional_task_touch(); }
static void task_switch_color_thread(uint32_t now) { (void)now; optional_task_touch(); /* task_led_thread owns physical LED */ }
static void task_display_process_thread(uint32_t now) { (void)now; (void)VESC_CAP_CUSTOM_DISPLAY; optional_task_touch(); }
static void task_sense_thread(uint32_t now) { (void)now; optional_task_touch(); }
static void task_temp_thread(uint32_t now) { (void)now; (void)VESC_CAP_BOARD_I2C_TEMP; optional_task_touch(); }
static void task_fan_control_thread(uint32_t now) { (void)now; (void)VESC_CAP_FAN; optional_task_touch(); }
static void task_mag_thread(uint32_t now) { (void)now; (void)VESC_CAP_BOARD_MAG; optional_task_touch(); }
static void task_lisp_event_thread(uint32_t now) { (void)now; (void)VESC_CAP_LISPBM; optional_task_touch(); }
static void task_lisp_cmds_send_task(uint32_t now) { (void)now; (void)VESC_CAP_LISPBM; optional_task_touch(); }

/* This call list is intentionally explicit. Do not hide it behind a task table:
 * it is the bare-metal equivalent of the VESC thread inventory and is easy to
 * audit against upstream source/documentation. */
static void main_scheduler_run(uint32_t now)
{
    /* Highest-priority event/safety and communication work first. */
    task_fault_stop_thread(now);
    task_packet_process_thread(now);
    task_blocking_thread(now);
    task_sample_send_thread(now);

    /* Motor interface / FOC outer-loop tasks. Fast FOC itself stays in ISR. */
    task_mc_interface_timer_thread(now);
    task_mcpwm_foc_timer_thread(now);
    task_pid_thread(now);
    task_encoder_routine_thread(now);
    task_timeout_thread(now);

    /* Active application / main housekeeping. */
    task_adc_thread(now);
    task_periodic_thread(now);
    task_stat_thread(now);
    task_work_thread(now);
    task_shutdown_thread(now);
    task_led_thread(now);
    task_flash_integrity_check_thread(now);

    /* All conditional/board-specific upstream task hooks are still executed,
     * but on a bounded 10-ms compatibility slice rather than every tight loop. */
    if (task_due(&s_optional_clock, now, 10U)) {
        task_usb_serial_read_thread(now);
        task_usb_serial_process_thread(now);
        task_cancom_read_thread(now);
        task_cancom_process_thread(now);
        task_cancom_status_thread(now);
        task_cancom_status_thread_2(now);
        task_cancom_status_internal_thread(now);
        task_mcpwm_timer_thread(now);
        task_rpm_thread(now);
        task_hfi_thread(now);
        task_ppm_thread(now);
        task_chuk_thread(now);
        task_nunchuk_output_thread(now);
        task_pas_thread(now);
        task_imu_thread(now);
        task_nrf_rx_thread(now);
        task_nrf_tx_thread(now);
        task_lora_packet_process_thread(now);
        task_si_read_thread(now);
        task_ts5700n8501_thread(now);
        task_canard_thread(now);
        task_lisp_eval_thread(now);
        task_lisp_lib_thread(now);
        task_lisp_event_thread(now);
        task_lisp_cmds_send_task(now);
        task_dpv_thread(now);
        task_sten_uart_thread(now);
        task_app_custom_thread(now);
        task_finn_control_thread(now);
        task_finn_status_thread(now);
        task_skypuff_thread(now);
        task_erockit_thread(now);
        task_smart_switch_thread(now);
        task_mux_thread(now);
        task_switch_color_thread(now);
        task_display_process_thread(now);
        task_sense_thread(now);
        task_temp_thread(now);
        task_fan_control_thread(now);
        task_mag_thread(now);
    }

    task_boot_supervisor(now);

    vescMainLastAliveMs = now;
    if (vescMainSchedulerPasses != UINT32_MAX) ++vescMainSchedulerPasses;
}

static void main_scheduler_init(void)
{
    const uint32_t now = RuntimeControl_MonotonicMs();
    s_boot_ms = now;
    s_runtime_clock.last_ms = now;
    s_app_clock.last_ms = now;
    s_led_clock.last_ms = now;
    s_stat_clock.last_ms = now;
    s_flash_clock.last_ms = now;
    s_shutdown_clock.last_ms = now;
    s_optional_clock.last_ms = now;
    s_status_epoch_ms = now;
    s_battery_low_since_ms = 0U;
    s_last_status_code = 0U;
    s_startup_melody_active = true;
    s_board_temp_adc_fix = ((int32_t)adc_buffer.temp) << 16;
    s_board_temp_adc_filt = (int16_t)adc_buffer.temp;
    s_iwdg_started = false;
    s_last_dma_heartbeat = buzzerTimer;
    s_last_dma_seen_ms = now;

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    beepCount(0U, 0U, 0U);
}

int main(void)
{
    /* Preserve reset cause before HAL/application code can clear it. A watchdog
     * or CPU exception from the previous boot selects safe communication-only
     * recovery instead of immediately re-entering the same crash path. */
    const uint32_t reset_flags = RCC->CSR;
    vescMainResetFlags = reset_flags;
    vescMainBootFaultCode = SystemFault_GetBootCode();
    if ((reset_flags & (RCC_CSR_IWDGRSTF | RCC_CSR_WWDGRSTF)) != 0U ||
        vescMainBootFaultCode != 0U) {
        vescMainDegradedMode = 1U;
        vescMainBootPhase = VESC_BOOT_DEGRADED;
    }
    RCC->CSR |= RCC_CSR_RMVF;

    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Keep the currently validated ISR priority scheme unchanged. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();

    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();
    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);

    MX_ADC1_Init();
    MX_ADC2_Init();
    const bool adc1_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
    const bool adc2_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK;
    MotorControl_SetAdcHardwareCalibrationResult(adc1_hw_cal_ok, adc2_hw_cal_ok);

    MX_TIM_Init();
    MotorSystem_Init();

    VescApp_Init();
    RuntimeControl_Init();

    /* Fixed dual-controller contract. */
    vescAppConfig.controller_id = 1U;

    LeftEncoder_Init();
    LeftEncoder_SetMode(motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB);

    InputLimits_Init();
    SerialInput_Init();
    VescProtocol_Init();

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    /* IMPORTANT: current-offset calibration is deliberately NOT requested here.
     * task_boot_supervisor() starts it after the full main scheduler and VESC Tool
     * transport have been alive longer than the complete startup melody. */
    main_scheduler_init();

    for (;;) {
        const uint32_t now = RuntimeControl_MonotonicMs();
        ADC_Slow_Service(now);
        main_scheduler_run(now);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}
