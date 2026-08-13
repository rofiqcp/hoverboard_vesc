/*
 * Firmware ESC board 0 - USART3 runtime controller
 * ================================================
 * Mode VLT/TRQ/SPD/POS dipilih langsung oleh host untuk setiap motor saat runtime.
 * DMA ADC tetap 16 kHz; seperti V1, LEFT dan RIGHT FOC sama-sama update setiap DMA ISR.
 */
#include <stdlib.h>
#include <stdint.h>
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

void SystemClock_Config(void);

extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;
extern UART_HandleTypeDef huart3;

extern int16_t curL_DC;
extern int16_t curR_DC;
extern int16_t speedAvg;
extern int16_t speedAvgAbs;
extern int16_t batVoltage;
extern uint8_t enable;
extern volatile uint32_t buzzerTimer;

/* V8 HBTS buzzer diagnostics. Reason values:
 * 0 none, 1 hard runtime fault, 2 temperature warning,
 * 3 battery level-1, 4 battery level-2, 5 reverse warning. */
volatile uint8_t runtimeBuzzerReason = 0U;
volatile uint8_t runtimeBuzzerLastReason = 0U;
volatile uint16_t runtimeBuzzerEventCount = 0U;

uint8_t backwardDrive = 0;
volatile uint32_t main_loop_counter = 0;
int16_t batVoltageCalib = 0;
int16_t board_temp_deci_c = 0;
int16_t left_dc_curr = 0;
int16_t right_dc_curr = 0;
int16_t dc_curr = 0;


/**
 * Entry point. Hardware dan FOC diinisialisasi lebih dahulu, konfigurasi PID
 * persistent dimuat dari EEPROM emulasi, lalu USART3 mulai menerima frame host.
 */
int main(void)
{
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();
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

    /* STM32F1 ADC self-calibration must run while PWM trigger timers are still
     * stopped. This keeps the first current-offset acquisition deterministic. */
    MX_ADC1_Init();
    MX_ADC2_Init();
    const bool adc1_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
    const bool adc2_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK;
    MotorControl_SetAdcHardwareCalibrationResult(adc1_hw_cal_ok, adc2_hw_cal_ok);

    MX_TIM_Init();
    MotorSystem_Init();
    /* VESC app defaults must exist before RuntimeControl_Init(), because the
     * EEPROM v16 loader may overwrite them with a persistent VESC APP config. */
    VescApp_Init();
    RuntimeControl_Init();
    LeftEncoder_Init();
    LeftEncoder_SetMode(motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB);

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    InputLimits_Init();
    SerialInput_Init();
    VescProtocol_Init();
    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);
    /* Request zero-current calibration only after both ADCs, timers and the
     * complete runtime are initialized. V11 intentionally calibrates in the stock
     * active LOW-FET zero-vector ADC domain, then releases MOE before normal use. */
    (void)MotorControl_RequestCurrentOffsetCalibration();

    poweronMelody();
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    int32_t board_temp_adc_fix = ((int32_t)adc_buffer.temp) << 16;
    int16_t board_temp_adc_filt = (int16_t)adc_buffer.temp;
    uint32_t lastSlowTick = RuntimeControl_MonotonicMs();
    uint32_t lastAppTick = lastSlowTick;
    /* V14: a one-shot BATTERY_LEVEL1 beep was proven in the 00:23:19 log even
     * though the settled pack is ~40 V. Qualify low voltage only after the ADC /
     * battery filter has had time to settle and only if it remains low. This
     * suppresses the boot transient without disabling a genuine sustained low-
     * battery warning. */
    const uint32_t batteryWarningBootTick = lastSlowTick;
    uint32_t batteryLowSinceTick = 0U;

    /* Jangan blok boot menunggu BUTTON_PIN. Host/telemetry harus tetap hidup. */
    for (;;) {
        /* Poll DMA RX/TX dari thread context. Ini membuat komunikasi tetap maju
         * walaupun IRQ USART/TX completion terlambat karena ADC/PWM 16 kHz. */
        /* VESC Tool transport berjalan sepenuhnya di background: RX circular DMA
         * diparse di sini dan TX memakai DMA queue. Tidak ada parsing/float/CRC
         * di current-control ISR 16 kHz. */
        VescProtocol_Service();

        const uint32_t now = RuntimeControl_MonotonicMs();
        ADC_Slow_Service(now);

        /* APP_ADC runs independently of the 200 Hz outer-control loop. The ADC2
         * injected source is sampled at 500 Hz, so clamp the requested VESC rate
         * to that physical maximum. APP_UART keeps working continuously via DMA. */
        uint32_t app_rate_hz = vescAppConfig.update_rate_hz;
        if (app_rate_hz == 0U) app_rate_hz = 1U;
        if (app_rate_hz > 500U) app_rate_hz = 500U;
        uint32_t app_period_ms = 1000U / app_rate_hz;
        if (app_period_ms == 0U) app_period_ms = 1U;
        uint32_t app_dt_ms = now - lastAppTick;
        if (app_dt_ms >= app_period_ms) {
            if (app_dt_ms > 50U) app_dt_ms = 50U;
            lastAppTick = now;
            VescApp_Update(app_dt_ms);
        }

        uint32_t dt_ms = now - lastSlowTick;
        if (dt_ms < DELAY_IN_MAIN_LOOP) continue;
        if (dt_ms > 50U) dt_ms = 50U;
        lastSlowTick = now;

        /* Ambil engineering current terbaru SEBELUM state-machine runtime.
         * Auto Detect/ARM fault gate tidak lagi melihat sample 5 ms sebelumnya. */
        /* V7: expose validated DC-link current only. The raw ADC delta remains
         * available to the 16-kHz protection and HBTS diagnostics, but VESC
         * Input Current/Battery Current must never be built from passive-spin
         * common-mode spikes while the bridge is released. */
        left_dc_curr = MotorControl_GetDcInputCentiAmp(true);
        right_dc_curr = MotorControl_GetDcInputCentiAmp(false);
        int32_t dc_sum = (int32_t)left_dc_curr + (int32_t)right_dc_curr;
        if (dc_sum > INT16_MAX) dc_sum = INT16_MAX;
        if (dc_sum < INT16_MIN) dc_sum = INT16_MIN;
        dc_curr = (int16_t)dc_sum;

        /* Match VESC GET_VALUES semantics: accumulate read-reset current averages
         * in background. Instantaneous samples remain available through HBTS. */
        VescProtocol_CurrentTelemetrySample();

        /* Watchdog + VESC-style speed/position outer loops run around 200 Hz. */
        RuntimeControl_UpdateSlow(dt_ms);
        calcAvgSpeed();

        /* Temperatur board dan tegangan baterai diproses seperti firmware asli. */
        filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adc_fix);
        board_temp_adc_filt = (int16_t)(board_temp_adc_fix >> 16);
        board_temp_deci_c = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) *
                           (board_temp_adc_filt - TEMP_CAL_LOW_ADC) /
                           (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;
        batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;

        // poweroffPressCheck();

        /* Safety thermal/battery/fault dipertahankan. */
        // if ((TEMP_POWEROFF_ENABLE && board_temp_deci_c >= TEMP_POWEROFF && speedAvgAbs < 20) ||
        //     (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {
        //     poweroff();
        // } 
        bool battery_lvl1_qualified = false;
        if ((uint32_t)(now - batteryWarningBootTick) >= 2000U &&
            BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {
            if (batteryLowSinceTick == 0U) batteryLowSinceTick = now;
            battery_lvl1_qualified = (uint32_t)(now - batteryLowSinceTick) >= 500U;
        } else {
            batteryLowSinceTick = 0U;
        }

        uint8_t buzzer_reason = 0U;
        if (RuntimeControl_ShouldSoundFaultBuzzer()) {
            buzzer_reason = 1U;
            /* Buzzer hanya alarm. Jangan lagi menulis master enable global di sini:
             * RuntimeControl sudah memutus gate motor yang fault secara per-sisi. */
            beepCount(1, 24, 1);
        } else if (TEMP_WARNING_ENABLE && board_temp_deci_c >= TEMP_WARNING) {
            buzzer_reason = 2U;
            beepCount(5, 24, 1);
        } else if (battery_lvl1_qualified) {
            buzzer_reason = 3U;
            beepCount(0, 10, 6);
        } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {
            buzzer_reason = 4U;
            beepCount(0, 10, 30);
        } else if (BEEPS_BACKWARD && speedAvg < -50) {
            buzzer_reason = 5U;
            beepCount(0, 5, 1);
            backwardDrive = 1;
        } else {
            beepCount(0, 0, 0);
            backwardDrive = 0;
        }
        if (buzzer_reason != runtimeBuzzerReason) {
            if (buzzer_reason != 0U) {
                runtimeBuzzerLastReason = buzzer_reason;
                if (runtimeBuzzerEventCount != UINT16_MAX) ++runtimeBuzzerEventCount;
            }
            runtimeBuzzerReason = buzzer_reason;
        }

        // if (abs(runtimeCommandLeft) > 50 || abs(runtimeCommandRight) > 50) {
        //     inactivity_timeout_counter = 0;
        // } else {
        //     ++inactivity_timeout_counter;
        // }
        // if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60UL * 1000UL) / (DELAY_IN_MAIN_LOOP + 1U)) {
        //     poweroff();
        // }

        ++main_loop_counter;
    }
}

/**
 * Mengatur clock STM32F103 dari HSI+PLL menjadi 64 MHz, pembagi APB, clock ADC,
 * dan SysTick 1 ms. Nilai clock dipertahankan dari firmware board 0 sebelumnya.
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  /* Keep the stock hoverboard ADC timing. The original FOC firmware uses /4
   * together with 7.5-cycle phase-current sampling and TIM8 offset=80 ticks.
   * That timing is part of the low-side-shunt sampling geometry. */
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn priority TIDAK di-set ulang di sini. main() sudah menetapkan
   * (0,0) sebelum SystemClock_Config() dipanggil, sama dengan DMA1_Channel1
   * (current-control) dan DMA1_Channel2 (UART TX DMA) -- ketiganya harus tetap
   * satu preemption priority supaya tidak saling preempt (lihat komentar di
   * main()). Baris HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0) yang dulu ada di
   * sini adalah sisa boilerplate CubeMX yang diam-diam menimpa (0,0) tersebut
   * menjadi prioritas 3 -- itu bug, bukan konfigurasi yang disengaja. */
}
