/*
 * HOVERBOARD VESC V22 - cooperative non-RTOS main
 * ================================================
 * LEFT  = local VESC (default CAN/controller ID 1)
 * RIGHT = virtual CAN VESC (default ID 2)
 *
 * Fast FOC/current control remains in ADC/PWM ISR. All VESC-style background
 * threads are represented by bounded non-blocking services in vesc_services.c.
 */
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "foc_motor.h"
#include "runtime_control.h"
#include "vesc_protocol.h"
#include "vesc_app.h"
#include "vesc_services.h"
#include "left_encoder.h"
#include "motor_current_cal.h"

void SystemClock_Config(void);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

/* Standard diagnostic globals retained for HBTS/custom diagnostics. */
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

int main(void)
{
    HAL_Init();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Keep current-control/DMA timing deterministic. */
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

    SystemClock_Config();

    /* setup.c enables DMA again where required. Start from known state. */
    __HAL_RCC_DMA1_CLK_DISABLE();
    MX_GPIO_Init();

    /* STM32F1 ADC self calibration is done before PWM trigger timers run. */
    MX_ADC1_Init();
    MX_ADC2_Init();
    const bool adc1_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
    const bool adc2_hw_cal_ok = HAL_ADCEx_Calibration_Start(&hadc2) == HAL_OK;
    MotorControl_SetAdcHardwareCalibrationResult(adc1_hw_cal_ok, adc2_hw_cal_ok);

    MX_TIM_Init();
    MotorSystem_Init();

    /* APP defaults must exist before RuntimeControl loads persistent settings. */
    VescApp_Init();
    RuntimeControl_Init();

    /* Product contract: LEFT is always local ID 1, RIGHT virtual CAN ID 2.
     * Persistent APPCONF from older builds must not silently move these IDs. */
    vescAppConfig.controller_id = 1U;

    LeftEncoder_Init();
    LeftEncoder_SetMode(motorConfigLeft.sensor_type == MOTOR_SENSOR_ENCODER_AB);

    HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);
    InputLimits_Init();
    SerialInput_Init();

    /* VESC transport is enabled before any melody/status sequence. */
    VescProtocol_Init();

    HAL_ADC_Start(&hadc1);
    HAL_ADC_Start(&hadc2);

    /* Current-offset calibration is asynchronous and owned by RuntimeControl. */
    (void)MotorControl_RequestCurrentOffsetCalibration();

    /* Non-blocking VESC-style services. No HAL_Delay in the runtime main path. */
    VescServices_Init();

    for (;;) {
        VescServices_Run();
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

    /* Stock hoverboard low-side shunt timing geometry. */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* Do not overwrite SysTick priority here; it was set above deliberately. */
}
