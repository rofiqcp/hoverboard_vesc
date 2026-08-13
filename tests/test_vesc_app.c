#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "vesc_app.h"
#include "defines.h"

volatile adc_buf_t adc_buffer;
static int16_t last_permille;
static bool last_speed;
static unsigned calls;

void VescProtocol_AdcSetNormalized(int16_t permille, bool speed_mode)
{
    last_permille = permille;
    last_speed = speed_mode;
    calls++;
}

int main(void)
{
    VescApp_Init();
    assert(vescAppConfig.app_to_use == VESC_APP_UART);
    assert(vescAppConfig.uart_baud == 115200U);
    assert(VescApp_AdcControlSupported(VESC_ADC_CURRENT));
    assert(VescApp_AdcControlSupported(VESC_ADC_PID_REV_CENTER));
    assert(!VescApp_AdcControlSupported(VESC_ADC_CURRENT_REV_BUTTON));

    /* ADC mode safe-start: low input first arms the input-side interlock. */
    vescAppConfig.app_to_use = VESC_APP_ADC;
    vescAppConfig.adc_ctrl_type = VESC_ADC_CURRENT;
    adc_buffer.pa2Analog = 0U;
    adc_buffer.pa3Analog = 0U;
    VescApp_Update(2U);
    assert(calls > 0U);
    assert(last_permille == 0);
    assert(!last_speed);

    /* Full-scale input must become a positive torque request, but ramped. */
    adc_buffer.pa2Analog = 4095U;
    for (unsigned i = 0; i < 40U; ++i) VescApp_Update(2U);
    assert(last_permille > 0);
    assert(last_permille <= 1000);
    assert(!last_speed);
    assert(VescApp_GetVoltage1MicroV() >= 3299000);

    /* PID ADC mode emits speed-mode commands. */
    VescApp_ResetRuntime();
    last_permille = 0;
    vescAppConfig.adc_ctrl_type = VESC_ADC_PID;
    adc_buffer.pa2Analog = 0U;
    VescApp_Update(2U);
    adc_buffer.pa2Analog = 4095U;
    for (unsigned i = 0; i < 40U; ++i) VescApp_Update(2U);
    assert(last_permille > 0);
    assert(last_speed);

    /* Invalid center geometry must fail closed and never divide by zero. */
    VescApp_ResetRuntime();
    vescAppConfig.adc_ctrl_type = VESC_ADC_CURRENT_REV_CENTER;
    vescAppConfig.voltage_start_mV = 1600U;
    vescAppConfig.voltage_center_mV = 1600U;
    vescAppConfig.voltage_end_mV = 1600U;
    adc_buffer.pa2Analog = 0U;
    VescApp_Update(2U);
    adc_buffer.pa2Analog = 2048U;
    VescApp_Update(2U);
    assert(last_permille == 0);

    puts("VESC_APP_FIXEDPOINT_TESTS_PASS");
    return 0;
}
