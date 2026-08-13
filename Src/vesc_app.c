#include "vesc_app.h"
#include "defines.h"
#include "vesc_protocol.h"
#include <limits.h>
#include <stddef.h>

extern volatile adc_buf_t adc_buffer;

VescAppConfig vescAppConfig;

static int32_t filtered1_q15 = 0;
static int32_t filtered2_q15 = 0;
static int16_t ramped_permille = 0;
static bool safe_start_ok = false;
static uint8_t previous_app_mode = 0xFFU;
static uint8_t previous_control_type = 0xFFU;

static int32_t clamp32(int32_t value, int32_t minimum, int32_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static uint16_t adc_to_mv(uint16_t raw)
{
    return (uint16_t)(((uint32_t)raw * 3300U + 2047U) / 4095U);
}

static int32_t map_q15(uint16_t mv, uint16_t start, uint16_t end, bool inverted)
{
    if (end <= start) return 0;
    int32_t q = ((int32_t)mv - (int32_t)start) * 32767 / (int32_t)(end - start);
    q = clamp32(q, 0, 32767);
    return inverted ? (32767 - q) : q;
}

static int32_t map_center_permille(uint16_t mv, uint16_t start,
                                   uint16_t center, uint16_t end)
{
    if (center <= start || end <= center) return 0;
    const int32_t x = (int32_t)mv - (int32_t)center;
    if (x >= 0) {
        return clamp32((x * 1000) / (int32_t)(end - center), 0, 1000);
    }
    return clamp32((x * 1000) / (int32_t)(center - start), -1000, 0);
}

static int16_t ramp_permille(int16_t current, int16_t target,
                             uint32_t dt_ms, uint16_t positive_ms,
                             uint16_t negative_ms)
{
    if (current == target) return current;
    const bool increasing = target > current;
    const uint16_t ramp_ms = increasing ? positive_ms : negative_ms;
    if (ramp_ms == 0U || dt_ms >= ramp_ms) return target;

    uint32_t step = (1000U * dt_ms + (uint32_t)ramp_ms - 1U) / (uint32_t)ramp_ms;
    if (step == 0U) step = 1U;
    if (step > 2000U) step = 2000U;

    int32_t next = current;
    if (increasing) {
        next += (int32_t)step;
        if (next > target) next = target;
    } else {
        next -= (int32_t)step;
        if (next < target) next = target;
    }
    return (int16_t)clamp32(next, -1000, 1000);
}

bool VescApp_AdcControlSupported(uint8_t control_type)
{
    switch (control_type) {
    case VESC_ADC_NONE:
    case VESC_ADC_CURRENT:
    case VESC_ADC_CURRENT_REV_CENTER:
    case VESC_ADC_CURRENT_NOREV_BRAKE_ADC:
    case VESC_ADC_DUTY:
    case VESC_ADC_DUTY_REV_CENTER:
    case VESC_ADC_PID:
    case VESC_ADC_PID_REV_CENTER:
        return true;
    default:
        /* Button-based VESC ADC modes require digital button inputs that are not
         * present on this hoverboard connector. Reject rather than emulate them
         * with unsafe or surprising semantics. */
        return false;
    }
}

void VescApp_SetDefaults(VescAppConfig *config)
{
    if (config == NULL) return;
    config->controller_id = 10U;
    config->timeout_ms = 1000U;
    config->timeout_brake_cA = 0;
    config->app_to_use = VESC_APP_UART;
    config->uart_baud = 115200U;
    config->adc_ctrl_type = VESC_ADC_NONE;
    config->adc_hyst_mV = 20U;
    config->voltage_start_mV = 900U;
    config->voltage_end_mV = 3000U;
    config->voltage_min_mV = 0U;
    config->voltage_max_mV = 3300U;
    config->voltage_center_mV = 1650U;
    config->voltage2_start_mV = 900U;
    config->voltage2_end_mV = 3000U;
    config->use_filter = 1U;
    config->safe_start = 1U;
    config->buttons = 0U;
    config->voltage_inverted = 0U;
    config->voltage2_inverted = 0U;
    config->throttle_exp_milli = 0;
    config->throttle_exp_brake_milli = 0;
    config->throttle_exp_mode = 0U;
    config->ramp_time_pos_ms = 400U;
    config->ramp_time_neg_ms = 200U;
    config->multi_esc = 0U;
    config->tc = 0U;
    config->tc_max_diff_milli = 3000U;
    config->update_rate_hz = 500U;
}

void VescApp_ResetRuntime(void)
{
    filtered1_q15 = 0;
    filtered2_q15 = 0;
    ramped_permille = 0;
    safe_start_ok = false;
    previous_app_mode = 0xFFU;
    previous_control_type = 0xFFU;
}

void VescApp_Init(void)
{
    VescApp_SetDefaults(&vescAppConfig);
    VescApp_ResetRuntime();
}

bool VescApp_UartEnabled(void)
{
    /* Permanent UART is intentional: unlike a normal VESC, this board has no USB
     * VESC Tool transport. USART3 must remain reachable in APP_ADC mode too. */
    return vescAppConfig.app_to_use == VESC_APP_UART ||
           vescAppConfig.app_to_use == VESC_APP_ADC_UART ||
           vescAppConfig.app_to_use == VESC_APP_ADC;
}

int32_t VescApp_GetDecoded1Micro(void)
{
    return (filtered1_q15 * 1000000) / 32767;
}

int32_t VescApp_GetDecoded2Micro(void)
{
    return (filtered2_q15 * 1000000) / 32767;
}

int32_t VescApp_GetVoltage1MicroV(void)
{
    return (int32_t)adc_to_mv(adc_buffer.pa2Analog) * 1000;
}

int32_t VescApp_GetVoltage2MicroV(void)
{
    return (int32_t)adc_to_mv(adc_buffer.pa3Analog) * 1000;
}

void VescApp_Update(uint32_t dt_ms)
{
    const bool adc_active = vescAppConfig.app_to_use == VESC_APP_ADC ||
                            vescAppConfig.app_to_use == VESC_APP_ADC_UART;
    if (!adc_active) {
        ramped_permille = 0;
        return;
    }

    if (previous_app_mode != vescAppConfig.app_to_use ||
        previous_control_type != vescAppConfig.adc_ctrl_type) {
        filtered1_q15 = 0;
        filtered2_q15 = 0;
        ramped_permille = 0;
        safe_start_ok = false;
        previous_app_mode = vescAppConfig.app_to_use;
        previous_control_type = vescAppConfig.adc_ctrl_type;
    }

    if (!VescApp_AdcControlSupported(vescAppConfig.adc_ctrl_type)) {
        VescProtocol_AdcSetNormalized(0, false);
        return;
    }

    const uint16_t voltage1_mv = adc_to_mv(adc_buffer.pa2Analog);
    const uint16_t voltage2_mv = adc_to_mv(adc_buffer.pa3Analog);
    const int32_t input1_q15 = map_q15(voltage1_mv,
                                       vescAppConfig.voltage_start_mV,
                                       vescAppConfig.voltage_end_mV,
                                       vescAppConfig.voltage_inverted != 0U);
    const int32_t input2_q15 = map_q15(voltage2_mv,
                                       vescAppConfig.voltage2_start_mV,
                                       vescAppConfig.voltage2_end_mV,
                                       vescAppConfig.voltage2_inverted != 0U);

    if (vescAppConfig.use_filter != 0U) {
        filtered1_q15 += (input1_q15 - filtered1_q15) >> 2;
        filtered2_q15 += (input2_q15 - filtered2_q15) >> 2;
    } else {
        filtered1_q15 = input1_q15;
        filtered2_q15 = input2_q15;
    }

    if (vescAppConfig.safe_start != 0U && !safe_start_ok) {
        if (filtered1_q15 < 1200 && filtered2_q15 < 1200) {
            safe_start_ok = true;
        } else {
            VescProtocol_AdcSetNormalized(0, false);
            return;
        }
    }

    int32_t target_permille = 0;
    bool speed_mode = false;
    switch (vescAppConfig.adc_ctrl_type) {
    case VESC_ADC_CURRENT:
        target_permille = (filtered1_q15 * 1000) / 32767;
        break;
    case VESC_ADC_CURRENT_REV_CENTER:
        target_permille = map_center_permille(voltage1_mv,
                                              vescAppConfig.voltage_start_mV,
                                              vescAppConfig.voltage_center_mV,
                                              vescAppConfig.voltage_end_mV);
        break;
    case VESC_ADC_CURRENT_NOREV_BRAKE_ADC:
        target_permille = ((filtered1_q15 - filtered2_q15) * 1000) / 32767;
        break;
    case VESC_ADC_DUTY:
        target_permille = (filtered1_q15 * 1000) / 32767;
        break;
    case VESC_ADC_DUTY_REV_CENTER:
        target_permille = map_center_permille(voltage1_mv,
                                              vescAppConfig.voltage_start_mV,
                                              vescAppConfig.voltage_center_mV,
                                              vescAppConfig.voltage_end_mV);
        break;
    case VESC_ADC_PID:
        target_permille = (filtered1_q15 * 1000) / 32767;
        speed_mode = true;
        break;
    case VESC_ADC_PID_REV_CENTER:
        target_permille = map_center_permille(voltage1_mv,
                                              vescAppConfig.voltage_start_mV,
                                              vescAppConfig.voltage_center_mV,
                                              vescAppConfig.voltage_end_mV);
        speed_mode = true;
        break;
    default:
        VescProtocol_AdcSetNormalized(0, false);
        return;
    }

    target_permille = clamp32(target_permille, -1000, 1000);
    ramped_permille = ramp_permille(ramped_permille, (int16_t)target_permille,
                                    dt_ms, vescAppConfig.ramp_time_pos_ms,
                                    vescAppConfig.ramp_time_neg_ms);
    VescProtocol_AdcSetNormalized(ramped_permille, speed_mode);
}
