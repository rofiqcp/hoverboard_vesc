#ifndef VESC_APP_H
#define VESC_APP_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    VESC_APP_NONE = 0,
    VESC_APP_PPM = 1,
    VESC_APP_ADC = 2,
    VESC_APP_UART = 3,
    VESC_APP_PPM_UART = 4,
    VESC_APP_ADC_UART = 5
} VescAppMode;

typedef enum {
    VESC_ADC_NONE = 0,
    VESC_ADC_CURRENT = 1,
    VESC_ADC_CURRENT_REV_CENTER = 2,
    VESC_ADC_CURRENT_REV_BUTTON = 3,
    VESC_ADC_CURRENT_REV_BUTTON_BRAKE_ADC = 4,
    VESC_ADC_CURRENT_REV_BUTTON_BRAKE_CENTER = 5,
    VESC_ADC_CURRENT_NOREV_BRAKE_CENTER = 6,
    VESC_ADC_CURRENT_NOREV_BRAKE_BUTTON = 7,
    VESC_ADC_CURRENT_NOREV_BRAKE_ADC = 8,
    VESC_ADC_DUTY = 9,
    VESC_ADC_DUTY_REV_CENTER = 10,
    VESC_ADC_DUTY_REV_BUTTON = 11,
    VESC_ADC_PID = 12,
    VESC_ADC_PID_REV_CENTER = 13,
    VESC_ADC_PID_REV_BUTTON = 14
} VescAdcCtrl;

typedef struct {
    uint8_t controller_id;
    uint32_t timeout_ms;
    int16_t timeout_brake_cA;
    uint8_t app_to_use;
    uint32_t uart_baud;

    uint8_t adc_ctrl_type;
    uint16_t adc_hyst_mV;
    uint16_t voltage_start_mV;
    uint16_t voltage_end_mV;
    uint16_t voltage_min_mV;
    uint16_t voltage_max_mV;
    uint16_t voltage_center_mV;
    uint16_t voltage2_start_mV;
    uint16_t voltage2_end_mV;
    uint8_t use_filter;
    uint8_t safe_start;
    uint8_t buttons;
    uint8_t voltage_inverted;
    uint8_t voltage2_inverted;
    int16_t throttle_exp_milli;
    int16_t throttle_exp_brake_milli;
    uint8_t throttle_exp_mode;
    uint16_t ramp_time_pos_ms;
    uint16_t ramp_time_neg_ms;
    uint8_t multi_esc;
    uint8_t tc;
    uint16_t tc_max_diff_milli;
    uint16_t update_rate_hz;
} VescAppConfig;

extern VescAppConfig vescAppConfig;

void VescApp_Init(void);
void VescApp_SetDefaults(VescAppConfig *config);
void VescApp_ResetRuntime(void);
void VescApp_Update(uint32_t dt_ms);
int32_t VescApp_GetDecoded1Micro(void);
int32_t VescApp_GetDecoded2Micro(void);
int32_t VescApp_GetVoltage1MicroV(void);
int32_t VescApp_GetVoltage2MicroV(void);
bool VescApp_UartEnabled(void);
bool VescApp_AdcControlSupported(uint8_t control_type);

#endif /* VESC_APP_H */
