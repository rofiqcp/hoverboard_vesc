#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "foc_motor.h"
#include "motor_sensor.h"
#include "vesc_app.h"
#include "vesc_config_compat.h"
#include "vesc_buffer.h"
#include "config.h"
#include "motor_current_cal.h"

mc_configuration motorConfLeft, motorConfRight;
MotorRuntimeConfig motorConfigLeft, motorConfigRight;
MotorSensorState motorSensorStateLeft, motorSensorStateRight;
VescAppConfig vescAppConfig;
static bool armed;
static int save_count;
static int encoder_mode_calls;

void MotorRuntimeConfig_GetHallSequence(const MotorRuntimeConfig *cfg, uint8_t seq[6]) {
    (void)cfg;
    const uint8_t s[6] = {1, 3, 2, 6, 4, 5};
    memcpy(seq, s, sizeof(s));
}
void mc_foc_conf_prepare(mc_configuration *c) { (void)c; }
void MotorSensor_PrepareRuntime(const MotorRuntimeConfig *c, MotorSensorState *s, uint8_t p) {
    (void)c; (void)s; (void)p;
}
void LeftEncoder_SetMode(bool on) { encoder_mode_calls += on ? 1 : 2; }
bool RuntimeSettings_Save(void) { save_count++; return true; }
bool RuntimeControl_Armed(void) { return armed; }
bool VescApp_AdcControlSupported(uint8_t type) {
    return type == VESC_ADC_NONE || type == VESC_ADC_CURRENT ||
           type == VESC_ADC_CURRENT_REV_CENTER ||
           type == VESC_ADC_CURRENT_NOREV_BRAKE_ADC ||
           type == VESC_ADC_DUTY || type == VESC_ADC_DUTY_REV_CENTER ||
           type == VESC_ADC_PID || type == VESC_ADC_PID_REV_CENTER;
}
void VescApp_ResetRuntime(void) { }
void MotorControl_GetCurrentOffsetDebug(MotorCurrentOffsetDebug *out) {
    memset(out, 0, sizeof(*out));
    out->state = MOTOR_CURRENT_CAL_VALID;
    out->valid = 1U;
    out->adc1_hw_cal_ok = 1U;
    out->adc2_hw_cal_ok = 1U;
    out->offset[0] = 2011U; out->offset[1] = 2022U;
    out->offset[2] = 2033U; out->offset[3] = 2044U;
    out->offset[4] = 2055U; out->offset[5] = 2066U;
}

static void defaults(void) {
    memset(&motorConfLeft, 0, sizeof(motorConfLeft));
    memset(&motorConfRight, 0, sizeof(motorConfRight));
    memset(&motorConfigLeft, 0, sizeof(motorConfigLeft));
    memset(&motorConfigRight, 0, sizeof(motorConfigRight));
    memset(&vescAppConfig, 0, sizeof(vescAppConfig));
    motorConfLeft.foc_current_units_per_amp = motorConfRight.foc_current_units_per_amp = 800U;
    motorConfLeft.l_current_max = motorConfRight.l_current_max = 12000;
    motorConfLeft.l_current_min = motorConfRight.l_current_min = -12000;
    motorConfLeft.l_max_speed_rpm_q4 = motorConfRight.l_max_speed_rpm_q4 = 16000;
    motorConfLeft.foc_motor_pole_pairs = motorConfRight.foc_motor_pole_pairs = 15U;
    motorConfLeft.foc_current_kp_q16 = motorConfRight.foc_current_kp_q16 = 16384;
    motorConfLeft.foc_current_ki_q16 = motorConfRight.foc_current_ki_q16 = 15728640;
    motorConfigLeft.sensor_type = MOTOR_SENSOR_HALL_UVW;
    motorConfigRight.sensor_type = MOTOR_SENSOR_HALL_UVW;
    motorConfigLeft.encoder_cpr = 800U;
    motorConfigRight.encoder_cpr = 800U;
    vescAppConfig.controller_id = 10U;
    vescAppConfig.timeout_ms = 1000U;
    vescAppConfig.app_to_use = VESC_APP_UART;
    vescAppConfig.uart_baud = USART3_BAUD;
    vescAppConfig.update_rate_hz = 500U;
    armed = false; save_count = 0; encoder_mode_calls = 0;
}

int main(void) {
    uint8_t mc[700], app[700];
    defaults();
    const int32_t mlen = VescConfig_SerializeMc(mc, false, false);
    const int32_t alen = VescConfig_SerializeApp(app, false, false);
    assert(mlen == 481);
    assert(alen == 493);
    assert(((uint32_t)mc[0] << 24 | (uint32_t)mc[1] << 16 | (uint32_t)mc[2] << 8 | mc[3]) == VESC6_MCCONF_SIGNATURE);
    assert(((uint32_t)app[0] << 24 | (uint32_t)app[1] << 16 | (uint32_t)app[2] << 8 | app[3]) == VESC6_APPCONF_SIGNATURE);
    /* Python one-shot bench tester safely patches only these two VESC6 wire
       bytes when it temporarily exercises APP_UART/APP_ADC/APP_ADC_UART. */
    assert(app[33] == VESC_APP_UART);
    assert(app[90] == VESC_ADC_NONE);

    /* V17 unit/config contract. Stock hoverboard configuration is 15 pole-pairs
       (30 magnetic poles), 15 A normal motor current and 1000 mechanical RPM,
       which is 15000 electrical RPM. The VESC wire absolute-current field is a
       separate compatibility limit and slow ABS must be disabled. */
    int32_t wi = 0;
    assert(vesc_buf_get_u32(mc, &wi) == VESC6_MCCONF_SIGNATURE);
    wi += 4; /* pwm, comm, motor, legacy sensor */
    const float wire_motor_max = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_motor_min = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_input_max = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_input_min = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_abs_max = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_erpm_min = vesc_buf_get_float32_auto(mc, &wi);
    const float wire_erpm_max = vesc_buf_get_float32_auto(mc, &wi);
    assert(fabsf(wire_motor_max - 15.0f) < 0.02f);
    assert(fabsf(wire_motor_min + 15.0f) < 0.02f);
    assert(fabsf(wire_input_max - 15.0f) < 0.02f);
    assert(fabsf(wire_input_min + 15.0f) < 0.02f);
    assert(fabsf(wire_abs_max - 22.5f) < 0.05f);
    assert(fabsf(wire_erpm_min + 15000.0f) < 1.0f);
    assert(fabsf(wire_erpm_max - 15000.0f) < 1.0f);
    const float wire_erpm_start = vesc_buf_get_float16(mc, 10000.0f, &wi);
    assert(fabsf(wire_erpm_start - 0.8f) < 0.001f);
    for (int n = 0; n < 6; ++n) (void)vesc_buf_get_float32_auto(mc, &wi);
    assert(mc[wi++] == 0U); /* l_slow_abs_current */
    for (int n = 0; n < 4; ++n) (void)vesc_buf_get_float16(mc, 10.0f, &wi);
    for (int n = 0; n < 3; ++n) (void)vesc_buf_get_float16(mc, 10000.0f, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    for (int n = 0; n < 3; ++n) (void)vesc_buf_get_float16(mc, 10000.0f, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    (void)vesc_buf_get_float16(mc, 10.0f, &wi);
    (void)vesc_buf_get_float16(mc, 10000.0f, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    (void)vesc_buf_get_float32_auto(mc, &wi);
    const uint8_t expected_legacy_hall[8] = {255U,1U,3U,2U,5U,6U,4U,255U};
    assert(memcmp(&mc[wi], expected_legacy_hall, 8U) == 0);
    wi += 8;
    /* Regression: FOC angle values such as 50/183 must NEVER leak into the
       legacy BLDC Hall table, which is what triggered VESC Tool truncation. */
    for (int n = 1; n <= 6; ++n) assert(mc[wi - 8 + n] >= 1U && mc[wi - 8 + n] <= 6U);

    /* A same-value LEFT config write must be exactly idempotent on the wire.
       Hardware V3 exposed a regression where GET->SET->GET changed bytes. */
    uint8_t mc_after[700];
    assert(VescConfig_DeserializeMc(mc, (uint32_t)mlen, false, false));
    const int32_t mlen_after = VescConfig_SerializeMc(mc_after, false, false);
    assert(mlen_after == mlen);
    if (memcmp(mc, mc_after, (size_t)mlen) != 0) {
        for (int32_t n = 0; n < mlen; ++n) {
            if (mc[n] != mc_after[n]) {
                fprintf(stderr, "MCCONF_DIFF offset=%ld before=%u after=%u\n",
                        (long)n, (unsigned)mc[n], (unsigned)mc_after[n]);
                break;
            }
        }
        assert(0 && "same-value MCCONF roundtrip changed wire image");
    }
    assert(VescConfig_DeserializeMc(mc, (uint32_t)mlen, false, true));
    assert(save_count == 1);

    /* Repeat exact idempotence with the LEFT ABI/encoder configuration used on
       the real bench log (CPR 2048, 15 pole-pairs). */
    motorConfigLeft.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    motorConfigLeft.encoder_cpr = 2048U;
    motorConfigLeft.sensor_inverted = 0U;
    motorConfigLeft.encoder_offset_deg = 0U;
    motorConfigLeft.encoder_ratio = 14U; /* deliberately independent of physical pole-pairs */
    motorConfLeft.foc_motor_pole_pairs = 15U;
    const int32_t emlen = VescConfig_SerializeMc(mc, false, false);
    assert(emlen == 481);
    /* Regression: VESC Motor Poles and FOC Encoder Ratio are independent fields.
       A Tool write of Encoder Ratio must not rewrite the physical motor poles. */
    motorConfigLeft.encoder_ratio = 7U;
    motorConfLeft.foc_motor_pole_pairs = 12U;
    assert(VescConfig_DeserializeMc(mc, (uint32_t)emlen, false, false));
    assert(motorConfigLeft.encoder_ratio == 14U);
    assert(motorConfLeft.foc_motor_pole_pairs == 15U);
    const int32_t emlen2 = VescConfig_SerializeMc(mc_after, false, false);
    assert(emlen2 == emlen);
    if (memcmp(mc, mc_after, (size_t)emlen) != 0) {
        for (int32_t n = 0; n < emlen; ++n) if (mc[n] != mc_after[n]) {
            fprintf(stderr, "ENCODER_MCCONF_DIFF offset=%ld before=%u after=%u\n",
                    (long)n, (unsigned)mc[n], (unsigned)mc_after[n]);
            break;
        }
        assert(0 && "same-value encoder MCCONF roundtrip changed wire image");
    }

    /* RIGHT is a virtual VESC node but physically Hall-only, even if a Tool
       packet originated from a configuration that would otherwise select ABI. */
    motorConfigRight.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    assert(VescConfig_DeserializeMc(mc, (uint32_t)mlen, true, true));
    assert(motorConfigRight.sensor_type == MOTOR_SENSOR_HALL_UVW);

    /* App UART baud on the wire may vary, but the only physical VESC Tool port
       stays fixed at USART3_BAUD so the user cannot lock themselves out. */
    assert(VescConfig_DeserializeApp(app, (uint32_t)alen, false, true));
    assert(vescAppConfig.uart_baud == USART3_BAUD);

    /* Unsupported ADC button/expo/TC semantics must be reported and persisted
       as OFF, never accepted as cosmetic settings that do nothing. */
    vescAppConfig.buttons = 1U;
    vescAppConfig.throttle_exp_milli = 700;
    vescAppConfig.throttle_exp_brake_milli = -300;
    vescAppConfig.throttle_exp_mode = 2U;
    vescAppConfig.tc = 1U;
    vescAppConfig.tc_max_diff_milli = 5000U;
    const int32_t safe_alen = VescConfig_SerializeApp(app, false, false);
    assert(safe_alen == 493);
    assert(VescConfig_DeserializeApp(app, (uint32_t)safe_alen, false, false));
    assert(vescAppConfig.buttons == 0U);
    assert(vescAppConfig.throttle_exp_milli == 0);
    assert(vescAppConfig.throttle_exp_brake_milli == 0);
    assert(vescAppConfig.throttle_exp_mode == 0U);
    assert(vescAppConfig.tc == 0U);
    assert(vescAppConfig.tc_max_diff_milli == 0U);

    /* Configuration writes while energized are rejected before mutation. */
    armed = true;
    const uint8_t old_id = vescAppConfig.controller_id;
    assert(!VescConfig_DeserializeApp(app, (uint32_t)alen, false, true));
    assert(vescAppConfig.controller_id == old_id);
    assert(!VescConfig_DeserializeMc(mc, (uint32_t)mlen, false, true));

    puts("VESC_CONFIG_WIRE_TESTS_PASS");
    return 0;
}
