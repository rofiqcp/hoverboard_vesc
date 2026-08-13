/*
 * VESC Tool 6.00 wire-configuration compatibility.
 *
 * IMPORTANT:
 * - Wire layout follows vedderb/bldc tag 6.00 confgenerator.c byte-for-byte.
 * - Only parameters that exist on the hoverboard controller are applied.
 * - Unsupported VESC fields are serialized with conservative values and ignored
 *   when written back. They never enter the 16 kHz fast loop.
 * - RIGHT motor is hard constrained to Hall. LEFT may use Hall or ABI encoder.
 */
#include "vesc_config_compat.h"
#include "vesc_buffer.h"
#include "vesc_app.h"
#include "runtime_control.h"
#include "foc_motor.h"
#include "motor_sensor.h"
#include "left_encoder.h"
#include "config.h"
#include "motor_current_cal.h"
#include <math.h>
#include <string.h>

#define VESC_PWM_MODE_SYNCHRONOUS 1U
#define VESC_COMM_MODE_INTEGRATE  0U
#define VESC_MOTOR_TYPE_FOC       2U
#define VESC_SENSOR_MODE_SENSORED 1U
#define VESC_FOC_SENSOR_ENCODER   1U
#define VESC_FOC_SENSOR_HALL      2U
#define VESC_SENSOR_PORT_HALL     0U
#define VESC_SENSOR_PORT_ABI      1U

#define VESC6_MCCONF_WIRE_SIZE 481U
#define VESC6_APPCONF_WIRE_SIZE 493U

/* VESC Tool compatibility values. The hoverboard still enforces its physical
 * 15-A regulated motor-current limit and the independent 17-A DC-link hard chop.
 * This is only the VESC l_abs_current_max field, which should sit above the
 * normal motor-current limit so normal regulation is not reported as an ABS
 * over-current configuration error. */
#define VESC_UI_ABS_CURRENT_MULTIPLIER 1.5f
#define VESC_ERPM_LIMIT_START 0.8f

static float q16_to_float(int32_t v) { return (float)v / 65536.0f; }
static int32_t float_to_q16(float v) {
    if (v > 32767.0f) v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int32_t)lrintf(v * 65536.0f);
}
static float q15_to_float(uint16_t v) { return (float)v / 32767.0f; }
static uint16_t float_to_q15(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return (uint16_t)lrintf(v * 32767.0f);
}
static float current_to_amp(const mc_configuration *c, int32_t raw) {
    const uint16_t units = c->foc_current_units_per_amp ? c->foc_current_units_per_amp : 1U;
    return (float)raw / (float)units;
}
static int16_t amp_to_current(const mc_configuration *c, float amp) {
    const uint16_t units = c->foc_current_units_per_amp ? c->foc_current_units_per_amp : 1U;
    float raw = amp * (float)units;
    if (raw > 32767.0f) raw = 32767.0f;
    if (raw < -32768.0f) raw = -32768.0f;
    return (int16_t)lrintf(raw);
}
static float max_erpm(const mc_configuration *c) {
    return ((float)c->l_max_speed_rpm_q4 / 16.0f) * (float)c->foc_motor_pole_pairs;
}
static void append_auto(uint8_t *b, float v, int32_t *i) { vesc_buf_append_float32_auto(b, v, i); }
static void append_f16(uint8_t *b, float v, float scale, int32_t *i) { vesc_buf_append_float16(b, v, scale, i); }
static float get_auto(const uint8_t *b, int32_t *i) { return vesc_buf_get_float32_auto(b, i); }
static float get_f16(const uint8_t *b, float scale, int32_t *i) { return vesc_buf_get_float16(b, scale, i); }

/* VESC 6.00 has TWO unrelated Hall tables on the mcconf wire:
 *   - legacy BLDC hall_table[8] is int8_t commutation state (-1,1..6)
 *   - FOC foc_hall_table[8] is uint8_t electrical angle (0..200, 255 invalid)
 * V1-V16 accidentally serialized the FOC-angle table into both fields. Values
 * such as 50/183/116 are valid FOC angles but illegal legacy BLDC states, which
 * is exactly why VESC Tool displayed "Parameters truncated" after Hall detect. */
static void hall_table_legacy_vesc(uint8_t out[8]) {
    static const uint8_t default_bldc[8] = {
        255U, 1U, 3U, 2U, 5U, 6U, 4U, 255U
    }; /* 255 on wire == int8_t -1 */
    memcpy(out, default_bldc, sizeof(default_bldc));
}

static void hall_table_foc_vesc(const MotorRuntimeConfig *cfg, uint8_t out[8]) {
    for (uint8_t n = 0; n < 8U; ++n) out[n] = 255U;
    uint8_t seq[6];
    MotorRuntimeConfig_GetHallSequence(cfg, seq);
    for (uint8_t sector = 0; sector < 6U; ++sector) {
        const uint8_t raw = seq[sector] & 7U;
        if (raw > 0U && raw < 7U) {
            /* VESC FOC Hall table uses 0..200 for 0..360 electrical degrees;
             * 255 is reserved for invalid/unseen 000/111 Hall states. */
            out[raw] = (uint8_t)(((uint32_t)(2U * sector + 1U) * 200U) / 12U);
        }
    }
}

int32_t VescConfig_SerializeMc(uint8_t *b, bool right, bool defaults) {
    (void)defaults;
    if (b == NULL) return 0;
    int32_t i = 0;
    const mc_configuration *c = right ? &motorConfRight : &motorConfLeft;
    const MotorRuntimeConfig *r = right ? &motorConfigRight : &motorConfigLeft;
    uint8_t legacy_hall[8];
    uint8_t foc_hall[8];
    hall_table_legacy_vesc(legacy_hall);
    hall_table_foc_vesc(r, foc_hall);

    vesc_buf_append_u32(b, VESC6_MCCONF_SIGNATURE, &i);
    b[i++] = VESC_PWM_MODE_SYNCHRONOUS;
    b[i++] = VESC_COMM_MODE_INTEGRATE;
    b[i++] = VESC_MOTOR_TYPE_FOC;
    b[i++] = VESC_SENSOR_MODE_SENSORED;

    append_auto(b, current_to_amp(c, c->l_current_max), &i); /* l_current_max */
    append_auto(b, current_to_amp(c, c->l_current_min), &i); /* l_current_min */
    append_auto(b, current_to_amp(c, c->l_current_max), &i); /* l_in_current_max */
    append_auto(b, current_to_amp(c, c->l_current_min), &i); /* l_in_current_min */
    append_auto(b, current_to_amp(c, c->l_current_max) * VESC_UI_ABS_CURRENT_MULTIPLIER, &i); /* l_abs_current_max */
    append_auto(b, -max_erpm(c), &i);
    append_auto(b, max_erpm(c), &i);
    append_f16(b, VESC_ERPM_LIMIT_START, 10000.0f, &i); /* l_erpm_start: VESC default */
    append_auto(b, max_erpm(c), &i);
    append_auto(b, max_erpm(c), &i);
    append_auto(b, 10.0f, &i);  /* min vin */
    append_auto(b, 60.0f, &i);  /* max vin */
    append_auto(b, 36.0f, &i);  /* battery cut start */
    append_auto(b, 32.0f, &i);  /* battery cut end */
    b[i++] = 0U; /* l_slow_abs_current: VESC recommends the unfiltered fast ABS check */
    append_f16(b, 80.0f, 10.0f, &i);
    append_f16(b, 100.0f, 10.0f, &i);
    append_f16(b, 80.0f, 10.0f, &i);
    append_f16(b, 100.0f, 10.0f, &i);
    append_f16(b, 0.15f, 10000.0f, &i);
    append_f16(b, 0.0f, 10000.0f, &i);
    append_f16(b, 0.95f, 10000.0f, &i);
    append_auto(b, 5000.0f, &i);
    append_auto(b, -5000.0f, &i);
    append_f16(b, 1.0f, 10000.0f, &i);
    append_f16(b, 1.0f, 10000.0f, &i);
    append_f16(b, 0.0f, 10000.0f, &i);

    /* Legacy sensorless/BLDC parameters. */
    append_auto(b, 250.0f, &i);
    append_auto(b, 250.0f, &i);
    append_auto(b, 10.0f, &i);
    append_f16(b, 0.0f, 10.0f, &i);
    append_f16(b, 0.0f, 10000.0f, &i);
    append_auto(b, 1000.0f, &i);
    append_auto(b, 0.0f, &i);
    for (uint8_t n = 0; n < 8U; ++n) b[i++] = legacy_hall[n];
    append_auto(b, 2000.0f, &i);

    /* FOC parameters. */
    append_auto(b, q16_to_float(c->foc_current_kp_q16), &i);
    append_auto(b, q16_to_float(c->foc_current_ki_q16), &i);
    append_auto(b, (float)PWM_FREQ, &i);
    append_auto(b, 1000000.0f / (float)PWM_FREQ, &i);
    b[i++] = (!right && r->sensor_inverted) ? 1U : 0U;
    append_auto(b, right ? 0.0f : (float)r->encoder_offset_deg, &i);
    append_auto(b, (float)(r->encoder_ratio != 0U ? r->encoder_ratio : c->foc_motor_pole_pairs), &i);
    b[i++] = (!right && r->sensor_type == MOTOR_SENSOR_ENCODER_AB) ? VESC_FOC_SENSOR_ENCODER : VESC_FOC_SENSOR_HALL;
    append_auto(b, 2000.0f, &i);  /* pll kp */
    append_auto(b, 30000.0f, &i); /* pll ki */
    append_auto(b, 20e-6f, &i);   /* motor L */
    append_auto(b, (float)c->foc_motor_ld_lq_diff_uh * 1e-6f, &i);
    append_auto(b, 0.05f, &i);    /* motor R */
    append_auto(b, (float)c->foc_motor_flux_linkage_uwb * 1e-6f, &i);
    append_auto(b, 1000.0f, &i);
    append_auto(b, 1000.0f, &i);
    append_f16(b, 0.0f, 1000.0f, &i);
    append_auto(b, 0.0f, &i);
    append_auto(b, 0.0f, &i);
    append_f16(b, 0.0f, 10000.0f, &i);
    append_auto(b, 0.0f, &i);
    append_auto(b, 0.0f, &i);
    append_f16(b, 0.0f, 1000.0f, &i); /* openloop rpm low */
    append_f16(b, 0.0f, 1000.0f, &i); /* d gain scale start */
    append_f16(b, 0.0f, 1000.0f, &i); /* d gain scale max */
    append_f16(b, 0.0f, 100.0f, &i);  /* sl hyst */
    append_f16(b, 0.0f, 100.0f, &i);  /* lock */
    append_f16(b, 0.0f, 100.0f, &i);  /* ramp */
    append_f16(b, 0.0f, 100.0f, &i);  /* openloop time */
    append_f16(b, 0.0f, 100.0f, &i);  /* boost q */
    append_f16(b, 0.0f, 100.0f, &i);  /* max q */
    for (uint8_t n = 0; n < 8U; ++n) b[i++] = foc_hall[n];
    append_auto(b, 500.0f, &i); /* hall interpolation ERPM */
    append_auto(b, 0.0f, &i);   /* sensorless transition */
    b[i++] = 1U; /* sample v0/v7 */
    b[i++] = 0U;
    b[i++] = 0U;
    append_f16(b, 0.0f, 1000.0f, &i);
    b[i++] = 0U;
    append_f16(b, 25.0f, 100.0f, &i);
    append_f16(b, 0.1f, 10000.0f, &i);
    b[i++] = 0U;
    b[i++] = 0U;
    append_f16(b, 0.0f, 10.0f, &i);
    append_f16(b, 0.0f, 10.0f, &i);
    append_f16(b, 0.0f, 10.0f, &i);
    append_f16(b, 0.0f, 1000.0f, &i);
    append_f16(b, 0.0f, 100.0f, &i);
    append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i);
    append_auto(b, 0.0f, &i);
    b[i++] = 0U; /* HFI samples */

    /* VESC 6.00 exposes foc_offsets_cal_on_boot plus three raw current offsets.
     * This board physically measures only two phase currents per inverter, so the
     * third VESC offset is reported as zero (reconstructed phase, no ADC channel).
     * DCL/DCR are separate DC-link sensors and are exposed in HBTS diagnostics. */
    MotorCurrentOffsetDebug cdbg;
    MotorControl_GetCurrentOffsetDebug(&cdbg);
    b[i++] = 1U; /* foc_offsets_cal_on_boot: this port always recalibrates safely */
    const uint8_t ofs0 = right ? 2U : 0U;
    const uint8_t ofs1 = right ? 3U : 1U;
    append_auto(b, cdbg.valid ? (float)cdbg.offset[ofs0] : 0.0f, &i);
    append_auto(b, cdbg.valid ? (float)cdbg.offset[ofs1] : 0.0f, &i);
    append_auto(b, 0.0f, &i); /* no physical third phase-current ADC */
    for (uint8_t n = 0; n < 6U; ++n) append_f16(b, 0.0f, 10000.0f, &i);
    b[i++] = 0U;
    b[i++] = 0U;
    append_auto(b, 0.0f, &i);
    b[i++] = (uint8_t)c->foc_mtpa_mode;
    append_auto(b, current_to_amp(c, c->foc_fw_current_max), &i);
    append_f16(b, q15_to_float(c->foc_fw_duty_start_q15), 10000.0f, &i);
    append_f16(b, (float)c->foc_fw_ramp_time_ms / 1000.0f, 1000.0f, &i);
    append_f16(b, q15_to_float(c->foc_fw_q_current_factor_q15), 10000.0f, &i);
    b[i++] = 0U; /* corrected speed source */

    /* General-purpose drive fields. */
    vesc_buf_append_i16(b, 0, &i);
    vesc_buf_append_i16(b, 0, &i);
    append_f16(b, 0.0f, 10000.0f, &i);
    append_auto(b, 0.0f, &i);
    append_auto(b, 0.0f, &i);
    b[i++] = 4U; /* speed PID loop rate */

    append_auto(b, q16_to_float(c->s_pid_kp_q16), &i);
    append_auto(b, q16_to_float(c->s_pid_ki_q16), &i);
    append_auto(b, q16_to_float(c->s_pid_kd_q16), &i);
    append_f16(b, q15_to_float(c->s_pid_kd_filter_q15), 10000.0f, &i);
    append_auto(b, (float)c->s_pid_min_erpm, &i);
    b[i++] = c->s_pid_allow_braking ? 1U : 0U;
    append_auto(b, (float)c->s_pid_ramp_erpms_s, &i);

    append_auto(b, q16_to_float(c->p_pid_kp_q16), &i);
    append_auto(b, q16_to_float(c->p_pid_ki_q16), &i);
    append_auto(b, q16_to_float(c->p_pid_kd_q16), &i);
    append_auto(b, q16_to_float(c->p_pid_kd_proc_q16), &i);
    append_f16(b, q15_to_float(c->p_pid_kd_filter_q15), 10000.0f, &i);
    append_auto(b, 1.0f, &i);
    append_f16(b, 0.0f, 10.0f, &i);
    append_auto(b, 0.0f, &i);

    /* Current-control/startup and generic motor fields. */
    append_f16(b, 0.0f, 10000.0f, &i);
    append_auto(b, 0.0f, &i);
    append_auto(b, 1.0f, &i);
    append_f16(b, 0.01f, 10000.0f, &i);
    vesc_buf_append_i32(b, 500, &i);
    append_f16(b, 0.02f, 10000.0f, &i);
    append_auto(b, 1.0f, &i);
    vesc_buf_append_u32(b, (!right && r->sensor_type == MOTOR_SENSOR_ENCODER_AB) ? r->encoder_cpr : 0U, &i);
    for (uint8_t n = 0; n < 6U; ++n) append_f16(b, 0.0f, 1000.0f, &i);
    b[i++] = (!right && r->sensor_type == MOTOR_SENSOR_ENCODER_AB) ? VESC_SENSOR_PORT_ABI : VESC_SENSOR_PORT_HALL;
    b[i++] = r->motor_inverted ? 1U : 0U;
    b[i++] = 0U;
    b[i++] = 0U;
    append_auto(b, (float)PWM_FREQ, &i);
    append_auto(b, (float)PWM_FREQ, &i);
    append_auto(b, (float)PWM_FREQ, &i);
    append_auto(b, 3435.0f, &i);
    b[i++] = 0U;
    b[i++] = 8U; /* motor temp sensor disabled */
    append_auto(b, 1.0f, &i);
    append_f16(b, 10000.0f, 0.1f, &i);
    append_f16(b, 25.0f, 10.0f, &i);
    b[i++] = 0U;
    b[i++] = 8U;
    /* VESC si_motor_poles is TOTAL magnetic poles. Stock hoverboard has
     * 15 pole-pairs, therefore VESC Tool must show 30 motor poles. */
    b[i++] = (uint8_t)(c->foc_motor_pole_pairs * 2U);
    append_auto(b, 1.0f, &i);
    append_auto(b, 0.1f, &i);
    b[i++] = 0U;
    b[i++] = 10U;
    append_auto(b, 10.0f, &i);
    append_auto(b, 1.0f, &i);
    b[i++] = 0U;
    b[i++] = 0U;
    append_f16(b, 60.0f, 100.0f, &i);
    append_f16(b, 80.0f, 100.0f, &i);
    append_f16(b, 0.8f, 1000.0f, &i);
    append_f16(b, 0.9f, 1000.0f, &i);
    b[i++] = 0U;

    return (i == (int32_t)VESC6_MCCONF_WIRE_SIZE) ? i : 0;
}

bool VescConfig_DeserializeMc(const uint8_t *b, uint32_t len, bool right, bool store) {
    if (b == NULL || len != VESC6_MCCONF_WIRE_SIZE) return false;
    if (store && RuntimeControl_Armed()) return false;
    /* Exact same-value writes from VESC Tool must be idempotent. This avoids
     * needless float->fixed quantization and never revokes calibration proof
     * when the requested wire image is already the active one. */
    uint8_t active_wire[VESC6_MCCONF_WIRE_SIZE];
    if (VescConfig_SerializeMc(active_wire, right, false) == (int32_t)VESC6_MCCONF_WIRE_SIZE &&
        memcmp(active_wire, b, VESC6_MCCONF_WIRE_SIZE) == 0) {
        return !store || RuntimeSettings_Save();
    }
    int32_t i = 0;
    if (vesc_buf_get_u32(b, &i) != VESC6_MCCONF_SIGNATURE) return false;

    mc_configuration *c = right ? &motorConfRight : &motorConfLeft;
    MotorRuntimeConfig *r = right ? &motorConfigRight : &motorConfigLeft;

    i += 4; /* pwm, comm, motor, legacy sensor mode */
    const float current_max = get_auto(b, &i);
    const float current_min = get_auto(b, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i); (void)get_auto(b, &i);
    (void)get_auto(b, &i);
    float erpm_max = get_auto(b, &i);
    (void)get_f16(b, 10000.0f, &i);
    for (uint8_t n = 0; n < 6U; ++n) (void)get_auto(b, &i);
    i += 1; /* slow abs current */
    for (uint8_t n = 0; n < 7U; ++n) (void)get_f16(b, (n < 4U) ? 10.0f : 10000.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    for (uint8_t n = 0; n < 3U; ++n) (void)get_f16(b, 10000.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i); (void)get_auto(b, &i);
    (void)get_f16(b, 10.0f, &i); (void)get_f16(b, 10000.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    i += 8; (void)get_auto(b, &i);

    const float current_kp = get_auto(b, &i);
    const float current_ki = get_auto(b, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    const uint8_t encoder_inverted = b[i++];
    float encoder_offset = get_auto(b, &i);
    float encoder_ratio = get_auto(b, &i);
    uint8_t foc_sensor_mode = b[i++];
    (void)get_auto(b, &i); (void)get_auto(b, &i); /* PLL */
    (void)get_auto(b, &i); /* L */
    const float ld_lq_diff = get_auto(b, &i);
    (void)get_auto(b, &i); /* R */
    const float flux = get_auto(b, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    (void)get_f16(b, 1000.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    (void)get_f16(b, 10000.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    for (uint8_t n = 0; n < 3U; ++n) (void)get_f16(b, 1000.0f, &i);
    for (uint8_t n = 0; n < 6U; ++n) (void)get_f16(b, 100.0f, &i);
    i += 8;
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    i += 3;
    (void)get_f16(b, 1000.0f, &i); i += 1;
    (void)get_f16(b, 100.0f, &i); (void)get_f16(b, 10000.0f, &i); i += 2;
    for (uint8_t n = 0; n < 3U; ++n) (void)get_f16(b, 10.0f, &i);
    (void)get_f16(b, 1000.0f, &i); (void)get_f16(b, 100.0f, &i);
    (void)get_auto(b, &i); (void)vesc_buf_get_u16(b, &i); (void)get_auto(b, &i);
    i += 1; /* HFI samples */
    const uint8_t requested_offsets_on_boot = b[i++];
    (void)requested_offsets_on_boot; /* safety policy: calibration is always enabled */
    /* Manual VESC current-offset writes are deliberately ignored. The values
     * remain visible in GET_MCCONF, while the only writer is the zero-current
     * averaging routine with both bridges released. */
    (void)get_auto(b, &i); (void)get_auto(b, &i); (void)get_auto(b, &i);
    for (uint8_t n = 0; n < 6U; ++n) (void)get_f16(b, 10000.0f, &i);
    i += 2; (void)get_auto(b, &i);

    const uint8_t mtpa_mode = b[i++];
    const float fw_current = get_auto(b, &i);
    const float fw_duty = get_f16(b, 10000.0f, &i);
    const float fw_ramp = get_f16(b, 1000.0f, &i);
    const float fw_q_factor = get_f16(b, 10000.0f, &i);
    i += 1; /* speed source */
    (void)vesc_buf_get_i16(b, &i); (void)vesc_buf_get_i16(b, &i);
    (void)get_f16(b, 10000.0f, &i); (void)get_auto(b, &i); (void)get_auto(b, &i); i += 1;

    const float s_kp = get_auto(b, &i);
    const float s_ki = get_auto(b, &i);
    const float s_kd = get_auto(b, &i);
    const float s_kd_filter = get_f16(b, 10000.0f, &i);
    const float s_min_erpm = get_auto(b, &i);
    const uint8_t s_allow_brake = b[i++];
    const float s_ramp = get_auto(b, &i);
    const float p_kp = get_auto(b, &i);
    const float p_ki = get_auto(b, &i);
    const float p_kd = get_auto(b, &i);
    const float p_kd_proc = get_auto(b, &i);
    const float p_kd_filter = get_f16(b, 10000.0f, &i);
    (void)get_auto(b, &i); (void)get_f16(b, 10.0f, &i); (void)get_auto(b, &i);

    (void)get_f16(b, 10000.0f, &i); (void)get_auto(b, &i); (void)get_auto(b, &i);
    (void)get_f16(b, 10000.0f, &i); (void)vesc_buf_get_i32(b, &i);
    (void)get_f16(b, 10000.0f, &i); (void)get_auto(b, &i);
    const uint32_t encoder_counts = vesc_buf_get_u32(b, &i);
    for (uint8_t n = 0; n < 6U; ++n) (void)get_f16(b, 1000.0f, &i);
    const uint8_t sensor_port = b[i++];
    const uint8_t invert_direction = b[i++];
    i += 2;
    for (uint8_t n = 0; n < 4U; ++n) (void)get_auto(b, &i);
    i += 2; (void)get_auto(b, &i); (void)get_f16(b, 0.1f, &i); (void)get_f16(b, 10.0f, &i);
    i += 2;
    const uint8_t motor_poles_wire = b[i++];
    (void)get_auto(b, &i); (void)get_auto(b, &i); i += 2; (void)get_auto(b, &i); (void)get_auto(b, &i);
    i += 2; for (uint8_t n = 0; n < 4U; ++n) (void)get_f16(b, n < 2U ? 100.0f : 1000.0f, &i); i += 1;
    if (i != (int32_t)VESC6_MCCONF_WIRE_SIZE) return false;

    if (!isfinite(current_max) || !isfinite(current_min) || !isfinite(erpm_max) ||
        !isfinite(current_kp) || !isfinite(current_ki) || !isfinite(encoder_ratio)) return false;
    if (motor_poles_wire < 2U || motor_poles_wire > 120U || (motor_poles_wire & 1U) != 0U) {
        /* VESC's si_motor_poles is total magnetic poles. This fixed-point core
         * intentionally supports an integer pole-pair count, so odd total-pole
         * values are not physically representable. */
        return false;
    }

    c->l_current_max = amp_to_current(c, fabsf(current_max));
    c->l_current_min = amp_to_current(c, -fabsf(current_min));
    if (erpm_max < 0.0f) erpm_max = -erpm_max;
    /* Apply the eRPM limit only AFTER the authoritative pole-pair ratio below
     * has been decoded. Otherwise changing Encoder Ratio in VESC Tool would use
     * the old ratio for this conversion and silently change max eRPM on readback. */
    c->foc_current_kp_q16 = float_to_q16(current_kp);
    c->foc_current_ki_q16 = float_to_q16(current_ki);
    if (isfinite(ld_lq_diff)) c->foc_motor_ld_lq_diff_uh = (int32_t)lrintf(ld_lq_diff * 1e6f);
    if (isfinite(flux) && flux >= 0.0f) c->foc_motor_flux_linkage_uwb = (uint32_t)fminf(4294967295.0f, flux * 1e6f);
    if (mtpa_mode <= (uint8_t)MTPA_MODE_IQ_MEASURED) c->foc_mtpa_mode = (mc_mtpa_mode)mtpa_mode;
    c->foc_fw_current_max = amp_to_current(c, fabsf(fw_current));
    c->foc_fw_duty_start_q15 = float_to_q15(fw_duty);
    c->foc_fw_ramp_time_ms = (uint16_t)fminf(65535.0f, fmaxf(0.0f, fw_ramp * 1000.0f));
    c->foc_fw_q_current_factor_q15 = float_to_q15(fw_q_factor);
    c->s_pid_kp_q16 = float_to_q16(s_kp);
    c->s_pid_ki_q16 = float_to_q16(s_ki);
    c->s_pid_kd_q16 = float_to_q16(s_kd);
    c->s_pid_kd_filter_q15 = float_to_q15(s_kd_filter);
    c->s_pid_min_erpm = (int32_t)fmaxf(0.0f, s_min_erpm);
    c->s_pid_allow_braking = s_allow_brake != 0U;
    c->s_pid_ramp_erpms_s = (int32_t)fmaxf(0.0f, s_ramp);
    c->p_pid_kp_q16 = float_to_q16(p_kp);
    c->p_pid_ki_q16 = float_to_q16(p_ki);
    c->p_pid_kd_q16 = float_to_q16(p_kd);
    c->p_pid_kd_proc_q16 = float_to_q16(p_kd_proc);
    c->p_pid_kd_filter_q15 = float_to_q15(p_kd_filter);

    if (right) {
        r->sensor_type = MOTOR_SENSOR_HALL_UVW;
        r->sensor_inverted = 0U;
        r->encoder_calibrated = 0U;
        r->encoder_sequence_valid = 0U;
    } else {
        const bool request_encoder = foc_sensor_mode == VESC_FOC_SENSOR_ENCODER || sensor_port == VESC_SENSOR_PORT_ABI;
        r->sensor_type = request_encoder ? MOTOR_SENSOR_ENCODER_AB : MOTOR_SENSOR_HALL_UVW;
        r->sensor_inverted = encoder_inverted ? 1U : 0U;
        r->motor_inverted = invert_direction ? 1U : 0U;
        while (encoder_offset < 0.0f) encoder_offset += 360.0f;
        while (encoder_offset >= 360.0f) encoder_offset -= 360.0f;
        r->encoder_offset_deg = (uint16_t)lrintf(encoder_offset);
        if (encoder_counts >= 4U && encoder_counts <= 65535U) r->encoder_cpr = (uint16_t)encoder_counts;
        if (encoder_ratio >= 1.0f && encoder_ratio <= 60.0f) {
            const long ratio_rounded = lrintf(encoder_ratio);
            if (fabsf(encoder_ratio - (float)ratio_rounded) > 0.001f) return false;
            r->encoder_ratio = (uint8_t)ratio_rounded;
        }
        LeftEncoder_SetMode(r->sensor_type == MOTOR_SENSOR_ENCODER_AB);
    }

    /* Physical motor poles and encoder electrical ratio are independent VESC
     * fields. For this board physical pole-pairs remain the authoritative eRPM
     * conversion for BOTH motors; LEFT encoder_ratio only scales A/B angle. */
    c->foc_motor_pole_pairs = (uint8_t)(motor_poles_wire / 2U);

    /* VESC limits and COMM_SET_RPM are electrical RPM. Internal sensor speed is
     * mechanical RPM*16, so preserve the wire eRPM limit using the FINAL ratio. */
    if (c->foc_motor_pole_pairs > 0U && erpm_max >= 1.0f) {
        float rpm_q4 = (erpm_max / (float)c->foc_motor_pole_pairs) * 16.0f;
        if (rpm_q4 > 32767.0f) rpm_q4 = 32767.0f;
        c->l_max_speed_rpm_q4 = (int16_t)lrintf(rpm_q4);
    }

    mc_foc_conf_prepare(c);
    MotorSensor_PrepareRuntime(r, right ? &motorSensorStateRight : &motorSensorStateLeft, c->foc_motor_pole_pairs);
    return !store || RuntimeSettings_Save();
}

int32_t VescConfig_SerializeApp(uint8_t *b, bool right, bool defaults) {
    (void)defaults;
    if (b == NULL) return 0;
    int32_t i = 0;
    vesc_buf_append_u32(b, VESC6_APPCONF_SIGNATURE, &i);
    b[i++] = right ? (uint8_t)(vescAppConfig.controller_id + 1U) : vescAppConfig.controller_id;
    vesc_buf_append_u32(b, vescAppConfig.timeout_ms, &i);
    append_auto(b, (float)vescAppConfig.timeout_brake_cA / 100.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); /* CAN status rate 1 */
    vesc_buf_append_u16(b, 0U, &i); /* CAN status rate 2 */
    b[i++] = 0U; b[i++] = 0U; b[i++] = 0U; /* status masks + CAN baud */
    b[i++] = 1U; /* pairing done */
    b[i++] = 1U; /* permanent UART */
    b[i++] = 0U; b[i++] = 0U; b[i++] = 0U; b[i++] = 0U;
    append_auto(b, 100000.0f, &i);
    b[i++] = 0U; b[i++] = 0U; b[i++] = 0U;
    b[i++] = vescAppConfig.app_to_use;

    /* PPM block, exact 6.00 footprint. */
    b[i++] = 0U;
    for (uint8_t n = 0; n < 5U; ++n) append_auto(b, 0.0f, &i);
    b[i++] = 0U; b[i++] = 1U;
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i);
    b[i++] = 0U;
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i);
    b[i++] = 0U; b[i++] = 0U;
    append_auto(b, 0.0f, &i);
    append_f16(b, 0.0f, 1.0f, &i);
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i);

    /* ADC app maps PA2 -> ADC1 and PA3 -> ADC2. */
    b[i++] = vescAppConfig.adc_ctrl_type;
    append_auto(b, (float)vescAppConfig.adc_hyst_mV / 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage_start_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage_end_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage_min_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage_max_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage_center_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage2_start_mV / 1000.0f, 1000.0f, &i);
    append_f16(b, (float)vescAppConfig.voltage2_end_mV / 1000.0f, 1000.0f, &i);
    b[i++] = vescAppConfig.use_filter; b[i++] = vescAppConfig.safe_start;
    b[i++] = 0U; /* No digital ADC-app button is wired on this board. */
    b[i++] = vescAppConfig.voltage_inverted; b[i++] = vescAppConfig.voltage2_inverted;
    /* ADC throttle expo is intentionally forced off. The fast app path is
     * fixed-point and this port does not emulate VESC's nonlinear expo modes.
     * Expose zero rather than presenting a setting that would have no effect. */
    append_auto(b, 0.0f, &i);
    append_auto(b, 0.0f, &i);
    b[i++] = 0U;
    append_auto(b, (float)vescAppConfig.ramp_time_pos_ms / 1000.0f, &i);
    append_auto(b, (float)vescAppConfig.ramp_time_neg_ms / 1000.0f, &i);
    b[i++] = vescAppConfig.multi_esc;
    b[i++] = 0U; /* ADC traction-control option unsupported on this single-MCU virtual-CAN facade. */
    append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, vescAppConfig.update_rate_hz, &i);
    vesc_buf_append_u32(b, USART3_BAUD, &i);

    /* Nunchuk exact footprint. */
    b[i++] = 0U;
    for (uint8_t n = 0; n < 6U; ++n) append_auto(b, 0.0f, &i);
    b[i++] = 0U; b[i++] = 0U; b[i++] = 0U;
    append_auto(b, 0.0f, &i); b[i++] = 0U;
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i);

    for (uint8_t n = 0; n < 10U; ++n) b[i++] = 0U; /* NRF */

    /* Balance block, exact 6.00 footprint. */
    b[i++] = 0U;
    for (uint8_t n = 0; n < 6U; ++n) append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); vesc_buf_append_u16(b, 0U, &i);
    for (uint8_t n = 0; n < 5U; ++n) append_auto(b, 0.0f, &i);
    for (uint8_t n = 0; n < 6U; ++n) vesc_buf_append_u16(b, 0U, &i);
    b[i++] = 0U;
    append_f16(b, 0.0f, 100.0f, &i); append_f16(b, 0.0f, 100.0f, &i); append_f16(b, 0.0f, 1000.0f, &i);
    append_f16(b, 0.0f, 100.0f, &i); append_f16(b, 0.0f, 100.0f, &i); append_auto(b, 0.0f, &i);
    append_f16(b, 0.0f, 100.0f, &i); append_f16(b, 0.0f, 100.0f, &i); append_auto(b, 0.0f, &i);
    append_f16(b, 0.0f, 100.0f, &i); append_auto(b, 0.0f, &i); vesc_buf_append_u16(b, 0U, &i);
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i); append_f16(b, 0.0f, 100.0f, &i);
    for (uint8_t n = 0; n < 4U; ++n) append_auto(b, 0.0f, &i);
    b[i++] = 0U;
    for (uint8_t n = 0; n < 6U; ++n) append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i);
    append_auto(b, 0.0f, &i); append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); vesc_buf_append_u16(b, 0U, &i);
    for (uint8_t n = 0; n < 12U; ++n) append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); append_auto(b, 0.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); vesc_buf_append_u16(b, 0U, &i);

    /* PAS exact footprint. */
    b[i++] = 0U; b[i++] = 0U;
    append_f16(b, 0.0f, 1000.0f, &i); append_f16(b, 0.0f, 10.0f, &i); append_f16(b, 0.0f, 10.0f, &i);
    b[i++] = 0U; vesc_buf_append_u16(b, 0U, &i); b[i++] = 0U;
    append_f16(b, 0.0f, 100.0f, &i); append_f16(b, 0.0f, 100.0f, &i); vesc_buf_append_u16(b, 0U, &i);

    /* IMU exact footprint. */
    b[i++] = 0U; b[i++] = 0U; b[i++] = 0U;
    for (uint8_t n = 0; n < 4U; ++n) append_f16(b, 0.0f, 1.0f, &i);
    vesc_buf_append_u16(b, 0U, &i); b[i++] = 0U;
    for (uint8_t n = 0; n < 13U; ++n) append_auto(b, 0.0f, &i);

    return (i == (int32_t)VESC6_APPCONF_WIRE_SIZE) ? i : 0;
}

bool VescConfig_DeserializeApp(const uint8_t *b, uint32_t len, bool right, bool store) {
    if (b == NULL || len != VESC6_APPCONF_WIRE_SIZE) return false;
    if (store && RuntimeControl_Armed()) return false;
    int32_t i = 0;
    if (vesc_buf_get_u32(b, &i) != VESC6_APPCONF_SIGNATURE) return false;
    const uint8_t controller_id = b[i++];
    const uint32_t timeout_ms = vesc_buf_get_u32(b, &i);
    const float timeout_brake = get_auto(b, &i);
    (void)vesc_buf_get_u16(b, &i); (void)vesc_buf_get_u16(b, &i);
    i += 9;
    (void)get_auto(b, &i);
    i += 3;
    const uint8_t app_to_use = b[i++];

    /* PPM exact skip. */
    i += 1;
    for (uint8_t n = 0; n < 5U; ++n) (void)get_auto(b, &i);
    i += 2;
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    i += 1;
    (void)get_auto(b, &i); (void)get_auto(b, &i);
    i += 2;
    (void)get_auto(b, &i); (void)get_f16(b, 1.0f, &i);
    (void)get_auto(b, &i); (void)get_auto(b, &i);

    const uint8_t ctrl = b[i++];
    const float hyst = get_auto(b, &i);
    const float vs = get_f16(b, 1000.0f, &i);
    const float ve = get_f16(b, 1000.0f, &i);
    const float vmin = get_f16(b, 1000.0f, &i);
    const float vmax = get_f16(b, 1000.0f, &i);
    const float vc = get_f16(b, 1000.0f, &i);
    const float v2s = get_f16(b, 1000.0f, &i);
    const float v2e = get_f16(b, 1000.0f, &i);
    const uint8_t use_filter = b[i++];
    const uint8_t safe_start = b[i++];
    const uint8_t buttons = b[i++];
    const uint8_t inv1 = b[i++];
    const uint8_t inv2 = b[i++];
    const float exp = get_auto(b, &i);
    const float exp_brake = get_auto(b, &i);
    const uint8_t exp_mode = b[i++];
    const float ramp_pos = get_auto(b, &i);
    const float ramp_neg = get_auto(b, &i);
    const uint8_t multi_esc = b[i++];
    const uint8_t tc = b[i++];
    const float tc_diff = get_auto(b, &i);
    const uint16_t update_hz = vesc_buf_get_u16(b, &i);
    const uint32_t uart_baud = vesc_buf_get_u32(b, &i);

    if (right) {
        /* App config is a controller-level resource. Virtual RIGHT accepts the
         * packet for VESC Tool compatibility, but cannot create a second UART or
         * second PA2/PA3 input. */
        return true;
    }
    if (controller_id > 253U || timeout_ms > 600000U) return false;
    if (!(app_to_use == VESC_APP_NONE || app_to_use == VESC_APP_ADC ||
          app_to_use == VESC_APP_UART || app_to_use == VESC_APP_ADC_UART)) return false;
    if (!VescApp_AdcControlSupported(ctrl) || update_hz == 0U || update_hz > 2000U) return false;
    (void)uart_baud; /* USART3 is the only VESC Tool port and remains fixed. */

    #define MV_CLAMP(x) ((uint16_t)fminf(3300.0f, fmaxf(0.0f, (x) * 1000.0f)))
    vescAppConfig.controller_id = controller_id;
    vescAppConfig.timeout_ms = timeout_ms;
    vescAppConfig.timeout_brake_cA = (int16_t)fminf(32767.0f, fmaxf(-32768.0f, timeout_brake * 100.0f));
    vescAppConfig.app_to_use = app_to_use;
    vescAppConfig.adc_ctrl_type = ctrl;
    vescAppConfig.adc_hyst_mV = MV_CLAMP(hyst);
    vescAppConfig.voltage_start_mV = MV_CLAMP(vs);
    vescAppConfig.voltage_end_mV = MV_CLAMP(ve);
    vescAppConfig.voltage_min_mV = MV_CLAMP(vmin);
    vescAppConfig.voltage_max_mV = MV_CLAMP(vmax);
    vescAppConfig.voltage_center_mV = MV_CLAMP(vc);
    vescAppConfig.voltage2_start_mV = MV_CLAMP(v2s);
    vescAppConfig.voltage2_end_mV = MV_CLAMP(v2e);
    vescAppConfig.use_filter = use_filter ? 1U : 0U;
    vescAppConfig.safe_start = safe_start ? 1U : 0U;
    (void)buttons;
    vescAppConfig.buttons = 0U;
    vescAppConfig.voltage_inverted = inv1 ? 1U : 0U;
    vescAppConfig.voltage2_inverted = inv2 ? 1U : 0U;
    /* Parse to preserve the exact VESC 6.00 wire footprint, but force unsupported
     * nonlinear throttle semantics off instead of silently pretending to apply them. */
    (void)exp;
    (void)exp_brake;
    (void)exp_mode;
    vescAppConfig.throttle_exp_milli = 0;
    vescAppConfig.throttle_exp_brake_milli = 0;
    vescAppConfig.throttle_exp_mode = 0U;
    vescAppConfig.ramp_time_pos_ms = (uint16_t)fminf(65535.0f, fmaxf(0.0f, ramp_pos * 1000.0f));
    vescAppConfig.ramp_time_neg_ms = (uint16_t)fminf(65535.0f, fmaxf(0.0f, ramp_neg * 1000.0f));
    vescAppConfig.multi_esc = multi_esc ? 1U : 0U;
    (void)tc;
    (void)tc_diff;
    vescAppConfig.tc = 0U;
    vescAppConfig.tc_max_diff_milli = 0U;
    vescAppConfig.update_rate_hz = update_hz > 500U ? 500U : update_hz;
    vescAppConfig.uart_baud = USART3_BAUD;
    #undef MV_CLAMP
    VescApp_ResetRuntime();

    return !store || RuntimeSettings_Save();
}
