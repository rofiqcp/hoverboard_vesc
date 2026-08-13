/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-point STM32F103 adaptation of the VESC-style FOC architecture.
 * VESC reference: Copyright 2016-2022 Benjamin Vedder.
 * Board firmware lineage: Copyright 2019-2020 Emanuel FERU.
 * See NOTICE.md and COPYING.
 */

#include "foc_motor.h"

#include <limits.h>
#include <string.h>

#define ONE_BY_SQRT3_Q14 9459
#define TWO_BY_SQRT3_Q14 18919
#define SQRT3_BY_2_Q14   14189
#define MODULATION_MAX_Q14 SQRT3_BY_2_Q14

/* Keep all products in the per-motor current loop inside signed int32. The outer
 * loops use int64 but are also bounded so malformed GUI/EEPROM gains cannot
 * overflow before saturation. */
#define FOC_CURRENT_KP_Q16_MAX 65536L
#define FOC_CURRENT_KI_DT_Q16_MAX 65535L
#define FOC_CURRENT_KI_Q16_MAX ((int32_t)(FOC_CURRENT_KI_DT_Q16_MAX * (int32_t)FOC_CONTROL_FREQUENCY_HZ))
#define FOC_OUTER_GAIN_Q16_MAX 1048576L    /* 16.0 */
/* Detect Hall/Encoder keeps conservative current-loop gains until current
 * polarity/sampling is proven, but V11 hardware logs showed Vd pinned at the old
 * 500-internal cap for almost the whole sweep (RIGHT mean 499.65/500) while a
 * requested 0.50 A only reached ~0.35 A. That cap limited field stiffness, not
 * current. V12 keeps the requested current unchanged and raises only voltage
 * headroom. V15 restores the V1 16-kHz-per-motor current loop; the extra
 * commissioning fast guard is no longer in the DMA hot path. */
#define FOC_COMMISSIONING_KP_Q16_MAX      3277L  /* 0.05 */
#define FOC_COMMISSIONING_KI_DT_Q16_MAX     82L  /* ~10/s at 8 kHz */
#define FOC_COMMISSIONING_VOLTAGE_MAX      2400  /* 16.7% of 14400 full scale */

static int16_t sat_s16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

static int16_t clamp_s16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return (int16_t)value;
}

static int32_t clamp_s32(int64_t value, int32_t min_value, int32_t max_value)
{
    if (value > (int64_t)max_value) return max_value;
    if (value < (int64_t)min_value) return min_value;
    return (int32_t)value;
}

static int16_t abs_s16_sat(int16_t value)
{
    if (value == INT16_MIN) return INT16_MAX;
    return value < 0 ? (int16_t)-value : value;
}

static int16_t mul_q14(int16_t a, int16_t b)
{
    int32_t product = (int32_t)a * (int32_t)b;
    /* Symmetric round-to-nearest. Avoid the negative bias of adding a signed
     * half-LSB before an implementation-defined arithmetic right shift. */
    if (product >= 0) {
        product = (product + (1 << 13)) >> 14;
    } else {
        product = -(((-product) + (1 << 13)) >> 14);
    }
    return sat_s16(product);
}

static int16_t trig_q14(uint16_t phase)
{
    const uint8_t index = (uint8_t)(phase >> 8);
    const uint8_t fraction = (uint8_t)phase;
    const int16_t y0 = focSinTableQ14[index];
    const int16_t y1 = focSinTableQ14[(uint8_t)(index + 1U)];
    const int32_t delta = (int32_t)y1 - y0;
    return (int16_t)((int32_t)y0 + ((delta * fraction + 128) >> 8));
}

static void update_trig(motor_state_t *state)
{
    state->phase_sin = trig_q14(state->phase);
    state->phase_cos = trig_q14((uint16_t)(state->phase + 16384U));
}

static void reset_pid(mc_pid_state_t *pid)
{
    memset(pid, 0, sizeof(*pid));
}

static void reset_current_integrators(motor_state_t *state)
{
    state->vd_int_q16 = 0;
    state->vq_int_q16 = 0;
    reset_pid(&state->pid_d);
    reset_pid(&state->pid_q);
}

void mc_foc_conf_set_defaults(mc_configuration *conf,
                              mc_foc_current_sample_mode current_sample_mode)
{
    if (conf == NULL) return;
    memset(conf, 0, sizeof(*conf));

    /* Stock-board current unit is locked to the EFeru hardware scale. */
    conf->l_current_max = 12000;
    conf->l_current_min = -12000;
    conf->l_max_voltage = 14400;
    conf->l_max_speed_rpm_q4 = 16000;
    conf->foc_current_units_per_amp = CONTROL_CURRENT_INTERNAL_PER_A;

    conf->foc_current_kp_q16 = 16384;       /* 0.25 */
    conf->foc_current_ki_q16 = 15728640;    /* 240.0/s */

    /* VESC speed-PID structure uses electrical RPM and an internal 1/20
     * scaling. These defaults are deliberately conservative and preserve the
     * approximate mechanical-loop response of the previous board defaults at
     * the default 15 pole pairs. They are not universal motor tuning values. */
    conf->s_pid_kp_q16 = 175;               /* ~0.00267 before /20 */
    conf->s_pid_ki_q16 = 874;               /* ~0.01334 before /20 */
    conf->s_pid_kd_q16 = 0;
    conf->s_pid_kd_filter_q15 = 6553U;       /* 0.20 */
    conf->s_pid_min_erpm = 30;               /* 2 mechanical RPM at 15 pole pairs */
    conf->s_pid_ramp_erpms_s = 0;
    conf->s_pid_allow_braking = true;

    conf->p_pid_kp_q16 = 1024;        /* 0.015625: safe coarse-Hall default */
    conf->p_pid_ki_q16 = 0;
    conf->p_pid_kd_q16 = 0;
    conf->p_pid_kd_proc_q16 = 0;
    conf->p_pid_kd_filter_q15 = 6553U;       /* 0.20 */
    conf->p_pid_gain_dec_ticks = 8U;       /* taper near target; ~32 deg on 15-pp Hall */
    conf->p_pid_pos_min = -1000000;
    conf->p_pid_pos_max = 1000000;
    conf->p_pid_deadband_ticks = 2U;

    /* MTPA and FW are safe-off by default. Motor model must be entered before
     * MTPA can be enabled; FW current=0 is a hard disable. */
    conf->foc_motor_flux_linkage_uwb = 0U;
    conf->foc_motor_ld_lq_diff_uh = 0;
    conf->foc_mtpa_mode = MTPA_MODE_OFF;
    conf->foc_fw_current_max = 0;
    conf->foc_fw_duty_start_q15 = 29490U;    /* 0.90, irrelevant while max=0 */
    conf->foc_fw_ramp_time_ms = 500U;
    conf->foc_fw_q_current_factor_q15 = 0U;
    conf->foc_fw_backoff_q15 = 0U;

    conf->foc_motor_pole_pairs = 15U;
    conf->si_gear_ratio_milli = 1000U;
    conf->foc_current_sample_mode = current_sample_mode;
    mc_foc_conf_prepare(conf);
}

void mc_foc_conf_prepare(mc_configuration *conf)
{
    if (conf == NULL) return;
    if (conf->l_current_max < 1) conf->l_current_max = 1;
    if (conf->l_current_max > 12000) conf->l_current_max = 12000; /* 15 A board limit */
    if (conf->l_current_min >= 0) conf->l_current_min = -1;
    if (conf->l_current_min < -12000) conf->l_current_min = -12000;
    if (conf->l_max_voltage < 1) conf->l_max_voltage = 1;
    if (conf->foc_motor_pole_pairs == 0U) conf->foc_motor_pole_pairs = 1U;
    if (conf->si_gear_ratio_milli == 0U || conf->si_gear_ratio_milli > 60000U)
        conf->si_gear_ratio_milli = 1000U;
    /* Physical current gain of the stock board. Do not let GUI/EEPROM mutate it. */
    conf->foc_current_units_per_amp = CONTROL_CURRENT_INTERNAL_PER_A;

    if (conf->foc_current_kp_q16 < 0) conf->foc_current_kp_q16 = 0;
    if (conf->foc_current_kp_q16 > FOC_CURRENT_KP_Q16_MAX)
        conf->foc_current_kp_q16 = FOC_CURRENT_KP_Q16_MAX;
    if (conf->foc_current_ki_q16 < 0) conf->foc_current_ki_q16 = 0;
    if (conf->foc_current_ki_q16 > FOC_CURRENT_KI_Q16_MAX)
        conf->foc_current_ki_q16 = FOC_CURRENT_KI_Q16_MAX;

    int32_t *outer_gains[] = {
        &conf->s_pid_kp_q16, &conf->s_pid_ki_q16, &conf->s_pid_kd_q16,
        &conf->p_pid_kp_q16, &conf->p_pid_ki_q16, &conf->p_pid_kd_q16,
        &conf->p_pid_kd_proc_q16
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(outer_gains) / sizeof(outer_gains[0])); ++i) {
        if (*outer_gains[i] < 0) *outer_gains[i] = 0;
        if (*outer_gains[i] > FOC_OUTER_GAIN_Q16_MAX)
            *outer_gains[i] = FOC_OUTER_GAIN_Q16_MAX;
    }

    if (conf->s_pid_kd_filter_q15 > FOC_Q15_ONE) conf->s_pid_kd_filter_q15 = FOC_Q15_ONE;
    if (conf->p_pid_kd_filter_q15 > FOC_Q15_ONE) conf->p_pid_kd_filter_q15 = FOC_Q15_ONE;
    if (conf->foc_fw_duty_start_q15 > FOC_Q15_ONE) conf->foc_fw_duty_start_q15 = FOC_Q15_ONE;
    if (conf->foc_fw_q_current_factor_q15 > FOC_Q15_ONE) conf->foc_fw_q_current_factor_q15 = FOC_Q15_ONE;
    if (conf->foc_fw_backoff_q15 > FOC_Q15_ONE) conf->foc_fw_backoff_q15 = FOC_Q15_ONE;
    if (conf->foc_fw_current_max < 0) conf->foc_fw_current_max = 0;
    if (conf->foc_fw_current_max > conf->l_current_max) conf->foc_fw_current_max = conf->l_current_max;
    if (conf->foc_mtpa_mode > MTPA_MODE_IQ_MEASURED) conf->foc_mtpa_mode = MTPA_MODE_OFF;

    conf->foc_current_ki_dt_q16 =
        (conf->foc_current_ki_q16 + (int32_t)(FOC_CONTROL_FREQUENCY_HZ / 2U)) /
        (int32_t)FOC_CONTROL_FREQUENCY_HZ;
    conf->foc_voltage_to_mod_q16 =
        (uint32_t)(((uint32_t)MODULATION_MAX_Q14 << 16) /
                   (uint16_t)conf->l_max_voltage);
    conf->foc_voltage_index_q16 =
        (uint32_t)(((uint32_t)64U << 16) /
                   (uint16_t)conf->l_max_voltage);

    /* lambda[µWb] / (Ld-Lq)[µH] has units A. Precompute in board current
     * units so the 16-kHz MTPA equation needs only integer sqrt/multiply. */
    conf->foc_mtpa_lambda_over_ld_lq_current = 0;
    if (conf->foc_motor_ld_lq_diff_uh != 0 && conf->foc_motor_flux_linkage_uwb != 0U) {
        const int64_t numerator = (int64_t)conf->foc_motor_flux_linkage_uwb *
                                  (int64_t)conf->foc_current_units_per_amp;
        int64_t ratio = numerator / (int64_t)conf->foc_motor_ld_lq_diff_uh;
        if (ratio > INT32_MAX) ratio = INT32_MAX;
        if (ratio < INT32_MIN) ratio = INT32_MIN;
        conf->foc_mtpa_lambda_over_ld_lq_current = (int32_t)ratio;
    }
}

void mc_foc_init(motor_all_state_t *motor, mc_configuration *conf)
{
    if (motor == NULL || conf == NULL) return;
    memset(motor, 0, sizeof(*motor));
    motor->m_conf = conf;
    motor->m_control_mode = CONTROL_MODE_NONE;
}

void mc_foc_reset_control(motor_all_state_t *motor)
{
    if (motor == NULL) return;
    reset_current_integrators(&motor->m_motor_state);
    reset_pid(&motor->m_speed_pid);
    reset_pid(&motor->m_pos_pid);
    motor->m_id_set = 0;
    motor->m_iq_set = 0;
    motor->m_i_fw_set = 0;
    motor->m_i_fw_set_q16 = 0;
    motor->m_voltage_q_set = 0;
    motor->m_duty_cycle_set_q15 = 0;
    motor->m_motor_state.id_target = 0;
    motor->m_motor_state.iq_target = 0;
    motor->m_motor_state.id_target_mtpa = 0;
    motor->m_motor_state.id_target_fw = 0;
}

void mc_foc_reset_outer_loops(motor_all_state_t *motor)
{
    if (motor == NULL) return;
    reset_pid(&motor->m_speed_pid);
    reset_pid(&motor->m_pos_pid);
}

void mc_foc_set_control_mode(motor_all_state_t *motor, mc_control_mode mode)
{
    if (motor == NULL) return;
    if (motor->m_control_mode == mode) return;
    motor->m_control_mode = mode;
    reset_pid(&motor->m_speed_pid);
    reset_pid(&motor->m_pos_pid);
    reset_current_integrators(&motor->m_motor_state);
    /* Match VESC FW mode-change behavior: ramp FW out instead of abruptly
     * dropping Id. foc_run_fw_fixed() handles the decay in the fast loop. */
}

void mc_foc_set_phase_override(motor_all_state_t *motor, bool active, uint16_t phase_q16)
{
    if (motor == NULL) return;
    motor->m_phase_override = active;
    motor->m_phase_now_override = phase_q16;
}

void mc_foc_set_current_commissioning(motor_all_state_t *motor, bool active)
{
    if (motor == NULL) return;
    motor->m_current_commissioning = active;
}

void mc_foc_set_voltage_override(motor_all_state_t *motor, bool active,
                                 int16_t vd, int16_t vq)
{
    if (motor == NULL) return;
    motor->m_voltage_override = active;
    motor->m_vd_override = vd;
    motor->m_vq_override = vq;
}

static int32_t lp_q16(int32_t state, int32_t sample, uint16_t alpha_q15)
{
    const int64_t delta = (int64_t)sample - state;
    return clamp_s32((int64_t)state + ((delta * alpha_q15) >> 15), INT32_MIN, INT32_MAX);
}

static uint64_t isqrt_u64(uint64_t value)
{
    uint64_t res = 0U;
    uint64_t bit = (uint64_t)1U << 62;
    while (bit > value) bit >>= 2;
    while (bit != 0U) {
        if (value >= res + bit) {
            value -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return res;
}

static int32_t abs_s32_sat(int32_t value)
{
    if (value == INT32_MIN) return INT32_MAX;
    return value < 0 ? -value : value;
}

static int32_t step_towards_i32(int32_t value, int32_t target, int32_t step)
{
    if (step <= 0 || value == target) return target;
    if (value < target) {
        const int64_t next = (int64_t)value + step;
        return next > target ? target : (int32_t)next;
    }
    const int64_t next = (int64_t)value - step;
    return next < target ? target : (int32_t)next;
}

/* VESC speed PID, fixed point. Error is electrical RPM and P/I/D use the
 * upstream 1/20 scaling. Integrator is updated after output calculation. */
int16_t mc_foc_run_pid_control_speed(motor_all_state_t *motor, uint32_t dt_ms)
{
    if (motor == NULL || motor->m_conf == NULL) return 0;
    mc_configuration *conf = motor->m_conf;
    mc_pid_state_t *pid = &motor->m_speed_pid;
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 100U) dt_ms = 100U;

    if (motor->m_control_mode != CONTROL_MODE_SPEED) {
        reset_pid(pid);
        return motor->m_iq_set;
    }

    if (conf->s_pid_ramp_erpms_s > 0) {
        int32_t step = (int32_t)(((int64_t)conf->s_pid_ramp_erpms_s * dt_ms + 999LL) / 1000LL);
        if (step < 1) step = 1;
        motor->m_speed_pid_set_rpm = step_towards_i32(
            motor->m_speed_pid_set_rpm, motor->m_speed_command_rpm, step);
    } else {
        motor->m_speed_pid_set_rpm = motor->m_speed_command_rpm;
    }

    const int32_t measured_erpm = ((int32_t)motor->m_speed_rpm_q4 * conf->foc_motor_pole_pairs) >> 4;
    const int32_t error = clamp_s32((int64_t)motor->m_speed_pid_set_rpm - measured_erpm,
                                    INT32_MIN, INT32_MAX);
    if (abs_s32_sat(motor->m_speed_pid_set_rpm) < conf->s_pid_min_erpm) {
        reset_pid(pid);
        pid->previous_error = error;
        pid->initialized = true;
        motor->m_iq_set = 0;
        return 0;
    }

    if (!pid->initialized) {
        pid->previous_error = error;
        pid->initialized = true;
    }

    const int32_t p_q16 = clamp_s32(((int64_t)error * conf->s_pid_kp_q16) / 20LL,
                                    INT32_MIN, INT32_MAX);
    const int64_t derr = (int64_t)error - pid->previous_error;
    int32_t d_raw_q16 = clamp_s32((derr * conf->s_pid_kd_q16 * 1000LL) /
                                  ((int64_t)dt_ms * 20LL), INT32_MIN, INT32_MAX);
    pid->derivative_filter_q16 = lp_q16(pid->derivative_filter_q16,
                                        d_raw_q16, conf->s_pid_kd_filter_q15);
    const int32_t d_q16 = pid->derivative_filter_q16;
    pid->previous_error = error;

    int32_t output_q16 = clamp_s32((int64_t)p_q16 + pid->integrator_q16 + d_q16,
                                   -65536, 65536);

    const int64_t di = ((int64_t)error * conf->s_pid_ki_q16 * dt_ms) / (20LL * 1000LL);
    pid->integrator_q16 = clamp_s32((int64_t)pid->integrator_q16 + di, -65536, 65536);
    if (conf->s_pid_ki_q16 == 0) pid->integrator_q16 = 0;

    if (!conf->s_pid_allow_braking) {
        if ((measured_erpm > 20 && output_q16 < 0) ||
            (measured_erpm < -20 && output_q16 > 0)) output_q16 = 0;
    }

    const int32_t imax = conf->l_current_max;
    motor->m_iq_set = sat_s16((output_q16 * imax) >> 16);
    pid->last_p = sat_s16(((int64_t)p_q16 * imax) >> 16);
    pid->last_i = sat_s16(((int64_t)pid->integrator_q16 * imax) >> 16);
    pid->last_d = sat_s16(((int64_t)d_q16 * imax) >> 16);
    pid->last_output = motor->m_iq_set;
    pid->anti_windup = (output_q16 == 65536 || output_q16 == -65536);
    return motor->m_iq_set;
}

/* VESC position PID structure adapted to signed multi-turn ticks. The VESC
 * circular angle-difference is intentionally replaced by linear tick error so
 * existing homing/position limits remain physically correct on this board. */
int16_t mc_foc_run_pid_control_pos(motor_all_state_t *motor, uint32_t dt_ms)
{
    if (motor == NULL || motor->m_conf == NULL) return 0;
    mc_configuration *conf = motor->m_conf;
    mc_pid_state_t *pid = &motor->m_pos_pid;
    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 100U) dt_ms = 100U;

    if (motor->m_control_mode != CONTROL_MODE_POS) {
        reset_pid(pid);
        pid->previous_process = motor->m_position_ticks;
        return motor->m_iq_set;
    }

    int32_t setpoint = motor->m_pos_pid_set;
    if (setpoint < conf->p_pid_pos_min) setpoint = conf->p_pid_pos_min;
    if (setpoint > conf->p_pid_pos_max) setpoint = conf->p_pid_pos_max;
    const int32_t error = clamp_s32((int64_t)setpoint - motor->m_position_ticks,
                                    INT32_MIN, INT32_MAX);

    if ((int64_t)error >= -(int64_t)conf->p_pid_deadband_ticks &&
        (int64_t)error <=  (int64_t)conf->p_pid_deadband_ticks) {
        reset_pid(pid);
        pid->previous_process = motor->m_position_ticks;
        motor->m_iq_set = 0;
        return 0;
    }

    if (!pid->initialized) {
        pid->previous_error = error;
        pid->previous_process = motor->m_position_ticks;
        pid->initialized = true;
    }

    int32_t kp = conf->p_pid_kp_q16;
    int32_t ki = conf->p_pid_ki_q16;
    int32_t kd = conf->p_pid_kd_q16;
    int32_t kd_proc = conf->p_pid_kd_proc_q16;
    const uint32_t error_abs = (uint32_t)abs_s32_sat(error);
    if (conf->p_pid_gain_dec_ticks > 0U && error_abs < conf->p_pid_gain_dec_ticks) {
        const uint32_t scale_q15 = (uint32_t)(((uint64_t)error_abs << 15) /
                                              conf->p_pid_gain_dec_ticks);
        kp = (int32_t)(((int64_t)kp * scale_q15) >> 15);
        ki = (int32_t)(((int64_t)ki * scale_q15) >> 15);
        kd = (int32_t)(((int64_t)kd * scale_q15) >> 15);
        kd_proc = (int32_t)(((int64_t)kd_proc * scale_q15) >> 15);
    }

    const int32_t p_q16 = clamp_s32((int64_t)error * kp, INT32_MIN, INT32_MAX);
    pid->integrator_q16 = clamp_s32((int64_t)pid->integrator_q16 +
        ((int64_t)error * ki * dt_ms) / 1000LL, INT32_MIN, INT32_MAX);

    if (UINT32_MAX - pid->derivative_dt_accum_ms < dt_ms) pid->derivative_dt_accum_ms = UINT32_MAX;
    else pid->derivative_dt_accum_ms += dt_ms;
    int32_t d_raw_q16 = 0;
    if (error != pid->previous_error && pid->derivative_dt_accum_ms != 0U) {
        d_raw_q16 = clamp_s32(((int64_t)error - pid->previous_error) * kd * 1000LL /
                              pid->derivative_dt_accum_ms, INT32_MIN, INT32_MAX);
        pid->derivative_dt_accum_ms = 0U;
    }
    pid->derivative_filter_q16 = lp_q16(pid->derivative_filter_q16,
                                        d_raw_q16, conf->p_pid_kd_filter_q15);

    if (UINT32_MAX - pid->process_dt_accum_ms < dt_ms) pid->process_dt_accum_ms = UINT32_MAX;
    else pid->process_dt_accum_ms += dt_ms;
    int32_t d_proc_raw_q16 = 0;
    if (motor->m_position_ticks != pid->previous_process && pid->process_dt_accum_ms != 0U) {
        d_proc_raw_q16 = clamp_s32(-((int64_t)motor->m_position_ticks - pid->previous_process) *
                                   kd_proc * 1000LL / pid->process_dt_accum_ms,
                                   INT32_MIN, INT32_MAX);
        pid->process_dt_accum_ms = 0U;
    }
    pid->process_derivative_filter_q16 = lp_q16(pid->process_derivative_filter_q16,
        d_proc_raw_q16, conf->p_pid_kd_filter_q15);

    const int32_t p_limited = clamp_s32(p_q16, -65536, 65536);
    const int32_t i_limit = 65536 - abs_s32_sat(p_limited);
    pid->integrator_q16 = clamp_s32(pid->integrator_q16, -i_limit, i_limit);

    const int32_t output_q16 = clamp_s32((int64_t)p_limited + pid->integrator_q16 +
        pid->derivative_filter_q16 + pid->process_derivative_filter_q16, -65536, 65536);
    pid->previous_error = error;
    pid->previous_process = motor->m_position_ticks;

    const int32_t imax = conf->l_current_max;
    motor->m_iq_set = sat_s16(((int64_t)output_q16 * imax) >> 16);
    pid->last_p = sat_s16(((int64_t)p_q16 * imax) >> 16);
    pid->last_i = sat_s16(((int64_t)pid->integrator_q16 * imax) >> 16);
    const int64_t d_total_q16 = (int64_t)pid->derivative_filter_q16 +
                                pid->process_derivative_filter_q16;
    pid->last_d = sat_s16((d_total_q16 * imax) >> 16);
    pid->last_output = motor->m_iq_set;
    pid->anti_windup = abs_s32_sat(pid->integrator_q16) >= i_limit;
    return motor->m_iq_set;
}

static void reconstruct_currents(const mc_configuration *conf,
                                 int16_t sample_1, int16_t sample_2,
                                 int16_t *i_alpha, int16_t *i_beta)
{
    int32_t ia;
    int32_t ib;
    int32_t ic;

    const int32_t first = clamp_s16((int32_t)sample_1 * 16, -27200, 27200);
    const int32_t second = clamp_s16((int32_t)sample_2 * 16, -27200, 27200);

    if (conf->foc_current_sample_mode == FOC_CURRENT_SAMPLE_IB_IC) {
        ib = first;
        ic = second;
        ia = -(ib + ic);
    } else {
        ia = first;
        ib = second;
        ic = -(ia + ib);
    }
    (void)ic;

    *i_alpha = sat_s16(ia);
    /* Standard Clarke: beta = (Ia + 2*Ib)/sqrt(3). */
    *i_beta = sat_s16(((ia + (2 * ib)) * ONE_BY_SQRT3_Q14) >> 14);
}

static int16_t current_pi_axis(int16_t target,
                               int16_t measured,
                               int32_t kp_q16,
                               int32_t ki_dt_q16,
                               int16_t min_output,
                               int16_t max_output,
                               int32_t *integrator_q16,
                               mc_pid_state_t *pid)
{
    const int16_t error = sat_s16((int32_t)target - measured);
    const int32_t p = ((int32_t)error * kp_q16) >> 16;
    const int32_t delta_i_q16 = (int32_t)error * ki_dt_q16;
    int32_t candidate_i_q16;

    if (delta_i_q16 > 0 && *integrator_q16 > INT32_MAX - delta_i_q16) {
        candidate_i_q16 = INT32_MAX;
    } else if (delta_i_q16 < 0 && *integrator_q16 < INT32_MIN - delta_i_q16) {
        candidate_i_q16 = INT32_MIN;
    } else {
        candidate_i_q16 = *integrator_q16 + delta_i_q16;
    }

    /* Multiplication is defined for negative values; left-shifting a negative
     * signed integer is undefined in C. The configured limits fit int32 here. */
    const int32_t max_i_q16 = (int32_t)max_output * 65536L;
    const int32_t min_i_q16 = (int32_t)min_output * 65536L;
    if (candidate_i_q16 > max_i_q16) candidate_i_q16 = max_i_q16;
    if (candidate_i_q16 < min_i_q16) candidate_i_q16 = min_i_q16;

    int32_t raw = p + (candidate_i_q16 >> 16);
    const bool high = raw > max_output;
    const bool low = raw < min_output;
    const bool anti_windup = (high && error > 0) || (low && error < 0);
    if (!anti_windup) *integrator_q16 = candidate_i_q16;

    raw = p + (*integrator_q16 >> 16);
    const int16_t output = clamp_s16(raw, min_output, max_output);

    pid->previous_error = error;
    pid->last_p = sat_s16(p);
    pid->last_i = sat_s16(*integrator_q16 >> 16);
    pid->last_d = 0;
    pid->last_output = output;
    pid->anti_windup = anti_windup;
    pid->initialized = true;
    pid->integrator_q16 = *integrator_q16;
    return output;
}


static int16_t min_abs_s16(int16_t a, int16_t b)
{
    return abs_s16_sat(a) <= abs_s16_sat(b) ? a : b;
}

static int16_t max_abs_s16(int16_t a, int16_t b)
{
    return abs_s16_sat(a) >= abs_s16_sat(b) ? a : b;
}

static int16_t current_circle_q_limit(int16_t id, int16_t iq, int16_t imax)
{
    int32_t d = id;
    if (d > imax) d = imax;
    if (d < -imax) d = -imax;
    const uint64_t im2 = (uint64_t)(uint32_t)imax * (uint32_t)imax;
    const int64_t d64 = d;
    const uint64_t id2 = (uint64_t)(d64 * d64);
    const uint64_t remain = id2 >= im2 ? 0U : im2 - id2;
    const int32_t qmax = (int32_t)isqrt_u64(remain);
    return clamp_s16(iq, (int16_t)-qmax, (int16_t)qmax);
}

static int16_t mtpa_id_fixed(const mc_configuration *conf, int16_t iq_ref)
{
    if (conf->foc_mtpa_mode == MTPA_MODE_OFF ||
        conf->foc_motor_ld_lq_diff_uh == 0 ||
        conf->foc_mtpa_lambda_over_ld_lq_current == 0) return 0;

    const int64_t a = conf->foc_mtpa_lambda_over_ld_lq_current;
    const int64_t iq = iq_ref;
    const uint64_t radicand = (uint64_t)(a * a) +
                              (uint64_t)(8LL * iq * iq);
    const int64_t root = (int64_t)isqrt_u64(radicand);
    /* Algebraically identical to VESC:
     * (lambda - sqrt(lambda^2 + 8*(dL*Iq)^2)) / (4*dL)
     * after dividing numerator by dL. */
    const int64_t signed_root = conf->foc_motor_ld_lq_diff_uh > 0 ? root : -root;
    return sat_s16((int32_t)((a - signed_root) / 4LL));
}

static void foc_run_fw_fixed(motor_all_state_t *motor)
{
    mc_configuration *conf = motor->m_conf;
    motor_state_t *state = &motor->m_motor_state;

    if (conf->foc_fw_current_max <= 0) {
        motor->m_i_fw_set = 0;
        motor->m_i_fw_set_q16 = 0;
        return;
    }

    const bool fw_mode = motor->m_control_mode == CONTROL_MODE_CURRENT ||
                         motor->m_control_mode == CONTROL_MODE_SPEED;
    int32_t fw_now = 0;

    /* As in VESC, an already-active FW request is allowed to ramp out after a
     * mode change instead of being dropped in one sample. */
    if (fw_mode || motor->m_i_fw_set > 0) {
        const uint16_t duty = state->duty_abs_filtered_q15;
        const uint16_t start = conf->foc_fw_duty_start_q15;
        if (start < 32440U && duty > start) { /* VESC: foc_fw_duty_start < 0.99 */
            int32_t i_fw_max = conf->foc_fw_current_max;

            if (conf->foc_fw_backoff_q15 > 0U && i_fw_max > 0) {
                const int32_t speed_sign = motor->m_speed_rpm_q4 > 0 ? 1 :
                                           (motor->m_speed_rpm_q4 < 0 ? -1 : 0);
                /* Every factor is bounded to int16/Q15, so the product is at
                 * most 65535*32767 = 2147385345 and fits signed int32. Keep
                 * this division 32-bit so Cortex-M3 can use SDIV instead of
                 * the expensive __aeabi_ldivmod helper in the 16-kHz path. */
                const int32_t iq_error = (int32_t)state->iq - state->iq_target;
                int32_t backoff_num = iq_error * (int32_t)conf->foc_fw_backoff_q15;
                if (speed_sign < 0) backoff_num = -backoff_num;
                else if (speed_sign == 0) backoff_num = 0;
                int32_t backoff_q15 = backoff_num / i_fw_max;
                if (backoff_q15 < 0) backoff_q15 = 0;
                if (backoff_q15 > 32767) backoff_q15 = 32767;
                i_fw_max = (int32_t)(((int64_t)i_fw_max * (32767 - backoff_q15)) >> 15);
            }

            const uint32_t span = (uint32_t)32767U - start;
            if (span > 0U) {
                /* Product <= 32767^2, therefore a native 32-bit UDIV is exact. */
                const uint32_t fw_num = (uint32_t)(duty - start) * (uint32_t)i_fw_max;
                fw_now = (int32_t)(fw_num / span);
            }
        }
    }

    const int32_t target_q16 = fw_now * 65536L;
    if (conf->foc_fw_ramp_time_ms == 0U) {
        motor->m_i_fw_set_q16 = target_q16;
    } else {
        /* uint16 milliseconds * 16 kHz still fits uint32. Keeping both
         * divisions 32-bit avoids 64-bit runtime division in the fast loop. */
        const uint32_t samples = ((uint32_t)conf->foc_fw_ramp_time_ms *
                                  FOC_CONTROL_FREQUENCY_HZ + 999U) / 1000U;
        const uint32_t step_num = (uint32_t)(uint16_t)conf->foc_fw_current_max << 16;
        int32_t step_q16 = samples > 0U
            ? (int32_t)(step_num / samples)
            : target_q16;
        if (step_q16 < 1) step_q16 = 1;
        motor->m_i_fw_set_q16 = step_towards_i32(motor->m_i_fw_set_q16,
                                                  target_q16, step_q16);
    }
    motor->m_i_fw_set = clamp_s16(motor->m_i_fw_set_q16 >> 16, 0,
                                   conf->foc_fw_current_max);
}

static void apply_mtpa_fw_and_current_limits(motor_all_state_t *motor,
                                              int16_t *id_target,
                                              int16_t *iq_target)
{
    mc_configuration *conf = motor->m_conf;
    motor_state_t *state = &motor->m_motor_state;
    int16_t id = *id_target;
    int16_t iq = *iq_target;
    state->id_target_mtpa = 0;
    state->id_target_fw = 0;

    /* Match current VESC behavior: MTPA can run in current-command open-loop
     * operation. Sensor commissioning uses voltage_override, which bypasses this
     * block and therefore cannot be disturbed by MTPA. */
    if (!motor->m_voltage_override &&
        conf->foc_mtpa_mode != MTPA_MODE_OFF && conf->foc_motor_ld_lq_diff_uh != 0 &&
        conf->foc_motor_flux_linkage_uwb != 0U) {
        int16_t iq_ref = iq;
        if (conf->foc_mtpa_mode == MTPA_MODE_IQ_MEASURED) {
            iq_ref = min_abs_s16(iq, state->iq_filter);
        }
        const int16_t id_mtpa = mtpa_id_fixed(conf, iq_ref);
        state->id_target_mtpa = id_mtpa;
        id = id_mtpa;

        const int64_t iq_abs = abs_s16_sat(iq);
        const int64_t id_abs = abs_s16_sat(id_mtpa);
        const uint64_t iq2 = (uint64_t)(iq_abs * iq_abs);
        const uint64_t id2 = (uint64_t)(id_abs * id_abs);
        const int16_t iq_mag = (int16_t)isqrt_u64(iq2 > id2 ? iq2 - id2 : 0U);
        iq = iq < 0 ? (int16_t)-iq_mag : iq_mag;
    }

    foc_run_fw_fixed(motor);
    const int16_t id_fw = (int16_t)-motor->m_i_fw_set;
    state->id_target_fw = id_fw;
    /* VESC 7.00: choose whichever of MTPA and FW requires the larger |Id|;
     * do not sum them. Preserve an explicit nonzero Id command if it is larger. */
    id = max_abs_s16(id, id_fw);

    if (motor->m_i_fw_set > 0 && conf->foc_fw_q_current_factor_q15 > 0U) {
        const int32_t comp = (int32_t)(((int64_t)motor->m_i_fw_set *
                              conf->foc_fw_q_current_factor_q15) >> 15);
        const int32_t qsign = state->mod_q_filter > 0 ? 1 :
                              (state->mod_q_filter < 0 ? -1 : 0);
        iq = sat_s16((int32_t)iq - qsign * comp);
    }

    id = clamp_s16(id, conf->l_current_min, conf->l_current_max);
    iq = current_circle_q_limit(id, iq, conf->l_current_max);
    *id_target = id;
    *iq_target = iq;
}

static void update_modulation_filters(motor_state_t *state)
{
    const int64_t md = state->mod_d;
    const int64_t mq = state->mod_q;
    uint32_t mag = (uint32_t)isqrt_u64((uint64_t)(md * md + mq * mq));
    if (mag > MODULATION_MAX_Q14) mag = MODULATION_MAX_Q14;
    const uint16_t duty_q15 = (uint16_t)(((uint64_t)mag * 32767U) /
                                          MODULATION_MAX_Q14);
    /* VESC uses 0.01 for m_duty_abs_filtered. Q15 alpha 328 ~= 0.01001. */
    state->duty_abs_filtered_q15 = (uint16_t)lp_q16(
        state->duty_abs_filtered_q15, duty_q15, 328U);
    state->mod_q_filter = (int16_t)lp_q16(state->mod_q_filter, state->mod_q, 6553U);
}

static void limit_voltage_vector(const mc_configuration *conf,
                                 int16_t *vd, int16_t *vq)
{
    const int16_t vmax = conf->l_max_voltage;
    *vd = clamp_s16(*vd, (int16_t)-vmax, vmax);

    const uint16_t abs_d = (uint16_t)abs_s16_sat(*vd);
    uint32_t index = ((uint32_t)abs_d * conf->foc_voltage_index_q16 + 0x8000U) >> 16;
    if (index > 64U) index = 64U;
    const int16_t qmax = (int16_t)(((uint32_t)(uint16_t)vmax * focVoltageCircleQ15[index]) >> 15);
    *vq = clamp_s16(*vq, (int16_t)-qmax, qmax);
}

static void voltage_to_modulation(const mc_configuration *conf,
                                  int16_t vd, int16_t vq,
                                  int16_t *mod_d, int16_t *mod_q)
{
    int32_t md = ((int32_t)vd * (int32_t)conf->foc_voltage_to_mod_q16) >> 16;
    int32_t mq = ((int32_t)vq * (int32_t)conf->foc_voltage_to_mod_q16) >> 16;
    *mod_d = clamp_s16(md, -MODULATION_MAX_Q14, MODULATION_MAX_Q14);
    *mod_q = clamp_s16(mq, -MODULATION_MAX_Q14, MODULATION_MAX_Q14);
}

static int32_t q14_to_counts_trunc(int32_t value_q14, uint16_t period)
{
    const int32_t product = value_q14 * (int32_t)period;
    /* Match VESC float-to-int truncation toward zero without a runtime divide. */
    return product >= 0 ? (product >> 14) : -((-product) >> 14);
}

static uint16_t clamp_u16_i32(int32_t value, uint16_t max_value)
{
    if (value <= 0) return 0U;
    if ((uint32_t)value >= max_value) return max_value;
    return (uint16_t)value;
}

/* Fixed-point port of VESC foc_svm(): same six-sector timing construction. */
static void foc_svm_q14(int16_t alpha, int16_t beta, uint16_t period,
                        uint16_t *ta, uint16_t *tb, uint16_t *tc, uint8_t *sector_out)
{
    uint8_t sector;
    const int16_t beta_by_sqrt3 = mul_q14(beta, ONE_BY_SQRT3_Q14);

    if (beta >= 0) {
        if (alpha >= 0) sector = (beta_by_sqrt3 > alpha) ? 2U : 1U;
        else sector = ((int16_t)-beta_by_sqrt3 > alpha) ? 3U : 2U;
    } else {
        if (alpha >= 0) sector = ((int16_t)-beta_by_sqrt3 > alpha) ? 5U : 6U;
        else sector = (beta_by_sqrt3 > alpha) ? 4U : 5U;
    }

    int32_t a = 0;
    int32_t b = 0;
    int32_t c = 0;
#define TO_COUNTS(x_q14) q14_to_counts_trunc((int32_t)(x_q14), period)
    switch (sector) {
        case 1: {
            const int32_t t1 = TO_COUNTS((int32_t)alpha - beta_by_sqrt3);
            const int32_t t2 = TO_COUNTS(mul_q14(beta, TWO_BY_SQRT3_Q14));
            a = ((int32_t)period + t1 + t2) >> 1;
            b = a - t1;
            c = b - t2;
            break;
        }
        case 2: {
            const int32_t t2 = TO_COUNTS((int32_t)alpha + beta_by_sqrt3);
            const int32_t t3 = TO_COUNTS(-(int32_t)alpha + beta_by_sqrt3);
            b = ((int32_t)period + t2 + t3) >> 1;
            a = b - t3;
            c = a - t2;
            break;
        }
        case 3: {
            const int32_t t3 = TO_COUNTS(mul_q14(beta, TWO_BY_SQRT3_Q14));
            const int32_t t4 = TO_COUNTS(-(int32_t)alpha - beta_by_sqrt3);
            b = ((int32_t)period + t3 + t4) >> 1;
            c = b - t3;
            a = c - t4;
            break;
        }
        case 4: {
            const int32_t t4 = TO_COUNTS(-(int32_t)alpha + beta_by_sqrt3);
            const int32_t t5 = TO_COUNTS(-mul_q14(beta, TWO_BY_SQRT3_Q14));
            c = ((int32_t)period + t4 + t5) >> 1;
            b = c - t5;
            a = b - t4;
            break;
        }
        case 5: {
            const int32_t t5 = TO_COUNTS(-(int32_t)alpha - beta_by_sqrt3);
            const int32_t t6 = TO_COUNTS((int32_t)alpha - beta_by_sqrt3);
            c = ((int32_t)period + t5 + t6) >> 1;
            a = c - t5;
            b = a - t6;
            break;
        }
        default: { /* sector 6 */
            const int32_t t6 = TO_COUNTS(-mul_q14(beta, TWO_BY_SQRT3_Q14));
            const int32_t t1 = TO_COUNTS((int32_t)alpha + beta_by_sqrt3);
            a = ((int32_t)period + t6 + t1) >> 1;
            c = a - t1;
            b = c - t6;
            break;
        }
    }
#undef TO_COUNTS

    *ta = clamp_u16_i32(a, period);
    *tb = clamp_u16_i32(b, period);
    *tc = clamp_u16_i32(c, period);
    *sector_out = sector;
}

static void output_observation_only(mc_foc_output_t *output,
                                    const motor_state_t *state,
                                    const mc_foc_sample_t *sample)
{
    /* VESC realtime values remain meaningful while the power stage is released.
     * Observation (ADC -> Clarke/Park) is deliberately independent from actuation.
     * Only PWM/targets/FW are forced to zero here; measured Id/Iq are preserved. */
    output->duty_a = 0;
    output->duty_b = 0;
    output->duty_c = 0;
    output->id = state->id_filter;
    output->iq = state->iq_filter;
    output->id_target = 0;
    output->iq_target = 0;
    output->fw_current = 0;
    output->duty_abs_q15 = 0U;
    output->speed_rpm = (int16_t)(sample->speed_rpm_q4 >> 4);
    output->electrical_angle_deg = (int16_t)(((uint32_t)state->phase * 360U) >> 16);
    output->fault_code = 0U;
}

void mc_foc_run_current_control(motor_all_state_t *motor,
                                const mc_foc_sample_t *sample,
                                mc_foc_output_t *output,
                                uint16_t pwm_period)
{
    if (motor == NULL || motor->m_conf == NULL || sample == NULL || output == NULL || pwm_period == 0U) {
        return;
    }

    mc_configuration *conf = motor->m_conf;
    motor_state_t *state = &motor->m_motor_state;
    motor->m_speed_rpm_q4 = sample->speed_rpm_q4;
    motor->m_position_ticks = sample->position_ticks;

    const bool handbrake_mode = motor->m_control_mode == CONTROL_MODE_HANDBRAKE;
    const bool phase_is_forced = motor->m_phase_override || handbrake_mode;
    const bool closed_loop_phase_valid = sample->feedback_valid;

    /* OBSERVATION IS ALWAYS ON. MotorSensor publishes its best available phase
     * even before closed-loop proof is valid. That phase must never energize the
     * bridge until feedback_valid is true, but it is still useful for telemetry,
     * commissioning and checking current-sensor offsets while DISARMED. */
    state->phase = motor->m_phase_override ? motor->m_phase_now_override :
                   (handbrake_mode ? 0U : sample->phase_q16);
    update_trig(state);

    reconstruct_currents(conf, sample->phase_current_1, sample->phase_current_2,
                         &state->i_alpha, &state->i_beta);

    /* VESC Park convention:
     * Id = cos*Ialpha + sin*Ibeta
     * Iq = cos*Ibeta  - sin*Ialpha
     */
    state->id = sat_s16((int32_t)mul_q14(state->phase_cos, state->i_alpha) +
                        mul_q14(state->phase_sin, state->i_beta));
    state->iq = sat_s16((int32_t)mul_q14(state->phase_cos, state->i_beta) -
                        mul_q14(state->phase_sin, state->i_alpha));
    state->id_filter = (int16_t)((int32_t)state->id_filter +
                                 (((int32_t)state->id - state->id_filter) >> 2));
    state->iq_filter = (int16_t)((int32_t)state->iq_filter +
                                 (((int32_t)state->iq - state->iq_filter) >> 2));

    /* ACTUATION remains strictly gated. A valid measurement is never permission
     * to switch MOSFETs. This fixes VESC Tool Id/Iq/Imotor at idle without
     * weakening the sensor-calibration/ARM safety proof. */
    if (!sample->output_enabled || (!phase_is_forced && !closed_loop_phase_valid)) {
        reset_current_integrators(state);
        motor->m_i_fw_set = 0;
        motor->m_i_fw_set_q16 = 0;
        state->id_target = 0;
        state->iq_target = 0;
        state->vd = 0;
        state->vq = 0;
        state->mod_d = 0;
        state->mod_q = 0;
        state->mod_alpha_raw = 0;
        state->mod_beta_raw = 0;
        state->pwm_a = 0;
        state->pwm_b = 0;
        state->pwm_c = 0;
        state->duty_abs_filtered_q15 = 0U;
        state->mod_q_filter = 0;
        output_observation_only(output, state, sample);
        return;
    }

    /* Keep the previous final iq_target visible to foc_run_fw_fixed() until the
     * new target has been fully processed. VESC uses the previous current-loop
     * target for its FW Iq-error backoff, then commits the new Id/Iq targets. */
    int16_t id_target = clamp_s16(motor->m_id_set, conf->l_current_min, conf->l_current_max);
    int16_t iq_target = clamp_s16(motor->m_iq_set, conf->l_current_min, conf->l_current_max);

    /* VESC CURRENT_BRAKE does not store a one-time signed torque command. It
     * recomputes the Iq sign from instantaneous speed in the fast loop, making
     * positive and negative brake-current requests equivalent. Keep SIGN(0)
     * behavior deterministic (negative Iq at exactly zero) like the upstream
     * expression -SIGN(speed)*fabs(iq). */
    if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {
        int16_t mag = iq_target < 0 ? (int16_t)-iq_target : iq_target;
        /* VESC brake current is a magnitude and the fast loop applies the
         * opposite sign of measured speed. At exactly zero speed SIGN(0)=0,
         * therefore dynamic brake current is zero; use HANDBRAKE for static hold. */
        const int16_t speed_q4 = sample->speed_rpm_q4;
        if (speed_q4 < 0) iq_target = mag;
        else if (speed_q4 > 0) iq_target = (int16_t)-mag;
        else iq_target = 0;
        id_target = 0;
    }

    if (!motor->m_current_commissioning && !motor->m_voltage_override &&
        motor->m_control_mode != CONTROL_MODE_VOLTAGE &&
        motor->m_control_mode != CONTROL_MODE_CURRENT_BRAKE &&
        motor->m_control_mode != CONTROL_MODE_HANDBRAKE) {
        apply_mtpa_fw_and_current_limits(motor, &id_target, &iq_target);
    } else {
        /* Voltage/alignment control must not inherit an old FW request. */
        motor->m_i_fw_set = 0;
        motor->m_i_fw_set_q16 = 0;
        state->id_target_mtpa = 0;
        state->id_target_fw = 0;
    }
    state->id_target = id_target;
    state->iq_target = iq_target;

    if (motor->m_voltage_override ||
        motor->m_control_mode == CONTROL_MODE_VOLTAGE) {
        state->vd = motor->m_voltage_override ? motor->m_vd_override : 0;
        state->vq = motor->m_voltage_override ? motor->m_vq_override : motor->m_voltage_q_set;
    } else {
        int32_t kp_q16 = conf->foc_current_kp_q16;
        int32_t ki_dt_q16 = conf->foc_current_ki_dt_q16;
        int16_t axis_vmax = conf->l_max_voltage;

        if (motor->m_current_commissioning) {
            if (kp_q16 > FOC_COMMISSIONING_KP_Q16_MAX)
                kp_q16 = FOC_COMMISSIONING_KP_Q16_MAX;
            if (ki_dt_q16 > FOC_COMMISSIONING_KI_DT_Q16_MAX)
                ki_dt_q16 = FOC_COMMISSIONING_KI_DT_Q16_MAX;
            if (axis_vmax > FOC_COMMISSIONING_VOLTAGE_MAX)
                axis_vmax = FOC_COMMISSIONING_VOLTAGE_MAX;
        }

        if (motor->m_control_mode == CONTROL_MODE_DUTY) {
            /* VESC duty mode still runs through the FOC current controller; the
             * requested duty limits available voltage/modulation instead of
             * bypassing PI with a raw Vq request. This preserves current control
             * and makes LEFT/RIGHT duty behavior symmetric. */
            int32_t dq = motor->m_duty_cycle_set_q15;
            if (dq < 0) dq = -dq;
            if (dq > 32767) dq = 32767;
            int32_t lim = ((int32_t)conf->l_max_voltage * dq + 16383) / 32767;
            if (lim < 1 && dq != 0) lim = 1;
            if (lim < axis_vmax) axis_vmax = (int16_t)lim;
        }

        state->vd = current_pi_axis(state->id_target, state->id,
                                    kp_q16, ki_dt_q16,
                                    (int16_t)-axis_vmax, axis_vmax,
                                    &state->vd_int_q16, &state->pid_d);

        int16_t qmax = axis_vmax;
        if (!motor->m_current_commissioning) {
            /* D-axis priority exactly like VESC circle limiting: q-axis receives
             * the remaining voltage vector after Vd is known. */
            int16_t vd_limited = state->vd;
            /* Preserve a duty-mode voltage cap on the q-axis as well. V19
             * originally seeded q_probe with full l_max_voltage, which silently
             * discarded the duty cap after the D-axis PI and made requested
             * duty depend on load/current. */
            int16_t q_probe = axis_vmax;
            limit_voltage_vector(conf, &vd_limited, &q_probe);
            qmax = abs_s16_sat(q_probe);
            if (qmax > axis_vmax) qmax = axis_vmax;
            state->vd = vd_limited;
        }

        state->vq = current_pi_axis(state->iq_target, state->iq,
                                    kp_q16, ki_dt_q16,
                                    (int16_t)-qmax, qmax,
                                    &state->vq_int_q16, &state->pid_q);
    }

    limit_voltage_vector(conf, &state->vd, &state->vq);
    voltage_to_modulation(conf, state->vd, state->vq, &state->mod_d, &state->mod_q);
    update_modulation_filters(state);

    /* Inverse Park follows VESC. */
    state->mod_alpha_raw = sat_s16((int32_t)mul_q14(state->phase_cos, state->mod_d) -
                                   mul_q14(state->phase_sin, state->mod_q));
    state->mod_beta_raw = sat_s16((int32_t)mul_q14(state->phase_cos, state->mod_q) +
                                  mul_q14(state->phase_sin, state->mod_d));

    uint16_t ta;
    uint16_t tb;
    uint16_t tc;
    foc_svm_q14(state->mod_alpha_raw, state->mod_beta_raw, pwm_period,
                &ta, &tb, &tc, &state->svm_sector);

    const int32_t center = (int32_t)pwm_period >> 1;
    state->pwm_a = sat_s16((int32_t)ta - center);
    state->pwm_b = sat_s16((int32_t)tb - center);
    state->pwm_c = sat_s16((int32_t)tc - center);

    output->duty_a = state->pwm_a;
    output->duty_b = state->pwm_b;
    output->duty_c = state->pwm_c;
    output->id = state->id_filter;
    output->iq = state->iq_filter;
    output->id_target = state->id_target;
    output->iq_target = state->iq_target;
    output->fw_current = motor->m_i_fw_set;
    output->duty_abs_q15 = state->duty_abs_filtered_q15;
    output->speed_rpm = (int16_t)(sample->speed_rpm_q4 >> 4);
    output->electrical_angle_deg = (int16_t)(((uint32_t)state->phase * 360U) >> 16);
    output->fault_code = 0U;
}

int16_t mc_foc_commissioning_voltage_limit(void)
{
    return FOC_COMMISSIONING_VOLTAGE_MAX;
}
