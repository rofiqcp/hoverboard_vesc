/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-point STM32F103 adaptation of the VESC FOC control architecture.
 * VESC reference: Copyright 2016-2026 Benjamin Vedder / vedderb/bldc.
 * Board firmware lineage: Copyright 2019-2020 Emanuel FERU.
 * See NOTICE.md and COPYING.
 */

#ifndef FOC_MOTOR_H
#define FOC_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "control_profile.h"

#define FOC_CONTROL_FREQUENCY_HZ CONTROL_FOC_MOTOR_FREQUENCY_HZ
#define FOC_PHASE_FULL_TURN      65536U
#define FOC_TRIG_Q               14U
#define FOC_GAIN_Q               16U
#define FOC_Q15_ONE              32767U

typedef enum {
    CONTROL_MODE_NONE = 0,
    CONTROL_MODE_VOLTAGE,
    CONTROL_MODE_SPEED,
    CONTROL_MODE_CURRENT,
    /* VESC FOC has distinct current-brake and handbrake semantics. */
    CONTROL_MODE_CURRENT_BRAKE,
    CONTROL_MODE_HANDBRAKE,
    CONTROL_MODE_POS,
    CONTROL_MODE_OPENLOOP,
    /* VESC duty request. Fast FOC applies the requested q-axis modulation
     * through the existing fixed-point voltage-to-modulation path. */
    CONTROL_MODE_DUTY
} mc_control_mode;

typedef enum {
    FOC_CURRENT_SAMPLE_IA_IB = 0,
    FOC_CURRENT_SAMPLE_IB_IC = 1
} mc_foc_current_sample_mode;

/* Names and semantics follow VESC MTPA_MODE. */
typedef enum {
    MTPA_MODE_OFF = 0,
    MTPA_MODE_IQ_TARGET = 1,
    MTPA_MODE_IQ_MEASURED = 2
} mc_mtpa_mode;

/* Shared PID bookkeeping. Q16 terms are normalized controller output. */
typedef struct {
    int32_t integrator_q16;
    int32_t previous_error;
    int32_t previous_process;
    int32_t derivative_filter_q16;
    int32_t process_derivative_filter_q16;
    uint32_t derivative_dt_accum_ms;
    uint32_t process_dt_accum_ms;
    int16_t last_p;
    int16_t last_i;
    int16_t last_d;
    int16_t last_output;
    bool anti_windup;
    bool initialized;
} mc_pid_state_t;

/*
 * Fixed-point mc_configuration subset used by this board.
 * Current/voltage use the existing internal board current domain (ADC delta*16).
 * PID gains use signed Q16.16. Filter constants/duty fractions use Q15.
 * Motor model is integer engineering units so the 16-kHz path has no float:
 *   foc_motor_flux_linkage_uwb : micro-weber
 *   foc_motor_ld_lq_diff_uh    : micro-henry, signed Ld-Lq
 */
typedef struct {
    int16_t l_current_max;
    int16_t l_current_min;
    int16_t l_max_voltage;
    int16_t l_max_speed_rpm_q4;          /* mechanical RPM * 16 */

    int32_t foc_current_kp_q16;
    int32_t foc_current_ki_q16;
    int32_t foc_current_ki_dt_q16;
    uint32_t foc_voltage_to_mod_q16;
    uint32_t foc_voltage_index_q16;
    uint16_t foc_current_units_per_amp;

    /* VESC speed PID. Controller itself operates on electrical RPM (eRPM). */
    int32_t s_pid_kp_q16;
    int32_t s_pid_ki_q16;
    int32_t s_pid_kd_q16;
    uint16_t s_pid_kd_filter_q15;
    int32_t s_pid_min_erpm;
    int32_t s_pid_ramp_erpms_s;
    bool s_pid_allow_braking;

    /* VESC position PID structure, adapted to this board's signed multi-turn ticks. */
    int32_t p_pid_kp_q16;
    int32_t p_pid_ki_q16;
    int32_t p_pid_kd_q16;
    int32_t p_pid_kd_proc_q16;
    uint16_t p_pid_kd_filter_q15;
    uint32_t p_pid_gain_dec_ticks;
    int32_t p_pid_pos_min;
    int32_t p_pid_pos_max;
    uint16_t p_pid_deadband_ticks;

    /* VESC MTPA motor model. MTPA is disabled unless mode and model are valid. */
    uint32_t foc_motor_flux_linkage_uwb;
    int32_t foc_motor_ld_lq_diff_uh;
    mc_mtpa_mode foc_mtpa_mode;
    int32_t foc_mtpa_lambda_over_ld_lq_current; /* prepared signed current units */

    /* VESC 7 fast-loop field weakening subset, normalized to this board. */
    int16_t foc_fw_current_max;
    uint16_t foc_fw_duty_start_q15;
    uint16_t foc_fw_ramp_time_ms;
    uint16_t foc_fw_q_current_factor_q15;
    uint16_t foc_fw_backoff_q15;

    uint8_t foc_motor_pole_pairs;
    mc_foc_current_sample_mode foc_current_sample_mode;
} mc_configuration;

typedef struct {
    uint16_t phase;
    int16_t phase_sin;
    int16_t phase_cos;

    int16_t i_alpha;
    int16_t i_beta;
    int16_t id;
    int16_t iq;
    int16_t id_filter;
    int16_t iq_filter;
    int16_t id_target;
    int16_t iq_target;
    int16_t id_target_mtpa;
    int16_t id_target_fw;

    int16_t vd;
    int16_t vq;
    int32_t vd_int_q16;
    int32_t vq_int_q16;

    int16_t mod_alpha_raw;
    int16_t mod_beta_raw;
    int16_t mod_d;
    int16_t mod_q;
    int16_t mod_q_filter;
    uint16_t duty_abs_filtered_q15;
    uint8_t svm_sector;

    int16_t pwm_a;
    int16_t pwm_b;
    int16_t pwm_c;

    mc_pid_state_t pid_d;
    mc_pid_state_t pid_q;
} motor_state_t;

typedef struct {
    mc_configuration *m_conf;
    motor_state_t m_motor_state;
    mc_control_mode m_control_mode;

    int16_t m_id_set;
    int16_t m_iq_set;
    int32_t m_speed_command_rpm;         /* VESC semantic: target electrical RPM */
    int32_t m_speed_pid_set_rpm;         /* ramped electrical RPM */
    int32_t m_pos_pid_set;               /* board extension: signed mechanical ticks */
    int16_t m_voltage_q_set;
    int16_t m_duty_cycle_set_q15;       /* signed VESC duty target -32767..32767 */

    int16_t m_speed_rpm_q4;              /* measured mechanical RPM * 16 */
    int32_t m_position_ticks;

    mc_pid_state_t m_speed_pid;
    mc_pid_state_t m_pos_pid;

    int16_t m_i_fw_set;
    int32_t m_i_fw_set_q16;

    bool m_phase_override;
    uint16_t m_phase_now_override;
    /* Dedicated sensor-detect flag: forced phase + closed-loop Id current.
     * It disables MTPA/FW without changing generic OPEN/phase-override behavior. */
    bool m_current_commissioning;
    bool m_voltage_override;
    int16_t m_vd_override;
    int16_t m_vq_override;
} motor_all_state_t;

typedef struct {
    bool output_enabled;
    bool feedback_valid;
    uint16_t phase_q16;
    int16_t speed_rpm_q4;
    int32_t position_ticks;
    int16_t phase_current_1;
    int16_t phase_current_2;
} mc_foc_sample_t;

typedef struct {
    int16_t duty_a;
    int16_t duty_b;
    int16_t duty_c;
    int16_t id;
    int16_t iq;
    int16_t id_target;
    int16_t iq_target;
    int16_t fw_current;
    uint16_t duty_abs_q15;
    int16_t speed_rpm;
    int16_t electrical_angle_deg;
    uint8_t fault_code;
} mc_foc_output_t;

extern mc_configuration motorConfLeft;
extern mc_configuration motorConfRight;
extern motor_all_state_t motorLeft;
extern motor_all_state_t motorRight;
extern mc_foc_output_t motorOutputLeft;
extern mc_foc_output_t motorOutputRight;

extern const int16_t focSinTableQ14[256];
extern const uint16_t focVoltageCircleQ15[65];

void mc_foc_conf_set_defaults(mc_configuration *conf,
                              mc_foc_current_sample_mode current_sample_mode);
void mc_foc_conf_prepare(mc_configuration *conf);
void mc_foc_init(motor_all_state_t *motor, mc_configuration *conf);
void mc_foc_reset_control(motor_all_state_t *motor);
void mc_foc_reset_outer_loops(motor_all_state_t *motor);
void mc_foc_set_control_mode(motor_all_state_t *motor, mc_control_mode mode);

/* VESC-style outer loops: run at configured slow/PID rate, never in ADC ISR. */
int16_t mc_foc_run_pid_control_speed(motor_all_state_t *motor, uint32_t dt_ms);
int16_t mc_foc_run_pid_control_pos(motor_all_state_t *motor, uint32_t dt_ms);

/* Fixed-point current-loop FOC + MTPA/FW + SVPWM. */
void mc_foc_run_current_control(motor_all_state_t *motor,
                                const mc_foc_sample_t *sample,
                                mc_foc_output_t *output,
                                uint16_t pwm_period);

void mc_foc_set_phase_override(motor_all_state_t *motor, bool active, uint16_t phase_q16);
void mc_foc_set_current_commissioning(motor_all_state_t *motor, bool active);
void mc_foc_set_voltage_override(motor_all_state_t *motor, bool active,
                                 int16_t vd, int16_t vq);
/* Exact V12 detect-only voltage ceiling exported for HBTS saturation diagnostics. */
int16_t mc_foc_commissioning_voltage_limit(void);

#endif /* FOC_MOTOR_H */
