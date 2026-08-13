#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "foc_motor.h"

static int32_t q16(double v) {
    return (int32_t)llround(v * 65536.0);
}

static int abs_i32(int32_t v) {
    return v < 0 ? -v : v;
}

static void init_for_current(mc_configuration *c, motor_all_state_t *m,
                             mc_foc_current_sample_mode sample_mode) {
    mc_foc_conf_set_defaults(c, sample_mode);
    mc_foc_init(m, c);
    mc_foc_set_control_mode(m, CONTROL_MODE_CURRENT);
    mc_foc_set_phase_override(m, true, 0U);
}

static mc_foc_sample_t zero_sample(void) {
    mc_foc_sample_t s;
    memset(&s, 0, sizeof(s));
    s.output_enabled = true;
    s.feedback_valid = false; /* forced phase makes this legal */
    return s;
}

static void test_speed_pid_matches_vesc_structure(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_conf_set_defaults(&c, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_motor_pole_pairs = 10U;
    c.l_current_max = 10000;
    c.s_pid_kp_q16 = q16(0.010);
    c.s_pid_ki_q16 = q16(0.020);
    c.s_pid_kd_q16 = 0;
    c.s_pid_min_erpm = 1;
    c.s_pid_ramp_erpms_s = 0;
    c.s_pid_allow_braking = true;
    mc_foc_conf_prepare(&c);
    mc_foc_init(&m, &c);
    mc_foc_set_control_mode(&m, CONTROL_MODE_SPEED);

    m.m_speed_command_rpm = 2000;       /* eRPM */
    m.m_speed_pid_set_rpm = 2000;
    m.m_speed_rpm_q4 = 100 * 16;        /* 100 mech RPM -> 1000 eRPM */

    const int16_t first = mc_foc_run_pid_control_speed(&m, 10U);
    const double kp = c.s_pid_kp_q16 / 65536.0;
    const double expected_first = (1000.0 * kp / 20.0) * c.l_current_max;
    assert(fabs((double)first - expected_first) <= 2.0);

    /* VESC calculates output before adding this cycle's integral. Therefore the
     * second call includes the I contribution accumulated by the first call. */
    const int32_t i_after_first = m.m_speed_pid.integrator_q16;
    assert(i_after_first > 0);
    const int16_t second = mc_foc_run_pid_control_speed(&m, 10U);
    const double expected_second = expected_first +
        ((double)i_after_first / 65536.0) * c.l_current_max;
    assert(fabs((double)second - expected_second) <= 3.0);

    /* eRPM ramp is done in the outer loop before calculating the error. */
    mc_foc_reset_outer_loops(&m);
    c.s_pid_ki_q16 = 0;
    c.s_pid_ramp_erpms_s = 1000;
    mc_foc_conf_prepare(&c);
    m.m_speed_pid_set_rpm = 0;
    m.m_speed_command_rpm = 1000;
    m.m_speed_rpm_q4 = 0;
    (void)mc_foc_run_pid_control_speed(&m, 10U);
    assert(m.m_speed_pid_set_rpm == 10);

    /* VESC's optional no-braking gate must release negative torque while the
     * measured speed is positive. */
    c.s_pid_ramp_erpms_s = 0;
    c.s_pid_allow_braking = false;
    m.m_speed_command_rpm = 500;
    m.m_speed_pid_set_rpm = 500;
    m.m_speed_rpm_q4 = 100 * 16; /* 1000 eRPM measured */
    mc_foc_reset_outer_loops(&m);
    assert(mc_foc_run_pid_control_speed(&m, 10U) == 0);
}

static void test_position_pid_matches_vesc_structure(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_conf_set_defaults(&c, FOC_CURRENT_SAMPLE_IA_IB);
    c.l_current_max = 10000;
    c.p_pid_kp_q16 = q16(0.010);
    c.p_pid_ki_q16 = 0;
    c.p_pid_kd_q16 = 0;
    c.p_pid_kd_proc_q16 = 0;
    c.p_pid_kd_filter_q15 = 32767U;
    c.p_pid_deadband_ticks = 0U;
    c.p_pid_pos_min = -1000000;
    c.p_pid_pos_max = 1000000;
    mc_foc_conf_prepare(&c);
    mc_foc_init(&m, &c);
    mc_foc_set_control_mode(&m, CONTROL_MODE_POS);

    m.m_position_ticks = 0;
    m.m_pos_pid_set = 20;
    const int16_t out = mc_foc_run_pid_control_pos(&m, 5U);
    const double expected = 20.0 * (c.p_pid_kp_q16 / 65536.0) * c.l_current_max;
    assert(fabs((double)out - expected) <= 2.0);

    /* VESC process-D is derivative-on-measurement with a negative sign. */
    mc_foc_reset_outer_loops(&m);
    c.p_pid_kp_q16 = 0;
    c.p_pid_kd_proc_q16 = q16(0.0001);
    mc_foc_conf_prepare(&c);
    m.m_position_ticks = 0;
    m.m_pos_pid_set = 100;
    assert(mc_foc_run_pid_control_pos(&m, 5U) == 0);
    m.m_position_ticks = 1;
    assert(mc_foc_run_pid_control_pos(&m, 5U) < 0);
    assert(m.m_pos_pid.last_d < 0);

    /* P-based I wind-up protection: with |P| >= 1, available I headroom is 0. */
    mc_foc_reset_outer_loops(&m);
    c.p_pid_kp_q16 = q16(1.0);
    c.p_pid_ki_q16 = q16(1.0);
    c.p_pid_gain_dec_ticks = 0U; /* isolate saturation/anti-windup behavior */
    c.p_pid_kd_proc_q16 = 0;
    mc_foc_conf_prepare(&c);
    m.m_position_ticks = 0;
    m.m_pos_pid_set = 2;
    assert(mc_foc_run_pid_control_pos(&m, 10U) == c.l_current_max);
    assert(m.m_pos_pid.integrator_q16 == 0);
}

static int16_t vesc_mtpa_expected(const mc_configuration *c, int16_t iq) {
    const double lambda = c->foc_motor_flux_linkage_uwb * 1e-6;
    const double dl = c->foc_motor_ld_lq_diff_uh * 1e-6;
    const double iq_amp = iq / (double)c->foc_current_units_per_amp;
    const double id_amp = (lambda - sqrt(lambda * lambda +
        8.0 * (dl * iq_amp) * (dl * iq_amp))) / (4.0 * dl);
    return (int16_t)(id_amp * c->foc_current_units_per_amp);
}

static void test_mtpa_numeric_against_vesc_equation(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s = zero_sample();
    init_for_current(&c, &m, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_current_kp_q16 = 0;
    c.foc_current_ki_q16 = 0;
    c.foc_motor_flux_linkage_uwb = 1000U; /* 1.0 mWb */
    c.foc_motor_ld_lq_diff_uh = 100;      /* +100 uH */
    c.foc_mtpa_mode = MTPA_MODE_IQ_TARGET;
    mc_foc_conf_prepare(&c);

    const int16_t iq_tests[] = {-10000, -8000, -4000, 4000, 8000, 10000};
    for (unsigned i = 0U; i < sizeof(iq_tests) / sizeof(iq_tests[0]); ++i) {
        mc_foc_reset_control(&m);
        m.m_iq_set = iq_tests[i];
        mc_foc_run_current_control(&m, &s, &o, 2000U);
        const int16_t expected_id = vesc_mtpa_expected(&c, iq_tests[i]);
        assert(abs_i32((int32_t)m.m_motor_state.id_target_mtpa - expected_id) <= 2);
        assert(m.m_motor_state.id_target_mtpa < 0);
        const int64_t vec2 = (int64_t)o.id_target * o.id_target +
                             (int64_t)o.iq_target * o.iq_target;
        assert(vec2 <= (int64_t)c.l_current_max * c.l_current_max);
    }

    /* Negative Ld-Lq must produce the sign dictated by the same VESC formula. */
    c.foc_motor_ld_lq_diff_uh = -100;
    mc_foc_conf_prepare(&c);
    mc_foc_reset_control(&m);
    m.m_iq_set = 8000;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    const int16_t expected_pos_id = vesc_mtpa_expected(&c, 8000);
    assert(expected_pos_id > 0);
    assert(abs_i32((int32_t)o.id_target - expected_pos_id) <= 2);

    /* Current-command OPEN follows VESC and may use MTPA. Commissioning is
     * protected separately by voltage_override, which bypasses MTPA/FW. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_OPENLOOP);
    c.foc_motor_ld_lq_diff_uh = 100;
    mc_foc_conf_prepare(&c);
    mc_foc_reset_control(&m);
    m.m_iq_set = 8000;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(m.m_motor_state.id_target_mtpa < 0);

    mc_foc_set_voltage_override(&m, true, 1000, 0);
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(m.m_motor_state.id_target_mtpa == 0);
    assert(o.fw_current == 0);
    mc_foc_set_voltage_override(&m, false, 0, 0);
}

static void test_flux_weakening_fast_loop(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s = zero_sample();
    init_for_current(&c, &m, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_current_kp_q16 = 0;
    c.foc_current_ki_q16 = 0;
    c.foc_mtpa_mode = MTPA_MODE_OFF;
    c.foc_fw_current_max = 4000;
    c.foc_fw_duty_start_q15 = 26214U; /* ~80 % */
    c.foc_fw_ramp_time_ms = 0U;
    c.foc_fw_q_current_factor_q15 = 0U;
    c.foc_fw_backoff_q15 = 0U;
    mc_foc_conf_prepare(&c);

    /* Below/equal threshold: no FW. */
    m.m_motor_state.duty_abs_filtered_q15 = c.foc_fw_duty_start_q15;
    m.m_iq_set = 0;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(o.fw_current == 0);
    assert(o.id_target == 0);

    /* At normalized max duty, FW reaches configured max and becomes negative Id. */
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(o.fw_current == c.foc_fw_current_max);
    assert(o.id_target == -c.foc_fw_current_max);

    /* Current-vector circle must reserve headroom for Id. */
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    m.m_iq_set = c.l_current_max;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    const int64_t vec2 = (int64_t)o.id_target * o.id_target +
                         (int64_t)o.iq_target * o.iq_target;
    assert(vec2 <= (int64_t)c.l_current_max * c.l_current_max);
    assert(o.iq_target < c.l_current_max);

    /* Ramp time must change FW gradually at the 16-kHz fast-loop rate. */
    c.foc_fw_ramp_time_ms = 100U;
    mc_foc_conf_prepare(&c);
    m.m_i_fw_set = 0;
    m.m_i_fw_set_q16 = 0;
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    m.m_iq_set = 0;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(o.fw_current > 0 && o.fw_current < c.foc_fw_current_max);
}

static void test_mtpa_fw_vesc7_max_abs_and_q_comp(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s = zero_sample();
    init_for_current(&c, &m, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_current_kp_q16 = 0;
    c.foc_current_ki_q16 = 0;
    c.foc_motor_flux_linkage_uwb = 1000U;
    c.foc_motor_ld_lq_diff_uh = 100;
    c.foc_mtpa_mode = MTPA_MODE_IQ_TARGET;
    c.foc_fw_current_max = 7000;
    c.foc_fw_duty_start_q15 = 26214U;
    c.foc_fw_ramp_time_ms = 0U;
    c.foc_fw_q_current_factor_q15 = 16384U; /* 0.5 */
    c.foc_fw_backoff_q15 = 0U;
    mc_foc_conf_prepare(&c);

    m.m_iq_set = 8000;
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    m.m_motor_state.mod_q_filter = 1000; /* positive Q modulation for FW compensation sign */
    mc_foc_run_current_control(&m, &s, &o, 2000U);

    assert(m.m_motor_state.id_target_mtpa < 0);
    assert(m.m_motor_state.id_target_fw == -c.foc_fw_current_max);
    /* VESC 7.00 uses max-absolute MTPA/FW Id, not their sum. */
    const int mtpa_abs = abs_i32(m.m_motor_state.id_target_mtpa);
    const int fw_abs = abs_i32(m.m_motor_state.id_target_fw);
    assert(abs_i32(o.id_target) == (mtpa_abs > fw_abs ? mtpa_abs : fw_abs));
    assert(abs_i32(o.id_target) != mtpa_abs + fw_abs);
    assert(o.iq_target < 8000); /* MTPA magnitude preservation + FW Q compensation */
}

static void test_fw_backoff_uses_previous_final_iq_target(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s = zero_sample();
    init_for_current(&c, &m, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_current_kp_q16 = 0;
    c.foc_current_ki_q16 = 0;
    c.foc_fw_current_max = 4000;
    c.foc_fw_duty_start_q15 = 26214U;
    c.foc_fw_ramp_time_ms = 0U;
    c.foc_fw_backoff_q15 = 32767U;
    mc_foc_conf_prepare(&c);

    /* Previous final target=5000. New raw request=8000. Measured Iq is about
     * 6000, therefore VESC backoff must reduce FW based on the previous 5000
     * target. If the new raw 8000 target were used, no backoff would occur. */
    m.m_motor_state.iq_target = 5000;
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    m.m_iq_set = 8000;
    s.speed_rpm_q4 = 100 * 16;
    s.phase_current_1 = 0;
    s.phase_current_2 = 325; /* Iq ~ 6000 internal at phase=0 */
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(o.fw_current > 2000);
    assert(o.fw_current < 3800);
}

static void test_mtpa_sweep_and_fw_mapping(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s = zero_sample();
    init_for_current(&c, &m, FOC_CURRENT_SAMPLE_IA_IB);
    c.foc_current_kp_q16 = 0;
    c.foc_current_ki_q16 = 0;
    c.foc_mtpa_mode = MTPA_MODE_IQ_TARGET;
    c.foc_fw_current_max = 0;

    const uint32_t lambdas[] = {500U, 1000U, 2500U, 5000U};
    const int32_t dls[] = {-500, -100, -50, 50, 100, 500};
    int max_id_error = 0;
    unsigned mtpa_cases = 0U;
    for (unsigned li = 0; li < sizeof(lambdas)/sizeof(lambdas[0]); ++li) {
        for (unsigned di = 0; di < sizeof(dls)/sizeof(dls[0]); ++di) {
            c.foc_motor_flux_linkage_uwb = lambdas[li];
            c.foc_motor_ld_lq_diff_uh = dls[di];
            mc_foc_conf_prepare(&c);
            for (int iq = -12000; iq <= 12000; iq += 97) {
                if (iq == 0) continue;
                mc_foc_reset_control(&m);
                m.m_iq_set = (int16_t)iq;
                mc_foc_run_current_control(&m, &s, &o, 2000U);
                const int16_t expected = vesc_mtpa_expected(&c, (int16_t)iq);
                const int err = abs_i32((int32_t)m.m_motor_state.id_target_mtpa - expected);
                if (err > max_id_error) max_id_error = err;
                assert(err <= 4);
                const int64_t vec2 = (int64_t)o.id_target * o.id_target +
                                     (int64_t)o.iq_target * o.iq_target;
                assert(vec2 <= (int64_t)c.l_current_max * c.l_current_max);
                ++mtpa_cases;
            }
        }
    }

    /* Measured mode must use the smaller-magnitude measured/target Iq. */
    c.foc_motor_flux_linkage_uwb = 1000U;
    c.foc_motor_ld_lq_diff_uh = 100;
    c.foc_mtpa_mode = MTPA_MODE_IQ_MEASURED;
    mc_foc_conf_prepare(&c);
    mc_foc_reset_control(&m);
    m.m_iq_set = 8000;
    m.m_motor_state.iq_filter = 2000;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    /* Zero measured sample applies the production 1/4 LP once: 2000 -> 1500. */
    const int16_t expected_measured = vesc_mtpa_expected(&c, 1500);
    assert(abs_i32((int32_t)m.m_motor_state.id_target_mtpa - expected_measured) <= 2);

    /* Linear FW duty map with no backoff/ramp must match integer mapping exactly. */
    c.foc_mtpa_mode = MTPA_MODE_OFF;
    c.foc_fw_current_max = 4000;
    c.foc_fw_duty_start_q15 = 25000U;
    c.foc_fw_ramp_time_ms = 0U;
    c.foc_fw_q_current_factor_q15 = 0U;
    c.foc_fw_backoff_q15 = 0U;
    mc_foc_conf_prepare(&c);
    unsigned fw_cases = 0U;
    int max_fw_error = 0;
    for (unsigned duty = 0U; duty <= 32767U; duty += 113U) {
        mc_foc_reset_control(&m);
        m.m_motor_state.duty_abs_filtered_q15 = (uint16_t)duty;
        m.m_iq_set = 0;
        mc_foc_run_current_control(&m, &s, &o, 2000U);
        int expected_fw = 0;
        if (duty > c.foc_fw_duty_start_q15) {
            expected_fw = (int)(((uint64_t)(duty - c.foc_fw_duty_start_q15) *
                                c.foc_fw_current_max) /
                                (32767U - c.foc_fw_duty_start_q15));
        }
        const int err = abs_i32(o.fw_current - expected_fw);
        if (err > max_fw_error) max_fw_error = err;
        assert(err == 0);
        ++fw_cases;
        if (duty > 32767U - 113U) break;
    }
    /* Explicit endpoint, because 32767 is not generally hit by the stride. */
    mc_foc_reset_control(&m);
    m.m_motor_state.duty_abs_filtered_q15 = 32767U;
    mc_foc_run_current_control(&m, &s, &o, 2000U);
    assert(o.fw_current == c.foc_fw_current_max);

    printf("MTPA_SWEEP cases=%u max_id_error=%d_internal | FW_MAP cases=%u max_error=%d\n",
           mtpa_cases, max_id_error, fw_cases, max_fw_error);
}

int main(void) {
    test_speed_pid_matches_vesc_structure();
    test_position_pid_matches_vesc_structure();
    test_mtpa_numeric_against_vesc_equation();
    test_flux_weakening_fast_loop();
    test_mtpa_fw_vesc7_max_abs_and_q_comp();
    test_fw_backoff_uses_previous_final_iq_target();
    test_mtpa_sweep_and_fw_mapping();
    puts("VESC_ADVANCED_CONTROL_TESTS_PASS");
    return 0;
}
