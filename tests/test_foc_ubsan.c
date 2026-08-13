#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc_motor.h"
#include "motor_sensor.h"

static uint32_t rng_state = 0xC001D00DU;
static uint32_t rng32(void) {
    rng_state = rng_state * 1664525U + 1013904223U;
    return rng_state;
}
static int16_t rand_s16_limit(int16_t lim) {
    int32_t span = (int32_t)lim * 2 + 1;
    return (int16_t)((int32_t)(rng32() % (uint32_t)span) - lim);
}

int main(void) {
    mc_configuration c;
    motor_all_state_t m;
    mc_foc_output_t o;
    mc_foc_sample_t s;
    mc_foc_conf_set_defaults(&c, FOC_CURRENT_SAMPLE_IA_IB);

    /* Deliberately hostile config input must be clamped safely by prepare. */
    c.foc_current_kp_q16 = INT32_MAX;
    c.foc_current_ki_q16 = INT32_MAX;
    c.s_pid_kp_q16 = INT32_MAX;
    c.s_pid_ki_q16 = INT32_MAX;
    c.s_pid_kd_q16 = INT32_MAX;
    c.p_pid_kp_q16 = INT32_MAX;
    c.p_pid_ki_q16 = INT32_MAX;
    c.p_pid_kd_q16 = INT32_MAX;
    c.p_pid_kd_proc_q16 = INT32_MAX;
    c.s_pid_kd_filter_q15 = UINT16_MAX;
    c.p_pid_kd_filter_q15 = UINT16_MAX;
    c.foc_fw_duty_start_q15 = UINT16_MAX;
    c.foc_fw_q_current_factor_q15 = UINT16_MAX;
    c.foc_fw_backoff_q15 = UINT16_MAX;
    c.foc_fw_current_max = INT16_MAX;
    c.foc_motor_flux_linkage_uwb = UINT32_MAX;
    c.foc_motor_ld_lq_diff_uh = INT32_MIN;
    c.foc_mtpa_mode = (mc_mtpa_mode)255;
    mc_foc_conf_prepare(&c);
    assert(c.foc_current_kp_q16 <= 65536);
    assert(c.s_pid_kp_q16 <= 1048576);
    assert(c.s_pid_kd_filter_q15 <= FOC_Q15_ONE);
    assert(c.p_pid_kd_filter_q15 <= FOC_Q15_ONE);
    assert(c.foc_fw_current_max <= c.l_current_max);
    assert(c.foc_mtpa_mode == MTPA_MODE_OFF);

    /* Exercise valid MTPA/FW math under UBSan after hostile-input clamping. */
    c.foc_motor_flux_linkage_uwb = 1000U;
    c.foc_motor_ld_lq_diff_uh = 100;
    c.foc_mtpa_mode = MTPA_MODE_IQ_TARGET;
    c.foc_fw_current_max = c.l_current_max / 2;
    c.foc_fw_duty_start_q15 = 24575U;
    c.foc_fw_ramp_time_ms = 250U;
    c.foc_fw_q_current_factor_q15 = 16384U;
    c.foc_fw_backoff_q15 = 8192U;
    c.s_pid_ramp_erpms_s = 50000;
    mc_foc_conf_prepare(&c);

    mc_foc_init(&m, &c);
    mc_foc_set_control_mode(&m, CONTROL_MODE_CURRENT);
    memset(&s, 0, sizeof(s));
    s.output_enabled = true;
    s.feedback_valid = true;

    for (unsigned k = 0; k < 250000U; ++k) {
        if ((k & 0x3FFFU) == 0U) {
            c.foc_mtpa_mode = (mc_mtpa_mode)(1U + (rng32() & 1U));
            mc_foc_conf_prepare(&c);
        }
        s.phase_q16 = (uint16_t)rng32();
        s.speed_rpm_q4 = rand_s16_limit(16000);
        s.position_ticks = (int32_t)rng32();
        s.phase_current_1 = rand_s16_limit(2200);
        s.phase_current_2 = rand_s16_limit(2200);
        m.m_id_set = rand_s16_limit(c.l_current_max);
        m.m_iq_set = rand_s16_limit(c.l_current_max);
        mc_foc_run_current_control(&m, &s, &o, 2000U);
        const int64_t current_vec2 = (int64_t)o.id_target * o.id_target +
                                     (int64_t)o.iq_target * o.iq_target;
        assert(current_vec2 <= (int64_t)c.l_current_max * c.l_current_max);
        assert(o.duty_a >= -1000 && o.duty_a <= 1000);
        assert(o.duty_b >= -1000 && o.duty_b <= 1000);
        assert(o.duty_c >= -1000 && o.duty_c <= 1000);
    }

    mc_foc_set_control_mode(&m, CONTROL_MODE_SPEED);
    for (unsigned k = 0; k < 10000U; ++k) {
        m.m_speed_command_rpm = (int32_t)rand_s16_limit(30000);
        m.m_speed_rpm_q4 = rand_s16_limit(16000);
        (void)mc_foc_run_pid_control_speed(&m, (rng32() % 100U) + 1U);
    }

    mc_foc_set_control_mode(&m, CONTROL_MODE_POS);
    c.p_pid_pos_min = INT32_MIN;
    c.p_pid_pos_max = INT32_MAX;
    for (unsigned k = 0; k < 10000U; ++k) {
        m.m_pos_pid_set = (int32_t)rng32();
        m.m_position_ticks = (int32_t)rng32();
        (void)mc_foc_run_pid_control_pos(&m, (rng32() % 100U) + 1U);
    }

    puts("FOC_UBSAN_FUZZ_PASS");
    return 0;
}
