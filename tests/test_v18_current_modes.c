#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc_motor.h"

static mc_foc_sample_t sample_for_speed(int16_t mechanical_rpm_q4, bool feedback_valid) {
    mc_foc_sample_t s;
    memset(&s, 0, sizeof(s));
    s.output_enabled = true;
    s.feedback_valid = feedback_valid;
    s.phase_q16 = 12345U;
    s.speed_rpm_q4 = mechanical_rpm_q4;
    return s;
}

int main(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_output_t out;

    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    conf.foc_mtpa_mode = MTPA_MODE_OFF;
    conf.foc_fw_current_max = 0;
    mc_foc_conf_prepare(&conf);
    mc_foc_init(&m, &conf);

    /* Normal SET_CURRENT: signed Iq means signed torque. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_CURRENT);
    m.m_iq_set = 400;
    mc_foc_sample_t s = sample_for_speed(100 * 16, true);
    mc_foc_run_current_control(&m, &s, &out, 2000U);
    assert(out.iq_target == 400);

    /* VESC current-brake: command sign is irrelevant. Fast-loop Iq must oppose
     * instantaneous speed, not a sign sampled once in the UART handler. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_CURRENT_BRAKE);
    m.m_iq_set = 400;
    s = sample_for_speed(100 * 16, true);
    mc_foc_run_current_control(&m, &s, &out, 2000U);
    assert(out.iq_target == -400);

    m.m_iq_set = -400;
    s = sample_for_speed(-100 * 16, true);
    mc_foc_run_current_control(&m, &s, &out, 2000U);
    assert(out.iq_target == 400);

    /* VESC dynamic brake follows -SIGN(speed)*|Iq|. At exact zero speed
     * SIGN(0)=0, so dynamic brake current is zero; HANDBRAKE is the static hold. */
    m.m_iq_set = 400;
    s = sample_for_speed(0, true);
    mc_foc_run_current_control(&m, &s, &out, 2000U);
    assert(out.iq_target == 0);

    /* Handbrake is not current-brake. It is a fixed phase-0 current vector and
     * therefore does not require closed-loop Hall/encoder phase to energize. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_HANDBRAKE);
    m.m_iq_set = 400;
    s = sample_for_speed(0, false);
    s.phase_q16 = 40000U;
    mc_foc_run_current_control(&m, &s, &out, 2000U);
    assert(m.m_motor_state.phase == 0U);
    assert(out.iq_target == 400);
    assert(out.duty_abs_q15 > 0U);

    puts("V18_CURRENT_BRAKE_HANDBRAKE_TESTS_PASS");
    return 0;
}
