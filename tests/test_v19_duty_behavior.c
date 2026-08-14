#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc_motor.h"

static uint16_t run_duty(mc_configuration *conf, int16_t duty_q15, int16_t iq_set) {
    motor_all_state_t m;
    mc_foc_output_t out;
    mc_foc_sample_t s;
    memset(&s, 0, sizeof(s));
    s.output_enabled = true;
    s.feedback_valid = true;
    s.phase_q16 = 12000U;
    s.speed_rpm_q4 = 0;
    mc_foc_init(&m, conf);
    mc_foc_set_control_mode(&m, CONTROL_MODE_DUTY);
    m.m_duty_cycle_set_q15 = duty_q15;
    m.m_iq_set = iq_set;
    for (int i = 0; i < 700; ++i) {
        /* Zero measured current represents unloaded startup. Current PI therefore
         * asks for available torque while the duty target caps modulation. */
        s.phase_current_1 = 0; s.phase_current_2 = 0;
        mc_foc_run_current_control(&m, &s, &out, 2000U);
    }
    assert((iq_set > 0 && out.iq_target > 0) || (iq_set < 0 && out.iq_target < 0));
    return out.duty_abs_q15;
}

int main(void) {
    mc_configuration conf;
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    conf.foc_mtpa_mode = MTPA_MODE_OFF;
    conf.foc_fw_current_max = 0;
    mc_foc_conf_prepare(&conf);

    const int16_t d03 = (int16_t)((3L * 32767L) / 100L);
    const int16_t d90 = (int16_t)((90L * 32767L) / 100L);
    const uint16_t low = run_duty(&conf, d03, conf.l_current_max);
    const uint16_t high = run_duty(&conf, d90, conf.l_current_max);
    const uint16_t neg = run_duty(&conf, (int16_t)-d90, conf.l_current_min);

    motor_all_state_t z; mc_foc_output_t zo; mc_foc_sample_t zs;
    memset(&zs, 0, sizeof(zs)); zs.output_enabled=true; zs.feedback_valid=true;
    mc_foc_init(&z,&conf); mc_foc_set_control_mode(&z,CONTROL_MODE_DUTY);
    z.m_duty_cycle_set_q15=0; z.m_iq_set=conf.l_current_max;
    mc_foc_run_current_control(&z,&zs,&zo,2000U);
    assert(zo.zero_duty_phase_brake);
    assert(zo.duty_abs_q15==0U);

    /* 3% duty must remain a low-modulation command; 90% must materially open the
     * modulation ceiling instead of behaving like the old low-duty/raw-Vq path. */
    assert(low > (uint16_t)(d03 / 2));
    assert(low < (uint16_t)(d03 + d03 / 2));
    assert(high > 10U * low);
    assert(high > 20000U);
    assert(neg > 20000U);
    assert(high < 32767U && neg < 32767U);

    printf("V19_DUTY_BEHAVIOR_PASS low=%u high=%u neg=%u\n",
           (unsigned)low, (unsigned)high, (unsigned)neg);
    return 0;
}
