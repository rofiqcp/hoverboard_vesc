#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=R/path; s=p.read_text()
    if s.count(old)!=1: raise SystemExit(f'{label}: expected 1 got {s.count(old)}')
    p.write_text(s.replace(old,new,1))

rep('Src/foc_motor.h',
'''    uint16_t duty_abs_q15;\n    int16_t speed_rpm;''',
'''    uint16_t duty_abs_q15;\n    bool zero_duty_phase_brake;\n    int16_t speed_rpm;''',
'output flag')
rep('Src/foc_motor.c',
'''    if (motor == NULL || motor->m_conf == NULL || sample == NULL || output == NULL || pwm_period == 0U) {\n        return;\n    }\n\n    mc_configuration *conf = motor->m_conf;''',
'''    if (motor == NULL || motor->m_conf == NULL || sample == NULL || output == NULL || pwm_period == 0U) {\n        return;\n    }\n    output->zero_duty_phase_brake = false;\n\n    mc_configuration *conf = motor->m_conf;''',
'clear output flag')
rep('Src/foc_motor.c',
'''        output_observation_only(output, state, sample);\n        return;\n    }\n\n    /* Keep the previous final iq_target visible to foc_run_fw_fixed() until the''',
'''        output_observation_only(output, state, sample);\n        return;\n    }\n\n    if (motor->m_control_mode == CONTROL_MODE_DUTY &&\n        motor->m_duty_cycle_set_q15 == 0) {\n        reset_current_integrators(state);\n        motor->m_i_fw_set = 0;\n        motor->m_i_fw_set_q16 = 0;\n        state->id_target = 0; state->iq_target = 0;\n        state->vd = 0; state->vq = 0;\n        state->mod_d = 0; state->mod_q = 0;\n        state->pwm_a = 0; state->pwm_b = 0; state->pwm_c = 0;\n        output_observation_only(output, state, sample);\n        output->duty_abs_q15 = 0U;\n        output->zero_duty_phase_brake = true;\n        return;\n    }\n\n    /* Keep the previous final iq_target visible to foc_run_fw_fixed() until the''',
'zero duty output policy')
rep('Src/motor.c',
'''static void set_pwm_left(const mc_foc_output_t *out)\n{\n    LEFT_TIM->LEFT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);\n    LEFT_TIM->LEFT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);\n    LEFT_TIM->LEFT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);\n}''',
'''static void set_pwm_left(const mc_foc_output_t *out)\n{\n    if (out->zero_duty_phase_brake) {\n        set_current_zero_vector_left();\n        return;\n    }\n    LEFT_TIM->LEFT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);\n    LEFT_TIM->LEFT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);\n    LEFT_TIM->LEFT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);\n}''',
'left mapper')
rep('Src/motor.c',
'''static void set_pwm_right(const mc_foc_output_t *out)\n{\n    RIGHT_TIM->RIGHT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);\n    RIGHT_TIM->RIGHT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);\n    RIGHT_TIM->RIGHT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);\n}''',
'''static void set_pwm_right(const mc_foc_output_t *out)\n{\n    if (out->zero_duty_phase_brake) {\n        set_current_zero_vector_right();\n        return;\n    }\n    RIGHT_TIM->RIGHT_TIM_U = pwm_compare_from_centered_duty(out->duty_a);\n    RIGHT_TIM->RIGHT_TIM_V = pwm_compare_from_centered_duty(out->duty_b);\n    RIGHT_TIM->RIGHT_TIM_W = pwm_compare_from_centered_duty(out->duty_c);\n}''',
'right mapper')

p=R/'tests/test_v19_duty_behavior.c'; s=p.read_text()
anchor='''    const uint16_t neg = run_duty(&conf, (int16_t)-d90, conf.l_current_min);\n'''
insert='''    const uint16_t neg = run_duty(&conf, (int16_t)-d90, conf.l_current_min);\n\n    motor_all_state_t z; mc_foc_output_t zo; mc_foc_sample_t zs;\n    memset(&zs, 0, sizeof(zs)); zs.output_enabled=true; zs.feedback_valid=true;\n    mc_foc_init(&z,&conf); mc_foc_set_control_mode(&z,CONTROL_MODE_DUTY);\n    z.m_duty_cycle_set_q15=0; z.m_iq_set=conf.l_current_max;\n    mc_foc_run_current_control(&z,&zs,&zo,2000U);\n    assert(zo.zero_duty_phase_brake);\n    assert(zo.duty_abs_q15==0U);\n'''
if s.count(anchor)!=1: raise SystemExit('duty test anchor')
p.write_text(s.replace(anchor,insert,1))
(R/'tests/test_v21_full_brake_205320.py').write_text('''from pathlib import Path\nR=Path(__file__).resolve().parents[1]\ndef s(p): return (R/p).read_text()\ndef test_zero_duty_output_mapping():\n f=s("Src/foc_motor.c"); m=s("Src/motor.c"); h=s("Src/foc_motor.h")\n assert "bool zero_duty_phase_brake" in h\n assert "output->zero_duty_phase_brake = true" in f\n assert "if (out->zero_duty_phase_brake)" in m\n assert "set_current_zero_vector_left();" in m and "set_current_zero_vector_right();" in m\n''')
print('source layer staged')
