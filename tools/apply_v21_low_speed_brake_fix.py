#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=ROOT/path; s=p.read_text(); n=s.count(old)
    if n!=1: raise SystemExit(f'{label}: expected one match, got {n}')
    p.write_text(s.replace(old,new,1))

rep('Src/motor.c',
'''    const bool left_full_brake = motorLeft.m_control_mode == CONTROL_MODE_DUTY &&
        motorLeft.m_duty_cycle_set_q15 == 0;
    const bool right_full_brake = motorRight.m_control_mode == CONTROL_MODE_DUTY &&
        motorRight.m_duty_cycle_set_q15 == 0;
''',
'''    const bool left_full_brake = motorLeft.m_control_mode == CONTROL_MODE_DUTY &&
        motorLeft.m_duty_cycle_set_q15 == 0;
    const bool right_full_brake = motorRight.m_control_mode == CONTROL_MODE_DUTY &&
        motorRight.m_duty_cycle_set_q15 == 0;
    /* Upstream VESC FOC shorts the phases at equal/zero duty when
     * foc_short_ls_on_zero_duty is enabled, explicitly to avoid dead-time
     * distortion and increase braking torque at low speed. Our current-brake
     * loop already defines a 2 mechanical RPM deadband; use the board's proven
     * all-low-side zero vector inside that deadband instead of alternating Iq
     * sign or leaving centered SVPWM at standstill. Feedback is still required
     * for CURRENT_BRAKE; only DUTY(0) itself is sensor-independent. */
    const bool left_current_brake_short =
        motorLeft.m_control_mode == CONTROL_MODE_CURRENT_BRAKE &&
        motorLeft.m_iq_set != 0 && motorSensorSampleLeft.feedback_valid != 0U &&
        motorSensorSampleLeft.mechanical_speed_q4 > -32 && motorSensorSampleLeft.mechanical_speed_q4 < 32;
    const bool right_current_brake_short =
        motorRight.m_control_mode == CONTROL_MODE_CURRENT_BRAKE &&
        motorRight.m_iq_set != 0 && motorSensorSampleRight.feedback_valid != 0U &&
        motorSensorSampleRight.mechanical_speed_q4 > -32 && motorSensorSampleRight.mechanical_speed_q4 < 32;
    const bool left_low_side_brake = left_full_brake || left_current_brake_short;
    const bool right_low_side_brake = right_full_brake || right_current_brake_short;
''','low speed brake flags')

rep('Src/motor.c',
'''    if (left_full_brake && left_domain_ready) set_current_zero_vector_left();
    if (right_full_brake && right_domain_ready) set_current_zero_vector_right();
''',
'''    if (left_low_side_brake && left_domain_ready) set_current_zero_vector_left();
    if (right_low_side_brake && right_domain_ready) set_current_zero_vector_right();
''','low side vector gate')

rep('Src/motor.c',
'''        if (left_full_brake && left_domain_ready) {
            /* Do not let centered SVPWM overwrite the all-low-side short. */
''',
'''        if (left_low_side_brake && left_domain_ready) {
            /* Do not let centered SVPWM overwrite the all-low-side short. */
''','left skip low-side brake')

rep('Src/motor.c',
'''        if (right_full_brake && right_domain_ready) {
            /* RIGHT is independently shorted; LEFT state is untouched. */
''',
'''        if (right_low_side_brake && right_domain_ready) {
            /* RIGHT is independently shorted; LEFT state is untouched. */
''','right skip low-side brake')

p=ROOT/'tests/test_v21_dual_motor_runtime.py'; s=p.read_text()
needle="""def test_dma_architecture_remains_dual_sensor_16k_interleaved_foc():\n"""
insert="""def test_current_brake_near_zero_uses_same_low_side_short():\n    assert 'left_current_brake_short' in motor and 'right_current_brake_short' in motor\n    assert 'motorSensorSampleLeft.mechanical_speed_q4 > -32' in motor\n    assert 'motorSensorSampleRight.mechanical_speed_q4 > -32' in motor\n    assert 'left_low_side_brake = left_full_brake || left_current_brake_short' in motor\n    assert 'right_low_side_brake = right_full_brake || right_current_brake_short' in motor\n\n"""
if insert not in s:
    if needle not in s: raise SystemExit('test insertion anchor missing')
    s=s.replace(needle,insert+needle,1)
p.write_text(s)

p=ROOT/'CHANGELOG_V21.md'; s=p.read_text()
line='- CURRENT_BRAKE inside the existing ±2 mechanical RPM deadband now uses the same all-low-side hardware short, matching VESC low-speed short semantics and preventing Hall sign chatter/freewheel at standstill.\n'
if line not in s: s += line
p.write_text(s)
print('V21 low-speed brake patch applied')
