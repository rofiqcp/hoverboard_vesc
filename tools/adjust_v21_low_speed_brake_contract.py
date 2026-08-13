#!/usr/bin/env python3
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'tests/test_v21_dual_motor_runtime.py'
s=p.read_text()
old="""def test_f103_full_brake_uses_low_side_zero_vector_per_motor():
    assert 'left_full_brake' in motor and 'right_full_brake' in motor
    assert 'if (left_full_brake && left_domain_ready) set_current_zero_vector_left();' in motor
    assert 'if (right_full_brake && right_domain_ready) set_current_zero_vector_right();' in motor
    assert 'Do not let centered SVPWM overwrite the all-low-side short.' in motor
"""
new="""def test_f103_full_brake_uses_low_side_zero_vector_per_motor():
    assert 'left_full_brake' in motor and 'right_full_brake' in motor
    assert 'left_low_side_brake = left_full_brake || left_current_brake_short' in motor
    assert 'right_low_side_brake = right_full_brake || right_current_brake_short' in motor
    assert 'if (left_low_side_brake && left_domain_ready) set_current_zero_vector_left();' in motor
    assert 'if (right_low_side_brake && right_domain_ready) set_current_zero_vector_right();' in motor
    assert 'Do not let centered SVPWM overwrite the all-low-side short.' in motor
"""
if s.count(old)!=1:
    raise SystemExit(f'expected one old full-brake regression block, got {s.count(old)}')
p.write_text(s.replace(old,new,1))
print('V21 low-side brake regression contract migrated')
