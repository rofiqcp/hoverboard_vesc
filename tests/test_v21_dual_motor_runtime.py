from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
proto=(ROOT/'Src/vesc_protocol.c').read_text()
runtime=(ROOT/'Src/runtime_control.c').read_text()
motor=(ROOT/'Src/motor.c').read_text()
tester=(ROOT/'tools/vesc_full_test.py').read_text()

def test_apply_all_commissions_right_before_left_and_preserves_partial_right():
    block=proto[proto.index('static void auto_detect_service'):proto.index('static void send_print')]
    assert block.index('case AUTO_DETECT_RIGHT_HALL') < block.index('case AUTO_DETECT_LEFT_ENCODER')
    assert 'auto_detect.right_hall_ok=(res.state==ESC_SENSOR_CAL_SUCCESS);' in block
    assert 'Continue LEFT regardless of RIGHT result' in block
    assert 'RIGHT proof, when successful, remains committed and usable' in block

def test_full_brake_is_sensor_independent_but_stop_is_release():
    assert '(mode == ESC_MODE_DUTY && target == 0)' in runtime
    assert 'const bool run=true; /* SET_DUTY(0) is VESC Tool Full Brake, not release. */' in proto
    assert 'const bool run=v!=0;' in proto  # SET_CURRENT(0) release path remains

def test_f103_full_brake_uses_low_side_zero_vector_per_motor():
    assert 'left_full_brake' in motor and 'right_full_brake' in motor
    assert 'if (left_full_brake && left_domain_ready) set_current_zero_vector_left();' in motor
    assert 'if (right_full_brake && right_domain_ready) set_current_zero_vector_right();' in motor
    assert 'Do not let centered SVPWM overwrite the all-low-side short.' in motor

def test_dma_architecture_remains_dual_sensor_16k_interleaved_foc():
    assert 'MotorSensor_UpdateHardwareEncoder(&motorConfigLeft' in motor
    assert 'MotorSensor_Update(&motorConfigRight' in motor
    assert 'const bool run_right_slot = motorFocSlotRight;' in motor
    assert 'motorFocSlotRight = !motorFocSlotRight;' in motor
    assert 'bridge_current_domain_service(true' in motor
    assert 'bridge_current_domain_service(false' in motor

def test_tester_runs_each_side_from_actual_post_detect_readiness():
    assert 'left_ready = bool(post_left and post_left.get("calibrated"))' in tester
    assert 'right_ready = bool(post_right and post_right.get("calibrated"))' in tester
    assert '14d_left_full_brake_then_stop' in tester
    assert '14e_right_full_brake_then_stop' in tester
    assert 'default=65.0' in tester
