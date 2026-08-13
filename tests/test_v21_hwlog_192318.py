from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
motor=(ROOT/"Src/motor.c").read_text()
runtime=(ROOT/"Src/runtime_control.c").read_text()
foc=(ROOT/"Src/foc_motor.c").read_text()
proto=(ROOT/"Src/vesc_protocol.c").read_text()
tester=(ROOT/"tools/vesc_full_test.py").read_text()

def test_dma_isr_is_restored_to_v20_no_v21_brake_hotpath():
    assert "left_low_side_brake" not in motor
    assert "right_low_side_brake" not in motor
    assert "left_current_brake_short" not in motor
    assert "right_current_brake_short" not in motor
    assert "V18 SENSOR CADENCE" in motor
    assert "MotorSensor_UpdateHardwareEncoder" in motor
    assert "motorFocSlotRight" in motor

def test_hall_position_is_bounded_in_slow_layer():
    assert "cfg->sensor_type == MOTOR_SENSOR_HALL_UVW" in runtime
    assert "CONTROL_CURRENT_INTERNAL_PER_A" in runtime
    assert "motor->m_iq_set = iq;" in runtime
    assert "p_limited + pid->integrator_q16" in foc
    assert "p_pid_kp_q16 = 1024" in foc

def test_rotor_stream_mode_is_per_virtual_controller():
    line=[x for x in proto.splitlines() if "case C_SET_DETECT:" in x][0]
    assert "display_position_mode_right=mode" in line
    assert "display_position_mode_left=mode" in line
    assert "if(mode!=0U)display_position_mode_left=0U" not in line
    assert "if(mode!=0U)display_position_mode_right=0U" not in line
    assert "left_ready or right_ready" in tester

def test_standard_current_telemetry_timing_is_v20_contract():
    assert "#define VESC_CURRENT_HOLD_MAX_MS 25U" in proto
    assert "!MotorControl_CurrentMeasurementValid(left))" in proto

def test_current_limits_are_independent_and_persistent():
    assert "conf->l_current_min = (int16_t)-conf->l_current_max" not in foc
    assert "EEPROM_LEFT_CURRENT_MAX" in runtime
    assert "EEPROM_LEFT_CURRENT_MIN" in runtime
    assert "EEPROM_RIGHT_CURRENT_MAX" in runtime
    assert "EEPROM_RIGHT_CURRENT_MIN" in runtime

def test_brake_is_dynamic_not_unconditional_isr_short():
    assert "left_low_side_brake" not in motor
    assert "if (speed_q4 < 0) iq_target = mag" in foc
    assert 'if kind == "brake":' in tester
    assert 'label + "_prespin"' in tester
