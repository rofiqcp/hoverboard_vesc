from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def src(name):
    return (ROOT / name).read_text()

def test_encoder_detect_never_fabricates_configured_ratio():
    s = src("Src/runtime_control.c")
    assert "does NOT prove encoder ratio/pole-pairs" in s
    assert "sensor_cal_measured_encoder_offset_deg" in s
    assert "candidate_config.encoder_offset_deg = 0U;\n                        candidate_config.encoder_ratio" not in s

def test_gear_ratio_is_roundtripped_and_persisted():
    c = src("Src/vesc_config_compat.c")
    h = src("Src/foc_motor.h")
    r = src("Src/runtime_control.c")
    assert "si_gear_ratio_milli" in h
    assert "gear_ratio_wire" in c
    assert "EEPROM_LEFT_GEAR_RATIO_MILLI" in r
    assert "EEPROM_RIGHT_GEAR_RATIO_MILLI" in r

def test_v20_fixed_gear_ratio_literal_removed():
    c = src("Src/vesc_config_compat.c")
    anchor = "b[i++] = (uint8_t)(c->foc_motor_pole_pairs * 2U);"
    tail = c.split(anchor, 1)[1][:180]
    assert "si_gear_ratio_milli" in tail

def test_full_brake_and_stop_are_not_aliased():
    p = src("Src/vesc_protocol.c")
    assert "SET_DUTY(0) is VESC Tool Full Brake" in p
    assert "const bool run=true" in p

def test_brake_uses_standard_speed_opposition_without_v21_deadband():
    f = src("Src/foc_motor.c")
    assert "speed_q4 > -32 && speed_q4 < 32" not in f
    assert "if (speed_q4 < 0) iq_target = mag;" in f
    assert "else if (speed_q4 > 0) iq_target = (int16_t)-mag;" in f

def test_realtime_current_validity_uses_proven_v20_window():
    p = src("Src/vesc_protocol.c")
    assert "VESC_CURRENT_HOLD_MAX_MS 25U" in p
    assert "MotorControl_CurrentMeasurementValid(left)" in p

def test_vdq_uses_live_vbus_scaling():
    p = src("Src/vesc_protocol.c")
    m = src("Src/main.c")
    assert "Vd/q = mod_d/q * (2/3) * Vbus" in p
    assert m.find("batVoltageCalib = batVoltage") < m.find("VescProtocol_CurrentTelemetrySample();")
