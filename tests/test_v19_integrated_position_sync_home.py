from pathlib import Path

root = Path(__file__).resolve().parents[1]
runtime = (root / "Src/runtime_control.c").read_text()
runtime_h = (root / "Src/runtime_control.h").read_text()
sensor = (root / "Src/motor_sensor.c").read_text()
proto = (root / "Src/vesc_protocol.c").read_text()
config = (root / "Src/vesc_config_compat.c").read_text()
eeprom_h = (root / "Src/eeprom.h").read_text()
tester = (root / "tools/vesc_full_test.py").read_text()
foc = (root / "Src/foc_motor.c").read_text()

# LEFT AB electrical synchronization: the configured offset must participate in
# the power-on zero equation; storing raw alone recreates the V18 phase error.
sync = sensor[sensor.index("uint16_t MotorSensor_SyncEncoderElectricalPhase"):
              sensor.index("static int16_t electrical_phase_q16_to_q6")]
assert "state->encoder_electrical_phase_q16 +" in sync
assert "state->encoder_offset_phase_q16 -" in sync
assert "desired_phase_q16" in sync

# FOC electrical calibration and mechanical 0..360 homing are independent state.
for token in ("SteeringCalibration", "right_zero_ticks", "span_ticks", "calibrated", "homed"):
    assert token in runtime
for token in ("RuntimeControl_PositionTargetTicks", "RuntimeControl_PositionDeg",
              "RuntimeControl_StartHomingCalibration", "RuntimeControl_StartHomingOne",
              "RuntimeControl_RequestEncoderSync", "RuntimeControl_SetHomingOnBoot"):
    assert token in runtime_h
# 0 and 360 must be clamped endpoints, not modulo-equivalent position commands.
pos = runtime[runtime.index("bool RuntimeControl_PositionTargetTicks"):
              runtime.index("float RuntimeControl_PositionDeg")]
assert "if (deg < 0.0f) deg = 0.0f" in pos
assert "if (deg > 360.0f) deg = 360.0f" in pos
assert "% 360" not in pos

# Hard-stop homing requires current + low speed + encoder no-motion over debounce,
# and full calibration persists a signed measured span before returning center.
home_start = runtime.index("static bool homing_hardstop_detected")
home = runtime[home_start:
               runtime.index("static void refresh_master_enable", home_start)]
for token in ("current_high && speed_low && no_motion", "homingDebounceMs",
              "ESC_HOMING_CAL_RIGHT", "ESC_HOMING_CAL_LEFT", "span_ticks = span",
              "ESC_HOMING_RETURN_CENTER", "settingsDirty = true"):
    assert token in home

# Boot homing is a one-stop re-reference only after a previously calibrated span.
boot = runtime[runtime.index("bool RuntimeControl_SetHomingOnBoot"):
               runtime.index("static void homing_service_pending_after_sync")]
assert "cal->calibrated == 0U || cal->span_ticks == 0" in boot
assert "RuntimeSettings_Save()" in boot

# Encoder ratio is independent of physical motor pole pairs. GUI Motor Poles
# remains physical poles (2*pair), while FOC Encoder Ratio scales A/B angle.
assert "r->encoder_ratio != 0U ? r->encoder_ratio : c->foc_motor_pole_pairs" in config
assert "c->foc_motor_pole_pairs = (uint8_t)(motor_poles_wire / 2U)" in config
assert "r->encoder_ratio = (uint8_t)ratio_rounded" in config
assert "candidate_config.encoder_ratio = sensorCal.detected_pole_pairs" in runtime
assert "params->foc_motor_pole_pairs = sensorCal.detected_pole_pairs" not in runtime

# EEPROM image owns encoder ratio plus both mechanical calibration records.
assert "NB_OF_VAR             ((uint8_t)158U)" in eeprom_h
assert "EEPROM_LEFT_GEAR_RATIO_MILLI" in runtime
assert "EEPROM_RIGHT_GEAR_RATIO_MILLI" in runtime
for token in ("EEPROM_LEFT_ENCODER_RATIO", "EEPROM_RIGHT_ENCODER_RATIO",
              "EEPROM_LEFT_STEER_ZERO_LO", "EEPROM_LEFT_STEER_SPAN_LO",
              "EEPROM_RIGHT_STEER_ZERO_LO", "EEPROM_RIGHT_STEER_SPAN_LO"):
    assert token in runtime

# Rotor Position VESC Tool compatibility: COMM_SET_DETECT selects display source;
# COMM_ROTOR_POSITION is streamed on ~10 ms cadence, only one motor at a time.
for token in ("C_SET_DETECT=11", "C_ROTOR_POSITION=22", "rotor_position_stream_service",
              ">=10U", "display_position_mode_left=0U", "display_position_mode_right=0U"):
    assert token in proto

# Integrated board commissioning flow is explicit LEFT encoder -> RIGHT Hall ->
# LEFT electrical sync -> EEPROM -> terminal reply. It must not be represented as
# full R/L/flux detection in code/tests.
for token in ("AUTO_DETECT_LEFT_ENCODER", "AUTO_DETECT_RIGHT_HALL",
              "AUTO_DETECT_LEFT_SYNC", "RuntimeSettings_Save()?2:-1", "C_DETECT_APPLY_ALL_FOC"):
    assert token in proto
auto = proto[proto.index("static void auto_detect_service"):proto.index("static void send_print")]
assert "(void)RuntimeSettings_Save()" not in auto

# V19 duty path: modulation target is a current-loop mode with a q-axis voltage
# ceiling derived from the duty target; qmax must not reopen to full voltage.
assert "m_duty_cycle_set_q15" in foc
assert "int16_t q_probe = axis_vmax" in foc
assert "if (qmax > axis_vmax) qmax = axis_vmax" in foc

# Diagnostic/tester contract for the new integrated states.
assert 'TESTER_RELEASE = "V21"' in tester
assert "HBTS_VERSION = 15" in tester
for token in ("encoder_electrical_ready", "steering_calibrated", "steering_homed",
              "logical_position_deg", "homing_on_boot", "auto_detect_stage",
              "integrated_auto_detect", "collect_rotor_positions",
              "COMM_DETECT_APPLY_ALL_FOC", "--homing-calibrate", "--homing-on"):
    assert token in tester
# Python position test must use bounded endpoint semantics too; no circular modulo.
pos_test = tester[tester.index("    def position_test"):tester.index("    def stop_and_verify")]
assert "% 360.0" not in pos_test
assert "bounded 0..360; endpoints distinct" in pos_test

print("V19_INTEGRATED_POSITION_SYNC_HOME_CONTRACT_PASS")
