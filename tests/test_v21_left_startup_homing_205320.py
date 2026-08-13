from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_boot_sync_proves_phase_before_accepting_zero():
 r=s("Src/runtime_control.c")
 a=r.index("static int16_t encoder_alignment_target_current")
 b=r.index("static void Homing_SetDefaults",a)
 block=r[a:b]
 assert "1.00 A D-axis phase-0 lock" in block
 assert "SENSOR_CAL_ENCODER_PROBE_Q4" in block and "probe_delta_counts" in block
 assert "encoder_alignment_probe_matches" in block
 assert "encoderAlign.probe_direction = -1" in block
 assert "MotorSensor_SyncEncoderElectricalPhase(state, 0U)" in block
 assert block.index("direction_proved") < block.rindex("MotorSensor_SyncEncoderElectricalPhase(state, 0U)")
 assert "RuntimeSettings_Save" not in block
def test_detect_auto_launches_full_homing_and_boot_rehomes_after_cal():
 r=s("Src/runtime_control.c")
 assert "RuntimeControl_StartHomingCalibration(true)" in r
 assert "if (left) homingOnBootLeft = true;" in r
 assert "RuntimeControl_SteeringReady" in r
def test_persistent_encoder_offset_is_zero():
 c=s("Src/vesc_config_compat.c")
 assert "r->encoder_offset_deg = 0U;" in c
 r=s("Src/runtime_control.c")
 assert "snap->result.encoder_offset_deg = 0U;" in r
def test_integrated_detect_waits_mechanical_ready():
 p=s("Src/vesc_protocol.c")
 assert "RuntimeControl_SteeringReady(true)" in p
