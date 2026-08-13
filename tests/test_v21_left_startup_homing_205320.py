from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_boot_sync_is_static_one_amp_no_direction_relearn():
 r=s("Src/runtime_control.c")
 a=r.index("static int16_t encoder_alignment_target_current")
 b=r.index("static void Homing_SetDefaults",a)
 block=r[a:b]
 assert "1.00 A D-axis phase-0 lock" in block
 assert "+120" not in block and "probe_delta_counts" not in block
 assert "MotorSensor_SyncEncoderElectricalPhase(state, 0U)" in block
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
