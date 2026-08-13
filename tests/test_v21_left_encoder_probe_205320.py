from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_vesc_encoder_detect_is_local_120_degree_probe():
 r=s("Src/runtime_control.c")
 assert "SENSOR_CAL_ENCODER_PROBE_Q4            1920" in r
 assert "targets[4]" in r and "-SENSOR_CAL_ENCODER_PROBE_Q4" in r
 assert "cfg->encoder_cpr + den / 2U" in r
 assert "SENSOR_CAL_ENCODER_FORWARD_SWEEPS" not in r
def test_probe_uses_raw_tim4_and_requires_quadrature_quality():
 r=s("Src/runtime_control.c")
 assert "const int32_t raw_now = LeftEncoder_GetCount();" in r
 assert "invalid_edges <= (2U + valid_edges / 16U)" in r
 assert "encoder_ratio_fallback_used = false" in r
def test_electrical_offset_is_detect_result_not_persistent_config():
 r=s("Src/runtime_control.c")
 assert "sensorCal.detected_encoder_offset_deg" in r
 assert "candidate_config.encoder_offset_deg = 0U;" in r
