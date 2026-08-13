from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_pid_persistence_owner():
 r=s("Src/runtime_control.c"); assert "positionPidConfigLeft.kp_q16 = motorConfLeft.p_pid_kp_q16" in r
 v=s("Src/vesc_config_compat.c"); assert "const mc_configuration old_c = *c;" in v and "*c = old_c;" in v
def test_rotor_tester_result_dict():
 t=s("tools/vesc_full_test.py"); i=t.index("def rotor_position_stream_test"); j=t.index("def homing_calibration_test",i); assert "out: dict[str, Any] = {}" in t[i:j]
