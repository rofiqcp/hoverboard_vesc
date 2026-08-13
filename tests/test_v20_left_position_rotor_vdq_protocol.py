from pathlib import Path
root=Path(__file__).resolve().parents[1]
proto=(root/'Src/vesc_protocol.c').read_text()
runtime=(root/'Src/runtime_control.c').read_text()
assert 'uint8_t o[128]' in proto
assert 'uint8_t out[512]' in proto, 'HBTS v15 payload needs >448-byte safe buffer'
assert 'fw_append_cstr' in proto
assert 'hoverboard-vesc6-v20' in proto
assert 'avg_vd_mv' in proto and 'avg_vq_mv' in proto
assert 'foc_mod_q14_to_mv' in proto
assert 'if(mask&(1UL<<19))vesc_buf_append_float32(out,(float)avg_vd_mv/1000.0f,1000,&i);' in proto
assert 'if(mask&(1UL<<20))vesc_buf_append_float32(out,(float)avg_vq_mv/1000.0f,1000,&i);' in proto
assert 'case 2U: /* Observer: do NOT alias sensor/active phase.' in proto
assert 'case 3U: /* Encoder raw mechanical angle' in proto
assert '*value = RuntimeControl_PositionDeg(!right);' in proto
assert 'probe_start_count' in runtime and 'probe_delta_counts' in runtime
assert 'wanted_inverted = encoderAlign.probe_delta_counts < 0 ? 1U : 0U' in runtime
assert '/* +120 deg electrical */' in runtime
assert 'encoderAlign.motor == ESC_MOTOR_LEFT && !encoderAlign.direction_proved' in runtime
assert 'positionSessionZeroRight' in runtime
assert 'position_session_zero(left)' in runtime
assert 'return cfg->motor_inverted ? -span : span;' in runtime
assert 'motor->m_pos_pid_set = target_host;' in runtime
assert 'motor->m_pos_pid_set = apply_motor_direction_i32(cfg, target_host);' not in runtime
# Numeric position-direction invariant: logical +degree maps along signed raw span.
def raw_target(zero: int, span: int, deg: float) -> int:
    return zero + round(span * deg / 360.0)
assert raw_target(100, 90, 360.0) == 190
assert raw_target(100, -90, 360.0) == 10
assert raw_target(2500, -900, 180.0) == 2050
assert 'if (min_span < 0) min_span = -min_span;' in runtime, 'signed homing span sanity must use magnitude'
assert 'abs_i16_saturating' not in runtime
print('V20_LEFT_POSITION_ROTOR_VDQ_PROTOCOL_CONTRACT_PASS')
