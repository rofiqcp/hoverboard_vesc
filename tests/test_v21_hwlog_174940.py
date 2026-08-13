from pathlib import Path
root=Path(__file__).resolve().parents[1]
runtime=(root/'Src/runtime_control.c').read_text()
proto=(root/'Src/vesc_protocol.c').read_text()
tester=(root/'tools/vesc_full_test.py').read_text()

def test_encoder_retry_restarts_forward_and_rebases_direction_window():
    assert 'sensor_cal_reset_capture_tables(state, sample, false);' in runtime
    assert 'sensorCal.sweep_direction = 1;' in runtime
    retry=runtime[runtime.index('V21: every retry is a complete'):]
    assert retry.index('sensorCal.sweep_direction = 1;') < retry.index('sensorCal.last_motion_tick = now;')

def test_encoder_ratio_can_use_unblocked_reverse_half():
    assert 'raw_reverse = total_delta - (int64_t)sensorCal.encoder_forward_delta' in runtime
    assert '(mag_reverse > mag_forward) ? -raw_reverse : raw_forward' in runtime
    assert 'configured_pp' not in runtime

def test_integrated_detect_uses_conservative_one_amp_default():
    assert '#define VESC_AUTO_DETECT_CURRENT_A 1.0f' in proto
    assert proto.count('VESC_AUTO_DETECT_CURRENT_A') >= 4
    assert 'detect_current_internal(false,0.5f)' not in proto
    assert 'detect_current_internal(true,0.5f)' not in proto

def test_rotor_protocol_matrix_uses_upstream_stream_semantics():
    assert 'rotor_samples=self.dev.collect_rotor_positions(node,4,seconds=0.12)' in tester
    matrix=tester[tester.index('def protocol_get_matrix'):tester.index('def zero_set_routing_matrix')]
    assert 'self.dev.rotor_position(node)' not in matrix

def test_release_identity_is_v21():
    assert 'TESTER_RELEASE = "V21"' in tester
    assert 'hoverboard-vesc6-v21' in proto
