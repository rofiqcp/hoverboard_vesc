#!/usr/bin/env python3
from pathlib import Path

def rep(path, old, new):
    p=Path(path); s=p.read_text()
    if s.count(old)!=1:
        raise SystemExit(f'{path}: expected one anchor, got {s.count(old)}')
    p.write_text(s.replace(old,new,1))

p=Path('Src/runtime_control.c'); r=p.read_text()
anchor='#define SENSOR_CAL_VESC_POLE_PAIRS_MAX           60U'
if r.count(anchor)!=1: raise SystemExit('pole-pair macro anchor')
r=r.replace(anchor,
    '#define SENSOR_CAL_NATIVE_ENCODER_FORWARD_SWEEPS 3U\n'
    '#define SENSOR_CAL_NATIVE_ENCODER_REVERSE_SWEEPS 3U\n'+anchor,1)
r=r.replace('SENSOR_CAL_ENCODER_FORWARD_SWEEPS','SENSOR_CAL_NATIVE_ENCODER_FORWARD_SWEEPS')
r=r.replace('SENSOR_CAL_ENCODER_REVERSE_SWEEPS','SENSOR_CAL_NATIVE_ENCODER_REVERSE_SWEEPS')
r=r.replace('''    if (sensor_type == MOTOR_SENSOR_ENCODER_AB && sensorCal.vesc_wire_detect)\n        return SENSOR_CAL_VESC_ENCODER_CYCLES;''',
            '''    if (sensor_type == MOTOR_SENSOR_ENCODER_AB && sensorCal.vesc_wire_detect)\n        return SENSOR_CAL_ENCODER_PROBE_MAX_COUNT;''')
r=r.replace('    const bool sequence_ready = sensor_cal_finalize_encoder_sequence(&candidate, state);\n','')
p.write_text(r)

rep('tests/test_v12_commissioning_contract.py',
'''# 3) Encoder VESC detect must complete before timeout as well.\nenc_cycles = macro_int(runtime, "SENSOR_CAL_VESC_ENCODER_CYCLES")\nenc_ms = (enc_cycles * 5760 + rate_q4_per_ms - 1) // rate_q4_per_ms\nassert enc_cycles == 6 and align_ms + enc_ms < timeout_ms''',
'''# 3) Encoder VESC detect follows upstream-shaped local +/-120 degree probes.\nprobe_q4 = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_Q4")\nprobe_max = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_MAX_COUNT")\nprobe_settle = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_SETTLE_MS")\nenc_ms = probe_max * (((probe_q4 + rate_q4_per_ms - 1) // rate_q4_per_ms) + probe_settle)\nassert probe_q4 == 1920 and probe_max >= 8 and align_ms + enc_ms < timeout_ms''')
rep('tests/test_v13_hardware_regressions.py',
    "assert 'does NOT prove encoder ratio/pole-pairs' in runtime",
    "assert 'cfg->encoder_cpr + den / 2U' in runtime and 'encoder_probe_ratio_sum' in runtime")
rep('tests/test_v14_latest_hardware_regressions.py',
    "assert 'does NOT prove encoder ratio/pole-pairs' in runtime",
    "assert 'cfg->encoder_cpr + den / 2U' in runtime and 'encoder_probe_ratio_sum' in runtime")
rep('tests/test_v17_units_config_speed_contract.py',
'''for token in ('SENSOR_CAL_ENCODER_FORWARD_SWEEPS 3U',\n              'SENSOR_CAL_ENCODER_REVERSE_SWEEPS 3U',\n              'encoder_transition_mid[16]', 'encoder_direction_normal_score',\n              'encoder_direction_inverted_score', 'encoder_direction_proved',\n              'sensor_cal_encoder_direction_proof'):\n    assert token in runtime, token\nassert 'sensorCal.encoder_forward_delta = state->position_ticks - sensorCal.encoder_start;' in runtime''',
'''for token in ('SENSOR_CAL_ENCODER_PROBE_Q4            1920',\n              'SENSOR_CAL_ENCODER_PROBE_MIN_VALID',\n              'encoder_direction_normal_score', 'encoder_direction_inverted_score',\n              'encoder_direction_proved', 'sensor_cal_encoder_probe_record',\n              'LeftEncoder_GetCount()'):\n    assert token in runtime, token\nassert 'SENSOR_CAL_NATIVE_ENCODER_FORWARD_SWEEPS 3U' in runtime''')
rep('tests/test_v21_hwlog_174940.py',
    "    assert 'raw_reverse = total_delta - (int64_t)sensorCal.encoder_forward_delta' in runtime",
    "    assert 'static const int8_t directions[4] = {1, -1, -1, 1};' in runtime\n    assert 'if (mag < 3U' in runtime")
rep('tests/test_v21_hwlog_174940.py',
    "    assert '(mag_reverse > mag_forward) ? -raw_reverse : raw_forward' in runtime",
    "    assert 'const uint32_t ratio = ((uint32_t)cfg->encoder_cpr + den / 2U) / den;' in runtime")
rep('tests/test_v21_regressions.py',
    '    assert "does NOT prove encoder ratio/pole-pairs" in s',
    '    assert "cfg->encoder_cpr + den / 2U" in s\n    assert "encoder_ratio_fallback_used = false" in s')
rep('tests/test_resource_contract.py',
    "assert 'SENSOR_CAL_VESC_ENCODER_CYCLES' in runtime",
    "assert 'SENSOR_CAL_ENCODER_PROBE_Q4' in runtime and 'SENSOR_CAL_ENCODER_PROBE_MAX_COUNT' in runtime")

print('encoder probe contracts migrated')
