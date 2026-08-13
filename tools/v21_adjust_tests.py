#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]

def replace_exact(path, old, new, label):
    p = root / path
    s = p.read_text()
    if s.count(old) != 1:
        raise SystemExit(f'{label}: expected exactly one match, got {s.count(old)}')
    p.write_text(s.replace(old, new, 1))

# V14 post-detect contract: VESC Tool Full Brake is SET_DUTY(0), so zero duty
# remains an armed command. Stop remains the separate zero-current/e-stop path.
replace_exact(
    'tests/test_v14_post_detect_arm_contract.py',
    "    'duty_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);' in vesc and 'const bool run=normalized!=0;' in vesc,",
    "    # V21 follows VESC Tool: SET_DUTY(0) is Full Brake and therefore remains armed.\n    'duty_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);' in vesc and 'const bool run=true;' in vesc,",
    'v14 duty-side-arm')

# Resource contract: V21 keeps the last accepted steady current/Vdq sample for a
# bounded 100 ms through a transient low-side sampling-invalid instant. Bridge
# release still clears immediately, so there is no cross-node stale telemetry.
replace_exact(
    'tests/test_resource_contract.py',
    "assert 'VESC_CURRENT_HOLD_MAX_MS 25U' in protocol",
    "assert 'VESC_CURRENT_HOLD_MAX_MS 100U' in protocol",
    'resource telemetry hold')

# V13 originally introduced a configured-pole-pair fallback for a loaded encoder.
# V21 intentionally removes it: clean quadrature proves sensor health, not ratio.
p = root / 'tests/test_v13_hardware_regressions.py'
s = p.read_text()
old = """# 2) Loaded-wheel encoder fallback: first prefer strict ratio, then allow the
# configured pole-pair value only with clean TIM4 + all-four-state evidence.
for token in ('encoder_ratio_fallback_used', 'configured_pp',
              'valid_edges >= 32U', 'invalid_edges <= invalid_limit',
              'sensorCal.encoder_session_seen_mask == 0x0FU'):
    assert token in runtime, f'missing encoder fallback guard: {token}'
"""
new = """# 2) V21 keeps TIM4/quadrature health evidence but never turns the already
# configured pole-pair into a fake measured encoder ratio. Ratio must come from
# commanded electrical motion versus measured A/B motion.
assert 'encoder_ratio_fallback_used' in runtime
assert 'does NOT prove encoder ratio/pole-pairs' in runtime
assert 'configured_pp' not in runtime
assert 'sensorCal.encoder_ratio_fallback_used = false;' in runtime
"""
if s.count(old) != 1:
    raise SystemExit('v13 encoder fallback contract location changed')
p.write_text(s.replace(old, new, 1))

# V14 hardware regression: preserve the real observation (healthy quadrature with
# near-zero net displacement), but update the conclusion. It now must NOT pass as
# a measured ratio merely because the configured motor happens to say 15 pp.
p = root / 'tests/test_v14_latest_hardware_regressions.py'
s = p.read_text()
old = """# 2) Latest LEFT log proved all-four-state clean quadrature with 208 valid/0
# invalid edges but near-zero net displacement. Guarded configured-ratio fallback
# must not require net displacement and must preserve configured direction.
fb=runtime[runtime.index('V14 hardware-log 00:23:19'):runtime.index('return false;', runtime.index('V14 hardware-log 00:23:19'))+20]
assert 'valid_edges >= 32U' in fb
assert 'sensorCal.encoder_session_seen_mask == 0x0FU' in fb
assert 'mag >=' not in fb
assert 'sensorCal.detected_encoder_inverted = sensorCal.original_sensor_inverted;' in fb
assert 'sensorCal.encoder_ratio_fallback_used = true;' in fb
"""
new = """# 2) Latest LEFT log proved all-four-state clean quadrature with many valid/0
# invalid edges but near-zero net displacement. V21 correctly treats that only as
# sensor-health evidence; it must not fabricate configured pole-pairs as measured ratio.
assert 'does NOT prove encoder ratio/pole-pairs' in runtime
assert 'configured_pp' not in runtime
assert 'sensorCal.encoder_ratio_fallback_used = false;' in runtime
assert 'sensor_cal_measured_encoder_offset_deg' in runtime
"""
if s.count(old) != 1:
    raise SystemExit('v14 loaded-encoder contract location changed')
p.write_text(s.replace(old, new, 1))

# V15 SET contract: V21 deliberately supersedes V17 zero-duty release semantics.
replace_exact(
    'tests/test_v15_vescflow_v1isr.py',
    "# V17 refines V15: all non-zero primary SETs still route directly, while exact\n# zero Duty/Current/RPM has VESC stop semantics rather than holding MOE active.\nassert 'const bool run=normalized!=0;' in vesc",
    "# V21 matches VESC Tool: exact zero Duty is Full Brake and remains armed;\n# zero Current/RPM retain their release/stop semantics.\nassert 'const bool run=true;' in vesc",
    'v15 zero-duty contract')
replace_exact(
    'tests/test_v15_vescflow_v1isr.py',
    "for token in ('encoder_session_valid_edges_start', 'encoder_session_invalid_transitions_start',\n              'encoder_session_seen_mask', 'sensorCal.encoder_session_seen_mask == 0x0FU'):",
    "for token in ('encoder_session_valid_edges_start', 'encoder_session_invalid_transitions_start',\n              'encoder_session_seen_mask'):",
    'v15 session evidence contract')

# V18 brake polarity contract: preserve moving-speed sign opposition and require
# the new zero-speed deadband that prevents +/- Hall RPM chatter from flipping Iq.
replace_exact(
    'tests/test_v18_rpm_current_brake_contract.py',
    "assert 'sample->speed_rpm_q4 < 0 ? mag : (int16_t)-mag' in foc_c",
    "assert 'speed_q4 > -32 && speed_q4 < 32' in foc_c\nassert 'speed_q4 < 0 ? mag : (int16_t)-mag' in foc_c",
    'v18 brake deadband contract')

# V19 EEPROM image grows by two words for independent LEFT/RIGHT gear ratio.
replace_exact(
    'tests/test_v19_integrated_position_sync_home.py',
    'assert "NB_OF_VAR             ((uint8_t)156U)" in eeprom_h',
    'assert "NB_OF_VAR             ((uint8_t)158U)" in eeprom_h\nassert "EEPROM_LEFT_GEAR_RATIO_MILLI" in runtime\nassert "EEPROM_RIGHT_GEAR_RATIO_MILLI" in runtime',
    'v19 EEPROM-size contract')

print('V21 historical contracts migrated to corrected semantics')
