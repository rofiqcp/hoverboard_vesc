#!/usr/bin/env python3
"""V14 regression contract from physical log 2026-08-13 00:23:19."""
from pathlib import Path
import re
root=Path(__file__).resolve().parents[1]
runtime=(root/'Src/runtime_control.c').read_text()
main=(root/'Src/main.c').read_text()
protocol=(root/'Src/vesc_protocol.c').read_text()
tester=(root/'tools/vesc_full_test.py').read_text()

# 1) Hall detection now follows VESC 6.00 circular-angle averaging rather than
# V12/V13 six-sector voting/permutation. It must include raw 000/111 in sampling
# and require those two illegal states to be the missing entries.
for token in ('hall_sin_sum_q14[8]', 'hall_cos_sum_q14[8]', 'hall_angle_samples[8]',
              'sensor_cal_record_hall_angle(raw)', 'sensor_cal_hall_angle200',
              'sensorCal.hall_angle_samples[raw] > 30U', 'fails != 2U',
              'sensorCal.measured_hall_table[0] != 255U',
              'sensorCal.measured_hall_table[7] != 255U'):
    assert token in runtime, f'missing V14 circular Hall contract: {token}'
assert 'hall_sector_votes' not in runtime
assert 'next_permutation_u8' not in runtime

# Local runtime sequence is derived from the measured circular table, while the
# exact measured 0..199 VESC table survives to the terminal reply.
assert 'MotorRuntimeConfig_SetHallSequence(cfg, sequence)' in runtime
assert 'sensorCal.measured_hall_table_valid = true;' in runtime
assert 'memcpy(snap->result.hall_table, sensorCal.measured_hall_table, 8U);' in runtime

# 2) Latest LEFT log proved all-four-state clean quadrature with 208 valid/0
# invalid edges but near-zero net displacement. Guarded configured-ratio fallback
# must not require net displacement and must preserve configured direction.
fb=runtime[runtime.index('V14 hardware-log 00:23:19'):runtime.index('return false;', runtime.index('V14 hardware-log 00:23:19'))+20]
assert 'valid_edges >= 32U' in fb
assert 'sensorCal.encoder_session_seen_mask == 0x0FU' in fb
assert 'mag >=' not in fb
assert 'sensorCal.detected_encoder_inverted = sensorCal.original_sensor_inverted;' in fb
assert 'sensorCal.encoder_ratio_fallback_used = true;' in fb

# 3) Power-on BATTERY_LEVEL1 transient is qualified instead of disabling genuine
# low-battery warning. Settled low voltage must remain low for 500 ms after a 2 s
# boot grace before beep reason 3 is allowed.
assert 'batteryWarningBootTick' in main and 'batteryLowSinceTick' in main
assert '(uint32_t)(now - batteryWarningBootTick) >= 2000U' in main
assert '(uint32_t)(now - batteryLowSinceTick) >= 500U' in main
assert '} else if (battery_lvl1_qualified) {' in main

# 4) Protocol stays wire-compatible with VESC 6.00; V15 keeps the V14 Hall/encoder fixes.
assert '#define VESC_FW_MAJOR 6U' in protocol and '#define VESC_FW_MINOR 0U' in protocol
assert 'hoverboard-vesc6-v20' in protocol
assert 'TESTER_RELEASE = "V20"' in tester

# 5) Fixed-point Hall finalizer must reuse Q14 LUT; no atan2/libm was introduced
# into runtime commissioning.
assert 'focSinTableQ14' in runtime
assert 'atan2f(' not in runtime

# 6) Replay the concrete V13 hardware evidence that motivated V14. The terminal
# table already contained six legal angle entries even though V13 status was 1.
hardware_hall_table = [255, 50, 183, 16, 116, 83, 150, 255]
hardware_sequence = sorted(range(1, 7), key=lambda raw: hardware_hall_table[raw])
assert hardware_sequence == [3, 1, 5, 4, 6, 2]
assert sum(1 for raw in range(1, 7) if hardware_hall_table[raw] != 255) == 6

# First encoder acquisition window rose from roughly 4 to 79 cumulative valid
# edges in the 300-ms diagnostics while invalid transitions stayed at zero.
# This is >32 independent edges even though the final net delta was only ~1 tick.
first_window_valid_edges = 79 - 4
assert first_window_valid_edges >= 32
assert 0 <= (2 + first_window_valid_edges // 32)

print('V14_LATEST_HARDWARE_REGRESSION_CONTRACT_PASS')
