#!/usr/bin/env python3
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'tests/test_v15_vescflow_v1isr.py'
s=p.read_text()
old='''# Failed sweep retry must NOT move the transaction edge baselines. That was the
# subtle V14/V15-pre-release bug that made a healthy loaded encoder fail forever.
retry_anchor = runtime.index('/* V15: restart only the displacement window.')
retry_end = runtime.index('sensorCal.last_motion_tick = now;', retry_anchor)
retry_block = runtime[retry_anchor:retry_end]
assert 'encoder_session_valid_edges_start =' not in retry_block
assert 'encoder_session_invalid_transitions_start =' not in retry_block
assert 'sensorCal.encoder_start = state->position_ticks;' in retry_block
'''
new='''# V21 hardware log 17:49: transaction-wide valid-edge evidence must survive a
# retry, but the directional transition window must be rebased and restart forward.
retry_anchor = runtime.index('/* V21: every retry is a complete')
retry_end = runtime.index('sensorCal.last_motion_tick = now;', retry_anchor)
retry_block = runtime[retry_anchor:retry_end]
assert 'encoder_session_valid_edges_start =' not in retry_block
assert 'encoder_session_invalid_transitions_start =' not in retry_block
assert 'sensor_cal_reset_capture_tables(state, sample, false);' in retry_block
assert 'sensorCal.sweep_direction = 1;' in retry_block
'''
if s.count(old)!=1:
    raise SystemExit(f'v15 retry contract expected one old block, got {s.count(old)}')
p.write_text(s.replace(old,new,1))
print('V15 retry contract migrated to V21 hardware evidence')
