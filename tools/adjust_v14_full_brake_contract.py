#!/usr/bin/env python3
from pathlib import Path
p=Path(__file__).resolve().parents[1]/'tests/test_v14_post_detect_arm_contract.py'
s=p.read_text()
old="'side_local_feedback': 'prearm_feedback_not_ready(mode, cfg, sample, encoder_aligned)' in runtime,"
new="'side_local_feedback': 'prearm_feedback_not_ready(mode, requested_target, cfg, sample, encoder_aligned)' in runtime and '(mode == ESC_MODE_DUTY && target == 0)' in runtime,"
if s.count(old)!=1:
    raise SystemExit(f'expected one old side_local_feedback contract, found {s.count(old)}')
s=s.replace(old,new,1)
p.write_text(s)
print('V14 full-brake feedback contract migrated')
