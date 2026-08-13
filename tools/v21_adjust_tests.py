#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
p = root / 'tests/test_v14_post_detect_arm_contract.py'
s = p.read_text()
old = "    'duty_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);' in vesc and 'const bool run=normalized!=0;' in vesc,"
new = "    # V21 follows VESC Tool: SET_DUTY(0) is Full Brake and therefore remains armed.\n    'duty_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);' in vesc and 'const bool run=true;' in vesc,"
if s.count(old) != 1:
    raise SystemExit('legacy duty-side-arm contract location changed')
p.write_text(s.replace(old, new, 1))
print('V21 legacy contracts adjusted')
