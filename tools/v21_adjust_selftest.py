#!/usr/bin/env python3
from pathlib import Path
p = Path(__file__).resolve().parents[1] / 'tests/test_v21_regressions.py'
s = p.read_text()
old = 'assert "do NOT prove encoder ratio/pole-pairs" in s'
new = 'assert "does NOT prove encoder ratio/pole-pairs" in s'
if s.count(old) != 1:
    raise SystemExit(f'V21 self-test wording location changed: {s.count(old)}')
p.write_text(s.replace(old, new, 1))
print('V21 self-test wording corrected')
