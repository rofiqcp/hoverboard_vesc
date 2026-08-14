#!/usr/bin/env python3
"""Regression for V11 current-zero acceptance policy.

Uses the exact constants parsed from Src/motor.c. It proves that:
1) one large raw spike does not fail a stable offset;
2) a drifting block mean does fail;
3) out-of-range means fail.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
src = (root / "Src/motor.c").read_text()

def const(name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+(\d+)U", src)
    assert m, name
    return int(m.group(1))

N = const("CURRENT_CAL_COLLECT_SAMPLES")
B = const("CURRENT_CAL_BLOCK_SAMPLES")
MAX_BLOCK_SPAN = const("CURRENT_CAL_MAX_BLOCK_MEAN_SPAN_COUNTS")
MIN_MEAN = const("CURRENT_CAL_MIN_MEAN_COUNTS")
MAX_MEAN = const("CURRENT_CAL_MAX_MEAN_COUNTS")
assert N % B == 0

def evaluate(channels):
    fail = 0
    raw_span = []
    block_span = []
    means = []
    for idx, xs in enumerate(channels):
        assert len(xs) == N
        mean = (sum(xs) + N // 2) // N
        means.append(mean)
        raw_span.append(max(xs) - min(xs))
        blocks = []
        for i in range(0, N, B):
            block = xs[i:i+B]
            blocks.append((sum(block) + B // 2) // B)
        bs = max(blocks) - min(blocks)
        block_span.append(bs)
        if not (MIN_MEAN <= mean <= MAX_MEAN):
            fail |= 1 << idx
        if bs > MAX_BLOCK_SPAN:
            fail |= 1 << (idx + 6)
    return fail, means, raw_span, block_span

bases = [2010, 2005, 2018, 2002, 1945, 1930]
stable = []
for ch, base in enumerate(bases):
    xs = [base + ((i * (3 + ch)) % 13) - 6 for i in range(N)]
    stable.append(xs)

# Reproduce the hardware-log shape: one channel has >300-count raw p-p due to
# a rare spike, while the block mean remains stable.
stable[1][777] += 310
fail, means, raw, block = evaluate(stable)
assert raw[1] > 300, raw
assert block[1] <= MAX_BLOCK_SPAN, block
assert fail == 0, (fail, means, raw, block)

# Slow drift across calibration blocks must still fail.
drift = [list(x) for x in stable]
for bi, start in enumerate(range(0, N, B)):
    add = bi * 4
    for j in range(start, start+B):
        drift[1][j] += add
fail2, _, _, block2 = evaluate(drift)
assert fail2 & (1 << (1 + 6)), (fail2, block2)

# Structurally invalid ADC mean remains a hard failure.
bad = [list(x) for x in stable]
bad[0] = [0] * N
fail3, _, _, _ = evaluate(bad)
assert fail3 & 1

print("CURRENT_CAL_ROBUST_POLICY_PASS",
      f"N={N}", f"BLOCK={B}", f"raw_span_rlB={raw[1]}",
      f"block_span_rlB={block[1]}", f"drift_block_span={block2[1]}")
