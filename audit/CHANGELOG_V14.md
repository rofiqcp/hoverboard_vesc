# CHANGELOG V14

Baseline: V13 + hardware log `vesc_full_test_20260813_002319.zip`.

## Hall RIGHT

- Removed V12/V13 fixed-sector vote/permutation Hall finalizer.
- Added VESC-6.00-style circular sin/cos accumulator per raw Hall code.
- Kept 3 forward + 3 reverse current-controlled sweeps.
- Require >30 samples for each valid Hall state.
- Require exactly two missing raw codes and additionally require those to be 000/111 for this board.
- Build runtime Hall sequence from measured circular electrical angles.
- Store/send exact measured VESC Hall table in terminal detect snapshot.

## Encoder LEFT

- Kept PB6/PB7 TIM4 A/B path from V13.
- Strict ratio inference remains preferred.
- Removed net displacement requirement from guarded configured-pole-pair fallback.
- Raised integrity threshold to >=32 valid TIM4 edges per acquisition window.
- Require all four quadrature states and low invalid transition rate.
- Preserve configured encoder direction on fallback; expose fallback in diagnostics.
- Kept current-controlled first-ARM incremental electrical-zero alignment.

## Current / telemetry

- No fake idle current.
- Tester explicitly distinguishes `STANDARD_ZERO_VALID_BRIDGE_OFF` from unavailable active telemetry.
- Standard current path and VESC field layout unchanged because latest hardware log proves it becomes non-zero during Detect.

## Startup buzzer

- BAT_LVL1 warning qualified with 2 s post-melody grace + 500 ms persistence.
- Genuine sustained low-battery warning remains enabled.

## SET / ARM regression

- Added V14 post-detect static contract covering Duty/Current/RPM/POS side routing, per-side ARM, calibration commit, encoder alignment, and absence of a both-motors-calibrated gate.

## Validation

- 18/18 host regression PASS.
- Cortex-M3 hot-path division-helper audit PASS (current loop / encoder update / DMA current ISR = 0 helpers).
- Python tester protocol/HBTS self-test PASS.
- Physical motor run still requires the target board and is not claimed by host validation.
