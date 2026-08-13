# V5 Changes

## Hardware-log driven fixes
- Fixed V4 current-zero false rejection seen in `vesc_full_test_20260812_171916.zip`.
- Replaced raw min/max span as calibration pass/fail criterion with robust 64-sample block-mean stability.
- Increased pre-calibration settling from 256 to 512 current-loop samples.
- Kept 2048-sample arithmetic mean as the actual ADC zero reference.
- Added candidate mean, raw span and block-mean span separately to HBTS diagnostics.
- Added decoded current-calibration failure reasons.
- Added up to two automatic tester retries for transient block-mean instability only.
- Structural calibration failures are never bypassed.

## Observation / actuation split
- Hall, encoder, position and RPM now keep updating while current calibration is settling or failed.
- Current measurements are forced to zero and PWM/MOE stays OFF until current offsets are valid.
- This prevents stale `hall_raw=000` diagnostics from being mistaken for a physical Hall failure.

## Tester improvements
- HBTS diagnostic protocol bumped to v4.
- Hall detection now performs a valid-state preflight after current calibration.
- Motion tests are dependency-aware: if current calibration or sensor detect fails, dependent tests are SKIP rather than repeated misleading FAILs.
- Current-calibration test logs `candidate_mean`, `raw_span`, `block_span`, residual and human-readable failure reasons.
- Added `--current-cal-retries` (default 2 retries).

## Regression
- Host regression increased to 12 tests.
- New robust-calibration test reproduces a >300-count isolated rlB spike and proves it is tolerated when 64-sample block means remain stable.
- The same test proves sustained block drift is rejected.
- Cortex-M3 codegen audit now includes the complete DMA current ISR hot path and verifies zero `__aeabi_*div` helper calls.
