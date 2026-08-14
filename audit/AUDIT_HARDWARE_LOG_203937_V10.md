# Audit Hardware Log 2026-08-12 20:39:37 → V10

Input: `vesc_full_test_20260812_203937.zip` (tester release V9).

## Executive result

The V9 run reached the controller correctly: local ID 10, virtual RIGHT ID 11, UART framing, FW version, current-zero calibration, ADC, configuration round-trip and parser recovery all passed. The only first-order hardware failures were LEFT encoder detect and RIGHT Hall detect, both aborted by FAST CURRENT GUARD. All motion tests were correctly skipped after the failed sensor proof.

PASS=16, FAIL=2, WARN=3, SKIP=16.

## Evidence that is already healthy

- Current-zero calibration: VALID.
- ADC1/ADC2 hardware calibration: PASS.
- LEFT encoder configuration visible: CPR 2048, pole pairs 15.
- RIGHT Hall idle state: `010`, a valid non-000/non-111 Hall state.
- ISR overrun count: zero; observed max about 3365 cycles against 4000-cycle deadline.
- Idle current after calibration is close to zero.
- Local/right UUID and controller IDs are distinct and virtual CAN discovery is correct.

## Detect failure

Both detector calls returned FAST CURRENT GUARD almost immediately. The V9 diagnostic response was taken after abort cleanup, so its live `detect_target`, Vd/Vq and measured target values had already returned to zero. That is insufficient to identify whether the original high-current sample came from ADC timing, polarity, mapping or a genuine PI transient.

V10 therefore preserves the first guard-trip sample in firmware before cleanup and exports it in HBTS v9.

## Manual VESC Tool symptom supplied by user

The user observed that moving LEFT by hand could display `Imotor` while viewing RIGHT virtual CAN, and Id/Iq/Imotor could remain stuck after movement. This occurred outside the script's passive-spin window, so the V9 ZIP cannot reproduce the exact GUI event. Source audit did reveal two mechanisms that can produce the symptom:

1. released-bridge raw current observation could enter standard VESC current telemetry;
2. a cached average could be reused when no new current sample existed.

Both are removed in V10 standard telemetry. Raw passive observation remains available only through HBTS.

## Stock-board timing discrepancy found during source comparison

The original hoverboard firmware aligns LEFT and RIGHT phase-current conversions by offsetting TIM8 by one 7.5-cycle phase-current conversion at ADC clock divider 4, giving 80 timer ticks. V9 used divider 6 and a timing constant based on 1.5-cycle sampling while phase ranks remained 7.5-cycle. V10 restores the original phase-current timing geometry.

This is a strong candidate for the detect-time current anomaly but remains a hypothesis until V10 is run on the actual board. The new first-fault snapshot will distinguish it from phase polarity/mapping if the guard still trips.

## V10 next-run decision tree

1. If current-zero fails: troubleshoot analog zero/reference first.
2. If passive LEFT/RIGHT isolation leaks standard current with both MOE off: firmware routing/telemetry regression.
3. If zero SET routing increments the peer set counter: virtual-CAN routing regression.
4. If Detect guard trips: inspect preserved raw ADC/offset/delta, Id/Iq, forced phase and duty from the first fault sample.
5. If Detect passes but Duty/Current/RPM/POS fails: inspect command route count, arm reject, sensor validity, mode targets, peer bridge state and fault fields in the corresponding motion log.
