# V8 Changelog

## Build hygiene
- Removed unused `motor_current_amp()` from `Src/vesc_protocol.c`; the V7 `-Wunused-function` warning is eliminated.
- Host strict syntax tests still compile with warnings promoted to errors.

## VESC current telemetry
- Keeps VESC-style read/reset averaging for Motor Current, Input Current, Id and Iq.
- Released bridge no longer hard-clears all standard current observations.
- Small plausible idle current/noise is averaged and shown.
- Large passive-spin/common-mode excursions are rejected from standard VESC fields but preserved in HBTS raw diagnostics.
- Empty accumulator returns last proven average; never falls back to instantaneous 8-kHz current.

## LEFT Encoder / RIGHT Hall detect safety
- Commissioning current PI gets conservative Kp/Ki and a small voltage-vector cap.
- Detect current request capped to 2 A on stock board; one-shot tester defaults to 0.5 A.
- New ISR fast-current guard cuts MOE in ~1 ms when measured phase-current grossly exceeds requested commissioning current.
- Guard failure is a commissioning failure, not a 3-second runtime/buzzer fault.
- Guard evidence remains readable in post-detect HBTS until the next detect transaction.

## Tester correctness
- HBTS node ID is validated for every local/right diagnostic request.
- Stale LEFT frames can no longer be mistaken for RIGHT virtual-CAN data.
- Detect progress frames with mismatched node ID are discarded and logged.
- Sensor detect is run before APP mode cycling to avoid readiness noise before commissioning.
- Detect failure log includes target/measured current, Id/Iq, Vd/Vq, raw ADC evidence and fast guard mask.

## Buzzer diagnostics
- HBTS diagnostic version 7 adds current buzzer reason, last buzzer reason and event counter.
- Hard fault beep remains audible only for real hard runtime protection; sensor readiness is silent.

## One-command Linux workflow
`RUN_BUILD_UPLOAD_TEST_LINUX.sh /dev/ttyUSB0` now builds/uploads with PlatformIO, records build output and source SHA-256 manifest, runs the full hardware tester, and injects those files into the final troubleshooting ZIP.
