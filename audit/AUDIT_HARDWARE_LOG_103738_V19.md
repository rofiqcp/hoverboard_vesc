# Audit Hardware Log 2026-08-13 10:37:38 -> V19

Source bundle: `vesc_full_test_20260813_103738.zip` (tester V18).

## Overall

Hardware run completed with PASS=57, FAIL=4, WARN=1, SKIP=2. The important
progress is that serial, current calibration, both sensor detects, current modes,
and RIGHT speed already work. Remaining failures are concentrated around LEFT
commutation/speed and position telemetry/control.

## LEFT evidence

- Duty +0.03/-0.03 reached approximately +0.034/-0.026 actual duty, but ERPM
  remained roughly -1..+1.
- Current +0.50/-0.50 A produced GET_VALUES Iq about +0.49/-0.50 A, but ERPM
  stayed roughly -1..+1.
- Speed +/-900 eRPM drove Iq toward roughly +/-6.9 A while measured speed stayed
  near zero/wrong sign.
- Position drove high Iq/duty but did not track reliably.

Interpretation: current-loop scaling is alive, but LEFT electrical rotor phase was
not synchronized consistently enough for useful torque. Source audit found the
V18 sync equation omitted the configured encoder offset when creating the
power-on electrical zero. V19 corrects that equation and verifies it numerically.

LEFT detect also used encoder-ratio fallback after seeing all A/B states. V19
keeps encoder ratio separate from physical motor pole-pairs so commissioning can
change the encoder electrical transform without rewriting Motor Poles.

## RIGHT evidence

- RIGHT Current +/-0.5 A and RPM +/-900 eRPM produce signed motion.
- RIGHT 3% duty reaches about +/-0.026 actual modulation. Low speed during a short
  3% voltage command is not, by itself, proof that duty scaling is wrong.
- RIGHT position motor activity existed, but V18 GET_VALUES position remained 0;
  V19 replaces this position telemetry stub with Hall-derived mechanical ticks and
  the shared 0..360 mapping.

## Detect evidence

- LEFT encoder terminal detect: SUCCESS, A/B mask 0x0F, clean transitions, ratio
  fallback to 15.
- RIGHT Hall terminal detect: SUCCESS, Hall mask 0x7E.

V19 therefore does not rewrite the Hall/current-cal algorithms that are already
working. It adds the missing post-detect electrical sync and persistence flow.
