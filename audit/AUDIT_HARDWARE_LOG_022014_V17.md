# Audit hardware log 2026-08-13 02:20:14 -> V17

Source: `vesc_full_test_20260813_022014.zip`, produced by V16.

## What is now proven working

### Connection and standard telemetry

V16 restored VESC transport liveness. `COMM_GET_VALUES` and custom HBTS replies
were present before the later speed test interruption.

### Current command/scaling

For LEFT `SET_CURRENT +0.50 A`:

| sample | wire command | VESC Iq | diagnostic Iq | VESC input current |
|---|---:|---:|---:|---:|
| settled 1 | +500 | +0.47 A | +0.504 A | 0.14 A |
| settled 2 | +500 | +0.48 A | +0.494 A | 0.15 A |
| settled 3 | +500 | +0.51 A | +0.544 A | 0.17 A |

For `SET_CURRENT -0.50 A`, settled standard Iq is approximately -0.46 to
-0.49 A. This validates:

`VESC SET_CURRENT raw / 1000 -> amp target -> internal current units -> GET_VALUES Iq`.

Input current is a DC-link quantity and is not expected to equal phase/Iq current,
especially at low duty.

## V16 failures that matter

### Zero SET did not release

The tester found exact zero Duty/Current/RPM commands could leave the bridge
armed. Release checks after motion therefore failed and MCCONF roundtrip could
time out while runtime still considered a motor energized.

V17 makes exact zero Duty/Current/RPM a release request. Position zero remains
an active target.

### First positive duty overlapped encoder alignment

The first 0.9-s duty window ended with `ALIGNMENT_BUSY`. The following negative
duty test then ran after alignment and passed. V17 tester extends the same first
command when the only blocker is alignment, avoiding a false functional failure.

### Speed polarity mismatch

At `COMM_SET_RPM +900`:

| time | requested | standard eRPM | standard Iq | diagnostic Iq | duty |
|---|---:|---:|---:|---:|---:|
| first sampled | +900 | 0 | 0 A | ~1.39 A | 0.000 |
| next | +900 | -60 | +2.76 A | ~3.98 A | 0.059 |
| next | +900 | -15 | +4.93 A | ~6.17 A | 0.069 |

The wire target itself is correct (`raw=900`). The measured speed sign is wrong
for the positive target, so the speed controller increases torque while seeing a
large positive error. Soon after this window the serial device returns EIO; the
log alone cannot prove causation, so V17 treats the polarity mismatch as the
proven bug and the USB loss as a later hardware/transport symptom.

V17 therefore both improves encoder direction proof and makes the tester abort
on opposite-sign speed before current can continue climbing.

## Detect evidence

LEFT encoder Detect succeeded in V16 using the configured 15-pole-pair fallback,
with clean A/B evidence. The fallback, however, preserved the existing encoder
inversion because the old forward-only sweep could not prove direction. This is
why a successful Detect could still leave the speed sign wrong.

V17 changes the VESC-wire encoder sweep to 3 forward + 3 reverse and records the
directed quadrature evidence separately for each half.

RIGHT Hall Detect already succeeds and is intentionally not reworked.

## Hall-table truncation root cause

The VESC Tool popup showed FOC-angle values (`50,183,16,116,83,150`) in the
legacy Hall table. V16 used one FOC table image for both VESC wire fields. V17
separates legacy BLDC and FOC Hall tables.
