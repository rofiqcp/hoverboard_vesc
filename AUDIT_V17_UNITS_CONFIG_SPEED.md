# V17 unit/config/speed audit

## Canonical motor unit model

The original hoverboard FOC controller data defines `n_polePairs = 15`.
Therefore this port uses:

- pole-pairs = 15;
- total motor poles = 30;
- stock mechanical max = 1000 rpm;
- electrical max = 1000 * 15 = 15000 eRPM.

VESC FOC speed commands and reported motor speed are electrical RPM. Hence:

`mechanical_rpm = erpm / pole_pairs`

Examples:

- 500 eRPM -> 33.33 mechanical rpm;
- 900 eRPM -> 60 mechanical rpm;
- 2500 eRPM -> 166.67 mechanical rpm;
- 15000 eRPM -> 1000 mechanical rpm.

These numbers explain why a raw speed target cannot be compared directly to a
mechanical-RPM expectation.

## Why Duty/Current/Speed produce different free-wheel speeds

- Duty mode controls requested inverter modulation. It is not an RPM setpoint.
- Current mode controls torque-producing current (Iq). On an unloaded lifted
  wheel, a positive torque current can keep accelerating until losses,
  back-EMF/modulation or configured speed limits balance it.
- Speed mode closes an outer PID around eRPM and should settle around the eRPM
  target when feedback sign/scaling are correct.

Thus a current command eventually reaching near 15000 eRPM is not evidence that
`0.5 A` was interpreted as `15000`. The V16 hardware samples directly prove the
current command is approximately 0.5 A. The speed problem in that run is the
opposite sign of measured eRPM.

## VESC wire mappings retained

- `COMM_SET_DUTY`: int32 / 100000.
- `COMM_SET_CURRENT`: int32 / 1000 A.
- `COMM_SET_RPM`: direct signed int32 electrical RPM.
- `COMM_SET_POS`: int32 / 1000000 degrees.
- GET_VALUES motor current/input current/Id/Iq: VESC-style read/reset averages,
  encoded at 1e2 resolution.
- GET_VALUES duty: 1e3.
- GET_VALUES RPM: direct signed electrical RPM.

## MCCONF Hall fields

VESC 6.00 wire layout contains two arrays with different meanings. V17 enforces
those types rather than reusing one array:

- legacy BLDC table: -1/1..6 only;
- FOC table: 0..200 electrical angle, 255 invalid.

The detected RIGHT sequence can still generate FOC values such as 16, 50, 83,
116, 150 and 183. These are legal in `foc_hall_table` and must not be truncated.

## MCCONF pole fields

- `foc_encoder_ratio` is authoritative for LEFT pole-pair ratio and serializes 15.
- `si_motor_poles` is display/SI metadata and serializes 30.
- `l_max_erpm` serializes 15000 with stock configuration.
- deserialization converts eRPM limit to internal mechanical Q4 only after the
  final encoder ratio has been accepted.

## Current limits

The original hoverboard limits are preserved in the actual controller:

- motor phase/current target limit: 15 A;
- DC-link hard current chopping: 17 A.

VESC Tool expects the ABS-current setting to have headroom over normal regulated
motor current. V17 therefore advertises `l_abs_current_max = 22.5 A` and disables
`l_slow_abs_current`. This is configuration compatibility/reporting; it does not
replace the board's independent 17-A DC-link chop.

## Speed feedback direction

V17 adds a bidirectional encoder commissioning proof. It compares quadrature
transition evidence captured while electrical phase is commanded forward and
reverse. This is the key missing proof in V16: Detect could accept a healthy,
dithering encoder while preserving the wrong inversion.

The tester independently validates the final outcome: if a nontrivial measured
eRPM has the opposite sign to `COMM_SET_RPM`, it immediately releases the motor
and reports `SPEED_FEEDBACK_SIGN_MISMATCH`.


## VESC-like eRPM current taper

V17 serializes `l_erpm_start = 0.8` and applies the same operating concept in
the slow control path: same-direction accelerating Iq is untouched below 80%
of the configured electrical-speed limit, then is reduced linearly to zero at
the limit. With 15 pole-pairs and the stock 1000-rpm mechanical ceiling this
means 12000 eRPM -> 15000 eRPM. Opposite-direction braking current remains
available. This is a normal operating-limit calculation, not a sticky PWM
fault latch.
