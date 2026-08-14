# V17 changelog — VESC units/config + speed direction

V17 is a focused delta over V16. The current ADC calibration, V1-style ISR
liveness relief, working RIGHT Hall acquisition and FOC current core are not
reworked.

## Fixed: VESC Tool `Parameters truncated` after Hall Detect

VESC 6.00 serializes two independent Hall arrays in `mc_configuration`:

- legacy `hall_table[8]`: signed BLDC commutation states (`-1`, `1..6`);
- `foc_hall_table[8]`: FOC electrical angle (`0..200`, `255` invalid).

V1-V16 wrote the FOC angle table into both fields. V17 adds separate serializers:
`hall_table_legacy_vesc()` and `hall_table_foc_vesc()`. Values such as
`50/183/16/116/83/150` remain valid in the FOC field and no longer leak into the
legacy field.

## Fixed: pole vs pole-pair semantics

The stock hoverboard FOC model uses 15 pole-pairs. V17 exposes this consistently:

- VESC `si_motor_poles` = `2 * foc_motor_pole_pairs` = 30 total poles;
- VESC `foc_encoder_ratio` = 15 pole-pairs;
- speed-limit conversion is applied after the final encoder ratio is decoded;
- default 1000 mechanical rpm maps to 15000 eRPM.

`si_motor_poles` is treated as SI/display metadata in this port; the authoritative
LEFT electrical/mechanical ratio is `foc_encoder_ratio`.

## Fixed: VESC Tool current-limit warnings

- `l_abs_current_max` readback = `1.5 * l_current_max` (22.5 A at stock 15 A).
- `l_slow_abs_current` readback = disabled.
- regulated motor-current limit remains 15 A.
- independent board DC-link hard chop remains 17 A.

## Verified: current scaling from hardware

The V16 02:20:14 log proves `SET_CURRENT 0.50 A` is encoded as `500` and settles
at approximately 0.47–0.51 A standard VESC Iq. Negative current similarly tracks
about -0.46 to -0.49 A. No current-scale multiplier was changed in V17.

## Fixed: exact zero primary SET release

- Duty 0 -> release/disarm.
- Current 0 -> release/disarm.
- RPM 0 -> release/disarm.
- Position 0 deg remains a valid active position command.

This fixes V16 zero-routing/release failures and prevents a stale zero command
from keeping MCCONF writes blocked as `RuntimeControl_Armed()`.

## Fixed: speed unit/readback precision

`COMM_SET_RPM` remains direct signed electrical RPM as in VESC 6.00. Runtime
stores it as a normalized host command, then reconstructs exact eRPM against the
configured eRPM limit for the speed PID.

GET_VALUES eRPM now converts directly from Q4 mechanical speed multiplied by
pole-pairs before rounding. It no longer first truncates to whole mechanical RPM,
which previously quantized a 15-pole-pair motor in 15-eRPM increments.

## Fixed/strengthened: LEFT encoder direction Detect

The latest V16 hardware log showed +900 eRPM command with negative measured eRPM.
V17 changes VESC-wire encoder detect to:

- 3 forward electrical sweeps;
- capture midpoint directed A/B transition counts and signed forward delta;
- 3 reverse electrical sweeps;
- score `forward-positive + reverse-negative` versus the inverted hypothesis;
- commit `sensor_inverted` when the direction proof is strong;
- preserve the explicit pole-pair fallback path for a loaded/dithering wheel.

New HBTS v13 fields expose forward delta, normal/inverted scores and whether
encoder direction was proved.

## Tester safety and proof

- tester release V17;
- HBTS diagnostic version 13, payload self-test 414 bytes;
- RPM test aborts immediately on clear opposite-sign eRPM and sends repeated
  stop commands;
- current tests calculate a numeric proof against requested Iq;
- first motion window receives an alignment extension when the only reason is
  `ALIGNMENT_BUSY`;
- SET_RPM result logs requested raw eRPM plus the 15-pole-pair/30-pole semantics.

## Regression

V17 adds `test_v17_units_config_speed_contract.py` and expands MCCONF wire tests
to decode/verify the values VESC Tool actually receives.


## VESC-like eRPM current taper

V17 serializes `l_erpm_start = 0.8` and applies the same operating concept in
the slow control path: same-direction accelerating Iq is untouched below 80%
of the configured electrical-speed limit, then is reduced linearly to zero at
the limit. With 15 pole-pairs and the stock 1000-rpm mechanical ceiling this
means 12000 eRPM -> 15000 eRPM. Opposite-direction braking current remains
available. This is a normal operating-limit calculation, not a sticky PWM
fault latch.
