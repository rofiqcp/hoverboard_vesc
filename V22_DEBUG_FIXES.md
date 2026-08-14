# hoverboard_vescv22 debug fixes

Based on vesc_full_test_20260814_094316.

- Fixed tester rotor-position UnboundLocalError (`out` initialized before observer cases).
- Fixed tester Full Brake zero-value assertions; integer zero is a valid VESC wire value and must not be replaced through Python truthiness. This was the root of the subsequent false virtual-CAN cross-talk cascade.
- Firmware Apply-All auto-detect now reuses already-verified current offsets instead of always starting a redundant calibration transaction. This removes the V21 transient that could make RIGHT Hall detection terminate as START_REJECTED (result 9).
- Firmware identity updated to v22.
- RIGHT RPM/POS sign failures are expected to be re-evaluated only after successful Hall commissioning; V21 test ran them with an old/failed Hall commissioning state. No blind hard-coded RIGHT sign inversion was introduced because that would hide wiring/LUT direction errors and can break correctly commissioned hardware.
