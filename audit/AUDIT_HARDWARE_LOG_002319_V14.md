# Hardware Audit — V14 from 2026-08-13 00:23:19 Run

Source log: `vesc_full_test_20260813_002319.zip`.

## Test result baseline

- PASS: 19
- FAIL: 2
- SKIP: 22
- Fails: LEFT encoder Detect timeout; RIGHT Hall Detect timeout.
- Non-zero motion SET tests were skipped because Detect prerequisites failed.

## Current sensing / FOC

Current-zero calibration completed VALID with 2048 samples. Active-domain final residuals were 0/1 ADC count. Standard VESC wire during Detect proved non-zero current telemetry:

- LEFT GET_VALUES: Motor Current range about -0.53..+0.51 A; Id up to ~0.53 A; Input Current up to ~0.12 A.
- RIGHT GET_VALUES: Motor Current range about -0.55..+0.55 A; Id up to ~0.55 A; Input Current up to ~0.08 A.
- Standard VESC fault remained 0 in those packets.

Therefore idle zeros after bridge release are not evidence that ADC/current telemetry is dead.

## LEFT Encoder

Terminal V13 result: FAILED_TIMEOUT.

Evidence before timeout:

- encoder A/B mask: 0x0F (all four quadrature states)
- valid TIM4 edges: 208 total; about 75 new edges in the first six-electrical-cycle acquisition window
- invalid transitions: 0
- position dither: roughly -5..+2 ticks during sampled window
- configured CPR: 2048
- configured pole pairs: 15
- current target: 0.50 A
- no commissioning voltage saturation blocker

V13 fallback still required a meaningful net encoder displacement. Because the loaded rotor repeatedly returned near its start count, the healthy quadrature proof was discarded. V14 removes net displacement from the fallback health proof while keeping a strict edge/state/error requirement and explicitly marks the result as configured-ratio fallback.

## RIGHT Hall

Terminal V13 result: FAILED_TIMEOUT despite six legal raw states.

Evidence:

- raw Hall mask: 0x7E (states 1,2,3,4,5,6)
- representative V13 table: `[255, 50, 183, 16, 116, 83, 150, 255]`
- no UART TX drops in this run
- no commissioning voltage saturation blocker
- detect current remained around requested 0.50 A

The table itself contains six valid Hall angles but V13's fixed-sector-vote candidate logic rejected the acquisition. VESC 6.00 instead accumulates sin/cos of commanded electrical angle by raw Hall state over forward/reverse sweeps and succeeds when six states have enough samples. V14 adopts that method.

A coarse replay of logged Hall/phase samples orders the legal states approximately:

`3 -> 1 -> 5 -> 4 -> 6 -> 2`

which is consistent with the measured table sorted by electrical angle.

## Startup beep

Diagnostics recorded:

- current buzzer reason after connection: NONE
- last buzzer reason: BATTERY_LEVEL1
- event count: 1
- settled battery approximately 40..41 V

V14 treats this as a startup-filter transient and qualifies BAT_LVL1 with a 2-second grace plus 500-ms persistence.

## Why SET commands appeared unusable

The V13 tester intentionally skipped all non-zero Duty/Current/RPM/POS tests after both sensor Detects failed. Zero routing and parser/config/current tests did pass. V14 therefore focuses first on making sensor commissioning terminal-success from the already healthy hardware evidence, while preserving the per-side SET/ARM path and adding a post-detect regression contract.
