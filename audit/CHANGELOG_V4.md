# V4 Changes

- Replaced fragile startup current-offset learning with deterministic 256-sample settle + 2048-sample mean for 6 physical current channels.
- Added STM32F1 ADC1/ADC2 hardware self-calibration before timer/PWM trigger initialization.
- Added current-calibration hard gate before arming and Hall/encoder commissioning.
- Added VESC 6.00 MCCONF visibility for physical phase-current offsets.
- Added safe VESC Tool Terminal commands: `hb_current_cal`, `foc_dc_cal`, `hb_current_status`, `hb_help`.
- Defined Imotor as signed FOC vector magnitude; retained per-node DCL/DCR as VESC input current; added whole-board DCL+DCR debug current.
- Replaced DMA-TC overrun heuristic with Cortex-M3 DWT cycle timing.
- Added HBTS v3 debug payload with ADC raw/offset/span/residual and ISR timing.
- Made exact same-wire MCCONF write idempotent.
- Updated Python one-shot tester to V4 diagnostic layout and current calibration test-first workflow.
- Updated fault test to distinguish actual fault-stop from expected sensor-not-calibrated rejection.
