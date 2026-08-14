# Audit Python Full Tester V3

## Scope

Tester: `tools/vesc_full_test.py`

Target firmware: hoverboard VESC dual V3 with HBTS read-only diagnostics over VESC `COMM_CUSTOM_APP_DATA`.

## Verified locally in this environment

- Python syntax (`py_compile`): PASS.
- VESC CRC16 reference (`123456789 -> 0x31C3`): PASS.
- Short frame roundtrip: PASS.
- Long frame roundtrip (300-byte payload): PASS.
- Bad-CRC rejection followed by valid-frame recovery: PASS.
- Synthetic HBTS C/Python field-layout decode: PASS, payload length 108 bytes.
- C host regression: 11/11 PASS.
- VESC6 APPCONF tester offsets are locked by C assertions:
  - `app_to_use` wire offset = 33;
  - `adc_ctrl_type` wire offset = 90.
- Cortex-M3 codegen hot paths: PASS; no runtime division helper in FOC current loop or hardware encoder update.

## Hardware tests implemented but not executable here

The tester is prepared to execute on the real board:

- local VESC FW/UUID;
- virtual CAN discovery and forwarded RIGHT FW/UUID;
- connect/disarm semantics;
- idle Id/Iq/Imotor/RPM telemetry;
- PA2/PA3 ADC;
- MCCONF/APPCONF read + same-value write roundtrip;
- APP_UART, APP_ADC, APP_ADC_UART dispatch test with ADC control temporarily NONE;
- parser recovery after a corrupt harmless frame;
- LEFT Hall or Encoder detect according to active sensor;
- RIGHT Hall detect;
- Duty +/-;
- Current +/-;
- ERPM +/-;
- Position step;
- release/disarm verification after every motion test;
- automatic 3-second fault-stop observation if a real runtime fault occurs.

No deliberate overcurrent injection is implemented. This is intentional: the tester records and times a real fault if one occurs, but does not short phases or command destructive current solely to prove fault logic.

## Automatic diagnostics

Each run creates:

- `summary.txt`
- `results.json`
- `session.log`
- `raw_packets.log`
- `telemetry.csv`
- `diagnostics.csv`
- `adc_samples.json`
- `local_mcconf.bin`
- `right_mcconf.bin`
- `appconf_original.bin`
- `appconf_test_mode.bin`
- `exceptions.log` when applicable

The complete folder is automatically zipped at the end of the run.

## Safety behavior of tester

- `--full` requires an explicit confirmation unless `--yes` is supplied.
- untouched connect-state is captured before configuration is modified.
- before full tests, both motors are released and APP is temporarily forced to UART + ADC control NONE.
- original APPCONF is kept and restored from `finally`.
- each motion test is followed by a zero-current release and armed/output gate verification.
- `finally` always attempts release on LEFT and RIGHT before closing UART.
- HBTS diagnostics are read-only; no fault injection, forced PWM, ARM bypass, or safety bypass exists.

## Current hardware result

`NOT_RUN_IN_CHATGPT_CONTAINER` because no physical hoverboard controller/motor/power stage is connected here. See `audit/HARDWARE_TEST_STATUS.txt`.
