# V19 Changelog — Integrated Encoder Sync / Position / Homing / EEPROM

## Hardware-log driven fixes

- Corrected LEFT incremental-encoder electrical synchronization. V18 captured
  `electrical_zero = raw_phase`, while the FOC reader later added the configured
  encoder offset again. V19 derives zero from the complete reader equation:
  `zero = raw_phase + configured_offset - desired_phase`.
- Kept active LOW-FET current-offset calibration and V18 ISR/sensor cadence;
  these were already supported by hardware evidence and are not reworked here.
- Added a numeric regression proving a phase-0 sync reads back phase 0 even with
  non-zero encoder offset.

## Motor geometry / VESC Tool config

- Physical `foc_motor_pole_pairs` and incremental `encoder_ratio` are now
  independent state.
- VESC Tool `Motor Poles` remains total magnetic poles (`2 * pole_pairs`).
- VESC Tool `FOC Encoder Ratio` changes only encoder electrical scaling.
- Encoder detect stores detected ratio without overwriting physical motor poles.
- Added `hb_set_pole_pairs N` terminal helper; it persists both physical motors.

## Position

- `SET_POS` is bounded to 0..360 degrees; 0 and 360 are distinct endpoints.
- Unhomed position exposes one mechanical sensor revolution.
- Homed/calibrated position maps the measured signed hard-stop span to 0..360.
- RIGHT Hall position is no longer a telemetry stub at 0 degrees.
- Position controller uses linear bounded error, not circular 0/360 wrapping.

## Encoder detect / sync / homing

- Added explicit one-per-power-cycle encoder electrical sync state.
- Added separate mechanical steering calibration state per motor:
  right-zero tick, signed span, calibrated, homed.
- Full calibration: RIGHT stop -> LEFT stop -> validate span -> return 180.
- One-stop homing: RIGHT stop -> refresh zero -> reuse saved span -> return 180.
- Stop proof requires current threshold + low speed + no encoder motion over
  debounce, with a hard timeout.
- Homing uses torque/current search, not speed PID, to avoid speed-integrator
  wind-up against a hard stop.
- Boot homing is optional and can only be enabled after a valid span exists.

## VESC protocol integration

- Implemented `COMM_SET_DETECT` display-mode selection and continuous
  `COMM_ROTOR_POSITION` stream (~100 Hz).
- Implemented integrated board commissioning on `COMM_DETECT_APPLY_ALL_FOC`:
  current-cal -> LEFT encoder -> RIGHT Hall -> LEFT sync -> EEPROM -> reply.
- Terminal reply is retried until queued; success is returned only after EEPROM
  save succeeds.

## Persistence / diagnostics

- EEPROM schema grows from 144 to 156 words.
- Persists independent encoder ratios and left/right steering zero/span/calibrated
  records. `homed` remains a boot-runtime state and is deliberately not restored.
- HBTS diagnostic version 14 exposes electrical-ready, position/homing and
  integrated-detect states.
- Python tester adds `rotor_position_stream.csv` and `homing_state_trace.csv`.
- Python `--full` uses integrated board commissioning by default. Legacy separate
  sensor detect remains available with `--individual-detect`.

## Duty regression

- Added numeric 3%/90% duty modulation regression.
- Low-duty hardware testing now checks that actual modulation stays near the
  requested ceiling instead of only checking that it is non-zero.
