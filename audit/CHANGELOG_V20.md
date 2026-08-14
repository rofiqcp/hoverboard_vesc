# CHANGELOG V20

V20 is a focused corrective release on top of V19. The current-offset domain,
Hall acquisition, and the timing-sensitive ADC/DMA motor ISR are intentionally
left unchanged.

## Protocol / connection

- Fixed a compiler-proven stack overflow in `COMM_FW_VERSION` from V19. The old
  80-byte local buffer was too small for the V19 firmware name. V20 uses a
  bounded string append helper, a 128-byte buffer, and a shorter firmware name.
- Enlarged the HBTS diagnostic payload buffer to 512 bytes. HBTS v15 is 453 bytes
  before VESC framing, so the previous 448-byte buffer had no safe headroom.
- Python inventory now retries the initial standard-VESC bootstrap up to three
  times while resetting parser/RX state between attempts.

## LEFT incremental encoder electrical synchronization

- V20 does not accept a one-point D-axis lock as sufficient proof of encoder
  direction.
- The slow commissioning state machine now locks at 0 electrical degrees, sweeps
  the forced field to +120 electrical degrees over 600 ms, measures signed raw
  TIM4 movement, returns to 0 degrees, and only then captures electrical zero.
- `raw TIM4 delta >= +2` proves non-inverted A/B; `<= -2` proves inverted A/B.
- **No direction proof = no encoder electrical READY.** The firmware therefore
  cannot silently arm a LEFT motor with plausible Id/Iq but a wrong torque-vector
  angle.
- If the proven direction differs from EEPROM configuration, the encoder runtime
  scale is rebuilt and the new direction is saved after the bridge is released.
- Electrical-zero invariant remains:
  `zero = raw_electrical + configured_offset - desired_phase`.

## Position 0..360

- Fixed a remaining RIGHT/LEFT position-direction issue: the logical 0..360
  coordinate is converted exactly once to signed raw sensor ticks. Absolute
  position ticks are no longer motor-inverted a second time inside the PID path.
- Before hard-stop homing, one logical revolution maps to one encoder revolution
  (LEFT) or `6 * pole_pairs` Hall sector ticks (RIGHT), signed to match positive
  host Duty/Current/RPM direction.
- Full-homing minimum-span validation now uses span magnitude, including inverted
  motors.

## Rotor Position / Vd-Vq

- `COMM_SET_DETECT` display modes no longer alias unrelated signals:
  - Detect/Inductance: forced detect phase only while commissioning is active.
  - Encoder: raw mechanical encoder angle.
  - PID Pos: current logical position.
  - PID Error: position target error.
  - Observer / observer-error modes: no fake packet is emitted because this port
    currently has no flux-model observer.
- Added standard VESC `GET_VALUES` Vd/Vq fields (bits 19/20). They are reconstructed
  from the FOC d/q modulation and DC bus voltage in the existing 200-Hz telemetry
  task. **No code was added to the DMA current ISR.**

## Auto Detect truthfulness

- Integrated board commissioning remains:
  `current offset -> LEFT encoder detect -> RIGHT Hall detect -> LEFT electrical sync -> EEPROM`.
- It does **not** yet run the full upstream VESC motor-model measurements for R, L,
  flux linkage, current-controller auto-tuning, or a sensorless flux observer.
- Therefore the R/L/flux fields shown by the VESC Tool Detection Result are current
  compatibility/configuration values, not newly measured motor-model parameters.
  The Python result explicitly reports `motor_model_measured: false`.

## Debug / tests

- HBTS version 15, synthetic payload length 453 bytes.
- Added `alignment_probe_delta`, `alignment_direction_proved`,
  `position_session_zero_ticks`, and `observer_valid` to debug traces.
- Added Vd/Vq to SET traces.
- Position test aborts and releases the motor when high Iq is present without
  mechanical position response, recording phase/sync/Vd/Vq evidence first.
- Removed the unused `abs_i16_saturating` warning.
- Host regression: 28/28 PASS.
