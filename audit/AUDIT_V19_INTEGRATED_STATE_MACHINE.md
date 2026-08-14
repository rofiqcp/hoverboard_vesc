# V19 Integrated State Machine Audit

## Independent domains

V19 intentionally keeps three different references separate:

1. FOC motor geometry: physical pole-pairs (`foc_motor_pole_pairs`).
2. Encoder electrical calibration: ratio, offset, inverted, electrical sync zero.
3. Mechanical position calibration: right-zero tick, signed travel span, homed.

Changing one domain must not silently rewrite another.

## Board commissioning

```text
IDLE
  -> current-offset calibration
  -> LEFT encoder detect
  -> RIGHT Hall detect
  -> LEFT encoder electrical sync
  -> EEPROM transaction
  -> reply success
```

Terminal success is only reported if EEPROM persistence succeeds.

## Encoder power-cycle state

```text
config loaded
  -> A/B counter valid only relatively
  -> electrical_ready = false
  -> first required closed-loop command / explicit hb_encoder_sync
  -> regulated D-axis alignment to known phase 0
  -> zero = raw + configured_offset - desired_phase
  -> electrical_ready = true
  -> normal LEFT FOC
```

Encoder DETECT and power-cycle SYNC are different operations. Detect derives
configuration; sync establishes the current absolute electrical reference.

## Mechanical full calibration

```text
FOC electrical ready
  -> low-current search RIGHT
  -> stop proof: current high + speed low + no encoder motion + debounce
  -> right_zero = current tick (logical 0 deg)
  -> low-current search LEFT
  -> stop proof
  -> span = left_tick - right_tick (logical 360 deg)
  -> sanity check span
  -> save EEPROM
  -> position control to 180 deg
  -> READY
```

## Power-on homing

Only available after full calibration saved a valid span:

```text
FOC electrical sync
  -> low-current search RIGHT only
  -> right_zero = current tick
  -> reuse saved signed span
  -> return 180 deg
  -> READY
```

Full calibration is never automatically started on boot.

## VESC Tool rotor-position stream

`COMM_SET_DETECT` selects display source. The board sends
`COMM_ROTOR_POSITION` approximately every 10 ms. On the shared UART only one
motor stream is active at a time to avoid ambiguous interleaving of packets that
have no CAN-ID field in the rotor-position payload.

## Position semantics

- `SET_POS` wire remains degrees * 1,000,000.
- Firmware clamps target to 0..360.
- 0 and 360 are distinct endpoints.
- Homed/calibrated: 0..360 maps the measured signed steering span.
- Unhomed: 0..360 maps one mechanical sensor revolution.
