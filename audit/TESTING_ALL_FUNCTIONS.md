# V20 Hardware Acceptance Test

## Phase A — protocol liveness

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0
```

Required:
- protocol self-test PASS
- serial open PASS
- connection inventory PASS
- both standard VESC identities readable
- HBTS local/right readable after IDs are known

## Phase B — integrated commissioning and motion

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes
```

Required commissioning proof before LEFT motion:
- LEFT encoder calibrated
- all required encoder evidence valid
- `alignment_direction_proved = true`
- `abs(alignment_probe_delta) >= 2`
- `encoder_electrical_ready = true`
- RIGHT Hall calibrated

Required motion groups:
- LEFT Duty +/-
- LEFT Current +/-
- LEFT RPM +/-
- LEFT Brake / Handbrake / Current Relative
- LEFT Position 0..360
- RIGHT Duty +/-
- RIGHT Current +/-
- RIGHT RPM +/-
- RIGHT Brake / Handbrake / Current Relative
- RIGHT Position 0..360

Trace each command through raw wire -> runtime target -> ARM/MOE -> Id/Iq -> Vd/Vq
-> duty -> eRPM -> position. Position testing releases immediately on high-current
no-motion stall evidence.

## Phase C — Rotor Position

Check:
- LEFT Encoder stream
- PID Pos stream
- PID Error stream
- Detect stream only during commissioning
- Observer modes do not produce fake encoder/Hall data in V20

## Phase D — hard-stop homing (only after A/B pass)

Full calibration is opt-in because it physically touches stops:

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes --homing-calibrate
```

Enable one-stop homing on later boots only after a valid span has been saved:

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes --homing-on
```
