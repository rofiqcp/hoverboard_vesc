# V20 Audit — LEFT Encoder, Position, Rotor Position, Vd/Vq, Protocol

## 1. Protocol memory safety

- `COMM_FW_VERSION`: 128-byte buffer + bounded strings.
- HBTS v15: 512-byte local payload buffer for a 453-byte diagnostic payload.
- Standard inventory remains the first source of controller IDs; HBTS is queried
  only after local/right IDs are known.

## 2. LEFT electrical angle proof

V20 requires:
1. D-axis current lock at 0 electrical degrees.
2. Slow +120 electrical-degree field sweep.
3. Signed raw TIM4 movement >= 2 counts.
4. Return to 0 degrees and settle.
5. Apply proven encoder inversion.
6. Rebuild encoder runtime state if direction changed.
7. Capture electrical zero.
8. Only then set electrical READY.

This follows the key idea used by VESC encoder detection: forced D-axis current and
forward/reverse electrical movement are used to establish encoder relationship;
a static lock alone is not sufficient direction evidence.

## 3. Position mapping

The logical coordinate is always 0..360. One signed raw span performs the complete
mapping. The absolute PID target is not motor-inverted a second time.

## 4. Rotor Position modes

V20 does not fabricate a flux observer. Mode sources are independent:
Detect phase, raw Encoder, PID position, PID error. Observer-derived modes are
silent until an actual observer is implemented.

## 5. Vd/Vq

Vd/Vq are controller voltages, not physical BEMF-sensor channels. V20 converts the
existing Q14 d/q modulation to volts using DC-link voltage and averages it in the
200-Hz VESC telemetry path. GET_VALUES bits 19 and 20 use VESC scaling 1e3.

## 6. Auto Detect scope

V20 integrated commissioning is sensor/current-offset commissioning. It does not
claim that the displayed R/L/flux values are freshly measured. A true upstream
FOC Auto Detect additionally measures motor R/L/flux, derives safe current, tunes
current PI/observer parameters, applies config, and stores it.
