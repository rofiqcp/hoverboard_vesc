# Hoverboard Dual FOC — V20 LEFT Encoder Phase / Position / Protocol Fix

V20 is a conservative delta from V19. The proven active-low-FET current-offset
calibration, Hall acquisition, and timing-sensitive ADC/DMA ISR are retained.
The release focuses on four remaining hardware issues: V19 protocol memory
corruption, LEFT incremental-encoder electrical reference, RIGHT/LEFT 0..360
position mapping, and truthful VESC Tool Rotor Position/Vd/Vq telemetry.

## Runtime architecture

```text
ADC DMA ISR (unchanged by V20)
  -> current samples
  -> hard DC-link chop
  -> LEFT/RIGHT sensor sample
  -> interleaved FOC
  -> PWM

Slow loop / protocol
  -> VESC SET/GET
  -> Vd/Vq averaging
  -> encoder electrical sync state machine
  -> homing state machine
  -> EEPROM
  -> debug
```

## LEFT Encoder AB

Commissioning and power-cycle electrical synchronization are separate concepts.
V20 requires a real signed encoder movement while a D-axis field is swept +120°
electrical. Only after the direction is proven does it return to 0°, rebuild the
encoder runtime if needed, and capture the session electrical zero.

```text
D-axis lock 0°
 -> +120° field sweep
 -> signed TIM4 proof
 -> return 0°
 -> settle
 -> apply proven A/B direction
 -> capture electrical zero
 -> LEFT electrical READY
```

No proof means no READY; closed-loop commands are not allowed to rely on a guessed
phase reference.

## 0..360 position

`SET_POS` is a bounded mechanical coordinate where 0° and 360° are different
endpoints. Before hard-stop homing, one sensor mechanical revolution is the
0..360 span. After homing, the measured signed right-to-left stop span replaces
that session span.

## VESC Tool Rotor Position

V20 follows the source meaning of the VESC display modes instead of making several
buttons aliases. Encoder is raw encoder angle; PID Pos is logical position; PID
Error is target error. Observer modes are not synthesized because V20 does not
have a real flux observer.

## Standard VESC telemetry

`GET_VALUES` publishes read-reset averages for Imotor/Input Current/Id/Iq and now
also Vd/Vq. Vd/Vq are reconstructed in the 200-Hz telemetry task from FOC d/q
modulation and DC bus voltage. The DMA ISR is not changed for this feature.

## Auto Detect scope

The integrated V20 command performs board commissioning:

```text
current offset
 -> LEFT Encoder AB detect
 -> RIGHT Hall detect
 -> LEFT electrical sync
 -> EEPROM save
```

It is not yet the full upstream VESC motor-model detection (R, L, flux linkage,
auto current-controller tuning, observer gain). Detection Result R/L values are
therefore configuration compatibility values.

## Test

First run a read-only connection test:

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0
```

Then, with the mechanism safe and unloaded:

```bash
python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes
```

Do not use `--homing-calibrate` until normal LEFT/RIGHT Duty, Current, RPM and
Position have passed.
