# Runtime Audit V2 — VESC Tool / Hall / Encoder / Id-Iq / Fault-Stop

## Final status

| Item | Status | Implementation |
|---|---|---|
| VESC Tool connect | PASS source/protocol audit | Connect never enables PWM by itself |
| Id/Iq while DISARMED | FIXED + host-tested | Clarke/Park observation always runs; actuation gated separately |
| Motor current while DISARMED | FIXED | Signed magnitude of Id/Iq exported as VESC motor current |
| RPM Hall/Encoder while DISARMED | PASS | Sensor observation independent of arm state |
| Hall Detect LEFT | IMPLEMENTED | COMM_DETECT_HALL_FOC -> auto sweep -> LUT -> EEPROM |
| Hall Detect RIGHT virtual CAN | IMPLEMENTED | Same command forwarded to virtual ID N+1 |
| Encoder Detect LEFT | IMPLEMENTED | Configured CPR + electrical sweep -> ratio/pole-pairs + inversion + sequence |
| Encoder Detect RIGHT | INTENTIONALLY REJECTED | Hardware contract RIGHT Hall-only |
| Encoder electrical zero after reboot | IMPLEMENTED | 450 ms first-run alignment; mechanical position preserved |
| SET_DUTY | FIXED | Dedicated duty/modulation mode |
| SET_CURRENT | AUDITED | Iq/torque path; zero releases run request |
| SET_RPM | AUDITED | ERPM path -> speed PI -> Iq |
| SET_POS | AUDITED | position -> speed/current cascade |
| 3 s overcurrent fault-stop | FIXED | immediate gate off + 3000 ms re-run inhibit |
| Automatic restart without command | DISABLED | Requires new/still-streaming valid command after stop window |
| APP ADC watchdog during detect | FIXED | pending detect holds VESC watchdog alive |
| Detect result persistence | FIXED | Hall/encoder proof + CPR + inferred pole pairs saved/read-back verified |

## Important behavioral distinction

**Observation is always on; actuation is conditional.** ADC sampling, rotor sensing, current reconstruction and Park transform run even with MOE/PWM disabled. `feedback_valid` is a permission for closed-loop actuation, not a permission to publish telemetry.

## Encoder Detect semantics

The VESC-compatible encoder detect assumes encoder CPR/counts is already configured. This is necessary because electrical/mechanical ratio cannot be solved if both CPR and pole-pair count are unknown from a single relative A/B stream. The detector uses a known 12-electrical-revolution sweep, validates quadrature sequence, measures count movement, infers pole pairs, and determines raw direction/inversion.

Because A/B is incremental, electrical absolute position is not intrinsically known after power loss. The detected persistent offset is therefore zero in the VESC response and the controller performs a short electrical alignment before the first closed-loop run after boot. Z/index is optional and retained on PB5.

## Fault-stop semantics

- Current chopping remains sample-fast in the DMA ISR.
- Eight consecutive overcurrent samples qualify a persistent overcurrent fault (~0.5 ms at 16 kHz).
- The affected motor's MOE is removed immediately.
- The slow loop latches fault telemetry and a fixed 3000 ms stop window.
- Overcurrent/ISR deadline faults are latched even when the motor was already DISARMED or commissioning, so the ISR latch cannot become permanent without a timer owner.
- Commissioning/alignment aborts when the target motor receives those hard runtime faults.
- At 3 s expiry the explicit ISR latch is cleared.
- No spontaneous motor restart occurs. A fresh or continuously streamed command is required, and all safety/readiness checks run again.

## Regression

`tests/run_host_tests.sh`:

```text
ALL_HOST_TESTS_PASS
```

11 groups cover strict syntax, sensor/FOC behavior, VESC PID/MTPA/FW, SVM comparison, trig accuracy, VESC framing/CRC/parser, VESC 6.00 config wire image, APP ADC fixed-point, compatibility syntax, resource contract, and UBSan stress.

Cortex-M3 codegen audit:

```text
FAST_CURRENT_LOOP_DIV_HELPERS=0
FAST_ENCODER_UPDATE_DIV_HELPERS=0
CORTEX_M3_CORE_OBJECT_PASS
```

## Verification boundary

This is not a claim of physical bench certification. PlatformIO could not be installed in the container because PyPI/DNS access was unavailable. There is no connected STM32 board, ST-Link, VESC Tool GUI, motor, current-limited supply or oscilloscope in this environment. The first powered tests must follow `README_VESC_TOOL.md` in order and at conservative limits.
