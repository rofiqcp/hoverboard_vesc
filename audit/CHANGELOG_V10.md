# V10 Changelog — Per-Motor Isolation, Stock ADC Timing, Full Command Audit

V10 is based on the V9 hardware log `vesc_full_test_20260812_203937.zip` plus a line-by-line comparison against the original hoverboard-firmware-hack-FOC board timing and the VESC 6.00 command behavior.

## 1. LEFT/RIGHT standard-current isolation

Standard VESC `Motor Current`, `Id`, `Iq`, and `Input Current` now belong strictly to the selected motor context.

- LEFT local standard current is accumulated only while LEFT TIM8 MOE is active.
- RIGHT virtual-CAN standard current is accumulated only while RIGHT TIM1 MOE is active.
- When a bridge is released, its standard current accumulator and cached value are reset to zero.
- A cached sample may only be held for 25 ms while that same bridge remains active, preventing the VESC Tool current display from getting stuck.
- Raw passive shunt observations are never discarded; they remain in HBTS diagnostics only.

This prevents manually moving LEFT from appearing as RIGHT `Imotor` merely because the released RIGHT low-side shunts pick up common-mode/back-EMF activity.

## 2. Explicit virtual-CAN motor context

`COMM_FORWARD_CAN` no longer routes through a free boolean. V10 has explicit immutable LEFT and RIGHT VESC contexts. A packet addressed to the virtual second ID is processed entirely in the RIGHT context and then returns to LEFT, mirroring the VESC dual-motor concept.

HBTS v9 adds per-context counters:

- `route_packets_this`
- `route_packets_other`
- `set_packets_this`
- `last_set_command`

The one-shot tester checks these counters before/after every safe zero SET and every motion command.

## 3. Stock hoverboard phase-current ADC timing restored

V9 had drifted away from the original board timing: ADC prescaler `/6` and a timer-offset constant derived from 1.5-cycle sampling even though phase-current ranks actually use 7.5-cycle sampling.

V10 restores the original stock geometry:

- ADC clock divider: 4
- phase-current sampling: 7.5 ADC cycles (+12.5 conversion cycles)
- `ADC_CONV_CLOCK_CYCLES = 20`
- `ADC_TOTAL_CONV_TIME = 4 * 20 = 80` timer ticks
- TIM8/LEFT timer offset remains tied to `ADC_TOTAL_CONV_TIME`
- current scale remains 50 ADC count/A; internal fixed-point remains 800 units/A

Battery, temperature and PA2/PA3 remain outside the 16 kHz regular current scan using injected conversions, so the fast current sequence remains only three dual-ADC ranks.

## 4. Current telemetry is no longer sticky

VESC standard current uses read/reset-style per-motor averaging. If the bridge becomes inactive, cached current is cleared immediately. If the bridge remains active but no new valid sample arrives, the cache expires after 25 ms.

HBTS still exposes instantaneous values and therefore remains useful as an oscilloscope-like diagnostic source.

## 5. Preserved commissioning first-fault sample

When FAST CURRENT GUARD trips, V10 captures the sample before cleanup:

- target current
- Id / Iq
- both measured phase-current ADC deltas
- DC-link delta
- raw ADC values
- zero offsets
- forced electrical phase
- duty A/B/C and absolute duty
- side, streak, generation and fault mask

The Python tester writes this to `<node>_detect_first_fault_snapshot.json` when a detect fails.

## 6. Comprehensive one-shot protocol matrix

The tester now exercises every command case implemented by the VESC facade. A static regression fails if firmware and tester command coverage drift apart.

Read/control matrix includes FW version, GET_VALUES normal/selective, GET_VALUES_SETUP normal/selective, MCCONF/APPCONF current/default, ADC, rotor position, virtual CAN ping, HBTS custom data, ALIVE and terminal current-status.

Safe zero SET routing is tested on LEFT and RIGHT for duty, current, brake current, handbrake, RPM and current-relative commands before any sensor motion.

After successful sensor commissioning, non-zero tests include positive/negative duty, current and RPM, brake, handbrake, current-relative and position on each motor. Every sample also reads the peer motor to prove it remains released and receives no SET packet.

## 7. Command pacing

Motion commands are streamed at a maximum nominal 50 Hz while telemetry is sampled. This avoids flooding USART3 and keeps command/timeout behavior comparable to normal VESC control use.

## 8. Release status

Host regression and Cortex-M3 hot-path code generation pass in the build environment used to prepare V10. Physical STM32 build/upload and motor motion remain hardware tests to be performed by the user; V10 does not claim hardware Detect success before the next `/dev/ttyUSB0` log is returned.
