# V15 architecture audit — VESC command flow + V1-style ISR

## 1. Standard VESC SET path

For local LEFT:

`UART packet -> VescPacket_Feed -> process_ctx(LEFT) -> exact VESC wire decode -> set_* facade -> RuntimeControl_VescSetOne(LEFT, mode, setpoint, true) -> RuntimeControl_VescAlive()`

For virtual RIGHT:

`COMM_FORWARD_CAN + second_id -> process_ctx(RIGHT) -> same exact decoder -> RuntimeControl_VescSetOne(RIGHT, ...)`

Primary wire scaling:

- Duty: int32 / 100000.
- Current: int32 / 1000 A.
- RPM: int32 ERPM.
- Position: int32 / 1000000 degree.

The protocol facade does not decide that a small/zero command is invalid. Command range and sensor/calibration readiness are runtime/core responsibilities.

## 2. Runtime -> FOC path

`RuntimeControl_VescSetOne -> requested mode/setpoint -> side-local arm request -> try_arm_motor(target) -> side-local controller mode -> slow outer-loop target -> DMA current ISR -> mc_foc_run_current_control(target) -> TIM compare`.

No opposite-side calibration/fault state is required for a single side to ARM.

## 3. GET_VALUES path

`FOC output/current-valid state -> slow 200-Hz accumulator -> GET_VALUES read/reset -> standard VESC field order/scaling`.

Standard current becomes zero when that motor bridge is released/current-domain invalid; raw ADC evidence remains available in HBTS. This prevents passive shunt pickup or the other motor from appearing as current for an inactive virtual controller.

## 4. DMA ISR path

V15 intentionally follows the simple V1/hoverboard layout while retaining corrected calibration:

1. clear ADC-DMA TC flag;
2. current-zero calibration service if required;
3. battery slow-filter trigger;
4. calculate six current deltas directly from calibrated offsets;
5. determine output requests;
6. sample-local DC-link over-current chop using MOE;
7. one coherent LEFT GPIO snapshot;
8. update LEFT Hall or TIM4 encoder sample;
9. apply commissioning/open-loop phase mode;
10. one-sample active-domain bridge prime;
11. run LEFT FOC and write LEFT PWM;
12. one coherent RIGHT GPIO snapshot;
13. update RIGHT Hall sample;
14. run RIGHT FOC and write RIGHT PWM;
15. publish raw current validity/timing diagnostics.

Both FOC calls occur on every ~16-kHz DMA interrupt.

## 5. Intentionally NOT copied from V1

- V1's recursive current offset learner is not restored; active low-FET calibration is proven on this hardware.
- V1's LEFT encoder PB5/PB6 observation is not restored; PB6/PB7 are A/B.
- V1's ISR overrun shed/sticky shutdown is not restored; overrun is diagnostic only.
- V14 Hall/encoder commissioning corrections are retained.

## 6. Minimal hard safety retained

V15 deliberately avoids using debug-quality metrics as PWM permissions. It still retains essential hardware/control prerequisites: valid current offsets, active transaction ownership, valid closed-loop feedback, encoder electrical alignment, configured setpoint limits, DC-link current chopping, communication timeout and catastrophic DMA heartbeat loss.

This release does not claim physical timing success until the STM32F103RCT6 log confirms the 16-kHz dual-loop ISR cycle budget on the actual board.
