# Audit Hardware Log 2026-08-12 17:19:16 — V5 Fix

## Input
`vesc_full_test_20260812_171916.zip` from `/dev/ttyUSB0`.

## What the hardware log proved

The transport and VESC compatibility layer are healthy enough to continue:
- VESC 6.00 local device connected.
- LEFT local CAN ID = 10.
- RIGHT virtual CAN ID = 11 and is discovered through `COMM_PING_CAN`.
- Firmware/UUID replies work on both nodes.
- Connect remains disarmed.
- PA2/PA3 ADC readback works.
- MCCONF same-value round-trip now passes on both LEFT and RIGHT.
- APP UART/ADC/ADC_UART test passes.
- CRC/parser recovery passes.
- DWT overrun counters are no longer falsely saturated.

The first real failure is current-zero calibration:
- ADC1 hardware calibration: PASS.
- ADC2 hardware calibration: PASS.
- 2048 samples were collected.
- Candidate current ADC levels are plausible:
  - rlA ≈ 2809
  - rlB ≈ 2739
  - rrB ≈ 2771
  - rrC ≈ 2751
  - DCL ≈ 1945
  - DCR ≈ 1930
- V4 failure mask = `0x0002`, caused only by LEFT phase-B raw peak-to-peak.
- V4 raw span:
  - rlA 88
  - **rlB 322**
  - rrB 53
  - rrC 55
  - DCL 33
  - DCR 24

V4 rejected any raw peak-to-peak >160 counts. That criterion was too sensitive to one isolated ADC/EMI spike. A single spike does not prove that the zero-current mean is unstable.

Because current calibration failed, V4 did not apply any offsets (`offset[]=0`), so sensor commissioning was correctly blocked. All subsequent Duty/Current/RPM/POS failures are downstream consequences, not separate control bugs.

## Additional architecture bug found from this log

V4 returned from the DMA ISR before Hall/encoder processing whenever current calibration was not VALID. Therefore `hall_raw=000`, encoder state, position and RPM in the failed-calibration log were stale/reset observations and could not be used to diagnose the physical sensor.

V5 fixes this by separating:
- **observation**: Hall/encoder/position/RPM always update;
- **actuation**: PWM/current FOC remains disabled until current-zero proof is valid.

## V5 current-calibration policy

V5 retains:
- STM32 ADC1 + ADC2 hardware self-calibration.
- MOE OFF during calibration.
- 512-sample settling window.
- 2048 calibration samples.
- arithmetic candidate mean for each of the six physical current channels.

V5 changes validation:
- raw single-sample peak-to-peak -> diagnostic only (`raw_span`);
- samples are grouped in 64-sample blocks;
- the mean of every block is tracked;
- only sustained block-mean instability >64 ADC counts fails calibration (`block_span`);
- candidate means outside safe ADC range still fail;
- ADC hardware self-cal failures still fail.

Failure mask V5:
- bits 0..5: mean out of ADC range for rlA,rlB,rrB,rrC,DCL,DCR;
- bits 6..11: block-mean instability for the same channels;
- bit 12: ADC1 hardware calibration failure;
- bit 13: ADC2 hardware calibration failure;
- bit 14: no complete block.

The Python tester automatically retries transient block-instability failures but never retries/ignores structural ADC failures.

## New diagnostics

HBTS diagnostic version is now 4 and logs:
- instantaneous raw ADC;
- `candidate_mean`;
- applied `offset`;
- `raw_span`;
- `block_span`;
- residual;
- failure reasons decoded to text;
- current-cal generation;
- Hall/encoder observation even while current calibration is invalid;
- ISR last/max/deadline cycles;
- Id/Iq, Imotor, DCL/DCR, and total battery current.

## Expected next run

The expected first checkpoint after flashing V5 is:
1. `04_current_offset_calibration = PASS`.
2. `current_cal_valid = true`.
3. Applied offsets close to the candidate means above.
4. Idle Id/Iq/Imotor near zero.
5. Only after that does the tester run LEFT sensor detect and RIGHT Hall detect.

If sensor detect fails next, the log will now contain real Hall/encoder states instead of reset values, so sensor wiring/mapping/calibration can be diagnosed independently.
