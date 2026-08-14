# CHANGELOG V16

## Fixed: V15 VESC Tool / UART no-connect regression

V15 restored both motor FOC loops on every ~16 kHz ADC DMA interrupt, but removed the V1 one-shot overload relief. With a 64 MHz STM32F103, the previously measured 5709-cycle ISR maximum can exceed the ~4000-cycle 16 kHz budget and starve thread/main context. RX DMA still receives bytes, but `VescProtocol_Service()` cannot run, producing FW_VERSION/connection timeouts.

V16 restores a **non-sticky, one-shot CPU-liveness relief**:

- if DMA TC is already pending again at the end of a full motor ISR, set `motorIsrShedNext`;
- on the next ISR, perform direct ADC current delta and sample-local hard current chop, then return quickly;
- never clear host SET, ARM, runtime motor enable, calibration ownership, or MOE as a sticky overrun response;
- the following interrupt resumes the complete LEFT + RIGHT sensor/FOC/PWM pass.

This preserves V15's VESC SET/GET path and proven current/sensor calibration fixes while restoring serial/main-loop liveness.

## Regression

- Host regression: 20/20 PASS.
- Cortex-M3 hot paths: no unexpected division helpers.
- Python tester compiles and identifies itself as V16.
