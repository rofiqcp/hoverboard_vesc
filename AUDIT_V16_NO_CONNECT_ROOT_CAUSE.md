# V16 audit — V15 no-connect root cause

## Symptom

Hardware UART device opens at 115200, but tester fails `02_connection_inventory` with a 1.5 s VESC response timeout. VESC Tool also cannot connect. Previous firmware could connect on the same hardware.

## Source-diff finding

V15 restored LEFT and RIGHT FOC execution on every ~16 kHz ADC DMA interrupt, but removed the V1 `motorIsrShedNext` overload-relief path. Prior hardware diagnostics recorded `isr_max_cycles = 5709`; at 64 MHz, a 16 kHz period is about 4000 CPU cycles. A full ISR that repeatedly exceeds one sample period can leave DMA TC pending continuously and starve thread/main context.

USART3 RX is circular DMA and packet parsing is deliberately done by `VescProtocol_Service()` in the main loop. Therefore main-loop starvation produces exactly this failure mode: bytes can physically arrive but no VESC command is parsed/replied.

## V16 correction

V16 restores a one-shot V1-style CPU-liveness relief without restoring sticky overrun faults:

1. Run current ADC delta and sample-local DC-link hard current chop first.
2. If the previous full ISR observed the next DMA transfer already pending, make this ISR short and return.
3. Do not clear runtime command, ARM, calibration ownership, or latch a fault.
4. Resume full LEFT + RIGHT sensor/FOC/PWM work on the following ISR.

This gives main context recurring CPU windows for VESC RX parsing/TX queue service while preserving the requested V1-like normal ISR order.

## Validation

- 20/20 host regression PASS.
- Dedicated V16 liveness regression ensures the shed path cannot mutate command/ARM/calibration state.
- Cortex-M3 current loop, encoder path, and DMA ISR contain no unexpected division helpers.
