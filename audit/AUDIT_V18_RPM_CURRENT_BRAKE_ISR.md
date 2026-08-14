# V18 Audit — RPM, Current, Brake, Handbrake, ISR

## Timing contract

STM32F103 clock: 64 MHz.

- ADC DMA / PWM: 16 kHz (62.5 us sample period)
- sensor observation LEFT + RIGHT: every 16-kHz DMA IRQ
- FOC current loop LEFT: every second IRQ = 8 kHz
- FOC current loop RIGHT: alternating IRQ = 8 kHz

Sensor speed math uses 16 kHz. Current PI Ki*dt uses 8 kHz. These frequencies
are intentionally separate.

## VESC command semantics mirrored

- SET_CURRENT -> signed Iq torque request.
- SET_CURRENT_BRAKE -> brake-current magnitude; fast loop continuously applies
  opposite sign to measured speed.
- SET_HANDBRAKE -> static current vector with electrical phase 0.
- SET_RPM -> electrical RPM target for outer speed PID.

## Hoverboard ISR properties retained

- current ADC delta is consumed directly in DMA IRQ;
- DC-link hard current chop is sample-local via MOE;
- active low-FET current calibration domain is retained;
- no UART/config/telemetry processing is inserted in the ISR;
- ISR emergency relief never clears command, ARM, or calibration state.

## RPM numerical proof

Tests derive speed from sample intervals, not FOC invocation count:

- Hall: one sector / 160 sensor samples @16kHz -> ~1000 eRPM.
- Encoder: CPR 2048, ten counts / 64 samples -> ~73.24 mechanical RPM,
  equivalent to ~1098.6 eRPM for 15 pole-pairs (after estimator settling).
