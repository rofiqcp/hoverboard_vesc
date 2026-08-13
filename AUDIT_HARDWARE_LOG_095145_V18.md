# Hardware Log Audit — 2026-08-13 09:51:45 -> V18

## Baseline V17

Latest bundle summary: 59 PASS, 2 FAIL, 1 WARN, 2 SKIP.

What is already proven usable:

- serial/VESC inventory works;
- current calibration works;
- LEFT encoder detect passes;
- RIGHT Hall detect passes;
- Duty +/- passes;
- Current +/- passes numeric Iq checks;
- current telemetry Id/Iq/Imotor/Ibattery is present;
- local/virtual-CAN routing isolation passes.

Failures in the automated run:

- LEFT negative RPM did not show signed motion in that short test window;
- RIGHT position did not move toward target.

The latest trace also shows a key semantic fact: right `SET_CURRENT +0.5 A`
accelerates the unloaded motor into the thousands of eRPM while measured Iq
remains near the requested current. This is expected torque-control behavior,
not proof of current scaling error.

## V18 response

- decouple speed observation rate from FOC scheduling;
- sample sensors every ADC DMA event at 16 kHz;
- interleave FOC at 8 kHz/motor to preserve CPU/UART liveness;
- retain VESC eRPM semantics in GET/SET;
- implement distinct current, current-brake, and handbrake modes;
- make brake sign dynamic from instantaneous speed;
- make handbrake fixed phase 0;
- remove the unused commissioning guard warning.
