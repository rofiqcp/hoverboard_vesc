# V24 Deep Audit — 2026-08-14

Baseline: V23 + `vesc_full_test_20260814_113745.zip`.

Changes:
1. STOP is now a dedicated safety path (`RuntimeControl_VescStopOne`) and cannot be blocked by encoder alignment/calibration.
2. Global serial/DMA release aborts calibration/alignment/homing before both outputs are disabled.
3. Full Brake `SET_DUTY(0)` cancels target alignment ownership and can keep the target bridge in zero-duty active state.
4. Same-value MCCONF roundtrip no longer writes flash, removing the observed config-response timeout root cause.
5. Full 481-byte MCCONF LEFT/RIGHT and 493-byte APPCONF wire shadows are persisted in a dedicated CRC-protected 2-KiB flash page, so VESC Tool fields survive MCU reset/power-cycle.
6. Encoder detect accepts decisive quadrature-direction evidence and compares ratio against physical motor pole-pairs.
7. Tester V24 makes a hardware serial disconnect a fatal motion-abort latch; remaining actuator tests are skipped.
8. Position growl/stall fail-fast tightened to >=0.6 A with <0.5 degree movement for >=0.15 s.
9. Standby test verifies Imotor, Ibattery, Id, Iq, Vd, Vq, eRPM, Vin and position are all present/finite.
10. FW string changed to `hoverboard-vesc6-v24`.

Validation available in this environment:
- Python tester compile: PASS.
- Python protocol/HBTS self-test: PASS, 1 PASS / 0 FAIL / 1 SKIP without hardware port.
- PlatformIO/arm-none-eabi toolchain is not installed in this environment, so STM32 target link and physical motor test remain to be done on the actual board.
