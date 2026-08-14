# V23 Deep Audit — 2026-08-14

Input debug: vesc_full_test_20260814_113745.zip.

## Root causes addressed
- Actual USB/UART disappearance starts at LEFT SET_POS; downstream RIGHT EIO failures are secondary.
- LEFT ABI probe has clean quadrature and decisive inverted direction, but coarse ratio rounded 14 vs configured 15 pole-pairs. V23 accepts +/-1 probe tolerance while preserving configured physical ratio.
- Full Brake SET_DUTY(0) no longer waits for ABI electrical-alignment permission because it is zero modulation; Stop remains SET_CURRENT(0).
- POSITION outer loop is hard capped to 1 A on both motors for bench safety.
- RX DMA failure immediately releases both motor outputs before UART reinitialization.
- Tester reconnects only to issue emergency STOP after USB reset/EIO; it never silently repeats a motion command.
- Periodic rotor streaming yields to long GET_MCCONF/GET_APPCONF replies; TX queue increased to reduce response starvation.
- SET_MCCONF ACK is sent only when exact-size validated VESC 6.00 configuration was accepted.
- All live PID/tuning floats are checked finite and bounded before application.

## Standby telemetry
COMM_GET_VALUES remains a 74-byte VESC-compatible response and reports motor current, input/battery current, Id, Iq, position, Vd and Vq even while stopped (zero where physically appropriate). COMM_ROTOR_POSITION remains available separately and streaming is lower priority than request/reply traffic.

## Command contract
Supported: Duty, Current, RPM/eRPM, Position 0..360, Current Brake, Handbrake, Full Brake=SET_DUTY(0), Stop=SET_CURRENT(0), local LEFT and forwarded virtual-CAN RIGHT.

## VESC Tool compatibility
The firmware advertises VESC firmware 6.00 and therefore intentionally implements the exact VESC 6.00 MCCONF/APPCONF wire footprint. It does not advertise a VESC 7.x schema. Truncated/malformed MCCONF is rejected rather than ACKed.
