# V21

V21 is a corrective release based on V20 and the 2026-08-13 hardware log.

## Confirmed fixes
- VESC SI gear ratio is writable/readable and persisted independently per motor; it is no longer hard-wired to 1.0.
- Physical pole-pairs remain independent from LEFT encoder ratio.
- LEFT `COMM_DETECT_ENCODER` no longer reports configured pole-pairs as if they were measured. If electrical/mechanical ratio cannot be proven by commanded motion, detection continues/fails instead of returning a fabricated `15`.
- LEFT encoder detect now reports a measured session electrical offset from the raw TIM4 count captured at the D-axis phase-0 lock.
- Realtime current/Vd/Vq telemetry keeps the last fresh sample through a transient low-side sampling-invalid instant while still clearing immediately when the bridge is released.
- Duty sign comes from q-axis modulation / duty command, not measured Iq sign.
- Vd/Vq conversion uses the live battery reading before telemetry sampling. VESC modulation convention is `Vdq = mod_dq * (2/3) * Vbus`; it is not simply `duty * Vbus`.
- `SET_DUTY(0)` remains armed, matching VESC Tool Full Brake. Stop remains the separate zero-current/release path.
- CURRENT_BRAKE has a 2 mechanical RPM zero-speed deadband to prevent Hall speed-sign chatter from reversing Iq around standstill.

## Preserved hardware invariants
- V20 ADC/DMA phase-current timing and active-low-side current-offset calibration are unchanged.
- RIGHT remains Hall-only on this PCB; LEFT remains the only AB encoder input.
- Full Auto Detect remains board commissioning (current offsets -> LEFT encoder -> RIGHT Hall -> LEFT electrical sync), not upstream R/L/flux motor-model identification.

## Hardware validation required
Run the V21 full tester on real hardware. In particular, a LEFT encoder detect that cannot physically follow the forced field should now fail instead of returning fallback ratio 15; increase detection current only within the board/motor safe range if required.

## Hardware correction — 2026-08-13 17:49 run
- Fixed encoder retry state: a failed +3/-3 sweep now restarts at + electrical direction instead of continuing reverse-only until timeout.
- Rebased per-window encoder transition counters on retry while preserving transaction-wide clean-edge/state evidence.
- Encoder ratio inference now accepts the stronger valid net displacement from either forward or reverse half-sweep, useful when steering starts against one mechanical stop.
- Integrated LEFT encoder / RIGHT Hall commissioning uses 1.00 A instead of 0.50 A; individual detect remains user-selectable and hard-capped by the existing 2 A safety limit.
- Full tester now checks Rotor Position with upstream VESC `COMM_SET_DETECT` streaming semantics instead of treating `COMM_ROTOR_POSITION` as a request.
- Firmware/tester release identity updated to V21.
