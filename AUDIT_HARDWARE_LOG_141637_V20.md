# Hardware Audit — V19 run 2026-08-13 14:16:37 -> V20

## Observed

The V19 tester opened `/dev/ttyUSB0`, then timed out in
`02_connection_inventory` before the local/right IDs could be established.
The same V19 build emitted a compiler warning proving that `fw_version()` copied a
46-byte firmware string into only 37 bytes of remaining stack-buffer space.

## Root cause fixed

`COMM_FW_VERSION` is one of the first inventory requests. V19's local 80-byte
packet buffer was therefore capable of corrupting stack state while answering the
very command used to establish the connection. V20 uses a 128-byte buffer and a
bounded string append helper.

During the V20 audit, HBTS v15 was also found to be 453 bytes while the C debug
buffer was 448 bytes. That second debug-buffer risk was fixed before release by
increasing the local HBTS buffer to 512 bytes.

## LEFT hardware symptom

Previous hardware logs showed meaningful Id/Iq but little/no correct LEFT motion.
A one-point electrical lock can establish a zero reference but cannot prove A/B
direction. V20 therefore uses a +120 electrical-degree forced-field sweep and raw
TIM4 count movement as mandatory direction proof. If the probe is not proven,
LEFT never becomes electrical READY.

## RIGHT position symptom

RIGHT Duty/Current/RPM already worked, while Position did not. V20 fixes the
logical-to-raw position coordinate so the signed sensor span handles direction and
`motor_inverted` is not applied again to the absolute position target.

## ISR policy

No V20 feature in this audit adds work to `DMA1_Channel1_IRQHandler`. Vd/Vq,
Rotor Position streaming, encoder synchronization sequencing, protocol and debug
remain outside the timing-sensitive DMA current ISR.
