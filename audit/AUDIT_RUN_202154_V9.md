# Audit Run 2026-08-12 20:21:54 → V9

## Evidence supplied by user

- PlatformIO Core 6.1.19.
- STM32F103RC target build succeeded.
- RAM 7860/49152 bytes (16.0%).
- Flash 83892/262144 bytes (32.0%).
- ST-Link programming and verify succeeded.
- Python tester opened `/dev/ttyUSB0`.
- Protocol self-test passed.
- Failure occurred immediately at `02_connection_inventory`: `RuntimeError: local controller ID unknown`.
- User separately verified that VESC Tool connects to the same V8 firmware.

## Conclusion

This evidence isolates the failure to the V8 Python tester inventory bootstrap, not the STM32 build/upload or standard VESC communication path.

## Source-level root cause

V8 did:

```text
connection_inventory
  -> diag(local)
       -> _expected_node_id(local)
            -> requires self.local_id
```

while `self.local_id` was supposed to be obtained from that same `diag(local)` call.

## V9 correction

V9 bootstraps from `COMM_GET_VALUES.controller_id` before any HBTS request. HBTS node validation remains strict after IDs are known, preserving the stale-frame fix from V8.

## Why V7 tester does not work reliably with V8 firmware

V8 HBTS layout/version was extended for current-loop and buzzer diagnostics. A V7 tester can still speak standard VESC commands, but its HBTS decoder does not match the V8/V9 diagnostic layout. Use the tester bundled with the matching release.
