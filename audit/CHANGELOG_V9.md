# V9 Changelog — Tester Inventory Bootstrap Fix

## Scope

V9 intentionally keeps the V8 motor-control firmware behavior unchanged. The user-proven V8 target build/upload succeeds and VESC Tool connects, so this release fixes the Python hardware tester/runner rather than changing FOC, current scaling, sensor logic, or commissioning control again.

## Root cause from the 2026-08-12 20:21 run

`02_connection_inventory` failed with `RuntimeError: local controller ID unknown`. V8 added strict HBTS node validation to reject stale LEFT/RIGHT diagnostic frames, but `connection_inventory()` attempted `diag("local")` before `local_id` had been established. `diag()` correctly refused because it needs the expected node ID. This was a circular dependency in the tester.

VESC Tool remained operational because standard VESC UART protocol was healthy.

## Fix

Inventory bootstrap now uses only standard VESC commands:

1. `COMM_FW_VERSION` local.
2. `COMM_GET_VALUES` local and read the standard controller-ID field.
3. `COMM_PING_CAN` and verify virtual RIGHT = local+1 (with existing wrap policy).
4. Set `local_id/right_id` in the tester.
5. `COMM_FW_VERSION` RIGHT through virtual CAN.
6. Only now request HBTS diagnostics and enforce exact node-ID matching.

A compatibility fallback derives local ID from a single nonzero virtual-CAN ID only when `GET_VALUES` lacks controller ID. Virtual ID 0 without a controller ID is rejected as ambiguous.

## Regression

A new `tests/test_tester_inventory.py` makes the old V8 sequence fail and the V9 sequence pass. Full host suite is now 13/13.

## Version compatibility

HBTS diagnostic wire version remains 7, matching V8 firmware. Old V7 tester code is not expected to decode V8/V9 HBTS layouts. Standard VESC Tool compatibility remains VESC wire/config profile 6.00.
