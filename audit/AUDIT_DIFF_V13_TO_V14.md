# Audit Diff V13 -> V14

## Kept because hardware already proved it

- active LOW-FET zero-vector current offset calibration;
- 6-sample bridge warm-up/current-domain gate;
- fixed-point FOC/SVPWM/current loop;
- VESC 6.00 SET scaling/routing;
- standard current telemetry field layout;
- false-DRV mapping policy;
- durable Detect terminal reply / TX retry from V13;
- per-motor terminal Detect snapshot;
- LEFT PB6/PB7 encoder pin fix;
- D-axis current-controlled encoder first-ARM alignment.

## Changed from new log evidence

1. Hall candidate finalization:
   `fixed 60-degree sector voting` -> `VESC circular angle average by raw Hall state`.
2. Encoder configured-ratio fallback:
   `clean quadrature + net displacement` -> `strong quadrature proof independent of net displacement`.
3. Startup BAT_LVL1:
   `immediate filtered voltage comparison` -> `2 s grace + 500 ms persistence`.
4. Tester idle-current interpretation:
   `zero current can look unavailable` -> `explicit valid-zero bridge-OFF classification`.
5. Regression:
   added post-detect side-local ARM/SET contract.

No current target increase was used to hide Detect failures.
