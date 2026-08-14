# CHANGELOG V11

## Root-cause fix: phase-current offset domain

- v10 mengambil phase-current offset saat MOE OFF.
- Hardware log menunjukkan raw phase berpindah sekitar 700–750 count saat PWM/low-FET domain aktif, sedangkan DC-link shunt tidak menunjukkan lompatan sebanding.
- V11 mengkalibrasi offset dengan active LOW-FET zero-vector (CCR A/B/C = 0, MOE ON).
- Menambahkan motion guard saat current calibration dan selalu release bridge setelah transaksi selesai.

## OFF → ON current-domain warm-up

- Menambahkan 6 ADC-sample warm-up per motor.
- Selama warm-up, zero-vector dipertahankan.
- Current PI dan standard current telemetry belum dianggap valid.
- Transition counter direkam untuk debug.

## Fault facade correction

- `FAULT_CODE_DRV` tidak lagi digunakan sebagai catch-all untuk custom internal faults.
- Absolute over-current tetap dipetakan ke VESC fault yang sesuai.
- Exact internal fault tetap dapat dibaca lewat HBTS.

## Exact SET provenance

Per-side diagnostics sekarang menyimpan:

- last VESC SET command ID;
- raw host/wire integer;
- normalized internal setpoint;
- runtime run mode;
- route/SET counters.

Tester mencocokkan exact raw scaling dan target context untuk Duty, Current, RPM, Position dan command lain.

## HBTS v10

Diagnostic extension version dinaikkan dari 9 ke 10. Tail baru:

- current-cal sampling mode;
- bridge active mask;
- per-side warm-up;
- bridge transitions;
- last SET host raw / normalized / run;
- final active raw current ADC [6];
- final active residual [6].

Self-test payload size saat packaging = 354 byte.

## Tester

- Release `V11`.
- `set_command_trace.csv` baru.
- `04a_no_false_drv_mapping` baru.
- Motion test menuntut bridge activation, warm-up completion, current-domain validity, standard current validity, dan tidak adanya fake DRV.
- Current calibration memvalidasi final active-domain residual, bukan phase raw setelah MOE release.

## Regression result

- `tests/run_host_tests.sh`: 14/14 PASS.
- Cortex-M3 FOC/current ISR/sensor hot-path codegen audit: PASS.
- Python V11 protocol/HBTS self-test: PASS.
- Hardware motor validation: harus dilakukan pada board fisik.
