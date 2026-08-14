# CHANGELOG V12

## 1. Root cause dari hardware log V11
V11 sudah menyelesaikan perpindahan common-mode ADC: `final_active_residual` tinggal sekitar 0–1 count. Kegagalan terbaru bukan offset lagi, tetapi commissioning tidak menghasilkan gerakan/sweep yang cukup. Pada 135 sample aktif per sisi, LEFT `Vd` rata-rata 493.45 dari cap 500 dan RIGHT 499.65 dari 500. RIGHT hanya mengamati Hall raw 2, 3, dan 6.

## 2. Commissioning voltage headroom
- `FOC_COMMISSIONING_VOLTAGE_MAX`: 500 -> 2400 internal.
- Current target tidak dinaikkan; tester default tetap 0.50 A.
- Fast current guard, current-domain validation, dan hard safety tetap aktif.

## 3. Hall / encoder detect
- Hall: 3 electrical revolution maju + 3 mundur.
- Rate: 3 Q4/ms = 0.1875 electrical degree/ms.
- Encoder VESC detect: 6 electrical revolutions.
- Tambah diagnostic direction, forward/reverse/completed cycles, motion counter/age, observed Hall/encoder masks, encoder delta.

## 4. Standard VESC telemetry
- `standard_current_valid` sekarang valid bila bridge + current measurement valid dan accumulator punya pending sample atau held sample yang masih fresh.
- `Imotor` tidak lagi di-zero-kan ketika Iq~0. Magnitudo memakai Id/Iq dan tanda mengikuti Vq*Iq seperti VESC FOC.
- `GET_VALUES` dan `GET_VALUES_SETUP` tetap read-reset/short-hold per motor dan tidak menyalin current peer.

## 5. False DRV proof
- `fault_code()` hanya mengeluarkan NONE atau real ABS_OVER_CURRENT; sensor/readiness/commissioning internal tidak dipalsukan sebagai DRV.
- Tambah exact standard-wire provenance per node: count, last command, last emitted fault byte, payload length.
- Python tester membaca GET_VALUES dan GET_VALUES_SETUP nyata dan fail langsung bila fault byte 3 terlihat.

## 6. Exact SET debug
- `set_command_contract.csv` selalu dibuat untuk Duty +/- , Current +/- , RPM +/- , Position +/- dengan command ID dan raw int32 VESC.
- `set_command_trace.csv` selalu punya header.
- Saat commissioning PASS, setiap non-zero SET harus cocok pada: requested -> raw int32 -> firmware decoder -> normalized setpoint -> runtime -> ARM/MOE -> current-domain -> VESC telemetry.
- Cross-side isolation diperiksa setiap sample.

## 7. HBTS v11
Tail append-only baru membuat payload diagnostic 388 byte dan menambah commissioning progress serta standard-wire proof tanpa menggeser field lama.

## 8. Regression
- `tests/run_host_tests.sh`: 15/15 PASS.
- `tests/test_v12_commissioning_contract.py`: PASS.
- Cortex-M3 codegen hot-path audit: PASS.
- Python tester self-test: PASS, payload 388 byte.
