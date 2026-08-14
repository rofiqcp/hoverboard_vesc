# Changelog V15 — VESC SET/GET flow + V1-style DMA/ISR

## Dasar perubahan

V15 dibuat setelah membaca `vesc_full_test_20260813_010411.zip` dan membandingkan lagi source V1 dengan V14. Fokus release ini bukan menambah safety, tetapi menghapus gate diagnostik/sticky yang tidak perlu dari jalur PWM sambil mempertahankan proteksi fisik dan kalibrasi yang sudah terbukti benar.

## SET/GET

- `COMM_SET_DUTY`, `COMM_SET_CURRENT`, `COMM_SET_RPM`, dan `COMM_SET_POS` mengikuti boundary command VESC: decode scaling wire, route motor context, kirim ke runtime setter, reset/alive timeout.
- Tidak ada lagi threshold facade `run = abs(setpoint) > ...` untuk empat SET utama.
- LEFT local dan RIGHT virtual-CAN tetap mempunyai context/counter/setpoint independen.
- `GET_VALUES` mempertahankan urutan/scaling VESC 6.00 dan read-reset average Motor Current/Input Current/Id/Iq.
- Current accumulator tetap di slow loop agar tidak menambah pekerjaan ISR 16 kHz.

## DMA/current/sensor path

- `CONTROL_PWM_FREQUENCY_HZ = 16000`.
- `CONTROL_FOC_MOTOR_FREQUENCY_HZ = 16000`.
- Menghapus interleave `motorFocInterleaveSlot`; LEFT dan RIGHT FOC dipanggil setiap DMA current ISR.
- Direct ADC current delta dipulihkan sebagai hot path: offset - ADC raw.
- Satu coherent GPIO snapshot per sisi per ISR.
- LEFT Encoder tetap menggunakan TIM4 PB6/PB7 (A/B), bukan bug V1 PB5/PB6.
- DC-link hard current chopping tetap sample-local lewat MOE.
- ISR overrun tetap dicatat tetapi tidak menghapus runtime command, calibration ownership atau MOE secara sticky.
- Commissioning fast-current software guard dikeluarkan dari hot ISR; hard physical DC chop tetap ada.
- Bridge active-domain prime dipangkas menjadi satu released-domain sample discard.

## Safety/gating disederhanakan

Dihapus sebagai PWM permission/sticky disarm:
- ISR overrun fault latch;
- slow-loop SensorHealth stale fault gate;
- commissioning fast-current streak gate pada DMA hot path;
- historical fault-report-window pre-arm blocker.

Tetap dipertahankan:
- valid current offsets;
- current calibration/commissioning ownership;
- same-sample sensor feedback validity;
- encoder alignment untuk incremental A/B;
- configured setpoint bounds;
- sample-local DC-link current chopping;
- VESC communication timeout;
- catastrophic ADC/DMA heartbeat loss.

## Calibration

- Active LOW-FET zero-vector current calibration V11+ dipertahankan.
- Hall circular averaging V14 dipertahankan.
- Encoder PB6/PB7 dan guarded fallback dipertahankan.
- Encoder detect proof sekarang cumulative sepanjang satu transaction; failed sweep window tidak lagi menghapus A/B mask/edge proof.
- Commissioning Ki*dt cap disesuaikan dari 82 ke 41 Q16 saat per-motor current-loop kembali 8 kHz -> 16 kHz, menjaga integral gain per detik kira-kira sama.

## Tester

- `TESTER_RELEASE = V15`.
- Memperbaiki Hall sensor-type zero bug (`0 or -1 -> -1`) yang membuat RIGHT Hall V14 terlihat FAIL walaupun firmware SUCCESS.
- Detailed standard-wire, detect, command route, ARM/MOE/current/ERPM/position trace dipertahankan.

## Regression

V15 menambah source contract yang memastikan:
- dual FOC 16 kHz dan tidak ada interleave;
- V1-like direct current/sensor sequence;
- active-domain current calibration tetap ada;
- PB6/PB7 tetap benar;
- safety hot-path minimal;
- SensorHealth tidak sticky-disarm;
- exact four-SET wire scaling dan direct routing;
- GET read-reset averages;
- cumulative encoder evidence;
- Hall type 0 tidak dianggap missing;
- commissioning Ki*dt tetap disesuaikan untuk 16 kHz.
