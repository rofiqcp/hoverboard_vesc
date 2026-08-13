# AUDIT DIFF V1–V12 → V13

Audit ini dibuat dari source yang dikirim ulang pengguna, bukan dari asumsi versi. Fokusnya adalah jalur yang terbukti relevan oleh hardware log `vesc_full_test_20260812_233410`.

## Evolusi jalur kritis

| Versi | Raw encoder observer | TX depth | HBTS | Hall F/R | Enc cycles | ARM encoder align | Active-domain current cal | Detect reply durable | Per-motor terminal | false-DRV tester | Tester release |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| V1 | left_u/left_v | 3U | - | -/- | - | voltage | no | no | no | - | - |
| V2 | left_u/left_v | 3U | - | -/- | 12U | voltage | no | no | no | - | - |
| V3 | left_u/left_v | 3U | 1U | -/- | 12U | voltage | no | no | no | - | - |
| V4 | left_u/left_v | 3U | 3U | -/- | 12U | voltage | no | no | no | - | - |
| V5 | left_u/left_v | 3U | 4U | -/- | 12U | voltage | no | no | no | - | - |
| V6 | left_u/left_v | 3U | 4U | -/- | 12U | voltage | no | no | no | - | - |
| V7 | left_u/left_v | 3U | 5U | -/- | 12U | voltage | no | no | no | - | - |
| V8 | left_u/left_v | 3U | 7U | -/- | 12U | voltage | no | no | no | - | - |
| V9 | left_u/left_v | 3U | 7U | -/- | 12U | voltage | no | no | no | - | V9 |
| V10 | left_u/left_v | 3U | 9U | -/- | 12U | voltage | no | no | no | - | V10 |
| V11a | left_u/left_v | 3U | 10U | -/- | 12U | voltage | no | no | no | - | V11 |
| V11-fixed | left_u/left_v | 3U | 10U | -/- | 12U | voltage | yes | no | no | - | V11 |
| V12 | left_u/left_v | 3U | 11U | 3U/3U | 6U | voltage | yes | no | no | bug | V12 |
| V13 | encoder_a/encoder_b | 6U | 12U | 3U/3U | 6U | current | yes | yes | yes | yes | V13 |

### Temuan diff yang menjelaskan log terbaru

1. **LEFT encoder diagnostic/commissioning A/B salah sumber.** `left_encoder.c` mendefinisikan PB6=TIM4_CH1/A dan PB7=TIM4_CH2/B, tetapi V12 memanggil hardware observer dengan `left_u,left_v` = PB5(Z),PB6(A). TIM4 count tetap hidup, sehingga log bisa memiliki banyak edge valid sementara `observed_encoder_mask` hanya dua state. V13 memakai PB6/PB7 secara eksplisit.
2. **Terminal `COMM_DETECT_*` bukan transaksi lossless sampai V12.** Queue V12 hanya tiga slot dan `send_payload()` membuang frame saat penuh. `detect_service()` lalu tetap menghapus `pending_detect`. Log terbaru menunjukkan RIGHT Hall sudah SUCCESS tetapi reply tidak pernah tiba; V13 menahan terminal result dan retry sampai enqueue berhasil.
3. **State commissioning live tunggal.** V1–V12 memakai satu `sensorCal`. Ini cukup untuk satu sweep, tetapi tidak cukup untuk menyimpan hasil terminal LEFT dan RIGHT secara independen saat protocol reply tertunda. V13 menambah snapshot terminal per motor.
4. **Encoder ratio V1–V12 mengasumsikan rotor mengikuti full electrical sweep.** Log terbaru menunjukkan TIM4 edge bersih tetapi wheel dither kecil. V13 tetap mencoba strict ratio terlebih dahulu; bila tidak valid, configured pole-pair fallback hanya diizinkan dengan all-4 A/B states, >=8 edge TIM4, invalid transition rendah, dan signed movement.
5. **Power-on encoder alignment V12 terlalu lemah dan berbasis tegangan.** Hardware log membutuhkan Vd sekitar ~500 internal untuk current 0.5 A, sedangkan ARM alignment V12 hanya 30→90 voltage internal. V13 memakai D-axis current-regulated 0.5 A dengan soft ramp dan fast-current guard.
6. **Tester V12 false-DRV crash.** Positional string `"false_drv_setup"` masuk ke parameter `mask`; firmware wire log justru menunjukkan seluruh standard GET_VALUES/SETUP yang tercatat fault=0. V13 menjadikan `mask` keyword-only dan memperbaiki call.

## Diffstat file kritis per versi

| Transisi | Baris + | Baris - | File berubah | File kritis |
|---|---:|---:|---:|---|
| V1→V2 | 398 | 58 | 3 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c |
| V2→V3 | 1263 | 1 | 3 | Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V3→V4 | 526 | 51 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V4→V5 | 238 | 100 | 3 | Src/motor.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V5→V6 | 276 | 123 | 5 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, Src/motor_sensor.c, tools/vesc_full_test.py |
| V6→V7 | 508 | 138 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V7→V8 | 307 | 59 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V8→V9 | 89 | 7 | 1 | tools/vesc_full_test.py |
| V9→V10 | 714 | 129 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V10→V11a | 337 | 9 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V11a→V11-fixed | 417 | 392 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V11-fixed→V12 | 388 | 94 | 3 | Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |
| V12→V13 | 357 | 142 | 4 | Src/motor.c, Src/runtime_control.c, Src/vesc_protocol.c, tools/vesc_full_test.py |

## Batas validasi

Diff dan host regression dapat membuktikan routing, state machine, parser, fixed-point contracts, dan source invariants. Putaran motor fisik, polaritas encoder aktual, torque margin, dan keberhasilan SET non-zero tetap harus dibuktikan pada board dengan roda terangkat dan supply current-limit.
