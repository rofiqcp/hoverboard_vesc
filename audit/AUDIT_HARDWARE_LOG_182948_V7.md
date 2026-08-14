# Audit Hardware Log 2026-08-12 18:29:48 → V7

Sumber audit: `vesc_full_test_20260812_182948.zip` dari `/dev/ttyUSB0`.

## Ringkasan hasil V6

- PASS: 15
- FAIL: 3
- WARN: 1
- SKIP: 16

First failures:

1. idle telemetry LEFT sempat menunjukkan Id besar,
2. LEFT Encoder Detect timeout/fail,
3. RIGHT Hall Detect timeout/fail.

Seluruh Duty/Current/RPM/POS kemudian di-SKIP karena sensor detect prerequisite belum lolos.

## Yang sudah terbukti sehat

### Current zero calibration

Kalibrasi ulang menghasilkan baseline sekitar:

- LEFT phase offsets: rlA ≈ 2747, rlB ≈ 2677,
- RIGHT phase offsets: rrB ≈ 2761, rrC ≈ 2742,
- DCL ≈ 1944,
- DCR ≈ 1931,
- ADC hardware calibration PASS,
- block-mean stability PASS.

### Current scale

Firmware dan log konsisten memakai:

- 50 ADC count/A,
- internal shift = 4,
- 800 internal unit/A.

V7 tidak mengubah angka ini secara arbitrer.

### ISR timing

Hardware log:

- overrun_count = 0,
- max ≈ 3186 cycles,
- deadline = 4000 cycles.

Jadi masalah sensor Detect terbaru bukan lagi deadline ISR.

## Masalah 1 — phase current saat passive spin tidak valid sebagai motor current

Saat motor RELEASE/MOE OFF dan rotor diputar dengan tangan, raw phase-current dapat bergeser sangat besar. Contoh dari log:

- LEFT instantaneous Id sampai sekitar -25.6 A dan Iq sekitar -24.3 A,
- RIGHT instantaneous Id sampai sekitar -35.4 A dan Iq sampai sekitar +26.5 A,
- tetapi raw DCL/DCR hanya bergerak sedikit di sekitar offset dan PSU tidak menunjukkan arus setara.

Kesimpulan: phase-shunt ADC masih berguna sebagai raw diagnostic pada kondisi ini, tetapi tidak boleh dipublikasikan sebagai standard VESC motor current ketika bridge OFF.

V7: standard VESC Id/Iq/Imotor = 0 saat current measurement tidak valid; raw HBTS tetap dicatat.

## Masalah 2 — Input Current/Battery Current V6 tidak fisik

Pada passive spin V6, standard `Input Current` tercatat puluhan sampai >100 A negatif. Diagnostic whole-board bahkan sempat menunjukkan sekitar +288 A / -183 A, sedangkan raw DCL/DCR hanya beberapa ADC count dari offset.

Ini bukan arus PSU yang sebenarnya. V7 membuat satu authoritative path:

- LEFT Input Current = validated DCL,
- RIGHT Input Current = validated DCR,
- whole-board = validated DCL + validated DCR,
- MOE OFF = validated current 0,
- raw values tetap di HBTS.

## Masalah 3 — V6 Detect Current ternyata open-voltage

V6 mengubah requested VESC Detect Current menjadi angka forced voltage kecil. Maka `1 A` tidak pernah berarti `Id target = 1 A`.

Gejalanya cocok dengan hardware:

- VESC Tool menampilkan current berubah besar,
- PSU tetap hampir 0 A,
- rotor tidak mengikuti sweep secara meyakinkan,
- Encoder/Hall Detect timeout.

V7 mengubah commissioning menjadi forced-phase D-axis current control:

`requested A -> internal units -> Id target -> current PI -> PWM`, dengan `Iq=0`.

## Masalah 4 — polling GUI dapat mencampur averaged dan instantaneous data

V6 memiliki fallback ke raw FOC snapshot ketika accumulator kosong. Jika VESC Tool meminta data lebih cepat dari producer average, satu sample dapat terlihat seperti spike besar.

V7 tidak memiliki fallback tersebut. Standard VESC current selalu berasal dari accumulated/validated samples.

## Apa yang harus dibuktikan pada run V7

Run berikut harus menjawab secara eksplisit:

1. Apakah `bridge_moe=true` selama Detect?
2. Apakah `detect_target_A≈1.0 A` mencapai `foc_id_target_A≈1.0 A`?
3. Berapa `detect_measured_A`?
4. Apakah raw phase ADC tetap rail/menyimpang >20 A-equivalent ketika target hanya 1 A?
5. Apakah PSU sekarang menunjukkan arus yang konsisten dengan current-regulated detect?
6. Apakah encoder A/B menghasilkan valid edges selama sweep?
7. Apakah Hall RIGHT melewati enam state legal dan menghasilkan LUT?

Jika target current benar tetapi measured phase current kembali puluhan ampere sementara PSU tetap ~0 A, fokus berikutnya adalah current-sense sampling/common-mode/polarity, bukan scaling `50 count/A`.
