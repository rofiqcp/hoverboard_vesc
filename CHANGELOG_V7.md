# Hoverboard VESC V7 — Changelog

V7 dibuat dari audit hardware `vesc_full_test_20260812_182948.zip`. Tujuannya bukan mengendurkan proteksi, tetapi memperbaiki semantik current telemetry dan sensor commissioning yang terbukti salah pada V6.

## 1. Standard VESC current tidak lagi memakai data shunt saat bridge OFF

Pada log V6, motor dilepas/MOE OFF tetapi VESC Tool masih dapat menampilkan phase-current puluhan ampere dan Input Current hingga puluhan/ratusan ampere, sementara PSU tidak melihat arus setara. Raw phase ADC memang berubah besar ketika rotor diputar pasif; kondisi ini tidak boleh diperlakukan sebagai arus motor yang valid.

V7 memisahkan:

- `HBTS raw phase current`: tetap direkam untuk diagnosis.
- `HBTS raw DCL/DCR`: tetap direkam untuk diagnosis.
- `VESC Motor Current / Id / Iq`: hanya berasal dari current measurement yang valid saat bridge aktif.
- `VESC Input Current LEFT`: DCL tervalidasi.
- `VESC Input Current RIGHT`: DCR tervalidasi.
- whole-board battery current diagnostic: `DCL + DCR` tervalidasi.

Saat MOE OFF, standard VESC current fields = 0 A. Raw HBTS tidak dipalsukan menjadi nol.

## 2. Tidak ada lagi fallback realtime ke snapshot FOC 8 kHz

VESC current accumulator diambil pada background rate. V6 dapat jatuh kembali ke nilai instantaneous ketika VESC Tool membaca lebih cepat daripada accumulator. Hal ini membuat GUI terlihat agresif/spiky.

V7: jika read-reset accumulator belum memiliki sampel baru, current standard mengembalikan 0 untuk interval itu dan menunggu accumulated sample berikutnya. Instantaneous values hanya tersedia melalui HBTS debug.

## 3. Detect Current VESC benar-benar current-regulated

Bug V6: `Detect Current = 1 A` dipetakan menjadi forced open-loop voltage kecil, bukan 1 A. Akibatnya PSU hampir tidak melihat arus, rotor tidak mendapat alignment torque yang cukup, tetapi phase ADC bisa tetap menunjukkan angka besar akibat kondisi sampling/common-mode/back-EMF.

V7:

- request 1.0 A -> 800 internal units (`50 ADC count/A`, internal `<<4`).
- forced electrical phase aktif.
- `Id target = requested detect current`.
- `Iq target = 0`.
- D-axis current PI tetap aktif.
- commissioning cap 0.25…5 A.
- phase-current software qualification 7 A + hard DC-link guard tetap aktif.

Encoder alignment power-on yang memang voltage-based tetap dipisahkan dari sensor Detect.

## 4. Detect dibuat transactional

V6 mencabut `hall_calibrated` / `encoder_calibrated` ketika Detect baru dimulai. V7 tidak menghapus proof lama sampai candidate baru benar-benar lolos. Timeout/failure tidak lagi menghancurkan calibration yang sebelumnya bekerja.

Saat commissioning selesai/abort/timeout, V7 juga membersihkan:

- forced phase ownership,
- current commissioning flag,
- voltage override,
- Id/Iq target,
- control mode,
- hardware gate.

## 5. Debug HBTS v5

Tambahan diagnosis:

- bridge MOE nyata,
- current measurement valid,
- raw vs validated DCL/DCR,
- DC telemetry reject count,
- detect current-control active,
- requested detect current,
- measured detect phase current,
- forced electrical phase,
- FOC Id/Iq target,
- raw/offset phase ADC,
- ISR last/max/deadline cycles.

## 6. Python one-shot tester V7

Tester selama Detect melakukan polling HBTS dan menyimpan target-vs-measured current. Ia juga memperingatkan jika:

- FOC target tidak pernah menerima requested current,
- bridge MOE tidak pernah aktif,
- measured current terlalu kecil,
- phase ADC menyentuh rail,
- phase ADC menyimpang >1000 count (~20 A pada skala board) saat requested detect hanya <=5 A.

Passive-spin test mengharuskan standard VESC current tetap mendekati 0 saat bridge OFF, sementara raw ADC tetap masuk log.

## 7. Scheduler tidak diubah lagi

Log V6 terbaru membuktikan interleaving bekerja:

- PWM/ADC: 16 kHz.
- FOC LEFT: 8 kHz.
- FOC RIGHT: 8 kHz.
- hardware log max ISR sekitar 3186 cycle.
- deadline 4000 cycle.
- overrun count 0.

Karena itu V7 tidak menaikkan deadline atau menurunkan PWM hanya untuk menyembunyikan masalah lain.
