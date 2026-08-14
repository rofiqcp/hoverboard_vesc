# Audit v10 Hardware Log → V11 Fix

Source audited: `hoverboard_vescv10(1).zip`  
Hardware log audited: `vesc_full_test_20260812_215947.zip`

## Kesimpulan utama

Kegagalan v10 bukan terutama pada routing `SET_DUTY`, `SET_CURRENT`, `SET_RPM`, atau `SET_POS`. Zero-command routing/cross-side tests menunjukkan context LEFT/RIGHT dan protocol facade hidup. Non-zero tests kemudian SKIP karena sensor Detect gagal lebih dulu dan safety gate menolak ARM.

## LEFT Detect evidence

Saat target Detect hanya 0.500 A, first FAST CURRENT GUARD snapshot menunjukkan kira-kira:

- Id = 12.750 A;
- Iq = 22.344 A;
- phase raw = [2100, 2012];
- phase offsets = [2801, 2731];
- delta = [701, 719] count ≈ [14.02, 14.38] A;
- duty vector sekitar [-47, -22, 47], duty absolute sekitar 0.37%;
- DC-link raw/offset sekitar 1950/1946, hanya beberapa count beda.

## RIGHT Detect evidence

Saat Hall Detect RIGHT:

- Id ≈ -26.797 A;
- Iq ≈ 0.021 A;
- phase raw = [2020, 2004];
- phase offsets = [2768, 2749];
- delta = [748, 745] count ≈ [14.96, 14.90] A;
- duty absolute sekitar 0.26%;
- DC-link raw/offset sekitar 1942/1932.

## Interpretasi

Phase current menunjukkan step hampir sama pada kedua channel ketika bridge memasuki domain PWM, sementara DC-link shunt tidak ikut berubah sebesar itu dan duty masih sangat kecil. Ini konsisten dengan **common-mode/domain offset mismatch**, bukan bukti bahwa motor benar-benar tiba-tiba menerima puluhan ampere.

Di source v10, current-offset calibration dilakukan dengan MOE OFF. Pada operasi FOC, low-side shunt dibaca pada domain switching aktif. Karena itu baseline OFF dan baseline active tidak boleh dicampur.

## Mengapa VESC Tool menampilkan DRV

Source v10 memetakan custom internal fault non-overcurrent ke angka VESC `3`. Pada enum VESC, angka tersebut berarti `FAULT_CODE_DRV`. Akibatnya sensor/commissioning failure tampil sebagai gate-driver fault walau firmware tidak mempunyai bukti hardware DRV fault.

## V11 corrective actions

1. Active LOW-FET zero-vector offset calibration.
2. Final active-domain raw/residual evidence sebelum MOE release.
3. 6-sample OFF→ON bridge warm-up sebelum current PI/standard current valid.
4. Custom internal fault tidak dipalsukan menjadi VESC DRV.
5. Exact SET raw/normalized/runtime provenance.
6. End-to-end motion test untuk Duty/Current/RPM/Position + peer isolation.

## What remains hardware-dependent

Host regression dapat membuktikan arithmetic, routing, parser, state gating, and diagnostic contracts, tetapi tidak dapat membuktikan ADC voltages, Hall/encoder wiring, MOSFET bridge behavior, atau rotor motion pada board fisik. Karena itu v11 harus sekali dijalankan memakai full hardware tester sebelum disebut validated-on-hardware.
