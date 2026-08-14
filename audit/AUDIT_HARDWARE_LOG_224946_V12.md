# Audit Hardware Log V11 20260812_224946 -> V12

## Kesimpulan
Kegagalan V11 terbaru terjadi setelah current-offset domain sudah benar. Kedua Detect timeout sehingga safety gate secara benar tidak pernah mengizinkan empat SET non-zero.

## Bukti current calibration V11 sudah sehat
`final_active_raw` sekitar LEFT 2080/2009 dan RIGHT 2020/2004, sedangkan `final_active_residual` sekitar 0,0,1,0,-1,-1 count. Ini menghapus hipotesis lama bahwa trip 14–15 A palsu V10 masih menjadi akar masalah.

## LEFT encoder Detect
- 135 diagnostic samples aktif.
- Requested current: 0.50 A sepanjang sweep.
- Measured-current L1 mean: 0.503 A; Id mean 0.446 A.
- Vd: min 468, mean 493.45, max 500 internal.
- Position ticks hanya sekitar 166..170 walau encoder valid-edge counter naik hingga 185.
- Terminal result: timeout; offset 1001 / ratio 0 placeholder failure.

Interpretasi: current loop mencapai target rata-rata, tetapi medan commissioning hampir selalu menempel ke voltage cap lama dan rotor tidak mengikuti commanded electrical phase secara kontinu.

## RIGHT Hall Detect
- 135 diagnostic samples aktif.
- Requested current: 0.50 A.
- Measured-current L1 mean: 0.346 A; Id mean 0.300 A.
- Vd: min 479, mean 499.65, max 500 internal.
- Hall observed saat aktif: state 2 = 71 sample, state 3 = 62, state 6 = 2; tidak pernah mendapatkan enam state.
- Position sekitar -1..1 tick.
- Terminal result: Hall table seluruhnya 255, timeout.

Interpretasi: RIGHT lebih jelas voltage-limited; 0.5-A current target tidak dapat dipertahankan sepanjang sweep dan rotor hanya berpindah di sebagian Hall sectors.

## Kenapa SET_DUTY/CURRENT/RPM/POS tidak berjalan
Tester V11 memerlukan sensor detect PASS sebelum mengirim command non-zero. Karena LEFT dan RIGHT sama-sama gagal commissioning, test motion di-skip. Routing zero-valued commands dan virtual-CAN sebelumnya PASS, sehingga failure tidak membuktikan parser empat SET salah.

## Telemetry issue terpisah
HBTS V11 menandai `standard_current_valid` dari `last_valid_ms`, yang baru diperbarui setelah GET_VALUES menguras accumulator. Ini bisa membuat diagnostic terlihat invalid saat pending current samples sebenarnya sudah ada. Selain itu `motor_current_from_raw()` V11 memaksa Imotor=0 ketika Iq hampir nol, sehingga D-axis commissioning/standstill tidak terlihat sebagai Motor Current.

## False DRV issue
HBTS log V11 sendiri menunjukkan internal error NONE dan VESC fault NONE pada failure terminal, tetapi VESC Tool dilaporkan menampilkan DRV. Karena V11 tester belum menyimpan exact raw fault byte dari standard GET_VALUES/SETUP, log tersebut belum cukup untuk menentukan apakah masalah berasal dari packet wire, stale GUI state, atau jalur lain. V12 menambahkan raw standard-wire logging dan firmware-side last-emitted-fault provenance agar ini dapat dibuktikan pada run berikutnya.

## Perubahan V12 yang langsung menargetkan bukti di atas
- voltage cap commissioning 500 -> 2400 tanpa menaikkan requested current 0.50 A;
- Hall 3 forward + 3 reverse sweeps;
- encoder VESC window 6 electrical revolutions;
- current validity melihat pending accumulator samples;
- Imotor VESC-compatible magnitude/sign behavior;
- exact GET_VALUES/GET_VALUES_SETUP fault-byte logging;
- richer commissioning progress and exact SET trace.
