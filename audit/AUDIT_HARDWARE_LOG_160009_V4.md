# Audit Hardware Log 2026-08-12 16:00:09 — V4

## Ringkasan

Input audit: `vesc_full_test_20260812_160009.zip` dari hardware Linux `/dev/ttyUSB0`.
Run V3 menghasilkan **28 PASS / 18 FAIL**. Kegagalan motion bukan 18 bug terpisah; log menunjukkan tiga akar masalah utama:

1. **Zero offset phase-current salah** sehingga Id/Iq besar saat inverter OFF.
2. **Detektor ISR overrun V3 false-positive** dan counter saturasi, lalu commissioning Hall/encoder dibatalkan.
3. **MCCONF same-value round-trip tidak idempotent**.

Setelah sensor detect gagal, `calibrated=false`, maka Duty/Current/RPM/POS memang harus ditolak oleh safety gate.

## Bukti dari log aktual

Pada kondisi connect/idle (RPM = 0, output tidak armed):

| Node | Id idle | Iq idle | DC-link current | ISR overrun count |
|---|---:|---:|---:|---:|
| LEFT/local | sekitar -14.3 A | sekitar -24.8 A | sekitar +0.02 A | 65535 |
| RIGHT/virtual CAN | sekitar -25.36 A | sekitar +14.64 A | sekitar 0.00 A | 65535 |

Nilai Id/Iq puluhan ampere pada motor diam dengan DC-link current hampir nol adalah bukti kuat bahwa phase-current zero reference V3 tidak valid. Karena Clarke/Park adalah rotasi linear, kesalahan offset phase current akan muncul sebagai offset Id/Iq yang besar.

Detect yang gagal pada run tersebut:

- LEFT encoder: `offset=1001`, `pole_pairs=0`, `success=false`.
- RIGHT Hall: seluruh Hall table `255`, `valid_count=0`, `success=false`.
- Saat commissioning, `CONTROL_ISR_OVERRUN` dan overrun mask muncul; semua motion test berikutnya kemudian gagal dengan `sensor not calibrated` / `never armed`.

## Definisi current yang dipakai V4

### Motor current

`Imotor` **bukan DCL/DCR**. Untuk FOC:

`Imotor = sign(Iq) * sqrt(Id^2 + Iq^2)`

Id dan Iq berasal dari phase current setelah raw ADC offset dikurangi, lalu Clarke dan Park transform. Nilai ini digunakan sebagai field **Motor Current** pada VESC telemetry.

### Input / battery current

DCL dan DCR adalah sensor DC-link:

- VESC LEFT/local `Current In` = DCL.
- VESC RIGHT virtual-CAN `Current In` = DCR.
- Whole-board battery current = `DCL + DCR`.

**Jangan** menghitung battery current dengan `Imotor_left + Imotor_right`. Phase/motor current dan DC-link current berbeda karena duty/modulation serta losses. V4 HBTS v3 mengirim `battery_current_total_A` secara eksplisit agar tidak ambigu.

## Perbaikan V4 — current calibration

V3 tidak lagi dipakai. V4 melakukan:

1. Initialize ADC1/ADC2.
2. Jalankan STM32F1 ADC hardware self-calibration pada ADC1 dan ADC2 sebelum trigger PWM timer berjalan.
3. Initialize timer/runtime, start ADC regular conversions.
4. Release kedua bridge dan paksa TIM1/TIM8 MOE OFF.
5. Buang **256 current-loop samples** sebagai settling.
6. Average **2048 samples** untuk enam physical current channels:
   - LEFT phase A (`rlA`)
   - LEFT phase B (`rlB`)
   - RIGHT phase B (`rrB`)
   - RIGHT phase C (`rrC`)
   - LEFT DC-link (`dcl`)
   - RIGHT DC-link (`dcr`)
7. Validasi raw mean dan span/noise tiap channel.
8. Baru tandai `current_cal_valid=true`; sebelum itu arming dan sensor commissioning ditolak.

Offset dikalibrasi **di raw ADC phase-current domain sebelum Clarke/Park**. Tidak ada independent learned `Id offset` atau `Iq offset`, karena hal itu dapat menyembunyikan kesalahan phase mapping atau rotor angle.

## Perbaikan V4 — VESC Tool current-offset integration

Compatibility profile tetap VESC firmware wire-format 6.00.

- `foc_offsets_cal_on_boot` dilaporkan aktif.
- `foc_offsets_current[0..2]` di MCCONF menampilkan physical phase-current offsets yang sedang aktif.
- Karena board hanya mengukur dua phase currents per inverter, field ketiga dilaporkan 0 (phase ketiga direkonstruksi, tidak mempunyai ADC sendiri).
- DCL/DCR **tidak** dimasukkan ke `foc_offsets_current`; keduanya mempunyai calibration internal terpisah dan tampil di HBTS diagnostics.
- Manual write terhadap VESC phase-current offset sengaja tidak menjadi authoritative writer. Firmware selalu memakai safe zero-current averaging dengan kedua bridge released.

Kalibrasi dapat dipicu langsung dari VESC Tool **Terminal**:

- `hb_current_cal` — safe recalibration, release kedua bridge terlebih dahulu.
- `foc_dc_cal` — alias kompatibilitas untuk command yang sama.
- `hb_current_status` — lihat state IDLE/SETTLING/COLLECTING/VALID/FAILED.
- `hb_help` — daftar command port ini.

Tester Python `--full` juga menjalankan safe current recalibration sebelum sensor detect/motion.

## Perbaikan V4 — ISR overrun

V3 memakai heuristik DMA completion flag yang pada hardware dapat terbaca lagi pada circular DMA dan menghasilkan false overrun sampai `65535`.

V4 menggunakan **Cortex-M3 DWT CYCCNT**:

- `motorControlIsrLastCycles`
- `motorControlIsrMaxCycles`
- `motorControlIsrDeadlineCycles`
- load percentage di HBTS/debug CSV

Overrun hanya dicatat ketika waktu eksekusi current ISR nyata >= satu PWM deadline. Persistent overrun tetap fail-safe.

## Perbaikan V4 — MCCONF idempotence

`GET_MCCONF -> SET_MCCONF exact same bytes -> GET_MCCONF` sekarang mempunyai exact-wire no-op path. Tester V4 menyimpan:

- `local_mcconf_before.bin`
- `local_mcconf_after.bin`
- `local_mcconf_diff.json`
- file yang sama untuk RIGHT

Jika masih berubah di hardware, byte offset pertama langsung terlihat dalam ZIP log.

## HBTS v3 debug evidence

V4 menambahkan read-only diagnostic evidence berikut tanpa mengubah protocol normal VESC Tool:

- current calibration state/valid
- ADC1/ADC2 hardware self-cal status
- collected/target samples
- failure mask + generation
- raw ADC keenam current channel
- stored offsets keenam current channel
- span/noise keenam channel
- residual keenam channel
- Id/Iq/Imotor
- DCL/DCR dan total board battery current
- armed/output/feedback/calibrated flags
- Hall raw / encoder AB / CPR / pole pairs
- detect result code
- ISR last/max/deadline cycles dan overrun masks
- UART parser/TX counters
- EEPROM verification
- PA2/PA3 APP ADC

## Urutan validasi hardware V4

Full tester sengaja melakukan current-zero validation **sebelum** sensor detect dan motion. Bila current calibration gagal, jangan lanjut menyalahkan Hall/encoder atau PID; lihat `current_cal_failure_mask`, raw/span/residual di ZIP.
