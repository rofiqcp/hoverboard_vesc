# Audit Hardware Log 2026-08-12 19:24:48 → V8

Source log: `vesc_full_test_20260812_192448.zip` from `/dev/ttyUSB0` hardware run V7.

## Kesimpulan first-failure

Current-zero calibration sudah sehat. ISR normal juga umumnya di bawah deadline. First failure nyata terjadi pada LEFT Encoder Detect: request current masih di bawah 1 A tetapi phase-current FOC terbaca sekitar 27–35 A. RIGHT Hall failure pada tester V7 bukan bukti Hall kanan 000; frame `before_detect` yang diberi label RIGHT ternyata membawa `node_id=10 side=LEFT`, sedangkan RIGHT pada connect/final membaca Hall `001`.

## Bukti current-zero

Offset hasil run:

- rlA = 2793
- rlB = 2722
- rrB = 2766
- rrC = 2747
- DCL = 1945
- DCR = 1931

Block-mean span hanya sekitar 1–20 count, sehingga zero-offset stabil walaupun raw rlB memiliki spike sesaat.

Idle HBTS setelah kalibrasi:

- LEFT: |Id| <= 0.235 A, |Iq| <= 0.399 A, |Imotor diagnostic| <= 0.54 A, raw DC <= 0.02 A.
- RIGHT: |Id| <= 0.118 A, |Iq| <= 0.041 A, |Imotor diagnostic| <= 0.11 A.

Jadi error ±20…35 A saat detect bukan berasal dari zero-offset startup yang gagal.

## Kenapa VESC Tool V7 menunjukkan current 0 saat steady

V7 mengosongkan accumulator standard VESC current ketika `MotorControl_CurrentMeasurementValid()` false (bridge/MOE off). Akibatnya `Motor Current`, `Input Current`, `Id`, `Iq` menjadi 0 walaupun ADC observation tetap hidup di HBTS.

V8 mengganti policy ini:

- read-reset average tetap digunakan;
- offset/noise kecil saat bridge off tetap boleh masuk standard current telemetry;
- passive-spin excursion besar ditolak dari standard VESC fields tetapi tetap dicatat HBTS;
- jika VESC Tool polling lebih cepat dari sampler, last proven average di-hold; tidak ada fallback ke instantaneous 8-kHz sample.

## LEFT Encoder Detect: bukti utama

Progress V7:

| Detect target | Measured L1 current | Id | Iq | DC raw | DC validated |
|---:|---:|---:|---:|---:|---:|
| 0.574 A | 32.93 A | 9.23 A | 26.66 A | 4.90 A | 3.28 A |
| 0.663 A | 27.25 A | 6.39 A | 21.91 A | 6.58 A | 5.84 A |
| 0.748 A | 28.53 A | 7.71 A | 21.26 A | 5.60 A | 5.13 A |
| 0.836 A | 34.66 A | 12.04 A | 23.83 A | 2.08 A | 1.72 A |
| 0.923 A | 34.89 A | 10.01 A | 19.83 A | 1.54 A | 1.27 A |

Encoder edge counter tetap bertambah (170 → 438 → 675 → 877 → 1084), sehingga A/B memang menghasilkan edge selama commissioning. Detect kemudian abort dan tidak boleh dianggap bukti bahwa encoder decoder rusak.

## Fix V8 untuk commissioning

1. Current PI khusus detect dibatasi konservatif:
   - Kp <= 0.05
   - Ki*dt <= 82/65536 per 8-kHz sample
   - voltage vector commissioning <= 500 internal (~3.5% full-scale)
2. Request detect firmware di-cap maksimal 2 A; tester default dimulai 0.5 A.
3. Fast commissioning guard berjalan di ISR 8 kHz per motor:
   - threshold = max(3 A, 2*target + 1 A)
   - jika dilampaui 8 sample berturut-turut (~1 ms), MOE langsung OFF.
   - calibration berhenti sebagai commissioning failure, bukan hard runtime fault.
4. Fast-current guard evidence dipertahankan sampai detect berikutnya agar post-failure diagnostic tidak kehilangan sebab kegagalan.

## RIGHT Hall: bug tester V7

V7 menerima frame HBTS pertama tanpa memeriksa embedded node ID. Setelah LEFT detect, stale LEFT diagnostic masih bisa tiba ketika tester meminta RIGHT preflight. Log menunjukkan baris berlabel `right before_detect` tetapi payload menyatakan:

- `node_id=10`
- `side=LEFT`
- `hall_raw=000`

Sedangkan RIGHT sebenarnya:

- connect: `node_id=11 side=RIGHT hall_raw=001`
- final: `node_id=11 side=RIGHT hall_raw=001`

V8 mengharuskan `node_id` response cocok dengan requested node dan membuang stale/mismatched HBTS frames. Detect-progress juga memakai aturan yang sama.

## Beep 1x

V7 belum menyimpan sumber buzzer secara eksplisit. V8 menambahkan:

- `buzzer_reason`
- `buzzer_last_reason`
- `buzzer_event_count`

Reason: NONE, HARD_RUNTIME_FAULT, TEMPERATURE_WARNING, BATTERY_LEVEL1, BATTERY_LEVEL2, REVERSE_WARNING. Sensor-not-calibrated/readiness tidak dibuat audible hard-fault beep.

## ISR timing

Sebelum detect: max sekitar 3744/4000 cycle. Selama transisi detect terdapat dua sample >4000 dan recorded max 5956, tetapi overrun fault mask tidak latch. V8 tidak menyembunyikan ini; HBTS tetap merekam last/max/deadline/overrun count. Fast current guard dan commissioning PI cap tidak mengubah normal FOC bandwidth.

## Status verifikasi V8

- Host regression: 12/12 PASS.
- Cortex-M3 current FOC: division helpers = 0.
- Cortex-M3 encoder update: division helpers = 0.
- Cortex-M3 DMA current ISR: division helpers = 0.
- Python HBTS/parser self-test: PASS.
- Actual PlatformIO target build/flash/motor bench V8: belum dijalankan di environment model; harus divalidasi pada board user.
