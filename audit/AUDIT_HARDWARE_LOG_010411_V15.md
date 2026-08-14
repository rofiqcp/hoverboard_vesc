# Audit hardware log 2026-08-13 01:04:11 -> V15

Sumber: `vesc_full_test_20260813_010411.zip`.

## Ringkasan hasil V14

Tester: **18 PASS / 3 FAIL / 22 SKIP**.

### LEFT encoder

- Detect timeout setelah ~30.2 s.
- Sensor type: Encoder A/B.
- Observed A/B mask: `0x0F` (semua empat quadrature state).
- TIM4 valid edges: **94**.
- Invalid transitions: **0**.
- Motion counter: **93**.
- `cal_motion_detected = true`.
- Configured encoder CPR: 2048.
- Configured motor pole pairs: 15.

Interpretasi: encoder hardware dan TIM4 tidak menunjukkan gejala sensor mati. Failure berasal dari transaksi commissioning/finalization logic yang membuang proof per sweep window ketika rotor/load hanya dither dan net displacement kembali kecil.

### RIGHT Hall

Firmware diagnostic setelah Detect menunjukkan:

- `cal_state = SUCCESS`;
- 3 forward + 3 reverse;
- Hall observed mask `126 = 0x7E`;
- terminal Detect valid dan state SUCCESS;
- terminal sensor type `0` = Hall;
- UART TX drops = 0.

Tetapi Python test 12 masih FAIL tanpa pesan. Audit tester menemukan assert menggunakan idiom `terminal_detect_sensor_type or -1`. Karena Hall bernilai numerik **0**, value valid itu berubah menjadi `-1`; firmware success salah dilaporkan fail. V15 memperbaiki test menjadi `is not None && int(value) == 0`.

### Current ADC / calibration

Current calibration tetap VALID:

- sample mode `ACTIVE_LOW_FET_ZERO_VECTOR`;
- 2048 sample;
- final active residual kira-kira `[0, 0, -1, 0, -1, 0]` count.

Ini membuktikan alasan untuk TIDAK mengembalikan offset learner V1. V15 hanya mengembalikan struktur timing ISR V1.

### ISR timing V14

Pada terminal LEFT setelah Detect:

- `isr_last_cycles = 3200`;
- `isr_max_cycles = 5709`;
- nominal deadline = 4000;
- overrun count = 4;
- overrun fault mask = 0.

V15 menjalankan LEFT+RIGHT current loop setiap DMA ISR sesuai permintaan user. Karena ini meningkatkan frekuensi per-motor dari 8 kHz menjadi 16 kHz, hardware V15 wajib mengukur ulang cycle timing. Deadline tidak lagi dipakai sebagai PWM permission gate.

## Perubahan langsung dari log ini

1. Encoder quadrature proof diakumulasi sepanjang transaction, bukan dihapus per failed sweep.
2. Hall type zero tester bug diperbaiki.
3. Kedua FOC dipanggil setiap DMA ISR 16 kHz.
4. ISR dibuat lebih linear: current -> gate current -> sensor LEFT -> FOC LEFT -> sensor RIGHT -> FOC RIGHT -> telemetry publish.
5. Overrun/SensorHealth/commissioning software streak tidak boleh menghapus command host.
6. Active-domain current calibration dan V14 Hall/encoder calibration logic dipertahankan.
