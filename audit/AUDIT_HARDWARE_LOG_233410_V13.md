# Audit Hardware Log 2026-08-12 23:34:10 — V13

## Scope

Audit ini memakai `vesc_full_test_20260812_233410.zip` sebagai ground truth hardware dan membandingkan jalur kritis source V1 sampai V12 sebelum membuat V13.

## Kesimpulan utama

V12 tidak gagal karena satu akar masalah tunggal. Current-domain fix V11/V12 sudah bekerja, RIGHT Hall acquisition juga sudah bekerja, tetapi hasilnya tertutup oleh beberapa bug di lapisan lain.

### 1. Current offset bukan lagi penyebab Detect gagal

Current calibration terakhir valid dalam `ACTIVE_LOW_FET_ZERO_VECTOR`. Pada akhir calibration active-domain, residual hanya kecil dibanding kegagalan V10 yang ratusan ADC count. Ini berarti pergeseran common-mode OFF->ON yang menyebabkan false 14-15 A pada V10 sudah tidak menjadi root cause V12.

### 2. RIGHT Hall selesai secara internal tetapi reply hilang

Diagnostic RIGHT menunjukkan:

- `cal_state = SUCCESS` pada `2026-08-12T23:35:06.833+07:00`;
- `cal_forward_cycles = 3`;
- `cal_reverse_cycles = 3`;
- `cal_completed_cycles = 6`;
- `cal_observed_hall_mask = 126 (0x7E)` = Hall state 1..6 semuanya terobservasi;
- motion counter = 35.

Namun Python berakhir dengan `right hall detect response timeout after 35.0s`.

Pada saat sukses, `uart_tx_drops` sudah 20 dan terus naik sampai 32. Source V12 memakai TX ring depth 3, `send_payload()` dapat gagal saat queue full, sedangkan `detect_service()` tetap menghapus `pending_detect` setelah mencoba mengirim. Raw packet log juga tidak memiliki terminal reply `COMM_DETECT_HALL_FOC (0x1C)` setelah status internal menjadi SUCCESS.

**Fix V13:** TX ring diperbesar, terminal Detect memakai lossless enqueue/retry, pending hanya dihapus setelah frame diterima ring, dan Detect dilayani sebelum RX polling baru mengisi queue lagi.

### 3. LEFT encoder physical observer memakai pin salah sejak V1-V12

Hardware encoder LEFT sebenarnya:

- PB6 = TIM4_CH1 = A;
- PB7 = TIM4_CH2 = B;
- PB5 = optional Z/index.

TIM4 pada run terbaru membuktikan encoder hidup: 145 valid edge dan 0 invalid transition. Tetapi commissioning observer V12 dipanggil dengan `left_u, left_v`, yaitu PB5(Z) + PB6(A), sehingga observed A/B mask hanya 0x03.

**Fix V13:** commissioning observer sekarang memakai PB6/PB7 eksplisit. PB5 tidak lagi dapat dipakai sebagai kanal B.

### 4. Encoder fail bukan karena current PI tidak bekerja

Selama LEFT Detect:

- target D-axis mencapai 0.50 A;
- measured Id mencapai sekitar rentang yang relevan, peak log ~0.776 A;
- Vd maksimum sekitar 532 internal, jauh di bawah commissioning ceiling 2400;
- bridge/current-domain aktif;
- TIM4 menangkap edge valid.

Rotor hanya bergerak/dither sekitar posisi -24..-2 tick dan tidak mengikuti beberapa electrical revolution penuh. Karena itu metode ratio V1-V12 yang mengandalkan net mechanical displacement penuh tidak selalu dapat mengestimasi pole-pair pada roda yang terbebani/stiction tinggi.

**Fix V13:** inferensi ratio ketat tetap pilihan pertama. Fallback ke configured pole-pair hanya diizinkan jika quadrature PB6/PB7 melihat semua 4 state, TIM4 memiliki >=8 valid edge, invalid transitions rendah, dan ada raw signed displacement minimum. Fallback dicatat di terminal snapshot/HBTS, bukan disamarkan sebagai auto-estimate.

### 5. State commissioning terminal masih global/shared

V1-V12 menyimpan runtime commissioning pada satu `sensorCal`. Setelah RIGHT Hall sukses, diagnostic LEFT dapat memantulkan progress/status RIGHT. Ini tidak aman untuk transaksi virtual-CAN dua motor.

**Fix V13:** hasil terminal di-copy ke `VescDetectTerminalSnapshot[2]`. Poll dan HBTS terminal evidence memilih snapshot berdasarkan motor target.

### 6. False-DRV test V12 gagal karena tester, bukan fault byte firmware

`exceptions.log` membuktikan `04a_no_false_drv_mapping` crash di Python:

`setup_values(node, "false_drv_setup")` membuat string masuk sebagai `mask`, lalu operasi `mask & 0xFFFFFFFF` menghasilkan TypeError.

Audit `vesc_standard_wire.jsonl` menghasilkan 437 standard replies:

- local GET_VALUES: 187;
- right GET_VALUES: 206;
- local/right GET_VALUES_SETUP: 20 + 20;
- selective values/setup: masing-masing 1 per side.

Semua packet yang mempunyai fault field mengirim `fault=0`. **Tidak ada satu pun fault code 3/DRV di standard wire capture tersebut.**

**Fix V13:** `setup_values()` keyword-only, seluruh call-site diperbaiki, dan regression memastikan tidak ada positional call lama. Firmware tetap tidak memalsukan internal commissioning/sensor fault sebagai DRV.

### 7. Alignment encoder setelah Detect juga perlu diperbaiki

V1-V12 melakukan first-ARM encoder electrical-zero alignment dengan open-loop voltage 30..90 internal. Hardware run menunjukkan current-controlled Detect membutuhkan Vd sekitar 500 internal hanya untuk mempertahankan ~0.5 A. Artinya Detect dapat dibuat PASS tetapi first ARM masih berisiko gagal karena alignment voltage terlalu kecil.

**Fix V13:** first-ARM alignment memakai current-regulated D-axis soft ramp 0.25 -> 0.50 A selama 300 ms, lock sampai 800 ms, dengan current-domain warmup dan fast commissioning current guard yang sama.

## Tester/debug V13

Ditambahkan `detect_transaction_trace.csv` yang merekam per sample:

- calibration state/sensor/motor;
- sweep direction dan forward/reverse/completed cycles;
- Hall/encoder observed masks, TIM4 valid/invalid edges, encoder delta;
- target/measured current, Id/Iq targets, Id/Iq, Vd/Vq dan saturation;
- MOE/warmup/current validity;
- UART drops, TX queue use, pending Detect owner dan reply retry;
- per-motor terminal snapshot;
- exact last standard-wire fault provenance.

Polling commissioning tester di-stagger supaya HBTS, GET_VALUES dan GET_VALUES_SETUP tidak dikirim sebagai burst yang mengalahkan throughput TX DMA.

## Host verification V13

- strict syntax integration: PASS;
- FOC/sensor behavioral: PASS;
- MTPA/FW numeric: PASS;
- SVM compare: PASS;
- trig LUT: PASS;
- VESC framing/CRC/parser: PASS;
- VESC config wire: PASS;
- APP ADC: PASS;
- resource/pin/ADC contract: PASS;
- current calibration robustness: PASS;
- UBSan stress: PASS;
- tester inventory: PASS;
- command coverage: PASS;
- V12 commissioning regression: PASS;
- V13 hardware-log regression: PASS;
- total: 16/16 stages PASS.

Cortex-M3 codegen audit:

- FAST_CURRENT_LOOP_DIV_HELPERS=0;
- FAST_ENCODER_UPDATE_DIV_HELPERS=0;
- FAST_DMA_CURRENT_ISR_DIV_HELPERS=0;
- CORTEX_M3_CORE_OBJECT_PASS.

Python protocol/HBTS self-test:

- parser lengths 1/99/300: PASS;
- CRC16 `123456789` = 0x31C3;
- HBTS v12 synthetic payload = 401 bytes, decode PASS.

## Batas klaim

Host regression dapat membuktikan source contract, protocol layout, numeric behavior, pin mapping source, queue semantics dan tester logic. Hanya board fisik yang dapat membuktikan motor benar-benar menggerakkan beban, arah encoder benar pada mekanik final, RIGHT Hall terminal reply diterima di UART nyata, serta empat SET non-zero mencapai PWM dengan motor fisik.
