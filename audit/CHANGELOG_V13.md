# CHANGELOG V13

## Hardware-rooted fixes

1. Fix LEFT encoder commissioning A/B dari PB5/PB6 menjadi PB6/PB7; PB5 tetap Z/index.
2. Tambah per-motor terminal Detect snapshot agar hasil LEFT/RIGHT tidak saling overwrite.
3. Detect terminal reply sekarang durable terhadap TX queue full; pending transaction baru selesai setelah enqueue berhasil.
4. TX queue depth dinaikkan 3 -> 6 dan Detect service diprioritaskan sebelum RX polling baru.
5. Immediate Detect failure juga dikirim sebagai durable asynchronous terminal reply.
6. Encoder ratio strict inference dipertahankan; guarded configured-pole-pair fallback ditambahkan untuk loaded/stiction case dengan bukti quadrature/TIM4 ketat.
7. First-ARM encoder alignment diganti dari low open-loop voltage menjadi current-controlled D-axis soft ramp 0.25->0.50 A.
8. Python `setup_values()` dibuat keyword-only; false-DRV positional string bug dan seluruh call-site lama diperbaiki.
9. Detect polling tester di-throttle/stagger untuk mengurangi TX DMA starvation.
10. Tambah `detect_transaction_trace.csv`.
11. HBTS naik ke diagnostic version 12 / synthetic payload 401 byte, dengan queue/pending/retry/per-motor-terminal evidence.
12. Firmware release string menjadi `hoverboard-vesc6-v13-hwlogfix`.

## Retained fixes

V13 sengaja mempertahankan fix V11/V12 yang sudah dibuktikan hardware:

- active LOW-FET zero-vector current calibration;
- bridge OFF->ON ADC warmup;
- standard current validity/accumulator;
- Imotor magnitude memakai Id dan Iq;
- Hall 3 forward + 3 reverse sweep;
- 2400 internal commissioning voltage headroom;
- standard VESC SET scaling/routing;
- no false internal->DRV mapping;
- fixed-point Cortex-M3 hot paths.
