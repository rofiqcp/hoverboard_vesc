HOVERBOARD VESC V16 — READ ME FIRST
===================================

ROOT CAUSE V15 NO-CONNECT
-------------------------
V15 mengembalikan LEFT + RIGHT FOC pada setiap ADC DMA ISR ~16 kHz, tetapi
menghapus one-shot overload relief yang ada pada baseline V1. Log V14 sudah
pernah mengukur max ISR 5709 cycle, sedangkan pada 64 MHz satu periode 16 kHz
hanya sekitar 4000 cycle. Jika DMA IRQ terus pending sebelum ISR selesai, main
loop bisa kelaparan dan VescProtocol_Service() tidak pernah memparse RX DMA atau
mengirim FW_VERSION/GET reply. Gejalanya persis: serial port terbuka tetapi
VESC Tool dan tester timeout saat connection inventory.

FIX V16
-------
- Kembalikan V1-style one-shot overload relief (`motorIsrShedNext`).
- Detector memakai flag DMA TC yang sudah pending lagi di akhir ISR.
- HANYA ISR berikutnya dibuat pendek.
- Hard DC-link current chop tetap dijalankan sebelum early return.
- Tidak clear runtimeMotorEnableMask.
- Tidak clear sensorCalibrationOpenLoopMask.
- Tidak clear ARM/requested SET.
- Tidak membuat fault overrun sticky.
- Full ISR berikutnya kembali membaca sensor LEFT/RIGHT dan menjalankan kedua FOC.

YANG TETAP DARI V15
-------------------
- VESC SET/GET wire flow dan scaling.
- Active LOW-FET current calibration V11+.
- PB6/PB7 encoder A/B fix.
- Hall circular calibration dan durable Detect reply.
- Encoder cumulative evidence.
- Minimal PWM safety: sample-local DC current chop + same-sample feedback.

TEST PERTAMA
------------
Setelah upload V16, uji koneksi TERLEBIH DAHULU sebelum --full:

  python3 tools/vesc_full_test.py --port /dev/ttyUSB0

Jika inventory PASS, baru:

  python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes

Firmware FW string: hoverboard-vesc6-v16-v1isr-liveness
Tester release: V16
