HOVERBOARD VESC DUAL — V12 DETECT / TELEMETRY / DRV FIX
=======================================================

SUMBER MASALAH YANG TERBUKTI DARI LOG V11 20260812_224946
- Current-domain calibration V11 SUDAH benar: final_active_residual sekitar 0..1 count.
- LEFT encoder Detect timeout setelah 30 s; encoder bergerak/twitch tetapi tidak menyelesaikan commissioning.
- RIGHT Hall Detect timeout; hanya Hall raw 2/3/6 yang terlihat, jadi LUT 6-state tidak pernah valid.
- Saat Detect aktif, current target tetap 0.50 A tetapi Vd hampir selalu mentok pada cap lama 500 internal.
  LEFT mean Vd=493.45/500; RIGHT mean Vd=499.65/500.
- Karena Detect gagal, semua SET non-zero Duty/Current/RPM/POS memang di-SKIP safety gate.
- HBTS V11 menandai standard_current_valid terlalu lambat (menunggu accumulator di-drain GET_VALUES).
- Imotor V11 dipaksa 0 ketika Iq mendekati 0, sehingga D-axis/standstill commissioning tidak terlihat sebagai Motor Current.

FIX UTAMA V12
1. Commissioning current tetap mengikuti request tester default 0.50 A. Tidak dinaikkan otomatis.
2. Voltage headroom commissioning dinaikkan 500 -> 2400 internal agar current PI tidak terus saturasi di cap lama.
3. Hall auto-detect: 3 sweep maju + 3 sweep mundur, 0.1875 electrical deg/ms, sesuai bentuk akuisisi VESC.
4. Encoder VESC detect: 6 electrical revolutions pada rate yang sama; window selesai sekitar 11.52 s + 1.2 s alignment.
5. Telemetry standard VESC valid bila bridge/current-domain valid dan ada pending/held average sample.
6. Imotor mengikuti semantik VESC FOC: magnitude sqrt(Id^2+Iq^2), sign dari Vq*Iq; Id-only tidak lagi dipaksa 0.
7. FAULT_CODE_DRV (3) tidak pernah dibuat dari custom sensor/commissioning/internal fault.
8. Firmware menyimpan provenance packet GET_VALUES/GET_VALUES_SETUP: reply count, command, payload length, exact fault byte terakhir.
9. Tester mem-poll GET_VALUES + GET_VALUES_SETUP selama Detect, sehingga jalur yang dipakai VESC Tool diuji langsung.
10. Debug SET selalu membuat scaling contract Duty/Current/RPM/POS walau commissioning gagal; setelah Detect PASS, trace non-zero membuktikan raw wire -> normalized -> runtime -> ARM/MOE -> standard telemetry.

FILE DEBUG BARU/PENTING
- diagnostics.csv
- telemetry.csv
- raw_packets.log
- vesc_standard_wire.jsonl
- set_command_contract.csv
- set_command_trace.csv
- command_route_evidence.log
- results.json
- summary.txt

RUN PERTAMA DI LINUX
--------------------
1. ANGKAT KEDUA RODA.
2. Gunakan supply/current-limit yang aman.
3. Tutup VESC Tool agar /dev/ttyUSB0 tidak dipakai dua program.
4. Jalankan:

   cd /media/sirobo/Data/BLDC/hoverboard_vescv12
   ./RUN_BUILD_UPLOAD_TEST_LINUX.sh /dev/ttyUSB0

Jika firmware sudah ter-upload dan hanya ingin menjalankan test:

   ./RUN_FULL_TEST_LINUX.sh /dev/ttyUSB0

JANGAN menaikkan --detect-current pada run pertama. Default tetap 0.50 A.

ACCEPTANCE RUN PERTAMA
----------------------
- Current calibration VALID dan final_active_residual tetap kecil.
- Detect LEFT encoder PASS sebelum timeout.
- Detect RIGHT Hall PASS dan observed_hall_mask mencakup state 1..6 (0x7E).
- cal_forward_cycles mencapai 3 dan cal_reverse_cycles mencapai 3 untuk Hall.
- commissioning_vd_saturated tidak terus menerus selama seluruh sweep.
- GET_VALUES/SETUP exact wire fault tidak pernah 3 (DRV).
- Saat bridge aktif: standard_current_valid=true dan Id/Iq/Imotor terlihat valid.
- Setelah Detect PASS: Duty +/-; Current +/-; RPM +/-; POS diuji end-to-end pada LEFT dan RIGHT virtual CAN.
- Peer motor tidak ikut ARM/MOE dan peer Imotor tetap ~0 saat target sisi lain bergerak.

PRE-PACKAGE VALIDATION V12
--------------------------
- Host regression: 15/15 PASS.
- FOC numeric / MTPA / FW / SVM / trig / VESC packet / config / APP ADC / UBSan: PASS.
- V12 commissioning regression contract: PASS.
- Cortex-M3 current loop / encoder / DMA ISR codegen: PASS; tidak ada division helper di hot path.
- Python tester protocol/HBTS self-test: PASS; HBTS diagnostic payload 388 byte.
- PlatformIO CLI dan board/ST-Link tidak tersedia di environment pembuat ZIP, sehingga upload dan gerak motor fisik belum dapat diklaim PASS sebelum test di board Anda.
