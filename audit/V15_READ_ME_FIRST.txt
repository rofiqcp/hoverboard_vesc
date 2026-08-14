HOVERBOARD VESC V15 — READ ME FIRST
===================================

Release ini dibuat dari:
  - log hardware vesc_full_test_20260813_010411.zip
  - source V1 yang dikirim ulang user
  - audit V1 -> V14
  - alur command/telemetry VESC firmware 6.00
  - struktur DMA/current-chop hoverboard-firmware-hack-FOC

ARAH V15
--------
V15 sengaja mengurangi lapisan safety/gating yang menahan PWM tanpa bukti fault
fisik, dan mengembalikan jalur ADC/sensor/FOC ke pola timing V1:

  DMA current sample @ ~16 kHz
    -> current delta LEFT/RIGHT
    -> hard DC-link current chop per sample
    -> snapshot sensor LEFT
    -> FOC LEFT
    -> PWM LEFT
    -> snapshot sensor RIGHT
    -> FOC RIGHT
    -> PWM RIGHT

Kedua motor kembali menjalankan current-loop FOC pada SETIAP DMA ISR (~16 kHz
per motor). Tidak ada lagi interleave 8 kHz/motor.

YANG TIDAK DIKEMBALIKAN DARI V1
-------------------------------
1. Current-offset calibration V1 TIDAK dipakai. V15 mempertahankan kalibrasi
   active LOW-FET zero-vector V11+ karena log hardware membuktikan offset domain
   aktif sudah benar (final residual sekitar 0..1 count).
2. Kesalahan encoder V1 PB5/PB6 TIDAK dipakai. Encoder LEFT tetap PB6=A,
   PB7=B; PB5 adalah index/Z.
3. Overrun ISR V1 tidak lagi mematikan command/PWM secara sticky. Timing tetap
   direkam untuk diagnosis tetapi tidak menjadi permission gate.
4. Hall/encoder calibration V14 yang sudah diperbaiki tetap dipertahankan.

TEMUAN LOG 20260813_010411
--------------------------
- RIGHT Hall firmware sebenarnya SUCCESS:
    Hall raw mask 0x7E (state 1..6 lengkap)
    3 forward + 3 reverse
    terminal_detect_state = SUCCESS
  Tester V14 tetap FAIL karena bug Python: sensor type Hall bernilai 0 tetapi
  dibaca dengan pola `value or -1`, sehingga 0 valid berubah menjadi -1.
  Bug tester ini sudah diperbaiki V15.

- LEFT encoder secara hardware sehat:
    A/B observed mask = 0x0F
    valid TIM4 edges = 94
    invalid transitions = 0
  V14 mereset evidence edge/mask pada setiap failed sweep window. V15
  mengakumulasi proof A/B sepanjang SATU transaksi Detect sehingga loaded-wheel
  dither tidak menghapus bukti encoder sehat.

- Current calibration tetap VALID dan active-domain residual sangat kecil.
- ISR log V14: last=3200 cycles, max=5709, nominal deadline=4000. Karena user
  meminta pola V1, V15 menjalankan kedua FOC setiap 16 kHz tetapi deadline
  hanya diagnostic. Hardware run V15 tetap wajib memverifikasi actual timing.

VESC SET/GET FLOW V15
---------------------
Empat command utama tidak lagi memakai threshold facade untuk menentukan apakah
ARM boleh diminta:

  COMM_SET_DUTY    raw / 100000 -> DUTY
  COMM_SET_CURRENT raw / 1000   -> CURRENT
  COMM_SET_RPM     raw integer  -> SPEED
  COMM_SET_POS     raw / 1000000-> POSITION

Setiap packet valid langsung:
  decode -> route LEFT/virtual-RIGHT -> RuntimeControl_VescSetOne(..., true)
         -> RuntimeControl_VescAlive()

Permission PWM tidak bergantung pada debug/telemetry/ISR-overrun/SensorHealth
stale window. Preconditions yang tetap ada hanya yang memang diperlukan:
  - current offsets harus valid;
  - commissioning/homing/alignment tidak sedang memiliki bridge target;
  - sensor target sudah dikalibrasi untuk closed-loop;
  - encoder incremental telah alignment electrical-zero;
  - feedback sample saat itu valid;
  - setpoint berada dalam range konfigurasi;
  - DC-link over-current tetap di-chop langsung per sample;
  - timeout komunikasi dan kehilangan total heartbeat ISR tetap fail-safe.

GET_VALUES mengikuti urutan/scaling VESC 6.00 dan menggunakan read-reset
average untuk Motor Current / Input Current / Id / Iq. Averaging dikumpulkan
di slow loop supaya ISR 16-kHz tetap ringan. Bridge OFF menghasilkan standard
0 A yang valid; raw evidence tetap tersedia di HBTS.

KALIBRASI
---------
- Current offset: active LOW-FET zero-vector, robust 2048 sample.
- RIGHT Hall: circular angle averaging, 3 forward + 3 reverse.
- LEFT Encoder: PB6/PB7; strict ratio inference lebih dulu; guarded configured
  pole-pair fallback hanya jika cumulative quadrature proof bersih.
- Encoder first ARM alignment: regulated D-axis current alignment.

FIRST HARDWARE RUN
------------------
1. Angkat KEDUA roda.
2. Gunakan PSU/current limit.
3. Extract ZIP langsung ke:
     /media/sirobo/Data/BLDC/hoverboard_vescv15
4. Jalankan:
     cd /media/sirobo/Data/BLDC/hoverboard_vescv15
     ./RUN_BUILD_UPLOAD_TEST_LINUX.sh /dev/ttyUSB0
5. Tetap gunakan Detect Current 0.50 A untuk baseline pertama.
6. Jangan buka VESC Tool dan Python tester pada UART yang sama bersamaan.

Yang harus dilihat pada log pertama V15:
- LEFT encoder Detect terminal SUCCESS;
- RIGHT Hall Detect terminal SUCCESS (tester tidak lagi salah membaca type 0);
- empat SET non-zero tidak lagi SKIP;
- arm_reject, MOE, Id/Iq, Imotor/Input Current, duty, ERPM dan position pada
  set_command_trace.csv;
- isr_last_cycles/isr_max_cycles untuk memastikan 16-kHz dual-loop muat pada
  hardware STM32F103RCT6.
