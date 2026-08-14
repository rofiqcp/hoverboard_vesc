# Audit Final — VESC-Style Outer PID + MTPA + Flux Weakening

Tanggal audit: **2026-08-12**  
Target: STM32F103 dual motor, sensored FOC Hall UVW / incremental Encoder A/B  
Core current loop: ~16 kHz DMA/ADC ISR  
Outer runtime scheduler: ~200 Hz main/runtime loop dengan `dt_ms` aktual  
Numerik: fixed-point/integer pada hot path

## 1. Scope dan sumber acuan

Audit ini membandingkan implementasi terhadap VESC upstream `vedderb/bldc` master yang tersedia pada tanggal audit, khususnya:

- `motor/foc_math.c`
  - `foc_run_pid_control_speed()`
  - `foc_run_pid_control_pos()`
  - `foc_run_fw()`
  - `foc_svm()`
- `motor/mcpwm_foc.c`
  - MTPA sebelum current control
  - kombinasi MTPA + Field Weakening
  - current-vector limiting
  - fast-loop FOC/current controller
- `datatypes.h`
  - `MTPA_MODE`
  - speed/position/FW configuration fields
  - configurable PID loop-rate model VESC
- `CHANGELOG.md` VESC 7.00 (2026-05-15)
  - FW dipindah ke fast loop
  - MTPA + FW memakai kebutuhan current D terbesar, bukan dijumlahkan
  - FW backoff parameter
  - AB encoder mode tanpa index

Implementasi ini **bukan copy penuh seluruh firmware VESC**. Observer sensorless, HFI, phase-filter, thermal/input-current override map, model decoupling BEMF/cross-coupling, overmodulation, dan hardware VESC yang tidak ada pada board ini tidak ditambahkan secara palsu sebagai dead code.

## 2. Arsitektur akhir

```text
Host / GUI
  │
  ├─ OPEN ──────────────── signed Iq + forced electrical phase
  ├─ VLT  ──────────────── Vq request
  ├─ TRQ  ──────────────── Iq request
  ├─ SPD  ─ Speed PID ──── Iq request
  └─ POS  ─ Position PID ─ Iq request
                              │
                              ▼
                    DMA current ISR 16 kHz
                              │
                  Hall / Encoder backend
                              │
                  electrical phase valid?
                              │
                 Clarke -> Park -> Id/Iq
                              │
                    requested Id / Iq
                              │
                       optional MTPA
                              │
                    optional fast-loop FW
                              │
                    current-vector circle
                              │
                       Id/Iq shared PI
                              │
                    voltage vector limit
                              │
                 inverse Park -> SVPWM
                              │
                         PWM + MOE
```

Outer PID tidak diletakkan di DMA ISR. ISR menangani inner current loop dan FW yang memang dipindahkan VESC 7 ke fast loop.

## 3. Speed PID

### 3.1 Semantik

Boundary host menerima mechanical RPM. Firmware mengubahnya satu kali menjadi electrical RPM:

```text
eRPM_set = mechanical_RPM_set × pole_pairs
```

Feedback speed berasal dari backend sensor yang sama dengan FOC, kemudian juga diubah menjadi eRPM. Ini menghindari GUI/controller memakai definisi speed yang berbeda.

### 3.2 Struktur VESC yang diterapkan

- ramp `s_pid_ramp_erpms_s` menuju command;
- error eRPM;
- P term dengan faktor `1/20`;
- D-on-error dengan `dt` aktual dan D low-pass filter;
- output sementara dibatasi ±1;
- I term diperbarui **setelah** output sementara, seperti VESC;
- integrator dibatasi ±1;
- integrator dipaksa 0 jika Ki=0;
- `s_pid_min_erpm` me-release target Iq ke 0;
- optional no-braking berdasarkan tanda speed/output;
- output normalized dikalikan motor current limit menjadi `m_iq_set`.

### 3.3 Adaptasi board

VESC menyediakan pilihan source PLL/FAST/FASTER dan configurable `PID_RATE`. Board ini mempunyai satu sensor backend authoritative dan runtime scheduler existing ~200 Hz. Karena itu source speed VESC tidak diduplikasi dan rate belum dibuat sebagai enum VESC; `dt_ms` aktual dipakai pada semua I/D/ramp calculations.

## 4. Position PID

### 4.1 Struktur VESC yang diterapkan

- P term;
- I term dengan `dt`;
- derivative-on-error;
- accumulated derivative `dt` jika encoder quantized tidak berubah;
- `p_pid_kd_proc` derivative-on-process dengan tanda negatif;
- low-pass filter untuk kedua D paths;
- gain-decrease dekat target;
- P-based integral windup clamp;
- output normalized ±1 -> Iq request.

### 4.2 Adaptasi yang disengaja

VESC position default memakai circular angle difference. Project ini memakai signed multi-turn `position_ticks`, homing, soft min/max, dan mechanical zero. Mengubahnya menjadi circular 0..360° akan merusak kebutuhan aktuator/AGV. Karena itu struktur PID VESC dipertahankan tetapi domain error adalah signed linear ticks.

## 5. MTPA

MTPA mengikuti persamaan referensi VESC:

```text
Id = (lambda - sqrt(lambda² + 8 × ((Ld-Lq) × Iq_ref)²)) / (4 × (Ld-Lq))
```

Untuk fixed-point, firmware menyimpan:

- `lambda` dalam µWb;
- `Ld-Lq` dalam µH signed;
- current engineering scale `current_units_per_amp`;
- ratio `lambda/(Ld-Lq)` diprecompute ketika config berubah.

Hot loop hanya melakukan integer multiply, integer square-root dan shifts. Test sweep membandingkan hasil fixed-point terhadap persamaan floating reference untuk berbagai `Iq`, positive/negative saliency, dan mode.

### MTPA mode

- `OFF`
- `IQ_TARGET`
- `IQ_MEASURED`: `Iq_ref` memakai minimum absolute target vs filtered measured Iq, sesuai VESC.

Setelah MTPA membentuk `Id`, magnitude Iq disesuaikan supaya current request asli tidak bertambah secara artifisial.

## 6. Flux Weakening

FW dijalankan dari current fast loop setiap sample saat fitur aktif.

### 6.1 Input FW

- `duty_abs_filtered_q15` — low-pass duty magnitude;
- `foc_fw_duty_start_q15`;
- `foc_fw_current_max`;
- `foc_fw_ramp_time_ms`;
- `foc_fw_backoff_q15`;
- `foc_fw_q_current_factor_q15`.

### 6.2 Duty mapping

Di atas duty-start, FW current dipetakan linier dari 0 menuju maximum FW current. Bila duty turun atau mode berubah, request diramp kembali menuju 0, bukan diputus mendadak.

### 6.3 Iq-error backoff

Backoff mengurangi maximum FW ketika measured Iq berada di sisi yang menunjukkan Q-axis kehilangan voltage headroom. Sign speed dipakai seperti referensi VESC.

### 6.4 MTPA + FW VESC 7

Implementasi lama VESC pernah menjumlahkan D-current. VESC 7.00 mengubahnya. Port ini menggunakan:

```text
Id_final = current dengan |Id| terbesar dari Id_MTPA / Id_FW / explicit Id
```

bukan `Id_MTPA + Id_FW`.

Q-current factor juga mengurangi Iq berdasarkan tanda filtered `mod_q` dan FW current.

### 6.5 Current-vector limit

Setelah D-current final ditentukan:

```text
|Iq| <= sqrt(Imax² - Id²)
```

sehingga selalu:

```text
Id² + Iq² <= Imax²
```

## 7. Mode behavior

| Mode board | Sensor phase | Outer PID | MTPA | FW start | Current PI | SVPWM |
|---|---|---|---|---|---|---|
| OPEN | forced | no | yes, bila enabled | no | yes | yes |
| VLT | sensor/forced sesuai control | no | no | no | bypass Vq command | yes |
| TRQ | Hall/Encoder | no | yes | yes | yes | yes |
| SPD | Hall/Encoder | speed | yes | yes | yes | yes |
| POS | Hall/Encoder | position | yes | tidak memulai FW | yes | yes |
| Calibration alignment | forced | no | bypass | bypass | voltage override | yes |

Jika FW sudah aktif lalu mode berubah, FW boleh ramp-out agar Id tidak meloncat.

## 8. Sensor → control → telemetry audit

Single source of truth adalah `MotorRuntimeConfig.sensor_type`.

### Hall

```text
raw U/V/W -> calibrated 6-state LUT -> phase / RPM / position / valid
```

### Encoder

```text
raw A/B -> calibrated quadrature sequence -> count
      -> exact electrical accumulator
      -> electrical zero alignment
      -> electrical trim
      -> phase / RPM / position / valid
```

Satu `MotorSensorSample` digunakan oleh:

- current FOC: `electrical_phase_q16`;
- speed controller: `mechanical_speed_q4`;
- position controller: `position_ticks`;
- odometry;
- telemetry;
- hardware closed-loop phase safety.

### Defense-in-depth yang ditambah pada audit ini

Encoder incremental sekarang `feedback_valid=1` hanya jika:

```text
CPR valid
AND encoder commissioned/calibrated
AND encoder sequence valid
AND fresh electrical alignment complete
```

Raw observation masih berjalan sebelum alignment supaya commissioning/telemetry tidak buta, tetapi closed-loop MOE tetap diblokir.

## 9. Encoder bandwidth dan FW

Encoder A/B board ini masih dipolling satu kali per ISR ~16 kHz. Edge rate:

```text
f_edge = CPR × RPM / 60
```

Karena decoder perlu melihat sequence state dengan margin, `f_edge` tidak boleh mendekati sampling 16 kHz. FW bisa menaikkan speed di atas base speed sehingga risiko alias/missed edge bertambah. Ini adalah batas hardware/software architecture yang **tidak dapat dihilangkan dengan rumus MTPA/FW**. Untuk speed tinggi, gunakan hardware encoder timer/EXTI yang sesuai atau batasi RPM.

## 10. GUI audit

Panel GUI ditambah/dirapikan:

### Shared current PI

VESC memakai satu `foc_current_kp/ki` untuk Id dan Iq. GUI sebelumnya masih dapat terlihat seperti dua controller terpisah. Sekarang panel Id dan Iq diberi label shared PI dan editing Kp/Ki satu sisi langsung mirror ke sisi lain. Current Kd disable.

### Hard limits

GUI hard-limit disamakan dengan firmware:

- current Kp <= 1.0;
- current Ki <= 16000 /s;
- current Kd = 0;
- speed/position gains <= 16.0.

Ini mencegah user memasukkan nilai yang firmware pasti clamp diam-diam.

### Advanced FOC panel

- MTPA mode;
- flux linkage λ;
- signed Ld-Lq;
- max FW current;
- duty start;
- ramp time;
- Q-current factor;
- Iq backoff;
- runtime FW current dan MTPA Id;
- speed D filter/braking/min eRPM/ramp;
- position Kd-process/D filter/gain-decrease;
- Apply RAM / Apply + Save EEPROM / Read;
- read-back match verification per motor.

GUI visual rendering belum dapat dijalankan di audit container karena tidak ada PyQt5/PySide6 runtime. Python syntax dan protocol logic diuji.

## 11. EEPROM/protocol audit

EEPROM config dinaikkan ke **v15**, total 120 virtual words. Advanced VESC config disimpan per motor. Migrasi v5..v14 menerima image yang CRC-valid tetapi tidak memakai tuning lama yang semantiknya berubah secara buta.

Advanced apply:

1. parse dan validate seluruh request ke candidate config;
2. DISARM hanya motor target;
3. commit candidate + `mc_foc_init()` di critical section;
4. motor sebelah tidak di-reset;
5. optional Save EEPROM;
6. GUI meminta read-back dan membandingkan nilai actual vs expected.

Ini memperbaiki risiko config setengah-terapply atau state motor lain ikut reset.

## 12. ISR / Cortex-M3 codegen audit

FW versi awal pada audit ini benar secara numerik tetapi menggunakan pembagian 64-bit di hot loop. Pada Cortex-M3 itu menghasilkan runtime helper `__aeabi_ldivmod/__aeabi_uldivmod` dan dapat menghabiskan budget ISR.

Perhitungan FW diperketat berdasarkan batas matematis:

- `|Iq error| <= 65535`;
- Q15 <= 32767;
- product max = 2,147,385,345 < INT32_MAX;
- duty/current mapping product <= 32767²;
- ramp numerator/current sample count juga muat uint32.

Karena itu pembagian hot-loop FW diubah menjadi exact 32-bit signed/unsigned divide. Cortex-M3 Clang cross-object setelah patch menunjukkan **tidak ada 64-bit divide relocation di range `mc_foc_run_current_control()`**. 64-bit divide helpers yang masih ada berasal dari config-preparation/outer PID, bukan current ISR.

## 13. Bug yang ditemukan dan diperbaiki pada audit ini

1. **Speed PID semantik gain/default** — dikoreksi ke struktur eRPM + `/20` VESC.
2. **Position PID process-D overflow** — sum derivative diperlebar ke int64 sebelum scale/saturation.
3. **FW target ordering** — backoff sekarang melihat previous final `iq_target` seperti ordering VESC.
4. **MTPA OPEN behavior** — current-command OPEN boleh MTPA seperti VESC; calibration voltage override tetap bypass.
5. **MTPA+FW combination** — VESC 7 max-absolute, bukan sum.
6. **Cross-motor config reset** — update advanced satu motor tidak me-reset motor lain.
7. **Config/ISR race** — commit config + FOC re-init dilakukan critical-section atomik.
8. **GUI Q16.16 semantics** — gain kecil tidak lagi diperlakukan integer mentah.
9. **GUI shared Id/Iq PI** — kedua panel disinkronkan; current Kd tidak dapat diedit.
10. **Advanced read-back** — rejected/clamped config tidak lagi terlihat sebagai Apply sukses.
11. **Encoder alignment safety** — commissioned encoder tetapi belum fresh-aligned tetap `feedback_valid=0`.
12. **Cortex-M3 FW 64-bit divisions** — dihapus dari fast current loop tanpa mengubah hasil numerik.
13. **Dokumentasi EEPROM lama** — diperbarui dari v12/78 words menjadi v15/120 words.
14. **Mechanical vs electrical zero wording** — dikoreksi: mechanical homing tidak memindahkan electrical reference.

## 14. Test results

Latest host regression:

```text
[1/7] strict syntax: FOC + sensor + ISR + runtime integration
[2/7] sensor/FOC behavioral tests
ALL_FOC_SENSOR_TESTS_PASS
[3/7] VESC outer PID + MTPA + FW numeric/behavior tests
MTPA_SWEEP cases=5952 max_id_error=0_internal | FW_MAP cases=290 max_error=0
VESC_ADVANCED_CONTROL_TESTS_PASS
[4/7] SVM numeric comparison against VESC foc_svm equations
SVM_VESC_COMPARE tested=176866 max_compare_error=1 mean_abs_error=0.032113 sector_mismatch=7
[5/7] Q14 trig LUT accuracy
TRIG_Q14_ACCURACY max_error=2_lsb mean_abs_error=0.553955 worst_phase=5978
[6/7] advanced protocol Q16/CRC/read-back roundtrip
PROTOCOL_ADVANCED_ROUNDTRIP_PASS
[7/7] UBSan deterministic stress incl. MTPA/FW
FOC_UBSAN_FUZZ_PASS
ALL_HOST_TESTS_PASS
```

Tambahan:

```text
CLANG_CORE_STRICT_PASS
GCC_ANALYZER_CORE_PASS
ASAN+UBSAN stress PASS
CORTEX_M3_CORE_OBJECT_PASS
FAST_CURRENT_LOOP_64BIT_DIV_HELPERS = 0
```

## 15. Hal yang sengaja tidak diklaim

Tidak ada software motor yang dapat secara jujur dinyatakan “pasti tanpa bug” hanya dari host test. Yang dapat dinyatakan adalah semua jalur yang diuji di atas lulus setelah bug yang ditemukan diperbaiki.

Environment audit ini **tidak memiliki**:

- `arm-none-eabi-gcc`;
- PlatformIO;
- board STM32F103 target;
- inverter/motor fisik;
- PyQt5/PySide6 runtime untuk visual GUI.

Jadi belum ada klaim:

- full embedded link sukses dengan toolchain project;
- ISR worst-case cycle count pada MCU asli;
- current ADC scaling benar terhadap Ampere fisik;
- motor phase order benar di hardware;
- PID/MTPA/FW tuning cocok untuk motor tertentu;
- high-speed encoder polling cukup;
- flux weakening aman terhadap rotor mechanical speed / battery / MOSFET / motor voltage.

## 16. Deviations dari full VESC yang relevan

- Tidak ada observer sensorless/HFI pada core ini.
- Tidak ada VESC selectable PLL/FAST/FASTER speed source; sensor backend project adalah authoritative source.
- PID rate VESC tidak dibuat configurable; board scheduler ~200 Hz dipertahankan dan memakai actual dt.
- Tidak ada VESC dynamic input-current/thermal override map; board memiliki DC-current hardware chop/safety yang berbeda.
- Tidak ada model current-controller BEMF/cross-coupling decoupling karena R/L motor belum menjadi parameter validated pada project ini.
- Position domain adalah signed multi-turn ticks, bukan circular angle.

Deviasi ini harus dianggap **adaptasi hardware**, bukan bug tersembunyi atau fitur VESC yang diklaim ada padahal tidak diimplementasikan.

## 17. Bench validation wajib

Urutan yang disarankan:

1. motor tanpa beban / roda terangkat;
2. MTPA OFF, FW current = 0;
3. validasi current ADC polarity/scaling;
4. validasi Hall/encoder direction;
5. Encoder: jalankan fresh alignment dan cek `feedback_valid`;
6. current PI Id/Iq dengan target kecil;
7. speed PID pada base-speed rendah;
8. position PID;
9. identifikasi/masukkan λ dan Ld-Lq;
10. aktifkan MTPA dan bandingkan Id target/measured serta current magnitude;
11. cek encoder edge-rate margin;
12. FW mulai dari current kecil, duty-start konservatif;
13. pantau DC current, Id/Iq, duty, RPM, temperature dan ISR overrun counter;
14. naikkan speed/FW bertahap saja jika semua safety margin terbukti.

