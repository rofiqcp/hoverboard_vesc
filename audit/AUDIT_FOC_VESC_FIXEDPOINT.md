> **SUPERSEDED / RIWAYAT AUDIT AWAL**  
> Dokumen ini menjelaskan rewrite sebelum MTPA dan Flux Weakening ditambahkan. Audit terbaru dan authoritative ada di `AUDIT_VESC_MTPA_FW_FINAL.md`.

# Audit Final — VESC-Style Fixed-Point FOC STM32F103

## 1. Ruang lingkup

Rewrite ini menggunakan dua baseline yang diberikan pengguna:

1. `motor.zip`: referensi algoritma/naming VESC (`foc_math.c`, `mcpwm_foc.c`, `mc_interface.c`, konfigurasi terkait).
2. `firmware(4).zip`: target STM32F103 aktual, termasuk pin/timer/ADC, runtime protocol, EEPROM, commissioning Hall/encoder, homing, dan GUI.

Tujuan rewrite bukan menyalin seluruh VESC secara verbatim. Yang dipindahkan adalah arsitektur FOC yang relevan untuk hardware project: current controller Id/Iq, outer speed/position -> Iq, Park/inverse Park, voltage vector limit, SVM enam sektor, serta penamaan state/config ala VESC. Observer/HFI dan sensorless estimator VESC tidak dimasukkan karena backend yang diminta adalah Hall/encoder dan menambahkan jalur yang tidak pernah dipakai akan menjadi dead code.

## 2. Hasil arsitektur akhir

### 2.1 Kepemilikan data sensor

`MotorRuntimeConfig.sensor_type` adalah satu-satunya selector:

- `MOTOR_SENSOR_HALL_UVW`
- `MOTOR_SENSOR_ENCODER_AB`

Tidak ada selector kedua di `mc_configuration` dan tidak ada `useMeasuredAngle/useMeasuredSpeed` lagi.

`DMA1_Channel1_IRQHandler()` mengambil snapshot GPIO, memanggil `MotorSensor_Update()`, kemudian membentuk `mc_foc_sample_t` dari **hasil backend yang sama**:

- `electrical_phase_q16`
- `mechanical_speed_q4`
- `position_ticks`
- `feedback_valid`

Core `mc_foc_run_current_control()` hanya menerima sample tersebut. Ia tidak mendecode raw Hall dan tidak menghitung encoder lagi.

### 2.2 Hall

Hall runtime memakai `hall_sequence[6]` dari config hasil commissioning. Sequence divalidasi sebagai enam state legal 1..6 dan Gray ring. `MotorSensor_PrepareRuntime()` membangun LUT raw->position sekali di slow/config path.

Pada ISR:

1. raw UVW dibentuk dari satu snapshot GPIO;
2. LUT calibrated raw->sector menentukan posisi listrik;
3. edge sector memperbarui position dan speed;
4. electrical phase diinterpolasi Q16 di antara boundary sektor;
5. phase/speed tersebut langsung menjadi input FOC.

Perbaikan reversal: estimator speed tidak lagi merata-ratakan sample +RPM lama dengan -RPM baru pada edge reversal. Arah baru dipakai langsung pada transition pertama sehingga feedback tidak jatuh ke nol selama satu Hall sector.

### 2.3 Encoder A/B

Pada mode encoder hanya U=A dan V=B dipakai; W diabaikan total. `encoder_sequence[4]` hasil calibration membentuk transition LUT runtime. CPR dan pole-pair diprecompute menjadi phase step fixed-point.

Electrical phase memakai quotient+remainder accumulator, sehingga:

```text
pole_pairs * 65536 = quotient * CPR + remainder
```

Setiap count menambah/mengurangi quotient dan remainder secara reversible tanpa division di normal edge path. Test membuktikan satu mechanical revolution penuh kembali ke electrical phase yang sama tanpa cumulative drift.

Encoder closed-loop membutuhkan:

- `encoder_calibrated=1`
- `encoder_sequence_valid=1`
- CPR valid
- electrical alignment per boot

Alignment menyimpan `encoder_electrical_zero_q16` yang merupakan field yang benar-benar dipakai oleh FOC. Mechanical zero/homing tidak mengubah electrical reference.

## 3. Mode kontrol

| Host mode | Core mode | Outer loop | Inner/current target |
|---|---|---|---|
| OPEN | `CONTROL_MODE_OPENLOOP` | forced phase generator | signed `m_iq_set` |
| VLT | `CONTROL_MODE_VOLTAGE` | tidak ada speed/pos loop | direct `m_voltage_q_set` |
| TRQ | `CONTROL_MODE_CURRENT` | tidak ada | direct `m_iq_set` |
| SPD | `CONTROL_MODE_SPEED` | speed PID slow-loop | PID output -> `m_iq_set` |
| POS | `CONTROL_MODE_POS` | position PID slow-loop | PID output -> `m_iq_set` |

Ini mengganti arsitektur lama yang membuat speed loop menghasilkan `Vq` langsung. Pada rewrite, SPD/POS/TRQ semuanya berakhir pada satu shared current controller Id/Iq, sesuai pola VESC.

## 4. Inner-loop FOC 16 kHz

Urutan current-loop:

1. current reconstruction dari dua ADC current:
   - LEFT: `Ia, Ib`, infer `Ic=-Ia-Ib`
   - RIGHT: `Ib, Ic`, infer `Ia=-Ib-Ic`
2. Clarke:

```text
Ialpha = Ia
Ibeta  = (Ia + 2*Ib) / sqrt(3)
```

3. Park, konvensi VESC:

```text
Id = cos(theta)*Ialpha + sin(theta)*Ibeta
Iq = cos(theta)*Ibeta  - sin(theta)*Ialpha
```

4. PI current untuk D dan Q memakai Kp/Ki yang sama tetapi integrator terpisah;
5. `Vd` diberi priority dan `Vq` dibatasi sisa voltage-circle;
6. inverse Park:

```text
Valpha = cos(theta)*Vd - sin(theta)*Vq
Vbeta  = cos(theta)*Vq + sin(theta)*Vd
```

7. six-sector SVM, fixed-point port dari konstruksi timing `foc_svm()` VESC;
8. output signed terhadap PWM center -> timer compare U/V/W.

## 5. Fixed-point dan beban ISR

| Besaran | Format |
|---|---|
| electrical phase | uint16 one-turn, 0..65535 |
| Hall interpolation accumulator | Q16.16 of one-turn phase |
| sin/cos, modulation | Q14 |
| mechanical speed | RPM Q4 |
| controller gains | Q16.16 |
| current/voltage internal | board Q4/count domain |

Hot path current controller tidak memakai `float`/`double`. `Ki/Fs`, voltage-to-modulation scale, dan voltage-circle index scale diprecompute di `mc_foc_conf_prepare()`.

Encoder normal sample/edge tidak memakai division untuk angle. Hall sample tanpa transition hanya melakukan accumulator add/decrement; division untuk speed dan phase step terjadi saat Hall edge, bukan setiap 16 kHz sample.

Gain input juga dibatasi di configuration boundary agar seluruh perkalian current-loop tetap aman di int32 dan malformed GUI/EEPROM value tidak menghasilkan signed overflow.

## 6. Safety ISR

Urutan safety diperketat:

- DC-current current chopping diperiksa sebelum FOC berat.
- Jika current chop aktif, sample `output_enabled=false`, sehingga current integrator reset dan tidak wind-up ketika MOE off.
- Setelah `MotorSensor_Update()`, closed-loop Hall/encoder dengan `feedback_valid=0` langsung clear `BDTR.MOE` pada sample ISR yang sama.
- OPEN/calibration boleh tetap berjalan karena memakai forced phase.
- Deadline monitor DMA tetap dipertahankan. Persistent overrun melatch fault mask dan mematikan MOE per motor.

## 7. Bug/regresi yang ditutup selama audit

1. **Hall reversal speed zero satu sektor** — diperbaiki dengan immediate sign change pada edge reversal.
2. **Encoder electrical alignment field mismatch** — alignment sekarang menyimpan `encoder_electrical_zero_q16`, field yang benar-benar dikonsumsi FOC.
3. **Potential encoder phase drift** dari reciprocal truncation — diganti quotient+remainder exact reversible phase accumulator.
4. **Duplicate sensor estimator/selector** — dihapus; single source of truth adalah `MotorSensor_Update()`.
5. **Speed loop langsung ke Vq** — diganti VESC-style speed PID -> Iq -> current PI.
6. **POS dipetakan ke speed command legacy** — diganti position PID -> Iq langsung, sesuai pola outer position VESC untuk current-producing loop.
7. **Signed negative shift UB** pada integrator clamp — diganti multiplication yang defined oleh C.
8. **Negative Q14 rounding bias** — diganti symmetric round-to-nearest.
9. **Current chop tanpa reset PI** — current-chopped sample sekarang men-disable current loop.
10. **Invalid sensor hanya duty=0 tetapi gate tetap aktif** — closed-loop invalid feedback sekarang clear hardware MOE pada ISR yang sama.
11. **Legacy MATLAB/Simulink-style state machine** (`active_branch_*`, `controlDelay*`, tiga state PID legacy) — dihapus dari source aktif.
12. **Dead controller macros/field-weakening constants legacy** yang tidak digunakan — dihapus dari config aktif.

## 8. Validasi yang dijalankan

### 8.1 Strict syntax integration

GCC dan Clang:

```text
-std=c11 -Wall -Wextra -Werror
```

Lulus untuk:

- `foc_motor.c`
- `foc_motor_data.c`
- `motor_sensor.c`
- `motor.c`
- `runtime_control.c`
- `util.c`

menggunakan HAL host stub untuk register/type yang disentuh jalur integrasi.

### 8.2 Behavioral tests

`tests/test_foc_sensor.c` mencakup:

- encoder backend dan W ignored;
- exact encoder electrical phase satu mechanical revolution;
- forward + reverse phase symmetry;
- electrical alignment Q16;
- mechanical zero tidak menggeser electrical phase;
- Hall proof gate;
- encoder proof gate;
- custom calibrated Hall sequence menguasai arah;
- reversed calibrated encoder sequence menguasai arah;
- Hall forward dan reverse speed sign;
- speed/position outer-loop sign;
- invalid phase -> zero output;
- forced phase OPEN/calibration;
- seluruh 6 SVM sector;
- LEFT `Ia/Ib` vs RIGHT `Ib/Ic` current reconstruction equivalence;
- positive/negative Iq -> Vq sign.

Result:

```text
ALL_FOC_SENSOR_TESTS_PASS
```

### 8.3 SVM vs VESC numeric reference

176,866 vector alpha/beta valid dibandingkan dengan persamaan float `foc_svm()` VESC:

```text
max_compare_error = 1 timer count
mean_abs_error    = 0.032113 timer count
sector_mismatch   = 7 / 176866
```

7 label sector berbeda hanya terjadi di boundary quantized Q14; timer compare tetap berada dalam error maksimum 1 count.

### 8.4 Trigonometry LUT

Seluruh 65,536 electrical phase dibandingkan sinus matematis:

```text
max_error      = 2 Q14 LSB
mean_abs_error = 0.553955 LSB
```

### 8.5 Undefined-behavior stress

UBSan deterministic stress menjalankan 250,000 current-loop sample + 20,000 outer-loop iterations, termasuk config gain ekstrem sebelum clamp.

```text
FOC_UBSAN_FUZZ_PASS
```

`tests/run_host_tests.sh` menghasilkan akhir:

```text
ALL_HOST_TESTS_PASS
```

## 9. Apa yang sengaja tidak dibawa dari full VESC

Untuk memenuhi syarat **tanpa dead code** dan **Hall/encoder sebagai sensor aktual**, modul berikut tidak dimasukkan:

- flux observer sensorless;
- HFI;
- encoder/observer blending sensorless;
- motor-model decoupling (`R`, `Ld`, `Lq`, flux-linkage feedforward) karena project belum memiliki parameter motor fisik terkalibrasi untuk itu;
- field weakening karena konfigurasi target sebelumnya nonaktif dan menambah jalur yang tidak tervalidasi.

Menyalakan fitur tersebut dengan parameter tebakan akan lebih berisiko daripada menghilangkannya. Core yang ada adalah closed-loop sensored FOC lengkap untuk mode yang diminta.

## 10. Batas validasi

Environment audit tidak memiliki `arm-none-eabi-gcc` maupun PlatformIO, sehingga **link final ARM, flash size, cycle timing nyata STM32F103, dead-time hardware, ADC sampling instant, dan respons motor fisik belum dapat divalidasi di sini**.

Sebelum high-power test:

1. gunakan power supply current-limited dan roda/motor tanpa beban;
2. verifikasi ADC current offset dekat nol saat PWM off;
3. jalankan sensor Auto Detect dan pastikan proof tersimpan;
4. Hall: putar manual maju/mundur, pastikan speed sign langsung berubah;
5. Encoder: lakukan electrical alignment dan cek angle tidak loncat setelah zero/homing;
6. mulai TRQ kecil, pastikan `Iq` dan arah torsi sesuai command;
7. baru tune current PI, lalu speed PID, terakhir position PID;
8. pantau `motorControlIsrOverrunCount`/fault mask;
9. scope PWM U/V/W dan gate complementary sebelum menaikkan tegangan/current.

## 11. Tuning

Nilai gain default tetap **starting point digital board**, bukan parameter universal VESC. Arsitektur dan nama variabel mengikuti VESC, tetapi nilai Kp/Ki current tidak boleh disalin buta dari VESC karena skala current ADC, bus voltage/modulation, motor R/L, dan sampling hardware berbeda.

Urutan tuning yang aman:

```text
current Id/Iq PI -> speed PID -> position PID
```

Jangan tuning speed/position untuk menutupi error sensor phase atau current reconstruction.

## 12. Lisensi

Lihat `NOTICE.md` dan `COPYING`. Rewrite mempertahankan GPL-3.0-or-later sesuai VESC reference dan firmware board asal.
