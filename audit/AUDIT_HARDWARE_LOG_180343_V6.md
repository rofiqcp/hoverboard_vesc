# Audit Hardware Log 2026-08-12 18:03:43 — V6 Fix

## Ringkasan first failure

Log `vesc_full_test_20260812_180343.zip` menunjukkan current-zero V5 sudah VALID dan MCCONF round-trip PASS. First blocking failure berpindah ke deadline current ISR:

- `isr_deadline_cycles = 4000` (64 MHz / 16 kHz)
- observed `isr_last_cycles ≈ 4763..4960`
- observed historical max `isr_max_cycles = 6241`
- `CONTROL_ISR_OVERRUN` kemudian abort Detect Encoder LEFT dan Detect Hall RIGHT

Jadi menaikkan deadline tanpa mengubah scheduler akan menyembunyikan overload nyata. V6 mempertahankan PWM/ADC 16 kHz tetapi menginterleave kalkulasi FOC: LEFT pada satu sample, RIGHT pada sample berikutnya. Per-motor FOC/sensor update menjadi 8 kHz.

## Audit current scaling

Log firmware menyatakan `current_units_per_amp = 800`. Core current reconstruction memperbesar ADC delta x16, sehingga:

- stock board physical gain = 50 ADC count/A
- internal gain = 50 x 16 = 800 internal unit/A
- `Id[A] = Id_internal / 800`
- `Iq[A] = Iq_internal / 800`

Ini identik dengan konstanta `A2BIT_CONV = 50` pada firmware EFeru stock. V6 menjadikan nilai ini satu hardware constant dan `mc_foc_conf_prepare()` selalu mengembalikannya ke 800; VESC/EEPROM tidak boleh mengubah physical current gain.

### Data idle dari run 18:03

Setelah current-zero valid, maksimum nilai idle yang direkam tester:

| Node | Imotor | Id | Iq | Iin/DC-link |
|---|---:|---:|---:|---:|
| LEFT/local | 0.79 A | 0.05 A | 0.56 A | 0.28 A |
| RIGHT | 0.17 A | 0.18 A | 0.10 A | 0.14 A |

Nilai ini tidak mendukung perubahan gain fisik secara spekulatif. Keluhan bahwa VESC Tool naik terlalu agresif saat roda digerakkan lebih sesuai dengan perbedaan semantics telemetry: V5 mengirim snapshot fast-filter, sedangkan firmware VESC memakai read-reset averages untuk Motor Current, Input Current, Id dan Iq.

## V6 current telemetry fix

`COMM_GET_VALUES` sekarang memakai accumulator background dan read-reset average untuk:

- Motor Current
- Input Current
- Id
- Iq

HBTS diagnostic tetap instantaneous. Ini sengaja: GUI menjadi stabil seperti VESC, tetapi troubleshooting tetap dapat melihat spike aktual.

Tester V6 menambah stage `06b_passive_spin_current_observation`. Output tetap OFF dan user dapat memutar roda manual selama default 3 s. Log membandingkan:

- VESC averaged current
- HBTS instantaneous current
- DCL/DCR
- fixed A/count scale

Jika instantaneous phase-vector >3 A tetapi DC-link <0.5 A, tester memberi WARN `passive_current_common_mode_suspect`. Raw data tidak di-clamp.

## V6 dual FOC scheduler

Hardware timeline:

```text
PWM + ADC DMA : 16 kHz
DMA slot 0    : LEFT sensor + LEFT FOC + LEFT SVPWM
DMA slot 1    : RIGHT sensor + RIGHT FOC + RIGHT SVPWM
repeat
```

Maka:

```text
LEFT current loop  = 8 kHz
RIGHT current loop = 8 kHz
PWM switching      = 16 kHz
DC-link chopping   = 16 kHz (kedua motor setiap sample)
```

Konstanta yang bergantung pada dt juga diubah menjadi 8 kHz:

- current PI Ki*dt
- Hall/encoder speed estimator update frequency
- Hall interpolation update period
- OPEN-loop electrical phase increment
- FW ramp timing

Deadline ISR tetap 4000 core cycle karena DMA masih 16 kHz. Jika V6 masih melewati 4000 cycle pada hardware, log berikutnya akan menunjukkan overrun aktual setelah pekerjaan FOC sudah dibagi dua; deadline tidak dinaikkan secara buta.

## Safety yang tidak diubah

- current offset calibration tetap wajib sebelum actuation
- DCL/DCR over-current check tetap berjalan tiap 16-kHz ADC sample
- sensor feedback proof tetap wajib untuk closed-loop
- Detect Hall/Encoder tetap abort pada internal/fault condition
- right motor tetap Hall-only
- connect VESC Tool tidak mengaktifkan PWM

## Verification di environment pengembangan

- host regression: 12/12 PASS
- UBSan FOC fuzz: PASS
- VESC packet/config/app tests: PASS
- current calibration robust-policy test: PASS
- static pin/resource/profile contract: PASS
- Cortex-M3 codegen audit: PASS
- no division helper in current-control, hardware-encoder, or DMA ISR hot paths

Belum tersedia board fisik atau `arm-none-eabi-gcc`/PlatformIO target toolchain di environment ini, sehingga cycle count V6 yang sesungguhnya harus diverifikasi dari log hardware berikutnya.
