# V6 Changes

## 1. Current scale dikunci ke hardware stock

- satu sumber: `CONTROL_CURRENT_ADC_COUNTS_PER_A = 50`
- fixed-point internal x16 -> `800 internal units/A`
- `mc_foc_conf_prepare()` selalu mengembalikan physical gain ke 800
- VESC/EEPROM tidak dapat mengubah current-sense gain secara tidak sengaja

## 2. VESC current telemetry mengikuti read-reset averaging

`COMM_GET_VALUES` Motor Current, Input Current, Id dan Iq sekarang memakai accumulator background yang di-reset saat dibaca. HBTS tetap instantaneous.

## 3. Passive manual-spin current debug

Python tester menambah `06b_passive_spin_current_observation` dengan output bridge OFF. Log merekam VESC average vs HBTS instantaneous vs DCL/DCR dan memberi warning bila phase current besar tanpa dukungan DC-link current.

## 4. Dual FOC interleaved scheduler

PWM/ADC tetap 16 kHz, tetapi kalkulasi FOC bergantian LEFT/RIGHT. Masing-masing motor FOC + rotor estimator berjalan 8 kHz. Ini memperbaiki overload V5 yang tercatat 4.8-6.2k cycle per ISR dibanding deadline 4000 cycle.

## 5. Timing-dependent control constants ikut 8 kHz

Current PI Ki dt, sensor speed estimator, Hall interpolation, FW timing dan OPEN phase step memakai `CONTROL_FOC_MOTOR_FREQUENCY_HZ = 8000`.

## 6. Additional numeric hardening

Setelah update dt ke 8 kHz, UBSan menemukan potensi overflow pada malicious maximum current-Ki configuration. `FOC_CURRENT_KI_Q16_MAX` sekarang diturunkan berdasarkan safe per-sample `Ki*dt <= 65535`, lalu regression UBSan kembali PASS.
