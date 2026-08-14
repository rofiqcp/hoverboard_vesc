# CHANGELOG V18

## Root cause dari hardware log 2026-08-13 09:51:45

- SET_CURRENT 0.5 A sendiri bukan speed command; right motor dapat akselerasi
  ke beberapa ribu eRPM sambil Iq tetap sekitar command.
- V17 brake dan handbrake masih collapse ke ordinary CURRENT di runtime, jadi
  semantik VESC keduanya belum benar.
- V16/V17 overload relief dapat melewati sensor+FOC update; speed formula tetap
  mengasumsikan cadence penuh, sehingga RPM update tidak mempunyai timing
  contract yang cukup kuat.
- LEFT/RIGHT two-FOC-every-16k path sebelumnya pernah membuat UART starvation
  pada STM32F103 64 MHz.

## Fix

- PWM/ADC/sensor cadence = 16 kHz.
- FOC deterministic interleave = 8 kHz per motor.
- Sensor read selalu dilakukan sebelum emergency shed.
- Encoder/Hall speed coefficients memakai sensor cadence 16 kHz.
- New dedicated `CONTROL_MODE_CURRENT_BRAKE`.
- New dedicated `CONTROL_MODE_HANDBRAKE`.
- Brake Iq sign recomputed from instantaneous speed every fast current update.
- Handbrake forces electrical phase 0.
- Positive/negative current commands use their corresponding configured current
  limits; brake magnitude uses braking-current limit.
- Removed unused `commissioning_current_guard` and compile warning.
- Added hardware-oriented brake/handbrake tester assertions.
- Added exact 16-kHz encoder/Hall RPM numerical regression.
