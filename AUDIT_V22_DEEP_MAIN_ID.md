# Audit V22 Deep Main Scheduler — STM32F103RCT6 Dual FOC

## Temuan akar masalah yang relevan dengan gejala boot/melody

1. Pada source v22 sebelumnya, `main()` hanya memanggil `VescServices_Run()`. Task VESC tidak terlihat sebagai slot eksekusi terpisah.
2. `VescProtocol_Service()` lama mencampur packet RX, detect/blocking state machine, rotor-position periodic dan TX queue dalam satu fungsi. RX ring juga dapat dikuras tanpa execution budget.
3. Current-offset calibration dimulai langsung saat boot. Selama calibration, ISR motor keluar lebih awal; begitu offset menjadi VALID, ISR yang sama langsung masuk ke path sensor + FOC penuh. Perubahan beban ISR ini terjadi saat startup melody masih dimainkan, sehingga melody yang berhenti bukan bukti bahwa buzzer adalah sumber hang.
4. Mesin melody lama memakai mode beep-count/pattern; pola tersebut mempunyai jendela ON/OFF yang tidak cocok untuk note 100 ms.
5. HardFault/BusFault/UsageFault sudah mempunyai warm-reset record, tetapi boot lama langsung mengulangi path yang sama. Tidak ada recovery mode untuk main/ISR starvation.

## Perubahan arsitektur

- ISR `src/motor.c` dan `src/stm32f1xx_it.c` tidak disentuh. Installer memeriksa SHA256 sebelum/sesudah.
- Semua semantic task VESC ditulis sebagai fungsi bernama dan dipanggil eksplisit dari `main_scheduler_run()`.
- Clock cooperative menggunakan `RuntimeControl_MonotonicMs()`; tidak ada `HAL_Delay()` di runtime main.
- UART parser dipisahkan dari blocking detect dan periodic/send worker.
- RX parser dibatasi 128 byte per pass, dengan max hard cap 512 byte.
- FW/config replies penting mempunyai deferred critical-reply backpressure.
- Startup current calibration dipindahkan ke `task_boot_supervisor()` setelah 1200 ms, sesudah melody 900 ms dan sesudah VESC Tool/main scheduler hidup.
- IWDG diaktifkan hanya setelah heartbeat DMA motor pertama terlihat. Jika main benar-benar starvation, watchdog reset. Boot berikutnya membaca reset cause dan masuk `VESC_BOOT_DEGRADED`: UART/VESC Tool tetap hidup, tetapi auto current-cal tidak diulang, sehingga reset loop tidak menutupi penyebab.
- Terminal VESC Tool baru: `hb_boot_status` / `hb_sched` untuk melihat phase boot, degraded flag, CPU-fault code, reset flags, scheduler passes, UART RX/CRC/TX drop/RX-budget-yield.
- Shutdown semantic secara aman terus menegaskan power-hold `OFF_PIN=SET`; tidak ada shutdown palsu tanpa state machine hardware yang tervalidasi.

## Ownership motor

- Fast current/FOC: tetap `DMA1_Channel1_IRQHandler()` -> `mc_foc_run_current_control()`.
- LEFT feedback: Hall PB5/PB6/PB7 atau ABI TIM4 A=PB6, B=PB7, Z=PB5.
- RIGHT feedback: Hall.
- FOC outer loop/arming/timeout/homing/detect: backend `RuntimeControl_UpdateSlow()` tetap menjadi single writer. `fault_stop`, `mc_interface timer`, `mcpwm_foc timer`, `pid` dan `timeout` wrappers semua memanggil owner yang sama; deadline guard membuat owner hanya dieksekusi sekali per 5 ms sehingga tidak ada dua PID writer.
- LEFT VESC ID = 1; RIGHT virtual CAN ID = 2.

## Fitur kondisional

Semua task upstream yang ditemukan di audit tetap mempunyai fungsi dan call-site di `main.c`. Fitur yang membutuhkan hardware yang tidak ada di board ini (USB OTG, physical CAN transceiver mapping, HFI, IMU, NRF, LoRa, SI8900, TS5700, UAVCAN, LispBM, PPM/Nunchuk/PAS, board mux/fan/mag/display) diberi capability `0` dan return tanpa akses hardware. Ini disengaja: task tidak hilang dari scheduler, tetapi firmware tidak mengklaim driver yang tidak tersedia.

## Debug hardware

Setelah apply dan build:

```bash
python3 tools/v22_deep_debug.py . --port /dev/ttyUSB0 --watch-seconds 8
```

Probe serial bersifat read-only: FW_VERSION, `hb_boot_status`, GET_VALUES, PING_CAN, RIGHT FW_VERSION via FORWARD_CAN, GET_MCCONF/APPCONF, dan recovery dari bad CRC. Tidak ada SET current/duty/rpm/position dan tidak memulai detect.
