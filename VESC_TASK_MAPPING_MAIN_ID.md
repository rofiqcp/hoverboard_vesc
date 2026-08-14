# Mapping Task VESC -> Cooperative `main.c`

Semua fungsi di kolom kanan dipanggil eksplisit oleh `main_scheduler_run()`.

| Upstream VESC task/family | Bare-metal main task | Target |
|---|---|---|
| flash_integrity_check_thread | task_flash_integrity_check_thread | called; no fake CRC manifest |
| led_thread | task_led_thread | active |
| periodic_thread | task_periodic_thread | active |
| work_thread | task_work_thread | hook |
| timeout_thread | task_timeout_thread | active + IWDG supervisor |
| commands blocking_thread | task_blocking_thread | active |
| UART packet_process_thread | task_packet_process_thread | active bounded RX |
| USB serial_read/process | task_usb_serial_* | called, capability 0 |
| CAN read/process/status/status2 | task_cancom_* | called, physical capability 0 |
| dual internal CAN status | task_cancom_status_internal_thread | called; ID2 handled virtually in protocol |
| mc_interface timer_thread | task_mc_interface_timer_thread | shared runtime owner |
| mc_interface stat_thread | task_stat_thread | active |
| sample_send_thread | task_sample_send_thread | active deferred TX |
| fault_stop_thread | task_fault_stop_thread | highest cooperative call; shared runtime owner |
| BLDC timer/rpm | task_mcpwm_timer_thread/task_rpm_thread | called, FOC target so no second controller |
| mcpwm_foc timer_thread | task_mcpwm_foc_timer_thread | shared runtime owner |
| hfi_thread | task_hfi_thread | called, disabled by product requirement |
| pid_thread | task_pid_thread | shared single writer |
| encoder routine_thread | task_encoder_routine_thread | called; TIM4/Hall fast feedback remains ISR/hardware |
| IMU thread | task_imu_thread | called, capability 0 |
| shutdown_thread | task_shutdown_thread | active power-hold supervision |
| APP ADC adc_thread | task_adc_thread | active |
| PPM/Nunchuk/PAS | task_ppm/chuk/nunchuk_output/pas | called, capability 0 |
| NRF rx/tx | task_nrf_rx/task_nrf_tx | called, capability 0 |
| LoRa packet task | task_lora_packet_process_thread | called, capability 0 |
| SI8900 | task_si_read_thread | called, capability 0 |
| TS5700 | task_ts5700n8501_thread | called, capability 0 |
| UAVCAN/canard | task_canard_thread | called, capability 0 |
| LispBM eval/lib/event/cmd | task_lisp_* | called, capability 0 |
| DPV/STEN/custom/FINN/SkyPuff/eRockit | task_* | called hooks |
| board smart switch/mux/color/display/sense/temp/fan/mag | task_* | called hooks; active only where board resource exists |
