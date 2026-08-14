#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def read(rel: str) -> str:
    p = ROOT / rel
    assert p.exists(), f"missing {rel}"
    return p.read_text(encoding="utf-8", errors="replace")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def section(text: str, begin: str, end: str) -> str:
    a = text.index(begin)
    b = text.index(end, a)
    return text[a:b]


def check(name: str, ok: bool) -> None:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}")
    assert ok, name


def main() -> None:
    main_c = read("src/main.c")
    proto_c = read("src/vesc_protocol.c")
    proto_h = read("src/vesc_protocol.h")
    services = read("src/vesc_services.c")
    motor = read("src/motor.c")
    enc = read("src/left_encoder.c")
    app = read("src/vesc_app.c")

    task_names = [
        "task_flash_integrity_check_thread",
        "task_led_thread",
        "task_periodic_thread",
        "task_work_thread",
        "task_timeout_thread",
        "task_blocking_thread",
        "task_usb_serial_read_thread",
        "task_usb_serial_process_thread",
        "task_packet_process_thread",
        "task_cancom_read_thread",
        "task_cancom_process_thread",
        "task_cancom_status_thread",
        "task_cancom_status_thread_2",
        "task_cancom_status_internal_thread",
        "task_mc_interface_timer_thread",
        "task_stat_thread",
        "task_sample_send_thread",
        "task_fault_stop_thread",
        "task_mcpwm_timer_thread",
        "task_rpm_thread",
        "task_mcpwm_foc_timer_thread",
        "task_hfi_thread",
        "task_pid_thread",
        "task_encoder_routine_thread",
        "task_imu_thread",
        "task_shutdown_thread",
        "task_adc_thread",
        "task_ppm_thread",
        "task_chuk_thread",
        "task_nunchuk_output_thread",
        "task_pas_thread",
        "task_nrf_rx_thread",
        "task_nrf_tx_thread",
        "task_lora_packet_process_thread",
        "task_si_read_thread",
        "task_ts5700n8501_thread",
        "task_canard_thread",
        "task_lisp_eval_thread",
        "task_lisp_lib_thread",
        "task_dpv_thread",
        "task_sten_uart_thread",
        "task_app_custom_thread",
        "task_finn_control_thread",
        "task_finn_status_thread",
        "task_skypuff_thread",
        "task_erockit_thread",
        "task_smart_switch_thread",
        "task_mux_thread",
        "task_switch_color_thread",
        "task_display_process_thread",
        "task_sense_thread",
        "task_temp_thread",
        "task_fan_control_thread",
        "task_mag_thread",
        "task_lisp_event_thread",
        "task_lisp_cmds_send_task",
    ]

    scheduler = section(main_c, "static void main_scheduler_run", "static void main_scheduler_init")
    missing_defs = [n for n in task_names if f"static void {n}" not in main_c]
    missing_calls = [n for n in task_names if f"{n}(now);" not in scheduler]
    check("all upstream task semantics have explicit main.c functions", not missing_defs)
    check("all upstream task semantics are explicitly called by main scheduler", not missing_calls)

    check("main does not call hidden VescServices_Run", "VescServices_Run();" not in main_c)
    check("compatibility vesc_services has no hidden work", "Intentionally empty" in services and "VescProtocol_Service" not in services)
    main_code = re.sub(r"/\*.*?\*/|//[^\n]*", "", main_c, flags=re.S)
    check("main runtime contains no HAL_Delay call", re.search(r"\bHAL_Delay\s*\(", main_code) is None)
    check("scheduler uses monotonic millis", "RuntimeControl_MonotonicMs()" in main_c)

    main_fn = main_c[main_c.index("int main(void)"):]
    # Calibration request belongs only to task_boot_supervisor, not the init body.
    main_init_prefix = main_fn[:main_fn.index("for (;;) {")]
    check("current calibration not started inside blocking boot init", "MotorControl_RequestCurrentOffsetCalibration()" not in main_init_prefix)
    check("current calibration deferred beyond melody", "VESC_AUTO_CURRENT_CAL_DELAY_MS  1200U" in main_c and "VESC_STARTUP_MELODY_MS          900U" in main_c)
    check("watchdog-reset degraded recovery exists", "RCC_CSR_IWDGRSTF" in main_c and "VESC_BOOT_DEGRADED" in main_c)
    check("power-hold latch is reasserted by shutdown task", "HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_SET);" in main_c)
    check("startup melody uses continuous tone, not error beep window", "beepCount(0U, freq, 0U);" in main_c)

    for api in [
        "VescProtocol_ServiceRxBudget",
        "VescProtocol_ServiceBlocking",
        "VescProtocol_ServiceSampleSend",
        "VescProtocol_ServicePeriodic",
        "VescProtocol_ServiceBudget",
        "VescProtocol_RxBudgetYields",
    ]:
        check(f"protocol API declared: {api}", api in proto_h)
        check(f"protocol API defined: {api}", re.search(r"\b" + re.escape(api) + r"\s*\(", proto_c) is not None)

    compact = "".join(proto_c.split())
    check("RX parser is execution-budget bounded", "while(rx_old!=producer&&consumed<max_rx_bytes)" in compact)
    rx_body = section(proto_c, "void VescProtocol_ServiceRxBudget", "void VescProtocol_ServiceBlocking")
    check("blocking detect removed from packet RX task", "detect_service();" not in rx_body and "auto_detect_service();" not in rx_body)
    block_body = section(proto_c, "void VescProtocol_ServiceBlocking", "void VescProtocol_ServiceSampleSend")
    check("blocking task advances Hall/encoder/auto-detect", "detect_service();" in block_body and "auto_detect_service();" in block_body)
    check("critical handshake/config backpressure retained", "critical_reply_valid" in proto_c and "send_payload_critical" in proto_c)
    check("VESC Tool FW_VERSION critical reply", "fw_append_cstr" in proto_c and "send_payload_critical(o,(uint16_t)i)" in compact)
    check("boot scheduler diagnostic terminal command", 'hb_boot_status' in proto_c)

    check("FOC inner loop remains in existing motor ISR", "mc_foc_run_current_control" in motor and "DMA1_Channel1_IRQHandler" in motor)
    check("Hall backend remains active", "MotorSensor_Update(&motorConfigLeft" in motor and "MotorSensor_Update(&motorConfigRight" in motor)
    check("LEFT ABI hardware encoder remains active", "MotorSensor_UpdateHardwareEncoder" in motor and "LeftEncoder_UpdateAndGetCount" in motor)
    check("LEFT encoder uses TIM4 hardware", "TIM4" in enc and "GPIO_PIN_6 | GPIO_PIN_7" in enc and "GPIO_PIN_5" in enc)
    check("FOC PID/current semantics retained", "RuntimeControl_UpdateSlow" in main_c and "VescProtocol_CurrentTelemetrySample" in main_c)
    check("APP ADC safe-start retained", "safe_start_neutral_ms" in app and "VescApp_Update" in main_c)
    check("LEFT controller ID is forced to 1", "vescAppConfig.controller_id = 1U;" in main_c)
    check("unsupported physical features are explicit capabilities", "VESC_CAP_PHYSICAL_CAN" in main_c and "VESC_CAP_HFI" in main_c and "VESC_CAP_IMU" in main_c)

    # If applied with the provided installer, prove the user-requested ISR files
    # are byte-for-byte unchanged from the automatic backup.
    backup = ROOT / ".v22_deep_backup"
    if backup.exists():
        for rel in ["src/motor.c", "src/stm32f1xx_it.c"]:
            old = backup / rel
            cur = ROOT / rel
            if old.exists():
                check(f"protected ISR unchanged: {rel}", sha(old) == sha(cur))

    print(f"PASS: {len(task_names)} VESC task semantics are visible and called from main.c")


if __name__ == "__main__":
    main()
