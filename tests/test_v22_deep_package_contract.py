#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
main = (ROOT / "src/main.c").read_text()
h = (ROOT / "src/vesc_protocol.h").read_text()
patcher = (ROOT / "tools/apply_v22_deep_main_scheduler.py").read_text()

assert "main_scheduler_run" in main
assert "VescServices_Run();" not in main
assert "MotorControl_RequestCurrentOffsetCalibration()" in main
main_body = main[main.index("int main(void)"):]
assert "MotorControl_RequestCurrentOffsetCalibration()" not in main_body[:main_body.index("for (;;) {")]
assert "beepCount(0U, freq, 0U);" in main
assert "RCC_CSR_IWDGRSTF" in main
assert "VESC_BOOT_DEGRADED" in main
assert "task_packet_process_thread(now);" in main
assert "task_blocking_thread(now);" in main
assert "task_fault_stop_thread(now);" in main
assert "task_pid_thread(now);" in main
assert "task_adc_thread(now);" in main
assert "task_encoder_routine_thread(now);" in main
for name in ["VescProtocol_ServiceRxBudget", "VescProtocol_ServiceBlocking", "VescProtocol_ServiceSampleSend", "VescProtocol_ServicePeriodic"]:
    assert name in h
    assert name in patcher
assert "PROTECTED_ISR" in patcher and '"src/motor.c"' in patcher and '"src/stm32f1xx_it.c"' in patcher
assert "while (rx_old != producer && consumed < max_rx_bytes)" in patcher
assert "hb_boot_status" in patcher
print("PASS: deep-main package contract")
