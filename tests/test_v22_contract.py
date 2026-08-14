#!/usr/bin/env python3
"""Static architecture/compatibility contracts for V22."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def read(name: str) -> str:
    return (SRC / name).read_text(encoding="utf-8", errors="replace")


def test_non_rtos_main_scheduler():
    main = read("main.c")
    svc = read("vesc_services.c")
    assert "VescServices_Init();" in main
    assert "VescServices_Run();" in main
    assert "poweronMelody();" not in main
    assert "HAL_Delay(" not in main
    assert "THD_FUNCTION" not in main + svc
    assert "chThd" not in main + svc
    assert "osThread" not in main + svc


def test_packet_process_is_bounded():
    proto = read("vesc_protocol.c")
    hdr = read("vesc_protocol.h")
    assert "VescProtocol_ServiceBudget(uint16_t max_rx_bytes)" in proto
    assert "consumed < max_rx_bytes" in proto
    assert "VESC_RX_SERVICE_DEFAULT_BUDGET 160U" in proto
    assert "VescProtocol_RxBudgetYields" in hdr


def test_critical_reply_backpressure():
    proto = read("vesc_protocol.c")
    assert "critical_reply_valid" in proto
    assert "send_payload_critical" in proto
    assert "critical_reply_service" in proto
    for marker in [
        "C_FW_VERSION",
        "C_GET_MCCONF",
        "C_SET_MCCONF",
        "C_GET_APPCONF",
        "C_SET_APPCONF",
        "C_PING_CAN",
    ]:
        assert marker in proto


def test_v22_identity_and_dual_virtual_can():
    app = read("vesc_app.c")
    main = read("main.c")
    proto = read("vesc_protocol.c")
    assert "config->controller_id = 1U;" in app
    assert "vescAppConfig.controller_id = 1U;" in main
    assert "vescAppConfig.controller_id=1U" in proto
    assert '"hoverboard-vesc6-v22"' in proto
    assert "return n>=254U?0U:(uint8_t)(n+1U);" in proto
    assert "C_FORWARD_CAN" in proto
    assert "C_PING_CAN" in proto


def test_app_adc_safety_and_curve():
    app = read("vesc_app.c")
    assert "adc_range_ok" in app
    assert "RuntimeControl_HasBlockingFault()" in app
    assert "safe_start_ok = false" in app
    assert "safe_start_neutral_ms" in app
    assert "safe_start_neutral_ms >= 500U" in app
    assert app.index("switch (vescAppConfig.adc_ctrl_type)") < app.index("safe_start_neutral_ms >= 500U")
    assert "powf(" in app
    assert "expf(" in app
    assert "Polynomial" in app
    assert "ramp_permille" in app


def test_thread_semantics_present():
    svc = read("vesc_services.c")
    for name in [
        "adc_thread",
        "packet_process_thread",
        "blocking_thread",
        "timer_thread",
        "sample_send_thread",
        "fault_stop_thread",
        "stat_thread",
        "pid_thread",
        "rpm_thread",
        "periodic_thread",
        "led_thread",
    ]:
        assert name in svc

    # There must be one actual FOC outer-loop call, not an extra BLDC RPM writer.
    calls = re.findall(r"(?m)^\s*RuntimeControl_UpdateSlow\s*\(", svc)
    assert len(calls) == 1


def test_status_pattern_contract():
    svc = read("vesc_services.c")
    assert "VESC_NORMAL_FLASH_PERIOD_MS   1000U" in svc
    assert "beepCount(code, 24U, 1U)" in svc
    assert "VESC_FAULT_FLASH_ON_MS" in svc
    assert "VESC_FAULT_FLASH_GAP_MS" in svc
    assert svc.index("BAT_LVL2_ENABLE") < svc.index("BAT_LVL1_ENABLE")


def test_legacy_raw_telemetry_not_mixed_into_vesc_uart():
    svc = read("vesc_services.c")
    calls = re.findall(r"(?m)^\s*RuntimeControl_ServiceTelemetry\s*\(", svc)
    assert calls == []


def test_detect_command_paths_retained():
    proto = read("vesc_protocol.c")
    for marker in [
        "C_DETECT_ENCODER",
        "C_DETECT_HALL_FOC",
        "C_DETECT_APPLY_ALL_FOC",
        "RuntimeControl_VescStartSensorDetect",
        "auto_detect_service",
    ]:
        assert marker in proto


def test_config_wire_compatibility_file_retained():
    conf = read("vesc_config_compat.c")
    # VESC 6.00 baseline uses 481-byte MCCONF and 493-byte APPCONF shadows.
    assert "481" in conf
    assert "493" in conf


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for fn in tests:
        fn()
        print("PASS", fn.__name__)


def test_linker_symbols_are_defined_not_only_declared():
    protocol = read("src/vesc_protocol.c")
    header = read("src/vesc_protocol.h")
    services = read("src/vesc_services.c")
    assert protocol.count("void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)") == 1
    assert protocol.count("uint32_t VescProtocol_RxBudgetYields(void)") == 1
    assert "VescProtocol_ServiceBudget(uint16_t max_rx_bytes)" in header
    assert "VescProtocol_RxBudgetYields(void)" in header
    assert "VescProtocol_ServiceBudget(VESC_RX_BUDGET_PER_PASS)" in services
    assert "VescProtocol_RxBudgetYields()" in services
