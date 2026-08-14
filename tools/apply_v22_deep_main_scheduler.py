#!/usr/bin/env python3
"""Apply the V22 deep main-owned cooperative VESC scheduler revision.

Targets both:
- current GitHub v22 source where VescProtocol_Service() is still monolithic, and
- local trees that already received the earlier V22 ServiceBudget/critical-reply hotfix.

Safety properties:
- src/motor.c and src/stm32f1xx_it.c are hash-checked before/after and NEVER edited.
- backups of every edited source file are stored under .v22_deep_backup/.
- protocol parser is bounded and split into explicit packet/blocking/sample/periodic services.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import shutil
import sys
from pathlib import Path

EDITED = [
    "src/main.c",
    "src/vesc_protocol.c",
    "src/vesc_protocol.h",
    "src/vesc_services.c",
    "src/vesc_services.h",
]
PROTECTED_ISR = ["src/motor.c", "src/stm32f1xx_it.c"]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text: str, old: str, new: str, label: str, *, allow_already: str | None = None) -> str:
    if allow_already and allow_already in text:
        return text
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"{label}: expected exactly one source marker, got {n}")
    return text.replace(old, new, 1)


def ensure_protocol_helpers(text: str) -> str:
    if "VESC_RX_SERVICE_DEFAULT_BUDGET" not in text:
        text = replace_once(
            text,
            "#define VESC_AUTO_DETECT_CURRENT_A 1.0f\n",
            "#define VESC_AUTO_DETECT_CURRENT_A 1.0f\n"
            "#define VESC_RX_SERVICE_DEFAULT_BUDGET 128U\n"
            "#define VESC_RX_SERVICE_MAX_BUDGET 512U\n",
            "RX budget defines",
        )

    if "rx_budget_yields" not in text:
        text = replace_once(
            text,
            "static uint32_t rx_packets=0,crc_or_parser_errors=0,tx_drops=0,last_uart_control_ms=0;\n",
            "static uint32_t rx_packets=0,crc_or_parser_errors=0,tx_drops=0,last_uart_control_ms=0;\n"
            "static uint32_t rx_budget_yields=0U;\n",
            "RX budget counter",
        )

    if "critical_reply_valid" not in text:
        text = replace_once(
            text,
            "static volatile uint8_t tx_head=0,tx_tail=0;\nstatic volatile bool tx_busy=false;\n",
            "static volatile uint8_t tx_head=0,tx_tail=0;\n"
            "static volatile bool tx_busy=false;\n"
            "/* One-slot backpressure for VESC Tool handshake/config replies. */\n"
            "static uint8_t critical_reply[VESC_PACKET_MAX_PAYLOAD];\n"
            "static uint16_t critical_reply_len=0U;\n"
            "static bool critical_reply_valid=false;\n",
            "critical reply storage",
        )

    if "uint32_t VescProtocol_RxBudgetYields(void)" not in text:
        text = replace_once(
            text,
            "uint32_t VescProtocol_TxDrops(void){return tx_drops;}\n",
            "uint32_t VescProtocol_TxDrops(void){return tx_drops;}\n"
            "uint32_t VescProtocol_RxBudgetYields(void){return rx_budget_yields;}\n",
            "RX budget getter",
        )

    if "static void critical_reply_service(void)" not in text:
        marker = "static bool send_payload_terminal(const uint8_t*p,uint16_t len){return send_payload_internal(p,len,false);}\n"
        helper = marker + r'''static void critical_reply_service(void)
{
    if(!critical_reply_valid)return;
    if(send_payload_internal(critical_reply,critical_reply_len,false)){
        critical_reply_valid=false;
        critical_reply_len=0U;
    }
}
static bool send_payload_critical(const uint8_t*p,uint16_t len)
{
    if(send_payload_internal(p,len,false))return true;
    if(!critical_reply_valid && p!=NULL && len>0U && len<=VESC_PACKET_MAX_PAYLOAD){
        memcpy(critical_reply,p,len);
        critical_reply_len=len;
        critical_reply_valid=true;
        return true;
    }
    ++tx_drops;
    return false;
}
'''
        text = replace_once(text, marker, helper, "critical reply helpers")

    return text


def make_critical_replies(text: str) -> str:
    # Firmware identity. The wire FW major/minor stays 6.00 for VESC Tool compatibility.
    text = text.replace('"hoverboard-vesc6-v24"', '"hoverboard-vesc6-v22-main"')
    text = text.replace('"hoverboard-vesc6-v22"', '"hoverboard-vesc6-v22-main"')

    # FW_VERSION response.
    text = text.replace(
        'if(!fw_append_cstr(o,sizeof(o),&i,"hoverboard-vesc6-v22-main")) return;\n    send_payload(o,(uint16_t)i);',
        'if(!fw_append_cstr(o,sizeof(o),&i,"hoverboard-vesc6-v22-main")) return;\n    (void)send_payload_critical(o,(uint16_t)i);',
    )

    replacements = [
        (
            'case C_GET_DECODED_ADC:{uint8_t o[17];int32_t k=0;o[k++]=C_GET_DECODED_ADC;vesc_buf_append_i32(o,VescApp_GetDecoded1Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage1MicroV(),&k);vesc_buf_append_i32(o,VescApp_GetDecoded2Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage2MicroV(),&k);send_payload(o,(uint16_t)k);}break;',
            'case C_GET_DECODED_ADC:{uint8_t o[17];int32_t k=0;o[k++]=C_GET_DECODED_ADC;vesc_buf_append_i32(o,VescApp_GetDecoded1Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage1MicroV(),&k);vesc_buf_append_i32(o,VescApp_GetDecoded2Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage2MicroV(),&k);(void)send_payload_critical(o,(uint16_t)k);}break;',
        ),
        (
            'case C_GET_MCCONF:case C_GET_MCCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeMc(o+1,r,cmd==C_GET_MCCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;',
            'case C_GET_MCCONF:case C_GET_MCCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeMc(o+1,r,cmd==C_GET_MCCONF_DEFAULT);if(k>0)(void)send_payload_critical(o,(uint16_t)(k+1));}break;',
        ),
        (
            'if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};send_payload(o,1);}',
            'if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};(void)send_payload_critical(o,1);}',
        ),
        (
            'case C_GET_APPCONF:case C_GET_APPCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeApp(o+1,r,cmd==C_GET_APPCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;',
            'case C_GET_APPCONF:case C_GET_APPCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeApp(o+1,r,cmd==C_GET_APPCONF_DEFAULT);if(k>0)(void)send_payload_critical(o,(uint16_t)(k+1));}break;',
        ),
        (
            'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){uint8_t o[1]={C_SET_APPCONF};send_payload(o,1);}}break;',
            'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){vescAppConfig.controller_id=1U;uint8_t o[1]={C_SET_APPCONF};(void)send_payload_critical(o,1);}}break;',
        ),
        (
            'case C_PING_CAN:{uint8_t o[2]={C_PING_CAN,second_id()};send_payload(o,2);}break;',
            'case C_PING_CAN:{uint8_t o[2]={C_PING_CAN,second_id()};(void)send_payload_critical(o,2);}break;',
        ),
    ]
    for old, new in replacements:
        if old in text:
            text = text.replace(old, new, 1)

    # If an earlier patch already converted APPCONF ACK but not fixed the ID, fix it.
    text = text.replace(
        'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){uint8_t o[1]={C_SET_APPCONF};(void)send_payload_critical(o,1);}}break;',
        'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){vescAppConfig.controller_id=1U;uint8_t o[1]={C_SET_APPCONF};(void)send_payload_critical(o,1);}}break;',
    )

    # Reset deferred reply state when DMA transport is reinitialized.
    if "critical_reply_valid=false;critical_reply_len=0U" not in text:
        text = text.replace(
            'tx_head=tx_tail=0;tx_busy=false;if(!protocol_initialized)',
            'tx_head=tx_tail=0;tx_busy=false;critical_reply_valid=false;critical_reply_len=0U;if(!protocol_initialized)',
            1,
        )

    return text


def add_boot_diagnostic(text: str) -> str:
    if "vescMainSchedulerPasses" not in text:
        marker = "extern volatile uint16_t runtimeBuzzerEventCount;\n"
        insertion = marker + (
            "extern volatile uint32_t vescMainSchedulerPasses;\n"
            "extern volatile uint32_t vescMainResetFlags;\n"
            "extern volatile uint32_t vescMainLastAliveMs;\n"
            "extern volatile uint32_t vescMainOptionalTaskTicks;\n"
            "extern volatile uint8_t vescMainBootFaultCode;\n"
            "extern volatile uint8_t vescMainBootPhase;\n"
            "extern volatile uint8_t vescMainDegradedMode;\n"
        )
        text = replace_once(text, marker, insertion, "main scheduler diagnostic externs")
    elif "extern volatile uint32_t vescMainOptionalTaskTicks;" not in text:
        marker = "extern volatile uint32_t vescMainLastAliveMs;\n"
        text = replace_once(
            text, marker, marker + "extern volatile uint32_t vescMainOptionalTaskTicks;\n",
            "optional task diagnostic extern",
        )

    if 'strcmp(c, "hb_boot_status")' not in text:
        marker = '    if (strcmp(c, "hb_help") == 0) {\n'
        block = r'''    if (strcmp(c, "hb_boot_status") == 0 || strcmp(c, "hb_sched") == 0) {
        char line[180];
        (void)snprintf(line, sizeof(line),
            "HB BOOT phase=%u degraded=%u fault=%u reset=0x%08lx main=%lu alive=%lu opt=%lu\n",
            (unsigned)vescMainBootPhase, (unsigned)vescMainDegradedMode,
            (unsigned)vescMainBootFaultCode, (unsigned long)vescMainResetFlags,
            (unsigned long)vescMainSchedulerPasses, (unsigned long)vescMainLastAliveMs,
            (unsigned long)vescMainOptionalTaskTicks);
        send_print(line);
        (void)snprintf(line, sizeof(line),
            "HB ISR max=%lu deadline=%lu overruns=%u cal=%u | UART rx=%lu crc=%lu txdrop=%lu yield=%lu\n",
            (unsigned long)motorControlIsrMaxCycles, (unsigned long)motorControlIsrDeadlineCycles,
            (unsigned)motorControlIsrOverrunCount, (unsigned)MotorControl_CurrentOffsetCalState(),
            (unsigned long)VescProtocol_RxPackets(), (unsigned long)VescProtocol_RxCrcErrors(),
            (unsigned long)VescProtocol_TxDrops(), (unsigned long)VescProtocol_RxBudgetYields());
        send_print(line);
        return;
    }
'''
        text = replace_once(text, marker, block + marker, "hb_boot_status terminal command")

    text = text.replace(
        'send_print("HB: hb_current_cal/status, hb_auto_detect, hb_encoder_sync, hb_home, hb_home_cal, hb_home_on 0|1, hb_home_status, hb_set_pole_pairs N\\n");',
        'send_print("HB: hb_boot_status, hb_current_cal/status, hb_auto_detect, hb_encoder_sync, hb_home, hb_home_cal, hb_home_on 0|1, hb_home_status, hb_set_pole_pairs N\\n");',
    )
    return text


SPLIT_SERVICE = r'''void VescProtocol_ServiceRxBudget(uint16_t max_rx_bytes)
{
    const uint32_t f = DMA1->ISR;

    /* TX DMA has a real IRQ, but polling the flags here makes recovery robust
     * even if an IRQ was briefly masked by a same-priority motor interrupt. */
    if (tx_busy && (f & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2))) {
        VescProtocol_TxDmaIrqHandler();
    }

    if ((f & DMA_ISR_TEIF3) || ((DMA1_Channel3->CCR & DMA_CCR_EN) == 0U)) {
        ++crc_or_parser_errors;
        RuntimeControl_VescReleaseAll();
        VescProtocol_Init();
        return;
    }

    if (max_rx_bytes == 0U) max_rx_bytes = VESC_RX_SERVICE_DEFAULT_BUDGET;
    if (max_rx_bytes > VESC_RX_SERVICE_MAX_BUDGET)
        max_rx_bytes = VESC_RX_SERVICE_MAX_BUDGET;

    /* VESC Tool handshake/config reply backpressure has priority. Never keep
     * consuming requests while a critical reply cannot enter the TX queue. */
    critical_reply_service();
    if (critical_reply_valid) {
        tx_start_next();
        return;
    }

    const uint16_t producer = (uint16_t)(RX_DMA_SIZE - DMA1_Channel3->CNDTR);
    uint16_t consumed = 0U;
    while (rx_old != producer && consumed < max_rx_bytes) {
        VescPacket_Feed(&parser, rx_dma[rx_old], rx_cb);
        ++rx_old;
        if (rx_old >= RX_DMA_SIZE) rx_old = 0U;
        ++consumed;

        critical_reply_service();
        if (critical_reply_valid) break;
    }

    if (rx_old != producer && rx_budget_yields != UINT32_MAX) {
        ++rx_budget_yields;
    }
    tx_start_next();
}

void VescProtocol_ServiceBlocking(void)
{
    /* Bare-metal equivalent of commands.c::blocking_thread. Detect/apply flows
     * remain asynchronous state machines so this call is always bounded. */
    critical_reply_service();
    detect_service();
    auto_detect_service();
    critical_reply_service();
    tx_start_next();
}

void VescProtocol_ServiceSampleSend(void)
{
    /* Deferred reply/sample TX worker. Heavy packet send never runs in motor ISR. */
    critical_reply_service();
    tx_start_next();
}

void VescProtocol_ServicePeriodic(void)
{
    /* main.c::periodic_thread semantic: rotor/encoder/position display stream. */
    rotor_position_stream_service();
    critical_reply_service();
    tx_start_next();
}

void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)
{
    /* Compatibility API from early V22. New main uses ServiceRxBudget directly. */
    VescProtocol_ServiceRxBudget(max_rx_bytes);
}

void VescProtocol_Service(void)
{
    /* Compatibility all-in-one wrapper for legacy tests/tools only. */
    VescProtocol_ServiceRxBudget(VESC_RX_SERVICE_DEFAULT_BUDGET);
    VescProtocol_ServiceBlocking();
    VescProtocol_ServiceSampleSend();
    VescProtocol_ServicePeriodic();
}
'''


def split_protocol_service(text: str) -> str:
    # Idempotency: once the four explicit task services exist, never stack a
    # second split block on top of the compatibility ServiceBudget wrapper.
    if ("void VescProtocol_ServiceRxBudget(uint16_t max_rx_bytes)" in text and
        "void VescProtocol_ServiceBlocking(void)" in text and
        "void VescProtocol_ServiceSampleSend(void)" in text and
        "void VescProtocol_ServicePeriodic(void)" in text):
        return text

    # Replace everything from either old ServiceBudget or old monolithic Service
    # up to (but not including) VescProtocol_AdcSetNormalized.
    patterns = [
        r"void VescProtocol_ServiceBudget\(uint16_t max_rx_bytes\).*?(?=void VescProtocol_AdcSetNormalized)",
        r"void VescProtocol_Service\(void\)\s*\{.*?(?=void VescProtocol_AdcSetNormalized)",
    ]
    for pattern in patterns:
        m = re.search(pattern, text, flags=re.S)
        if m:
            return text[:m.start()] + SPLIT_SERVICE + text[m.end():]
    raise RuntimeError("Could not locate VescProtocol service implementation to split")


def patch_protocol(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text = ensure_protocol_helpers(text)
    text = make_critical_replies(text)
    text = add_boot_diagnostic(text)
    text = split_protocol_service(text)

    required = [
        "void VescProtocol_ServiceRxBudget(uint16_t max_rx_bytes)",
        "void VescProtocol_ServiceBlocking(void)",
        "void VescProtocol_ServiceSampleSend(void)",
        "void VescProtocol_ServicePeriodic(void)",
        "uint32_t VescProtocol_RxBudgetYields(void)",
        "critical_reply_service",
        "hb_boot_status",
    ]
    missing = [x for x in required if x not in text]
    if missing:
        raise RuntimeError("protocol patch incomplete: " + ", ".join(missing))
    path.write_text(text, encoding="utf-8")


def copy_replacements(root: Path, package: Path) -> None:
    for name in ["main.c", "vesc_protocol.h", "vesc_services.c", "vesc_services.h"]:
        shutil.copy2(package / "src" / name, root / "src" / name)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path, help="Complete local hoverboard_vesc source tree")
    args = ap.parse_args()
    root = args.root.resolve()
    package = Path(__file__).resolve().parents[1]

    required = [
        "src/main.c", "src/vesc_protocol.c", "src/vesc_protocol.h",
        "src/vesc_services.c", "src/vesc_services.h", "src/motor.c",
        "src/stm32f1xx_it.c", "src/runtime_control.c", "src/vesc_app.c",
        "src/left_encoder.c", "platformio.ini",
    ]
    missing = [p for p in required if not (root / p).exists()]
    if missing:
        raise SystemExit("Not a complete target tree; missing: " + ", ".join(missing))

    protected_before = {p: sha256(root / p) for p in PROTECTED_ISR}

    backup = root / ".v22_deep_backup"
    backup.mkdir(exist_ok=True)
    for rel in EDITED:
        src = root / rel
        if src.exists():
            dst = backup / rel
            dst.parent.mkdir(parents=True, exist_ok=True)
            if not dst.exists():
                shutil.copy2(src, dst)

    print("[1/4] patch protocol split/backpressure/debug")
    patch_protocol(root / "src/vesc_protocol.c")
    print("[2/4] install explicit main scheduler + protocol header")
    copy_replacements(root, package)

    print("[3/4] install regression tests/debug runner")
    tests = root / "tests"
    tests.mkdir(exist_ok=True)
    for src in sorted((package / "tests").glob("test_v22_deep_*.py")):
        shutil.copy2(src, tests / src.name)
    tools = root / "tools"
    tools.mkdir(exist_ok=True)
    shutil.copy2(package / "tools" / "v22_deep_debug.py", tools / "v22_deep_debug.py")
    shutil.copy2(package / "tools" / "apply_v22_deep_main_scheduler.py", tools / "apply_v22_deep_main_scheduler.py")
    for doc_name in ["AUDIT_V22_DEEP_MAIN_ID.md", "VESC_TASK_MAPPING_MAIN_ID.md"]:
        src = package / doc_name
        if src.exists():
            shutil.copy2(src, root / doc_name)

    protected_after = {p: sha256(root / p) for p in PROTECTED_ISR}
    if protected_before != protected_after:
        raise SystemExit("FATAL: protected ISR source changed; restoring is required")

    marker = root / "V22_DEEP_MAIN_SCHEDULER_APPLIED.txt"
    marker.write_text(
        "V22 deep main-owned cooperative scheduler applied.\n"
        "Protected ISR files were SHA256-identical before/after.\n"
        "Run: python3 tests/test_v22_deep_main_scheduler.py\n"
        "Then: pio run\n",
        encoding="utf-8",
    )
    print("[4/4] PASS protected ISR hashes unchanged")
    print(f"Applied successfully: {root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
