#!/usr/bin/env python3
"""Apply V22 non-RTOS cooperative-service changes to hoverboard_vesc v21.

The script is intentionally fail-closed: every baseline marker must match exactly.
If the target source has drifted, it aborts instead of partially patching firmware.
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path

BASE_COMMIT = "d9ffe0ae69b5d5421d605e9fe2f8931403a4ba2c"


def replace_once(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"{label}: expected exactly one baseline match, got {count}")
    return text.replace(old, new, 1)


def patch_protocol_c(path: Path) -> None:
    text = path.read_text(encoding="utf-8")

    have_budget = "void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)" in text
    have_yields = "uint32_t VescProtocol_RxBudgetYields(void)" in text
    if have_budget and have_yields:
        print("[V22] vesc_protocol.c already patched; keeping implementation")
        return
    if have_budget != have_yields:
        raise RuntimeError("vesc_protocol.c is partially patched; refusing to stack another patch")

    text = replace_once(
        text,
        "#define VESC_AUTO_DETECT_CURRENT_A 1.0f\n",
        "#define VESC_AUTO_DETECT_CURRENT_A 1.0f\n"
        "#define VESC_RX_SERVICE_DEFAULT_BUDGET 160U\n"
        "#define VESC_RX_SERVICE_MAX_BUDGET 512U\n",
        "protocol budget defines",
    )

    text = replace_once(
        text,
        "static uint32_t rx_packets=0,crc_or_parser_errors=0,tx_drops=0,last_uart_control_ms=0;\n",
        "static uint32_t rx_packets=0,crc_or_parser_errors=0,tx_drops=0,last_uart_control_ms=0;\n"
        "static uint32_t rx_budget_yields=0U;\n",
        "protocol budget counter",
    )

    text = replace_once(
        text,
        "static volatile uint8_t tx_head=0,tx_tail=0;\nstatic volatile bool tx_busy=false;\n",
        "static volatile uint8_t tx_head=0,tx_tail=0;\n"
        "static volatile bool tx_busy=false;\n"
        "/* Critical VESC Tool request/reply backpressure slot. */\n"
        "static uint8_t critical_reply[VESC_PACKET_MAX_PAYLOAD];\n"
        "static uint16_t critical_reply_len=0U;\n"
        "static bool critical_reply_valid=false;\n",
        "critical reply storage",
    )

    text = replace_once(
        text,
        "uint32_t VescProtocol_TxDrops(void){return tx_drops;}\n",
        "uint32_t VescProtocol_TxDrops(void){return tx_drops;}\n"
        "uint32_t VescProtocol_RxBudgetYields(void){return rx_budget_yields;}\n",
        "protocol budget getter",
    )

    # Insert a reliable one-slot reply path for handshake/config transactions.
    helper_marker = """static bool send_payload_terminal(const uint8_t*p,uint16_t len){return send_payload_internal(p,len,false);}
"""
    helper_new = helper_marker + """static void critical_reply_service(void)
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
"""
    text = replace_once(text, helper_marker, helper_new, "critical reply helpers")

    text = replace_once(
        text,
        '"hoverboard-vesc6-v24"',
        '"hoverboard-vesc6-v22"',
        "firmware build string",
    )

    text = replace_once(
        text,
        'if(!fw_append_cstr(o,sizeof(o),&i,"hoverboard-vesc6-v22")) return;\n    send_payload(o,(uint16_t)i);',
        'if(!fw_append_cstr(o,sizeof(o),&i,"hoverboard-vesc6-v22")) return;\n    (void)send_payload_critical(o,(uint16_t)i);',
        "critical FW_VERSION reply",
    )

    critical_replacements = [
        (
            'case C_GET_DECODED_ADC:{uint8_t o[17];int32_t k=0;o[k++]=C_GET_DECODED_ADC;vesc_buf_append_i32(o,VescApp_GetDecoded1Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage1MicroV(),&k);vesc_buf_append_i32(o,VescApp_GetDecoded2Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage2MicroV(),&k);send_payload(o,(uint16_t)k);}break;',
            'case C_GET_DECODED_ADC:{uint8_t o[17];int32_t k=0;o[k++]=C_GET_DECODED_ADC;vesc_buf_append_i32(o,VescApp_GetDecoded1Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage1MicroV(),&k);vesc_buf_append_i32(o,VescApp_GetDecoded2Micro(),&k);vesc_buf_append_i32(o,VescApp_GetVoltage2MicroV(),&k);(void)send_payload_critical(o,(uint16_t)k);}break;',
            "decoded ADC critical reply",
        ),
        (
            'case C_GET_MCCONF:case C_GET_MCCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeMc(o+1,r,cmd==C_GET_MCCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;',
            'case C_GET_MCCONF:case C_GET_MCCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeMc(o+1,r,cmd==C_GET_MCCONF_DEFAULT);if(k>0)(void)send_payload_critical(o,(uint16_t)(k+1));}break;',
            "mcconf critical reply",
        ),
        (
            'if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};send_payload(o,1);}',
            'if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};(void)send_payload_critical(o,1);}',
            "mcconf ACK critical reply",
        ),
        (
            'case C_GET_APPCONF:case C_GET_APPCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeApp(o+1,r,cmd==C_GET_APPCONF_DEFAULT);if(k>0)send_payload(o,(uint16_t)(k+1));}break;',
            'case C_GET_APPCONF:case C_GET_APPCONF_DEFAULT:{uint8_t o[VESC_PACKET_MAX_PAYLOAD];o[0]=cmd;int32_t k=VescConfig_SerializeApp(o+1,r,cmd==C_GET_APPCONF_DEFAULT);if(k>0)(void)send_payload_critical(o,(uint16_t)(k+1));}break;',
            "appconf critical reply",
        ),
        (
            'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){uint8_t o[1]={C_SET_APPCONF};send_payload(o,1);}}break;',
            'case C_SET_APPCONF:{if(VescConfig_DeserializeApp(d,l,r,true)){vescAppConfig.controller_id=1U;uint8_t o[1]={C_SET_APPCONF};(void)send_payload_critical(o,1);}}break;',
            "appconf ACK critical reply + fixed local ID",
        ),
        (
            'case C_PING_CAN:{uint8_t o[2]={C_PING_CAN,second_id()};send_payload(o,2);}break;',
            'case C_PING_CAN:{uint8_t o[2]={C_PING_CAN,second_id()};(void)send_payload_critical(o,2);}break;',
            "PING_CAN critical reply",
        ),
    ]
    for old, new, label in critical_replacements:
        text = replace_once(text, old, new, label)

    # Reset deferred state whenever transport is reinitialized.
    text = replace_once(
        text,
        'tx_head=tx_tail=0;tx_busy=false;if(!protocol_initialized)',
        'tx_head=tx_tail=0;tx_busy=false;critical_reply_valid=false;critical_reply_len=0U;if(!protocol_initialized)',
        "critical reply reset",
    )

    old_service = '''void VescProtocol_Service(void){uint32_t f=DMA1->ISR;if(tx_busy&&(f&(DMA_ISR_TCIF2|DMA_ISR_TEIF2)))VescProtocol_TxDmaIrqHandler();if((f&DMA_ISR_TEIF3)||(DMA1_Channel3->CCR&DMA_CCR_EN)==0){
    ++crc_or_parser_errors;
    /* A broken UART/DMA path must fail torque OFF, not leave the last command
     * active until the normal application watchdog expires. */
    RuntimeControl_VescReleaseAll();
    VescProtocol_Init();
    return;
}detect_service();auto_detect_service();uint16_t pos=(uint16_t)(RX_DMA_SIZE-DMA1_Channel3->CNDTR);while(rx_old!=pos){VescPacket_Feed(&parser,rx_dma[rx_old],rx_cb);rx_old++;if(rx_old>=RX_DMA_SIZE)rx_old=0;}detect_service();auto_detect_service();rotor_position_stream_service();tx_start_next();}
'''

    new_service = '''void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)
{
    const uint32_t f = DMA1->ISR;

    if (tx_busy && (f & (DMA_ISR_TCIF2 | DMA_ISR_TEIF2))) {
        VescProtocol_TxDmaIrqHandler();
    }

    if ((f & DMA_ISR_TEIF3) || ((DMA1_Channel3->CCR & DMA_CCR_EN) == 0U)) {
        ++crc_or_parser_errors;

        /* UART/DMA failure is torque-off immediately, not watchdog-delayed. */
        RuntimeControl_VescReleaseAll();
        VescProtocol_Init();
        return;
    }

    if (max_rx_bytes == 0U) max_rx_bytes = VESC_RX_SERVICE_DEFAULT_BUDGET;
    if (max_rx_bytes > VESC_RX_SERVICE_MAX_BUDGET)
        max_rx_bytes = VESC_RX_SERVICE_MAX_BUDGET;

    /* A pending config/handshake reply has priority over accepting more work. */
    critical_reply_service();
    detect_service();
    auto_detect_service();
    if (critical_reply_valid) {
        tx_start_next();
        return;
    }

    /* Snapshot DMA producer position: newly arriving bytes wait for next pass. */
    const uint16_t pos = (uint16_t)(RX_DMA_SIZE - DMA1_Channel3->CNDTR);
    uint16_t consumed = 0U;

    while (rx_old != pos && consumed < max_rx_bytes) {
        VescPacket_Feed(&parser, rx_dma[rx_old], rx_cb);
        ++rx_old;
        if (rx_old >= RX_DMA_SIZE) rx_old = 0U;
        ++consumed;

        /* Stop consuming new requests when a critical reply is backpressured. */
        critical_reply_service();
        if (critical_reply_valid) break;
    }

    if (rx_old != pos && rx_budget_yields != UINT32_MAX) {
        ++rx_budget_yields;
    }

    detect_service();
    auto_detect_service();
    critical_reply_service();
    rotor_position_stream_service();
    tx_start_next();
}

void VescProtocol_Service(void)
{
    VescProtocol_ServiceBudget(VESC_RX_SERVICE_DEFAULT_BUDGET);
}
'''


    text = replace_once(text, old_service, new_service, "bounded protocol service")
    path.write_text(text, encoding="utf-8")


def install_replacements(target: Path, package_root: Path) -> None:
    replacement_root = package_root / "src"
    for src in replacement_root.iterdir():
        if src.is_file():
            shutil.copy2(src, target / "src" / src.name)


def apply(target: Path, package_root: Path) -> None:
    required = [
        "src/main.c",
        "src/vesc_protocol.c",
        "src/vesc_protocol.h",
        "src/vesc_app.c",
        "src/vesc_app.h",
        "src/runtime_control.c",
        "src/motor.c",
        "platformio.ini",
    ]
    missing = [p for p in required if not (target / p).exists()]
    if missing:
        raise RuntimeError("Not a complete hoverboard_vesc v21 tree; missing: " + ", ".join(missing))

    # Patch protocol BEFORE copying the replacement header.
    patch_protocol_c(target / "src" / "vesc_protocol.c")
    install_replacements(target, package_root)

    # Copy linker-contract test provided by this direct V22 pack.
    test_src = package_root / "tests" / "test_v22_contract.py"
    test_dst = target / "tests" / "test_v22_contract.py"
    test_dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(test_src, test_dst)

    marker = target / "V22_BASELINE.txt"
    marker.write_text(
        "V22 generated from rofiqcp/hoverboard_vesc v21\n"
        f"Pinned baseline commit: {BASE_COMMIT}\n"
        "Architecture: cooperative non-RTOS VESC services\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("target", type=Path, help="Path to extracted hoverboard_vesc v21 tree")
    args = parser.parse_args()

    package_root = Path(__file__).resolve().parents[1]
    target = args.target.resolve()
    apply(target, package_root)
    print(f"V22 applied successfully to: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
