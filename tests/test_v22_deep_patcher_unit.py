#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATCHER = ROOT / "tools" / "apply_v22_deep_main_scheduler.py"
spec = importlib.util.spec_from_file_location("deep_patcher", PATCHER)
assert spec and spec.loader
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)

OLD = r'''void VescProtocol_Service(void){uint32_t f=DMA1->ISR;if(tx_busy&&(f&(DMA_ISR_TCIF2|DMA_ISR_TEIF2)))VescProtocol_TxDmaIrqHandler();if((f&DMA_ISR_TEIF3)||(DMA1_Channel3->CCR&DMA_CCR_EN)==0){
 ++crc_or_parser_errors; RuntimeControl_VescReleaseAll(); VescProtocol_Init(); return;
}detect_service();auto_detect_service();uint16_t pos=(uint16_t)(RX_DMA_SIZE-DMA1_Channel3->CNDTR);while(rx_old!=pos){VescPacket_Feed(&parser,rx_dma[rx_old],rx_cb);rx_old++;if(rx_old>=RX_DMA_SIZE)rx_old=0;}detect_service();auto_detect_service();rotor_position_stream_service();tx_start_next();}
void VescProtocol_AdcSetNormalized(int16_t p,bool speed){}
'''

HOTFIX = r'''void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)
{
 const uint16_t pos=(uint16_t)(RX_DMA_SIZE-DMA1_Channel3->CNDTR);
 uint16_t consumed=0U;
 while(rx_old!=pos && consumed<max_rx_bytes){++consumed;}
 detect_service(); auto_detect_service(); rotor_position_stream_service();
}
void VescProtocol_Service(void){VescProtocol_ServiceBudget(160U);}
void VescProtocol_AdcSetNormalized(int16_t p,bool speed){}
'''


def check(src: str) -> str:
    out = mod.split_protocol_service(src)
    assert out.count("void VescProtocol_ServiceRxBudget(uint16_t max_rx_bytes)") == 1
    assert out.count("void VescProtocol_ServiceBlocking(void)") == 1
    assert out.count("void VescProtocol_ServiceSampleSend(void)") == 1
    assert out.count("void VescProtocol_ServicePeriodic(void)") == 1
    compact = "".join(out.split())
    assert "while(rx_old!=producer&&consumed<max_rx_bytes)" in compact
    rx = out[out.index("void VescProtocol_ServiceRxBudget"):out.index("void VescProtocol_ServiceBlocking")]
    assert "detect_service();" not in rx
    assert "auto_detect_service();" not in rx
    # Idempotent second transformation.
    out2 = mod.split_protocol_service(out)
    assert out2 == out
    return out


def main() -> None:
    check(OLD)
    check(HOTFIX)
    print("PASS: monolithic and previous-hotfix protocol services both split safely and idempotently")


if __name__ == "__main__":
    main()
