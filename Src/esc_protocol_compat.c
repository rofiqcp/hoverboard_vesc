/*
 * Minimal compatibility shim for the retired custom 64-byte UART protocol.
 *
 * runtime_control.c still reuses EscCommandFrame types and the historical
 * CRC16-CCITT-FALSE routine for EEPROM/config integrity. The actual UART
 * transport is VESC on USART3, therefore the legacy RX/TX engine must not be
 * linked: keeping it would waste RAM/flash and duplicate DMA ownership.
 */
#include "esc_protocol.h"
#include "vesc_protocol.h"
#include <stddef.h>

uint16_t EscProtocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL) return crc;
    for (uint16_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t)((crc << 1) ^ 0x1021U)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void EscProtocol_StartRx(void) { }
void EscProtocol_ServiceIo(void) { }
void EscProtocol_ServiceCommands(void) { }
uint8_t EscProtocol_ServiceCommandsBudget(uint8_t max_frames)
{
    (void)max_frames;
    return 0U;
}

/* Legacy telemetry is deliberately disabled. VESCProtocol owns USART3. */
bool EscProtocol_TxReady(void) { return false; }
bool EscProtocol_SendFeedback(const EscFeedbackFrame *frame)
{
    (void)frame;
    return false;
}
void EscProtocol_TxDmaIrqHandler(void) { }

/* Old diagnostics are mapped to the active VESC transport where meaningful. */
uint32_t EscProtocol_GetValidFrameCount(void) { return VescProtocol_RxPackets(); }
uint32_t EscProtocol_GetBadFrameCount(void) { return VescProtocol_RxCrcErrors(); }
uint32_t EscProtocol_GetTxDmaErrorCount(void) { return VescProtocol_TxDrops(); }
uint32_t EscProtocol_GetRxDmaRecoveryCount(void) { return VescProtocol_RxCrcErrors(); }
