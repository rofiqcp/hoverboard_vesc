#ifndef VESC_PROTOCOL_H
#define VESC_PROTOCOL_H
#include <stdint.h>
#include <stdbool.h>
void VescProtocol_Init(void);
void VescProtocol_Service(void);
/* Sample VESC-style current averages from the 200-Hz background loop. */
void VescProtocol_CurrentTelemetrySample(void);
void VescProtocol_TxDmaIrqHandler(void);
void VescProtocol_AdcSetNormalized(int16_t permille, bool speed_mode);
uint8_t VescProtocol_LocalCanId(void);
uint8_t VescProtocol_RightVirtualCanId(void);
uint32_t VescProtocol_RxPackets(void);
uint32_t VescProtocol_RxCrcErrors(void);
uint32_t VescProtocol_TxDrops(void);
#endif
