#ifndef VESC_PROTOCOL_H
#define VESC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

void VescProtocol_Init(void);

/* Compatibility wrapper using the default bounded RX work budget. */
void VescProtocol_Service(void);

/*
 * Cooperative packet_process_thread equivalent. At most max_rx_bytes are fed
 * into the parser per call, so UART bursts cannot starve fault/PID/app services.
 */
void VescProtocol_ServiceBudget(uint16_t max_rx_bytes);

/* Sample VESC-style current averages from the ~200-Hz background loop. */
void VescProtocol_CurrentTelemetrySample(void);
void VescProtocol_TxDmaIrqHandler(void);
void VescProtocol_AdcSetNormalized(int16_t permille, bool speed_mode);

uint8_t VescProtocol_LocalCanId(void);
uint8_t VescProtocol_RightVirtualCanId(void);
uint32_t VescProtocol_RxPackets(void);
uint32_t VescProtocol_RxCrcErrors(void);
uint32_t VescProtocol_TxDrops(void);
uint32_t VescProtocol_RxBudgetYields(void);

#endif /* VESC_PROTOCOL_H */
