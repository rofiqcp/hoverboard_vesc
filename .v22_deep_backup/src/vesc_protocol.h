#ifndef VESC_PROTOCOL_H
#define VESC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

void VescProtocol_Init(void);

/* Compatibility all-in-one service. New main scheduler does not use this path. */
void VescProtocol_Service(void);

/* Upstream packet_process_thread semantic: bounded DMA-ring -> packet parser. */
void VescProtocol_ServiceRxBudget(uint16_t max_rx_bytes);

/* Compatibility name used by older V22 overlays/tests. */
void VescProtocol_ServiceBudget(uint16_t max_rx_bytes);

/* Upstream commands.c blocking_thread semantic: detect/apply state machines. */
void VescProtocol_ServiceBlocking(void);

/* Upstream sample_send/deferred TX semantic: flush critical reply / TX queue. */
void VescProtocol_ServiceSampleSend(void);

/* Upstream main periodic_thread semantic: rotor-position stream/housekeeping. */
void VescProtocol_ServicePeriodic(void);

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
