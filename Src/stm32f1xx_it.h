#ifndef STM32F1XX_IT_H
#define STM32F1XX_IT_H

#include <stdint.h>

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void USART3_IRQHandler(void);

/* Fault record warm-reset: 0=no fault-record, 1=NMI, 2=HardFault,
 * 3=MemManage, 4=BusFault, 5=UsageFault. */
uint8_t SystemFault_GetBootCode(void);
uint32_t SystemFault_GetBootCfsr(void);
void SystemFault_AcknowledgeBootRecord(void);

#endif /* STM32F1XX_IT_H */
