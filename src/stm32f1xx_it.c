#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "util.h"
#include "defines.h"
#include "runtime_control.h"
#include "vesc_protocol.h"
#include "left_encoder.h"


/**
 * Mematikan Main Output Enable TIM1/TIM8 secepat mungkin pada fault CPU.
 * Ini memaksa PWM high-side dan complementary low-side kembali ke idle state
 * yang telah dikunci di setup.c (high OFF=LOW, low OFF=HIGH).
 */
static void EmergencyPwmOff(void)
{
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
}

#define SYSTEM_FAULT_MAGIC 0x45534346UL /* "ESCF" */
typedef struct {
    uint32_t magic;
    uint32_t code;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t bfar;
    uint32_t mmfar;
} SystemFaultRecord;

/* NOLOAD: bertahan melewati NVIC_SystemReset, tetapi tidak memakai flash/EEPROM
 * dari fault context. Pada power-cycle acak hanya dianggap valid bila magic cocok. */
static volatile SystemFaultRecord systemFaultRecord __attribute__((section(".noinit")));

static void SystemFault_Reset(uint32_t code)
{
    __disable_irq();
    EmergencyPwmOff();
    systemFaultRecord.magic = SYSTEM_FAULT_MAGIC;
    systemFaultRecord.code = code;
    systemFaultRecord.cfsr = SCB->CFSR;
    systemFaultRecord.hfsr = SCB->HFSR;
    systemFaultRecord.bfar = SCB->BFAR;
    systemFaultRecord.mmfar = SCB->MMFAR;
    __DSB();
    NVIC_SystemReset();
    while (1) { }
}

uint8_t SystemFault_GetBootCode(void)
{
    if (systemFaultRecord.magic != SYSTEM_FAULT_MAGIC) return 0U;
    return (systemFaultRecord.code <= 255U) ? (uint8_t)systemFaultRecord.code : 0U;
}

uint32_t SystemFault_GetBootCfsr(void)
{
    return (systemFaultRecord.magic == SYSTEM_FAULT_MAGIC) ? systemFaultRecord.cfsr : 0U;
}

void SystemFault_AcknowledgeBootRecord(void)
{
    systemFaultRecord.magic = 0U;
}

/** CPU fault: PWM OFF dahulu, simpan record SRAM, lalu reset agar UART tidak mati permanen. */
void NMI_Handler(void) { SystemFault_Reset(1U); }
void HardFault_Handler(void) { SystemFault_Reset(2U); }
void MemManage_Handler(void) { SystemFault_Reset(3U); }
void BusFault_Handler(void) { SystemFault_Reset(4U); }
void UsageFault_Handler(void) { SystemFault_Reset(5U); }
/** Handler SVC standar Cortex-M3. */
void SVC_Handler(void) {}
/** Handler debug monitor standar Cortex-M3. */
void DebugMon_Handler(void) {}
/** Handler PendSV standar Cortex-M3. */
void PendSV_Handler(void) {}

/** Menambah tick HAL 1 ms dan meneruskan event SysTick ke HAL. */
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

/** DMA1 channel 2 adalah TX USART3. Completion direct tanpa HAL UART state. */
void DMA1_Channel2_IRQHandler(void)
{
    VescProtocol_TxDmaIrqHandler();
}


/** LEFT encoder index Z uses PB5/EXTI5. A/B are decoded completely in TIM4
 * hardware, so no high-rate quadrature EXTI is needed. */
void EXTI9_5_IRQHandler(void)
{
    if ((EXTI->PR & EXTI_PR_PR5) != 0U) {
        EXTI->PR = EXTI_PR_PR5;
        LeftEncoder_IndexIrq();
    }
}

/** RX USART3 memakai circular DMA polling; IRQ ini seharusnya disabled. */
void DMA1_Channel3_IRQHandler(void)
{
    DMA1->IFCR = DMA_IFCR_CGIF3;
}

/**
 * USART3 IRQ is not part of the normal VESC transport. RX is circular DMA and
 * parsed from main context. Keep this defensive handler only to drain stale
 * status/data if an IRQ is left enabled by startup/HAL code.
 */
void USART3_IRQHandler(void)
{
    /* RX DMA remains the single source of truth. */
    volatile uint32_t sr = USART3->SR;
    volatile uint32_t dr = USART3->DR;
    (void)sr; (void)dr;
}

