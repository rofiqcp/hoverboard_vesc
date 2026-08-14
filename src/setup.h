#ifndef SETUP_H
#define SETUP_H

#include "stm32f1xx_hal.h"

/** Inisialisasi GPIO board variant 0, termasuk PA2/PA3 sebagai input analog. */
void MX_GPIO_Init(void);
/** Inisialisasi TIM1/TIM8 untuk PWM inverter tiga-fasa motor. */
void MX_TIM_Init(void);
/** Inisialisasi ADC1 pada mode dual ADC. */
void MX_ADC1_Init(void);
/** Inisialisasi ADC2; PA2 dan PA3 tetap menjadi channel ADC2. */
void MX_ADC2_Init(void);
/** Trigger injected ADC1 housekeeping + ADC2 APP_ADC immediately after a completed current sample. */
void ADC_Slow_TriggerFromCurrentISR(void);
/** Commit completed injected ADC results to adc_buffer from main context. */
void ADC_Slow_Service(uint32_t now_ms);
/** Inisialisasi satu-satunya port komunikasi: USART3 PB10/PB11 dengan DMA. */
void UART3_Init(void);

#endif /* SETUP_H */
