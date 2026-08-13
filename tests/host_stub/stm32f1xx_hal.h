#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H
#include <stdint.h>
#include <stdbool.h>

typedef struct { volatile uint32_t IDR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CCR1,CCR2,CCR3,BDTR; } TIM_TypeDef;
typedef struct { volatile uint32_t ISR,IFCR; } DMA_TypeDef;
typedef struct { volatile uint32_t CCR,CNDTR,CPAR,CMAR; } DMA_Channel_TypeDef;
typedef struct { int dummy; } TIM_HandleTypeDef;
typedef struct { int dummy; } ADC_HandleTypeDef;
typedef struct { volatile uint32_t CR1, CR2, CR3, SR, DR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef struct { int dummy; } DMA_HandleTypeDef;
typedef struct { volatile uint32_t CSR; } RCC_TypeDef;
extern RCC_TypeDef _RCC;
#define RCC (&_RCC)
#define RCC_CSR_PINRSTF (1U<<26)
#define RCC_CSR_PORRSTF (1U<<27)
#define RCC_CSR_SFTRSTF (1U<<28)
#define RCC_CSR_IWDGRSTF (1U<<29)
#define RCC_CSR_WWDGRSTF (1U<<30)
#define RCC_CSR_LPWRRSTF (1U<<31)
#define RCC_CSR_RMVF (1U<<24)
extern GPIO_TypeDef _GPIOA,_GPIOB,_GPIOC;
extern TIM_TypeDef _TIM1,_TIM8;
extern DMA_TypeDef _DMA1;
extern DMA_Channel_TypeDef _DMA1_Channel2,_DMA1_Channel3;
extern USART_TypeDef _USART3;
#define GPIOA (&_GPIOA)
#define GPIOB (&_GPIOB)
#define GPIOC (&_GPIOC)
#define TIM1 (&_TIM1)
#define TIM8 (&_TIM8)
#define DMA1 (&_DMA1)
#define DMA1_Channel2 (&_DMA1_Channel2)
#define DMA1_Channel3 (&_DMA1_Channel3)
#define USART3 (&_USART3)
#define GPIO_PIN_0 (1U<<0)
#define GPIO_PIN_1 (1U<<1)
#define GPIO_PIN_2 (1U<<2)
#define GPIO_PIN_3 (1U<<3)
#define GPIO_PIN_4 (1U<<4)
#define GPIO_PIN_5 (1U<<5)
#define GPIO_PIN_6 (1U<<6)
#define GPIO_PIN_7 (1U<<7)
#define GPIO_PIN_8 (1U<<8)
#define GPIO_PIN_9 (1U<<9)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_14 (1U<<14)
#define GPIO_PIN_15 (1U<<15)
#define TIM_BDTR_MOE (1U<<15)
#define DMA_ISR_TCIF1 (1U<<1)
#define DMA_IFCR_CTCIF1 (1U<<1)
#define DMA_IFCR_CGIF1 (1U<<0)
#define HAL_OK 0U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define RESET 0
#define SET 1
#define USART_CR1_PEIE (1U<<8)
#define USART_CR3_EIE (1U<<0)

#define DMA_CCR_EN (1U<<0)
#define DMA_CCR_TCIE (1U<<1)
#define DMA_CCR_TEIE (1U<<3)
#define DMA_CCR_DIR (1U<<4)
#define DMA_CCR_CIRC (1U<<5)
#define DMA_CCR_MINC (1U<<7)
#define DMA_ISR_TCIF2 (1U<<5)
#define DMA_ISR_TEIF2 (1U<<7)
#define DMA_ISR_TEIF3 (1U<<11)
#define DMA_IFCR_CGIF2 (1U<<4)
#define DMA_IFCR_CGIF3 (1U<<8)
#define USART_CR3_DMAR (1U<<6)
#define USART_CR3_DMAT (1U<<7)
#define SET_BIT(REG, BIT) ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
static inline uint32_t HAL_GetTick(void){return 0;}
static inline void HAL_Delay(uint32_t ms){(void)ms;}
static inline void HAL_FLASH_Unlock(void){}
static inline void HAL_FLASH_Lock(void){}
static inline uint32_t __get_PRIMASK(void){return 0;}
static inline void __disable_irq(void){}
static inline void __enable_irq(void){}
static inline void HAL_GPIO_TogglePin(GPIO_TypeDef *p,uint16_t pin){(void)p;(void)pin;}
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,int st){(void)p;(void)pin;(void)st;}
static inline int HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin){(void)p;(void)pin;return 1;}
#endif
