#ifndef STM32F1XX_HAL_H
#define STM32F1XX_HAL_H
#include <stdint.h>
#include <stdbool.h>

typedef struct { volatile uint32_t IDR; } GPIO_TypeDef;
typedef struct { volatile uint32_t CR1, CNT; } TIM_TypeDef;
typedef struct { volatile uint32_t IMR, PR; } EXTI_TypeDef;

typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
} TIM_Base_InitTypeDef;

typedef struct {
    TIM_TypeDef *Instance;
    TIM_Base_InitTypeDef Init;
} TIM_HandleTypeDef;

typedef struct {
    uint32_t EncoderMode;
    uint32_t IC1Polarity;
    uint32_t IC1Selection;
    uint32_t IC1Prescaler;
    uint32_t IC1Filter;
    uint32_t IC2Polarity;
    uint32_t IC2Selection;
    uint32_t IC2Prescaler;
    uint32_t IC2Filter;
} TIM_Encoder_InitTypeDef;

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

extern GPIO_TypeDef _GPIOA, _GPIOB, _GPIOC;
extern TIM_TypeDef _TIM1, _TIM4, _TIM8;
extern EXTI_TypeDef _EXTI;
#define GPIOA (&_GPIOA)
#define GPIOB (&_GPIOB)
#define GPIOC (&_GPIOC)
#define TIM1 (&_TIM1)
#define TIM4 (&_TIM4)
#define TIM8 (&_TIM8)
#define EXTI (&_EXTI)

#define GPIO_PIN_0  (1U<<0)
#define GPIO_PIN_1  (1U<<1)
#define GPIO_PIN_2  (1U<<2)
#define GPIO_PIN_3  (1U<<3)
#define GPIO_PIN_4  (1U<<4)
#define GPIO_PIN_5  (1U<<5)
#define GPIO_PIN_6  (1U<<6)
#define GPIO_PIN_7  (1U<<7)
#define GPIO_PIN_8  (1U<<8)
#define GPIO_PIN_9  (1U<<9)
#define GPIO_PIN_10 (1U<<10)
#define GPIO_PIN_11 (1U<<11)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_PIN_13 (1U<<13)
#define GPIO_PIN_14 (1U<<14)
#define GPIO_PIN_15 (1U<<15)

#define GPIO_MODE_INPUT 0U
#define GPIO_MODE_IT_FALLING 1U
#define GPIO_PULLUP 1U
#define GPIO_SPEED_FREQ_HIGH 3U
#define EXTI_IMR_MR5 (1U<<5)
#define TIM_COUNTERMODE_UP 0U
#define TIM_CLOCKDIVISION_DIV1 0U
#define TIM_ENCODERMODE_TI12 3U
#define TIM_ICPOLARITY_RISING 0U
#define TIM_ICSELECTION_DIRECTTI 1U
#define TIM_ICPSC_DIV1 0U
#define TIM_CHANNEL_ALL 0x3FU
#define TIM_CR1_CEN 1U
#define EXTI9_5_IRQn 23

#define __HAL_RCC_TIM4_CLK_ENABLE() ((void)0)
#define __HAL_TIM_SET_COUNTER(h, v) ((h)->Instance->CNT = (v))

static inline void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *cfg) {(void)port;(void)cfg;}
static inline void HAL_NVIC_SetPriority(int irq, uint32_t p, uint32_t s) {(void)irq;(void)p;(void)s;}
static inline void HAL_NVIC_EnableIRQ(int irq) {(void)irq;}
static inline int HAL_TIM_Encoder_Init(TIM_HandleTypeDef *h, TIM_Encoder_InitTypeDef *e) {(void)h;(void)e;return 0;}
static inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *h, uint32_t c) {(void)h;(void)c;return 0;}
static inline int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *h, uint32_t c) {(void)h;(void)c;return 0;}

#endif
