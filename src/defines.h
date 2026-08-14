#ifndef DEFINES_H
#define DEFINES_H

#include "stm32f1xx_hal.h"
#include "config.h"
#include <stdint.h>

/* ========================= HALL SENSOR ========================= */
#define LEFT_HALL_U_PIN          GPIO_PIN_5
#define LEFT_HALL_V_PIN          GPIO_PIN_6
#define LEFT_HALL_W_PIN          GPIO_PIN_7
#define LEFT_HALL_U_PORT         GPIOB
#define LEFT_HALL_V_PORT         GPIOB
#define LEFT_HALL_W_PORT         GPIOB

#define RIGHT_HALL_U_PIN         GPIO_PIN_10
#define RIGHT_HALL_V_PIN         GPIO_PIN_11
#define RIGHT_HALL_W_PIN         GPIO_PIN_12
#define RIGHT_HALL_U_PORT        GPIOC
#define RIGHT_HALL_V_PORT        GPIOC
#define RIGHT_HALL_W_PORT        GPIOC

/* ======================== INVERTER MOTOR ======================= */
#define LEFT_TIM                 TIM8
#define LEFT_TIM_U               CCR1
#define LEFT_TIM_UH_PIN          GPIO_PIN_6
#define LEFT_TIM_UH_PORT         GPIOC
#define LEFT_TIM_UL_PIN          GPIO_PIN_7
#define LEFT_TIM_UL_PORT         GPIOA
#define LEFT_TIM_V               CCR2
#define LEFT_TIM_VH_PIN          GPIO_PIN_7
#define LEFT_TIM_VH_PORT         GPIOC
#define LEFT_TIM_VL_PIN          GPIO_PIN_0
#define LEFT_TIM_VL_PORT         GPIOB
#define LEFT_TIM_W               CCR3
#define LEFT_TIM_WH_PIN          GPIO_PIN_8
#define LEFT_TIM_WH_PORT         GPIOC
#define LEFT_TIM_WL_PIN          GPIO_PIN_1
#define LEFT_TIM_WL_PORT         GPIOB

#define RIGHT_TIM                TIM1
#define RIGHT_TIM_U              CCR1
#define RIGHT_TIM_UH_PIN         GPIO_PIN_8
#define RIGHT_TIM_UH_PORT        GPIOA
#define RIGHT_TIM_UL_PIN         GPIO_PIN_13
#define RIGHT_TIM_UL_PORT        GPIOB
#define RIGHT_TIM_V              CCR2
#define RIGHT_TIM_VH_PIN         GPIO_PIN_9
#define RIGHT_TIM_VH_PORT        GPIOA
#define RIGHT_TIM_VL_PIN         GPIO_PIN_14
#define RIGHT_TIM_VL_PORT        GPIOB
#define RIGHT_TIM_W              CCR3
#define RIGHT_TIM_WH_PIN         GPIO_PIN_10
#define RIGHT_TIM_WH_PORT        GPIOA
#define RIGHT_TIM_WL_PIN         GPIO_PIN_15
#define RIGHT_TIM_WL_PORT        GPIOB

/* ======================= PENGUKURAN ARUS ======================= */
#define LEFT_DC_CUR_PIN          GPIO_PIN_0
#define LEFT_U_CUR_PIN           GPIO_PIN_0
#define LEFT_V_CUR_PIN           GPIO_PIN_3
#define LEFT_DC_CUR_PORT         GPIOC
#define LEFT_U_CUR_PORT          GPIOA
#define LEFT_V_CUR_PORT          GPIOC

#define RIGHT_DC_CUR_PIN         GPIO_PIN_1
#define RIGHT_U_CUR_PIN          GPIO_PIN_4
#define RIGHT_V_CUR_PIN          GPIO_PIN_5
#define RIGHT_DC_CUR_PORT        GPIOC
#define RIGHT_U_CUR_PORT         GPIOC
#define RIGHT_V_CUR_PORT         GPIOC

#define DCLINK_PIN               GPIO_PIN_2
#define DCLINK_PORT              GPIOC

/* ======================= IO BOARD UTAMA ======================== */
#define LED_PIN                  GPIO_PIN_2
#define LED_PORT                 GPIOB
#define BUZZER_PIN               GPIO_PIN_4
#define BUZZER_PORT              GPIOA
#define OFF_PIN                  GPIO_PIN_5
#define OFF_PORT                 GPIOA
#define BUTTON_PIN               GPIO_PIN_1
#define BUTTON_PORT              GPIOA
#define CHARGER_PIN              GPIO_PIN_12
#define CHARGER_PORT             GPIOA

#define DELAY_TIM_FREQUENCY_US   1000000

/* =========================== UTILITAS ========================== */
#define NO                       0
#define YES                      1
#define ABS(a)                   (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh)        (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-(lowhigh))) ? (-(lowhigh)) : (x)))
#define CLAMP(x, low, high)      (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high)   (((x) >= (low)) && ((x) <= (high)))
#define MIN(a, b)                (((a) < (b)) ? (a) : (b))
#define MAX(a, b)                (((a) > (b)) ? (a) : (b))
#define MIN3(a, b, c)            MIN((a), MIN((b), (c)))
#define MAX3(a, b, c)            MAX((a), MAX((b), (c)))
#define ARRAY_LEN(x)             ((uint32_t)(sizeof(x) / sizeof(*(x))))
#define MAP(x, in_min, in_max, out_min, out_max) \
    (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))

/*
 * Buffer ADC dual-mode.
 * pa2Analog dan pa3Analog adalah nama historis dari firmware asli. Pada firmware
 * bersih ini keduanya adalah ADC PA2 dan PA3, tetap murni sebagai kanal analog.
 */
typedef struct {
    uint16_t dcr;
    uint16_t dcl;
    uint16_t rlA;
    uint16_t rlB;
    uint16_t rrB;
    uint16_t rrC;
    uint16_t batt1;
    uint16_t pa2Analog;   /* PA2 / ADC2 channel 2 */
    uint16_t temp;
    uint16_t pa3Analog;   /* PA3 / ADC2 channel 3 */
} adc_buf_t;

#endif /* DEFINES_H */
