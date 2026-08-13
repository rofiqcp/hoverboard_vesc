/*
 * LEFT encoder backend.
 *
 * Hall mode:
 *   PB5 = Hall U, PB6 = Hall V, PB7 = Hall W.
 *
 * Encoder mode:
 *   PB6 = TIM4_CH1 / A
 *   PB7 = TIM4_CH2 / B
 *   PB5 = Z index / EXTI5
 *
 * TIM4 performs hardware quadrature decoding. The fast FOC loop only snapshots
 * the signed accumulated count; it never services one interrupt per A/B edge.
 */
#include "left_encoder.h"
#include "stm32f1xx_hal.h"
#include "defines.h"
#include <limits.h>

static TIM_HandleTypeDef htim4_encoder;
static volatile bool enabled = false;
static volatile bool index_seen = false;
static volatile int32_t index_count = 0;
static volatile int32_t accumulated = 0;
/* previous_cnt is intentionally owned only by LeftEncoder_UpdateAndGetCount(),
 * which is called from the 16-kHz DMA ISR. */
static uint16_t previous_cnt = 0;

static void pins_hall(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);

    EXTI->IMR &= ~EXTI_IMR_MR5;
}

static void pins_encoder(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void LeftEncoder_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    pins_hall();
    enabled = false;
    index_seen = false;
    index_count = 0;
    accumulated = 0;
    previous_cnt = 0;
}

void LeftEncoder_SetMode(bool on)
{
    if (on == enabled) return;

    if (on) {
        TIM_Encoder_InitTypeDef encoder = {0};
        pins_encoder();

        htim4_encoder.Instance = TIM4;
        htim4_encoder.Init.Prescaler = 0U;
        htim4_encoder.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim4_encoder.Init.Period = 0xFFFFU;
        htim4_encoder.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        encoder.EncoderMode = TIM_ENCODERMODE_TI12;
        encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
        encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
        encoder.IC1Prescaler = TIM_ICPSC_DIV1;
        encoder.IC1Filter = 4U;
        encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
        encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
        encoder.IC2Prescaler = TIM_ICPSC_DIV1;
        encoder.IC2Filter = 4U;

        (void)HAL_TIM_Encoder_Init(&htim4_encoder, &encoder);
        __HAL_TIM_SET_COUNTER(&htim4_encoder, 0U);
        previous_cnt = 0U;
        accumulated = 0;
        index_seen = false;
        index_count = 0;
        (void)HAL_TIM_Encoder_Start(&htim4_encoder, TIM_CHANNEL_ALL);
        enabled = true;
        return;
    }

    (void)HAL_TIM_Encoder_Stop(&htim4_encoder, TIM_CHANNEL_ALL);
    TIM4->CR1 &= ~TIM_CR1_CEN;
    enabled = false;
    pins_hall();
}

bool LeftEncoder_IsEnabled(void)
{
    return enabled;
}

int32_t LeftEncoder_UpdateAndGetCount(void)
{
    if (!enabled) return accumulated;

    const uint16_t now = (uint16_t)TIM4->CNT;
    const int16_t delta = (int16_t)(now - previous_cnt);
    previous_cnt = now;

    if (delta > 0 && accumulated > INT32_MAX - delta) {
        accumulated = INT32_MAX;
    } else if (delta < 0 && accumulated < INT32_MIN - delta) {
        accumulated = INT32_MIN;
    } else {
        accumulated += delta;
    }
    return accumulated;
}

int32_t LeftEncoder_GetCount(void)
{
    /* Cortex-M3 aligned 32-bit read is atomic. Do NOT touch TIM4/previous_cnt
     * here: slow-loop detect/alignment used to race the 16-kHz ISR and could
     * duplicate/drop encoder deltas, corrupting the electrical phase proof. */
    return accumulated;
}

void LeftEncoder_ZeroMechanical(void)
{
    accumulated = 0;
    previous_cnt = (uint16_t)TIM4->CNT;
}

void LeftEncoder_IndexIrq(void)
{
    if (!enabled) return;
    index_count = LeftEncoder_GetCount();
    index_seen = true;
}

bool LeftEncoder_IndexSeen(void)
{
    return index_seen;
}

int32_t LeftEncoder_GetIndexCount(void)
{
    return index_count;
}
