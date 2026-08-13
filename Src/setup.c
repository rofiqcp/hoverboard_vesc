/*
 * Inisialisasi hardware board variant 0.
 * Hanya USART3 yang digunakan untuk komunikasi. PA2 dan PA3 tidak pernah
 * dikonfigurasi sebagai UART; keduanya tetap menjadi kanal ADC2.
 */
#include "defines.h"
#include "config.h"
#include "setup.h"
#include <stdbool.h>
#include <string.h>

TIM_HandleTypeDef htim_right;
TIM_HandleTypeDef htim_left;
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
volatile adc_buf_t adc_buffer;
static volatile bool adcSlowPending = false;
static volatile bool adcAppPending = false;
static uint16_t adcHousekeepingDivider = 1600U; /* 10 Hz from 16 kHz current ISR */
static uint16_t adcAppDivider = 32U;            /* 500 Hz throttle/ADC app */

/**
 * Inisialisasi USART3 115200 baud dan interrupt DMA channel 2/3.
 * Dipanggil sekali dari SerialInput_Init(). USART3 memakai PB10(TX) dan PB11(RX).
 */
void UART3_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    /* DMA1_Channel1 (current-control ADC, prioritas 0), DMA1_Channel2 (UART TX
     * DMA) dan SysTick harus tetap SATU preemption priority yang sama (0,0).
     * NVIC tidak mempreempt antar-IRQ berprioritas sama -- itu artinya UART TX
     * DMA tidak akan pernah menyisip di tengah ISR motor DAN, jika motor ISR
     * baru selesai membuka window saat TX DMA IRQ sedang pending, keduanya
     * tetap berjalan sekuensial tanpa nested stacking tambahan. Memberi TX DMA
     * priority lebih rendah (mis. 1) pernah dicoba tetapi menyimpang dari
     * baseline hardware yang tervalidasi; jangan diulang tanpa pengujian ulang
     * di board fisik. */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel3_IRQn);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = USART3_BAUD;
    huart3.Init.WordLength = USART3_WORDLENGTH;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

/**
 * Menyiapkan GPIO, clock, DMA RX/TX, dan interrupt khusus USART3.
 * Fungsi callback HAL ini dipanggil oleh HAL_UART_Init().
 */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance != USART3) {
        return;
    }

    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    gpio.Pin = GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &gpio);

    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart3_rx);
    __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart3_tx);
    __HAL_LINKDMA(uartHandle, hdmatx, hdma_usart3_tx);

    /* Parser RX dipolling dari DMA NDTR. Tidak ada IDLE/USART IRQ yang dapat
     * berlomba dengan main-loop atau HAL TX state-machine. */
    __HAL_UART_DISABLE_IT(uartHandle, UART_IT_IDLE);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

/**
 * Melepas resource USART3 bila HAL melakukan de-inisialisasi peripheral.
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance != USART3) {
        return;
    }

    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11);
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

/**
 * PENJELASAN MX_GPIO_Init: mengatur seluruh GPIO board variant 0. Hall sebagai input, LED/buzzer/latch sebagai output, sensor arus serta PA2/PA3 sebagai analog, dan pin PWM motor sebagai alternate-function.
 */
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* LEFT starts in Hall mode. If VESC motor configuration selects ABI,
   * LeftEncoder_SetMode() later remaps PB6/PB7 to TIM4 CH1/CH2 and PB5 to
   * EXTI5 index Z. RIGHT PC10/11/12 remains Hall-only. */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  /* Hall/encoder board ini dibaca active-low. Pull-up internal menjaga input
   * tetap HIGH saat sensor open-collector melepas jalur atau kabel terputus,
   * sehingga raw 000/111 tidak muncul dari pin floating. */
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = LEFT_HALL_U_PIN;
  HAL_GPIO_Init(LEFT_HALL_U_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LEFT_HALL_V_PIN;
  HAL_GPIO_Init(LEFT_HALL_V_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LEFT_HALL_W_PIN;
  HAL_GPIO_Init(LEFT_HALL_W_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RIGHT_HALL_U_PIN;
  HAL_GPIO_Init(RIGHT_HALL_U_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RIGHT_HALL_V_PIN;
  HAL_GPIO_Init(RIGHT_HALL_V_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = RIGHT_HALL_W_PIN;
  HAL_GPIO_Init(RIGHT_HALL_W_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Pin = CHARGER_PIN;
  HAL_GPIO_Init(CHARGER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = BUTTON_PIN;
  HAL_GPIO_Init(BUTTON_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

  GPIO_InitStruct.Pin = LED_PIN;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BUZZER_PIN;
  HAL_GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = OFF_PIN;
  HAL_GPIO_Init(OFF_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;

  GPIO_InitStruct.Pin = LEFT_DC_CUR_PIN;
  HAL_GPIO_Init(LEFT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_U_CUR_PIN;
  HAL_GPIO_Init(LEFT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_V_CUR_PIN;
  HAL_GPIO_Init(LEFT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_DC_CUR_PIN;
  HAL_GPIO_Init(RIGHT_DC_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_U_CUR_PIN;
  HAL_GPIO_Init(RIGHT_U_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_V_CUR_PIN;
  HAL_GPIO_Init(RIGHT_V_CUR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = DCLINK_PIN;
  HAL_GPIO_Init(DCLINK_PORT, &GPIO_InitStruct);

  /* PA3 tetap digunakan sebagai input analog ADC2 channel 3. */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* PA2 tetap digunakan sebagai input analog ADC2 channel 2. */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;

  GPIO_InitStruct.Pin = LEFT_TIM_UH_PIN;
  HAL_GPIO_Init(LEFT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VH_PIN;
  HAL_GPIO_Init(LEFT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WH_PIN;
  HAL_GPIO_Init(LEFT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_UL_PIN;
  HAL_GPIO_Init(LEFT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_VL_PIN;
  HAL_GPIO_Init(LEFT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LEFT_TIM_WL_PIN;
  HAL_GPIO_Init(LEFT_TIM_WL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WH_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WH_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_UL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_UL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_VL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_VL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RIGHT_TIM_WL_PIN;
  HAL_GPIO_Init(RIGHT_TIM_WL_PORT, &GPIO_InitStruct);
}

/**
 * PENJELASAN MX_TIM_Init: menyiapkan TIM1 dan TIM8 sebagai PWM komplementer tiga-fasa dengan dead-time untuk inverter motor. Frekuensi dan hubungan master/slave dipertahankan.
 */
/*
 * POLARITAS GATE MOSFET BOARD VARIANT 0
 * --------------------------------------
 * High-side MOSFET : aktif HIGH -> output CHx aktif pada level logika HIGH.
 * Low-side MOSFET  : aktif LOW  -> output CHxN aktif pada level logika LOW.
 *
 * Kondisi OFF/idle dibuat berlawanan dengan level aktif:
 * - High-side idle = LOW  (RESET) -> MOSFET high-side OFF.
 * - Low-side idle  = HIGH (SET)   -> MOSFET low-side OFF.
 *
 * Dead-time hardware TIM1/TIM8 tetap digunakan agar high-side dan low-side
 * pada satu fasa tidak aktif bersamaan. Konfigurasi ini tidak mengubah
 * perhitungan FOC maupun nilai duty-cycle, hanya memperjelas polaritas gate.
 */
#define MOSFET_HIGH_ACTIVE_POLARITY  TIM_OCPOLARITY_HIGH
#define MOSFET_LOW_ACTIVE_POLARITY   TIM_OCNPOLARITY_LOW
#define MOSFET_HIGH_IDLE_STATE       TIM_OCIDLESTATE_RESET
#define MOSFET_LOW_IDLE_STATE        TIM_OCNIDLESTATE_SET

void MX_TIM_Init(void) {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM8_CLK_ENABLE();

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_SlaveConfigTypeDef sTimConfig;

  htim_right.Instance               = RIGHT_TIM;
  htim_right.Init.Prescaler         = 0;
  htim_right.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_right.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_right.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_right.Init.RepetitionCounter = 0;
  htim_right.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_right);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_right, &sMasterConfig);

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = MOSFET_HIGH_ACTIVE_POLARITY;
  sConfigOC.OCNPolarity  = MOSFET_LOW_ACTIVE_POLARITY;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = MOSFET_HIGH_IDLE_STATE;
  sConfigOC.OCNIdleState = MOSFET_LOW_IDLE_STATE;
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_right, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_right, &sBreakDeadTimeConfig);

  htim_left.Instance               = LEFT_TIM;
  htim_left.Init.Prescaler         = 0;
  htim_left.Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED1;
  htim_left.Init.Period            = 64000000 / 2 / PWM_FREQ;
  htim_left.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim_left.Init.RepetitionCounter = 0;
  htim_left.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(&htim_left);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim_left, &sMasterConfig);

  sTimConfig.InputTrigger = TIM_TS_ITR0;
  sTimConfig.SlaveMode    = TIM_SLAVEMODE_GATED;
  HAL_TIM_SlaveConfigSynchronization(&htim_left, &sTimConfig);

  // Start counting >0 to effectively offset timers by the time it takes for one ADC conversion to complete.
  // This method allows that the Phase currents ADC measurements are properly aligned with LOW-FET ON region for both motors
  LEFT_TIM->CNT 		     = ADC_TOTAL_CONV_TIME;

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = MOSFET_HIGH_ACTIVE_POLARITY;
  sConfigOC.OCNPolarity  = MOSFET_LOW_ACTIVE_POLARITY;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = MOSFET_HIGH_IDLE_STATE;
  sConfigOC.OCNIdleState = MOSFET_LOW_IDLE_STATE;
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim_left, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime         = DEAD_TIME;
  sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim_left, &sBreakDeadTimeConfig);

  LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
  RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;

  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_left, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_left, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim_right, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim_right, TIM_CHANNEL_3);

  htim_left.Instance->RCR = 1;

  __HAL_TIM_ENABLE(&htim_right);
}

/**
 * PENJELASAN MX_ADC1_Init: menyiapkan ADC1 dan DMA dual-mode agar sampling arus/tegangan tersinkron dengan timer PWM motor.
 */
void MX_ADC1_Init(void) {
  ADC_MultiModeTypeDef multimode;
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef injected;

  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance                   = ADC1;
  hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T8_TRGO;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion       = 3;
  HAL_ADC_Init(&hadc1);
  /**Enable or disable the remapping of ADC1_ETRGREG:
    * ADC1 External Event regular conversion is connected to TIM8 TRG0
    */
  __HAL_AFIO_REMAP_ADC1_ETRGREG_ENABLE();

  /**Configure the ADC multi-mode
    */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_11;  // pc1 left cur  ->  right
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_0;  // pa0 right a   ->  left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_14;  // pc4 left b   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  /* Battery dan temperature tidak boleh berada di scan current-control 16 kHz.
   * Rank temperature 239.5-cycle sebelumnya memakan hampir separuh deadline ISR.
   * Keduanya dipindah ke injected sequence yang dipicu pelan dari main-loop. */
  memset(&injected, 0, sizeof(injected));
  injected.InjectedChannel = ADC_CHANNEL_12;
  injected.InjectedRank = ADC_INJECTED_RANK_1;
  injected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  injected.InjectedOffset = 0U;
  injected.InjectedNbrOfConversion = 2U;
  injected.InjectedDiscontinuousConvMode = DISABLE;
  injected.AutoInjectedConv = DISABLE;
  injected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected);

  injected.InjectedChannel = ADC_CHANNEL_TEMPSENSOR;
  injected.InjectedRank = ADC_INJECTED_RANK_2;
  injected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &injected);

  hadc1.Instance->CR2 |= ADC_CR2_DMA | ADC_CR2_TSVREFE;

  __HAL_ADC_ENABLE(&hadc1);

  __HAL_RCC_DMA1_CLK_ENABLE();

  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CNDTR = 3;
  DMA1_Channel1->CPAR  = (uint32_t) & (ADC1->DR);
  DMA1_Channel1->CMAR  = (uint32_t)&adc_buffer;
  DMA1_Channel1->CCR   = DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_PL_1 | DMA_CCR_PL_0;
  DMA1_Channel1->CCR |= DMA_CCR_EN;

  /* Current-control ADC/DMA 16 kHz: samakan dengan baseline board yang terbukti stabil. */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* ADC2 init function */
/**
 * PENJELASAN MX_ADC2_Init: regular rank 1..3 tetap khusus current-loop.
 * PA2/PA3 dipindah ke injected ADC2 500 Hz untuk APP_ADC VESC tanpa
 * memperpanjang sequence arus 16 kHz.
 */
void MX_ADC2_Init(void) {
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef injected;

  __HAL_RCC_ADC2_CLK_ENABLE();

  // HAL_ADC_DeInit(&hadc2);
  // hadc2.Instance->CR2 = 0;
  /**Common config
    */
  hadc2.Instance                   = ADC2;
  hadc2.Init.ScanConvMode          = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode    = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion       = 3;
  HAL_ADC_Init(&hadc2);

  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = ADC_CHANNEL_10;  // pc0 right cur   -> left
  sConfig.Rank    = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  // sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfig.Channel = ADC_CHANNEL_13;  // pc3 right b   -> left
  sConfig.Rank    = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_15;  // pc5 left c   -> right
  sConfig.Rank    = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  /* VESC APP ADC inputs are deliberately NOT appended to the 16 kHz regular
   * dual-ADC scan. Doing so would lengthen every current-sampling sequence.
   * Instead PA2/PA3 use ADC2 injected conversions, launched just after a
   * completed current sample at 500 Hz. This keeps the current-loop timing
   * unchanged while providing deterministic throttle/brake samples. */
  memset(&injected, 0, sizeof(injected));
  injected.InjectedChannel = ADC_CHANNEL_2;  /* PA2 / VESC ADC1 */
  injected.InjectedRank = ADC_INJECTED_RANK_1;
  injected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  injected.InjectedOffset = 0U;
  injected.InjectedNbrOfConversion = 2U;
  injected.InjectedDiscontinuousConvMode = DISABLE;
  injected.AutoInjectedConv = DISABLE;
  injected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &injected);

  injected.InjectedChannel = ADC_CHANNEL_3;  /* PA3 / VESC ADC2 */
  injected.InjectedRank = ADC_INJECTED_RANK_2;
  injected.InjectedSamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &injected);

  hadc2.Instance->CR2 |= ADC_CR2_DMA;
  __HAL_ADC_ENABLE(&hadc2);
}

void ADC_Slow_TriggerFromCurrentISR(void)
{
  /* Called immediately after the regular dual-ADC current conversion has
   * completed. Injected conversions therefore run in the long quiet interval
   * before the next PWM-triggered sample instead of starting at an arbitrary
   * point from main(). No busy wait and no floating point is used here. */
  if (adcAppDivider > 0U) --adcAppDivider;
  if (adcAppDivider == 0U) {
    adcAppDivider = 32U;
    if (!adcAppPending) {
      ADC2->SR &= ~ADC_SR_JEOC;
      ADC2->CR2 |= ADC_CR2_JEXTTRIG | ADC_CR2_JSWSTART;
      adcAppPending = true;
    }
  }

  if (adcHousekeepingDivider > 0U) --adcHousekeepingDivider;
  if (adcHousekeepingDivider == 0U) {
    adcHousekeepingDivider = 1600U;
    if (!adcSlowPending) {
      ADC1->SR &= ~ADC_SR_JEOC;
      ADC1->CR2 |= ADC_CR2_JEXTTRIG | ADC_CR2_JSWSTART;
      adcSlowPending = true;
    }
  }
}

void ADC_Slow_Service(uint32_t now_ms)
{
  (void)now_ms;

  if (adcAppPending && (ADC2->SR & ADC_SR_JEOC) != 0U) {
    adc_buffer.pa2Analog = (uint16_t)ADC2->JDR1;
    adc_buffer.pa3Analog = (uint16_t)ADC2->JDR2;
    ADC2->SR &= ~ADC_SR_JEOC;
    adcAppPending = false;
  }

  if (adcSlowPending && (ADC1->SR & ADC_SR_JEOC) != 0U) {
    adc_buffer.batt1 = (uint16_t)ADC1->JDR1;
    adc_buffer.temp = (uint16_t)ADC1->JDR2;
    ADC1->SR &= ~ADC_SR_JEOC;
    adcSlowPending = false;
  }
}

