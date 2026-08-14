/**
  * This file is part of the hoverboard-firmware-hack project.
  *
  * Copyright (C) 2020-2021 Emanuel FERU <aerdronix@gmail.com>
  *
  * This program is free software: you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation, either version 3 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Includes
#include <stdio.h>
#include <stdlib.h> // for abs()
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "eeprom.h"
#include "util.h"
#include "esc_protocol.h"
#include "runtime_control.h"
#include "foc_motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;
extern UART_HandleTypeDef huart3;

extern int16_t batVoltage;
extern uint8_t backwardDrive;
extern uint8_t buzzerCount;             // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

extern uint8_t enable;                  // global variable for motor enable

extern volatile uint32_t main_loop_counter;
extern volatile uint32_t buzzerTimer;

//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
/* VESC-style motor state/configuration live in foc_motor_data.c. */
InputStruct input1[INPUTS_NR] = { {0, 0, 0, PRI_INPUT1} };
InputStruct input2[INPUTS_NR] = { {0, 0, 0, PRI_INPUT2} };

int16_t  speedAvg;                      // average measured speed
int16_t  speedAvgAbs;                   // average measured speed in absolute
uint8_t  timeoutFlgSerial = 0;          // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)



//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t INPUT_MAX;             // [-] Input target maximum limitation
static int16_t INPUT_MIN;             // [-] Input target minimum limitation

  static uint8_t  cur_spd_valid  = 0;
  static uint8_t  inp_cal_valid  = 0;


/* =========================== Retargeting printf =========================== */
/* retarget the C library printf function to the USART */

/* =========================== Initialization Functions =========================== */

/**
 * PENJELASAN MotorSystem_Init: menyalin parameter aktif ke motor kiri/kanan, menghubungkan pointer parameter-state-input-output ke instance controller, lalu memanggil inisialisasi FOC. Nilai parameter kontrol tidak diubah.
 */
void MotorSystem_Init(void) {
  /* Cortex-M3 cycle counter is the authoritative ISR deadline monitor. */
#if defined(DWT) && defined(CoreDebug)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  motorControlIsrDeadlineCycles = 64000000U / PWM_FREQ;
#else
  motorControlIsrDeadlineCycles = 0U; /* host tests do not emulate DWT */
#endif

  mc_foc_conf_set_defaults(&motorConfLeft, FOC_CURRENT_SAMPLE_IA_IB);
  mc_foc_conf_set_defaults(&motorConfRight, FOC_CURRENT_SAMPLE_IB_IC);

  motorConfLeft.l_current_max = (I_MOT_MAX * A2BIT_CONV) << 4;
  motorConfLeft.l_current_min = (int16_t)-motorConfLeft.l_current_max;
  motorConfLeft.l_max_speed_rpm_q4 = N_MOT_MAX << 4;
  motorConfLeft.foc_motor_pole_pairs = 15U;
  motorConfRight = motorConfLeft;
  motorConfRight.foc_current_sample_mode = FOC_CURRENT_SAMPLE_IB_IC;

  mc_foc_conf_prepare(&motorConfLeft);
  mc_foc_conf_prepare(&motorConfRight);
  mc_foc_init(&motorLeft, &motorConfLeft);
  mc_foc_init(&motorRight, &motorConfRight);
}

/**
 * PENJELASAN InputLimits_Init: menentukan batas command host minimum dan maksimum. Core FOC baru memakai rentang tetap -1000 sampai 1000 dan memetakan target sesuai mode.
 */
void InputLimits_Init(void) {
  INPUT_MAX = 1000;
  INPUT_MIN = -1000;
}

/**
 * PENJELASAN SerialInput_Init: hanya menginisialisasi USART3 PB10/PB11. RX circular DMA VESC dimulai oleh VescProtocol_Init() setelah peripheral siap. PA2/PA3 tetap murni ADC APP.
 */
void SerialInput_Init(void) {
    UART3_Init();
    UART_DisableRxErrors(&huart3);
}

/**
 * PENJELASAN UART_DisableRxErrors: mematikan interrupt parity/error/noise/frame pada UART RX agar DMA circular terus menerima byte. Validitas frame aplikasi tetap dijaga oleh START_FRAME dan CRC16.
 */
void UART_DisableRxErrors(UART_HandleTypeDef *huart)
{
  CLEAR_BIT(huart->Instance->CR1, USART_CR1_PEIE);    /* Disable PE (Parity Error) interrupts */
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);     /* Disable EIE (Frame error, noise error, overrun error) interrupts */
}

/* =========================== General Functions =========================== */

/**
 * PENJELASAN poweronMelody: memainkan urutan beep saat firmware selesai boot. Fungsi hanya untuk indikator dan tidak masuk ke perhitungan FOC.
 */
void poweronMelody(void) {
    /* Baseline hardware yang diberikan user terbukti boot dengan HAL_Delay.
     * SysTick dan current-DMA sekarang kembali pada preemption priority yang
     * sama, sehingga melody tidak bergantung pada heartbeat/DWT/loop polling. */
    buzzerCount = 0;
    for (int i = 8; i >= 0; --i) {
        buzzerFreq = (uint8_t)i;
        HAL_Delay(100U);
    }
    buzzerFreq = 0U;
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
}

/**
 * PENJELASAN beepCount: mengatur jumlah beep, frekuensi, dan pola. Pembentukan gelombang buzzer dilakukan di interrupt motor sehingga fungsi ini tidak memblokir loop.
 */
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) {
    buzzerCount   = cnt;
    buzzerFreq    = freq;
    buzzerPattern = pattern;
}

/**
 * PENJELASAN beepLong: membuat satu beep panjang dengan frekuensi yang diberikan.
 */
void beepLong(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(500);
    buzzerFreq = 0;
}

/**
 * PENJELASAN beepShort: membuat satu beep pendek dengan frekuensi yang diberikan.
 */
void beepShort(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    HAL_Delay(100);
    buzzerFreq = 0;
}

/**
 * PENJELASAN beepShortMany: membuat beberapa beep pendek dan dapat menaikkan/menurunkan pitch sesuai parameter arah.
 */
void beepShortMany(uint8_t cnt, int8_t dir) {
    if (dir >= 0) {   // increasing tone
      for(uint8_t i = 2*cnt; i >= 2; i=i-2) {
        beepShort(i + 3);
      }
    } else {          // decreasing tone
      for(uint8_t i = 2; i <= 2*cnt; i=i+2) {
        beepShort(i + 3);
      }
    }
}

/**
 * PENJELASAN calcAvgSpeed: mengambil motorSpeed dari kedua output controller, menghitung kecepatan rata-rata serta nilai absolutnya untuk safety dan power-off.
 */
void calcAvgSpeed(void) {
    // Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
      speedAvg    = ( motorOutputLeft.speed_rpm - motorOutputRight.speed_rpm) / 2;

    // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
    if (SPEED_COEFFICIENT & (1 << 16)) {
      speedAvg    = -speedAvg;
    }
    speedAvgAbs   = abs(speedAvg);
}

 /*
 * Auto-calibration of the ADC Limits
 * This function finds the Minimum, Maximum, and Middle for the ADC input
 * Procedure:
 * - press the power button for more than 5 sec and release after the beep sound
 * - move the potentiometers freely to the min and max limits repeatedly
 * - release potentiometers to the resting postion
 * - press the power button to confirm or wait for the 20 sec timeout
 * The Values will be saved to flash. Values are persistent if you flash with platformio. To erase them, make a full chip erase.
 */
/**
 * PENJELASAN adcCalibLim: mempertahankan prosedur kalibrasi batas input historis firmware dan menandai hasil agar dapat disimpan ke flash. Pada konfigurasi serial, fungsi tetap tersedia untuk kompatibilitas prosedur setting.
 */
void adcCalibLim(void) {
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // do not enter this mode if motors are spinning
    return;
  }

  readInputRaw();
  // Inititalization: MIN = a high value, MAX = a low value
  int32_t  input1_fixdt = input1[0].raw << 16;
  int32_t  input2_fixdt = input2[0].raw << 16;
  int16_t  INPUT1_MIN_temp = INT16_MAX;
  int16_t  INPUT1_MID_temp = 0;
  int16_t  INPUT1_MAX_temp = INT16_MIN;
  int16_t  INPUT2_MIN_temp = INT16_MAX;
  int16_t  INPUT2_MID_temp = 0;
  int16_t  INPUT2_MAX_temp = INT16_MIN;
  int16_t  input_margin    = 0;
  uint16_t input_cal_timeout = 0;

  // Extract MIN, MAX and MID from ADC while the power button is not pressed
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && input_cal_timeout++ < 4000) {   // 20 sec timeout
    readInputRaw();
    filtLowPass32(input1[0].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[0].raw, FILTER, &input2_fixdt);

    INPUT1_MID_temp = (int16_t)(input1_fixdt >> 16);// CLAMP(input1_fixdt >> 16, INPUT1_MIN, INPUT1_MAX);   // convert fixed-point to integer
    INPUT2_MID_temp = (int16_t)(input2_fixdt >> 16);// CLAMP(input2_fixdt >> 16, INPUT2_MIN, INPUT2_MAX);
    INPUT1_MIN_temp = MIN(INPUT1_MIN_temp, INPUT1_MID_temp);
    INPUT1_MAX_temp = MAX(INPUT1_MAX_temp, INPUT1_MID_temp);
    INPUT2_MIN_temp = MIN(INPUT2_MIN_temp, INPUT2_MID_temp);
    INPUT2_MAX_temp = MAX(INPUT2_MAX_temp, INPUT2_MID_temp);
    HAL_Delay(5);
  }

  input1[0].typ = checkInputType(INPUT1_MIN_temp, INPUT1_MID_temp, INPUT1_MAX_temp);
  if (input1[0].typ == input1[0].typDef || input1[0].typDef == 3) {  // Accept calibration only if the type is correct OR type was set to 3 (auto)
    input1[0].min = INPUT1_MIN_temp + input_margin;
    input1[0].mid = INPUT1_MID_temp;
    input1[0].max = INPUT1_MAX_temp - input_margin;
  } else {
    input1[0].typ = 0; // Disable input
  }

  input2[0].typ = checkInputType(INPUT2_MIN_temp, INPUT2_MID_temp, INPUT2_MAX_temp);
  if (input2[0].typ == input2[0].typDef || input2[0].typDef == 3) {  // Accept calibration only if the type is correct OR type was set to 3 (auto)
    input2[0].min = INPUT2_MIN_temp + input_margin;
    input2[0].mid = INPUT2_MID_temp;
    input2[0].max = INPUT2_MAX_temp - input_margin;
  } else {
    input2[0].typ = 0; // Disable input
  }
  inp_cal_valid = 1;    // Mark calibration to be saved in Flash at shutdown

}
 /*
 * Update Maximum Motor Current Limit (via ADC1) and Maximum Speed Limit (via ADC2)
 * Procedure:
 * - press the power button for more than 5 sec and immediatelly after the beep sound press one more time shortly
 * - move and hold the pots to a desired limit position for Current and Speed
 * - press the power button to confirm or wait for the 10 sec timeout
 */
/**
 * PENJELASAN updateCurSpdLim: memperbarui batas arus maxCurrent dan kecepatan maxSpeed dari input setting dengan skala fixed-point yang sama, lalu menandainya valid untuk disimpan.
 */
void updateCurSpdLim(void) {
  calcAvgSpeed();
  if (speedAvgAbs > 5) {    // do not enter this mode if motors are spinning
    return;
  }

  int32_t  input1_fixdt = input1[0].raw << 16;
  int32_t  input2_fixdt = input2[0].raw << 16;
  uint16_t cur_factor;    // fixdt(0,16,16)
  uint16_t spd_factor;    // fixdt(0,16,16)
  uint16_t cur_spd_timeout = 0;
  cur_spd_valid = 0;

  // Wait for the power button press
  while (!HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN) && cur_spd_timeout++ < 2000) {  // 10 sec timeout
    readInputRaw();
    filtLowPass32(input1[0].raw, FILTER, &input1_fixdt);
    filtLowPass32(input2[0].raw, FILTER, &input2_fixdt);
    HAL_Delay(5);
  }
  // Calculate scaling factors
  cur_factor = CLAMP((input1_fixdt - (input1[0].min << 16)) / (input1[0].max - input1[0].min), 6553, 65535);    // ADC1, MIN_cur(10%) = 1.5 A
  spd_factor = CLAMP((input2_fixdt - (input2[0].min << 16)) / (input2[0].max - input2[0].min), 3276, 65535);    // ADC2, MIN_spd(5%)  = 50 rpm

  if (input1[0].typ != 0){
    // Update current limit
    motorConfLeft.l_current_max = motorConfRight.l_current_max = (int16_t)((I_MOT_MAX * A2BIT_CONV * cur_factor) >> 12);
    motorConfLeft.l_current_min = motorConfRight.l_current_min = (int16_t)-motorConfLeft.l_current_max;
    mc_foc_conf_prepare(&motorConfLeft); mc_foc_conf_prepare(&motorConfRight);    // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid   = 1;  // Mark update to be saved in Flash at shutdown
  }

  if (input2[0].typ != 0){
    // Update speed limit
    motorConfLeft.l_max_speed_rpm_q4 = motorConfRight.l_max_speed_rpm_q4 = (int16_t)((N_MOT_MAX * spd_factor) >> 12);                 // fixdt(0,16,16) to fixdt(1,16,4)
    cur_spd_valid  += 2;  // Mark update to be saved in Flash at shutdown
  }

}

 /*
 * Standstill Hold Function
 * This function uses Cruise Control to provide an anti-roll functionality at standstill.
 * Only available and makes sense for FOC VOLTAGE or FOC TORQUE mode.
 *
 * Input:  none
 * Output: standstillAcv
 */
/*
 * Electric Brake Function
 * In case of TORQUE mode, this function replaces the motor "freewheel" with a constant braking when the input torque request is 0.
 * This is useful when a small amount of motor braking is desired instead of "freewheel".
 *
 * Input: speedBlend = fixdt(0,16,15), reverseDir = {0, 1}
 * Output: input2.cmd (Throtle) with brake component included
 */
/*
 * Cruise Control Function
 * This function activates/deactivates cruise control.
 *
 * Input: button (as a pulse)
 * Output: cruiseCtrlAcv
 */
/*
 * Check Input Type
 * This function identifies the input type: 0: Disabled, 1: Normal Pot, 2: Middle Resting Pot
 */
/**
 * PENJELASAN checkInputType: menentukan apakah input berperilaku normal atau center-rest berdasarkan posisi min/mid/max. Algoritma klasifikasi asli dipertahankan.
 */
int checkInputType(int16_t min, int16_t mid, int16_t max){

  int type = 0;
  int16_t threshold = 200;

  if ((min / threshold) == (max / threshold) || (mid / threshold) == (max / threshold) || min > max || mid > max) {
    type = 0;
  } else {
    if ((min / threshold) == (mid / threshold)){
      type = 1;
    } else {
      type = 2;
    }

  }

  return type;
}

/* =========================== Input Functions =========================== */

 /*
 * Calculate Input Command
 * This function realizes dead-band around 0 and scales the input between [out_min, out_max]
 */
/**
 * PENJELASAN calcInputCmd: mengubah nilai raw menjadi command keluaran berdasarkan tipe input, min/mid/max dan deadband. Pemetaan, clamp, tanda, dan pembagian integer tetap sama.
 */
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max) {
  switch (in->typ){
    case 1: // Input is a normal pot
      in->cmd = CLAMP(MAP(in->raw, in->min, in->max, 0, out_max), 0, out_max);
      break;
    case 2: // Input is a mid resting pot
      if( in->raw > in->mid - in->dband && in->raw < in->mid + in->dband ) {
        in->cmd = 0;
      } else if(in->raw > in->mid) {
        in->cmd = CLAMP(MAP(in->raw, in->mid + in->dband, in->max, 0, out_max), 0, out_max);
      } else {
        in->cmd = CLAMP(MAP(in->raw, in->mid - in->dband, in->min, 0, out_min), out_min, 0);
      }
      break;
    default: // Input is ignored
      in->cmd = 0;
      break;
  }
}

/**
 * PENJELASAN readInputRaw:
 * Membaca dua kanal analog yang memang dipertahankan pada board variant 0.
 * PA2 adalah ADC2 channel 2 dan menjadi input1, sedangkan PA3 adalah ADC2
 * channel 3 dan menjadi input2. Fungsi ini hanya digunakan oleh prosedur
 * kalibrasi ADC dan pengaturan limit arus/kecepatan berbasis tombol; command
 * motor normal tetap berasal dari protokol USART3 VLT/TRQ/SPD/POS.
 *
 * Mapping ini sama dengan mapping CONTROL_ADC firmware sumber:
 *   input1 <- ADC rank PA2
 *   input2 <- ADC rank PA3
 *
 * Fungsi tidak melakukan scaling agar adcCalibLim() dapat mengukur nilai raw
 * ADC 12-bit secara langsung (0..4095).
 */
void readInputRaw(void) {
  input1[0].raw = (int16_t)adc_buffer.pa2Analog;
  input2[0].raw = (int16_t)adc_buffer.pa3Analog;
}

/* =========================== Poweroff Functions =========================== */

 /*
 * Save Configuration to Flash
 * This function makes sure data is not lost after power-off
 */
/**
 * PENJELASAN saveConfig: menulis parameter arus/kecepatan dan batas input yang valid ke emulated EEPROM menggunakan virtual address asli.
 */
void saveConfig(void) {
    (void)RuntimeSettings_Save();
}

/**
 * PENJELASAN poweroff: menyimpan konfigurasi yang perlu disimpan lalu melepaskan latch power agar board mati dengan prosedur firmware asli.
 */
void poweroff(void) {
  enable = 0;
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) {
    buzzerFreq = (uint8_t)i;
    HAL_Delay(100);
  }
  saveConfig();
  HAL_GPIO_WritePin(OFF_PORT, OFF_PIN, GPIO_PIN_RESET);
  while(1) {}
}

/**
 * PENJELASAN poweroffPressCheck: membaca tombol power, mendeteksi pola tekan yang dipakai untuk power-off/auto-calibration, dan menjalankan aksi yang sama dengan firmware sumber.
 */
void poweroffPressCheck(void) {
    if(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
      enable = 0;
      uint16_t cnt_press = 0;
      while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {
        HAL_Delay(10);
        if (cnt_press++ == 5 * 100) { beepShort(5); }
      }
      if (cnt_press >= 5 * 100) {                         // Check if press is more than 5 sec
        HAL_Delay(1000);
        if (HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) {  // Double press: Adjust Max Current, Max Speed
          while(HAL_GPIO_ReadPin(BUTTON_PORT, BUTTON_PIN)) { HAL_Delay(10); }
          beepLong(8);
          updateCurSpdLim();
          beepShort(5);
        } else {                                          // Long press: Calibrate ADC Limits
          beepLong(16);
          adcCalibLim();
          beepShort(5);
        }
      } else if (cnt_press > 8) {                         // Short press: power off (80 ms debounce)
        poweroff();
      }
    }
}

/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  *
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  *
  * Example:
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
/**
 * PENJELASAN filtLowPass32: low-pass IIR fixed-point 32-bit. Koefisien berada pada skala 2^16 dan urutan operasi integer dipertahankan.
 */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y) {
  int64_t tmp;
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}
  // Old filter
  // Inputs:       u     = int16
  // Outputs:      y     = fixdt(1,32,20)
  // Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  // yint = (int16_t)(y >> 20); // the integer output is the fixed-point ouput shifted by 20 bits
  // void filtLowPass32(int16_t u, uint16_t coef, int32_t *y) {
  //   int32_t tmp;
  //   tmp = (int16_t)(u << 4) - (*y >> 16);
  //   tmp = CLAMP(tmp, -32768, 32767);  // Overflow protection
  //   *y  = coef * tmp + (*y);
  // }

  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,16,4)
  * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
  */
/**
 * PENJELASAN rateLimiter16: membatasi perubahan command per siklus agar setpoint tidak melonjak lebih cepat dari RATE.
 */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y) {
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) {
    q0 = rate;
  } else {
    q1 = -rate;
    if (q0 < q1) {
      q0 = q1;
    }
  }

  *y = q0 + *y;
}

  /* mixerFcn(input_speed, input_steer, &output_speedR, &output_speedL);
  * Inputs:       input_speed, input_steer                  = fixdt(1,16,4)
  * Outputs:      output_speedR, output_speedL                = int16_t
  * Parameters:   SPEED_COEFFICIENT, STEER_COEFFICIENT  = fixdt(0,16,14)
  */
/**
 * PENJELASAN mixerFcn: menggabungkan speed dan steer menjadi command motor kanan/kiri memakai koefisien fixed-point SPEED_COEFFICIENT dan STEER_COEFFICIENT, kemudian melakukan clamp.
 */
void mixerFcn(int16_t input_speed, int16_t input_steer, int16_t *output_speedR, int16_t *output_speedL) {
  int16_t prodSpeed;
  int16_t prodSteer;
  int32_t tmp;

  prodSpeed   = (int16_t)((input_speed * (int16_t)SPEED_COEFFICIENT) >> 14);
  prodSteer   = (int16_t)((input_steer * (int16_t)STEER_COEFFICIENT) >> 14);

  tmp         = prodSpeed - prodSteer;
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *output_speedR = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
  *output_speedR = CLAMP(*output_speedR, INPUT_MIN, INPUT_MAX);

  tmp         = prodSpeed + prodSteer;
  tmp         = CLAMP(tmp, -32768, 32767);  // Overflow protection
  *output_speedL = (int16_t)(tmp >> 4);        // Convert from fixed-point to int
  *output_speedL = CLAMP(*output_speedL, INPUT_MIN, INPUT_MAX);
}
