#ifndef UTIL_H
#define UTIL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/** Struktur konfigurasi dan hasil normalisasi satu kanal input. */
typedef struct {
    int16_t raw;
    int16_t cmd;
    uint8_t typ;
    uint8_t typDef;
    int16_t min;
    int16_t mid;
    int16_t max;
    int16_t dband;
} InputStruct;

/** Menyiapkan parameter, state, input, output dan instance FOC kedua motor. */
void MotorSystem_Init(void);
/** Menentukan rentang command aktif sesuai konfigurasi field weakening. */
void InputLimits_Init(void);
/** Mengaktifkan USART3 DMA dan membaca konfigurasi dari emulated EEPROM. */
void SerialInput_Init(void);
/** Menonaktifkan interrupt error RX agar circular DMA tidak berhenti. */
void UART_DisableRxErrors(UART_HandleTypeDef *huart);

/** Memainkan melodi startup. */
void poweronMelody(void);
/** Mengatur pola buzzer non-blocking. */
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
/** Menghasilkan beep panjang. */
void beepLong(uint8_t freq);
/** Menghasilkan beep pendek. */
void beepShort(uint8_t freq);
/** Menghasilkan beberapa beep pendek dengan perubahan pitch. */
void beepShortMany(uint8_t cnt, int8_t dir);

/** Menghitung kecepatan rata-rata kedua motor. */
void calcAvgSpeed(void);
/** Menjalankan prosedur kalibrasi batas input yang dipertahankan dari firmware asli. */
void adcCalibLim(void);
/** Memperbarui limit arus dan kecepatan yang akan disimpan ke EEPROM. */
void updateCurSpdLim(void);
/** Menentukan tipe input dari posisi minimum, tengah dan maksimum. */
int checkInputType(int16_t min, int16_t mid, int16_t max);
/** Mengubah nilai raw menjadi command ternormalisasi. */
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max);
/** Membaca nilai ADC raw PA2/PA3 untuk prosedur kalibrasi dan limit. */
void readInputRaw(void);

/** Memeriksa frame baru pada USART3 circular DMA. */
void usart3_rx_check(void);

/** Menyimpan konfigurasi valid ke emulated EEPROM. */
void saveConfig(void);
/** Menyimpan konfigurasi lalu melepas latch daya. */
void poweroff(void);
/** Memeriksa tombol power dan prosedur kalibrasi/limit. */
void poweroffPressCheck(void);

/** Low-pass filter fixed-point 32-bit. */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
/** Rate limiter fixed-point 16-bit. */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);
/** Mixer speed/steer menjadi command kanan dan kiri. */
void mixerFcn(int16_t input_speed,
              int16_t input_steer,
              int16_t *output_speedR,
              int16_t *output_speedL);

#endif /* UTIL_H */
