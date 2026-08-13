#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"
#include "control_profile.h"

/*
 * KONFIGURASI FIRMWARE TUNGGAL
 * ============================
 * Firmware ini sengaja hanya mendukung hardware yang digunakan sekarang:
 * - Board mapping : BOARD_VARIANT 0
 * - Perintah      : USART3 pada PB10/PB11
 * - Feedback      : USART3 pada PB10/PB11
 * - PA2 dan PA3   : tetap sebagai ADC2 channel 2 dan channel 3
 *
 * Tidak ada cabang konfigurasi hardware lain pada source aktif ini.
 * PWM tiga-fasa untuk inverter motor tetap dipakai karena merupakan bagian wajib
 * dari penggerak FOC, bukan sebagai antarmuka input eksternal.
 */

/* Timing utama */
#define PWM_FREQ                 CONTROL_PWM_FREQUENCY_HZ
#define DEAD_TIME               48
#define DELAY_IN_MAIN_LOOP       5
#define TIMEOUT                  20
#define A2BIT_CONV               CONTROL_CURRENT_ADC_COUNTS_PER_A

/* Timing ADC */
#define ADC_CONV_TIME_1C5        14
#define ADC_CONV_TIME_7C5        20
#define ADC_CONV_TIME_13C5       26
#define ADC_CONV_TIME_28C5       41
#define ADC_CONV_TIME_41C5       54
#define ADC_CONV_TIME_55C5       68
#define ADC_CONV_TIME_71C5       84
#define ADC_CONV_TIME_239C5      252
/* Stock hoverboard current-sampling contract. The TIM8 phase offset must use
 * the conversion time of the phase-current rank (7.5 sampling cycles), exactly
 * as in hoverboard-firmware-hack-FOC. This is deliberately kept together with
 * the stock ADC /4 clock because the low-side shunts are only valid in a narrow
 * LOW-FET conduction window; changing either value changes the physical sample
 * instant and can look like tens of amps of false current during PWM. */
#define ADC_CONV_CLOCK_CYCLES    ADC_CONV_TIME_7C5
#define ADC_CLOCK_DIV            4
#define ADC_TOTAL_CONV_TIME      (ADC_CLOCK_DIV * ADC_CONV_CLOCK_CYCLES)

/* Mapping board yang digunakan */
#define BOARD_VARIANT            0

/* Battery */
#define BAT_FILT_COEF            655
#define BAT_CALIB_REAL_VOLTAGE   3970
#define BAT_CALIB_ADC            1492
#define BAT_CELLS                10
#define BAT_LVL2_ENABLE          0
#define BAT_LVL1_ENABLE          1
#define BAT_BLINK_INTERVAL       80
#define BAT_LVL5                 ((390 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL4                 ((380 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL3                 ((370 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL2                 ((360 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_LVL1                 ((350 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)
#define BAT_DEAD                 ((337 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE)

/* Temperature */
#define TEMP_FILT_COEF           655
#define TEMP_CAL_LOW_ADC         1655
#define TEMP_CAL_LOW_DEG_C       358
#define TEMP_CAL_HIGH_ADC        1588
#define TEMP_CAL_HIGH_DEG_C      489
#define TEMP_WARNING_ENABLE      0
#define TEMP_WARNING             600
#define TEMP_POWEROFF_ENABLE     0
#define TEMP_POWEROFF            650

/* Batas motor board. Mode runtime didefinisikan satu kali oleh esc_protocol/foc_motor. */
#define I_MOT_MAX                15
#define I_DC_MAX                 17
#define N_MOT_MAX               1000

/* Safety dan penyimpanan */
#define INACTIVITY_TIMEOUT       30
#define BEEPS_BACKWARD           0
#define ADC_MARGIN               100
#define ADC_PROTECT_TIMEOUT      100
#define ADC_PROTECT_THRESH       200
#define AUTO_CALIBRATION_ENA

/* Filter perintah, fixed-point sama seperti konfigurasi asli */
#define RATE                     480
#define FILTER                   6553
#define SPEED_COEFFICIENT        16384
#define STEER_COEFFICIENT        8192

/* Protokol USART3 runtime. Mode tidak lagi ditentukan compile-time. */
#define INPUTS_NR                1
#define PRI_INPUT1               3, -1000, 0, 1000, 0
#define PRI_INPUT2               3, -1000, 0, 1000, 0
#define FLASH_WRITE_KEY          0xEC03
#define SERIAL_START_FRAME       0xABCD
#define SERIAL_BUFFER_SIZE       512
#define SERIAL_TIMEOUT_MS        1500
#define MOTOR_ISR_OVERRUN_LIMIT  8U     /* persistent missed PWM cycles -> fail-safe DISARM */
#define TELEMETRY_RATE_DEFAULT   50
#define TELEMETRY_RATE_MAX       100
#define POSITION_MIN_DEFAULT    (-200000L)
#define POSITION_MAX_DEFAULT     200000L
#define POSITION_DEADBAND_DEFAULT 2U

/*
 * Arah motor tidak lagi di-hardcode per sisi. LEFT dan RIGHT memakai konvensi
 * core yang sama; pembalikan arah dilakukan oleh MotorRuntimeConfig yang dapat
 * diubah dari GUI dan disimpan ke EEPROM.
 */

#define USART3_BAUD              115200
#define USART3_WORDLENGTH        UART_WORDLENGTH_8B

#endif /* CONFIG_H */
