/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-point STM32F103 adaptation of the VESC-style FOC architecture.
 * VESC reference: Copyright 2016-2022 Benjamin Vedder.
 * Board firmware lineage: Copyright 2019-2020 Emanuel FERU.
 * See NOTICE.md and COPYING.
 */

#ifndef MOTOR_SENSOR_H
#define MOTOR_SENSOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_SENSOR_HALL_UVW = 0,
    MOTOR_SENSOR_ENCODER_AB = 1
} MotorSensorType;

/*
 * Konfigurasi arah dan sensor per motor.
 * encoder_cpr adalah jumlah count quadrature x4 per satu putaran mekanik.
 * encoder_offset_deg adalah trim sudut LISTRIK 0..359 derajat, bukan mekanik.
 */
typedef struct {
    uint8_t motor_inverted;
    uint8_t sensor_type;
    uint8_t sensor_inverted;
    uint16_t encoder_cpr;
    uint16_t encoder_offset_deg;
    /* Electrical revolutions per encoder mechanical revolution. Kept separate
     * from physical motor pole-pairs because VESC exposes these as independent
     * MCCONF fields. 0 means follow foc_motor_pole_pairs for legacy EEPROM. */
    uint8_t encoder_ratio;
    /* Encoder incremental baru dianggap siap closed-loop setelah Auto Detect
     * memverifikasi arah, CPR x4, dan electrical alignment/offset. Flag ini
     * persistent di EEPROM melalui config word. OPEN mode tidak memerlukannya. */
    uint8_t encoder_calibrated;
    /* Urutan quadrature A/B yang benar-benar terdeteksi untuk arah mekanik positif.
     * Disimpan persistent di EEPROM dan dipakai langsung decoder runtime untuk
     * menentukan arah/count (contoh 00->10->11->01 atau kebalikannya). */
    uint8_t encoder_sequence[4];
    uint8_t encoder_sequence_valid;
    /* Proof terpisah bahwa Hall sequence berasal dari commissioning, bukan sekadar
     * LUT default bawaan firmware. */
    uint8_t hall_calibrated;
    /* Urutan Hall raw untuk posisi listrik 0..5. Setiap entry adalah kode UVW 1..6.
     * Dengan menyimpan LUT per motor, wiring Hall kiri/kanan boleh berbeda tanpa
     * mengubah LUT global FOC. Data ini dipelajari oleh auto/manual calibration. */
    uint8_t hall_sequence[6];
    uint8_t hall_lut_valid;
} MotorRuntimeConfig;

typedef struct {
    volatile int32_t position_ticks;
    uint8_t previous_hall_position;
    uint8_t previous_encoder_ab;
    uint32_t encoder_valid_edges;
    uint32_t encoder_invalid_transitions;
    /* Karena A/B dibaca dengan polling 16 kHz pada pin Hall-U/V (tanpa EXTI/timer),
     * satu state quadrature dapat terlewat pada kecepatan tinggi. Setelah arah
     * sudah terkunci oleh >=2 edge valid searah, diagonal jump dapat direkonstruksi
     * konservatif sebagai dua count. Jump ambigu tetap dihitung invalid. */
    uint32_t encoder_recovered_edges;
    /* Counter transisi directed AB 4x4. Increment hanya pada edge aktual di ISR,
     * sehingga Auto/Manual Detect dapat merekonstruksi urutan 4 state tanpa
     * bergantung polling main-loop 200 Hz. */
    uint16_t encoder_transition_counts[16];
    /* Counter transisi Hall raw UVW 8x8 dari ISR 16 kHz. Manual Hall detect
     * memakai delta counter ini agar urutan 6-state tidak bergantung pada main-loop
     * 200 Hz dan tetap tertangkap saat poros diputar tangan lebih cepat. */
    uint16_t hall_transition_counts[64];
    uint8_t previous_raw_hall_encoding;
    bool hall_raw_initialized;
    int8_t encoder_last_raw_direction;
    uint8_t encoder_direction_confidence;
    /* Estimator kecepatan encoder memakai delta-count pada jendela tetap 4 ms
     * (64 update @ 16 kHz) lalu IIR 1/2. Nilai disimpan sebagai RPM Q4
     * (rpm * 16), sama dengan skala speed internal core FOC. */
    int32_t encoder_speed_reference_ticks;
    uint16_t encoder_speed_window_count;
    int16_t encoder_speed_q4;
    bool encoder_speed_initialized;

    /* Estimator kecepatan Hall memakai urutan Hall yang AKTIF dari hasil
     * commissioning (config->hall_sequence), bukan LUT wiring hardcoded. Perioda
     * dihitung pada sampling kontrol 16 kHz dan dikonversi ke RPM Q4 memakai
     * pole-pair runtime motor. Dengan demikian perubahan hasil kalibrasi langsung
     * menjadi sumber posisi, arah, dan kecepatan setelah MotorSensor_Reset(). */
    uint32_t hall_period_samples;
    uint32_t hall_last_sector_period_samples;
    uint32_t hall_speed_coefficient_q4;
    /* Hall electrical phase interpolation. Accumulator is Q16.16 of one-turn Q16 phase;
     * step is computed only on Hall edge, so the per-sample path has no division. */
    uint32_t hall_phase_accumulator_q16;
    int32_t hall_phase_step_q16;
    uint32_t hall_phase_steps_remaining;
    uint8_t hall_speed_pole_pairs;
    int16_t hall_speed_q4;
    int8_t hall_last_direction;

    /* Referensi listrik terpisah dari position_ticks. Incremental encoder perlu
     * electrical alignment setelah boot, tetapi ZERO/HOMING mekanik tidak boleh
     * merusak referensi posisi. */
    bool encoder_electrical_aligned;

    /* Backend encoder dapat berupa polling legacy atau TIM4 hardware pada LEFT.
     * Agar ISR 16 kHz tetap
     * ringan, sudut tidak dihitung dengan pembagian 64-bit setiap sampel.
     * Counter fase modulo-CPR di-update hanya ketika A/B benar-benar berubah,
     * lalu dikonversi ke Q-angle memakai skala Q16 yang diprecompute sekali. */
    uint16_t encoder_mechanical_phase_count;
    uint16_t encoder_scale_cpr;
    uint16_t encoder_electrical_step_q16;
    /* Reciprocal floor(2^32 / CPR) untuk batched modulo tanpa software divide
     * pada ISR hardware encoder. Cortex-M3 mengeksekusi multiply-long secara
     * efisien; koreksi sisa maksimal satu langkah. */
    uint32_t encoder_cpr_recip_q32;
    uint16_t encoder_electrical_step_remainder;
    uint16_t encoder_electrical_remainder_accum;
    uint16_t encoder_electrical_phase_q16;
    uint16_t encoder_electrical_zero_q16;
    uint16_t encoder_offset_phase_q16;
    uint8_t encoder_scale_pole_pairs;
    uint32_t encoder_mechanical_scale_q16;
    uint32_t encoder_electrical_scale_q16;
    /* Speed scale memakai Q7, bukan Q16. Dengan polling 16 kHz, delta maksimal
     * per window 64 sample adalah 128 count (recovery maksimal +/-2/sample),
     * sehingga delta*scale selalu muat int32_t dan ISR tidak memerlukan int64. */
    uint32_t encoder_speed_per_count_q7;
    int8_t encoder_last_delta;

    /* LUT hot-path dibangun SEKALI dari config hasil GUI/EEPROM/kalibrasi.
     * ISR tidak melakukan scan sequence 4/6 elemen lagi. */
    int8_t encoder_transition_lut[16];
    int8_t hall_raw_to_position[8];
    uint8_t hall_raw_to_normalized[8];
    bool runtime_lut_prepared;

    int32_t encoder_hw_last_count;
    bool encoder_hw_count_initialized;
    bool hall_initialized;
    bool encoder_initialized;
} MotorSensorState;

typedef struct {
    uint8_t hall_a;
    uint8_t hall_b;
    uint8_t hall_c;
    uint8_t hall_encoding;
    uint8_t raw_hall_encoding;
    /* Raw encoder A/B untuk telemetry/commissioning. Encoder memakai hanya
     * Hall-U=A dan Hall-V=B; Hall-W tidak pernah menjadi bagian decoding encoder.
     * Pada Hall mode field ini nol dan hall_encoding berisi UVW normal. */
    uint8_t encoder_ab;
    uint8_t feedback_valid;
    int16_t mechanical_angle_q4;
    /* Sudut listrik encoder langsung dalam Q6-degree (0..23039). Ini menghindari
     * quantisasi/offset Hall sintetis saat FOC memakai Encoder A/B. */
    int16_t electrical_angle_q6;
    /* Unified electrical phase for FOC, one mechanical electrical turn = 0..65535.
     * Encoder and Hall both publish this field from their calibrated backend. */
    uint16_t electrical_phase_q16;
    /* Kecepatan mekanik encoder dalam RPM Q4 (rpm * 16). */
    int16_t mechanical_speed_q4;
    int32_t position_ticks;
} MotorSensorSample;

extern MotorSensorState motorSensorStateLeft;
extern MotorSensorState motorSensorStateRight;
extern MotorSensorSample motorSensorSampleLeft;
extern MotorSensorSample motorSensorSampleRight;

#define MOTOR_CONFIG_FLAG_MOTOR_INVERTED  (1U << 0)
#define MOTOR_CONFIG_FLAG_SENSOR_INVERTED (1U << 1)
#define MOTOR_CONFIG_FLAG_ENCODER_AB      (1U << 2)
#define MOTOR_CONFIG_FLAG_ENCODER_CALIBRATED (1U << 12)
#define MOTOR_CONFIG_FLAG_HALL_CALIBRATED    (1U << 13)
#define MOTOR_CONFIG_OFFSET_SHIFT         3U
#define MOTOR_CONFIG_OFFSET_MASK          (0x01FFU << MOTOR_CONFIG_OFFSET_SHIFT)

#define MOTOR_ENCODER_CPR_DEFAULT         2048U
#define MOTOR_ENCODER_CPR_MIN             4U
#define MOTOR_ENCODER_CPR_MAX             65535U

void MotorRuntimeConfig_SetDefaults(MotorRuntimeConfig *config);
/* Hapus proof-of-calibration + LUT Hall (hall_calibrated, hall_lut_valid,
 * hall_sequence) kembali ke default pabrik. sensor_type/CPR/offset/inverted
 * dan sisi encoder TIDAK disentuh, sehingga Reset LUT Hall aman dipanggil
 * meski motor sedang memakai Encoder A/B. */
void MotorRuntimeConfig_ResetHallCalibration(MotorRuntimeConfig *config);
/* Hapus proof-of-calibration + sequence Encoder (encoder_calibrated,
 * encoder_sequence_valid, encoder_sequence) kembali ke default pabrik.
 * encoder_cpr/encoder_offset_deg TIDAK direset karena keduanya parameter
 * wiring/mekanik yang independen dari arah/phase FOC yang dibuktikan Auto
 * Detect. sensor_type/hall TIDAK disentuh. */
void MotorRuntimeConfig_ResetEncoderCalibration(MotorRuntimeConfig *config);
bool MotorRuntimeConfig_IsValid(const MotorRuntimeConfig *config);
uint16_t MotorRuntimeConfig_PackWord(const MotorRuntimeConfig *config);
bool MotorRuntimeConfig_UnpackWord(uint16_t config_word, uint16_t encoder_cpr,
                                   MotorRuntimeConfig *config);
bool MotorRuntimeConfig_SetHallSequence(MotorRuntimeConfig *config, const uint8_t sequence[6]);
bool MotorRuntimeConfig_HallSequenceValid(const uint8_t sequence[6]);
/* Rekonstruksi urutan Hall dari counter transisi RAW 8x8 yang direkam ISR.
 * Mendukung putar tangan maju/mundur; sequence lama dipakai sebagai anchor
 * electrical dan tie-break arah agar manual commissioning tidak mengubah
 * absolute D-axis reference tanpa alignment aktif. */
bool MotorRuntimeConfig_DetectHallSequenceFromTransitions(
    const MotorRuntimeConfig *config,
    const uint16_t transition_delta[64],
    uint8_t sequence[6]);
bool MotorRuntimeConfig_SetEncoderSequence(MotorRuntimeConfig *config, const uint8_t sequence[4]);
bool MotorRuntimeConfig_EncoderSequenceValid(const uint8_t sequence[4]);
void MotorRuntimeConfig_GetEncoderSequence(const MotorRuntimeConfig *config, uint8_t sequence[4]);
void MotorRuntimeConfig_GetHallSequence(const MotorRuntimeConfig *config, uint8_t sequence[6]);

void MotorSensor_Reset(MotorSensorState *state);
/* Precompute LUT/scaling dari konfigurasi AKTIF. Aman dipanggil saat IRQ dimatikan
 * pada LOAD/APPLY/selesai kalibrasi sehingga cycle pertama closed-loop tetap ringan. */
void MotorSensor_PrepareRuntime(const MotorRuntimeConfig *config,
                                MotorSensorState *state,
                                uint8_t pole_pairs);

/* Establish the absolute electrical reference of an incremental A/B encoder
 * while the rotor is physically held at desired_phase_q16. Mechanical
 * position_ticks is intentionally untouched. Returns the phase that the FOC
 * reader will observe after synchronization. */
uint16_t MotorSensor_SyncEncoderElectricalPhase(MotorSensorState *state,
                                                uint16_t desired_phase_q16);
void MotorSensor_Zero(MotorSensorState *state);
void MotorSensor_UpdateHardwareEncoder(const MotorRuntimeConfig *config,
                                       MotorSensorState *state,
                                       int32_t hardware_count,
                                       uint8_t raw_a, uint8_t raw_b,
                                       uint8_t pole_pairs,
                                       MotorSensorSample *sample);
void MotorSensor_Update(const MotorRuntimeConfig *config,
                        MotorSensorState *state,
                        uint8_t raw_u,
                        uint8_t raw_v,
                        uint8_t raw_w,
                        uint8_t pole_pairs,
                        MotorSensorSample *sample);

#endif /* MOTOR_SENSOR_H */
