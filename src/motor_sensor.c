/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Fixed-point STM32F103 adaptation of the VESC-style FOC architecture.
 * VESC reference: Copyright 2016-2022 Benjamin Vedder.
 * Board firmware lineage: Copyright 2019-2020 Emanuel FERU.
 * See NOTICE.md and COPYING.
 */

#include "motor_sensor.h"
#include "control_profile.h"
#include <string.h>

MotorSensorState motorSensorStateLeft;
MotorSensorState motorSensorStateRight;
MotorSensorSample motorSensorSampleLeft;
MotorSensorSample motorSensorSampleRight;

/* Hall encoding yang mewakili posisi listrik 0..5 pada lookup controller. */
static const uint8_t hall_encoding_from_position[6] = {2U, 3U, 1U, 5U, 4U, 6U};

/* Quadrature transition table: index = previous_AB << 2 | current_AB. */
static const int8_t encoder_delta_table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/* Arah raw positif sesuai encoder_delta_table: 00->10->11->01->00.
 * Sequence hasil detect boleh kebalikannya; sensor_inverted menormalkan arah. */
static const uint8_t encoder_sequence_default[4] = {0U, 2U, 3U, 1U};

static uint8_t popcount2_u8(uint8_t value)
{
    value &= 0x03U;
    return (uint8_t)((value & 1U) + ((value >> 1) & 1U));
}

bool MotorRuntimeConfig_EncoderSequenceValid(const uint8_t sequence[4])
{
    if (sequence == NULL) return false;
    uint8_t seen = 0U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint8_t state = sequence[i];
        const uint8_t next = sequence[(uint8_t)((i + 1U) & 0x03U)];
        if (state > 3U || next > 3U) return false;
        const uint8_t bit = (uint8_t)(1U << state);
        if ((seen & bit) != 0U) return false;
        seen |= bit;
        if (popcount2_u8((uint8_t)(state ^ next)) != 1U) return false;
    }
    return seen == 0x0FU;
}

bool MotorRuntimeConfig_SetEncoderSequence(MotorRuntimeConfig *config, const uint8_t sequence[4])
{
    if (config == NULL || !MotorRuntimeConfig_EncoderSequenceValid(sequence)) return false;
    memcpy(config->encoder_sequence, sequence, 4U);
    config->encoder_sequence_valid = 1U;
    return true;
}

void MotorRuntimeConfig_GetEncoderSequence(const MotorRuntimeConfig *config, uint8_t sequence[4])
{
    if (sequence == NULL) return;
    if (config == NULL || config->encoder_sequence_valid == 0U ||
        !MotorRuntimeConfig_EncoderSequenceValid(config->encoder_sequence)) {
        memcpy(sequence, encoder_sequence_default, 4U);
        return;
    }
    memcpy(sequence, config->encoder_sequence, 4U);
}

#define MOTOR_CONTROL_FREQUENCY_HZ         CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ
#define ENCODER_SPEED_WINDOW_SAMPLES          64U
#define ENCODER_MAX_COUNTS_PER_SAMPLE          2U
#define ENCODER_SPEED_SCALE_FRAC_BITS          7U
#define HALL_SECTORS_PER_ELECTRICAL_REV        6U
#define HALL_SPEED_ZERO_PERIOD_MULTIPLIER       4U
/* Seluruh konstanta ini muat 32-bit pada per-motor FOC update rate. */
#define ENCODER_SPEED_NUMERATOR_Q4 \
    ((uint32_t)(60U * MOTOR_CONTROL_FREQUENCY_HZ * 16U / ENCODER_SPEED_WINDOW_SAMPLES))
#define HALL_SPEED_NUMERATOR_Q4 \
    ((uint32_t)(60U * MOTOR_CONTROL_FREQUENCY_HZ * 16U / HALL_SECTORS_PER_ELECTRICAL_REV))


static int32_t add_position_saturated(int32_t position, int8_t delta)
{
    /* Hall/software quadrature delta hanya kecil. Hindari int64 di hot path. */
    if (delta > 0 && position > INT32_MAX - (int32_t)delta) return INT32_MAX;
    if (delta < 0 && position < INT32_MIN - (int32_t)delta) return INT32_MIN;
    return position + (int32_t)delta;
}

static int32_t add_position_delta_saturated(int32_t position, int32_t delta)
{
    /* Hardware TIM4 dapat mengakumulasi beberapa count di antara dua sample. */
    if (delta > 0 && position > INT32_MAX - delta) return INT32_MAX;
    if (delta < 0 && position < INT32_MIN - delta) return INT32_MIN;
    return position + delta;
}

static int8_t hall_position_delta(uint8_t previous_position, uint8_t current_position)
{
    static const int8_t direction_table[6] = {0, -1, -2, 0, 2, 1};
    int8_t index = (int8_t)previous_position - (int8_t)current_position;
    if (index < 0) index = (int8_t)(index + 6);
    return direction_table[(uint8_t)index];
}

bool MotorRuntimeConfig_HallSequenceValid(const uint8_t sequence[6])
{
    if (sequence == NULL) return false;
    uint8_t seen = 0U;
    for (uint8_t i = 0U; i < 6U; ++i) {
        const uint8_t code = sequence[i];
        if (code == 0U || code == 7U || code > 7U) return false;
        const uint8_t bit = (uint8_t)(1U << code);
        if ((seen & bit) != 0U) return false;
        seen |= bit;
    }
    /* Urutan Hall fisik harus membentuk cincin Gray-code: setiap sektor hanya
     * boleh mengubah satu bit, termasuk transisi sektor terakhir -> pertama. */
    for (uint8_t i = 0U; i < 6U; ++i) {
        const uint8_t a = sequence[i];
        const uint8_t b = sequence[(uint8_t)((i + 1U) % 6U)];
        const uint8_t changed = (uint8_t)(a ^ b);
        if (changed == 0U || (changed & (uint8_t)(changed - 1U)) != 0U) return false;
    }
    return true;
}

bool MotorRuntimeConfig_SetHallSequence(MotorRuntimeConfig *config, const uint8_t sequence[6])
{
    if (config == NULL || !MotorRuntimeConfig_HallSequenceValid(sequence)) return false;
    memcpy(config->hall_sequence, sequence, 6U);
    config->hall_lut_valid = 1U;
    return true;
}

void MotorRuntimeConfig_GetHallSequence(const MotorRuntimeConfig *config, uint8_t sequence[6])
{
    if (sequence == NULL) return;
    if (config == NULL || !MotorRuntimeConfig_HallSequenceValid(config->hall_sequence)) {
        memcpy(sequence, hall_encoding_from_position, 6U);
        return;
    }
    memcpy(sequence, config->hall_sequence, 6U);
}

static bool hall_raw_edge_legal(uint8_t from, uint8_t to)
{
    if (from < 1U || from > 6U || to < 1U || to > 6U || from == to) return false;
    const uint8_t changed = (uint8_t)(from ^ to);
    return changed != 0U && (changed & (uint8_t)(changed - 1U)) == 0U;
}

bool MotorRuntimeConfig_DetectHallSequenceFromTransitions(
    const MotorRuntimeConfig *config,
    const uint16_t transition_delta[64],
    uint8_t sequence[6])
{
    if (transition_delta == NULL || sequence == NULL) return false;

    /* Enam state Hall valid selalu berada pada satu cincin Gray-code fisik.
     * Counter directed membuat commissioning tetap valid bila poros sempat berhenti
     * atau sedikit berbalik. Yang wajib terbukti adalah keenam state dan >=5 dari
     * 6 adjacency fisik, bukan lima edge searah tanpa putus. */
    static const uint8_t canonical_cycle[6] = {1U, 3U, 2U, 6U, 4U, 5U};
    uint8_t previous[6];
    MotorRuntimeConfig_GetHallSequence(config, previous);

    uint8_t anchor = previous[0];
    if (anchor < 1U || anchor > 6U) anchor = 2U;

    uint8_t anchor_index = 0xFFU;
    for (uint8_t i = 0U; i < 6U; ++i) {
        if (canonical_cycle[i] == anchor) {
            anchor_index = i;
            break;
        }
    }
    if (anchor_index >= 6U) return false;

    uint8_t forward[6];
    uint8_t reverse[6];
    for (uint8_t i = 0U; i < 6U; ++i) {
        forward[i] = canonical_cycle[(uint8_t)((anchor_index + i) % 6U)];
        reverse[i] = canonical_cycle[(uint8_t)((anchor_index + 6U - i) % 6U)];
    }

    uint8_t seen_mask = 0U;
    uint8_t observed_adjacencies = 0U;
    uint32_t forward_score = 0U;
    uint32_t reverse_score = 0U;

    for (uint8_t i = 0U; i < 6U; ++i) {
        const uint8_t a = forward[i];
        const uint8_t b = forward[(uint8_t)((i + 1U) % 6U)];
        if (!hall_raw_edge_legal(a, b)) return false;

        const uint8_t ab_index = (uint8_t)(((a & 0x07U) << 3) | (b & 0x07U));
        const uint8_t ba_index = (uint8_t)(((b & 0x07U) << 3) | (a & 0x07U));
        const uint16_t ab = transition_delta[ab_index];
        const uint16_t ba = transition_delta[ba_index];

        if ((uint32_t)ab + (uint32_t)ba != 0U) {
            ++observed_adjacencies;
            seen_mask |= (uint8_t)(1U << (a - 1U));
            seen_mask |= (uint8_t)(1U << (b - 1U));
        }
        forward_score += ab;
        reverse_score += ba;
    }

    if (seen_mask != 0x3FU || observed_adjacencies < 5U) return false;

    const uint8_t *selected = forward;
    if (reverse_score > forward_score) {
        selected = reverse;
    } else if (reverse_score == forward_score && memcmp(previous, reverse, 6U) == 0) {
        /* Putar tangan bolak-balik dapat menghasilkan vote sama. Pertahankan arah
         * commissioning sebelumnya agar tidak membalik closed-loop tanpa bukti. */
        selected = reverse;
    }

    if (!MotorRuntimeConfig_HallSequenceValid(selected)) return false;
    memcpy(sequence, selected, 6U);
    return true;
}

static void hall_prepare_speed_scale(MotorSensorState *state, uint8_t pole_pairs)
{
    if (pole_pairs == 0U) {
        state->hall_speed_pole_pairs = 0U;
        state->hall_speed_coefficient_q4 = 0U;
        state->hall_speed_q4 = 0;
        return;
    }
    if (state->hall_speed_pole_pairs == pole_pairs &&
        state->hall_speed_coefficient_q4 != 0U) {
        return;
    }

    /* Satu edge Hall tervalidasi = 1/6 putaran listrik. Pole-pair berasal dari
     * parameter runtime motor, sehingga tidak ada asumsi jumlah pole-pair tetap. */
    state->hall_speed_pole_pairs = pole_pairs;
    state->hall_speed_coefficient_q4 = HALL_SPEED_NUMERATOR_Q4 / (uint32_t)pole_pairs;
    state->hall_speed_q4 = 0;
    state->hall_period_samples = 0U;
    state->hall_last_sector_period_samples = 0U;
    state->hall_last_direction = 0;
}

static uint16_t hall_sector_boundary_q16(uint8_t sector)
{
    /* Exact canonical 60-degree boundary rounded to nearest Q16 count. This is
     * electrical geometry, not a raw Hall wiring table. Called only at Hall edge. */
    sector %= 6U;
    return (uint16_t)((((uint32_t)sector * 65536U) + 3U) / 6U);
}

static uint16_t hall_sector_center_q16(uint8_t sector)
{
    /* Center = (sector + 1/2) / 6 turn. Keeping the division on Hall edge avoids
     * the four-count/revolution bias of truncating 65536/6 once. */
    sector %= 6U;
    return (uint16_t)(((((uint32_t)(2U * sector + 1U)) * 65536U) + 6U) / 12U);
}

static int16_t update_hall_position_and_speed(MotorSensorState *state,
                                              uint8_t current_position)
{
    if (current_position > 5U || state->hall_speed_coefficient_q4 == 0U) {
        state->hall_speed_q4 = 0;
        return 0;
    }

    if (!state->hall_initialized) {
        state->previous_hall_position = current_position;
        state->hall_initialized = true;
        state->hall_period_samples = 0U;
        state->hall_speed_q4 = 0;
        state->hall_last_direction = 0;
        state->hall_phase_step_q16 = 0;
        state->hall_phase_steps_remaining = 0U;
        state->hall_phase_accumulator_q16 =
            (uint32_t)hall_sector_center_q16(current_position) << 16;
        return 0;
    }

    if (state->hall_period_samples != UINT32_MAX) ++state->hall_period_samples;

    if (state->previous_hall_position == current_position) {
        /* Interpolasi VESC-style: sesudah edge, phase bergerak dari boundary
         * sektor dengan velocity hasil period edge terakhir. Tidak ada division
         * di sample biasa; hanya satu add + decrement. */
        if (state->hall_phase_steps_remaining != 0U) {
            state->hall_phase_accumulator_q16 += (uint32_t)state->hall_phase_step_q16;
            --state->hall_phase_steps_remaining;
        }

        if (state->hall_last_sector_period_samples != 0U) {
            const uint32_t max_base = UINT32_MAX / HALL_SPEED_ZERO_PERIOD_MULTIPLIER;
            const uint32_t zero_after =
                (state->hall_last_sector_period_samples > max_base)
                    ? UINT32_MAX
                    : state->hall_last_sector_period_samples * HALL_SPEED_ZERO_PERIOD_MULTIPLIER;
            if (state->hall_period_samples >= zero_after) {
                state->hall_speed_q4 = 0;
                state->hall_phase_step_q16 = 0;
                state->hall_phase_steps_remaining = 0U;
                /* Pada standstill gunakan center state Hall terkalibrasi; jangan
                 * membiarkan extrapolasi berhenti di boundary 60 derajat. */
                state->hall_phase_accumulator_q16 =
                    (uint32_t)hall_sector_center_q16(current_position) << 16;
            }
        }
        return state->hall_speed_q4;
    }

    const uint8_t previous_position = state->previous_hall_position;
    const int8_t sector_delta = hall_position_delta(previous_position, current_position);
    state->previous_hall_position = current_position;

    if (sector_delta == 0) {
        state->hall_speed_q4 = 0;
        state->hall_last_direction = 0;
        state->hall_period_samples = 0U;
        state->hall_phase_step_q16 = 0;
        state->hall_phase_steps_remaining = 0U;
        state->hall_phase_accumulator_q16 =
            (uint32_t)hall_sector_center_q16(current_position) << 16;
        return 0;
    }

    state->position_ticks = add_position_saturated(state->position_ticks, sector_delta);
    const int8_t previous_direction = state->hall_last_direction;
    state->hall_last_direction = sector_delta > 0 ? 1 : -1;

    uint32_t period = state->hall_period_samples;
    if (period == 0U) period = 1U;
    uint32_t sectors = (uint32_t)(sector_delta < 0 ? -sector_delta : sector_delta);
    if (sectors == 0U) sectors = 1U;

    /* hall_position_delta() only returns |delta| 1 or 2 for a usable edge.
     * Convert a skipped-two-sector sample to one-sector period with a shift,
     * avoiding one generic divide in the ISR edge path. */
    uint32_t sector_period = (sectors == 2U) ? (period >> 1) : period;
    if (sector_period == 0U) sector_period = 1U;

    int32_t raw_speed_q4 =
        (int32_t)(state->hall_speed_coefficient_q4 / sector_period);
    if (raw_speed_q4 > INT16_MAX) raw_speed_q4 = INT16_MAX;
    if (state->hall_last_direction < 0) raw_speed_q4 = -raw_speed_q4;

    /* Never average opposite signs across a direction reversal. A +RPM sample
     * blended with a -RPM sample can become zero for one whole Hall sector and
     * destabilize the speed loop exactly when the command changes direction. */
    if (state->hall_last_sector_period_samples == 0U ||
        state->hall_speed_q4 == 0 ||
        (previous_direction != 0 && previous_direction != state->hall_last_direction)) {
        state->hall_speed_q4 = (int16_t)raw_speed_q4;
    } else {
        state->hall_speed_q4 = (int16_t)((int32_t)state->hall_speed_q4 +
            ((raw_speed_q4 - (int32_t)state->hall_speed_q4) / 2));
    }
    state->hall_last_sector_period_samples = sector_period;
    state->hall_period_samples = 0U;

    /* Hall table berasal dari hall_sequence hasil commissioning. Phase yang
     * dipublikasikan FOC tidak pernah mendecode raw wiring dengan tabel global.
     * Seperti VESC, tepat saat transition phase diletakkan di boundary antara
     * center Hall lama dan baru, lalu diinterpolasi menuju boundary berikutnya. */
    const uint8_t next_position = state->hall_last_direction > 0
        ? (uint8_t)((current_position + 1U) % 6U)
        : (uint8_t)((current_position + 5U) % 6U);
    const uint16_t boundary = state->hall_last_direction > 0
        ? hall_sector_boundary_q16(current_position)
        : hall_sector_boundary_q16((uint8_t)((current_position + 1U) % 6U));
    const uint16_t next_boundary = state->hall_last_direction > 0
        ? hall_sector_boundary_q16(next_position)
        : hall_sector_boundary_q16(current_position);

    state->hall_phase_accumulator_q16 = (uint32_t)boundary << 16;

    uint32_t phase_distance;
    if (state->hall_last_direction > 0) {
        phase_distance = (uint16_t)(next_boundary - boundary);
    } else {
        phase_distance = (uint16_t)(boundary - next_boundary);
    }
    int32_t phase_step = (int32_t)((phase_distance << 16) / sector_period);
    if (state->hall_last_direction < 0) phase_step = -phase_step;
    state->hall_phase_step_q16 = phase_step;
    state->hall_phase_steps_remaining = sector_period;

    return state->hall_speed_q4;
}

static void motor_sensor_on_encoder_edge(const MotorRuntimeConfig *config, MotorSensorState *state,
                                         uint8_t encoder_a, uint8_t encoder_b)
{
    state->encoder_last_delta = 0;
    if (config->sensor_type != MOTOR_SENSOR_ENCODER_AB) {
        return;
    }

    /* Encoder A/B secara eksplisit berasal dari Hall-U=A dan Hall-V=B. Kedua bit
     * dimask agar level selain 0/1 dari caller tidak dapat merusak index table. */
    const uint8_t current_ab = (uint8_t)(((encoder_a & 1U) << 1) | (encoder_b & 1U));
    if (!state->encoder_initialized) {
        state->previous_encoder_ab = current_ab;
        state->encoder_initialized = true;
        state->encoder_last_raw_direction = 0;
        state->encoder_direction_confidence = 0U;
        return;
    }

    const uint8_t previous_ab = state->previous_encoder_ab;
    if (current_ab == previous_ab) {
        return;
    }

    const uint8_t transition = (uint8_t)((previous_ab << 2) | current_ab);
    uint16_t *transition_count = &state->encoder_transition_counts[transition & 0x0FU];
    if (*transition_count != UINT16_MAX) ++(*transition_count);
    /* MotorSensor_Update() menjamin runtime LUT sudah diprecompute sebelum edge
     * diproses. Hot path encoder cukup satu indexed load; tidak ada scan sequence,
     * validasi ulang, atau fallback branch di ISR. */
    int8_t raw_delta = state->encoder_transition_lut[transition & 0x0FU];
    state->previous_encoder_ab = current_ab;

    /* Polling 16 kHz dapat melewatkan tepat satu state di antara dua sampel.
     * Diagonal jump mengubah dua bit sekaligus dan arah intrinsiknya ambigu.
     * Jangan menebak saat startup/noise: recovery hanya diizinkan sesudah arah
     * dikonfirmasi minimal dua edge valid berurutan. Dengan demikian 00->11,
     * misalnya, direkonstruksi menjadi +/-2 hanya ketika arah sebelumnya kuat. */
    if (raw_delta == 0) {
        if (state->encoder_last_raw_direction != 0 &&
            state->encoder_direction_confidence >= 2U) {
            raw_delta = (int8_t)(2 * state->encoder_last_raw_direction);
            if (state->encoder_recovered_edges <= UINT32_MAX - 2U)
                state->encoder_recovered_edges += 2U;
            else
                state->encoder_recovered_edges = UINT32_MAX;
            if (state->encoder_valid_edges <= UINT32_MAX - 2U)
                state->encoder_valid_edges += 2U;
            else
                state->encoder_valid_edges = UINT32_MAX;
        } else {
            if (state->encoder_invalid_transitions != UINT32_MAX)
                ++state->encoder_invalid_transitions;
            /* Recovery berikutnya harus membangun ulang keyakinan arah dari edge
             * satu-bit yang nyata, agar burst noise tidak diubah menjadi gerak. */
            state->encoder_last_raw_direction = 0;
            state->encoder_direction_confidence = 0U;
            return;
        }
    } else {
        if (raw_delta == state->encoder_last_raw_direction) {
            if (state->encoder_direction_confidence < UINT8_MAX)
                ++state->encoder_direction_confidence;
        } else {
            state->encoder_last_raw_direction = raw_delta;
            state->encoder_direction_confidence = 1U;
        }
        if (state->encoder_valid_edges != UINT32_MAX)
            ++state->encoder_valid_edges;
    }

    /* Arah sudah dibake ke encoder_transition_lut saat LOAD/APPLY/Auto Detect. */
    state->encoder_last_delta = raw_delta;
    state->position_ticks = add_position_saturated(state->position_ticks, raw_delta);
}

static int16_t encoder_speed_update_q4(MotorSensorState *state,
                                       int32_t position_ticks)
{
    if (!state->encoder_speed_initialized) {
        state->encoder_speed_reference_ticks = position_ticks;
        state->encoder_speed_window_count = 0U;
        state->encoder_speed_q4 = 0;
        state->encoder_speed_initialized = true;
        return 0;
    }

    if (++state->encoder_speed_window_count < ENCODER_SPEED_WINDOW_SAMPLES) {
        return state->encoder_speed_q4;
    }

    state->encoder_speed_window_count = 0U;

    /* Karena decoder dipolling satu kali per sample dan recovery maksimal +/-2,
     * perubahan fisik maksimum pada satu window 64 sample adalah 128 count.
     * Clamp defensive juga melindungi jika state diubah asinkron saat debug. */
    const int32_t reference = state->encoder_speed_reference_ticks;
    state->encoder_speed_reference_ticks = position_ticks;
    const int32_t max_delta = (int32_t)(ENCODER_SPEED_WINDOW_SAMPLES *
                                        ENCODER_MAX_COUNTS_PER_SAMPLE);
    int32_t delta;
    if (position_ticks >= reference) {
        const uint32_t diff = (uint32_t)position_ticks - (uint32_t)reference;
        delta = (diff > (uint32_t)max_delta) ? max_delta : (int32_t)diff;
    } else {
        const uint32_t diff = (uint32_t)reference - (uint32_t)position_ticks;
        delta = (diff > (uint32_t)max_delta) ? -max_delta : -(int32_t)diff;
    }

    /* Q7 cukup presisi untuk reciprocal CPR dan menjamin product muat int32_t:
     * max scale = (240000/4)<<7 = 7,680,000; 128*scale < 1e9. */
    int32_t scaled = delta * (int32_t)state->encoder_speed_per_count_q7;
    int32_t raw_q4;
    const int32_t rounding = (int32_t)(1U << (ENCODER_SPEED_SCALE_FRAC_BITS - 1U));
    if (scaled >= 0) raw_q4 = (scaled + rounding) >> ENCODER_SPEED_SCALE_FRAC_BITS;
    else raw_q4 = -(((-scaled) + rounding) >> ENCODER_SPEED_SCALE_FRAC_BITS);
    if (raw_q4 > INT16_MAX) raw_q4 = INT16_MAX;
    if (raw_q4 < INT16_MIN) raw_q4 = INT16_MIN;

    /* IIR 1/2 tetap fixed-point dan compiler mengubah /2 menjadi shift. */
    const int32_t filtered = (int32_t)state->encoder_speed_q4 +
                             ((raw_q4 - (int32_t)state->encoder_speed_q4) / 2);
    state->encoder_speed_q4 = (int16_t)filtered;
    return state->encoder_speed_q4;
}



/* Menyiapkan skala encoder hanya saat CPR/pole-pair berubah. Pembagian runtime
 * terjadi sekali pada perubahan konfigurasi, BUKAN pada setiap ISR 16 kHz. */
static void encoder_prepare_fast_scale(MotorSensorState *state,
                                       uint16_t encoder_cpr,
                                       uint8_t pole_pairs)
{
    if (encoder_cpr < MOTOR_ENCODER_CPR_MIN) encoder_cpr = MOTOR_ENCODER_CPR_MIN;
    if (pole_pairs == 0U) pole_pairs = 1U;
    if (state->encoder_scale_cpr == encoder_cpr &&
        state->encoder_scale_pole_pairs == pole_pairs &&
        state->encoder_mechanical_scale_q16 != 0U &&
        state->encoder_electrical_scale_q16 != 0U &&
        state->encoder_speed_per_count_q7 != 0U) {
        return;
    }

    state->encoder_scale_cpr = encoder_cpr;
    state->encoder_scale_pole_pairs = pole_pairs;
    /* Q16 dipilih sengaja: seluruh perkalian angle tetap muat uint32_t untuk
     * CPR 4..65535, jauh lebih ringan daripada int64 di ISR. Tambahkan half
     * divisor supaya reciprocal dibulatkan ke terdekat dan tidak bias ke bawah. */
    state->encoder_mechanical_scale_q16 =
        (uint32_t)((((uint32_t)5760U << 16) + (encoder_cpr / 2U)) / encoder_cpr);
    state->encoder_electrical_scale_q16 =
        (uint32_t)((((uint32_t)23040U << 16) + (encoder_cpr / 2U)) / encoder_cpr);
    state->encoder_speed_per_count_q7 =
        (uint32_t)((((uint32_t)ENCODER_SPEED_NUMERATOR_Q4 << ENCODER_SPEED_SCALE_FRAC_BITS) +
                    (encoder_cpr / 2U)) / encoder_cpr);

    /* Electrical phase increment per encoder count.
     *
     *   pole_pairs * 65536 = quotient * CPR + remainder
     *
     * quotient is added on every A/B edge. The remainder accumulator injects
     * the extra phase count exactly when it crosses CPR. This bounded integer
     * update keeps the ISR division-free and prevents cumulative phase drift. */
    const uint32_t phase_numerator = 65536U * (uint32_t)pole_pairs;
    state->encoder_electrical_step_q16 =
        (uint16_t)(phase_numerator / (uint32_t)encoder_cpr);
    state->encoder_cpr_recip_q32 =
        (uint32_t)(((uint64_t)1U << 32) / (uint32_t)encoder_cpr);
    state->encoder_electrical_step_remainder =
        (uint16_t)(phase_numerator % (uint32_t)encoder_cpr);
    state->encoder_electrical_remainder_accum = 0U;

    state->encoder_mechanical_phase_count = 0U;
    state->encoder_electrical_phase_q16 = 0U;
    state->encoder_electrical_zero_q16 = 0U;
}

static void motor_sensor_build_runtime_luts(const MotorRuntimeConfig *config,
                                            MotorSensorState *state)
{
    /* -------- Encoder transition LUT 4x4 --------
     * Seluruh 16 entry diisi sekali. Sequence commissioning menentukan +1/-1
     * secara langsung; config legacy tanpa proof memakai tabel default + invert. */
    memset(state->encoder_transition_lut, 0, sizeof(state->encoder_transition_lut));
    if (config != NULL && config->encoder_sequence_valid != 0U &&
        MotorRuntimeConfig_EncoderSequenceValid(config->encoder_sequence)) {
        /* Sequence hasil Auto Detect sudah didefinisikan pada arah FOC-positive.
         * Jangan terapkan sensor_inverted kedua kali di sini. Jika user mengubah
         * sensor_inverted, firmware membatalkan proof dan closed-loop dikunci
         * sampai Auto Detect baru selesai. */
        for (uint8_t i = 0U; i < 4U; ++i) {
            const uint8_t from = config->encoder_sequence[i] & 0x03U;
            const uint8_t to = config->encoder_sequence[(uint8_t)((i + 1U) & 0x03U)] & 0x03U;
            state->encoder_transition_lut[(uint8_t)((from << 2) | to)] = 1;
            state->encoder_transition_lut[(uint8_t)((to << 2) | from)] = -1;
        }
    } else {
        for (uint8_t i = 0U; i < 16U; ++i) {
            int8_t delta = encoder_delta_table[i];
            if (config != NULL && config->sensor_inverted != 0U) delta = (int8_t)-delta;
            state->encoder_transition_lut[i] = delta;
        }
    }

    /* -------- Hall raw -> sector/canonical LUT 8 entry --------
     * Hall sequence tetap 100% berasal dari config aktif. Inversi sensor dibake
     * ke LUT ini agar ISR cukup melakukan satu indexed load. */
    for (uint8_t raw = 0U; raw < 8U; ++raw) {
        state->hall_raw_to_position[raw] = -1;
        state->hall_raw_to_normalized[raw] = 0U;
    }
    const uint8_t *hall_sequence = hall_encoding_from_position;
    if (config != NULL && config->hall_lut_valid != 0U &&
        MotorRuntimeConfig_HallSequenceValid(config->hall_sequence)) {
        hall_sequence = config->hall_sequence;
    }
    for (uint8_t position = 0U; position < 6U; ++position) {
        const uint8_t raw = hall_sequence[position] & 0x07U;
        uint8_t normalized_position = position;
        if (config != NULL && config->sensor_inverted != 0U) {
            normalized_position = (position == 0U) ? 0U : (uint8_t)(6U - position);
        }
        state->hall_raw_to_position[raw] = (int8_t)normalized_position;
        state->hall_raw_to_normalized[raw] = hall_encoding_from_position[normalized_position];
    }
    state->runtime_lut_prepared = true;
}

void MotorSensor_PrepareRuntime(const MotorRuntimeConfig *config,
                                MotorSensorState *state,
                                uint8_t pole_pairs)
{
    if (config == NULL || state == NULL) return;
    motor_sensor_build_runtime_luts(config, state);
    const uint8_t encoder_ratio = config->encoder_ratio != 0U
        ? config->encoder_ratio : (pole_pairs != 0U ? pole_pairs : 1U);
    encoder_prepare_fast_scale(state, config->encoder_cpr, encoder_ratio);
    state->encoder_offset_phase_q16 = (uint16_t)(((uint32_t)config->encoder_offset_deg * 65536U + 180U) / 360U);
    hall_prepare_speed_scale(state, pole_pairs);
    state->hall_phase_step_q16 = 0;
    state->hall_phase_steps_remaining = 0U;
}

static uint32_t encoder_div_cpr_fast(uint32_t value, const MotorSensorState *state)
{
    /* Barrett-style reciprocal division. reciprocal=floor(2^32/CPR), sehingga
     * estimate tidak pernah lebih besar dari floor(value/CPR). Untuk value pada
     * jalur ini (<33*CPR) koreksi paling banyak satu increment. */
    const uint32_t cpr = state->encoder_scale_cpr;
    uint32_t q = (uint32_t)(((uint64_t)value * state->encoder_cpr_recip_q32) >> 32);
    if ((value - q * cpr) >= cpr) ++q;
    return q;
}

static uint32_t encoder_ceil_div_cpr_fast(uint32_t value, const MotorSensorState *state)
{
    if (value == 0U) return 0U;
    return encoder_div_cpr_fast(value + (uint32_t)state->encoder_scale_cpr - 1U, state);
}

static void encoder_advance_fast_phase(MotorSensorState *state, int32_t delta)
{
    if (delta == 0 || state->encoder_scale_cpr < MOTOR_ENCODER_CPR_MIN) return;

    const uint32_t cpr = state->encoder_scale_cpr;
    const uint32_t phase_step = state->encoder_electrical_step_q16;
    const uint32_t phase_rem = state->encoder_electrical_step_remainder;
    const uint32_t steps = (uint32_t)((delta < 0) ? -delta : delta);

    /* TIM4 is authoritative and delta is clamped to <=32 before this function.
     * Process the whole batch in O(1): no per-edge loop and no runtime integer
     * division. The quotient/remainder arithmetic is exactly equivalent to the
     * former one-edge-at-a-time Bresenham update. */
    if (delta > 0) {
        uint32_t mech = (uint32_t)state->encoder_mechanical_phase_count + steps;
        if (mech >= cpr) {
            const uint32_t wraps = encoder_div_cpr_fast(mech, state);
            mech -= wraps * cpr;
        }
        state->encoder_mechanical_phase_count = (uint16_t)mech;

        uint32_t rem = (uint32_t)state->encoder_electrical_remainder_accum +
                       steps * phase_rem;
        uint32_t extra = 0U;
        if (rem >= cpr) {
            extra = encoder_div_cpr_fast(rem, state);
            rem -= extra * cpr;
        }
        state->encoder_electrical_remainder_accum = (uint16_t)rem;
        state->encoder_electrical_phase_q16 =
            (uint16_t)(state->encoder_electrical_phase_q16 +
                       steps * phase_step + extra);
    } else {
        uint32_t mech = state->encoder_mechanical_phase_count;
        if (mech >= steps) {
            mech -= steps;
        } else {
            const uint32_t borrow = encoder_ceil_div_cpr_fast(steps - mech, state);
            mech += borrow * cpr - steps;
        }
        state->encoder_mechanical_phase_count = (uint16_t)mech;

        const uint32_t sub = steps * phase_rem;
        uint32_t rem = state->encoder_electrical_remainder_accum;
        uint32_t borrow = 0U;
        if (rem >= sub) {
            rem -= sub;
        } else {
            borrow = encoder_ceil_div_cpr_fast(sub - rem, state);
            rem += borrow * cpr - sub;
        }
        state->encoder_electrical_remainder_accum = (uint16_t)rem;
        state->encoder_electrical_phase_q16 =
            (uint16_t)(state->encoder_electrical_phase_q16 -
                       steps * phase_step - borrow);
    }
}

static int16_t encoder_fast_mechanical_angle_q4(const MotorSensorState *state)
{
    const uint32_t product = (uint32_t)state->encoder_mechanical_phase_count *
                             state->encoder_mechanical_scale_q16;
    return (int16_t)((product + 0x8000U) >> 16);
}

static uint16_t encoder_fast_electrical_phase_q16(const MotorSensorState *state)
{
    uint16_t phase = state->encoder_electrical_phase_q16;
    if (state->encoder_electrical_aligned) {
        phase = (uint16_t)(phase - state->encoder_electrical_zero_q16);
    }
    return (uint16_t)(phase + state->encoder_offset_phase_q16);
}

uint16_t MotorSensor_SyncEncoderElectricalPhase(MotorSensorState *state,
                                                uint16_t desired_phase_q16)
{
    if (state == NULL) return 0U;
    /* Reader equation:
     *   theta_e = raw_phase - zero + configured_offset
     * therefore:
     *   zero = raw_phase + configured_offset - desired.
     * This is the missing term in V11..V18 and is essential for AB encoders
     * after every power-cycle / commissioning alignment. */
    state->encoder_electrical_zero_q16 =
        (uint16_t)(state->encoder_electrical_phase_q16 +
                   state->encoder_offset_phase_q16 -
                   desired_phase_q16);
    state->encoder_electrical_aligned = true;
    return encoder_fast_electrical_phase_q16(state);
}

static int16_t electrical_phase_q16_to_q6(uint16_t phase)
{
    /* 23040/65536 = 45/128. Multiply+shift is exact mapping without division. */
    return (int16_t)(((uint32_t)phase * 45U + 64U) >> 7);
}

void MotorRuntimeConfig_SetDefaults(MotorRuntimeConfig *config)
{
    config->motor_inverted = 0U;
    config->sensor_type = MOTOR_SENSOR_HALL_UVW;
    config->sensor_inverted = 0U;
    config->encoder_cpr = MOTOR_ENCODER_CPR_DEFAULT;
    config->encoder_offset_deg = 0U;
    config->encoder_ratio = 0U;
    config->encoder_calibrated = 0U;
    memcpy(config->encoder_sequence, encoder_sequence_default, 4U);
    config->encoder_sequence_valid = 0U;
    config->hall_calibrated = 0U;
    memcpy(config->hall_sequence, hall_encoding_from_position, 6U);
    config->hall_lut_valid = 1U;
}

void MotorRuntimeConfig_ResetHallCalibration(MotorRuntimeConfig *config)
{
    if (config == NULL) return;
    config->hall_calibrated = 0U;
    memcpy(config->hall_sequence, hall_encoding_from_position, 6U);
    config->hall_lut_valid = 1U;
}

void MotorRuntimeConfig_ResetEncoderCalibration(MotorRuntimeConfig *config)
{
    if (config == NULL) return;
    config->encoder_calibrated = 0U;
    memcpy(config->encoder_sequence, encoder_sequence_default, 4U);
    config->encoder_sequence_valid = 0U;
}

bool MotorRuntimeConfig_IsValid(const MotorRuntimeConfig *config)
{
    if (config->motor_inverted > 1U || config->sensor_inverted > 1U) return false;
    if (config->sensor_type > MOTOR_SENSOR_ENCODER_AB) return false;
    if (config->encoder_cpr < MOTOR_ENCODER_CPR_MIN) return false;
    if (config->encoder_offset_deg >= 360U) return false;
    if (config->encoder_ratio > 60U) return false;
    if (config->encoder_calibrated > 1U) return false;
    if (config->encoder_sequence_valid > 1U) return false;
    if (config->encoder_sequence_valid != 0U &&
        !MotorRuntimeConfig_EncoderSequenceValid(config->encoder_sequence)) return false;
    if (config->encoder_calibrated != 0U &&
        (config->encoder_sequence_valid == 0U ||
         !MotorRuntimeConfig_EncoderSequenceValid(config->encoder_sequence))) return false;
    if (config->hall_calibrated > 1U) return false;
    if (config->hall_lut_valid > 1U) return false;
    if (config->hall_lut_valid != 0U && !MotorRuntimeConfig_HallSequenceValid(config->hall_sequence)) return false;
    if (config->hall_calibrated != 0U &&
        (config->hall_lut_valid == 0U ||
         !MotorRuntimeConfig_HallSequenceValid(config->hall_sequence))) return false;
    return true;
}

uint16_t MotorRuntimeConfig_PackWord(const MotorRuntimeConfig *config)
{
    uint16_t word = 0U;
    if (config->motor_inverted) word |= MOTOR_CONFIG_FLAG_MOTOR_INVERTED;
    if (config->sensor_inverted) word |= MOTOR_CONFIG_FLAG_SENSOR_INVERTED;
    if (config->sensor_type == MOTOR_SENSOR_ENCODER_AB) word |= MOTOR_CONFIG_FLAG_ENCODER_AB;
    if (config->encoder_calibrated) word |= MOTOR_CONFIG_FLAG_ENCODER_CALIBRATED;
    if (config->hall_calibrated) word |= MOTOR_CONFIG_FLAG_HALL_CALIBRATED;
    word |= (uint16_t)((config->encoder_offset_deg & 0x01FFU) << MOTOR_CONFIG_OFFSET_SHIFT);
    return word;
}

bool MotorRuntimeConfig_UnpackWord(uint16_t config_word, uint16_t encoder_cpr,
                                   MotorRuntimeConfig *config)
{
    const uint16_t supported_mask = MOTOR_CONFIG_FLAG_MOTOR_INVERTED |
                                    MOTOR_CONFIG_FLAG_SENSOR_INVERTED |
                                    MOTOR_CONFIG_FLAG_ENCODER_AB |
                                    MOTOR_CONFIG_FLAG_ENCODER_CALIBRATED |
                                    MOTOR_CONFIG_FLAG_HALL_CALIBRATED |
                                    MOTOR_CONFIG_OFFSET_MASK;
    if ((config_word & (uint16_t)~supported_mask) != 0U) {
        return false;
    }

    MotorRuntimeConfig decoded;
    MotorRuntimeConfig_SetDefaults(&decoded);
    decoded.motor_inverted = (config_word & MOTOR_CONFIG_FLAG_MOTOR_INVERTED) ? 1U : 0U;
    decoded.sensor_inverted = (config_word & MOTOR_CONFIG_FLAG_SENSOR_INVERTED) ? 1U : 0U;
    decoded.sensor_type = (config_word & MOTOR_CONFIG_FLAG_ENCODER_AB)
        ? MOTOR_SENSOR_ENCODER_AB : MOTOR_SENSOR_HALL_UVW;
    decoded.encoder_offset_deg = (uint16_t)((config_word & MOTOR_CONFIG_OFFSET_MASK) >> MOTOR_CONFIG_OFFSET_SHIFT);
    decoded.encoder_calibrated = (config_word & MOTOR_CONFIG_FLAG_ENCODER_CALIBRATED) ? 1U : 0U;
    decoded.hall_calibrated = (config_word & MOTOR_CONFIG_FLAG_HALL_CALIBRATED) ? 1U : 0U;
    decoded.encoder_cpr = encoder_cpr;

    /* Sequence Hall/Encoder dimuat terpisah dari config word saat EEPROM boot.
     * Jangan validasi proof lintas-field di sini karena sequence belum tersedia. */
    if (decoded.encoder_cpr < MOTOR_ENCODER_CPR_MIN ||
        decoded.encoder_offset_deg >= 360U) return false;
    *config = decoded;
    return true;
}

void MotorSensor_Reset(MotorSensorState *state)
{
    memset(state, 0, sizeof(*state));
}

void MotorSensor_Zero(MotorSensorState *state)
{
    state->position_ticks = 0;
    state->encoder_mechanical_phase_count = 0U;
    state->encoder_last_delta = 0;
    state->encoder_last_raw_direction = 0;
    state->encoder_direction_confidence = 0U;
    /* Mechanical zero/homing must never move the electrical reference. The
     * electrical phase accumulator and encoder_electrical_zero_q16 remain
     * untouched; only mechanical position/angle is zeroed. */
    state->encoder_speed_reference_ticks = 0;
    state->encoder_speed_window_count = 0U;
    state->encoder_speed_q4 = 0;
    state->encoder_speed_initialized = false;
}


void MotorSensor_UpdateHardwareEncoder(const MotorRuntimeConfig *config,
                                       MotorSensorState *state,
                                       int32_t hardware_count,
                                       uint8_t raw_a, uint8_t raw_b,
                                       uint8_t pole_pairs,
                                       MotorSensorSample *sample)
{
    if (config == NULL || state == NULL || sample == NULL) return;
    if (!state->runtime_lut_prepared) MotorSensor_PrepareRuntime(config, state, pole_pairs);

    sample->raw_hall_encoding = 0U;
    sample->hall_encoding = 0U;
    sample->encoder_ab = (uint8_t)(((raw_a & 1U) << 1) | (raw_b & 1U));

    /* Keep transition diagnostics for commissioning, but TIM4 is authoritative for count. */
    if (!state->encoder_initialized) {
        state->previous_encoder_ab = sample->encoder_ab;
        state->encoder_initialized = true;
    } else if (sample->encoder_ab != state->previous_encoder_ab) {
        const uint8_t tr = (uint8_t)((state->previous_encoder_ab << 2) | sample->encoder_ab);
        if (state->encoder_transition_counts[tr] != UINT16_MAX) ++state->encoder_transition_counts[tr];
        state->previous_encoder_ab = sample->encoder_ab;
    }

    if (!state->encoder_hw_count_initialized) {
        state->encoder_hw_last_count = hardware_count;
        state->encoder_hw_count_initialized = true;
        state->position_ticks = hardware_count;
        state->encoder_speed_reference_ticks = hardware_count;
    }

    int32_t delta32 = hardware_count - state->encoder_hw_last_count;
    state->encoder_hw_last_count = hardware_count;
    if (config->sensor_inverted) delta32 = -delta32;
    /* An implausibly large per-FOC-sample jump means configuration/noise. Do not
     * spend unbounded ISR time walking it; reject this sample and preserve phase. */
    if (delta32 > 32 || delta32 < -32) {
        if (state->encoder_invalid_transitions != UINT32_MAX) ++state->encoder_invalid_transitions;
        delta32 = 0;
    }
    if (delta32 != 0) {
        /* TIM4 already decoded every A/B edge. Integrate its signed delta in one
         * bounded arithmetic step; phase batching below is O(1) and exact. */
        state->position_ticks = add_position_delta_saturated(state->position_ticks, delta32);
        encoder_advance_fast_phase(state, delta32);
        const uint32_t edges = (uint32_t)(delta32 < 0 ? -delta32 : delta32);
        if (state->encoder_valid_edges <= UINT32_MAX - edges)
            state->encoder_valid_edges += edges;
        else state->encoder_valid_edges = UINT32_MAX;
    }

    const bool feedback_ready = config->encoder_calibrated != 0U &&
        config->encoder_sequence_valid != 0U && state->encoder_electrical_aligned;
    sample->feedback_valid = feedback_ready ? 1U : 0U;
    sample->electrical_phase_q16 = encoder_fast_electrical_phase_q16(state);
    sample->electrical_angle_q6 = electrical_phase_q16_to_q6(sample->electrical_phase_q16);
    sample->mechanical_angle_q4 = encoder_fast_mechanical_angle_q4(state);
    sample->mechanical_speed_q4 = encoder_speed_update_q4(state, state->position_ticks);
    sample->position_ticks = state->position_ticks;
}

void MotorSensor_Update(const MotorRuntimeConfig *config,
                        MotorSensorState *state,
                        uint8_t raw_u,
                        uint8_t raw_v,
                        uint8_t raw_w,
                        uint8_t pole_pairs,
                        MotorSensorSample *sample)
{
    /* Normal runtime sudah diprepare saat LOAD/APPLY/kalibrasi. Guard ini hanya
     * fallback aman untuk boot/test langsung setelah Reset; setelah itu satu branch. */
    if (!state->runtime_lut_prepared) {
        MotorSensor_PrepareRuntime(config, state, pole_pairs);
    }

    /* Pisahkan OBSERVATION dari CONTROL PROOF.
     * Sensor harus tetap menghitung position/speed untuk telemetry, commissioning,
     * dan diagnosis walaupun Auto Detect belum membuktikan closed-loop FOC.
     * feedback_valid HANYA berarti feedback aman dipakai controller. */
    const bool encoder_observation_ready =
        config->sensor_type == MOTOR_SENSOR_ENCODER_AB &&
        config->encoder_cpr >= MOTOR_ENCODER_CPR_MIN;
    const bool encoder_feedback_ready =
        encoder_observation_ready &&
        config->encoder_calibrated != 0U &&
        config->encoder_sequence_valid != 0U &&
        state->encoder_electrical_aligned;
    const bool hall_feedback_ready =
        config->sensor_type == MOTOR_SENSOR_HALL_UVW &&
        config->hall_calibrated != 0U &&
        config->hall_lut_valid != 0U;
    sample->feedback_valid = encoder_feedback_ready ? 1U : 0U;
    sample->electrical_phase_q16 = 0U;
    sample->electrical_angle_q6 = 0;

    if (config->sensor_type == MOTOR_SENSOR_ENCODER_AB) {
        /* Encoder A/B memakai EKSKLUSIF pin Hall-U sebagai A dan Hall-V sebagai B.
         * Hall-W sengaja tidak dibaca/didecode pada mode ini. raw_w diabaikan total
         * agar noise/level pada pin W tidak pernah memengaruhi count, angle, speed,
         * position, calibration, health-check, maupun telemetry encoder. */
        (void)raw_w;
        sample->raw_hall_encoding = 0U;
        sample->encoder_ab = (uint8_t)((raw_u << 1) | raw_v);

        /* PENTING: Encoder dibaca PERSIS pada jalur sampling Hall, yaitu sekali
         * setiap ISR kontrol motor (~16 kHz). Tidak ada EXTI/timer tambahan.
         * Fungsi decoder quadrature hanya membandingkan state U/V saat ini dengan
         * state U/V dari sampel kontrol sebelumnya. Hall-W tidak pernah ikut. */
        /* CPR/pole-pair scaling sudah diprecompute oleh MotorSensor_PrepareRuntime()
         * saat LOAD/APPLY/kalibrasi. Jangan ulangi pengecekan scale di 16 kHz. */
        motor_sensor_on_encoder_edge(config, state, raw_u, raw_v);
        encoder_advance_fast_phase(state, state->encoder_last_delta);

        /* Jangan pernah zero/return hanya karena closed-loop proof belum ada.
         * Decoder raw tetap valid sebagai sensor observation; proof hanya menjaga
         * electrical feedback supaya tidak masuk FOC sebelum Auto Detect selesai. */
        const int32_t encoder_position = state->position_ticks;
        sample->electrical_phase_q16 = encoder_fast_electrical_phase_q16(state);
        sample->electrical_angle_q6 = electrical_phase_q16_to_q6(sample->electrical_phase_q16);
        /* Encoder backend langsung mempublikasikan phase listrik terkalibrasi.
         * FOC tidak membuat Hall sintetis/estimator kedua dari nilai ini. */
        sample->hall_encoding = 0U;
        sample->mechanical_angle_q4 = encoder_fast_mechanical_angle_q4(state);
        sample->mechanical_speed_q4 = encoder_speed_update_q4(
            state, encoder_position);
        sample->position_ticks = encoder_position;
    } else {
        /* Hanya Hall mode yang membentuk kode UVW 3-bit dan membaca W. */
        sample->raw_hall_encoding = (uint8_t)((raw_u << 2) | (raw_v << 1) | raw_w);
        sample->encoder_ab = 0U;

        /* Rekam directed transition RAW di ISR, bukan main-loop. Counter ini hanya
         * diagnostic/commissioning sehingga saturating uint16 cukup dan tidak
         * menambah pembagian/modulo pada current-loop. */
        const uint8_t raw_hall = sample->raw_hall_encoding & 0x07U;
        const int8_t mapped_position = state->hall_raw_to_position[raw_hall];
        const bool hall_observation_ready = mapped_position >= 0;
        const bool hall_sample_ready = hall_feedback_ready && hall_observation_ready;
        sample->feedback_valid = hall_sample_ready ? 1U : 0U;
        if (!state->hall_raw_initialized) {
            state->previous_raw_hall_encoding = raw_hall;
            state->hall_raw_initialized = true;
        } else if (raw_hall != state->previous_raw_hall_encoding) {
            const uint8_t index = (uint8_t)(((state->previous_raw_hall_encoding & 0x07U) << 3) | raw_hall);
            if (state->hall_transition_counts[index] != UINT16_MAX)
                ++state->hall_transition_counts[index];
            state->previous_raw_hall_encoding = raw_hall;
        }

        if (hall_observation_ready) {
            /* Observation Hall tetap berjalan walau closed-loop proof belum ada.
             * Mapping runtime berasal dari sequence aktif bila valid, atau default
             * commissioning map bila belum ada. feedback_valid tetap 0 sampai
             * Auto Detect membuktikan mapping aman untuk FOC. */
            sample->hall_encoding = state->hall_raw_to_normalized[raw_hall];
            sample->mechanical_speed_q4 = update_hall_position_and_speed(
                state, (uint8_t)mapped_position);
            sample->electrical_phase_q16 =
                (uint16_t)(state->hall_phase_accumulator_q16 >> 16);
            sample->electrical_angle_q6 = electrical_phase_q16_to_q6(sample->electrical_phase_q16);
        } else {
            /* Raw 000/111/noise tidak boleh menghapus odometri yang sudah valid.
             * Putuskan kontinuitas sector/speed, tetapi pertahankan posisi terakhir. */
            state->hall_initialized = false;
            state->hall_period_samples = 0U;
            state->hall_last_sector_period_samples = 0U;
            state->hall_speed_q4 = 0;
            state->hall_last_direction = 0;
            sample->hall_encoding = 0U;
            sample->mechanical_speed_q4 = 0;
            sample->electrical_phase_q16 =
                (uint16_t)(state->hall_phase_accumulator_q16 >> 16);
            sample->electrical_angle_q6 = electrical_phase_q16_to_q6(sample->electrical_phase_q16);
        }
        sample->mechanical_angle_q4 = 0;
        state->encoder_initialized = false;
        state->encoder_speed_initialized = false;
        state->encoder_speed_window_count = 0U;
        sample->position_ticks = state->position_ticks;
    }

    sample->hall_a = (uint8_t)((sample->hall_encoding >> 2) & 1U);
    sample->hall_b = (uint8_t)((sample->hall_encoding >> 1) & 1U);
    sample->hall_c = (uint8_t)(sample->hall_encoding & 1U);
}
