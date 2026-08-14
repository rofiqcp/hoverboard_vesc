#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=R/path; s=p.read_text()
    if s.count(old)!=1: raise SystemExit(f'{label}: expected 1 got {s.count(old)}')
    p.write_text(s.replace(old,new,1))

# Replace the full-electrical-revolution endpoint assumption with local 120-degree
# probes, matching upstream VESC's inversion/ratio measurement shape.
rep('Src/runtime_control.c',
'''/* V17 encoder detect mirrors the important VESC inversion/ratio shape: acquire
 * both commanded directions. Three forward + three reverse electrical turns fit
 * the same time budget as V16's six forward turns, but preserve sign evidence. */
#define SENSOR_CAL_ENCODER_FORWARD_SWEEPS 3U
#define SENSOR_CAL_ENCODER_REVERSE_SWEEPS 3U
#define SENSOR_CAL_VESC_ENCODER_CYCLES 6U
#define SENSOR_CAL_VESC_POLE_PAIRS_MAX 60U''',
'''/* Upstream VESC measures encoder inversion/ratio with repeated +/-120 degree
 * electrical moves. A steering axis can start against either hard stop, so this
 * port repeats +120 -> 0 -> -120 -> 0 and accepts only the probes that actually
 * moved. This state machine runs at 200 Hz; ISR work is unchanged. */
#define SENSOR_CAL_ENCODER_PROBE_Q4            1920
#define SENSOR_CAL_ENCODER_PROBE_SETTLE_MS      180U
#define SENSOR_CAL_ENCODER_PROBE_MIN_VALID        4U
#define SENSOR_CAL_ENCODER_PROBE_EARLY_COUNT      8U
#define SENSOR_CAL_ENCODER_PROBE_MAX_COUNT       16U
#define SENSOR_CAL_VESC_POLE_PAIRS_MAX           60U''',
'encoder detect constants')

rep('Src/runtime_control.c',
'''    bool encoder_direction_proved;
    bool encoder_ratio_fallback_used;
} SensorCalibrationRuntime;''',
'''    bool encoder_direction_proved;
    bool encoder_ratio_fallback_used;
    bool encoder_probe_initialized;
    uint8_t encoder_probe_stage;
    uint8_t encoder_probe_total;
    uint8_t encoder_probe_valid;
    int32_t encoder_probe_start_raw;
    uint32_t encoder_probe_hold_until;
    uint16_t encoder_probe_ratio_sum;
    uint8_t encoder_probe_ratio_min;
    uint8_t encoder_probe_ratio_max;
    uint16_t detected_encoder_offset_deg;
} SensorCalibrationRuntime;''',
'encoder probe state')

# Do not let the old full-sweep transition proof overwrite the direction measured
# by the local +/-120 raw TIM4 probes.
rep('Src/runtime_control.c',
'''    if (sensorCal.vesc_wire_detect) {
        sensor_cal_encoder_direction_proof(state, seq_negative_raw, seq_positive_raw);
    }''',
'''    if (sensorCal.vesc_wire_detect && sensorCal.encoder_probe_valid == 0U) {
        sensor_cal_encoder_direction_proof(state, seq_negative_raw, seq_positive_raw);
    }''',
'probe owns direction')

# Keep measured offset only as a detect result. Runtime electrical phase will be
# synchronized dynamically; persistent offset stays zero.
rep('Src/runtime_control.c',
'''                        candidate_config.sensor_inverted = sensorCal.detected_encoder_inverted;
                        candidate_config.encoder_ratio = sensorCal.detected_pole_pairs;
                        candidate_config.encoder_offset_deg =
                            sensor_cal_measured_encoder_offset_deg(&candidate_config,
                                sensorCal.detected_pole_pairs, sensorCal.detected_encoder_inverted);
                        candidate_config.encoder_calibrated = 1U;''',
'''                        candidate_config.sensor_inverted = sensorCal.detected_encoder_inverted;
                        candidate_config.encoder_ratio = sensorCal.detected_pole_pairs;
                        sensorCal.detected_encoder_offset_deg =
                            sensor_cal_measured_encoder_offset_deg(&candidate_config,
                                sensorCal.detected_pole_pairs, sensorCal.detected_encoder_inverted);
                        candidate_config.encoder_offset_deg = 0U;
                        candidate_config.encoder_calibrated = 1U;''',
'nonpersistent electrical offset')

rep('Src/runtime_control.c',
'''    snap->result.encoder_offset_deg = cfg->encoder_offset_deg;''',
'''    snap->result.encoder_offset_deg =
        (sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB &&
         sensorCal.detected_encoder_offset_deg < 360U)
            ? sensorCal.detected_encoder_offset_deg : cfg->encoder_offset_deg;''',
'detect reply measured offset')

# Insert the dedicated encoder-probe service before sensor_cal_service.
marker='''static void sensor_cal_service(uint32_t now, uint32_t dt_ms)
{'''
helper=r'''static void sensor_cal_encoder_probe_observe(MotorSensorState *state,
                                             const MotorSensorSample *sample)
{
    ++sensorCal.samples;
    const uint8_t ab = sample->encoder_ab & 0x03U;
    sensorCal.encoder_session_seen_mask |= (uint8_t)(1U << ab);
    bool known = false;
    for (uint8_t i = 0U; i < sensorCal.observed_encoder_count; ++i) {
        if (sensorCal.observed_encoder_sequence[i] == ab) { known = true; break; }
    }
    if (!known && sensorCal.observed_encoder_count < 4U)
        sensorCal.observed_encoder_sequence[sensorCal.observed_encoder_count++] = ab;
    if (state->position_ticks != sensorCal.encoder_start) sensorCal.motion_detected = true;
}

static void sensor_cal_encoder_probe_record(int8_t commanded_direction)
{
    const MotorRuntimeConfig *cfg = sensor_cal_config(sensorCal.motor);
    const int32_t raw_now = LeftEncoder_GetCount();
    const int32_t delta = raw_now - sensorCal.encoder_probe_start_raw;
    sensorCal.encoder_probe_start_raw = raw_now;
    const uint32_t mag = (uint32_t)(delta < 0 ? -delta : delta);
    if (mag < 3U || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN) return;

    const uint32_t den = 3U * mag;
    const uint32_t ratio = ((uint32_t)cfg->encoder_cpr + den / 2U) / den;
    if (ratio < 1U || ratio > SENSOR_CAL_VESC_POLE_PAIRS_MAX) return;

    ++sensorCal.encoder_probe_valid;
    sensorCal.encoder_probe_ratio_sum =
        (uint16_t)(sensorCal.encoder_probe_ratio_sum + ratio);
    if (sensorCal.encoder_probe_ratio_min == 0U || ratio < sensorCal.encoder_probe_ratio_min)
        sensorCal.encoder_probe_ratio_min = (uint8_t)ratio;
    if (ratio > sensorCal.encoder_probe_ratio_max)
        sensorCal.encoder_probe_ratio_max = (uint8_t)ratio;

    const bool same = (delta > 0 && commanded_direction > 0) ||
                      (delta < 0 && commanded_direction < 0);
    if (same) sensorCal.encoder_direction_normal_score += mag;
    else sensorCal.encoder_direction_inverted_score += mag;
}

static bool sensor_cal_service_vesc_encoder_probe(uint32_t now, uint32_t dt_ms,
                                                  MotorSensorState *state,
                                                  const MotorSensorSample *sample)
{
    if (sensorCal.sensor_type != MOTOR_SENSOR_ENCODER_AB ||
        !sensorCal.vesc_wire_detect || sensorCal.motor != ESC_MOTOR_LEFT)
        return false;

    sensor_cal_encoder_probe_observe(state, sample);
    if (!sensorCal.encoder_probe_initialized) {
        sensorCal.encoder_probe_initialized = true;
        sensorCal.encoder_probe_start_raw = LeftEncoder_GetCount();
        sensorCal.encoder_probe_stage = 0U;
        sensorCal.encoder_probe_total = 0U;
        sensorCal.encoder_probe_valid = 0U;
        sensorCal.encoder_probe_ratio_sum = 0U;
        sensorCal.encoder_probe_ratio_min = 0U;
        sensorCal.encoder_probe_ratio_max = 0U;
        sensorCal.encoder_direction_normal_score = 0U;
        sensorCal.encoder_direction_inverted_score = 0U;
        sensorCal.encoder_probe_hold_until = 0U;
        sensorCal.phase_q4 = 0;
    }

    static const int16_t targets[4] = {
        SENSOR_CAL_ENCODER_PROBE_Q4, 0,
        -SENSOR_CAL_ENCODER_PROBE_Q4, 0
    };
    static const int8_t directions[4] = {1, -1, -1, 1};
    const uint8_t stage = sensorCal.encoder_probe_stage & 3U;
    const int16_t target = targets[stage];

    if (sensorCal.encoder_probe_hold_until != 0U) {
        sensor_cal_set_current_override(sensorCal.motor, true, sensorCal.phase_q4,
                                        sensorCal.requested_drive_current_internal);
        refresh_master_enable();
        if ((int32_t)(now - sensorCal.encoder_probe_hold_until) < 0) return true;

        sensor_cal_encoder_probe_record(directions[stage]);
        if (directions[stage] > 0) {
            if (sensorCal.forward_cycles != UINT8_MAX) ++sensorCal.forward_cycles;
        } else {
            if (sensorCal.reverse_cycles != UINT8_MAX) ++sensorCal.reverse_cycles;
        }
        if (sensorCal.completed_cycles != UINT16_MAX) ++sensorCal.completed_cycles;
        if (sensorCal.encoder_probe_total != UINT8_MAX) ++sensorCal.encoder_probe_total;
        sensorCal.encoder_probe_stage = (uint8_t)((stage + 1U) & 3U);
        sensorCal.encoder_probe_hold_until = 0U;

        const bool enough =
            sensorCal.encoder_probe_total >= SENSOR_CAL_ENCODER_PROBE_EARLY_COUNT &&
            sensorCal.encoder_probe_valid >= SENSOR_CAL_ENCODER_PROBE_MIN_VALID;
        const bool exhausted = sensorCal.encoder_probe_total >= SENSOR_CAL_ENCODER_PROBE_MAX_COUNT;
        if (enough || exhausted) {
            MotorRuntimeConfig candidate = *sensor_cal_config(sensorCal.motor);
            const bool sequence_ok = sensor_cal_finalize_encoder_sequence(&candidate, state);
            uint8_t ratio = 0U;
            bool ratio_ok = false;
            if (sensorCal.encoder_probe_valid != 0U) {
                ratio = (uint8_t)((sensorCal.encoder_probe_ratio_sum +
                    sensorCal.encoder_probe_valid / 2U) / sensorCal.encoder_probe_valid);
                uint8_t tolerance = ratio / 4U;
                if (tolerance < 2U) tolerance = 2U;
                ratio_ok = ratio >= 1U && ratio <= SENSOR_CAL_VESC_POLE_PAIRS_MAX &&
                    sensorCal.encoder_probe_ratio_max >= sensorCal.encoder_probe_ratio_min &&
                    (uint8_t)(sensorCal.encoder_probe_ratio_max -
                              sensorCal.encoder_probe_ratio_min) <= tolerance;
            }
            const uint32_t normal = sensorCal.encoder_direction_normal_score;
            const uint32_t inverted = sensorCal.encoder_direction_inverted_score;
            const uint32_t total = normal + inverted;
            const uint32_t diff = normal > inverted ? normal - inverted : inverted - normal;
            const bool direction_ok = total >= 8U && diff * 4U >= total;
            const uint32_t valid_edges = state->encoder_valid_edges -
                sensorCal.encoder_session_valid_edges_start;
            const uint32_t invalid_edges = state->encoder_invalid_transitions -
                sensorCal.encoder_session_invalid_transitions_start;
            const bool quadrature_ok = valid_edges >= 8U &&
                invalid_edges <= (2U + valid_edges / 16U);

            if (sequence_ok && ratio_ok && direction_ok && quadrature_ok && enough) {
                sensorCal.detected_pole_pairs = ratio;
                sensorCal.detected_encoder_inverted = inverted > normal ? 1U : 0U;
                sensorCal.encoder_direction_proved = true;
                sensorCal.encoder_ratio_fallback_used = false;
                sensorCal.encoder_delta = state->position_ticks - sensorCal.encoder_start;
                sensor_cal_finish(ESC_SENSOR_CAL_SUCCESS, 0U);
            } else if (exhausted) {
                sensor_cal_finish(ESC_SENSOR_CAL_FAILED_SEQUENCE, 1U);
            }
        }
        return true;
    }

    if (dt_ms == 0U) dt_ms = 1U;
    if (dt_ms > 50U) dt_ms = 50U;
    int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
    int32_t phase = sensorCal.phase_q4;
    bool reached = false;
    if (phase < target) {
        phase += step;
        if (phase >= target) { phase = target; reached = true; }
    } else if (phase > target) {
        phase -= step;
        if (phase <= target) { phase = target; reached = true; }
    } else {
        reached = true;
    }
    sensorCal.phase_q4 = (int16_t)phase;
    sensor_cal_set_current_override(sensorCal.motor, true, sensorCal.phase_q4,
                                    sensorCal.requested_drive_current_internal);
    refresh_master_enable();
    if (reached)
        sensorCal.encoder_probe_hold_until = now + SENSOR_CAL_ENCODER_PROBE_SETTLE_MS;
    return true;
}

'''
p=R/'Src/runtime_control.c'; s=p.read_text()
if s.count(marker)!=1: raise SystemExit('sensor_cal_service marker')
p.write_text(s.replace(marker,helper+marker,1))

# Route only VESC-wire encoder detect into the probe state machine after the common
# phase-0 current alignment. Hall and native/manual paths remain unchanged.
rep('Src/runtime_control.c',
'''    sensorCal.drive_current_internal = sensorCal.requested_drive_current_internal;

    if (dt_ms == 0U) dt_ms = 1U;''',
'''    sensorCal.drive_current_internal = sensorCal.requested_drive_current_internal;

    if (sensor_cal_service_vesc_encoder_probe(now, dt_ms, state, sample)) return;

    if (dt_ms == 0U) dt_ms = 1U;''',
'route VESC encoder probes')

# Old endpoint candidate logic no longer references removed encoder sweep constants.
p=R/'Src/runtime_control.c'; s=p.read_text()
a=s.find('    if (sensorCal.vesc_wire_detect) {', s.find('static bool sensor_cal_auto_candidate_ready'))
b=s.find('\n    if (delta < 0) delta = -delta;', a)
if a<0 or b<0: raise SystemExit('old VESC candidate block')
s=s[:a]+'''    if (sensorCal.vesc_wire_detect) {
        /* VESC-wire encoder commissioning is finalized by the dedicated +/-120
         * degree probe service above. Reaching this generic endpoint is never a
         * valid ratio proof. */
        sensorCal.encoder_ratio_fallback_used = false;
        return false;
    }
'''+s[b:]
p.write_text(s)

(R/'tests/test_v21_left_encoder_probe_205320.py').write_text('''from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_vesc_encoder_detect_is_local_120_degree_probe():
 r=s("Src/runtime_control.c")
 assert "SENSOR_CAL_ENCODER_PROBE_Q4            1920" in r
 assert "targets[4]" in r and "-SENSOR_CAL_ENCODER_PROBE_Q4" in r
 assert "cfg->encoder_cpr + den / 2U" in r
 assert "SENSOR_CAL_ENCODER_FORWARD_SWEEPS" not in r
def test_probe_uses_raw_tim4_and_requires_quadrature_quality():
 r=s("Src/runtime_control.c")
 assert "const int32_t raw_now = LeftEncoder_GetCount();" in r
 assert "invalid_edges <= (2U + valid_edges / 16U)" in r
 assert "encoder_ratio_fallback_used = false" in r
def test_electrical_offset_is_detect_result_not_persistent_config():
 r=s("Src/runtime_control.c")
 assert "sensorCal.detected_encoder_offset_deg" in r
 assert "candidate_config.encoder_offset_deg = 0U;" in r
''')
print('left encoder probe patch staged')
