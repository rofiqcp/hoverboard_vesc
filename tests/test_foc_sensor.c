#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "foc_motor.h"
#include "motor_sensor.h"

static void test_encoder_backend(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg.encoder_cpr = 64;
    cfg.encoder_calibrated = 1;
    const uint8_t seq[4] = {0,2,3,1};
    assert(MotorRuntimeConfig_SetEncoderSequence(&cfg, seq));
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg, &st, 2);

    memset(&sm,0,sizeof(sm));
    MotorSensor_Update(&cfg,&st,0,0,1,2,&sm); /* W must be ignored */
    /* Calibrated AB sequence alone is not enough for closed-loop FOC: an
     * incremental encoder must also be electrically aligned after boot. Raw
     * observation still updates position/phase while feedback_valid stays 0. */
    assert(sm.feedback_valid == 0);
    st.encoder_electrical_zero_q16 = st.encoder_electrical_phase_q16;
    st.encoder_electrical_aligned = true;
    MotorSensor_Update(&cfg,&st,0,0,0,2,&sm);
    assert(sm.feedback_valid == 1);
    uint16_t p0 = sm.electrical_phase_q16;
    MotorSensor_Update(&cfg,&st,1,0,0,2,&sm); /* 00 -> 10 */
    uint16_t p1 = sm.electrical_phase_q16;
    assert(p1 != p0);
    MotorSensor_Update(&cfg,&st,1,1,1,2,&sm); /* 10 -> 11, W ignored */
    MotorSensor_Update(&cfg,&st,0,1,0,2,&sm); /* 11 -> 01 */
    MotorSensor_Update(&cfg,&st,0,0,1,2,&sm); /* 01 -> 00 */
    assert(st.position_ticks == 4);
    assert(sm.encoder_ab == 0);
}


static void encoder_feed_state(const MotorRuntimeConfig *cfg,
                               MotorSensorState *st,
                               MotorSensorSample *sm,
                               uint8_t ab,
                               uint8_t pole_pairs) {
    MotorSensor_Update(cfg, st, (uint8_t)((ab >> 1) & 1U),
                       (uint8_t)(ab & 1U), 1U, pole_pairs, sm);
}


static void hall_feed_code(const MotorRuntimeConfig *cfg,
                           MotorSensorState *st,
                           MotorSensorSample *sm,
                           uint8_t code,
                           uint8_t pole_pairs)
{
    MotorSensor_Update(cfg, st,
                       (uint8_t)((code >> 2) & 1U),
                       (uint8_t)((code >> 1) & 1U),
                       (uint8_t)(code & 1U),
                       pole_pairs, sm);
}

static void test_calibrated_sequence_owns_direction(void)
{
    /* Hall: use a rotated + reversed valid Gray sequence. Raw code 110 is defined
     * as sector 0 and 100 as sector 1. FOC/runtime must follow this calibration,
     * not the canonical factory order. */
    MotorRuntimeConfig hall_cfg;
    MotorSensorState hall_st;
    MotorSensorSample hall_sm;
    MotorRuntimeConfig_SetDefaults(&hall_cfg);
    hall_cfg.sensor_type = MOTOR_SENSOR_HALL_UVW;
    hall_cfg.hall_calibrated = 1U;
    const uint8_t hall_custom[6] = {6U, 4U, 5U, 1U, 3U, 2U};
    assert(MotorRuntimeConfig_SetHallSequence(&hall_cfg, hall_custom));
    MotorSensor_Reset(&hall_st);
    MotorSensor_PrepareRuntime(&hall_cfg, &hall_st, 15U);
    memset(&hall_sm, 0, sizeof(hall_sm));

    hall_feed_code(&hall_cfg, &hall_st, &hall_sm, hall_custom[0], 15U);
    for (unsigned i = 0; i < 80U; ++i) {
        hall_feed_code(&hall_cfg, &hall_st, &hall_sm, hall_custom[0], 15U);
    }
    hall_feed_code(&hall_cfg, &hall_st, &hall_sm, hall_custom[1], 15U);
    assert(hall_sm.feedback_valid == 1U);
    assert(hall_st.position_ticks == 1);
    assert(hall_sm.mechanical_speed_q4 > 0);

    for (unsigned i = 0; i < 80U; ++i) {
        hall_feed_code(&hall_cfg, &hall_st, &hall_sm, hall_custom[1], 15U);
    }
    hall_feed_code(&hall_cfg, &hall_st, &hall_sm, hall_custom[0], 15U);
    assert(hall_st.position_ticks == 0);
    assert(hall_sm.mechanical_speed_q4 < 0);

    /* Encoder: reverse the calibrated quadrature sequence. Advancing through that
     * stored sequence must still count positive. */
    MotorRuntimeConfig enc_cfg;
    MotorSensorState enc_st;
    MotorSensorSample enc_sm;
    MotorRuntimeConfig_SetDefaults(&enc_cfg);
    enc_cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    enc_cfg.encoder_cpr = 64U;
    enc_cfg.encoder_calibrated = 1U;
    const uint8_t enc_reverse[4] = {0U, 1U, 3U, 2U};
    assert(MotorRuntimeConfig_SetEncoderSequence(&enc_cfg, enc_reverse));
    MotorSensor_Reset(&enc_st);
    MotorSensor_PrepareRuntime(&enc_cfg, &enc_st, 2U);
    memset(&enc_sm, 0, sizeof(enc_sm));

    encoder_feed_state(&enc_cfg, &enc_st, &enc_sm, enc_reverse[0], 2U);
    enc_st.encoder_electrical_zero_q16 = enc_st.encoder_electrical_phase_q16;
    enc_st.encoder_electrical_aligned = true;
    encoder_feed_state(&enc_cfg, &enc_st, &enc_sm, enc_reverse[0], 2U);
    for (unsigned i = 1U; i <= 4U; ++i) {
        encoder_feed_state(&enc_cfg, &enc_st, &enc_sm, enc_reverse[i & 3U], 2U);
    }
    assert(enc_sm.feedback_valid == 1U);
    assert(enc_st.position_ticks == 4);
}

static void test_encoder_phase_exact_and_alignment(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg.encoder_cpr = 800U;
    cfg.encoder_calibrated = 1U;
    const uint8_t seq[4] = {0U, 2U, 3U, 1U};
    assert(MotorRuntimeConfig_SetEncoderSequence(&cfg, seq));
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg, &st, 15U);
    memset(&sm, 0, sizeof(sm));

    /* Initialize at AB=00. */
    encoder_feed_state(&cfg, &st, &sm, 0U, 15U);
    const uint16_t initial_phase = sm.electrical_phase_q16;

    /* Exactly one mechanical revolution must not accumulate phase error. */
    for (unsigned i = 0U; i < 800U; ++i) {
        encoder_feed_state(&cfg, &st, &sm, seq[(i + 1U) & 3U], 15U);
    }
    assert(st.position_ticks == 800);
    assert(sm.electrical_phase_q16 == initial_phase);

    /* A forward path followed by the exact reverse path must be reversible. */
    const int32_t pos_before = st.position_ticks;
    const uint16_t phase_before = st.encoder_electrical_phase_q16;
    const uint16_t rem_before = st.encoder_electrical_remainder_accum;
    uint8_t index = 0U;
    for (unsigned i = 0U; i < 37U; ++i) {
        index = (uint8_t)((index + 1U) & 3U);
        encoder_feed_state(&cfg, &st, &sm, seq[index], 15U);
    }
    for (unsigned i = 0U; i < 37U; ++i) {
        index = (uint8_t)((index + 3U) & 3U);
        encoder_feed_state(&cfg, &st, &sm, seq[index], 15U);
    }
    assert(st.position_ticks == pos_before);
    assert(st.encoder_electrical_phase_q16 == phase_before);
    assert(st.encoder_electrical_remainder_accum == rem_before);

    /* Electrical alignment captures the Q16 phase actually consumed by FOC. */
    for (unsigned i = 0U; i < 7U; ++i) {
        index = (uint8_t)((index + 1U) & 3U);
        encoder_feed_state(&cfg, &st, &sm, seq[index], 15U);
    }
    st.encoder_electrical_zero_q16 = st.encoder_electrical_phase_q16;
    st.encoder_electrical_aligned = true;
    encoder_feed_state(&cfg, &st, &sm, seq[index], 15U); /* no edge */
    assert(sm.electrical_phase_q16 == 0U);

    index = (uint8_t)((index + 1U) & 3U);
    encoder_feed_state(&cfg, &st, &sm, seq[index], 15U);
    const uint16_t aligned_phase = sm.electrical_phase_q16;
    assert(aligned_phase != 0U);

    /* Mechanical zero must not disturb the electrical reference/phase. */
    MotorSensor_Zero(&st);
    encoder_feed_state(&cfg, &st, &sm, seq[index], 15U);
    assert(sm.electrical_phase_q16 == aligned_phase);
    assert(sm.position_ticks == 0);
}


static void test_hardware_encoder_batched_update(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg.encoder_cpr = 800U;
    cfg.encoder_calibrated = 1U;
    const uint8_t seq[4] = {0U, 2U, 3U, 1U};
    assert(MotorRuntimeConfig_SetEncoderSequence(&cfg, seq));
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg, &st, 15U);
    memset(&sm, 0, sizeof(sm));

    MotorSensor_UpdateHardwareEncoder(&cfg, &st, 0, 0U, 0U, 15U, &sm);
    st.encoder_electrical_zero_q16 = st.encoder_electrical_phase_q16;
    st.encoder_electrical_aligned = true;

    static const int32_t jumps[] = {1, 2, 7, 16, 31, 4, 19, 1, 32, 3};
    int32_t count = 0;
    for (unsigned i = 0; i < sizeof(jumps)/sizeof(jumps[0]); ++i) {
        count += jumps[i];
        MotorSensor_UpdateHardwareEncoder(&cfg, &st, count,
                                          (uint8_t)((count >> 1) & 1),
                                          (uint8_t)(count & 1), 15U, &sm);
        assert(sm.position_ticks == count);
    }
    const uint16_t forward_phase = st.encoder_electrical_phase_q16;
    const uint16_t forward_rem = st.encoder_electrical_remainder_accum;
    assert(forward_phase != 0U || forward_rem != 0U);

    for (unsigned i = sizeof(jumps)/sizeof(jumps[0]); i > 0U; --i) {
        count -= jumps[i - 1U];
        MotorSensor_UpdateHardwareEncoder(&cfg, &st, count,
                                          (uint8_t)((count >> 1) & 1),
                                          (uint8_t)(count & 1), 15U, &sm);
        assert(sm.position_ticks == count);
    }
    assert(count == 0);
    assert(st.position_ticks == 0);
    assert(st.encoder_electrical_phase_q16 == 0U);
    assert(st.encoder_electrical_remainder_accum == 0U);

    /* A jump outside the bounded physical budget is rejected rather than
     * causing an unbounded ISR workload or phase discontinuity. */
    const uint32_t invalid_before = st.encoder_invalid_transitions;
    MotorSensor_UpdateHardwareEncoder(&cfg, &st, 100, 0U, 0U, 15U, &sm);
    assert(st.position_ticks == 0);
    assert(st.encoder_invalid_transitions == invalid_before + 1U);
}

static void test_hall_backend(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_HALL_UVW;
    cfg.hall_calibrated = 1;
    cfg.hall_lut_valid = 1;
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg,&st,2);

    /* Default calibrated canonical sequence begins 010 -> 011. */
    memset(&sm,0,sizeof(sm));
    MotorSensor_Update(&cfg,&st,0,1,0,2,&sm);
    assert(sm.feedback_valid == 1);
    assert(sm.hall_encoding == 2);
    uint16_t p0 = sm.electrical_phase_q16;
    MotorSensor_Update(&cfg,&st,0,1,1,2,&sm);
    assert(sm.feedback_valid == 1);
    assert(st.position_ticks > 0);
    assert(sm.mechanical_speed_q4 > 0);
    assert(sm.electrical_phase_q16 != p0);

    /* Invalid raw Hall must block closed-loop proof immediately. */
    MotorSensor_Update(&cfg,&st,0,0,0,2,&sm);
    assert(sm.feedback_valid == 0);
}

static void test_sensor_proof_gate(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_HALL_UVW;
    cfg.hall_calibrated = 0;
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg,&st,2);
    MotorSensor_Update(&cfg,&st,0,1,0,2,&sm);
    assert(sm.feedback_valid == 0);

    cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg.encoder_calibrated = 0;
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg,&st,2);
    MotorSensor_Update(&cfg,&st,0,0,0,2,&sm);
    assert(sm.feedback_valid == 0);

    /* Commissioning proof without a fresh electrical alignment must still be
     * rejected by the ISR safety gate. Observation remains available. */
    cfg.encoder_cpr = 64U;
    cfg.encoder_calibrated = 1U;
    const uint8_t enc_seq[4] = {0U, 2U, 3U, 1U};
    assert(MotorRuntimeConfig_SetEncoderSequence(&cfg, enc_seq));
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg,&st,2);
    MotorSensor_Update(&cfg,&st,0,0,0,2,&sm);
    assert(sm.feedback_valid == 0);
    st.encoder_electrical_zero_q16 = st.encoder_electrical_phase_q16;
    st.encoder_electrical_aligned = true;
    MotorSensor_Update(&cfg,&st,0,0,0,2,&sm);
    assert(sm.feedback_valid == 1);
}

static void test_outer_loops(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&m,&conf);

    mc_foc_set_control_mode(&m, CONTROL_MODE_SPEED);
    m.m_speed_rpm_q4 = 0;
    m.m_speed_command_rpm = 100 * conf.foc_motor_pole_pairs;
    m.m_speed_pid_set_rpm = m.m_speed_command_rpm;
    assert(mc_foc_run_pid_control_speed(&m,5) > 0);
    mc_foc_reset_outer_loops(&m);
    m.m_speed_command_rpm = -100 * conf.foc_motor_pole_pairs;
    m.m_speed_pid_set_rpm = m.m_speed_command_rpm;
    assert(mc_foc_run_pid_control_speed(&m,5) < 0);

    mc_foc_set_control_mode(&m, CONTROL_MODE_POS);
    m.m_position_ticks = 100;
    m.m_pos_pid_set = 110;
    assert(mc_foc_run_pid_control_pos(&m,5) > 0);
    mc_foc_reset_outer_loops(&m);
    m.m_pos_pid_set = 90;
    assert(mc_foc_run_pid_control_pos(&m,5) < 0);
}

static void test_phase_gate_and_svm(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_output_t out;
    mc_foc_sample_t sample = {0};
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&m,&conf);
    mc_foc_set_control_mode(&m, CONTROL_MODE_VOLTAGE);
    m.m_voltage_q_set = conf.l_max_voltage / 3;
    sample.output_enabled = true;
    sample.feedback_valid = false;

    /* Invalid feedback without override must produce zero duty. */
    mc_foc_run_current_control(&m,&sample,&out,2000);
    assert(out.duty_a == 0 && out.duty_b == 0 && out.duty_c == 0);

    /* Forced phase is allowed for OPEN/calibration and must cover all SVM sectors. */
    mc_foc_set_phase_override(&m,true,0);
    unsigned sector_mask = 0;
    int nonzero = 0;
    for (unsigned phase=0; phase<65536; phase+=256) {
        mc_foc_set_phase_override(&m,true,(uint16_t)phase);
        mc_foc_run_current_control(&m,&sample,&out,2000);
        assert(out.duty_a >= -1000 && out.duty_a <= 1000);
        assert(out.duty_b >= -1000 && out.duty_b <= 1000);
        assert(out.duty_c >= -1000 && out.duty_c <= 1000);
        assert(m.m_motor_state.svm_sector >= 1 && m.m_motor_state.svm_sector <= 6);
        sector_mask |= 1U << m.m_motor_state.svm_sector;
        if (out.duty_a || out.duty_b || out.duty_c) nonzero = 1;
    }
    assert(nonzero);
    assert((sector_mask & 0x7EU) == 0x7EU);
}


static void test_current_reconstruction_left_right(void) {
    mc_configuration left_conf, right_conf;
    motor_all_state_t left, right;
    mc_foc_output_t out_l, out_r;
    mc_foc_sample_t sl = {0}, sr = {0};
    mc_foc_conf_set_defaults(&left_conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_conf_set_defaults(&right_conf, FOC_CURRENT_SAMPLE_IB_IC);
    mc_foc_init(&left, &left_conf);
    mc_foc_init(&right, &right_conf);
    mc_foc_set_control_mode(&left, CONTROL_MODE_CURRENT);
    mc_foc_set_control_mode(&right, CONTROL_MODE_CURRENT);
    sl.output_enabled = sr.output_enabled = true;
    sl.feedback_valid = sr.feedback_valid = true;
    sl.phase_q16 = sr.phase_q16 = 0U;

    /* Same physical balanced current: Ia=+100, Ib=-50, Ic=-50 ADC counts. */
    sl.phase_current_1 = 100;
    sl.phase_current_2 = -50;
    sr.phase_current_1 = -50;
    sr.phase_current_2 = -50;
    mc_foc_run_current_control(&left, &sl, &out_l, 2000U);
    mc_foc_run_current_control(&right, &sr, &out_r, 2000U);
    assert(left.m_motor_state.i_alpha == right.m_motor_state.i_alpha);
    assert(left.m_motor_state.i_beta == right.m_motor_state.i_beta);
    assert(left.m_motor_state.id == right.m_motor_state.id);
    assert(left.m_motor_state.iq == right.m_motor_state.iq);
    assert(left.m_motor_state.i_beta == 0);
}

static void test_current_pi_sign(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_output_t out;
    mc_foc_sample_t sample = {0};
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&m,&conf);
    mc_foc_set_control_mode(&m, CONTROL_MODE_CURRENT);
    mc_foc_set_phase_override(&m,true,0);
    sample.output_enabled = true;
    sample.feedback_valid = false;
    m.m_iq_set = conf.l_current_max / 4;
    mc_foc_run_current_control(&m,&sample,&out,2000);
    assert(m.m_motor_state.iq_target > 0);
    assert(m.m_motor_state.vq > 0);
    mc_foc_reset_control(&m);
    m.m_iq_set = -conf.l_current_max / 4;
    mc_foc_run_current_control(&m,&sample,&out,2000);
    assert(m.m_motor_state.iq_target < 0);
    assert(m.m_motor_state.vq < 0);
}


static void test_disarmed_current_observation_and_duty(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_output_t out;
    mc_foc_sample_t sample = {0};
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&m, &conf);

    /* Telemetry observation must remain active with MOE/output disabled. */
    sample.output_enabled = false;
    sample.feedback_valid = false;
    sample.phase_q16 = 0U;
    sample.phase_current_1 = 320;
    sample.phase_current_2 = -80;
    for (int i = 0; i < 8; ++i) {
        mc_foc_run_current_control(&m, &sample, &out, 2000U);
    }
    assert(out.duty_a == 0 && out.duty_b == 0 && out.duty_c == 0);
    assert(out.id != 0 || out.iq != 0);
    assert(out.id_target == 0 && out.iq_target == 0);

    /* VESC duty mode is a distinct actuation mode and produces modulation only
     * after the normal sensor/output gate has been satisfied. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_DUTY);
    m.m_voltage_q_set = 0;
    m.m_duty_cycle_set_q15 = 8192; /* 25% modulation target */
    m.m_iq_set = conf.l_current_max; /* runtime duty mode requests available torque */
    sample.output_enabled = true;
    sample.feedback_valid = true;
    sample.phase_current_1 = 0;
    sample.phase_current_2 = 0;
    mc_foc_run_current_control(&m, &sample, &out, 2000U);
    assert(out.duty_a != 0 || out.duty_b != 0 || out.duty_c != 0);
    assert(m.m_motor_state.vq > 0);
}

static void test_detect_current_regulated_forced_phase(void) {
    mc_configuration conf;
    motor_all_state_t m;
    mc_foc_output_t out;
    mc_foc_sample_t sample = {0};
    mc_foc_conf_set_defaults(&conf, FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&m, &conf);
    assert(conf.foc_current_units_per_amp == 800U);

    /* V7 Detect Current=1.0 A must mean Id=800 internal under a forced
     * electrical phase. Sensor feedback proof is intentionally not required. */
    mc_foc_set_control_mode(&m, CONTROL_MODE_CURRENT);
    mc_foc_set_phase_override(&m, true, 0U);
    mc_foc_set_current_commissioning(&m, true);
    mc_foc_set_voltage_override(&m, false, 0, 0);
    m.m_id_set = 800;
    m.m_iq_set = 0;
    sample.output_enabled = true;
    sample.feedback_valid = false;
    sample.phase_q16 = 12345U; /* ignored because forced phase owns theta_e */

    mc_foc_run_current_control(&m, &sample, &out, 2000U);
    assert(m.m_motor_state.id_target == 800);
    assert(m.m_motor_state.iq_target == 0);
    assert(m.m_motor_state.vd > 0);
    assert(m.m_motor_state.vq == 0);
    assert(out.duty_a != 0 || out.duty_b != 0 || out.duty_c != 0);

    /* 50 ADC count = 1 A. Balanced Ia=+50, Ib=-25, Ic=-25 maps to
     * Id=800 and Iq=0 at forced electrical phase 0. */
    sample.phase_current_1 = 50;
    sample.phase_current_2 = -25;
    mc_foc_run_current_control(&m, &sample, &out, 2000U);
    assert(m.m_motor_state.id == 800);
    assert(m.m_motor_state.iq == 0);
    assert(m.m_motor_state.id_target == 800);
    assert(m.m_motor_state.iq_target == 0);
}

int main(void) {
    test_encoder_backend();
    test_encoder_phase_exact_and_alignment();
    test_hardware_encoder_batched_update();
    test_calibrated_sequence_owns_direction();
    test_hall_backend();
    test_sensor_proof_gate();
    test_outer_loops();
    test_phase_gate_and_svm();
    test_current_reconstruction_left_right();
    test_current_pi_sign();
    test_detect_current_regulated_forced_phase();
    test_disarmed_current_observation_and_duty();
    puts("ALL_FOC_SENSOR_TESTS_PASS");
    return 0;
}
