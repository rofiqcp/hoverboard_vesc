#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensor.h"

static void test_encoder_16khz_speed_scale(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg.encoder_cpr = 2048U;
    cfg.encoder_calibrated = 1U;
    cfg.encoder_sequence_valid = 1U;
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg, &st, 15U);
    memset(&sm, 0, sizeof(sm));

    int32_t count = 0;
    MotorSensor_UpdateHardwareEncoder(&cfg, &st, count, 0U, 0U, 15U, &sm);
    st.encoder_electrical_zero_q16 = st.encoder_electrical_phase_q16;
    st.encoder_electrical_aligned = true;

    /* 10 counts per 64 samples at 16 kHz = 73.2421875 mechanical RPM for
     * CPR=2048. Run several windows so the 1/2 IIR converges. */
    for (unsigned window = 0; window < 7U; ++window) {
        for (unsigned i = 0; i < 64U; ++i) {
            if (((i + 1U) % 6U) == 0U && (i + 1U) <= 60U) ++count;
            MotorSensor_UpdateHardwareEncoder(&cfg, &st, count,
                                               (uint8_t)((count >> 1) & 1),
                                               (uint8_t)(count & 1), 15U, &sm);
        }
    }
    const int32_t mechanical_q4 = sm.mechanical_speed_q4;
    const int32_t erpm = (mechanical_q4 * 15) / 16;
    assert(mechanical_q4 > 1120 && mechanical_q4 < 1190); /* ~70..74.4 rpm */
    assert(erpm > 1050 && erpm < 1120);                   /* ~1.1k eRPM */
}

static void hall_feed(const MotorRuntimeConfig *cfg, MotorSensorState *st,
                      MotorSensorSample *sm, uint8_t raw) {
    MotorSensor_Update(cfg, st,
        (uint8_t)((raw >> 2) & 1U),
        (uint8_t)((raw >> 1) & 1U),
        (uint8_t)(raw & 1U), 15U, sm);
}

static void test_hall_16khz_speed_scale(void) {
    MotorRuntimeConfig cfg;
    MotorSensorState st;
    MotorSensorSample sm;
    MotorRuntimeConfig_SetDefaults(&cfg);
    cfg.sensor_type = MOTOR_SENSOR_HALL_UVW;
    cfg.hall_calibrated = 1U;
    cfg.hall_lut_valid = 1U;
    MotorSensor_Reset(&st);
    MotorSensor_PrepareRuntime(&cfg, &st, 15U);
    memset(&sm, 0, sizeof(sm));

    static const uint8_t seq[6] = {2U,3U,1U,5U,4U,6U};
    hall_feed(&cfg, &st, &sm, seq[0]);
    for (unsigned edge = 1; edge < 7U; ++edge) {
        for (unsigned i = 0; i < 159U; ++i) hall_feed(&cfg, &st, &sm, seq[(edge - 1U) % 6U]);
        hall_feed(&cfg, &st, &sm, seq[edge % 6U]);
    }

    /* One Hall sector every 160 samples at 16 kHz =>
     * electrical RPM = 60*16000/(6*160) = 1000 eRPM.
     * Mechanical RPM at 15 pole-pairs = 66.666... rpm. */
    const int32_t mechanical_q4 = sm.mechanical_speed_q4;
    const int32_t erpm = (mechanical_q4 * 15) / 16;
    assert(mechanical_q4 >= 1060 && mechanical_q4 <= 1075);
    assert(erpm >= 993 && erpm <= 1008);
}

int main(void) {
    test_encoder_16khz_speed_scale();
    test_hall_16khz_speed_scale();
    puts("V18_RPM_SCALING_TESTS_PASS");
    return 0;
}
