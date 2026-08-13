#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensor.h"

static uint16_t expected_zero(uint16_t raw, uint16_t offset, uint16_t desired) {
    return (uint16_t)(raw + offset - desired);
}

int main(void) {
    MotorSensorState s;
    memset(&s, 0, sizeof(s));

    /* Reader invariant: theta = raw - zero + configured_offset. */
    s.encoder_electrical_phase_q16 = 0x3456U;
    s.encoder_offset_phase_q16 = 0x1234U;
    uint16_t out = MotorSensor_SyncEncoderElectricalPhase(&s, 0U);
    assert(s.encoder_electrical_aligned);
    assert(s.encoder_electrical_zero_q16 == expected_zero(0x3456U, 0x1234U, 0U));
    assert(out == 0U);

    /* Arbitrary non-zero desired phase, including wrap-around. */
    s.encoder_electrical_phase_q16 = 0xF200U;
    s.encoder_offset_phase_q16 = 0x2800U;
    out = MotorSensor_SyncEncoderElectricalPhase(&s, 0x9A00U);
    assert(s.encoder_electrical_zero_q16 == expected_zero(0xF200U, 0x2800U, 0x9A00U));
    assert(out == 0x9A00U);

    /* Re-sync at another raw count must establish the same requested electrical phase. */
    s.encoder_electrical_phase_q16 = 0x0100U;
    s.encoder_offset_phase_q16 = 0xF000U;
    out = MotorSensor_SyncEncoderElectricalPhase(&s, 0x7FFFU);
    assert(out == 0x7FFFU);

    puts("V19_ENCODER_SYNC_INVARIANT_PASS");
    return 0;
}
