#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_sensor.h"
#include "foc_motor.h"

static void encoder_cfg(MotorRuntimeConfig *cfg, uint8_t inverted) {
    MotorRuntimeConfig_SetDefaults(cfg);
    cfg->sensor_type = MOTOR_SENSOR_ENCODER_AB;
    cfg->sensor_inverted = inverted;
    cfg->encoder_cpr = 4096U;
    cfg->encoder_ratio = 16U; /* exactly 256 counts/electrical revolution */
    cfg->encoder_calibrated = 1U;
    cfg->encoder_sequence_valid = 1U;
    cfg->encoder_sequence[0]=0U; cfg->encoder_sequence[1]=2U;
    cfg->encoder_sequence[2]=3U; cfg->encoder_sequence[3]=1U;
}

static MotorSensorSample run_quarter_turn(uint8_t inverted) {
    MotorRuntimeConfig cfg; MotorSensorState st; MotorSensorSample sm;
    encoder_cfg(&cfg,inverted); memset(&st,0,sizeof(st)); memset(&sm,0,sizeof(sm));
    MotorSensor_PrepareRuntime(&cfg,&st,15U);
    MotorSensor_UpdateHardwareEncoder(&cfg,&st,0,0,0,15U,&sm);
    assert(MotorSensor_SyncEncoderElectricalPhase(&st,0U)==0U);
    for (int32_t i=1;i<=64;i++) {
        int32_t raw=inverted ? -i : i;
        MotorSensor_UpdateHardwareEncoder(&cfg,&st,raw,0,0,15U,&sm);
    }
    assert(sm.position_ticks==64);
    assert(sm.feedback_valid==1U);
    assert(sm.electrical_phase_q16>=16380U && sm.electrical_phase_q16<=16388U);
    return sm;
}

int main(void) {
    MotorSensorSample a=run_quarter_turn(0U);
    MotorSensorSample b=run_quarter_turn(1U);
    assert(a.electrical_phase_q16==b.electrical_phase_q16);

    mc_configuration conf; motor_all_state_t motor; mc_foc_output_t out; mc_foc_sample_t fs;
    mc_foc_conf_set_defaults(&conf,FOC_CURRENT_SAMPLE_IA_IB);
    mc_foc_init(&motor,&conf); memset(&fs,0,sizeof(fs)); memset(&out,0,sizeof(out));
    fs.output_enabled=true; fs.feedback_valid=true; fs.phase_q16=a.electrical_phase_q16;
    fs.speed_rpm_q4=0; fs.position_ticks=a.position_ticks;
    fs.phase_current_1=0; fs.phase_current_2=0;
    mc_foc_set_control_mode(&motor,CONTROL_MODE_CURRENT);
    motor.m_iq_set=800; motor.m_id_set=0;
    mc_foc_run_current_control(&motor,&fs,&out,2000U);
    assert(out.electrical_angle_deg>=89 && out.electrical_angle_deg<=91);
    assert(out.duty_a!=out.duty_b || out.duty_b!=out.duty_c);
    puts("V21_LEFT_ENCODER_PHASE_CHAIN_PASS");
    return 0;
}
