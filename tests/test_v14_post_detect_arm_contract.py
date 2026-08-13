#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
runtime = (root / 'Src/runtime_control.c').read_text()
vesc = (root / 'Src/vesc_protocol.c').read_text()

checks = {
    # Every VESC non-zero SET is routed into a side-local ARM request.
    # V21 follows VESC Tool: SET_DUTY(0) is Full Brake and therefore remains armed.
    'duty_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);' in vesc and 'const bool run=true;' in vesc,
    'current_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_TRQ,v,run);' in vesc and 'const bool run=v!=0;' in vesc,
    'rpm_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_SPD,v,run);' in vesc and 'const bool run = e != 0;' in vesc,
    'pos_side_arm': 'RuntimeControl_VescSetOne(!r,ESC_MODE_POS,ticks,true);' in vesc,
    # ARM is evaluated independently for each motor, not behind a both-motors-ready gate.
    'arm_left_then_right': 'try_arm_motor(true, now);\n    try_arm_motor(false, now);' in runtime,
    'side_local_bit': 'const uint8_t bit = left ? 0x01U : 0x02U;' in runtime,
    'side_local_feedback': 'prearm_feedback_not_ready(mode, requested_target, cfg, sample, encoder_aligned)' in runtime and '(mode == ESC_MODE_DUTY && target == 0)' in runtime,
    # Successful Detect commits the calibrated proof into active RAM and resets health.
    'encoder_cal_commit': 'candidate_config.encoder_calibrated = 1U;' in runtime,
    'hall_cal_commit': 'candidate_config.hall_calibrated = 1U;' in runtime,
    'health_reset_left': 'sensor_health_reset_one(&sensorHealthLeft, &motorSensorStateLeft);' in runtime,
    'health_reset_right': 'sensor_health_reset_one(&sensorHealthRight, &motorSensorStateRight);' in runtime,
    # Encoder first ARM uses regulated D-axis current, then returns to pending ARM request.
    'encoder_alignment_start': 'encoder_alignment_start(motor);\n        return;' in runtime,
    'encoder_alignment_current': 'sensor_cal_set_current_override(encoderAlign.motor, true, forced_angle_q4,' in runtime,
    'encoder_alignment_proof': 'MotorSensor_SyncEncoderElectricalPhase(state, 0U);' in runtime,
    # No old global requirement that BOTH sensors must be calibrated before either side can arm.
    'no_both_calibrated_gate': 'hall_calibrated && encoder_calibrated' not in runtime,
}

failed = [name for name, ok in checks.items() if not ok]
if failed:
    raise SystemExit('V14_POST_DETECT_ARM_CONTRACT_FAIL: ' + ', '.join(failed))
print('V14_POST_DETECT_ARM_CONTRACT_PASS')
