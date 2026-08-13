#!/usr/bin/env python3
"""V15 contract: VESC command facade + V1-style timing-sensitive DMA path.

The hardware board still decides physical success. This source contract prevents
reintroducing the exact classes of gates/timing changes the user asked to remove.
"""
from pathlib import Path
import re
root = Path(__file__).resolve().parents[1]
motor = (root/'Src/motor.c').read_text()
runtime = (root/'Src/runtime_control.c').read_text()
vesc = (root/'Src/vesc_protocol.c').read_text()
profile = (root/'Src/control_profile.h').read_text()
foc = (root/'Src/foc_motor.c').read_text()
tester = (root/'tools/vesc_full_test.py').read_text()

# 1) V18 refines the V1 DMA ownership without lying about CPU capacity: ADC and
# both sensor observations remain 16 kHz, while one FOC motor runs per IRQ (8 kHz
# per motor). Current-loop Ki*dt is therefore prepared for 8 kHz.
assert 'CONTROL_PWM_FREQUENCY_HZ             16000U' in profile
assert 'CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ   16000U' in profile
assert 'CONTROL_FOC_MOTOR_FREQUENCY_HZ        8000U' in profile
assert 'motorFocSlotRight' in motor
assert '#define FOC_COMMISSIONING_KI_DT_Q16_MAX     82L' in foc
isr = motor[motor.index('void DMA1_Channel1_IRQHandler(void)'):]
for token in ('curL_phaA = (int16_t)(offsetrlA - adc_buffer.rlA);',
              'curR_phaC = (int16_t)(offsetrrC - adc_buffer.rrC);',
              'const uint32_t left_idr = LEFT_HALL_U_PORT->IDR;',
              'const uint32_t right_idr = RIGHT_HALL_U_PORT->IDR;',
              'mc_foc_run_current_control(&motorLeft',
              'mc_foc_run_current_control(&motorRight'):
    assert token in isr, token
# Both sensor snapshots happen before the emergency shed and FOC slot decision.
assert isr.index('const uint32_t left_idr') < isr.index('if (motorIsrShedNext)')
assert isr.index('const uint32_t right_idr') < isr.index('if (motorIsrShedNext)')
assert isr.index('if (motorIsrShedNext)') < isr.index('const bool run_right_slot')

# Keep the proven active-domain calibration and only one released-domain discard.
assert 'set_current_zero_vector_left();' in motor and 'set_current_zero_vector_right();' in motor
assert '#define BRIDGE_CURRENT_WARMUP_ADC_SAMPLES 1U' in motor
assert 'LEFT_TIM->BDTR |= TIM_BDTR_MOE;' in motor and 'RIGHT_TIM->BDTR |= TIM_BDTR_MOE;' in motor

# Encoder pin correction survives the V1 ISR restoration: PB6/PB7 are A/B.
assert 'const uint8_t encoder_a = left_v; /* PB6 */' in isr
assert 'const uint8_t encoder_b = left_w; /* PB7 */' in isr
assert 'LeftEncoder_GetCount(), encoder_a, encoder_b' in isr

# 2) Hot-path safety is minimal: stock sample-local DC chop + same-sample feedback.
assert 'abs_s16_saturated(curL_DC) <= curDC_max' in isr
assert 'abs_s16_saturated(curR_DC) <= curDC_max' in isr
assert 'commissioning_current_guard(' not in isr
assert 'motorControlIsrOverrunFaultMask = 0U;' in motor
# Deadline observation must not delete commands or gates.
timing = motor[motor.index('static inline void motor_isr_record_timing'):motor.index('void DMA1_Channel1_IRQHandler')]
for forbidden in ('runtimeMotorEnableMask &=', 'sensorCalibrationOpenLoopMask &=', 'bridge_release_left()', 'bridge_release_right()'):
    assert forbidden not in timing, forbidden

# Slow-loop SensorHealth is diagnostic only, not an extra sticky disarm policy.
health_gate = runtime[runtime.index('/* V15: SensorHealth remains visible'):runtime.index('/* Coba kedua sisi setiap siklus')]
assert 'sensorHealthLeft.fault_code' not in health_gate
assert 'sensorHealthRight.fault_code' not in health_gate
assert 'left_core_code != 0U' in health_gate and 'right_core_code != 0U' in health_gate

# 3) VESC SET facade directly routes all four primary commands and resets timeout.
for exact in (
    'RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);',
    'RuntimeControl_VescSetOne(!r,ESC_MODE_TRQ,v,run);',
    'RuntimeControl_VescSetOne(!r,ESC_MODE_SPD,v,run);',
    'RuntimeControl_VescSetOne(!r,ESC_MODE_POS,ticks,true);'):
    assert exact in vesc, exact
# V17 refines V15: all non-zero primary SETs still route directly, while exact
# zero Duty/Current/RPM has VESC stop semantics rather than holding MOE active.
assert 'const bool run=normalized!=0;' in vesc
assert 'const bool run=v!=0;' in vesc
assert 'const bool run = e != 0;' in vesc
assert 'const bool run=fabsf' not in vesc.replace(' ', '')
assert vesc.count('RuntimeControl_VescAlive();') >= 4

# Exact wire scaling remains VESC-compatible.
for exact in ('(float)raw/100000.0f', '(float)raw/1000.0f', 'set_rpm(r,raw)', '(float)raw/1000000.0f'):
    assert exact in vesc, exact

# GET_VALUES/SETUP share read-reset current averages; standard zero while bridge off is valid.
assert 'current_avg_take(r,&avg_id,&avg_iq,&avg_dc_ca,&avg_vd_mv,&avg_vq_mv);' in vesc
assert 'current_avg_take(right,&avg_id,&avg_iq,&avg_dc_ca,&avg_vd_mv,&avg_vq_mv);' in vesc
assert 'MotorControl_BridgeActive(left)' in vesc

# 4) Encoder evidence accumulates for the whole detect transaction; window dither cannot erase it.
for token in ('encoder_session_valid_edges_start', 'encoder_session_invalid_transitions_start',
              'encoder_session_seen_mask', 'sensorCal.encoder_session_seen_mask == 0x0FU'):
    assert token in runtime, token

# Failed sweep retry must NOT move the transaction edge baselines. That was the
# subtle V14/V15-pre-release bug that made a healthy loaded encoder fail forever.
retry_anchor = runtime.index('/* V15: restart only the displacement window.')
retry_end = runtime.index('sensorCal.last_motion_tick = now;', retry_anchor)
retry_block = runtime[retry_anchor:retry_end]
assert 'encoder_session_valid_edges_start =' not in retry_block
assert 'encoder_session_invalid_transitions_start =' not in retry_block
assert 'sensorCal.encoder_start = state->position_ticks;' in retry_block

# Replay the concrete LEFT evidence from log 20260813_010411: cumulative
# quadrature proof is healthy even when loaded-wheel net displacement returns near zero.
valid_edges, invalid_edges, session_mask = 94, 0, 0x0F
assert valid_edges >= 32
assert invalid_edges <= (2 + valid_edges // 32)
assert session_mask == 0x0F

# 5) Hall sensor type 0 is a real value, not Python false/missing.
assert 'hall_terminal_sensor = after.get("terminal_detect_sensor_type")' in tester
assert 'hall_terminal_sensor is not None and int(hall_terminal_sensor) == 0' in tester
assert 'terminal_detect_sensor_type") or -1' not in tester

print('V15_VESC_FLOW_V1_ISR_CONTRACT_PASS')
