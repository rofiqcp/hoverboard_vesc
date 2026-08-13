#!/usr/bin/env python3
"""V18 regression: true sensor timing + VESC current/brake/handbrake semantics."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
profile = (root/'Src/control_profile.h').read_text()
sensor = (root/'Src/motor_sensor.c').read_text()
motor = (root/'Src/motor.c').read_text()
foc_h = (root/'Src/foc_motor.h').read_text()
foc_c = (root/'Src/foc_motor.c').read_text()
esc_h = (root/'Src/esc_protocol.h').read_text()
runtime = (root/'Src/runtime_control.c').read_text()
proto = (root/'Src/vesc_protocol.c').read_text()
tester = (root/'tools/vesc_full_test.py').read_text()

# 1) RPM/speed estimators use the ACTUAL sensor observation cadence, not the
# per-motor FOC cadence. Hardware V16/V17 had reactive shed and could otherwise
# make Hall/encoder speed off by roughly 2x.
assert 'CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ   16000U' in profile
assert 'CONTROL_FOC_MOTOR_FREQUENCY_HZ        8000U' in profile
assert '#define MOTOR_CONTROL_FREQUENCY_HZ         CONTROL_SENSOR_SAMPLE_FREQUENCY_HZ' in sensor
isr = motor[motor.index('void DMA1_Channel1_IRQHandler(void)'):]
assert isr.index('MotorSensor_UpdateHardwareEncoder') < isr.index('if (motorIsrShedNext)')
assert isr.index('MotorSensor_Update(&motorConfigRight') < isr.index('if (motorIsrShedNext)')
assert 'const bool run_right_slot = motorFocSlotRight;' in isr
assert 'motorFocSlotRight = !motorFocSlotRight;' in isr
assert '#define FOC_COMMISSIONING_KI_DT_Q16_MAX     82L' in foc_c

# 2) Old unused guard must be gone, including the compiler warning the hardware
# build reported.
assert 'static bool commissioning_current_guard' not in motor

# 3) VESC has distinct FOC modes for current, current brake and handbrake.
for token in ('CONTROL_MODE_CURRENT_BRAKE', 'CONTROL_MODE_HANDBRAKE'):
    assert token in foc_h and token in foc_c
for token in ('ESC_MODE_BRAKE = 6', 'ESC_MODE_HANDBRAKE = 7'):
    assert token in esc_h
assert 'case ESC_MODE_BRAKE: return (uint8_t)CONTROL_MODE_CURRENT_BRAKE;' in runtime
assert 'case ESC_MODE_HANDBRAKE: return (uint8_t)CONTROL_MODE_HANDBRAKE;' in runtime

# 4) UART brake and handbrake must not collapse into set_current().
assert 'RuntimeControl_VescSetOne(!r,ESC_MODE_BRAKE,v,run);' in proto
assert 'RuntimeControl_VescSetOne(!r,ESC_MODE_HANDBRAKE,v,run);' in proto
assert 'case C_SET_CURRENT_BRAKE:if(l>=4)' in proto
assert 'case C_SET_HANDBRAKE:if(l>=4)' in proto
assert 'set_handbrake(r,(float)raw/1000.0f)' in proto

# 5) Brake sign is chosen in the fast loop every current update and handbrake
# forces phase zero. Current mode remains signed torque control.
assert 'motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE' in foc_c
assert 'sample->speed_rpm_q4 < 0 ? mag : (int16_t)-mag' in foc_c
assert 'const bool handbrake_mode = motor->m_control_mode == CONTROL_MODE_HANDBRAKE;' in foc_c
assert '(handbrake_mode ? 0U : sample->phase_q16)' in foc_c

# 6) Handbrake is static/open-loop phase and must not be blocked waiting for an
# encoder electrical-zero alignment that it does not use.
assert 'mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE' in runtime

# 7) Tester proves distinct core modes and moving brake-current polarity.
assert 'expected_core_mode = 4 if kind == "brake" else 5' in tester
assert 'brake Iq did not oppose speed' in tester
assert 'TESTER_RELEASE = "V20"' in tester
assert 'hoverboard-vesc6-v20' in proto

print('V18_RPM_CURRENT_BRAKE_CONTRACT_PASS')
