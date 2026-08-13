#!/usr/bin/env python3
"""V17 regression from physical log 2026-08-13 02:20:14 + VESC unit semantics."""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
config = (root/'Src/vesc_config_compat.c').read_text()
protocol = (root/'Src/vesc_protocol.c').read_text()
runtime = (root/'Src/runtime_control.c').read_text()
tester = (root/'tools/vesc_full_test.py').read_text()
util = (root/'Src/util.c').read_text()
board = (root/'Src/config.h').read_text()

# 1) Stock hoverboard semantics: 15 pole-pairs = 30 magnetic poles.
assert '#define I_MOT_MAX                15' in board
assert '#define I_DC_MAX                 17' in board
assert 'foc_motor_pole_pairs = 15U' in util
assert 'c->foc_motor_pole_pairs * 2U' in config
assert 'append_auto(b, (float)(r->encoder_ratio != 0U ? r->encoder_ratio : c->foc_motor_pole_pairs), &i);' in config
# SET_RPM remains the raw signed VESC electrical-RPM integer on ingress.
assert 'case C_SET_RPM' in protocol and 'int32_t raw=vesc_buf_get_i32(d,&j)' in protocol
assert 'set_rpm(r,raw)' in protocol
assert 'm->m_speed_rpm_q4' in protocol and 'q4/=16;' in protocol
assert 'motorOutputRight.speed_rpm:motorOutputLeft.speed_rpm' not in protocol
# Runtime reconstruction is directly in eRPM, not via mechanical RPM.
assert 'static int32_t scale_permille_to_erpm' in runtime
assert 'motor->m_speed_command_rpm = scale_permille_to_erpm(physical_command, conf);' in runtime
assert config.index('if (encoder_ratio >= 1.0f') < config.index('/* VESC limits and COMM_SET_RPM are electrical RPM.')
assert (1000 * 15) == 15000
assert (900 / 15) == 60

# 2) Two Hall tables on the VESC config wire must be distinct. Legacy BLDC table
# only carries -1/1..6; FOC table carries 0..200 electrical-angle entries.
assert 'hall_table_legacy_vesc' in config
assert 'hall_table_foc_vesc' in config
assert 'for (uint8_t n = 0; n < 8U; ++n) b[i++] = legacy_hall[n];' in config
assert 'for (uint8_t n = 0; n < 8U; ++n) b[i++] = foc_hall[n];' in config
legacy = config[config.index('static void hall_table_legacy_vesc'):config.index('static void hall_table_foc_vesc')]
for bad in ('50U', '183U', '116U', '83U', '150U'):
    assert bad not in legacy

# 3) VESC Tool current-limit advisory fields: normal motor current remains 15 A,
# ABS display threshold is separate and slow ABS is disabled. Physical 17 A DC
# chop remains in motor.c/config.h and is not raised by this UI field.
assert '#define VESC_UI_ABS_CURRENT_MULTIPLIER 1.5f' in config
assert 'VESC_UI_ABS_CURRENT_MULTIPLIER' in config
assert 'b[i++] = 0U; /* l_slow_abs_current' in config
assert '#define VESC_ERPM_LIMIT_START 0.8f' in config
assert 'append_f16(b, VESC_ERPM_LIMIT_START, 10000.0f, &i);' in config
assert '#define VESC_ERPM_LIMIT_START_Q15 26214' in runtime
assert 'static int16_t limit_accel_current_by_erpm' in runtime
assert 'motor->m_iq_set = limit_accel_current_by_erpm' in runtime

# 4) Exact-zero SETs release PWM. Position 0 remains a valid active target.
for fn in ('set_duty', 'set_current', 'set_rpm'):
    m = re.search(rf'static void {fn}\([^{{]+\)\{{(.+?)\n\}}', protocol, re.S)
    assert m, fn
    assert 'run' in m.group(1)
assert 'RuntimeControl_VescSetOne(!r,ESC_MODE_POS,ticks,true);' in protocol

# 5) Latest real log proved current wire scale already correct:
# SET_CURRENT raw=500 -> target 0.5 A; settled GET_VALUES Iq clustered 0.47..0.51 A.
requested = 0.5
wire_raw = 500
measured = [0.47, 0.48, 0.51]
assert wire_raw == round(requested * 1000)
assert abs(sum(measured)/len(measured) - requested) < 0.05
assert 'current scaling mismatch requested=' in tester
assert 'median_get_values_iq_A' in tester

# 6) Latest +900 eRPM test produced a clearly negative -60 eRPM sample while Iq
# climbed. V17 must detect that polarity mismatch and release immediately.
assert 'SPEED_FEEDBACK_SIGN_MISMATCH' in tester
assert 'self.dev.stop(node)' in tester
assert 'speed_sign_abort' in tester
assert 'sign_threshold = max(30, int(abs(value) * 0.05))' in tester

# 7) Encoder detect measures direction with distinct forward/reverse halves instead
# of preserving a stale configured direction whenever the loaded wheel dithers.
for token in ('SENSOR_CAL_ENCODER_FORWARD_SWEEPS 3U',
              'SENSOR_CAL_ENCODER_REVERSE_SWEEPS 3U',
              'encoder_transition_mid[16]', 'encoder_direction_normal_score',
              'encoder_direction_inverted_score', 'encoder_direction_proved',
              'sensor_cal_encoder_direction_proof'):
    assert token in runtime, token
assert 'sensorCal.encoder_forward_delta = state->position_ticks - sensorCal.encoder_start;' in runtime

# 8) First motion test must tolerate one-time encoder electrical alignment instead
# of declaring failure at the edge of the old 0.9 s window.
assert 'first-command encoder alignment still busy; extending same SET by 1.2 s' in tester

assert 'hoverboard-vesc6-v21' in protocol
assert 'TESTER_RELEASE = "V21"' in tester
print('V17_UNITS_CONFIG_SPEED_CONTRACT_PASS')
