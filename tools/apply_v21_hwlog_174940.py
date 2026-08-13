#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def rw(path):
    p = ROOT / path
    return p, p.read_text()

def replace_once(path, old, new, label):
    p, s = rw(path)
    n = s.count(old)
    if n != 1:
        raise SystemExit(f"{label}: expected 1 match, got {n}")
    p.write_text(s.replace(old, new, 1))

# ---------------------------------------------------------------------------
# 1) LEFT encoder detect: use the stronger of forward or reverse sweep evidence.
#    The 17:49 hardware run started close to a steering stop: forward endpoint
#    moved only +2 counts while quadrature itself was clean. A valid detector must
#    not assume the forward half is always the unblocked half.
# ---------------------------------------------------------------------------
replace_once(
    'Src/runtime_control.c',
'''        int64_t raw_delta = (int64_t)sensorCal.encoder_forward_delta;
        if (sensorCal.original_sensor_inverted) raw_delta = -raw_delta;
        const uint64_t mag = (uint64_t)(raw_delta < 0 ? -raw_delta : raw_delta);
        const uint32_t cycles = SENSOR_CAL_ENCODER_FORWARD_SWEEPS;
        const uint64_t numerator = (uint64_t)cycles * (uint64_t)candidate.encoder_cpr;

        if (sequence_ready && mag >= 4U) {
            const uint32_t pp = (uint32_t)((numerator + (mag / 2U)) / mag);
            if (pp >= 1U && pp <= SENSOR_CAL_VESC_POLE_PAIRS_MAX) {
                const uint32_t expected = (uint32_t)((numerator + (pp / 2U)) / pp);
                const uint32_t measured = (uint32_t)mag;
                const uint32_t error = expected > measured ? expected - measured : measured - expected;
                uint32_t tolerance = expected / 4U;
                if (tolerance < 8U) tolerance = 8U;
                if (error <= tolerance) {
                    sensorCal.detected_pole_pairs = (uint8_t)pp;
                    if (!sensorCal.encoder_direction_proved)
                        sensorCal.detected_encoder_inverted = raw_delta < 0 ? 1U : 0U;
                    sensorCal.encoder_ratio_fallback_used = false;
                    return true;
                }
            }
        }
''',
'''        /* V21 hardware log 17:49: a steering motor can begin against one hard
         * stop. Use whichever commanded half-sweep produced the larger NET encoder
         * displacement. Reverse evidence is sign-normalized so positive always
         * means raw encoder follows positive commanded electrical rotation. */
        int64_t raw_forward = (int64_t)sensorCal.encoder_forward_delta;
        const int64_t total_delta = (int64_t)state->position_ticks - (int64_t)sensorCal.encoder_start;
        int64_t raw_reverse = total_delta - (int64_t)sensorCal.encoder_forward_delta;
        if (sensorCal.original_sensor_inverted) {
            raw_forward = -raw_forward;
            raw_reverse = -raw_reverse;
        }
        const uint64_t mag_forward = (uint64_t)(raw_forward < 0 ? -raw_forward : raw_forward);
        const uint64_t mag_reverse = (uint64_t)(raw_reverse < 0 ? -raw_reverse : raw_reverse);
        const int64_t normalized_delta = (mag_reverse > mag_forward) ? -raw_reverse : raw_forward;
        const uint64_t mag = (uint64_t)(normalized_delta < 0 ? -normalized_delta : normalized_delta);
        const uint32_t cycles = SENSOR_CAL_ENCODER_FORWARD_SWEEPS;
        const uint64_t numerator = (uint64_t)cycles * (uint64_t)candidate.encoder_cpr;

        if (sequence_ready && mag >= 4U) {
            const uint32_t pp = (uint32_t)((numerator + (mag / 2U)) / mag);
            if (pp >= 1U && pp <= SENSOR_CAL_VESC_POLE_PAIRS_MAX) {
                const uint32_t expected = (uint32_t)((numerator + (pp / 2U)) / pp);
                const uint32_t measured = (uint32_t)mag;
                const uint32_t error = expected > measured ? expected - measured : measured - expected;
                uint32_t tolerance = expected / 4U;
                if (tolerance < 8U) tolerance = 8U;
                if (error <= tolerance) {
                    sensorCal.detected_pole_pairs = (uint8_t)pp;
                    if (!sensorCal.encoder_direction_proved)
                        sensorCal.detected_encoder_inverted = normalized_delta < 0 ? 1U : 0U;
                    sensorCal.encoder_ratio_fallback_used = false;
                    return true;
                }
            }
        }
''',
    'encoder bidirectional ratio proof')

# Retry bug proven by the trace: after first 3F+3R failure, sweep_direction was
# left at -1, therefore all following windows were reverse-only until timeout.
replace_once(
    'Src/runtime_control.c',
'''                /* V15: restart only the displacement window. Keep cumulative A/B
                 * transition evidence and the session edge baseline; otherwise a
                 * clean loaded encoder can dither through many valid edges yet
                 * never satisfy a per-window proof threshold. */
                sensorCal.completed_cycles = 0U;
                sensorCal.forward_cycles = 0U;
                sensorCal.reverse_cycles = 0U;
                sensorCal.phase_q4 = 0;
                sensorCal.encoder_start = state->position_ticks;
                /* Keep encoder_session_* baselines from transaction start. Only
                 * the displacement origin moves to the next sweep window. */
                sensorCal.last_motion_position = (int32_t)sensor_cal_motion_counter(
                    state, MOTOR_SENSOR_ENCODER_AB);
                sensorCal.last_motion_tick = now;
''',
'''                /* V21: every retry is a complete +3/-3 electrical sweep again.
                 * The 17:49 trace proved V20/V21-pre left sweep_direction=-1 after
                 * the first failed window, creating reverse-only retries. Reset the
                 * per-window transition counters/direction snapshot while keeping
                 * transaction-wide valid-edge and seen-state evidence intact. */
                const bool motion_seen = sensorCal.motion_detected;
                sensor_cal_reset_capture_tables(state, sample, false);
                sensorCal.motion_detected = motion_seen;
                sensorCal.sweep_direction = 1;
                sensorCal.forward_cycles = 0U;
                sensorCal.reverse_cycles = 0U;
                sensorCal.phase_q4 = 0;
                /* encoder_session_* baselines and encoder_session_seen_mask are
                 * intentionally NOT reset by the per-window helper. */
                sensorCal.last_motion_position = (int32_t)sensor_cal_motion_counter(
                    state, MOTOR_SENSOR_ENCODER_AB);
                sensorCal.last_motion_tick = now;
''',
    'encoder retry direction reset')

# ---------------------------------------------------------------------------
# 2) Integrated board commissioning current. 0.50 A was demonstrably too weak:
#    ~255 valid / 0 invalid A/B edges but only a few net counts. Use the firmware's
#    conservative 1 A commissioning default. Individual detect still honors the
#    user-selected current and remains capped at 2 A.
# ---------------------------------------------------------------------------
p, s = rw('Src/vesc_protocol.c')
if '#define VESC_AUTO_DETECT_CURRENT_A' not in s:
    s = s.replace('#define UART_OVERRIDE_MS 300U\n',
                  '#define UART_OVERRIDE_MS 300U\n#define VESC_AUTO_DETECT_CURRENT_A 1.0f\n', 1)
count = s.count('detect_current_internal(false,0.5f)')
if count != 2:
    raise SystemExit(f'expected two LEFT integrated 0.5A sites, got {count}')
s = s.replace('detect_current_internal(false,0.5f)',
              'detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)')
count = s.count('detect_current_internal(true,0.5f)')
if count != 1:
    raise SystemExit(f'expected one RIGHT integrated 0.5A site, got {count}')
s = s.replace('detect_current_internal(true,0.5f)',
              'detect_current_internal(true,VESC_AUTO_DETECT_CURRENT_A)')
# Correct release identity; this also makes hardware logs unambiguous.
s = s.replace('hoverboard-vesc6-v20', 'hoverboard-vesc6-v21')
p.write_text(s)

# ---------------------------------------------------------------------------
# 3) Tester: COMM_ROTOR_POSITION is a pushed stream in upstream VESC. The old
#    matrix sent it as a request while display mode was NONE, causing the exact
#    1.5 s timeout in the uploaded log. Test SET_DETECT -> stream instead.
# ---------------------------------------------------------------------------
replace_once(
    'tools/vesc_full_test.py',
'''            rotor=self.dev.rotor_position(node)
            mc=self.dev.transact(bytes((COMM_GET_MCCONF,)),node=node,timeout=3.0)
''',
'''            rotor_samples=self.dev.collect_rotor_positions(node,4,seconds=0.12)
            assert rotor_samples, f"{node} PID-position rotor stream produced no COMM_ROTOR_POSITION frames"
            rotor=rotor_samples[-1]
            mc=self.dev.transact(bytes((COMM_GET_MCCONF,)),node=node,timeout=3.0)
''',
    'tester rotor stream matrix')

p, s = rw('tools/vesc_full_test.py')
s = s.replace('One-shot VESC-compatible V20 bench test', 'One-shot VESC-compatible V21 bench test')
s = s.replace('TESTER_RELEASE = "V20"', 'TESTER_RELEASE = "V21"')
s = s.replace('legacy separate LEFT encoder and RIGHT Hall detect instead of V20 integrated board commissioning',
              'legacy separate LEFT encoder and RIGHT Hall detect instead of V21 integrated board commissioning')
s = s.replace('--detect-current", type=float, default=0.5, help="sensor detect current [A] for --individual-detect; V20 integrated board commissioning intentionally uses fixed 0.50 A"',
              '--detect-current", type=float, default=1.0, help="sensor detect current [A] for --individual-detect; V21 integrated board commissioning uses conservative fixed 1.00 A"')
p.write_text(s)

# Current-release contract tests that intentionally identify the active firmware
# should follow the V21 identity. Historical behavior assertions remain intact.
for p in (ROOT / 'tests').glob('test_*.py'):
    s = p.read_text()
    s = s.replace('hoverboard-vesc6-v20', 'hoverboard-vesc6-v21')
    s = s.replace('TESTER_RELEASE = "V20"', 'TESTER_RELEASE = "V21"')
    p.write_text(s)

# Add a regression contract directly from the uploaded 17:49 trace.
(ROOT / 'tests/test_v21_hwlog_174940.py').write_text(r'''from pathlib import Path
root=Path(__file__).resolve().parents[1]
runtime=(root/'Src/runtime_control.c').read_text()
proto=(root/'Src/vesc_protocol.c').read_text()
tester=(root/'tools/vesc_full_test.py').read_text()

def test_encoder_retry_restarts_forward_and_rebases_direction_window():
    assert 'sensor_cal_reset_capture_tables(state, sample, false);' in runtime
    assert 'sensorCal.sweep_direction = 1;' in runtime
    retry=runtime[runtime.index('V21: every retry is a complete'):]
    assert retry.index('sensorCal.sweep_direction = 1;') < retry.index('sensorCal.last_motion_tick = now;')

def test_encoder_ratio_can_use_unblocked_reverse_half():
    assert 'raw_reverse = total_delta - (int64_t)sensorCal.encoder_forward_delta' in runtime
    assert '(mag_reverse > mag_forward) ? -raw_reverse : raw_forward' in runtime
    assert 'configured_pp' not in runtime

def test_integrated_detect_uses_conservative_one_amp_default():
    assert '#define VESC_AUTO_DETECT_CURRENT_A 1.0f' in proto
    assert proto.count('VESC_AUTO_DETECT_CURRENT_A') >= 4
    assert 'detect_current_internal(false,0.5f)' not in proto
    assert 'detect_current_internal(true,0.5f)' not in proto

def test_rotor_protocol_matrix_uses_upstream_stream_semantics():
    assert 'rotor_samples=self.dev.collect_rotor_positions(node,4,seconds=0.12)' in tester
    matrix=tester[tester.index('def protocol_get_matrix'):tester.index('def zero_set_routing_matrix')]
    assert 'self.dev.rotor_position(node)' not in matrix

def test_release_identity_is_v21():
    assert 'TESTER_RELEASE = "V21"' in tester
    assert 'hoverboard-vesc6-v21' in proto
''')

# Changelog: preserve previous content, append this hardware-driven correction.
p, s = rw('CHANGELOG_V21.md')
append = '''\n## Hardware correction — 2026-08-13 17:49 run\n- Fixed encoder retry state: a failed +3/-3 sweep now restarts at + electrical direction instead of continuing reverse-only until timeout.\n- Rebased per-window encoder transition counters on retry while preserving transaction-wide clean-edge/state evidence.\n- Encoder ratio inference now accepts the stronger valid net displacement from either forward or reverse half-sweep, useful when steering starts against one mechanical stop.\n- Integrated LEFT encoder / RIGHT Hall commissioning uses 1.00 A instead of 0.50 A; individual detect remains user-selectable and hard-capped by the existing 2 A safety limit.\n- Full tester now checks Rotor Position with upstream VESC `COMM_SET_DETECT` streaming semantics instead of treating `COMM_ROTOR_POSITION` as a request.\n- Firmware/tester release identity updated to V21.\n'''
if 'Hardware correction — 2026-08-13 17:49 run' not in s:
    s += append
p.write_text(s)

print('V21 17:49 hardware-log patch applied')
