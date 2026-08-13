#!/usr/bin/env python3
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]

def read(path):
    return (ROOT / path).read_text()

def write(path, text):
    (ROOT / path).write_text(text)

def rep(path, old, new, label, count=1):
    p = ROOT / path
    s = p.read_text()
    n = s.count(old)
    if n != count:
        raise SystemExit(f"{label}: expected {count} match(es), got {n}")
    p.write_text(s.replace(old, new, count))

def replace_region(path, start, end, replacement, label):
    p = ROOT / path
    s = p.read_text()
    a = s.find(start)
    if a < 0: raise SystemExit(f"{label}: start anchor missing")
    b = s.find(end, a)
    if b < 0: raise SystemExit(f"{label}: end anchor missing")
    p.write_text(s[:a] + replacement + s[b:])

# 1) The 19:23 log showed 49k ISR overruns and 128.7% max load after the
# V21 low-side-brake hot-path additions. Restore the proven V20 ADC/DMA/ISR
# implementation byte-for-byte. All new behavior must live above this layer.
v20_motor = subprocess.check_output(["git", "show", "origin/v20:Src/motor.c"], text=True)
write("Src/motor.c", v20_motor)

# 2) Restore V20 standard-current telemetry acquisition/validity timing exactly.
# Keep the other V21 protocol corrections (duty sign, config, commissioning, etc.).nv21 = read("Src/vesc_protocol.c")
v20 = subprocess.check_output(["git", "show", "origin/v20:Src/vesc_protocol.c"], text=True)
start = "#define VESC_CURRENT_HOLD_MAX_MS"
end = "static float motor_current_from_raw"
a1 = nv21.find(start); b1 = nv21.find(end, a1)
a0 = v20.find(start); b0 = v20.find(end, a0)
if min(a1,b1,a0,b0) < 0: raise SystemExit("telemetry block anchors missing")
nv21 = nv21[:a1] + v20[a0:b0] + nv21[b1:]
write("Src/vesc_protocol.c", nv21)

# 3) COMM_SET_DETECT is per controller upstream. Never clear the peer virtual
# controller's display mode as a side effect. That made Rotor Position fragile
# when VESC Tool switched local/CAN contexts.
rep("Src/vesc_protocol.c",
'''case C_SET_DETECT:if(l>=1){uint8_t mode=d[0];if(mode>7U)mode=0U;if(r){display_position_mode_right=mode;display_position_last_right_ms=0U;if(mode!=0U)display_position_mode_left=0U;}else{display_position_mode_left=mode;display_position_last_left_ms=0U;if(mode!=0U)display_position_mode_right=0U;}RuntimeControl_VescAlive();}break;''',
'''case C_SET_DETECT:if(l>=1){uint8_t mode=d[0];if(mode>7U)mode=0U;if(r){display_position_mode_right=mode;display_position_last_right_ms=0U;}else{display_position_mode_left=mode;display_position_last_left_ms=0U;}RuntimeControl_VescAlive();}break;''',
"COMM_SET_DETECT per-node stream")

# 4) DUTY(0) is not an unconditional hardware-low-side path in upstream VESC.
# Restore normal sensored FOC readiness for DUTY, while OPEN/HANDBRAKE retain
# their independent phase semantics.
rep("Src/runtime_control.c",
'''    /* VESC Full Brake is SET_DUTY(0). It does not need rotor angle because the
     * F103 implementation shorts all three low sides directly. OPEN and
     * handbrake also own their phase independently from the feedback backend. */
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE ||
        (mode == ESC_MODE_DUTY && target == 0)) return false;''',
'''    /* OPEN and handbrake own their phase independently. SET_DUTY(0) remains
     * the normal VESC DUTY control state at zero modulation; static low-side
     * shorting is only a separate foc_short_ls_on_zero_duty policy upstream and
     * must not bypass this board's proven sensored FOC/ADC path. */
    (void)target;
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE) return false;''',
"prearm DUTY zero semantics")

# 5) RIGHT Hall position is a board extension. The 19:23 trace showed a 15-degree
# request saturating the position controller and driving +/-6.5k eRPM. Cap only
# Hall-position Iq in the slow control layer; do not add work to the DMA ISR.
rep("Src/runtime_control.c",
'''        motor->m_pos_pid_set = target_host;
        const int16_t iq = mc_foc_run_pid_control_pos(motor, dt_ms);
        *runtime_command = iq_to_host_permille(cfg, conf, iq);
        return;''',
'''        motor->m_pos_pid_set = target_host;
        int16_t iq = mc_foc_run_pid_control_pos(motor, dt_ms);
        if (cfg->sensor_type == MOTOR_SENSOR_HALL_UVW) {
            /* One Hall sector is about 4 mechanical degrees at 15 pole-pairs.
             * Limit the coarse Hall position loop to 1 A so one sector of error
             * cannot command the full 15-A current limit. Encoder POS is not
             * affected. */
            int16_t hall_cap = (int16_t)CONTROL_CURRENT_INTERNAL_PER_A;
            if (hall_cap > conf->l_current_max) hall_cap = conf->l_current_max;
            if (iq > hall_cap) iq = hall_cap;
            if (iq < -hall_cap) iq = (int16_t)-hall_cap;
            motor->m_iq_set = iq;
        }
        *runtime_command = iq_to_host_permille(cfg, conf, iq);
        return;''',
"Hall position slow-layer current cap")

# 6) Position PID had a real saturation bug: p_limited was computed for anti-
# windup but the final output summed the raw, potentially huge p_q16. Use the
# bounded term. Also make the new default conservative; persisted old gains are
# still protected by the Hall current cap above.
rep("Src/foc_motor.c",
'''    conf->p_pid_kp_q16 = 16384;
    conf->p_pid_ki_q16 = 0;''',
'''    conf->p_pid_kp_q16 = 1024;        /* 0.015625: safe coarse-Hall default */
    conf->p_pid_ki_q16 = 0;''',
"position default kp")
rep("Src/foc_motor.c",
'''    conf->p_pid_gain_dec_ticks = 0U;''',
'''    conf->p_pid_gain_dec_ticks = 8U;       /* taper near target; ~32 deg on 15-pp Hall */''',
"position gain decrease default")
rep("Src/foc_motor.c",
'''    const int32_t output_q16 = clamp_s32((int64_t)p_q16 + pid->integrator_q16 +
        pid->derivative_filter_q16 + pid->process_derivative_filter_q16, -65536, 65536);''',
'''    const int32_t output_q16 = clamp_s32((int64_t)p_limited + pid->integrator_q16 +
        pid->derivative_filter_q16 + pid->process_derivative_filter_q16, -65536, 65536);''',
"position bounded P term")

# 7) Match the standard brake sign expression without the V21 arbitrary 2-rpm
# hard deadband. Exact zero produces zero Iq; moving speed produces opposing Iq.
rep("Src/foc_motor.c",
'''        /* 2 mechanical RPM deadband (Q4=32). Without this, a stationary Hall
         * estimator toggling +1/-1 RPM reverses brake Iq every control update and
         * produces the audible/mechanical jitter seen on RIGHT in V20. */
        const int16_t speed_q4 = sample->speed_rpm_q4;
        if (speed_q4 > -32 && speed_q4 < 32) iq_target = 0;
        else iq_target = speed_q4 < 0 ? mag : (int16_t)-mag;
        id_target = 0;''',
'''        /* VESC brake current is a magnitude and the fast loop applies the
         * opposite sign of measured speed. At exactly zero speed SIGN(0)=0,
         * therefore dynamic brake current is zero; use HANDBRAKE for static hold. */
        const int16_t speed_q4 = sample->speed_rpm_q4;
        if (speed_q4 < 0) iq_target = mag;
        else if (speed_q4 > 0) iq_target = (int16_t)-mag;
        else iq_target = 0;
        id_target = 0;''',
"current brake sign")

# 8) VESC l_current_max and l_current_min are independent MCCONF parameters.
# Preserve that semantics inside this board's safe +/-15 A phase-current range
# instead of silently forcing min=-max (which triggers Parameters truncated).
rep("Src/foc_motor.c",
'''    if (conf->l_current_max < 1) conf->l_current_max = 1;
    conf->l_current_min = (int16_t)-conf->l_current_max;''',
'''    if (conf->l_current_max < 1) conf->l_current_max = 1;
    if (conf->l_current_max > 12000) conf->l_current_max = 12000; /* 15 A board limit */
    if (conf->l_current_min >= 0) conf->l_current_min = -1;
    if (conf->l_current_min < -12000) conf->l_current_min = -12000;''',
"independent VESC current limits")

# 9) Persist per-motor max/min current independently. Keep legacy global words for
# migration, add four v19 words, and accept v18 images then auto-migrate.
rep("Src/eeprom.h",
'''#define NB_OF_VAR             ((uint8_t)158U)        /* v17/V19: + encoder ratio + steering 0..360 calibration */''',
'''#define NB_OF_VAR             ((uint8_t)162U)        /* v19: + per-motor max/min VESC current limits */''',
"EEPROM variable count")

p = ROOT / "Src/runtime_control.c"
s = p.read_text()
s = s.replace(
'''2150,2151,2152,2153,2154,2155,2156,2157\n};''',
'''2150,2151,2152,2153,2154,2155,2156,2157,2158,2159,2160,2161\n};''', 1)
s = s.replace(
'''#define EEPROM_CONFIG_VERSION 18U
#define EEPROM_CONFIG_VERSION_V17 17U''',
'''#define EEPROM_CONFIG_VERSION 19U
#define EEPROM_CONFIG_VERSION_V18 18U
#define EEPROM_CONFIG_VERSION_V17 17U''', 1)
s = s.replace(
'''#define EEPROM_LEFT_GEAR_RATIO_MILLI          156U
#define EEPROM_RIGHT_GEAR_RATIO_MILLI         157U''',
'''#define EEPROM_LEFT_GEAR_RATIO_MILLI          156U
#define EEPROM_RIGHT_GEAR_RATIO_MILLI         157U
#define EEPROM_LEFT_CURRENT_MAX                158U
#define EEPROM_LEFT_CURRENT_MIN                159U
#define EEPROM_RIGHT_CURRENT_MAX               160U
#define EEPROM_RIGHT_CURRENT_MIN               161U''', 1)
s = s.replace(
'''    w[EEPROM_LEFT_GEAR_RATIO_MILLI] = motorConfLeft.si_gear_ratio_milli;
    w[EEPROM_RIGHT_GEAR_RATIO_MILLI] = motorConfRight.si_gear_ratio_milli;''',
'''    w[EEPROM_LEFT_GEAR_RATIO_MILLI] = motorConfLeft.si_gear_ratio_milli;
    w[EEPROM_RIGHT_GEAR_RATIO_MILLI] = motorConfRight.si_gear_ratio_milli;
    w[EEPROM_LEFT_CURRENT_MAX] = (uint16_t)motorConfLeft.l_current_max;
    w[EEPROM_LEFT_CURRENT_MIN] = (uint16_t)motorConfLeft.l_current_min;
    w[EEPROM_RIGHT_CURRENT_MAX] = (uint16_t)motorConfRight.l_current_max;
    w[EEPROM_RIGHT_CURRENT_MIN] = (uint16_t)motorConfRight.l_current_min;''', 1)
s = s.replace(
'''    const bool version_v17 = (version == EEPROM_CONFIG_VERSION_V17);''',
'''    const bool version_v18 = (version == EEPROM_CONFIG_VERSION_V18);
    const bool version_v17 = (version == EEPROM_CONFIG_VERSION_V17);''', 1)
s = s.replace(
'''!version_v15 && !version_v16 && !version_v17 && !current_version)) return false;''',
'''!version_v15 && !version_v16 && !version_v17 && !version_v18 && !current_version)) return false;''', 1)
s = s.replace(
'''    if (current_version) {
        if (w[EEPROM_LEFT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_LEFT_GEAR_RATIO_MILLI] > 60000U ||''',
'''    if (version_v18 || current_version) {
        if (w[EEPROM_LEFT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_LEFT_GEAR_RATIO_MILLI] > 60000U ||''', 1)
# Insert v19 current-limit load immediately after the gear/ratio v18+ block.
anchor = '''        if (w[EEPROM_RIGHT_STEER_CAL] > 1U) return false;'''
# The exact line may be combined with LEFT; use a more stable insertion before the
# next version-independent prepare block if present.
insert_anchor = '''    mc_foc_conf_prepare(&left);
    mc_foc_conf_prepare(&right);'''
if insert_anchor not in s:
    raise SystemExit("EEPROM current-load insertion anchor missing")
s = s.replace(insert_anchor,
'''    if (current_version) {
        left.l_current_max = (int16_t)w[EEPROM_LEFT_CURRENT_MAX];
        left.l_current_min = (int16_t)w[EEPROM_LEFT_CURRENT_MIN];
        right.l_current_max = (int16_t)w[EEPROM_RIGHT_CURRENT_MAX];
        right.l_current_min = (int16_t)w[EEPROM_RIGHT_CURRENT_MIN];
        if (left.l_current_max <= 0 || left.l_current_max > 12000 ||
            left.l_current_min >= 0 || left.l_current_min < -12000 ||
            right.l_current_max <= 0 || right.l_current_max > 12000 ||
            right.l_current_min >= 0 || right.l_current_min < -12000) return false;
    }

''' + insert_anchor, 1)
# Verification used to force both motors equal to legacy global max. Keep only
# physical/signed validity; v19 is intentionally per motor.
s = s.replace(
'''        if (conf[i]->l_current_max <= 0 || conf[i]->l_current_max != (int16_t)max_current_word) return false;''',
'''        if (conf[i]->l_current_max <= 0 || conf[i]->l_current_max > 12000 ||
            conf[i]->l_current_min >= 0 || conf[i]->l_current_min < -12000) return false;''', 1)
p.write_text(s)

# 10) Tester: correct stage names, test Rotor Position per available side, do not
# label SET_DUTY(0) as a sensor-independent forced low-side short, and exercise
# dynamic brake after spinning the motor so VESC brake semantics are measurable.
p = ROOT / "tools/vesc_full_test.py"
s = p.read_text()
s = s.replace(
'''AUTO_DETECT_STAGE_NAMES = {
    0: "IDLE", 1: "WAIT_CURRENT_CAL", 2: "LEFT_ENCODER",
    3: "RIGHT_HALL", 4: "LEFT_SYNC", 5: "REPLY",
}''',
'''AUTO_DETECT_STAGE_NAMES = {
    0: "IDLE", 1: "WAIT_CURRENT_CAL", 2: "RIGHT_HALL",
    3: "LEFT_ENCODER", 4: "LEFT_SYNC", 5: "REPLY",
}''', 1)
s = s.replace(
'''            self.result("14a_rotor_position_stream_modes", self.rotor_position_stream_test,
                        skip=not (left_ready and right_ready),
                        skip_reason="both sensor contexts must be commissioned for Rotor Position stream test")''',
'''            self.result("14a_rotor_position_stream_modes", self.rotor_position_stream_test,
                        skip=not (left_ready or right_ready),
                        skip_reason="no commissioned motor available for Rotor Position stream test")''', 1)
s = s.replace(
'''            # These two are sensor-independent hardware semantics and must run
            # even when one sensor commissioning path failed.
            self.result("14d_left_full_brake_then_stop", lambda: self.full_brake_and_stop_test("local"),
                        skip=not current_ok, skip_reason="current offsets not ready")
            self.result("14e_right_full_brake_then_stop", lambda: self.full_brake_and_stop_test("right"),
                        skip=not current_ok, skip_reason="current offsets not ready")''',
'''            # VESC Tool Full Brake is SET_DUTY(0), but the FOC low-side short is
            # a configuration option upstream. Test zero-modulation vs Stop only
            # on a commissioned feedback path; never force an ISR hardware short.
            self.result("14d_left_full_brake_then_stop", lambda: self.full_brake_and_stop_test("local"),
                        skip=not left_ready, skip_reason="LEFT feedback not commissioned")
            self.result("14e_right_full_brake_then_stop", lambda: self.full_brake_and_stop_test("right"),
                        skip=not right_ready, skip_reason="RIGHT feedback not commissioned")''', 1)
# Pre-spin dynamic brake. Insert immediately before normal stream command.
needle = '''        rows = self._stream_command(node, kind, value, self.args.motion_duration, label)'''
repl = '''        if kind == "brake":
            # Upstream CURRENT_BRAKE is direction-dependent. Establish measured
            # speed first; testing it at standstill incorrectly expects static Iq.
            spin_erpm = self.args.erpm if self.args.erpm > 0 else 900
            self._stream_command(node, "rpm", spin_erpm, max(0.55, self.args.motion_duration * 0.65), label + "_prespin")
            time.sleep(0.05)
        rows = self._stream_command(node, kind, value, self.args.motion_duration, label)'''
if s.count(needle) != 1: raise SystemExit("tester motion stream anchor mismatch")
s = s.replace(needle, repl, 1)
# Full brake doc/assumption: selected bridge may be normal FOC zero modulation,
# not guaranteed low-side static short. Keep DUTY state then verify Stop release.
s = s.replace(
'''        Full Brake must keep only the selected bridge active at duty zero; Stop is
        SET_CURRENT(0) and must release that bridge. This test is safe without a
        calibrated rotor sensor because the full-brake path is a static low-side''',
'''        Full Brake sends SET_DUTY(0); Stop sends SET_CURRENT(0). Upstream FOC may
        optionally convert equal zero-voltage PWM to a low-side short when
        foc_short_ls_on_zero_duty is enabled, so this test only requires safe zero
        modulation with the selected sensored controller active before Stop''', 1)
p.write_text(s)

# 11) Add hardware-log regression contract. Source-level tests intentionally pin
# the V20 ISR, slow-layer Hall POS cap, rotor stream independence and config fix.
test = ROOT / "tests/test_v21_hwlog_192318.py"
test.write_text('''from pathlib import Path\nROOT=Path(__file__).resolve().parents[1]\nmotor=(ROOT/"Src/motor.c").read_text()\nruntime=(ROOT/"Src/runtime_control.c").read_text()\nfoc=(ROOT/"Src/foc_motor.c").read_text()\nproto=(ROOT/"Src/vesc_protocol.c").read_text()\ntester=(ROOT/"tools/vesc_full_test.py").read_text()\n\ndef test_dma_isr_is_restored_to_v20_no_v21_brake_hotpath():\n    assert "left_low_side_brake" not in motor\n    assert "right_low_side_brake" not in motor\n    assert "left_current_brake_short" not in motor\n    assert "right_current_brake_short" not in motor\n    assert "V18 SENSOR CADENCE" in motor\n    assert "MotorSensor_UpdateHardwareEncoder" in motor\n    assert "motorFocSlotRight" in motor\n\ndef test_hall_position_is_bounded_in_slow_layer():\n    assert "cfg->sensor_type == MOTOR_SENSOR_HALL_UVW" in runtime\n    assert "CONTROL_CURRENT_INTERNAL_PER_A" in runtime\n    assert "motor->m_iq_set = iq;" in runtime\n    assert "p_limited + pid->integrator_q16" in foc\n    assert "p_pid_kp_q16 = 1024" in foc\n\ndef test_rotor_stream_mode_is_per_virtual_controller():\n    line=[x for x in proto.splitlines() if "case C_SET_DETECT:" in x][0]\n    assert "display_position_mode_right=mode" in line\n    assert "display_position_mode_left=mode" in line\n    assert "if(mode!=0U)display_position_mode_left=0U" not in line\n    assert "if(mode!=0U)display_position_mode_right=0U" not in line\n    assert "left_ready or right_ready" in tester\n\ndef test_standard_current_telemetry_timing_is_v20_contract():\n    assert "#define VESC_CURRENT_HOLD_MAX_MS 25U" in proto\n    assert "!MotorControl_CurrentMeasurementValid(left))" in proto\n\ndef test_current_limits_are_independent_and_persistent():\n    assert "conf->l_current_min = (int16_t)-conf->l_current_max" not in foc\n    assert "EEPROM_LEFT_CURRENT_MAX" in runtime\n    assert "EEPROM_LEFT_CURRENT_MIN" in runtime\n    assert "EEPROM_RIGHT_CURRENT_MAX" in runtime\n    assert "EEPROM_RIGHT_CURRENT_MIN" in runtime\n\ndef test_brake_is_dynamic_not_unconditional_isr_short():\n    assert "left_low_side_brake" not in motor\n    assert "if (speed_q4 < 0) iq_target = mag" in foc\n    assert 'if kind == "brake":' in tester\n    assert 'label + "_prespin"' in tester\n''')

# Update host test count and include the new contract at the end.
p = ROOT / "tests/run_host_tests.sh"
s = p.read_text()
s = s.replace("[1/28]", "[1/29]", 1)
for n in range(2,29):
    s = s.replace(f"[{n}/28]", f"[{n}/29]", 1)
s = s.replace('''echo "[28/29] V20 LEFT/position/rotor/VdVq/protocol contract"\npython3 tests/test_v20_left_position_rotor_vdq_protocol.py\n\necho "ALL_HOST_TESTS_PASS"''',
'''echo "[28/29] V20 LEFT/position/rotor/VdVq/protocol contract"\npython3 tests/test_v20_left_position_rotor_vdq_protocol.py\n\necho "[29/29] V21 19:23 hardware-log regression contract"\npython3 tests/test_v21_hwlog_192318.py\n\necho "ALL_HOST_TESTS_PASS"''', 1)
p.write_text(s)

# Changelog
p = ROOT / "CHANGELOG_V21.md"
s = p.read_text()
entry = '''\n## Hardware log 2026-08-13 19:23 corrections\n- Restored `Src/motor.c` byte-for-byte from V20: ADC DMA stays 16 kHz, both sensors sample at 16 kHz and LEFT/RIGHT FOC remains 8 kHz interleaved; V21 brake logic no longer adds work to the hot ISR.\n- Restored V20 current/Id/Iq/Vd/Vq averaging-validity timing for standard `COMM_GET_VALUES` telemetry.\n- `COMM_SET_DETECT` Rotor Position streaming is independent per virtual controller; selecting one node no longer clears the peer display mode.\n- Fixed position PID saturation (`p_limited` is now the term actually summed), added conservative defaults, and capped Hall-position output to 1 A in the slow loop to prevent the 19:23 RIGHT +/-6.5k-eRPM hunting.\n- Removed unconditional V21 low-side Full-Brake/current-brake shortcuts; `SET_DUTY(0)` returns to normal VESC zero-modulation FOC, while dynamic current brake opposes measured speed.\n- VESC motor current max/min are now independently editable inside the stock board +/-15 A safety limit and persisted independently for LEFT/RIGHT in EEPROM v19.\n'''
if entry not in s: s += entry
p.write_text(s)
print("V21 19:23 regression patch staged")
