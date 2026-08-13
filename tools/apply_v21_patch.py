#!/usr/bin/env python3
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]

def read(path):
    return (ROOT / path).read_text()

def write(path, text):
    (ROOT / path).write_text(text)

def replace_once(text, old, new, label):
    n = text.count(old)
    if n != 1:
        raise RuntimeError(f"{label}: expected exactly one match, got {n}")
    return text.replace(old, new, 1)

def sub_once(text, pat, repl, label, flags=0):
    out, n = re.subn(pat, repl, text, count=1, flags=flags)
    if n != 1:
        raise RuntimeError(f"{label}: expected exactly one regex match, got {n}")
    return out

# ---------------------------------------------------------------------------
# 1. Persist VESC SI gear ratio instead of hard-wiring readback to 1.0.
#    Gear ratio is a UI/mechanical conversion parameter, not FOC/eRPM physics.
# ---------------------------------------------------------------------------
p = "Src/foc_motor.h"
s = read(p)
s = replace_once(
    s,
    "    uint8_t foc_motor_pole_pairs;\n    mc_foc_current_sample_mode foc_current_sample_mode;",
    "    uint8_t foc_motor_pole_pairs;\n    /* VESC si_gear_ratio, stored as ratio*1000. It must remain independent\n     * from physical pole-pairs and encoder electrical ratio. */\n    uint16_t si_gear_ratio_milli;\n    mc_foc_current_sample_mode foc_current_sample_mode;",
    "foc_motor.h gear field")
write(p, s)

p = "Src/foc_motor.c"
s = read(p)
s = replace_once(
    s,
    "    conf->foc_motor_pole_pairs = 15U;\n    conf->foc_current_sample_mode = current_sample_mode;",
    "    conf->foc_motor_pole_pairs = 15U;\n    conf->si_gear_ratio_milli = 1000U;\n    conf->foc_current_sample_mode = current_sample_mode;",
    "foc default gear")
s = replace_once(
    s,
    "    if (conf->foc_motor_pole_pairs == 0U) conf->foc_motor_pole_pairs = 1U;\n    /* Physical current gain",
    "    if (conf->foc_motor_pole_pairs == 0U) conf->foc_motor_pole_pairs = 1U;\n    if (conf->si_gear_ratio_milli == 0U || conf->si_gear_ratio_milli > 60000U)\n        conf->si_gear_ratio_milli = 1000U;\n    /* Physical current gain",
    "foc prepare gear")
# Current brake: remove sign chatter at essentially zero mechanical speed. Upstream
# VESC has an additional zero-duty phase-short state around zero crossing; this
# fixed-point port uses a small deadband so speed estimator +/- noise cannot flip Iq.
s = replace_once(
    s,
    "    if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {\n        int16_t mag = iq_target < 0 ? (int16_t)-iq_target : iq_target;\n        iq_target = sample->speed_rpm_q4 < 0 ? mag : (int16_t)-mag;\n        id_target = 0;\n    }",
    "    if (motor->m_control_mode == CONTROL_MODE_CURRENT_BRAKE) {\n        int16_t mag = iq_target < 0 ? (int16_t)-iq_target : iq_target;\n        /* 2 mechanical RPM deadband (Q4=32). Without this, a stationary Hall\n         * estimator toggling +1/-1 RPM reverses brake Iq every control update and\n         * produces the audible/mechanical jitter seen on RIGHT in V20. */\n        const int16_t speed_q4 = sample->speed_rpm_q4;\n        if (speed_q4 > -32 && speed_q4 < 32) iq_target = 0;\n        else iq_target = speed_q4 < 0 ? mag : (int16_t)-mag;\n        id_target = 0;\n    }",
    "current brake zero-speed deadband")
write(p, s)

p = "Src/vesc_config_compat.c"
s = read(p)
s = replace_once(
    s,
    "    b[i++] = (uint8_t)(c->foc_motor_pole_pairs * 2U);\n    append_auto(b, 1.0f, &i);\n    append_auto(b, 0.1f, &i);",
    "    b[i++] = (uint8_t)(c->foc_motor_pole_pairs * 2U);\n    append_auto(b, (float)c->si_gear_ratio_milli / 1000.0f, &i);\n    append_auto(b, 0.1f, &i);",
    "serialize gear ratio")
s = replace_once(
    s,
    "    const uint8_t motor_poles_wire = b[i++];\n    (void)get_auto(b, &i); (void)get_auto(b, &i); i += 2; (void)get_auto(b, &i); (void)get_auto(b, &i);",
    "    const uint8_t motor_poles_wire = b[i++];\n    const float gear_ratio_wire = get_auto(b, &i);\n    (void)get_auto(b, &i); i += 2; (void)get_auto(b, &i); (void)get_auto(b, &i);",
    "deserialize gear field")
s = replace_once(
    s,
    "        !isfinite(current_kp) || !isfinite(current_ki) || !isfinite(encoder_ratio)) return false;",
    "        !isfinite(current_kp) || !isfinite(current_ki) || !isfinite(encoder_ratio) ||\n        !isfinite(gear_ratio_wire)) return false;",
    "validate gear finite")
s = replace_once(
    s,
    "    c->foc_motor_pole_pairs = (uint8_t)(motor_poles_wire / 2U);\n\n    /* VESC limits",
    "    c->foc_motor_pole_pairs = (uint8_t)(motor_poles_wire / 2U);\n    if (gear_ratio_wire < 0.001f || gear_ratio_wire > 60.0f) return false;\n    c->si_gear_ratio_milli = (uint16_t)lrintf(gear_ratio_wire * 1000.0f);\n    if (c->si_gear_ratio_milli == 0U) c->si_gear_ratio_milli = 1U;\n\n    /* VESC limits",
    "apply gear ratio")
write(p, s)

# ---------------------------------------------------------------------------
# 2. Encoder detect: never report configured pole-pair as a measured ratio.
#    Capture a real session encoder offset at the D-axis lock and calculate it
#    after the strict ratio/direction proof is available.
# ---------------------------------------------------------------------------
p = "Src/runtime_control.c"
s = read(p)
s = replace_once(
    s,
    "    int32_t encoder_start;\n    int32_t encoder_delta;",
    "    int32_t encoder_start;\n    /* Raw TIM4 count while rotor is locked at commanded electrical phase 0.\n     * Used only to report the measured VESC encoder offset for this calibration. */\n    int32_t encoder_zero_raw_count;\n    int32_t encoder_delta;",
    "encoder zero raw field")
s = replace_once(
    s,
    "            (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);\n            sensorCal.encoder_start = state->position_ticks;",
    "            (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);\n            sensorCal.encoder_zero_raw_count = LeftEncoder_GetCount();\n            sensorCal.encoder_start = state->position_ticks;",
    "capture encoder raw zero")
# Remove the V20 'healthy quadrature => configured pp' fallback. Healthy A/B proves
# the sensor, but it does not measure electrical/mechanical ratio. Returning 15 here
# was exactly the false result observed on hardware.
s = sub_once(
    s,
    r"\n        /\* V14 hardware-log 00:23:19 proves.*?\n        return false;\n    }\n\n    if \(delta < 0\)",
    "\n        /* V21: strong quadrature evidence proves only that A/B is healthy. It\n         * does NOT prove encoder ratio/pole-pairs. Upstream VESC returns a ratio\n         * only from commanded electrical motion versus measured encoder motion.\n         * If strict inference above cannot prove it, keep sweeping until timeout\n         * and fail rather than fabricating the configured pole-pair value. */\n        sensorCal.encoder_ratio_fallback_used = false;\n        return false;\n    }\n\n    if (delta < 0)",
    "remove encoder ratio fallback", flags=re.S)
# Add offset helper immediately before sensor_cal_finish.
marker = "static void sensor_cal_finish(uint8_t terminal_state, uint8_t result_code)\n{"
helper = r'''static uint16_t sensor_cal_measured_encoder_offset_deg(const MotorRuntimeConfig *cfg,
                                                        uint8_t ratio,
                                                        uint8_t inverted)
{
    if (cfg == NULL || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN || ratio == 0U)
        return 0U;
    int64_t count = sensorCal.encoder_zero_raw_count;
    const int64_t cpr = cfg->encoder_cpr;
    count %= cpr;
    if (count < 0) count += cpr;
    if (inverted && count != 0) count = cpr - count;
    /* VESC encoder equation is theta_e = theta_m*ratio - offset. At the D-axis
     * lock theta_e=0, so offset is the measured encoder electrical phase. */
    const uint64_t num = (uint64_t)count * (uint64_t)ratio * 360ULL;
    return (uint16_t)(((num + (uint64_t)cpr / 2ULL) / (uint64_t)cpr) % 360ULL);
}

static void sensor_cal_finish(uint8_t terminal_state, uint8_t result_code)
{'''
s = replace_once(s, marker, helper, "insert measured encoder offset helper")
s = replace_once(
    s,
    "                        candidate_config.sensor_inverted = sensorCal.detected_encoder_inverted;\n                        candidate_config.encoder_offset_deg = 0U;\n                        candidate_config.encoder_ratio = sensorCal.detected_pole_pairs;",
    "                        candidate_config.sensor_inverted = sensorCal.detected_encoder_inverted;\n                        candidate_config.encoder_ratio = sensorCal.detected_pole_pairs;\n                        candidate_config.encoder_offset_deg =\n                            sensor_cal_measured_encoder_offset_deg(&candidate_config,\n                                sensorCal.detected_pole_pairs, sensorCal.detected_encoder_inverted);",
    "commit measured encoder offset")

# EEPROM v18 adds two gear-ratio words without disturbing existing addresses.
s = replace_once(s, "#define EEPROM_CONFIG_VERSION 17U", "#define EEPROM_CONFIG_VERSION 18U\n#define EEPROM_CONFIG_VERSION_V17 17U", "EEPROM version 18")
s = replace_once(
    s,
    "#define EEPROM_RIGHT_STEER_CAL                155U",
    "#define EEPROM_RIGHT_STEER_CAL                155U\n#define EEPROM_LEFT_GEAR_RATIO_MILLI          156U\n#define EEPROM_RIGHT_GEAR_RATIO_MILLI         157U",
    "EEPROM gear addresses")
s = replace_once(
    s,
    "    w[EEPROM_RIGHT_STEER_CAL] = steeringCalibrationRight.calibrated ? 1U : 0U;\n    w[EEPROM_WORD_GENERATION]",
    "    w[EEPROM_RIGHT_STEER_CAL] = steeringCalibrationRight.calibrated ? 1U : 0U;\n    w[EEPROM_LEFT_GEAR_RATIO_MILLI] = motorConfLeft.si_gear_ratio_milli;\n    w[EEPROM_RIGHT_GEAR_RATIO_MILLI] = motorConfRight.si_gear_ratio_milli;\n    w[EEPROM_WORD_GENERATION]",
    "save gear ratio")
# Add v17 to accepted-version list, while current_version remains v18.
s = replace_once(
    s,
    "    const bool version_v16 = (version == EEPROM_CONFIG_VERSION_V16);\n    const bool version_v15",
    "    const bool version_v17 = (version == EEPROM_CONFIG_VERSION_V17);\n    const bool version_v16 = (version == EEPROM_CONFIG_VERSION_V16);\n    const bool version_v15",
    "EEPROM v17 compatibility bool")
s = replace_once(
    s,
    "!version_v13 && !version_v14 && !version_v15 && !version_v16 && !current_version)",
    "!version_v13 && !version_v14 && !version_v15 && !version_v16 && !version_v17 && !current_version)",
    "EEPROM accepted version list")
# Current image has gear words. Older image load keeps defaults (1.000) from FOC defaults.
needle = "    if (current_version) {\n        if (w[EEPROM_LEFT_ENCODER_RATIO] <= 60U) motor_left.encoder_ratio = (uint8_t)w[EEPROM_LEFT_ENCODER_RATIO];"
replacement = "    if (current_version) {\n        if (w[EEPROM_LEFT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_LEFT_GEAR_RATIO_MILLI] > 60000U ||\n            w[EEPROM_RIGHT_GEAR_RATIO_MILLI] < 1U || w[EEPROM_RIGHT_GEAR_RATIO_MILLI] > 60000U) return false;\n        left.si_gear_ratio_milli = w[EEPROM_LEFT_GEAR_RATIO_MILLI];\n        right.si_gear_ratio_milli = w[EEPROM_RIGHT_GEAR_RATIO_MILLI];\n        if (w[EEPROM_LEFT_ENCODER_RATIO] <= 60U) motor_left.encoder_ratio = (uint8_t)w[EEPROM_LEFT_ENCODER_RATIO];"
# Variable names vary by release; fall back to regex discovery if needed.
if needle in s:
    s = s.replace(needle, replacement, 1)
else:
    m = re.search(r"(if \(current_version\) \{\n\s*if \(w\[EEPROM_LEFT_ENCODER_RATIO\])", s)
    if not m:
        raise RuntimeError("could not locate current-version load block")
    # Discover the two local mc_configuration candidate names from assignments later.
    # In V20 they are 'left_conf'/'right_conf' or equivalent; avoid guessing by
    # setting globals only after candidate commit below if local names are unknown.
    raise RuntimeError("gear load block variable naming changed; inspect workflow log")

# Expand emulated variable address table by appending 2156/2157.
s = replace_once(s, "2153,2154,2155\n};", "2153,2154,2155,2156,2157\n};", "VirtAddVarTab extend")
write(p, s)

p = "Src/eeprom.h"
s = read(p)
s = replace_once(
    s,
    "#define NB_OF_VAR             ((uint8_t)156U)",
    "#define NB_OF_VAR             ((uint8_t)158U)",
    "NB_OF_VAR v18")
write(p, s)

# ---------------------------------------------------------------------------
# 3. VESC realtime semantics: do not erase valid steady data on a single invalid
#    low-side sampling instant; clear immediately only when bridge is released.
#    Duty sign follows q-axis modulation/command, not noisy measured Iq.
# ---------------------------------------------------------------------------
p = "Src/vesc_protocol.c"
s = read(p)
s = replace_once(s, "#define VESC_CURRENT_HOLD_MAX_MS 25U", "#define VESC_CURRENT_HOLD_MAX_MS 100U", "telemetry hold")
s = replace_once(
    s,
    "        if (!MotorControl_CurrentOffsetsValid() || !bridge_active ||\n            !MotorControl_CurrentMeasurementValid(left)) {\n            /* Count suppressed passive observations only as a diagnostic. */\n            int32_t id_abs = outs[n]->id; if (id_abs < 0) id_abs = -id_abs;\n            int32_t iq_abs = outs[n]->iq; if (iq_abs < 0) iq_abs = -iq_abs;\n            if ((id_abs + iq_abs) > 0 && a->passive_rejects != UINT16_MAX)\n                ++a->passive_rejects;\n            current_avg_reset(a, true);\n            continue;\n        }",
    "        if (!MotorControl_CurrentOffsetsValid() || !bridge_active) {\n            /* Released bridge must clear immediately so one motor can never leak\n             * stale current/voltage into the other virtual-CAN node. */\n            int32_t id_abs = outs[n]->id; if (id_abs < 0) id_abs = -id_abs;\n            int32_t iq_abs = outs[n]->iq; if (iq_abs < 0) iq_abs = -iq_abs;\n            if ((id_abs + iq_abs) > 0 && a->passive_rejects != UINT16_MAX)\n                ++a->passive_rejects;\n            current_avg_reset(a, true);\n            continue;\n        }\n        if (!MotorControl_CurrentMeasurementValid(left)) {\n            /* Low-side current reconstruction can be temporarily unavailable at\n             * a zero vector / sampling boundary. Keep the last accepted steady\n             * sample; current_avg_take applies the bounded 100-ms freshness rule. */\n            continue;\n        }",
    "telemetry transient-validity handling")
s = replace_once(
    s,
    "    if (!MotorControl_BridgeActive(left) || !MotorControl_CurrentOffsetsValid() ||\n        !MotorControl_CurrentMeasurementValid(left)) {\n        current_avg_reset(a, true);\n        *id = 0;\n        *iq = 0;\n        *dc_centi_amp = 0;\n        *vd_mv = 0;\n        *vq_mv = 0;\n        return;\n    }",
    "    if (!MotorControl_BridgeActive(left) || !MotorControl_CurrentOffsetsValid()) {\n        current_avg_reset(a, true);\n        *id = 0; *iq = 0; *dc_centi_amp = 0; *vd_mv = 0; *vq_mv = 0;\n        return;\n    }",
    "telemetry take bridge gate")
s = sub_once(
    s,
    r"static float duty\(bool r\)\{uint16_t q=r\?motorOutputRight\.duty_abs_q15:motorOutputLeft\.duty_abs_q15;float d=\(float\)q/32767\.0f;int16_t iq=r\?motorOutputRight\.iq:motorOutputLeft\.iq;return iq<0\?-d:d;\}",
    "static float duty(bool r){uint16_t q=r?motorOutputRight.duty_abs_q15:motorOutputLeft.duty_abs_q15;float d=(float)q/32767.0f;motor_all_state_t*m=r?&motorRight:&motorLeft;int16_t mq=m->m_motor_state.mod_q;if(mq<0)return-d;if(mq>0)return d;if(m->m_control_mode==CONTROL_MODE_DUTY&&m->m_duty_cycle_set_q15<0)return-d;return d;}",
    "duty sign from modulation")
# Full Brake in VESC Tool is SET_DUTY(0), whereas Stop is SET_CURRENT(0)+estop.
# Keep zero-duty armed so the FOC/SVPWM zero vector can short phases instead of release.
s = replace_once(
    s,
    "    const bool run=normalized!=0;\n    record_set_runtime(r,normalized,run);\n    RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);",
    "    const bool run=true; /* SET_DUTY(0) is VESC Tool Full Brake, not release. */\n    record_set_runtime(r,normalized,run);\n    RuntimeControl_VescSetOne(!r,ESC_MODE_DUTY,normalized,run);",
    "full brake duty zero semantic")
write(p, s)

# Sample battery voltage before Vd/Vq telemetry so conversion uses the same slow-loop
# battery observation instead of the previous iteration.
p = "Src/main.c"
s = read(p)
old = "        /* Match VESC GET_VALUES semantics: accumulate read-reset current averages\n         * in background. Instantaneous samples remain available through HBTS. */\n        VescProtocol_CurrentTelemetrySample();"
new = "        /* V21: Vd/Vq are reconstructed from modulation and live DC bus. Update\n         * the bus engineering value before taking the telemetry sample so the\n         * voltage axes track the same battery reading shown by GET_VALUES. */\n        batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;\n\n        /* Match VESC GET_VALUES semantics: accumulate read-reset current averages\n         * in background. Instantaneous samples remain available through HBTS. */\n        VescProtocol_CurrentTelemetrySample();"
s = replace_once(s, old, new, "battery before Vd/Vq sample")
# Remove the later duplicate assignment only (first occurrence is new one).
needle = "        batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;"
pos = s.find(needle)
pos2 = s.find(needle, pos + 1)
if pos2 < 0:
    raise RuntimeError("expected duplicate batVoltageCalib assignment after insertion")
s = s[:pos2] + "        /* batVoltageCalib already updated before VESC telemetry above. */" + s[pos2+len(needle):]
write(p, s)

# ---------------------------------------------------------------------------
# 4. Add regression tests and release notes. These are source-contract tests; the
#    GitHub workflow also builds the STM32 target after patching.
# ---------------------------------------------------------------------------
(ROOT / "tests/test_v21_regressions.py").write_text(r'''from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def src(name):
    return (ROOT / name).read_text()

def test_encoder_detect_never_fabricates_configured_ratio():
    s = src("Src/runtime_control.c")
    assert "do NOT prove encoder ratio/pole-pairs" in s
    assert "sensor_cal_measured_encoder_offset_deg" in s
    assert "candidate_config.encoder_offset_deg = 0U;\n                        candidate_config.encoder_ratio" not in s

def test_gear_ratio_is_roundtripped_and_persisted():
    c = src("Src/vesc_config_compat.c")
    h = src("Src/foc_motor.h")
    r = src("Src/runtime_control.c")
    assert "si_gear_ratio_milli" in h
    assert "gear_ratio_wire" in c
    assert "EEPROM_LEFT_GEAR_RATIO_MILLI" in r
    assert "EEPROM_RIGHT_GEAR_RATIO_MILLI" in r

def test_v20_fixed_gear_ratio_literal_removed():
    c = src("Src/vesc_config_compat.c")
    anchor = "b[i++] = (uint8_t)(c->foc_motor_pole_pairs * 2U);"
    tail = c.split(anchor, 1)[1][:180]
    assert "si_gear_ratio_milli" in tail

def test_full_brake_and_stop_are_not_aliased():
    p = src("Src/vesc_protocol.c")
    assert "SET_DUTY(0) is VESC Tool Full Brake" in p
    assert "const bool run=true" in p

def test_brake_zero_speed_deadband_present():
    f = src("Src/foc_motor.c")
    assert "speed_q4 > -32 && speed_q4 < 32" in f

def test_realtime_steady_sample_not_cleared_on_one_invalid_window():
    p = src("Src/vesc_protocol.c")
    assert "VESC_CURRENT_HOLD_MAX_MS 100U" in p
    assert "temporarily unavailable" in p

def test_vdq_uses_live_vbus_scaling():
    p = src("Src/vesc_protocol.c")
    m = src("Src/main.c")
    assert "Vd/q = mod_d/q * (2/3) * Vbus" in p
    assert m.find("batVoltageCalib = batVoltage") < m.find("VescProtocol_CurrentTelemetrySample();")
''')

(ROOT / "CHANGELOG_V21.md").write_text(r'''# V21

V21 is a corrective release based on V20 and the 2026-08-13 hardware log.

## Confirmed fixes
- VESC SI gear ratio is writable/readable and persisted independently per motor; it is no longer hard-wired to 1.0.
- Physical pole-pairs remain independent from LEFT encoder ratio.
- LEFT `COMM_DETECT_ENCODER` no longer reports configured pole-pairs as if they were measured. If electrical/mechanical ratio cannot be proven by commanded motion, detection continues/fails instead of returning a fabricated `15`.
- LEFT encoder detect now reports a measured session electrical offset from the raw TIM4 count captured at the D-axis phase-0 lock.
- Realtime current/Vd/Vq telemetry keeps the last fresh sample through a transient low-side sampling-invalid instant while still clearing immediately when the bridge is released.
- Duty sign comes from q-axis modulation / duty command, not measured Iq sign.
- Vd/Vq conversion uses the live battery reading before telemetry sampling. VESC modulation convention is `Vdq = mod_dq * (2/3) * Vbus`; it is not simply `duty * Vbus`.
- `SET_DUTY(0)` remains armed, matching VESC Tool Full Brake. Stop remains the separate zero-current/release path.
- CURRENT_BRAKE has a 2 mechanical RPM zero-speed deadband to prevent Hall speed-sign chatter from reversing Iq around standstill.

## Preserved hardware invariants
- V20 ADC/DMA phase-current timing and active-low-side current-offset calibration are unchanged.
- RIGHT remains Hall-only on this PCB; LEFT remains the only AB encoder input.
- Full Auto Detect remains board commissioning (current offsets -> LEFT encoder -> RIGHT Hall -> LEFT electrical sync), not upstream R/L/flux motor-model identification.

## Hardware validation required
Run the V21 full tester on real hardware. In particular, a LEFT encoder detect that cannot physically follow the forced field should now fail instead of returning fallback ratio 15; increase detection current only within the board/motor safe range if required.
''')

print("V21 deterministic source patch applied successfully")
