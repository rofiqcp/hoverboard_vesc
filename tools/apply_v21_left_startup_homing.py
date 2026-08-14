#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=R/path; s=p.read_text()
    if s.count(old)!=1:
        raise SystemExit(f'{label}: expected 1 got {s.count(old)}')
    p.write_text(s.replace(old,new,1))

def replace_region(path, start, end, new, label):
    p=R/path; s=p.read_text(); a=s.find(start); b=s.find(end,a)
    if a<0 or b<0:
        raise SystemExit(f'{label}: region missing')
    p.write_text(s[:a]+new+s[b:])

# ---------------------------------------------------------------------------
# TIM4 has ONE accumulator owner: the 16-kHz DMA ISR. Slow-loop code may only
# read an atomic 32-bit snapshot; it must never advance previous_cnt/accumulated.
# ---------------------------------------------------------------------------
rep('Src/left_encoder.h',
'''bool LeftEncoder_IsEnabled(void);\nint32_t LeftEncoder_GetCount(void);''',
'''bool LeftEncoder_IsEnabled(void);\n/* Only the 16-kHz current-DMA ISR may advance the TIM4 software accumulator. */\nint32_t LeftEncoder_UpdateAndGetCount(void);\n/* Passive aligned 32-bit snapshot for slow-loop commissioning/diagnostics. */\nint32_t LeftEncoder_GetCount(void);''',
'encoder API single owner')

rep('Src/left_encoder.c',
'''static int32_t accumulated = 0;\nstatic uint16_t previous_cnt = 0;''',
'''static volatile int32_t accumulated = 0;\n/* previous_cnt is intentionally owned only by LeftEncoder_UpdateAndGetCount(),\n * which is called from the 16-kHz DMA ISR. */\nstatic uint16_t previous_cnt = 0;''',
'volatile accumulated')

replace_region('Src/left_encoder.c',
'''int32_t LeftEncoder_GetCount(void)\n{''',
'''void LeftEncoder_ZeroMechanical(void)''',
'''int32_t LeftEncoder_UpdateAndGetCount(void)
{
    if (!enabled) return accumulated;

    const uint16_t now = (uint16_t)TIM4->CNT;
    const int16_t delta = (int16_t)(now - previous_cnt);
    previous_cnt = now;

    if (delta > 0 && accumulated > INT32_MAX - delta) {
        accumulated = INT32_MAX;
    } else if (delta < 0 && accumulated < INT32_MIN - delta) {
        accumulated = INT32_MIN;
    } else {
        accumulated += delta;
    }
    return accumulated;
}

int32_t LeftEncoder_GetCount(void)
{
    /* Cortex-M3 aligned 32-bit read is atomic. Do NOT touch TIM4/previous_cnt
     * here: slow-loop detect/alignment used to race the 16-kHz ISR and could
     * duplicate/drop encoder deltas, corrupting the electrical phase proof. */
    return accumulated;
}

''',
'encoder single-owner implementation')

rep('Src/motor.c',
'''MotorSensor_UpdateHardwareEncoder(&motorConfigLeft, &motorSensorStateLeft,\n                       LeftEncoder_GetCount(), encoder_a, encoder_b,''',
'''MotorSensor_UpdateHardwareEncoder(&motorConfigLeft, &motorSensorStateLeft,\n                       LeftEncoder_UpdateAndGetCount(), encoder_a, encoder_b,''',
'ISR owns TIM4 accumulator')

# ---------------------------------------------------------------------------
# Startup encoder synchronization.
# A static phase-0 lock alone is not proof that a loaded steering rotor actually
# reached phase 0. Probe +120 electrical first, fall back to -120 when the first
# direction is blocked by a mechanical hard stop, return to phase 0, settle, and
# only then define the incremental encoder electrical zero.
# ---------------------------------------------------------------------------
rep('Src/runtime_control.c',
'''typedef struct {\n    bool active;\n    uint8_t motor;\n    uint8_t stage;\n    uint32_t start_tick;\n    uint32_t stage_tick;\n    int16_t target_current_internal;\n    int32_t probe_start_count;\n    int16_t probe_delta_counts;\n    bool direction_proved;\n    bool direction_changed;\n} EncoderAlignmentRuntime;''',
'''typedef struct {\n    bool active;\n    uint8_t motor;\n    uint8_t stage;\n    uint32_t start_tick;\n    uint32_t stage_tick;\n    int16_t target_current_internal;\n    int16_t phase_q4;\n    int8_t probe_direction;\n    uint8_t probe_attempts;\n    int32_t probe_start_count;\n    int16_t probe_delta_counts;\n    bool direction_proved;\n    bool direction_changed;\n} EncoderAlignmentRuntime;''',
'alignment runtime fields')

replace_region('Src/runtime_control.c',
'''static void encoder_alignment_start(uint8_t motor)''',
'''static void Homing_SetDefaults(void)''',
'''static int16_t encoder_alignment_probe_delta_s16(int32_t delta)
{
    if (delta > INT16_MAX) return INT16_MAX;
    if (delta < INT16_MIN) return INT16_MIN;
    return (int16_t)delta;
}

static bool encoder_alignment_probe_matches(const MotorRuntimeConfig *cfg,
                                            int8_t direction, int32_t delta)
{
    if (cfg == NULL || cfg->encoder_cpr < MOTOR_ENCODER_CPR_MIN ||
        cfg->encoder_ratio == 0U || direction == 0) return false;

    /* 120 electrical degrees = one third electrical revolution. Because the
     * fast runtime phase uses encoder_ratio, the same physical proof must move
     * approximately CPR/(3*ratio) normalized TIM4 counts. position_ticks is
     * already normalized by sensor_inverted in the 16-kHz hardware path. */
    const uint32_t den = 3U * (uint32_t)cfg->encoder_ratio;
    uint32_t expected = ((uint32_t)cfg->encoder_cpr + den / 2U) / den;
    if (expected < 2U) expected = 2U;

    int64_t d = delta;
    const bool sign_ok = direction > 0 ? d > 0 : d < 0;
    if (!sign_ok) return false;
    if (d < 0) d = -d;
    const uint32_t mag = d > UINT32_MAX ? UINT32_MAX : (uint32_t)d;

    /* Detect already proved the ratio. Startup only verifies that the rotor
     * follows that electrical field. Allow load/compliance, but reject a tiny
     * twitch or a count rate incompatible with the persisted ratio. */
    uint32_t lower = (expected * 55U) / 100U;
    if (lower < 2U) lower = 2U;
    const uint32_t upper = (expected * 145U) / 100U + 2U;
    return mag >= lower && mag <= upper;
}

static void encoder_alignment_fail(bool left, MotorSensorState *state)
{
    if (state != NULL) state->encoder_electrical_aligned = false;
    if (left) {
        encoderAlignedLeft = false;
        armRequestedLeft = false;
        armRejectLeft = ESC_ARM_REJECT_SENSOR_OR_FOC;
    } else {
        encoderAlignedRight = false;
        armRequestedRight = false;
        armRejectRight = ESC_ARM_REJECT_SENSOR_OR_FOC;
    }
    encoder_alignment_abort();
}

static void encoder_alignment_start(uint8_t motor)
{
    fault_report_reset_all();
    const uint8_t bit = motor == ESC_MOTOR_RIGHT ? 0x02U : 0x01U;
    sensorCalibrationFastCurrentFaultMask &= (uint8_t)~bit;
    MotorControl_ClearCommissioningFaultSnapshot(motor == ESC_MOTOR_LEFT);
    memset(&encoderAlign, 0, sizeof(encoderAlign));
    encoderAlign.active = true;
    encoderAlign.motor = motor;
    encoderAlign.stage = 0U;
    encoderAlign.start_tick = RuntimeControl_MonotonicMs();
    encoderAlign.stage_tick = encoderAlign.start_tick;
    encoderAlign.target_current_internal = encoder_alignment_target_current(motor);
    encoderAlign.phase_q4 = 0;
    encoderAlign.probe_direction = 1;

    int16_t start_current = (int16_t)(encoderAlign.target_current_internal / 2);
    if (start_current < SENSOR_CAL_CURRENT_MIN_INTERNAL)
        start_current = SENSOR_CAL_CURRENT_MIN_INTERNAL;
    if (start_current > encoderAlign.target_current_internal)
        start_current = encoderAlign.target_current_internal;
    sensor_cal_set_current_override(motor, true, 0, start_current);
    if (motor == ESC_MOTOR_RIGHT) controlModeRightFoc = (uint8_t)CONTROL_MODE_CURRENT;
    else controlModeLeftFoc = (uint8_t)CONTROL_MODE_CURRENT;
    refresh_master_enable();
}

static bool encoder_alignment_service(uint32_t now)
{
    if (!encoderAlign.active) return false;
    const bool left = encoderAlign.motor == ESC_MOTOR_LEFT;
    const uint8_t bit = left ? 0x01U : 0x02U;
    MotorRuntimeConfig *cfg = left ? &motorConfigLeft : &motorConfigRight;
    MotorSensorState *state = left ? &motorSensorStateLeft : &motorSensorStateRight;
    mc_configuration *conf = left ? &motorConfLeft : &motorConfRight;

    if ((sensorCalibrationFastCurrentFaultMask & bit) != 0U ||
        (motorControlOvercurrentFaultMask & bit) != 0U ||
        cfg->sensor_type != MOTOR_SENSOR_ENCODER_AB ||
        cfg->encoder_calibrated == 0U || cfg->encoder_sequence_valid == 0U ||
        cfg->encoder_ratio == 0U || !left) {
        encoder_alignment_fail(left, state);
        return false;
    }

    /* Stage 0: establish a stable D-axis lock at commanded electrical phase 0.
     * Current ramps from about 0.5 A to 1.0 A, but phase is not accepted as the
     * encoder zero yet. A following motion probe must prove the rotor is coupled. */
    if (encoderAlign.stage == 0U) {
        const uint32_t elapsed = now - encoderAlign.start_tick;
        int32_t current = encoderAlign.target_current_internal;
        if (elapsed < 350U) {
            int32_t start = current / 2;
            if (start < SENSOR_CAL_CURRENT_MIN_INTERNAL) start = SENSOR_CAL_CURRENT_MIN_INTERNAL;
            current = start + ((current - start) * (int32_t)elapsed) / 350;
        }
        encoderAlign.phase_q4 = 0;
        sensor_cal_set_current_override(encoderAlign.motor, true, 0, (int16_t)current);
        refresh_master_enable();
        if (elapsed < 700U) return true;
        encoderAlign.probe_start_count = state->position_ticks;
        encoderAlign.stage = 1U;
        encoderAlign.stage_tick = now;
        return true;
    }

    /* Stage 1: slowly move the forced electrical field by +/-120 degrees.
     * This is slow-loop only (~200 Hz); ISR cadence/work remains unchanged. */
    if (encoderAlign.stage == 1U) {
        uint32_t dt_ms = now - encoderAlign.stage_tick;
        encoderAlign.stage_tick = now;
        if (dt_ms == 0U) dt_ms = 1U;
        if (dt_ms > 50U) dt_ms = 50U;
        int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
        const int32_t target = encoderAlign.probe_direction > 0
            ? SENSOR_CAL_ENCODER_PROBE_Q4 : -SENSOR_CAL_ENCODER_PROBE_Q4;
        int32_t phase = encoderAlign.phase_q4;
        if (phase < target) {
            phase += step;
            if (phase > target) phase = target;
        } else if (phase > target) {
            phase -= step;
            if (phase < target) phase = target;
        }
        encoderAlign.phase_q4 = (int16_t)phase;
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if (phase == target) {
            encoderAlign.stage = 2U;
            encoderAlign.stage_tick = now;
        }
        return true;
    }

    /* Stage 2: settle at +/-120, then compare normalized TIM4 displacement with
     * the encoder ratio already proven by COMM_DETECT_ENCODER. */
    if (encoderAlign.stage == 2U) {
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if ((now - encoderAlign.stage_tick) < SENSOR_CAL_ENCODER_PROBE_SETTLE_MS) return true;

        const int32_t delta = state->position_ticks - encoderAlign.probe_start_count;
        encoderAlign.probe_delta_counts = encoder_alignment_probe_delta_s16(delta);
        ++encoderAlign.probe_attempts;
        encoderAlign.direction_proved = encoder_alignment_probe_matches(
            cfg, encoderAlign.probe_direction, delta);
        if (!encoderAlign.direction_proved && encoderAlign.probe_attempts < 2U) {
            /* Positive probe can be blocked when steering already sits on that
             * mechanical stop. Return to phase 0 and retry the opposite direction. */
            encoderAlign.probe_direction = -1;
            encoderAlign.direction_changed = true;
        }
        encoderAlign.stage = 3U;
        encoderAlign.stage_tick = now;
        return true;
    }

    /* Stage 3: always return the stator field to exact electrical phase 0 before
     * either trying the opposite probe or committing the runtime zero. */
    if (encoderAlign.stage == 3U) {
        uint32_t dt_ms = now - encoderAlign.stage_tick;
        encoderAlign.stage_tick = now;
        if (dt_ms == 0U) dt_ms = 1U;
        if (dt_ms > 50U) dt_ms = 50U;
        int32_t step = (int32_t)SENSOR_CAL_PHASE_Q4_PER_MS * (int32_t)dt_ms;
        int32_t phase = encoderAlign.phase_q4;
        if (phase > 0) {
            phase -= step;
            if (phase < 0) phase = 0;
        } else if (phase < 0) {
            phase += step;
            if (phase > 0) phase = 0;
        }
        encoderAlign.phase_q4 = (int16_t)phase;
        sensor_cal_set_current_override(encoderAlign.motor, true,
                                        encoderAlign.phase_q4,
                                        encoderAlign.target_current_internal);
        refresh_master_enable();
        if (phase != 0) return true;

        if (encoderAlign.direction_proved) {
            encoderAlign.stage = 4U;
            encoderAlign.stage_tick = now;
            return true;
        }
        if (encoderAlign.probe_attempts < 2U) {
            encoderAlign.probe_start_count = state->position_ticks;
            encoderAlign.stage = 1U;
            encoderAlign.stage_tick = now;
            return true;
        }

        /* Neither direction produced the expected count/ratio relationship.
         * Never run DUTY/CURRENT/RPM/POS with a guessed electrical zero. */
        encoder_alignment_fail(left, state);
        return false;
    }

    /* Stage 4: phase is proven and back at zero. Hold briefly, then rebuild the
     * incremental phase accumulator at the current TIM4 ISR-owned snapshot and
     * define THIS boot's electrical zero. Nothing electrical is persisted. */
    sensor_cal_set_current_override(encoderAlign.motor, true, 0,
                                    encoderAlign.target_current_internal);
    refresh_master_enable();
    if ((now - encoderAlign.stage_tick) < 350U) return true;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    MotorSensor_Reset(state);
    MotorSensor_PrepareRuntime(cfg, state, conf->foc_motor_pole_pairs);
    const int32_t raw = LeftEncoder_GetCount(); /* passive ISR-owned snapshot */
    state->encoder_hw_last_count = raw;
    state->encoder_hw_count_initialized = true;
    state->position_ticks = 0;
    state->encoder_speed_reference_ticks = 0;
    state->encoder_speed_window_count = 0U;
    state->encoder_speed_q4 = 0;
    state->encoder_speed_initialized = false;
    (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);
    encoderAlignedLeft = true;
    if (primask == 0U) __enable_irq();

    position_session_rezero(true);
    sensor_cal_set_current_override(encoderAlign.motor, false, 0, 0);
    encoderAlign.active = false;
    refresh_master_enable();
    return false;
}

''',
'alignment proof state machine')

# Update the older startup contract: boot must now PROVE phase coupling instead
# of intentionally avoiding all +/-120 electrical motion.
p=R/'tests/test_v20_left_position_rotor_vdq_protocol.py'; s=p.read_text()
old='''# Boot sync must not re-learn direction at a steering hard-stop. Direction/ratio\n# are commissioning proof; boot only performs a current-regulated phase-0 lock.\nassert '1.00 A D-axis phase-0 lock' in runtime\na=runtime.index('static int16_t encoder_alignment_target_current')\nb=runtime.index('static void Homing_SetDefaults', a)\nboot=runtime[a:b]\nassert 'wanted_inverted' not in boot and '+120 deg electrical' not in boot\nassert 'MotorSensor_SyncEncoderElectricalPhase(state, 0U)' in boot'''
new='''# Boot must not re-learn/persist inversion, but it MUST prove that the loaded\n# rotor follows the already-detected encoder ratio before accepting phase zero.\nassert '1.00 A D-axis phase-0 lock' in runtime\na=runtime.index('static int16_t encoder_alignment_target_current')\nb=runtime.index('static void Homing_SetDefaults', a)\nboot=runtime[a:b]\nassert 'wanted_inverted' not in boot\nassert 'encoder_alignment_probe_matches' in boot\nassert 'SENSOR_CAL_ENCODER_PROBE_Q4' in boot\nassert 'encoderAlign.probe_direction = -1' in boot\nassert 'Never run DUTY/CURRENT/RPM/POS with a guessed electrical zero' in boot\nassert boot.index('encoderAlign.direction_proved') < boot.rindex('MotorSensor_SyncEncoderElectricalPhase(state, 0U)')'''
if s.count(old)!=1: raise SystemExit('v20 startup contract block')
p.write_text(s.replace(old,new,1))

# New regression: single TIM4 owner + phase-proof architecture.
(R/'tests/test_v21_left_phase_chain.py').write_text(r'''from pathlib import Path
R=Path(__file__).resolve().parents[1]
def txt(p): return (R/p).read_text()

def test_tim4_accumulator_has_single_owner():
    h=txt('Src/left_encoder.h'); c=txt('Src/left_encoder.c'); m=txt('Src/motor.c'); r=txt('Src/runtime_control.c')
    assert 'LeftEncoder_UpdateAndGetCount' in h
    update=c[c.index('int32_t LeftEncoder_UpdateAndGetCount'):c.index('int32_t LeftEncoder_GetCount')]
    snap=c[c.index('int32_t LeftEncoder_GetCount'):c.index('void LeftEncoder_ZeroMechanical')]
    assert 'TIM4->CNT' in update and 'previous_cnt = now' in update
    assert 'TIM4->CNT' not in snap and 'previous_cnt' not in snap and 'return accumulated' in snap
    assert 'LeftEncoder_UpdateAndGetCount(), encoder_a, encoder_b' in m
    assert 'LeftEncoder_UpdateAndGetCount' not in r

def test_left_startup_phase_is_proved_before_sync():
    r=txt('Src/runtime_control.c')
    a=r.index('static bool encoder_alignment_probe_matches')
    b=r.index('static void Homing_SetDefaults',a)
    block=r[a:b]
    assert 'CPR/(3*ratio)' in block
    assert 'SENSOR_CAL_ENCODER_PROBE_Q4' in block
    assert 'encoderAlign.probe_direction = -1' in block
    assert 'encoderAlign.probe_attempts < 2U' in block
    assert 'encoder_alignment_fail(left, state)' in block
    assert 'MotorSensor_SyncEncoderElectricalPhase(state, 0U)' in block
    # Sync is terminal: it is impossible to reach before direction_proved stage.
    assert block.index('encoderAlign.direction_proved') < block.rindex('MotorSensor_SyncEncoderElectricalPhase(state, 0U)')

def test_isr_cadence_unchanged():
    m=txt('Src/motor.c')
    assert 'sample BOTH feedback paths on EVERY 16-kHz DMA sample' in m
    assert 'motorFocSlotRight = !motorFocSlotRight' in m
    assert 'MotorSensor_UpdateHardwareEncoder' in m
''')

# Numeric end-to-end sensor phase -> FOC phase/SVPWM test.
(R/'tests/test_v21_left_phase_chain.c').write_text(r'''#include <assert.h>
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
''')

p=R/'tests/run_host_tests.sh'; s=p.read_text().replace('/29]', '/30]')
anchor='''echo "ALL_HOST_TESTS_PASS"'''
insert='''echo "[30/30] LEFT TIM4 encoder phase -> FOC/SVPWM numeric chain"\n"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_v21_left_phase_chain.c \\\n  Src/motor_sensor.c Src/foc_motor.c Src/foc_motor_data.c -o /tmp/test_v21_left_phase_chain\n/tmp/test_v21_left_phase_chain\n\necho "ALL_HOST_TESTS_PASS"'''
if s.count(anchor)!=1: raise SystemExit('host test append anchor')
p.write_text(s.replace(anchor,insert,1))

print('V21 LEFT encoder phase-chain patch staged')
