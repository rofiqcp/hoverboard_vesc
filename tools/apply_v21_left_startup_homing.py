#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=R/path; s=p.read_text()
    if s.count(old)!=1: raise SystemExit(f'{label}: expected 1 got {s.count(old)}')
    p.write_text(s.replace(old,new,1))

def replace_region(path,start,end,new,label):
    p=R/path; s=p.read_text(); a=s.find(start); b=s.find(end,a)
    if a<0 or b<0: raise SystemExit(f'{label}: region missing')
    p.write_text(s[:a]+new+s[b:])

# Startup electrical sync: no direction re-detect. Commissioning already proved
# A/B sequence, inversion and ratio. Each boot only creates a fresh electrical
# zero by locking the rotor to stator phase 0 at about 1 A.
replace_region('Src/runtime_control.c',
'''static int32_t encoder_probe_count(void)''',
'''static void Homing_SetDefaults(void)''',
'''static int16_t encoder_alignment_target_current(uint8_t motor)
{
    const mc_configuration *conf = sensor_cal_params(motor);
    int32_t units = conf->foc_current_units_per_amp ?
        (int32_t)conf->foc_current_units_per_amp : 800;
    int32_t target = units; /* 1.00 A D-axis phase-0 lock */
    if (target < SENSOR_CAL_CURRENT_MIN_INTERNAL) target = SENSOR_CAL_CURRENT_MIN_INTERNAL;
    if (target > SENSOR_CAL_CURRENT_MAX_INTERNAL) target = SENSOR_CAL_CURRENT_MAX_INTERNAL;
    if (target > conf->l_current_max) target = conf->l_current_max;
    if (target < 1) target = 1;
    return (int16_t)target;
}

static void encoder_alignment_abort(void)
{
    if (!encoderAlign.active) return;
    const uint8_t motor = encoderAlign.motor;
    sensor_cal_set_current_override(motor, false, 0, 0);
    encoderAlign.active = false;
    refresh_master_enable();
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
        cfg->encoder_ratio == 0U) {
        state->encoder_electrical_aligned = false;
        if (left) encoderAlignedLeft = false; else encoderAlignedRight = false;
        encoder_alignment_abort();
        return false;
    }

    const uint32_t elapsed = now - encoderAlign.start_tick;
    int32_t current = encoderAlign.target_current_internal;
    if (elapsed < 350U) {
        int32_t start = current / 2;
        if (start < SENSOR_CAL_CURRENT_MIN_INTERNAL) start = SENSOR_CAL_CURRENT_MIN_INTERNAL;
        current = start + ((current - start) * (int32_t)elapsed) / 350;
    }
    sensor_cal_set_current_override(encoderAlign.motor, true, 0, (int16_t)current);
    refresh_master_enable();
    if (elapsed < 1100U) return true;

    /* Rotor is held at stator electrical phase 0. Rebuild the incremental runtime
     * accumulator at the current TIM4 count, then define that physical lock as
     * electrical 0. Mechanical steering zero is deliberately NOT persisted here;
     * hard-stop homing owns the 0..360 coordinate. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    MotorSensor_Reset(state);
    MotorSensor_PrepareRuntime(cfg, state, conf->foc_motor_pole_pairs);
    if (left) {
        const int32_t raw = LeftEncoder_GetCount();
        state->encoder_hw_last_count = raw;
        state->encoder_hw_count_initialized = true;
    }
    state->position_ticks = 0;
    state->encoder_speed_reference_ticks = 0;
    state->encoder_speed_window_count = 0U;
    state->encoder_speed_q4 = 0;
    state->encoder_speed_initialized = false;
    (void)MotorSensor_SyncEncoderElectricalPhase(state, 0U);
    if (left) encoderAlignedLeft = true; else encoderAlignedRight = true;
    if (primask == 0U) __enable_irq();

    position_session_rezero(left);
    sensor_cal_set_current_override(encoderAlign.motor, false, 0, 0);
    encoderAlign.active = false;
    refresh_master_enable();
    return false;
}

''',
'static startup electrical sync')

# A successful full LEFT hard-stop calibration becomes the boot policy: future
# boots perform one right-stop home then return to 180 using the saved span.
rep('Src/runtime_control.c',
'''        cal->span_ticks = span;
        cal->calibrated = 1U;
        cal->homed = 1U;
        settingsDirty = true;''',
'''        cal->span_ticks = span;
        cal->calibrated = 1U;
        cal->homed = 1U;
        if (left) homingOnBootLeft = true;
        settingsDirty = true;''',
'persist left boot rehome')

# Expose a compact readiness predicate for integrated VESC Apply-All and tester.
rep('Src/runtime_control.h',
'''bool RuntimeControl_HomingOnBoot(bool left);''',
'''bool RuntimeControl_HomingOnBoot(bool left);
bool RuntimeControl_SteeringReady(bool left);''',
'steering ready prototype')
rep('Src/runtime_control.c',
'''uint8_t RuntimeControl_HomingStateLeft(void) { return homingRuntimeLeft.state; }
uint8_t RuntimeControl_HomingStateRight(void) { return homingRuntimeRight.state; }''',
'''uint8_t RuntimeControl_HomingStateLeft(void) { return homingRuntimeLeft.state; }
uint8_t RuntimeControl_HomingStateRight(void) { return homingRuntimeRight.state; }
bool RuntimeControl_SteeringReady(bool left)
{
    const SteeringCalibration *cal = left ? &steeringCalibrationLeft : &steeringCalibrationRight;
    const MotorSensorState *sensor = left ? &motorSensorStateLeft : &motorSensorStateRight;
    return cal->calibrated != 0U && cal->homed != 0U && cal->span_ticks != 0 &&
           sensor->encoder_electrical_aligned;
}''',
'steering ready implementation')

# After an individual VESC encoder detect succeeds, keep the standard detect reply
# immediate but automatically launch the board-specific electrical sync + full
# right/left hard-stop calibration asynchronously.
rep('Src/runtime_control.c',
'''    /* Selalu antrekan satu final read-back setelah state terminal. */
    telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;''',
'''    /* Standard COMM_DETECT_ENCODER result is already frozen above. Board-specific
     * steering commissioning continues asynchronously so VESC Tool is not held
     * without a reply during right-stop -> left-stop -> center travel. */
    if (success && sensorCal.vesc_wire_detect &&
        sensorCal.method == ESC_SENSOR_CAL_METHOD_AUTO &&
        sensorCal.motor == ESC_MOTOR_LEFT &&
        sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB &&
        sensorCal.state == ESC_SENSOR_CAL_SUCCESS) {
        (void)RuntimeControl_StartHomingCalibration(true);
    }

    /* Selalu antrekan satu final read-back setelah state terminal. */
    telemetryOneShotPage = ESC_TELEM_SENSOR_CAL;''',
'auto launch left steering homing')

# Persistent VESC MCCONF encoder offset is intentionally 0 on LEFT. Consume the
# wire field for ABI compatibility but never let it become the next boot's phase.
rep('Src/vesc_config_compat.c',
'''        while (encoder_offset < 0.0f) encoder_offset += 360.0f;
        while (encoder_offset >= 360.0f) encoder_offset -= 360.0f;
        r->encoder_offset_deg = (uint16_t)lrintf(encoder_offset);''',
'''        /* Incremental ABI electrical zero is re-created by the 1-A phase lock
         * every boot. A persistent encoder offset would double-apply that zero. */
        (void)encoder_offset;
        r->encoder_offset_deg = 0U;''',
'ignore persistent encoder offset')

# Detect reply also reports zero persistent offset. Ratio and inversion remain
# standard VESC commissioning results; transient phase-lock offset is HBTS-only.
rep('Src/runtime_control.c',
'''    snap->result.encoder_offset_deg =
        (sensorCal.sensor_type == MOTOR_SENSOR_ENCODER_AB &&
         sensorCal.detected_encoder_offset_deg < 360U)
            ? sensorCal.detected_encoder_offset_deg : cfg->encoder_offset_deg;''',
'''    snap->result.encoder_offset_deg = 0U;''',
'detect wire offset zero')

# Integrated Apply-All recognizes the asynchronous full steering calibration and
# waits for it instead of issuing a second encoder sync that races the homing owner.
rep('Src/vesc_protocol.c',
'''    case AUTO_DETECT_LEFT_ENCODER:
        if(RuntimeControl_VescPollSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,&res)){
            auto_detect.left_encoder_ok=(res.state==ESC_SENSOR_CAL_SUCCESS);
            if(auto_detect.left_encoder_ok && RuntimeControl_RequestEncoderSync(true)){
                auto_detect.stage=AUTO_DETECT_LEFT_SYNC;
            } else if(auto_detect.left_encoder_ok && RuntimeControl_EncoderElectricalReady(true)) {
                auto_detect.left_sync_ok=true;
                auto_detect.result=(auto_detect.right_hall_ok && RuntimeSettings_Save())?2:-1;
                auto_detect.stage=AUTO_DETECT_REPLY;
            } else {
                /* RIGHT proof, when successful, remains committed and usable. */
                auto_detect.result=-1;
                auto_detect.stage=AUTO_DETECT_REPLY;
            }
        }
        break;
    case AUTO_DETECT_LEFT_SYNC:
        if(!RuntimeControl_EncoderAlignmentActive(true)){
            auto_detect.left_sync_ok=RuntimeControl_EncoderElectricalReady(true);
            if(auto_detect.right_hall_ok && auto_detect.left_encoder_ok && auto_detect.left_sync_ok){
                /* Terminal success means BOTH runtime states and EEPROM persistence
                 * succeeded. A partial board is intentionally reported as failure. */
                auto_detect.result=RuntimeSettings_Save()?2:-1;
            } else {
                auto_detect.result=-1;
            }
            auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;''',
'''    case AUTO_DETECT_LEFT_ENCODER:
        if(RuntimeControl_VescPollSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,&res)){
            auto_detect.left_encoder_ok=(res.state==ESC_SENSOR_CAL_SUCCESS);
            auto_detect.stage=auto_detect.left_encoder_ok ? AUTO_DETECT_LEFT_SYNC : AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_SYNC:
        /* LEFT detect launches phase-0 sync + full steering homing asynchronously.
         * Wait until both state machines release ownership. */
        if(!RuntimeControl_EncoderAlignmentActive(true) && RuntimeControl_HomingActiveMask()==0U){
            auto_detect.left_sync_ok=RuntimeControl_EncoderElectricalReady(true) &&
                                     RuntimeControl_SteeringReady(true);
            if(auto_detect.right_hall_ok && auto_detect.left_encoder_ok && auto_detect.left_sync_ok){
                auto_detect.result=RuntimeSettings_Save()?2:-1;
            } else {
                auto_detect.result=-1;
            }
            auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;''',
'integrated detect wait steering ready')

(R/'tests/test_v21_left_startup_homing_205320.py').write_text('''from pathlib import Path
R=Path(__file__).resolve().parents[1]
def s(p): return (R/p).read_text()
def test_boot_sync_is_static_one_amp_no_direction_relearn():
 r=s("Src/runtime_control.c")
 a=r.index("static int16_t encoder_alignment_target_current")
 b=r.index("static void Homing_SetDefaults",a)
 block=r[a:b]
 assert "1.00 A D-axis phase-0 lock" in block
 assert "+120" not in block and "probe_delta_counts" not in block
 assert "MotorSensor_SyncEncoderElectricalPhase(state, 0U)" in block
 assert "RuntimeSettings_Save" not in block
def test_detect_auto_launches_full_homing_and_boot_rehomes_after_cal():
 r=s("Src/runtime_control.c")
 assert "RuntimeControl_StartHomingCalibration(true)" in r
 assert "if (left) homingOnBootLeft = true;" in r
 assert "RuntimeControl_SteeringReady" in r
def test_persistent_encoder_offset_is_zero():
 c=s("Src/vesc_config_compat.c")
 assert "r->encoder_offset_deg = 0U;" in c
 r=s("Src/runtime_control.c")
 assert "snap->result.encoder_offset_deg = 0U;" in r
def test_integrated_detect_waits_mechanical_ready():
 p=s("Src/vesc_protocol.c")
 assert "RuntimeControl_SteeringReady(true)" in p
''')
print('left startup/homing patch staged')
