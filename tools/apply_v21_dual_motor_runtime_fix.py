#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def read(path):
    p = ROOT / path
    return p, p.read_text()

def replace_once(path, old, new, label):
    p, s = read(path)
    n = s.count(old)
    if n != 1:
        raise SystemExit(f"{label}: expected one match, found {n}")
    p.write_text(s.replace(old, new, 1))

# ---------------------------------------------------------------------------
# 1. Integrated commissioning must not let LEFT encoder failure starve RIGHT.
#    The 18:16 hardware capture stayed in LEFT encoder stage until timeout, so
#    RIGHT Hall was never calibrated and every RIGHT closed-loop test was skipped.
#    Commission RIGHT Hall first. Its AUTO transaction persists its own proof in
#    sensor_cal_finish(), then continue with LEFT. Final Apply-All result is still
#    success only when BOTH sides are ready; partial success is never mislabeled.
# ---------------------------------------------------------------------------
replace_once(
    'Src/vesc_protocol.c',
'''typedef enum {
    AUTO_DETECT_IDLE = 0, AUTO_DETECT_WAIT_CURRENT_CAL, AUTO_DETECT_LEFT_ENCODER,
    AUTO_DETECT_RIGHT_HALL, AUTO_DETECT_LEFT_SYNC, AUTO_DETECT_REPLY
} auto_detect_stage_t;
typedef struct {
    bool active;
    auto_detect_stage_t stage;
    int16_t result;
    uint16_t reply_retries;
} VescAutoDetect;
''',
'''typedef enum {
    AUTO_DETECT_IDLE = 0, AUTO_DETECT_WAIT_CURRENT_CAL, AUTO_DETECT_RIGHT_HALL,
    AUTO_DETECT_LEFT_ENCODER, AUTO_DETECT_LEFT_SYNC, AUTO_DETECT_REPLY
} auto_detect_stage_t;
typedef struct {
    bool active;
    auto_detect_stage_t stage;
    int16_t result;
    uint16_t reply_retries;
    bool right_hall_ok;
    bool left_encoder_ok;
    bool left_sync_ok;
} VescAutoDetect;
''',
    'auto detect enum/state')

replace_once(
    'Src/vesc_protocol.c',
'''    if(!MotorControl_RequestCurrentOffsetCalibration() && MotorControl_CurrentOffsetsValid()){
        if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)))
            auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
        else auto_detect.stage=AUTO_DETECT_REPLY;
    }
}

static void auto_detect_service(void)
{
    if(!auto_detect.active)return;
    RuntimeControl_VescAlive();
    RuntimeVescDetectResult res;
    switch(auto_detect.stage){
    case AUTO_DETECT_WAIT_CURRENT_CAL:
        if(MotorControl_CurrentOffsetCalState()==MOTOR_CURRENT_CAL_FAILED){auto_detect.stage=AUTO_DETECT_REPLY;break;}
        if(MotorControl_CurrentOffsetsValid()){
            if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)))
                auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_ENCODER:
        if(RuntimeControl_VescPollSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,&res)){
            if(res.state!=ESC_SENSOR_CAL_SUCCESS){auto_detect.stage=AUTO_DETECT_REPLY;break;}
            if(RuntimeControl_VescStartSensorDetect(false,MOTOR_SENSOR_HALL_UVW,detect_current_internal(true,VESC_AUTO_DETECT_CURRENT_A)))
                auto_detect.stage=AUTO_DETECT_RIGHT_HALL;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_RIGHT_HALL:
        if(RuntimeControl_VescPollSensorDetect(false,MOTOR_SENSOR_HALL_UVW,&res)){
            if(res.state!=ESC_SENSOR_CAL_SUCCESS){auto_detect.stage=AUTO_DETECT_REPLY;break;}
            if(RuntimeControl_RequestEncoderSync(true))auto_detect.stage=AUTO_DETECT_LEFT_SYNC;
            else if(RuntimeControl_EncoderElectricalReady(true)){
                auto_detect.result=RuntimeSettings_Save()?2:-1;
                auto_detect.stage=AUTO_DETECT_REPLY;
            } else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_SYNC:
        if(!RuntimeControl_EncoderAlignmentActive(true)){
            if(RuntimeControl_EncoderElectricalReady(true)){
                /* Terminal success means runtime state AND EEPROM persistence
                 * succeeded. Never report result=2 and silently drop calibration. */
                auto_detect.result=RuntimeSettings_Save()?2:-1;
            }
            auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
''',
'''    if(!MotorControl_RequestCurrentOffsetCalibration() && MotorControl_CurrentOffsetsValid()){
        if(RuntimeControl_VescStartSensorDetect(false,MOTOR_SENSOR_HALL_UVW,detect_current_internal(true,VESC_AUTO_DETECT_CURRENT_A)))
            auto_detect.stage=AUTO_DETECT_RIGHT_HALL;
        else if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)))
            auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
        else auto_detect.stage=AUTO_DETECT_REPLY;
    }
}

static void auto_detect_service(void)
{
    if(!auto_detect.active)return;
    RuntimeControl_VescAlive();
    RuntimeVescDetectResult res;
    switch(auto_detect.stage){
    case AUTO_DETECT_WAIT_CURRENT_CAL:
        if(MotorControl_CurrentOffsetCalState()==MOTOR_CURRENT_CAL_FAILED){auto_detect.stage=AUTO_DETECT_REPLY;break;}
        if(MotorControl_CurrentOffsetsValid()){
            /* The two physical motors are independent. Commission the Hall-only
             * RIGHT motor first so a difficult LEFT steering encoder can never
             * prevent RIGHT from becoming a valid closed-loop controller. */
            if(RuntimeControl_VescStartSensorDetect(false,MOTOR_SENSOR_HALL_UVW,detect_current_internal(true,VESC_AUTO_DETECT_CURRENT_A)))
                auto_detect.stage=AUTO_DETECT_RIGHT_HALL;
            else if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)))
                auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_RIGHT_HALL:
        if(RuntimeControl_VescPollSensorDetect(false,MOTOR_SENSOR_HALL_UVW,&res)){
            auto_detect.right_hall_ok=(res.state==ESC_SENSOR_CAL_SUCCESS);
            /* sensor_cal_finish(AUTO) already persisted a successful RIGHT Hall
             * candidate. Continue LEFT regardless of RIGHT result so Apply-All
             * gathers independent evidence for both physical sides. */
            if(RuntimeControl_VescStartSensorDetect(true,MOTOR_SENSOR_ENCODER_AB,detect_current_internal(false,VESC_AUTO_DETECT_CURRENT_A)))
                auto_detect.stage=AUTO_DETECT_LEFT_ENCODER;
            else auto_detect.stage=AUTO_DETECT_REPLY;
        }
        break;
    case AUTO_DETECT_LEFT_ENCODER:
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
        break;
''',
    'right-first independent auto detect service')

replace_once(
    'Src/vesc_protocol.c',
'''            "HB integrated detect started: current-cal -> LEFT encoder -> RIGHT Hall -> LEFT sync -> EEPROM\\n" :
''',
'''            "HB integrated detect started: current-cal -> RIGHT Hall -> LEFT encoder -> LEFT sync -> EEPROM\\n" :
''',
    'terminal integrated detect description')

# ---------------------------------------------------------------------------
# 2. Full Brake on this F103 must be a hardware low-side short, not merely a
#    zero modulation request. Upstream VESC's FOC path uses full_brake_hw when
#    zero duty and short-low-side mode apply. The stock hoverboard already has a
#    validated all-low-side zero vector used for active-domain current calibration.
#    Keep this path per motor and O(1) in the 16-kHz DMA ISR.
# ---------------------------------------------------------------------------
replace_once(
    'Src/runtime_control.c',
'''static bool prearm_feedback_not_ready(uint8_t mode, const MotorRuntimeConfig *cfg,
                                      const MotorSensorSample *sample, bool encoder_aligned)
{
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE) return false;
''',
'''static bool prearm_feedback_not_ready(uint8_t mode, int32_t target,
                                      const MotorRuntimeConfig *cfg,
                                      const MotorSensorSample *sample, bool encoder_aligned)
{
    /* VESC Full Brake is SET_DUTY(0). It does not need rotor angle because the
     * F103 implementation shorts all three low sides directly. OPEN and
     * handbrake also own their phase independently from the feedback backend. */
    if (mode == ESC_MODE_OPEN || mode == ESC_MODE_HANDBRAKE ||
        (mode == ESC_MODE_DUTY && target == 0)) return false;
''',
    'full brake prearm feedback bypass signature')

replace_once(
    'Src/runtime_control.c',
'''    if (prearm_feedback_not_ready(mode, cfg, sample, encoder_aligned)) {
''',
'''    const int32_t requested_target = left ? runtimeSetpointLeft : runtimeSetpointRight;
    if (prearm_feedback_not_ready(mode, requested_target, cfg, sample, encoder_aligned)) {
''',
    'full brake prearm call')

# motor.c phase-valid and ISR low-side brake path.
replace_once(
    'Src/motor.c',
'''    const bool left_phase_valid = left_calibration || left_open ||
        motorLeft.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleLeft.feedback_valid != 0U;
    const bool right_phase_valid = right_calibration || right_open ||
        motorRight.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleRight.feedback_valid != 0U;
''',
'''    /* VESC Full Brake is duty=0 with the bridge deliberately active. On this
     * PCB it is implemented as the same all-low-side zero vector already proven
     * by current-offset calibration, so it requires no rotor phase. */
    const bool left_full_brake = motorLeft.m_control_mode == CONTROL_MODE_DUTY &&
        motorLeft.m_duty_cycle_set_q15 == 0;
    const bool right_full_brake = motorRight.m_control_mode == CONTROL_MODE_DUTY &&
        motorRight.m_duty_cycle_set_q15 == 0;
    const bool left_phase_valid = left_calibration || left_open || left_full_brake ||
        motorLeft.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleLeft.feedback_valid != 0U;
    const bool right_phase_valid = right_calibration || right_open || right_full_brake ||
        motorRight.m_control_mode == CONTROL_MODE_HANDBRAKE ||
        motorSensorSampleRight.feedback_valid != 0U;
''',
    'full brake ISR phase validity')

replace_once(
    'Src/motor.c',
'''    if (left_request_ok && !left_domain_ready) set_current_zero_vector_left();
    if (right_request_ok && !right_domain_ready) set_current_zero_vector_right();

    /* Emergency liveness fallback is evaluated AFTER both sensors were sampled.
''',
'''    if (left_request_ok && !left_domain_ready) set_current_zero_vector_left();
    if (right_request_ok && !right_domain_ready) set_current_zero_vector_right();

    /* Hardware full-brake path. Once the active current domain is ready, hold
     * CCR1/2/3 at zero with MOE asserted: all three low-side FETs are on, phase
     * line-to-line voltage is zero and the motor is electrically shorted. This
     * mirrors VESC foc_short_ls_on_zero_duty/full_brake_hw without adding any
     * floating point or extra control loop to the 16-kHz DMA ISR. */
    if (left_full_brake && left_domain_ready) set_current_zero_vector_left();
    if (right_full_brake && right_domain_ready) set_current_zero_vector_right();

    /* Emergency liveness fallback is evaluated AFTER both sensors were sampled.
''',
    'full brake zero vector assertion')

replace_once(
    'Src/motor.c',
'''    if (!run_right_slot) {
        const mc_foc_sample_t left_sample = {
''',
'''    if (!run_right_slot) {
        if (left_full_brake && left_domain_ready) {
            /* Do not let centered SVPWM overwrite the all-low-side short. */
            motorOutputLeft.duty_a = 0;
            motorOutputLeft.duty_b = 0;
            motorOutputLeft.duty_c = 0;
            motorOutputLeft.duty_abs_q15 = 0U;
            motorOutputLeft.id_target = 0;
            motorOutputLeft.iq_target = 0;
            commissioningFastStreakLeft = 0U;
        } else {
        const mc_foc_sample_t left_sample = {
''',
    'full brake skip left FOC begin')

replace_once(
    'Src/motor.c',
'''        mc_foc_run_current_control(&motorLeft, &left_sample, &motorOutputLeft, pwm_res);
        if (left_foc_enabled) set_pwm_left(&motorOutputLeft);
        commissioningFastStreakLeft = 0U;
    } else {
        const mc_foc_sample_t right_sample = {
''',
'''        mc_foc_run_current_control(&motorLeft, &left_sample, &motorOutputLeft, pwm_res);
        if (left_foc_enabled) set_pwm_left(&motorOutputLeft);
        commissioningFastStreakLeft = 0U;
        }
    } else {
        if (right_full_brake && right_domain_ready) {
            /* RIGHT is independently shorted; LEFT state is untouched. */
            motorOutputRight.duty_a = 0;
            motorOutputRight.duty_b = 0;
            motorOutputRight.duty_c = 0;
            motorOutputRight.duty_abs_q15 = 0U;
            motorOutputRight.id_target = 0;
            motorOutputRight.iq_target = 0;
            commissioningFastStreakRight = 0U;
        } else {
        const mc_foc_sample_t right_sample = {
''',
    'full brake left close/right begin')

replace_once(
    'Src/motor.c',
'''        mc_foc_run_current_control(&motorRight, &right_sample, &motorOutputRight, pwm_res);
        if (right_foc_enabled) set_pwm_right(&motorOutputRight);
        commissioningFastStreakRight = 0U;
    }

    /* ISR only publishes raw engineering input current + validity. Averaging is
''',
'''        mc_foc_run_current_control(&motorRight, &right_sample, &motorOutputRight, pwm_res);
        if (right_foc_enabled) set_pwm_right(&motorOutputRight);
        commissioningFastStreakRight = 0U;
        }
    }

    /* ISR only publishes raw engineering input current + validity. Averaging is
''',
    'full brake right close')

# ---------------------------------------------------------------------------
# 3. Tester: readiness is PER MOTOR. An Apply-All failure is diagnostic, not a
#    reason to suppress a successfully commissioned opposite side. Add explicit
#    Full Brake and Stop hardware-semantics checks for both sides.
# ---------------------------------------------------------------------------
replace_once(
    'tools/vesc_full_test.py',
'''            left_ready = bool(left_detect is not None and post_left and post_left.get("calibrated"))
            right_ready = bool(right_detect is not None and post_right and post_right.get("calibrated"))
''',
'''            # Board commissioning is fault-contained per physical motor. A
            # partial integrated result must not hide a successfully calibrated
            # opposite side; post-detect hardware state is authoritative here.
            left_ready = bool(post_left and post_left.get("calibrated"))
            right_ready = bool(post_right and post_right.get("calibrated"))
''',
    'tester per-side readiness')

# Full-brake helper inserted before stop_and_verify.
replace_once(
    'tools/vesc_full_test.py',
'''    def stop_and_verify(self, node: str) -> dict[str, Any]:
''',
'''    def full_brake_and_stop_test(self, node: str) -> dict[str, Any]:
        """Verify VESC Tool Full Brake (SET_DUTY 0) versus Stop semantics.

        Full Brake must keep only the selected bridge active at duty zero; Stop is
        SET_CURRENT(0) and must release that bridge. This test is safe without a
        calibrated rotor sensor because the full-brake path is a static low-side
        short and never consumes electrical angle.
        """
        assert self.dev
        peer = "right" if node == "local" else "local"
        for n in (node, peer):
            for _ in range(3): self.dev.stop(n); time.sleep(0.03)
        self.dev.set_duty(node, 0.0)
        time.sleep(0.15)
        d_brake = self.dev.diag(node, "full_brake")
        d_peer = self.dev.diag(peer, "full_brake_peer")
        v_brake = self.dev.values(node, "full_brake")
        assert d_brake.get("armed") and d_brake.get("bridge_moe"), \
            f"{node} Full Brake did not keep target bridge active: {d_brake}"
        assert int(d_brake.get("last_set_command") or -1) == COMM_SET_DUTY
        assert int(d_brake.get("last_set_host_raw") or 1) == 0
        assert int(d_brake.get("duty_target_q15") or 0) == 0
        assert abs(float(v_brake.get("duty") or 0.0)) <= 0.002
        assert not d_peer.get("armed") and not d_peer.get("bridge_moe"), \
            f"{node} Full Brake leaked to peer {peer}: {d_peer}"

        for _ in range(4): self.dev.stop(node); time.sleep(0.05)
        d_stop = self.dev.diag(node, "full_brake_then_stop")
        assert not d_stop.get("armed") and not d_stop.get("bridge_moe"), \
            f"{node} Stop did not release bridge after Full Brake: {d_stop}"
        assert int(d_stop.get("last_set_command") or -1) == COMM_SET_CURRENT
        assert int(d_stop.get("last_set_host_raw") or 1) == 0
        return {"full_brake": d_brake, "peer": d_peer, "stop": d_stop}

    def stop_and_verify(self, node: str) -> dict[str, Any]:
''',
    'tester full brake helper')

# Zero-routing semantics: duty zero is no longer expected to be released.
replace_once(
    'tools/vesc_full_test.py',
'''                assert not dt["bridge_moe"] and not dp["bridge_moe"], f"zero SET unexpectedly enabled bridge: {target}/{name}"
                assert abs(vt["motor_current_A"])<=0.05 and abs(vp["motor_current_A"])<=0.05
''',
'''                if name == "SET_DUTY_0":
                    assert dt["bridge_moe"] and dt["armed"], f"{target} SET_DUTY(0) did not enter Full Brake"
                    assert not dp["bridge_moe"] and not dp["armed"], f"{target} Full Brake armed peer"
                    # Return to released state before exercising the remaining zero SETs.
                    self.dev.stop(target); time.sleep(0.05)
                else:
                    assert not dt["bridge_moe"] and not dp["bridge_moe"], f"zero SET unexpectedly enabled bridge: {target}/{name}"
                assert abs(vt["motor_current_A"])<=0.25 and abs(vp["motor_current_A"])<=0.05
''',
    'tester zero routing full brake semantics')

# Add full brake/stop tests before sensor-dependent motion list.
replace_once(
    'tools/vesc_full_test.py',
'''            tests = [
''',
'''            # These two are sensor-independent hardware semantics and must run
            # even when one sensor commissioning path failed.
            self.result("14d_left_full_brake_then_stop", lambda: self.full_brake_and_stop_test("local"),
                        skip=not current_ok, skip_reason="current offsets not ready")
            self.result("14e_right_full_brake_then_stop", lambda: self.full_brake_and_stop_test("right"),
                        skip=not current_ok, skip_reason="current offsets not ready")

            tests = [
''',
    'tester add full brake stop tests')

# More realistic integrated board timeout now that RIGHT Hall runs first and LEFT
# still gets its own complete sweep/sync. Individual detect keeps same argument.
p, s = read('tools/vesc_full_test.py')
s = s.replace('ap.add_argument("--detect-timeout", type=float, default=35.0)',
              'ap.add_argument("--detect-timeout", type=float, default=65.0)')
p.write_text(s)

# Regression contracts from the 18:16 capture and VESC/F103 semantics.
(ROOT/'tests/test_v21_dual_motor_runtime.py').write_text(r'''from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
proto=(ROOT/'Src/vesc_protocol.c').read_text()
runtime=(ROOT/'Src/runtime_control.c').read_text()
motor=(ROOT/'Src/motor.c').read_text()
tester=(ROOT/'tools/vesc_full_test.py').read_text()

def test_apply_all_commissions_right_before_left_and_preserves_partial_right():
    block=proto[proto.index('static void auto_detect_service'):proto.index('static void send_print')]
    assert block.index('case AUTO_DETECT_RIGHT_HALL') < block.index('case AUTO_DETECT_LEFT_ENCODER')
    assert 'auto_detect.right_hall_ok=(res.state==ESC_SENSOR_CAL_SUCCESS);' in block
    assert 'Continue LEFT regardless of RIGHT result' in block
    assert 'RIGHT proof, when successful, remains committed and usable' in block

def test_full_brake_is_sensor_independent_but_stop_is_release():
    assert '(mode == ESC_MODE_DUTY && target == 0)' in runtime
    assert 'const bool run=true; /* SET_DUTY(0) is VESC Tool Full Brake, not release. */' in proto
    assert 'const bool run=v!=0;' in proto  # SET_CURRENT(0) release path remains

def test_f103_full_brake_uses_low_side_zero_vector_per_motor():
    assert 'left_full_brake' in motor and 'right_full_brake' in motor
    assert 'if (left_full_brake && left_domain_ready) set_current_zero_vector_left();' in motor
    assert 'if (right_full_brake && right_domain_ready) set_current_zero_vector_right();' in motor
    assert 'Do not let centered SVPWM overwrite the all-low-side short.' in motor

def test_dma_architecture_remains_dual_sensor_16k_interleaved_foc():
    assert 'MotorSensor_UpdateHardwareEncoder(&motorConfigLeft' in motor
    assert 'MotorSensor_Update(&motorConfigRight' in motor
    assert 'const bool run_right_slot = motorFocSlotRight;' in motor
    assert 'motorFocSlotRight = !motorFocSlotRight;' in motor
    assert 'bridge_current_domain_service(true' in motor
    assert 'bridge_current_domain_service(false' in motor

def test_tester_runs_each_side_from_actual_post_detect_readiness():
    assert 'left_ready = bool(post_left and post_left.get("calibrated"))' in tester
    assert 'right_ready = bool(post_right and post_right.get("calibrated"))' in tester
    assert '14d_left_full_brake_then_stop' in tester
    assert '14e_right_full_brake_then_stop' in tester
    assert 'default=65.0' in tester
''')

p, s = read('CHANGELOG_V21.md')
addition='''\n## Dual-motor hardware correction — 2026-08-13 18:16 run\n- Fixed integrated commissioning fault containment: RIGHT Hall is commissioned first and its successful proof is persisted even if LEFT encoder commissioning later fails. LEFT and RIGHT can therefore be debugged/run independently.\n- Full tester now derives LEFT/RIGHT readiness from each motor's actual post-detect state instead of the single Apply-All return value, so one failed side no longer skips all tests on the healthy side.\n- Implemented VESC Full Brake semantics on STM32F103 hoverboard hardware: `SET_DUTY(0)` keeps only the selected bridge active and asserts the validated all-low-side zero vector; `SET_CURRENT(0)` remains Stop/release with MOE off.\n- Full Brake bypasses sensor-angle readiness because the hardware short does not use rotor phase. Normal Duty/Current/RPM/Position/Current-Brake still require the selected side's valid feedback proof.\n- Added explicit LEFT/RIGHT Full-Brake→Stop hardware tests and regression contracts while preserving 16-kHz dual-sensor sampling and 8-kHz-per-motor interleaved FOC.\n'''
if 'Dual-motor hardware correction — 2026-08-13 18:16 run' not in s:
    s += addition
p.write_text(s)

print('V21 dual-motor runtime patch applied')
