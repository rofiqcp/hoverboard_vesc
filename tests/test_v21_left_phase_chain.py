from pathlib import Path
R=Path(__file__).resolve().parents[1]
def txt(p): return (R/p).read_text()

def test_tim4_accumulator_has_single_owner():
    h=txt('Src/left_encoder.h'); c=txt('Src/left_encoder.c'); m=txt('Src/motor.c'); r=txt('Src/runtime_control.c')
    assert 'LeftEncoder_UpdateAndGetCount' in h
    update=c[c.index('int32_t LeftEncoder_UpdateAndGetCount'):c.index('int32_t LeftEncoder_GetCount')]
    snap=c[c.index('int32_t LeftEncoder_GetCount'):c.index('void LeftEncoder_ZeroMechanical')]
    assert 'TIM4->CNT' in update and 'previous_cnt = now' in update
    assert 'const uint16_t now = (uint16_t)TIM4->CNT;' not in snap and 'previous_cnt = now' not in snap and 'return accumulated' in snap
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
