#!/usr/bin/env python3
"""V16 regression: V1 overload relief must preserve main-loop/VESC transport liveness."""
from pathlib import Path
root = Path(__file__).resolve().parents[1]
motor = (root/'Src/motor.c').read_text()
main = (root/'Src/main.c').read_text()
vesc = (root/'Src/vesc_protocol.c').read_text()
tester = (root/'tools/vesc_full_test.py').read_text()

isr = motor[motor.index('void DMA1_Channel1_IRQHandler(void)'):]
# V18 services BOTH sensors every IRQ, but only one FOC loop per deterministic slot.
assert 'MotorSensor_UpdateHardwareEncoder' in isr
assert 'MotorSensor_Update(&motorConfigRight' in isr
assert 'mc_foc_run_current_control(&motorLeft' in isr
assert 'mc_foc_run_current_control(&motorRight' in isr
assert 'const bool run_right_slot = motorFocSlotRight;' in isr
# Exact V1-style overload detector + one-shot shed are present.
assert 'static bool motorIsrShedNext = false;' in motor
assert 'if ((DMA1->ISR & DMA_ISR_TCIF1) != 0U)' in isr
assert 'motorIsrShedNext = true;' in isr
shed = isr[isr.index('if (motorIsrShedNext)'):isr.index('const bool run_right_slot')]
assert 'motorIsrShedNext = false;' in shed
assert 'return;' in shed
# Shed is CPU-liveness only, never a sticky PWM/command safety action.
for forbidden in ('runtimeMotorEnableMask &=', 'sensorCalibrationOpenLoopMask &=',
                  'armRequested = false', 'motorControlIsrOverrunFaultMask |='):
    assert forbidden not in shed, forbidden
# Hard current chop executes BEFORE the relief return.
assert isr.index('abs_s16_saturated(curL_DC) <= curDC_max') < isr.index('if (motorIsrShedNext)')
assert isr.index('abs_s16_saturated(curR_DC) <= curDC_max') < isr.index('if (motorIsrShedNext)')
assert isr.index('MotorSensor_UpdateHardwareEncoder') < isr.index('if (motorIsrShedNext)')
assert isr.index('MotorSensor_Update(&motorConfigRight') < isr.index('if (motorIsrShedNext)')
# VESC transport remains main-loop first service and must not depend on motor state.
loop = main[main.index('for (;;)'):]
assert loop.index('VescProtocol_Service();') < loop.index('RuntimeControl_UpdateSlow(dt_ms);')
# Firmware/tester identity proves the correct binary/log pair.
assert 'hoverboard-vesc6-v20' in vesc
assert 'TESTER_RELEASE = "V20"' in tester
print('V16_SERIAL_LIVENESS_CONTRACT_PASS')
