#!/usr/bin/env python3
from pathlib import Path
R=Path(__file__).resolve().parents[1]

def rep(path, old, new, label):
    p=R/path; s=p.read_text()
    if s.count(old)!=1: raise SystemExit(f'{label}: {s.count(old)} anchors')
    p.write_text(s.replace(old,new,1))

# Position PID: mc_configuration is VESC-wire owner. Mirror it into the legacy
# PositionPidConfig immediately before EEPROM packing so GET->SET->reboot->GET
# cannot revert to stale tuning values.
rep('Src/runtime_control.c',
'''    store_foc_words(w, EEPROM_LEFT_FOC_BASE, &motorConfLeft);\n    store_foc_words(w, EEPROM_RIGHT_FOC_BASE, &motorConfRight);''',
'''    positionPidConfigLeft.kp_q16 = motorConfLeft.p_pid_kp_q16;\n    positionPidConfigLeft.ki_q16 = motorConfLeft.p_pid_ki_q16;\n    positionPidConfigLeft.kd_q16 = motorConfLeft.p_pid_kd_q16;\n    positionPidConfigRight.kp_q16 = motorConfRight.p_pid_kp_q16;\n    positionPidConfigRight.ki_q16 = motorConfRight.p_pid_ki_q16;\n    positionPidConfigRight.kd_q16 = motorConfRight.p_pid_kd_q16;\n    store_foc_words(w, EEPROM_LEFT_FOC_BASE, &motorConfLeft);\n    store_foc_words(w, EEPROM_RIGHT_FOC_BASE, &motorConfRight);''',
'position pid persistence')

# MCCONF RAM rollback on failed persistence.
p=R/'Src/vesc_config_compat.c'; s=p.read_text()
a='''    mc_configuration *c = right ? &motorConfRight : &motorConfLeft;\n    MotorRuntimeConfig *r = right ? &motorConfigRight : &motorConfigLeft;'''
b='''    mc_configuration *c = right ? &motorConfRight : &motorConfLeft;\n    MotorRuntimeConfig *r = right ? &motorConfigRight : &motorConfigLeft;\n    const mc_configuration old_c = *c;\n    const MotorRuntimeConfig old_r = *r;\n    PositionPidConfig *pos = right ? &positionPidConfigRight : &positionPidConfigLeft;\n    const PositionPidConfig old_pos = *pos;'''
if s.count(a)!=1: raise SystemExit('mc snapshot anchor')
s=s.replace(a,b,1)
a='''    mc_foc_conf_prepare(c);\n    MotorSensor_PrepareRuntime(r, right ? &motorSensorStateRight : &motorSensorStateLeft, c->foc_motor_pole_pairs);\n    return !store || RuntimeSettings_Save();'''
b='''    mc_foc_conf_prepare(c);\n    pos->kp_q16 = c->p_pid_kp_q16;\n    pos->ki_q16 = c->p_pid_ki_q16;\n    pos->kd_q16 = c->p_pid_kd_q16;\n    MotorSensor_PrepareRuntime(r, right ? &motorSensorStateRight : &motorSensorStateLeft, c->foc_motor_pole_pairs);\n    if (!store) return true;\n    if (RuntimeSettings_Save()) return true;\n    *c = old_c;\n    *r = old_r;\n    *pos = old_pos;\n    mc_foc_conf_prepare(c);\n    MotorSensor_PrepareRuntime(r, right ? &motorSensorStateRight : &motorSensorStateLeft, c->foc_motor_pole_pairs);\n    return false;'''
if s.count(a)!=1: raise SystemExit('mc save anchor')
s=s.replace(a,b,1); p.write_text(s)

# Keep transport deterministic. A failed store is rolled back above; GET then
# exposes unchanged active values instead of leaving VESC Tool waiting forever.
rep('Src/vesc_protocol.c',
''' case C_SET_MCCONF:{if(VescConfig_DeserializeMc(d,l,r,true)){uint8_t o[1]={C_SET_MCCONF};send_payload(o,1);}}break;''',
''' case C_SET_MCCONF:{(void)VescConfig_DeserializeMc(d,l,r,true);uint8_t o[1]={C_SET_MCCONF};send_payload(o,1);}break;''',
'MCCONF ack')

# Rotor Position tester crash: result dict was never initialized.
p=R/'tools/vesc_full_test.py'; s=p.read_text(); start=s.find('    def rotor_position_stream_test(self)')
if start<0: raise SystemExit('rotor test missing')
case=s.find('        cases = [',start)
if case<0: raise SystemExit('rotor cases missing')
if 'out: dict[str, Any] = {}' not in s[start:case]:
    s=s[:case]+'        out: dict[str, Any] = {}\n'+s[case:]
p.write_text(s)

# Standalone MCCONF unit links vesc_config_compat without runtime_control.c.
rep('tests/test_vesc_config_wire.c',
'''#include "motor_current_cal.h"\n''',
'''#include "motor_current_cal.h"\n#include "runtime_control.h"\n''',
'host fixture runtime type include')
rep('tests/test_vesc_config_wire.c',
'''mc_configuration motorConfLeft, motorConfRight;\nMotorRuntimeConfig motorConfigLeft, motorConfigRight;''',
'''mc_configuration motorConfLeft, motorConfRight;\nPositionPidConfig positionPidConfigLeft, positionPidConfigRight;\nMotorRuntimeConfig motorConfigLeft, motorConfigRight;''',
'host fixture position mirrors')

(R/'tests/test_v21_config_stream_205320.py').write_text('''from pathlib import Path\nR=Path(__file__).resolve().parents[1]\ndef s(p): return (R/p).read_text()\ndef test_pid_persistence_owner():\n r=s("Src/runtime_control.c"); assert "positionPidConfigLeft.kp_q16 = motorConfLeft.p_pid_kp_q16" in r\n v=s("Src/vesc_config_compat.c"); assert "const mc_configuration old_c = *c;" in v and "*c = old_c;" in v\ndef test_rotor_tester_result_dict():\n t=s("tools/vesc_full_test.py"); i=t.index("def rotor_position_stream_test"); j=t.index("def homing_calibration_test",i); assert "out: dict[str, Any] = {}" in t[i:j]\n''')
print('config/stream patch staged')
