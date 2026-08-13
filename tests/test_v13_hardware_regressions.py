#!/usr/bin/env python3
"""V13 source contract for failures proven by hardware log 2026-08-12 23:34:10.

This test deliberately checks the exact regressions that cannot be reproduced on a
host PC: LEFT encoder pin routing, asynchronous Detect reply durability, per-motor
terminal state isolation, encoder alignment actuation, and Python tester argument/
UART-polling behavior. Physical rotation still requires the real board.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
motor = (root/'Src/motor.c').read_text()
runtime = (root/'Src/runtime_control.c').read_text()
protocol = (root/'Src/vesc_protocol.c').read_text()
tester = (root/'tools/vesc_full_test.py').read_text()

# 1) LEFT encoder physical contract: PB6=A, PB7=B. PB5 is Z and must never be
# reused as A by the hardware-encoder observer/commissioning path.
assert 'const uint8_t encoder_a = left_v; /* PB6 */' in motor
assert 'const uint8_t encoder_b = left_w; /* PB7 */' in motor
assert 'LeftEncoder_GetCount(), encoder_a, encoder_b' in motor
assert 'LeftEncoder_GetCount(), left_u, left_v' not in motor

# 2) Loaded-wheel encoder fallback: first prefer strict ratio, then allow the
# configured pole-pair value only with clean TIM4 + all-four-state evidence.
for token in ('encoder_ratio_fallback_used', 'configured_pp',
              'valid_edges >= 32U', 'invalid_edges <= invalid_limit',
              'sensorCal.encoder_session_seen_mask == 0x0FU'):
    assert token in runtime, f'missing encoder fallback guard: {token}'
assert 'VescDetectTerminalSnapshot vescDetectTerminal[2]' in runtime
assert 'RuntimeControl_VescGetLastSensorDetect' in runtime

# 3) Detect terminal reply may experience UART backpressure but must never be
# forgotten. V12 had depth 3 and cleared pending_detect unconditionally.
m = re.search(r'#define\s+TX_QUEUE_DEPTH\s+(\d+)U', protocol)
assert m and int(m.group(1)) >= 6, 'TX ring still too shallow for VESC Tool polling'
assert 'static bool send_payload_terminal' in protocol
svc = protocol[protocol.index('static void detect_service(void)'):protocol.index('static void send_print')]
assert 'if(sent)' in svc and 'pending_detect.active=false' in svc
assert 'else if(pending_detect.reply_retries!=UINT16_MAX)' in svc
assert svc.index('pending_detect.active=false') > svc.index('if(sent)')
main_svc = protocol[protocol.index('void VescProtocol_Service(void)'):]
assert main_svc.index('detect_service();') < main_svc.index('while(rx_old!=pos)')

# 4) Encoder electrical zero at first ARM uses regulated D-axis current, not
# the V1-V12 30..90 arbitrary voltage override that was below measured Vd need.
a0 = runtime.index('static void encoder_alignment_start')
a1 = runtime.index('static bool encoder_alignment_service', a0)
seg = runtime[a0:a1]
assert 'sensor_cal_set_current_override' in seg
assert 'sensor_cal_set_voltage_override' not in seg
assert 'target_current_internal' in seg

# 5) Python false-DRV regression and polling throttle. The V12 run failed from a
# positional string being interpreted as the selective-value mask.
assert 'def setup_values(self, node: str, *, mask:' in tester
assert 'setup_values(node, label="false_drv_setup")' in tester
assert 'setup_values(node, label="get_matrix_setup")' in tester
assert 'setup_values(node, mask=setup_mask, label="get_matrix_setup_selective")' in tester
assert 'setup_values(node,None' not in tester and 'setup_values(node,setup_mask' not in tester
assert 'detect_transaction_trace.csv' in tester and 'def detect_trace(' in tester
assert 'terminal_detect_valid' in tester and 'pending_detect_owner' in tester
assert 'next_diag = now_poll + 0.30' in tester
assert 'next_values = now_poll + 0.50' in tester
assert 'next_setup = now_poll + 2.00' in tester

# 6) HBTS must expose queue/terminal transaction evidence for the next board run.
assert '#define HBTS_DIAG_VERSION          15U' in protocol
for token in ('tx_queue_used()', 'pending_detect.reply_retries',
              'last_detect_valid', 'ratio_fallback'):
    assert token in protocol
for token in ('pending_detect_reply_retries', 'terminal_detect_valid',
              'terminal_encoder_ratio_fallback'):
    assert token in tester

print('V13_BASELINE_REGRESSION_CONTRACT_PASS')
