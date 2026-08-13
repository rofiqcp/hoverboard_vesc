#!/usr/bin/env python3
"""V12 regression for the hardware failure seen in the 2026-08-12 V11 log.

This is intentionally a source/numeric contract: the real motor/ADC behavior still
requires the user's board. It prevents re-introducing the exact commissioning and
telemetry mistakes that made V11 time out before non-zero SET testing.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
runtime = (root / "Src/runtime_control.c").read_text(encoding="utf-8")
foc = (root / "Src/foc_motor.c").read_text(encoding="utf-8")
vesc = (root / "Src/vesc_protocol.c").read_text(encoding="utf-8")
tester = (root / "tools/vesc_full_test.py").read_text(encoding="utf-8")

def macro_int(src: str, name: str) -> int:
    m = re.search(rf"^#define\s+{re.escape(name)}\s+([0-9]+)U?\b", src, re.M)
    assert m, f"missing numeric macro {name}"
    return int(m.group(1))

# 1) V11 hardware log had commissioning Vd pinned at 500. V12 retains the 0.5-A
# tester request but must not re-introduce that tiny voltage clamp.
vmax = macro_int(foc, "FOC_COMMISSIONING_VOLTAGE_MAX")
assert vmax == 2400, f"unexpected V12 commissioning voltage cap {vmax}"
assert "mc_foc_commissioning_voltage_limit" in foc

# 2) Hall acquisition mirrors the useful part of VESC detection: forward + reverse
# sweeps. At 3 Q4/ms, six electrical revolutions fit comfortably in the 30-s timeout.
fwd = macro_int(runtime, "SENSOR_CAL_HALL_FORWARD_SWEEPS")
rev = macro_int(runtime, "SENSOR_CAL_HALL_REVERSE_SWEEPS")
rate_q4_per_ms = macro_int(runtime, "SENSOR_CAL_PHASE_Q4_PER_MS")
timeout_ms = macro_int(runtime, "SENSOR_CAL_AUTO_TIMEOUT_MS")
align_ms = macro_int(runtime, "SENSOR_CAL_AUTO_ALIGN_MS")
assert (fwd, rev) == (3, 3)
assert rate_q4_per_ms == 3
sweep_ms = ((fwd + rev) * 5760 + rate_q4_per_ms - 1) // rate_q4_per_ms
assert align_ms + sweep_ms < timeout_ms, (align_ms, sweep_ms, timeout_ms)
assert "sensorCal.sweep_direction = -1" in runtime
assert "sensorCal.reverse_cycles" in runtime

# 3) Encoder VESC detect follows upstream-shaped local +/-120 degree probes.
probe_q4 = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_Q4")
probe_max = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_MAX_COUNT")
probe_settle = macro_int(runtime, "SENSOR_CAL_ENCODER_PROBE_SETTLE_MS")
enc_ms = probe_max * (((probe_q4 + rate_q4_per_ms - 1) // rate_q4_per_ms) + probe_settle)
assert probe_q4 == 1920 and probe_max >= 8 and align_ms + enc_ms < timeout_ms

# 4) Standard VESC telemetry validity must recognize live pending samples. V11's
# diagnostic incorrectly waited for current_avg_take() to drain them first.
assert "avg_has_live_or_held = avg->samples != 0U" in vesc
assert "bridge_moe && current_measurement_valid && avg_has_live_or_held" in vesc
# VESC motor-current magnitude includes Id+Iq; V11 incorrectly forced Imotor=0
# at Iq~=0, hiding D-axis commissioning current.
assert "(int64_t)vq * (int64_t)iq_raw" in vesc
assert "fabsf(iq) < 0.0005f" not in vesc

# 5) No internal sensor/commissioning error may masquerade as VESC DRV=3.
fault = re.search(r"static uint8_t fault_code\(bool r\)\{(.*?)\n\}", vesc, re.S)
assert fault, "fault_code() not found"
assert "return 3U" not in fault.group(1), "FAULT_CODE_DRV forged by facade"
assert "return 4U" in fault.group(1), "real ABS over-current mapping disappeared"
assert "record_values_wire" in vesc and "vesc_last_values_fault" in vesc

# 6) Hardware tester must exercise the exact VESC Tool wire path DURING detect and
# always emit the four requested SET scaling contracts even if commissioning fails.
for token in ("detect_standard_values", "COMM_GET_VALUES_SETUP", "vesc_standard_wire.jsonl",
              "set_command_contract.csv", "set_command_trace.csv"):
    assert token in tester, f"missing V12 debug proof {token}"
for kind, expr in (("duty", "100000.0"), ("current", "1000.0"),
                   ("rpm", "int(value)"), ("pos", "1000000.0")):
    assert kind in tester and expr in tester, f"missing SET scaling proof {kind}"
assert "if payload[5] >= 11 and p + 34 <= len(payload):" in tester
assert "if payload[5] >= 12 and p + 13 <= len(payload):" in tester

print(
    "V12_COMMISSIONING_CONTRACT_PASS "
    f"hall_sweep_ms={sweep_ms} encoder_sweep_ms={enc_ms} "
    f"align_ms={align_ms} timeout_ms={timeout_ms} vmax={vmax}"
)
