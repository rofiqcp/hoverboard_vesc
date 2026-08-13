#!/usr/bin/env python3
"""Static contract: every VESC facade command has an intentional tester path.

This is not a substitute for hardware testing. It prevents firmware/tester drift where
we add a command to vesc_protocol.c but forget to observe it in the one-shot bench test.
"""
from pathlib import Path
import re

root = Path(__file__).resolve().parents[1]
c = (root / "Src/vesc_protocol.c").read_text(encoding="utf-8", errors="replace")
py = (root / "tools/vesc_full_test.py").read_text(encoding="utf-8", errors="replace")

cases = sorted(set(re.findall(r"case\s+(C_[A-Z0-9_]+)\s*:", c)))
assert cases, "no VESC command cases found"

# C_ enum names intentionally mirror VESC COMM_ names.
missing = []
for c_name in cases:
    p_name = "COMM_" + c_name[2:]
    if p_name not in py:
        missing.append((c_name, p_name))
assert not missing, f"tester does not reference firmware commands: {missing}"

# Read-side matrix. CUSTOM_APP_DATA is HBTS, ALIVE/terminal are protocol-control
# coverage although not GET commands in the VESC enum.
for name in (
    "COMM_FW_VERSION", "COMM_GET_VALUES", "COMM_GET_VALUES_SELECTIVE",
    "COMM_GET_VALUES_SETUP", "COMM_GET_VALUES_SETUP_SELECTIVE",
    "COMM_GET_MCCONF", "COMM_GET_MCCONF_DEFAULT",
    "COMM_GET_APPCONF", "COMM_GET_APPCONF_DEFAULT",
    "COMM_GET_DECODED_ADC", "COMM_ROTOR_POSITION", "COMM_PING_CAN",
    "COMM_CUSTOM_APP_DATA", "COMM_ALIVE", "COMM_TERMINAL_CMD",
):
    assert name in py, f"missing protocol matrix coverage for {name}"

# Every safe zero-valued actuator command implemented by the facade must be sent
# on BOTH local and virtual-right contexts before any sensor commissioning motion.
for token in (
    "SET_DUTY_0", "SET_CURRENT_0", "SET_BRAKE_0", "SET_HANDBRAKE_0",
    "SET_RPM_0", "SET_CURRENT_REL_0",
):
    assert token in py, f"missing zero-routing test {token}"

# Non-zero motion matrix must cover all meaningful controller modes once sensor
# proof succeeds. MCCONF/APPCONF setters are covered by config round-trip tests.
for token in (
    '"duty"', '"current"', '"brake"', '"handbrake"', '"current_rel"', '"rpm"', '"pos"'
):
    assert token in py, f"missing non-zero motion path {token}"

# Both commissioning modes are mandatory: LEFT encoder and RIGHT Hall.
assert "COMM_DETECT_ENCODER" in py and "COMM_DETECT_HALL_FOC" in py
assert 'detect_node("local")' in py and 'detect_node("right")' in py

print(f"TESTER_COMMAND_COVERAGE_PASS firmware_cases={len(cases)}")
