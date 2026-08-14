#!/usr/bin/env python3
"""Regression for tester inventory bootstrap.

V8 introduced strict HBTS node validation, but connection_inventory called HBTS
before local_id existed. This test fails if any diagnostic request happens before
standard VESC FW/GET_VALUES/PING_CAN has established both node IDs.
"""
from pathlib import Path
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[1]
MOD_PATH = ROOT / "tools" / "vesc_full_test.py"
spec = importlib.util.spec_from_file_location("vesc_full_test_under_test", MOD_PATH)
mod = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = mod
assert spec.loader is not None
spec.loader.exec_module(mod)


class FakeDevice:
    def __init__(self):
        self.local_id = None
        self.right_id = None
        self.calls = []

    def fw(self, node):
        self.calls.append(("fw", node))
        return {
            "version": "6.00",
            "uuid": "00112233445566778899aabb" if node == "local" else "00112233445566778899aabc",
        }

    def values(self, node, label="sample"):
        self.calls.append(("values", node, label))
        assert node == "local"
        return {"controller_id": 10}

    def ping_can(self):
        self.calls.append(("ping_can",))
        return [11]

    def diag(self, node, label="diag"):
        self.calls.append(("diag", node, label))
        # This is the key V8 regression: diag must not happen until IDs exist.
        assert self.local_id == 10, f"diag called before local_id bootstrap: {self.local_id}"
        assert self.right_id == 11, f"diag called before right_id bootstrap: {self.right_id}"
        return {"node_id": 10 if node == "local" else 11}


# Verify the actual non-selective COMM_GET_VALUES decoder reaches the standard
# controller-ID byte at the expected VESC 6.00 offset used by V9 bootstrap.
import struct
payload = bytearray((mod.COMM_GET_VALUES,))
payload += struct.pack(">hh", 250, 0)
for v in (0, 0, 0, 0):
    payload += struct.pack(">i", v)
payload += struct.pack(">h", 0)          # duty
payload += struct.pack(">i", 0)          # erpm
payload += struct.pack(">h", 480)        # 48.0 V
for v in (0, 0, 0, 0):
    payload += struct.pack(">i", v)       # Ah/Wh fields
payload += struct.pack(">ii", 0, 0)      # tach/tach_abs
payload += bytes((0,))                     # fault
payload += struct.pack(">i", 0)          # PID position
payload += bytes((10,))                    # controller ID
payload += struct.pack(">hhh", 250, 250, 250)
payload += struct.pack(">ii", 0, 0)      # Vd/Vq
payload += bytes((0,))                     # status
decoded = mod.decode_values(bytes(payload))
assert decoded["controller_id"] == 10, decoded

suite = object.__new__(mod.TestSuite)
suite.dev = FakeDevice()
result = mod.TestSuite.connection_inventory(suite)
assert result["local_id"] == 10
assert result["right_id"] == 11
assert result["local_id_source"] == "COMM_GET_VALUES"
assert suite.dev.calls[:3] == [
    ("fw", "local"),
    ("values", "local", "inventory_values"),
    ("ping_can",),
], suite.dev.calls
assert suite.dev.calls[3] == ("fw", "right"), suite.dev.calls
assert suite.dev.calls[4][0:2] == ("diag", "local"), suite.dev.calls
assert suite.dev.calls[5][0:2] == ("diag", "right"), suite.dev.calls

# Pure resolver fallback checks.
assert mod.resolve_inventory_ids(10, [11]) == (10, 11, "COMM_GET_VALUES")
assert mod.resolve_inventory_ids(None, [11]) == (10, 11, "derived_from_single_virtual_CAN")
assert mod.resolve_inventory_ids(254, [0]) == (254, 0, "COMM_GET_VALUES")

print("TESTER_INVENTORY_BOOTSTRAP_PASS")
