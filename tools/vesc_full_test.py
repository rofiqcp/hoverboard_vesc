#!/usr/bin/env python3
"""One-shot VESC-compatible V21 bench test for the dual hoverboard controller.

The script produces a timestamped diagnostic bundle that can be sent back for
troubleshooting. It uses only the public VESC UART protocol plus a read-only
COMM_CUSTOM_APP_DATA diagnostic extension (magic HBTS).

Read-only test:
    python tools/vesc_full_test.py --port COM5

Full commissioning + motion test (motor WILL move):
    python tools/vesc_full_test.py --port COM5 --full --yes

Requirements:
    python -m pip install pyserial
"""
from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import json
import math
import os
from pathlib import Path
import shutil
import statistics
import platform
import struct
import sys
import time
import traceback
from typing import Any, Callable, Optional

# VESC 6.00 command IDs used by this firmware.
TESTER_RELEASE = "V21"

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM = 8
COMM_SET_POS = 9
COMM_SET_HANDBRAKE = 10
COMM_SET_DETECT = 11
COMM_SET_MCCONF = 13
COMM_GET_MCCONF = 14
COMM_GET_MCCONF_DEFAULT = 15
COMM_SET_APPCONF = 16
COMM_GET_APPCONF = 17
COMM_GET_APPCONF_DEFAULT = 18
COMM_TERMINAL_CMD = 20
COMM_PRINT = 21
COMM_ROTOR_POSITION = 22
COMM_DETECT_ENCODER = 27
COMM_DETECT_HALL_FOC = 28
COMM_ALIVE = 30
COMM_GET_DECODED_ADC = 32
COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36
COMM_GET_VALUES_SETUP = 47
COMM_GET_VALUES_SELECTIVE = 50
COMM_GET_VALUES_SETUP_SELECTIVE = 51
COMM_DETECT_APPLY_ALL_FOC = 58
COMM_PING_CAN = 62
COMM_SET_CURRENT_REL = 84

VESC6_MCCONF_SIGNATURE = 776184161
VESC6_APPCONF_SIGNATURE = 486554156
HBTS_MAGIC = b"HBTS"
HBTS_VERSION = 15

COMMAND_TRACE_FIELDS = [
    "timestamp","scope","node","peer","kind","requested","expected_command","expected_wire_raw",
    "last_set_command","last_set_host_raw","last_set_normalized","runtime_setpoint","runtime_command",
    "last_set_run","armed","arm_reject","bridge_moe","bridge_warmup","current_measurement_valid",
    "standard_current_valid","internal_error","vesc_fault","vesc_imotor_A","vesc_ibattery_A",
    "vesc_id_A","vesc_iq_A","vesc_vd_V","vesc_vq_V","diag_id_A","diag_iq_A","dc_validated_A","dc_raw_A","duty","erpm",
    "position_deg","logical_position_deg","position_target_ticks","encoder_electrical_ready",
    "alignment_probe_delta","alignment_direction_proved","position_session_zero_ticks","observer_valid",
    "steering_calibrated","steering_homed","steering_span_ticks","duty_target_q15",
    "homing_on_boot","homing_active_this","auto_detect_stage","auto_detect_result",
    "peer_armed","peer_bridge_moe","peer_imotor_A","wire_fault_last",
    "wire_values_reply_count","wire_values_command","wire_values_payload_len"
]
DETECT_TRACE_FIELDS = [
    "timestamp","node","label","sensor_type","cal_state","cal_state_name","cal_motor","cal_type",
    "detect_result_code","cal_sweep_direction","cal_forward_cycles","cal_reverse_cycles","cal_completed_cycles",
    "cal_motion_detected","cal_motion_counter","cal_motion_age_ms","cal_observed_hall_mask",
    "cal_observed_encoder_mask","cal_encoder_delta","encoder_valid_edges","encoder_invalid_transitions",
    "detect_target_A","detect_measured_A","foc_id_target_A","foc_iq_target_A","id_A","iq_A",
    "foc_vd_internal","foc_vq_internal","commissioning_voltage_limit_internal","commissioning_vd_saturated",
    "commissioning_vq_saturated","bridge_moe","bridge_warmup_this","current_measurement_valid",
    "standard_current_valid","standard_pending_samples","uart_tx_drops","tx_queue_used",
    "pending_detect_owner","pending_detect_reply_retries","terminal_detect_valid","terminal_detect_state",
    "terminal_detect_state_name","terminal_detect_result_code","terminal_detect_sensor_type",
    "terminal_encoder_ratio_fallback","terminal_detect_pole_pairs","terminal_detect_encoder_inverted",
    "terminal_detect_encoder_cpr","encoder_ratio","encoder_electrical_ready",
    "alignment_probe_delta","alignment_direction_proved","position_session_zero_ticks","observer_valid",
    "steering_calibrated","steering_homed","logical_position_deg","homing_on_boot","homing_active_this",
    "auto_detect_stage","auto_detect_result","auto_detect_reply_retries",
    "wire_fault_last","wire_values_reply_count","internal_error","vesc_fault"
]
HBTS_GET_DIAG = 1
HBTS_CURRENT_RECAL = 2
HBTS_DIAG_REPLY = 0x81
HBTS_CURRENT_RECAL_REPLY = 0x82

FAULT_NAMES = {
    0: "NONE",
    1: "OVER_VOLTAGE",
    2: "UNDER_VOLTAGE",
    3: "DRV",
    4: "ABS_OVER_CURRENT",
}
INTERNAL_ERROR_NAMES = {
    0x00: "NONE",
    0x10: "HALL_NOT_DETECTED",
    0x11: "HALL_LUT_INVALID",
    0x20: "ENCODER_NOT_CALIBRATED",
    0x21: "ENCODER_NO_SIGNAL",
    0x22: "ENCODER_SIGNAL_INVALID",
    0x30: "CONTROL_ISR_OVERRUN",
    0x31: "ABS_OVER_CURRENT",
}
ARM_REJECT_NAMES = {
    0: "NONE",
    1: "UNSAFE_SETPOINT",
    2: "SENSOR_OR_FOC",
    3: "CALIBRATION_BUSY",
    4: "ALIGNMENT_BUSY",
    5: "LINK_TIMEOUT",
    6: "HOMING_BUSY",
    7: "CONTROL_OVERRUN",
    8: "CURRENT_OFFSET_NOT_READY",
}
SENSOR_NAMES = {0: "HALL_UVW", 1: "ENCODER_AB"}
BUZZER_REASON_NAMES = {
    0: "NONE",
    1: "HARD_RUNTIME_FAULT",
    2: "TEMPERATURE_WARNING",
    3: "BATTERY_LEVEL1",
    4: "BATTERY_LEVEL2",
    5: "REVERSE_WARNING",
}
APP_NAMES = {0: "NONE", 1: "PPM", 2: "ADC", 3: "UART", 4: "PPM_UART", 5: "ADC_UART"}
CAL_NAMES = {
    0: "IDLE", 1: "RUNNING", 2: "SUCCESS", 3: "FAILED_SEQUENCE",
    4: "FAILED_TIMEOUT", 5: "OVERCURRENT", 6: "ABORTED", 7: "PERSIST_FAILED",
}
CURRENT_CAL_NAMES = {0: "IDLE", 1: "SETTLING", 2: "COLLECTING", 3: "VALID", 4: "FAILED"}
AUTO_DETECT_STAGE_NAMES = {
    0: "IDLE", 1: "WAIT_CURRENT_CAL", 2: "RIGHT_HALL",
    3: "LEFT_ENCODER", 4: "LEFT_SYNC", 5: "REPLY",
}
HOMING_STATE_NAMES = {
    0: "IDLE", 1: "SEARCHING", 2: "HOMED", 3: "TIMEOUT", 4: "ABORTED",
    5: "FAULT", 6: "SYNC_ELECTRICAL", 7: "CAL_RIGHT", 8: "CAL_LEFT",
    9: "RETURN_CENTER", 10: "READY",
}
CURRENT_ADC_NAMES = ["rlA", "rlB", "rrB", "rrC", "dcl", "dcr"]


def current_cal_failure_reasons(mask: int) -> list[str]:
    reasons: list[str] = []
    for i, name in enumerate(CURRENT_ADC_NAMES):
        if mask & (1 << i):
            reasons.append(f"{name}:mean_out_of_range")
        if mask & (1 << (i + 6)):
            reasons.append(f"{name}:block_mean_unstable")
    if mask & (1 << 12): reasons.append("ADC1_hw_cal_failed")
    if mask & (1 << 13): reasons.append("ADC2_hw_cal_failed")
    if mask & (1 << 14): reasons.append("no_complete_blocks")
    if mask & (1 << 15): reasons.append("wheel_motion_during_active_zero_cal")
    return reasons


def now_iso() -> str:
    return dt.datetime.now().astimezone().isoformat(timespec="milliseconds")


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    if not payload:
        raise ValueError("empty VESC payload")
    if len(payload) <= 255:
        head = bytes((2, len(payload)))
    elif len(payload) <= 65535:
        head = bytes((3, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF))
    else:
        raise ValueError("VESC payload too long")
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


class StreamParser:
    """VESC byte-stream parser, including 0x03 stop/start ambiguity handling."""
    WAIT, LEN8, LEN16H, LEN16L, DATA, CRCH, CRCL, STOP = range(8)

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.state = self.WAIT
        self.need = 0
        self.long_hi = 0
        self.payload = bytearray()
        self.rx_crc = 0

    def feed(self, b: int) -> Optional[bytes]:
        b &= 0xFF
        if self.state == self.WAIT:
            if b == 2:
                self.state = self.LEN8
            elif b == 3:
                self.state = self.LEN16H
        elif self.state == self.LEN8:
            self.need = b
            self.payload.clear()
            self.state = self.DATA if self.need else self.WAIT
        elif self.state == self.LEN16H:
            self.long_hi = b
            self.state = self.LEN16L
        elif self.state == self.LEN16L:
            self.need = (self.long_hi << 8) | b
            self.payload.clear()
            self.state = self.DATA if self.need else self.WAIT
        elif self.state == self.DATA:
            self.payload.append(b)
            if len(self.payload) >= self.need:
                self.state = self.CRCH
        elif self.state == self.CRCH:
            self.rx_crc = b << 8
            self.state = self.CRCL
        elif self.state == self.CRCL:
            self.rx_crc |= b
            self.state = self.STOP
        elif self.state == self.STOP:
            result = None
            if b == 3 and crc16(self.payload) == self.rx_crc:
                result = bytes(self.payload)
            self.reset()
            # 0x03 is consumed as the stop byte and must not be reused as long start.
            if b != 3 and b == 2:
                self.state = self.LEN8
            return result
        return None


def be_i16(data: bytes, p: int) -> tuple[int, int]:
    if p + 2 > len(data):
        raise ValueError("short i16")
    return struct.unpack_from(">h", data, p)[0], p + 2


def be_u16(data: bytes, p: int) -> tuple[int, int]:
    if p + 2 > len(data):
        raise ValueError("short u16")
    return struct.unpack_from(">H", data, p)[0], p + 2


def be_i32(data: bytes, p: int) -> tuple[int, int]:
    if p + 4 > len(data):
        raise ValueError("short i32")
    return struct.unpack_from(">i", data, p)[0], p + 4


def be_u32(data: bytes, p: int) -> tuple[int, int]:
    if p + 4 > len(data):
        raise ValueError("short u32")
    return struct.unpack_from(">I", data, p)[0], p + 4


def pack_i32(v: int) -> bytes:
    return struct.pack(">i", int(v))


def pack_float32_scaled(v: float, scale: float) -> bytes:
    return pack_i32(round(v * scale))


def cstring(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\0", offset)
    if end < 0:
        raise ValueError("unterminated string")
    return data[offset:end].decode("ascii", "replace"), end + 1



def resolve_inventory_ids(local_controller_id: Optional[int], can_ids: list[int]) -> tuple[int, int, str]:
    """Resolve local + virtual-right IDs without any HBTS dependency.

    Standard VESC COMM_GET_VALUES is authoritative when it carries controller_id.
    A single PING_CAN response can be used only as a compatibility fallback when
    its ID is nonzero; virtual ID 0 is ambiguous because firmware maps local IDs
    254 and 255 to virtual ID 0.
    """
    ids = [int(x) & 0xFF for x in can_ids]
    if not ids:
        raise RuntimeError("PING_CAN returned no virtual motor")

    if local_controller_id is not None:
        local_id = int(local_controller_id) & 0xFF
        source = "COMM_GET_VALUES"
    else:
        if len(ids) != 1:
            raise RuntimeError(
                f"local controller ID absent from GET_VALUES and CAN inventory is ambiguous: {ids}"
            )
        candidate_right = ids[0]
        if candidate_right == 0:
            raise RuntimeError(
                "local controller ID absent from GET_VALUES and virtual CAN ID 0 is ambiguous "
                "(local could be 254 or 255)"
            )
        local_id = candidate_right - 1
        source = "derived_from_single_virtual_CAN"

    right_id = 0 if local_id >= 254 else local_id + 1
    if right_id not in ids:
        raise RuntimeError(f"expected virtual CAN {right_id}, got {ids}")
    return local_id, right_id, source

def decode_fw(payload: bytes) -> dict[str, Any]:
    if len(payload) < 4 or payload[0] != COMM_FW_VERSION:
        raise ValueError("not COMM_FW_VERSION")
    major, minor = payload[1], payload[2]
    hw, p = cstring(payload, 3)
    if p + 12 > len(payload):
        raise ValueError("short FW UUID")
    uuid = payload[p:p + 12]
    p += 12
    extras = payload[p:p + 8]
    p += min(8, len(payload) - p)
    fw_name = ""
    if p < len(payload):
        try:
            fw_name, _ = cstring(payload, p)
        except ValueError:
            pass
    return {
        "major": major, "minor": minor, "version": f"{major}.{minor:02d}",
        "hardware": hw, "uuid": uuid.hex(), "extra": extras.hex(), "fw_name": fw_name,
    }


def decode_values(payload: bytes) -> dict[str, Any]:
    if not payload or payload[0] not in (COMM_GET_VALUES, COMM_GET_VALUES_SELECTIVE):
        raise ValueError("not COMM_GET_VALUES")
    p = 1
    # This tester requests non-selective values, so no mask follows.
    temp_fet, p = be_i16(payload, p)
    temp_motor, p = be_i16(payload, p)
    motor_current, p = be_i32(payload, p)
    input_current, p = be_i32(payload, p)
    id_raw, p = be_i32(payload, p)
    iq_raw, p = be_i32(payload, p)
    duty, p = be_i16(payload, p)
    erpm, p = be_i32(payload, p)
    vin, p = be_i16(payload, p)
    ah, p = be_i32(payload, p)
    ah_chg, p = be_i32(payload, p)
    wh, p = be_i32(payload, p)
    wh_chg, p = be_i32(payload, p)
    tach, p = be_i32(payload, p)
    tach_abs, p = be_i32(payload, p)
    if p >= len(payload):
        raise ValueError("short values fault")
    fault = payload[p]; p += 1
    pos = None
    controller_id = None
    mos_temps: list[float] = []
    vd = vq = None
    status = None
    if p + 4 <= len(payload):
        raw, p = be_i32(payload, p); pos = raw / 1_000_000.0
    if p < len(payload):
        controller_id = payload[p]; p += 1
    for _ in range(3):
        if p + 2 <= len(payload):
            raw, p = be_i16(payload, p); mos_temps.append(raw / 10.0)
    if p + 4 <= len(payload):
        raw, p = be_i32(payload, p); vd = raw / 1000.0
    if p + 4 <= len(payload):
        raw, p = be_i32(payload, p); vq = raw / 1000.0
    if p < len(payload):
        status = payload[p]; p += 1
    return {
        "temp_fet_C": temp_fet / 10.0,
        "temp_motor_C": temp_motor / 10.0,
        "motor_current_A": motor_current / 100.0,
        "input_current_A": input_current / 100.0,
        "id_A": id_raw / 100.0,
        "iq_A": iq_raw / 100.0,
        "duty": duty / 1000.0,
        "erpm": erpm,
        "vin_V": vin / 10.0,
        "ah": ah / 10000.0, "ah_charged": ah_chg / 10000.0,
        "wh": wh / 10000.0, "wh_charged": wh_chg / 10000.0,
        "tach": tach, "tach_abs": tach_abs,
        "fault": fault, "fault_name": FAULT_NAMES.get(fault, f"UNKNOWN_{fault}"),
        "position_deg": pos, "controller_id": controller_id,
        "mos_temps_C": mos_temps, "vd": vd, "vq": vq, "status": status,
        "payload_len": len(payload),
    }



def decode_values_selective(payload: bytes, expected_mask: int) -> dict[str, Any]:
    if len(payload) < 5 or payload[0] != COMM_GET_VALUES_SELECTIVE:
        raise ValueError("not COMM_GET_VALUES_SELECTIVE")
    p = 1
    mask, p = be_u32(payload, p)
    if mask != (expected_mask & 0xFFFFFFFF):
        raise ValueError(f"selective mask mismatch got=0x{mask:08x} expected=0x{expected_mask:08x}")
    out: dict[str, Any] = {"mask": mask}
    names = {
        0:"temp_fet_C",1:"temp_motor_C",2:"motor_current_A",3:"input_current_A",
        4:"id_A",5:"iq_A",6:"duty",7:"erpm",8:"vin_V",9:"ah",10:"ah_charged",
        11:"wh",12:"wh_charged",13:"tach",14:"tach_abs",15:"fault",16:"position_deg",
        17:"controller_id",18:"mos_temps_C",19:"vd",20:"vq",21:"status"
    }
    for bit in range(22):
        if not (mask & (1 << bit)):
            continue
        name = names[bit]
        if bit in (0,1): raw,p=be_i16(payload,p); out[name]=raw/10.0
        elif bit in (2,3,4,5): raw,p=be_i32(payload,p); out[name]=raw/100.0
        elif bit == 6: raw,p=be_i16(payload,p); out[name]=raw/1000.0
        elif bit == 7: raw,p=be_i32(payload,p); out[name]=raw
        elif bit == 8: raw,p=be_i16(payload,p); out[name]=raw/10.0
        elif bit in (9,10,11,12): raw,p=be_i32(payload,p); out[name]=raw/10000.0
        elif bit in (13,14): raw,p=be_i32(payload,p); out[name]=raw
        elif bit == 15: out[name]=payload[p]; p += 1
        elif bit == 16: raw,p=be_i32(payload,p); out[name]=raw/1_000_000.0
        elif bit == 17: out[name]=payload[p]; p += 1
        elif bit == 18:
            vals=[]
            for _ in range(3): raw,p=be_i16(payload,p); vals.append(raw/10.0)
            out[name]=vals
        elif bit in (19,20): raw,p=be_i32(payload,p); out[name]=raw/1000.0
        elif bit == 21: out[name]=payload[p]; p += 1
    out["payload_len"] = len(payload)
    out["decoded_bytes"] = p
    if p != len(payload):
        out["trailing_hex"] = payload[p:].hex()
    return out


def decode_setup_values(payload: bytes, expected_mask: Optional[int] = None) -> dict[str, Any]:
    if not payload or payload[0] not in (COMM_GET_VALUES_SETUP, COMM_GET_VALUES_SETUP_SELECTIVE):
        raise ValueError("not COMM_GET_VALUES_SETUP")
    p=1
    mask=0xFFFFFFFF
    if payload[0] == COMM_GET_VALUES_SETUP_SELECTIVE:
        mask,p=be_u32(payload,p)
        if expected_mask is not None and mask != (expected_mask & 0xFFFFFFFF):
            raise ValueError(f"setup mask mismatch got=0x{mask:08x} expected=0x{expected_mask:08x}")
    out: dict[str, Any] = {"mask": mask}
    names={0:"temp_fet_C",1:"temp_motor_C",2:"motor_current_A",3:"input_current_A",4:"duty",5:"erpm",
           6:"speed_mps",7:"vin_V",8:"battery_level",9:"ah",10:"ah_charged",11:"wh",12:"wh_charged",
           13:"distance_m",14:"distance_abs_m",15:"position_deg",16:"fault",17:"controller_id",18:"num_vescs",
           19:"wh_batt_left",20:"odometer_m",21:"uptime_ms"}
    for bit in range(22):
        if not (mask & (1 << bit)): continue
        name=names[bit]
        if bit in (0,1): raw,p=be_i16(payload,p); out[name]=raw/10.0
        elif bit in (2,3): raw,p=be_i32(payload,p); out[name]=raw/100.0
        elif bit == 4: raw,p=be_i16(payload,p); out[name]=raw/1000.0
        elif bit == 5: raw,p=be_i32(payload,p); out[name]=raw
        elif bit == 6: raw,p=be_i32(payload,p); out[name]=raw/1000.0
        elif bit == 7: raw,p=be_i16(payload,p); out[name]=raw/10.0
        elif bit == 8: raw,p=be_i16(payload,p); out[name]=raw/1000.0
        elif bit in (9,10,11,12): raw,p=be_i32(payload,p); out[name]=raw/10000.0
        elif bit in (13,14): raw,p=be_i32(payload,p); out[name]=raw/1000.0
        elif bit == 15: raw,p=be_i32(payload,p); out[name]=raw/1_000_000.0
        elif bit in (16,17,18): out[name]=payload[p]; p += 1
        elif bit == 19: raw,p=be_i32(payload,p); out[name]=raw/1000.0
        elif bit in (20,21): raw,p=be_u32(payload,p); out[name]=raw
    out["payload_len"]=len(payload); out["decoded_bytes"]=p
    if p != len(payload): out["trailing_hex"]=payload[p:].hex()
    return out


def decode_rotor_position(payload: bytes) -> float:
    if len(payload) != 5 or payload[0] != COMM_ROTOR_POSITION:
        raise ValueError(f"invalid rotor position reply len={len(payload)}")
    raw,_=be_i32(payload,1)
    return raw/100000.0

def decode_adc(payload: bytes) -> dict[str, Any]:
    if len(payload) < 17 or payload[0] != COMM_GET_DECODED_ADC:
        raise ValueError("short/not decoded ADC")
    p = 1
    d1, p = be_i32(payload, p); v1, p = be_i32(payload, p)
    d2, p = be_i32(payload, p); v2, p = be_i32(payload, p)
    return {
        "decoded1": d1 / 1_000_000.0,
        "voltage1_V": v1 / 1_000_000.0,
        "decoded2": d2 / 1_000_000.0,
        "voltage2_V": v2 / 1_000_000.0,
    }


def decode_diag(payload: bytes) -> dict[str, Any]:
    if len(payload) < 8 or payload[0] != COMM_CUSTOM_APP_DATA or payload[1:5] != HBTS_MAGIC:
        raise ValueError("not HBTS diagnostic")
    if payload[5] != HBTS_VERSION or payload[6] != HBTS_DIAG_REPLY:
        raise ValueError(f"unsupported HBTS version/op {payload[5:7].hex()}")
    p = 7
    node = payload[p]; p += 1
    side = payload[p]; p += 1
    flags, p = be_u16(payload, p)
    sensor_type = payload[p]; p += 1
    arm_reject = payload[p]; p += 1
    internal_error = payload[p]; p += 1
    vesc_fault = payload[p]; p += 1
    control_mode = payload[p]; p += 1
    hall_raw = payload[p]; p += 1
    encoder_ab = payload[p]; p += 1
    pole_pairs = payload[p]; p += 1
    encoder_cpr, p = be_u16(payload, p)
    fault_remaining, p = be_u16(payload, p)
    link_age, p = be_u16(payload, p)
    position_ticks, p = be_i32(payload, p)
    mech_q4, p = be_i16(payload, p)
    elec_phase, p = be_u16(payload, p)
    speed_q4, p = be_i16(payload, p)
    speed_rpm, p = be_i32(payload, p)
    id_raw, p = be_i16(payload, p)
    iq_raw, p = be_i16(payload, p)
    duty_q15, p = be_u16(payload, p)
    dc_ca, p = be_i16(payload, p)
    bat_cv, p = be_i16(payload, p)
    units_per_amp, p = be_u16(payload, p)
    runtime_setpoint, p = be_i32(payload, p)
    runtime_command, p = be_i16(payload, p)
    isr_overruns, p = be_u16(payload, p)
    isr_last_cycles, p = be_u32(payload, p)
    isr_max_cycles, p = be_u32(payload, p)
    isr_deadline_cycles, p = be_u32(payload, p)
    overrun_mask = payload[p]; p += 1
    overcurrent_mask = payload[p]; p += 1
    enc_valid, p = be_u32(payload, p)
    enc_invalid, p = be_u32(payload, p)
    rx_packets, p = be_u32(payload, p)
    rx_errors, p = be_u32(payload, p)
    tx_drops, p = be_u32(payload, p)
    eeprom_generation, p = be_u16(payload, p)
    eeprom_verify_fail, p = be_u16(payload, p)
    pa2_raw, p = be_u16(payload, p)
    pa3_raw, p = be_u16(payload, p)
    adc1_decoded, p = be_i32(payload, p)
    adc2_decoded, p = be_i32(payload, p)
    cal_state = payload[p]; p += 1
    cal_motor = payload[p]; p += 1
    cal_type = payload[p]; p += 1
    homing_state = payload[p]; p += 1
    app_mode = payload[p] if p < len(payload) else None; p += int(p < len(payload))
    adc_ctrl = payload[p] if p < len(payload) else None; p += int(p < len(payload))
    multi_esc = payload[p] if p < len(payload) else None; p += int(p < len(payload))
    timeout_ms = None
    if p + 4 <= len(payload):
        timeout_ms, p = be_u32(payload, p)

    # HBTS v5 current calibration + current-path evidence.
    current_cal = {}
    battery_total_A = None
    imotor_diag_A = None
    detect_result_code = None
    dc_input_validated_A = None
    dc_raw_A = None
    dc_reject_count = None
    detect_target_internal = None
    detect_measured_internal = None
    detect_phase_q16 = None
    foc_id_target_internal = None
    foc_iq_target_internal = None
    foc_vd_internal = None
    foc_vq_internal = None
    foc_mod_d = None
    foc_mod_q = None
    passive_current_rejects = None
    commissioning_fast_fault_mask = None
    foc_current_kp_q16 = None
    foc_current_ki_dt_q16 = None
    buzzer_reason = None
    buzzer_last_reason = None
    buzzer_event_count = None
    standard_bridge_active = None
    standard_current_valid = None
    standard_current_generation = None
    standard_current_age_ms = None
    standard_last_id_internal = None
    standard_last_iq_internal = None
    standard_last_dc_centi_A = None
    commissioning_fault_snapshot: dict[str, Any] = {}
    route_packets_this = None
    route_packets_other = None
    set_packets_this = None
    last_set_command = None
    current_cal_sampling_mode = None
    current_cal_bridge_active_mask = None
    bridge_warmup_left = None
    bridge_warmup_right = None
    bridge_transition_left = None
    bridge_transition_right = None
    last_set_host_raw = None
    last_set_normalized = None
    last_set_run = None
    current_cal_final_active_raw = None
    current_cal_final_active_residual = None
    cal_sweep_direction = None
    cal_forward_cycles = None
    cal_reverse_cycles = None
    cal_completed_cycles = None
    cal_motion_detected = None
    cal_motion_counter = None
    cal_motion_age_ms = None
    cal_observed_hall_mask = None
    cal_observed_encoder_mask = None
    cal_encoder_delta = None
    standard_pending_samples = None
    standard_pending_dc_samples = None
    commissioning_voltage_limit_internal = None
    commissioning_vd_saturated = None
    commissioning_vq_saturated = None
    wire_values_reply_count = None
    wire_fault_last = None
    wire_values_command = None
    wire_values_payload_len = None
    tx_queue_used = None
    pending_detect_owner = None
    pending_detect_reply_retries = None
    terminal_detect_valid = None
    terminal_detect_state = None
    terminal_detect_result_code = None
    terminal_detect_sensor_type = None
    terminal_encoder_ratio_fallback = None
    terminal_detect_pole_pairs = None
    terminal_detect_encoder_inverted = None
    terminal_detect_encoder_cpr = None
    cal_encoder_forward_delta = None
    cal_encoder_direction_normal_score = None
    cal_encoder_direction_inverted_score = None
    cal_encoder_direction_proved = None
    encoder_ratio = None
    alignment_probe_delta = None
    alignment_direction_proved = None
    position_session_zero_ticks = None
    observer_valid = None
    encoder_electrical_ready = None
    steering_calibrated = None
    steering_homed = None
    steering_zero_ticks = None
    steering_span_ticks = None
    logical_position_deg = None
    position_target_ticks = None
    duty_target_q15 = None
    homing_on_boot = None
    homing_active_this = None
    rotor_display_mode = None
    auto_detect_active = None
    auto_detect_stage = None
    auto_detect_result = None
    auto_detect_reply_retries = None
    if payload[5] >= 2 and p + 10 <= len(payload):
        cc_state = payload[p]; p += 1
        cc_valid = bool(payload[p]); p += 1
        adc1_hw = bool(payload[p]); p += 1
        adc2_hw = bool(payload[p]); p += 1
        cc_samples, p = be_u16(payload, p)
        cc_target, p = be_u16(payload, p)
        cc_fail, p = be_u16(payload, p)
        cc_gen, p = be_u16(payload, p)
        raw=[]; candidate=[]; off=[]; raw_span=[]; block_span=[]; residual=[]
        for _ in range(6): x,p=be_u16(payload,p); raw.append(x)
        for _ in range(6): x,p=be_u16(payload,p); candidate.append(x)
        for _ in range(6): x,p=be_u16(payload,p); off.append(x)
        for _ in range(6): x,p=be_u16(payload,p); raw_span.append(x)
        for _ in range(6): x,p=be_u16(payload,p); block_span.append(x)
        for _ in range(6): x,p=be_i16(payload,p); residual.append(x)
        batt_ca,p=be_i16(payload,p)
        im_ca,p=be_i16(payload,p)
        detect_result_code = payload[p] if p < len(payload) else None
        p += int(p < len(payload))
        current_cal={
            "state":cc_state,"state_name":CURRENT_CAL_NAMES.get(cc_state,str(cc_state)),
            "valid":cc_valid,"adc1_hw_cal_ok":adc1_hw,"adc2_hw_cal_ok":adc2_hw,
            "collected_samples":cc_samples,"target_samples":cc_target,
            "failure_mask":cc_fail,"failure_reasons":current_cal_failure_reasons(cc_fail),"generation":cc_gen,
            "raw":dict(zip(CURRENT_ADC_NAMES,raw)),
            "candidate_mean":dict(zip(CURRENT_ADC_NAMES,candidate)),
            "offset":dict(zip(CURRENT_ADC_NAMES,off)),
            "raw_span":dict(zip(CURRENT_ADC_NAMES,raw_span)),
            "block_span":dict(zip(CURRENT_ADC_NAMES,block_span)),
            "residual":dict(zip(CURRENT_ADC_NAMES,residual)),
        }
        battery_total_A=batt_ca/100.0
        imotor_diag_A=im_ca/100.0
        if payload[5] >= 5 and p + 16 <= len(payload):
            dc_valid_ca,p=be_i16(payload,p)
            dc_raw_ca,p=be_i16(payload,p)
            dc_reject_count,p=be_u16(payload,p)
            detect_target_internal,p=be_i16(payload,p)
            detect_measured_internal,p=be_i16(payload,p)
            detect_phase_q16,p=be_u16(payload,p)
            foc_id_target_internal,p=be_i16(payload,p)
            foc_iq_target_internal,p=be_i16(payload,p)
            dc_input_validated_A=dc_valid_ca/100.0
            dc_raw_A=dc_raw_ca/100.0
            if payload[5] >= 6 and p + 19 <= len(payload):
                foc_vd_internal,p=be_i16(payload,p)
                foc_vq_internal,p=be_i16(payload,p)
                foc_mod_d,p=be_i16(payload,p)
                foc_mod_q,p=be_i16(payload,p)
                passive_current_rejects,p=be_u16(payload,p)
                commissioning_fast_fault_mask=payload[p]; p += 1
                foc_current_kp_q16,p=be_i32(payload,p)
                foc_current_ki_dt_q16,p=be_i32(payload,p)
                if payload[5] >= 7 and p + 4 <= len(payload):
                    buzzer_reason = payload[p]; p += 1
                    buzzer_last_reason = payload[p]; p += 1
                    buzzer_event_count,p=be_u16(payload,p)
                    if payload[5] >= 8 and p + 12 <= len(payload):
                        standard_bridge_active = bool(payload[p]); p += 1
                        standard_current_valid = bool(payload[p]); p += 1
                        standard_current_generation,p=be_u16(payload,p)
                        standard_current_age_ms,p=be_u16(payload,p)
                        standard_last_id_internal,p=be_i16(payload,p)
                        standard_last_iq_internal,p=be_i16(payload,p)
                        standard_last_dc_centi_A,p=be_i16(payload,p)
                        if p + 40 <= len(payload):
                            fs_valid = bool(payload[p]); p += 1
                            fs_mask = payload[p]; p += 1
                            fs_streak = payload[p]; p += 1
                            fs_left = bool(payload[p]); p += 1
                            fs_gen,p=be_u16(payload,p)
                            fs_target,p=be_i16(payload,p)
                            fs_id,p=be_i16(payload,p)
                            fs_iq,p=be_i16(payload,p)
                            fs_p1,p=be_i16(payload,p)
                            fs_p2,p=be_i16(payload,p)
                            fs_dc,p=be_i16(payload,p)
                            fs_adc1,p=be_u16(payload,p)
                            fs_adc2,p=be_u16(payload,p)
                            fs_adcdc,p=be_u16(payload,p)
                            fs_off1,p=be_u16(payload,p)
                            fs_off2,p=be_u16(payload,p)
                            fs_offdc,p=be_u16(payload,p)
                            fs_phase,p=be_u16(payload,p)
                            fs_da,p=be_i16(payload,p)
                            fs_db,p=be_i16(payload,p)
                            fs_dc_duty,p=be_i16(payload,p)
                            fs_dabs,p=be_u16(payload,p)
                            commissioning_fault_snapshot={
                                "valid":fs_valid,"fault_mask":fs_mask,"streak":fs_streak,"left":fs_left,
                                "generation":fs_gen,"target_internal":fs_target,"id_internal":fs_id,"iq_internal":fs_iq,
                                "phase_current_1_delta":fs_p1,"phase_current_2_delta":fs_p2,"dc_delta":fs_dc,
                                "adc_phase_1_raw":fs_adc1,"adc_phase_2_raw":fs_adc2,"adc_dc_raw":fs_adcdc,
                                "offset_phase_1":fs_off1,"offset_phase_2":fs_off2,"offset_dc":fs_offdc,
                                "forced_phase_q16":fs_phase,"duty_a":fs_da,"duty_b":fs_db,"duty_c":fs_dc_duty,
                                "duty_abs_q15":fs_dabs,
                            }
                            if payload[5] >= 9 and p + 13 <= len(payload):
                                route_packets_this,p=be_u32(payload,p)
                                route_packets_other,p=be_u32(payload,p)
                                set_packets_this,p=be_u32(payload,p)
                                last_set_command=payload[p]; p += 1
                                if payload[5] >= 10 and p + 17 <= len(payload):
                                    current_cal_sampling_mode=payload[p]; p += 1
                                    current_cal_bridge_active_mask=payload[p]; p += 1
                                    bridge_warmup_left=payload[p]; p += 1
                                    bridge_warmup_right=payload[p]; p += 1
                                    bridge_transition_left,p=be_u16(payload,p)
                                    bridge_transition_right,p=be_u16(payload,p)
                                    last_set_host_raw,p=be_i32(payload,p)
                                    last_set_normalized,p=be_i32(payload,p)
                                    last_set_run=bool(payload[p]); p += 1
                                    if p + 24 <= len(payload):
                                        far=[]; fres=[]
                                        for _ in range(6): x,p=be_u16(payload,p); far.append(x)
                                        for _ in range(6): x,p=be_i16(payload,p); fres.append(x)
                                        current_cal_final_active_raw=dict(zip(CURRENT_ADC_NAMES,far))
                                        current_cal_final_active_residual=dict(zip(CURRENT_ADC_NAMES,fres))
                                        if payload[5] >= 11 and p + 34 <= len(payload):
                                            raw_dir=payload[p]; p += 1
                                            cal_sweep_direction=raw_dir-256 if raw_dir >= 128 else raw_dir
                                            cal_forward_cycles=payload[p]; p += 1
                                            cal_reverse_cycles=payload[p]; p += 1
                                            cal_completed_cycles,p=be_u16(payload,p)
                                            cal_motion_detected=bool(payload[p]); p += 1
                                            cal_motion_counter,p=be_u32(payload,p)
                                            cal_motion_age_ms,p=be_u16(payload,p)
                                            cal_observed_hall_mask=payload[p]; p += 1
                                            cal_observed_encoder_mask=payload[p]; p += 1
                                            cal_encoder_delta,p=be_i32(payload,p)
                                            standard_pending_samples,p=be_u16(payload,p)
                                            standard_pending_dc_samples,p=be_u16(payload,p)
                                            commissioning_voltage_limit_internal,p=be_u16(payload,p)
                                            commissioning_vd_saturated=bool(payload[p]); p += 1
                                            commissioning_vq_saturated=bool(payload[p]); p += 1
                                            wire_values_reply_count,p=be_u32(payload,p)
                                            wire_fault_last=payload[p]; p += 1
                                            wire_values_command=payload[p]; p += 1
                                            wire_values_payload_len,p=be_u16(payload,p)
    if payload[5] >= 12 and p + 13 <= len(payload):
        tx_queue_used = payload[p]; p += 1
        pending_detect_owner = payload[p]; p += 1
        pending_detect_reply_retries, p = be_u16(payload, p)
        terminal_detect_valid = bool(payload[p]); p += 1
        terminal_detect_state = payload[p]; p += 1
        terminal_detect_result_code = payload[p]; p += 1
        terminal_detect_sensor_type = payload[p]; p += 1
        terminal_encoder_ratio_fallback = bool(payload[p]); p += 1
        terminal_detect_pole_pairs = payload[p]; p += 1
        terminal_detect_encoder_inverted = bool(payload[p]); p += 1
        terminal_detect_encoder_cpr, p = be_u16(payload, p)
    if payload[5] >= 13 and p + 13 <= len(payload):
        cal_encoder_forward_delta, p = be_i32(payload, p)
        cal_encoder_direction_normal_score, p = be_u32(payload, p)
        cal_encoder_direction_inverted_score, p = be_u32(payload, p)
        cal_encoder_direction_proved = bool(payload[p]); p += 1
    if payload[5] >= 14 and p + 31 <= len(payload):
        encoder_ratio = payload[p]; p += 1
        encoder_electrical_ready = bool(payload[p]); p += 1
        steering_calibrated = bool(payload[p]); p += 1
        steering_homed = bool(payload[p]); p += 1
        steering_zero_ticks, p = be_i32(payload, p)
        steering_span_ticks, p = be_i32(payload, p)
        logical_position_mdeg, p = be_i32(payload, p)
        logical_position_deg = logical_position_mdeg / 1000.0
        position_target_ticks, p = be_i32(payload, p)
        duty_target_q15, p = be_i16(payload, p)
        homing_on_boot = bool(payload[p]); p += 1
        homing_active_this = bool(payload[p]); p += 1
        rotor_display_mode = payload[p]; p += 1
        auto_detect_active = bool(payload[p]); p += 1
        auto_detect_stage = payload[p]; p += 1
        auto_detect_result, p = be_i16(payload, p)
        auto_detect_reply_retries, p = be_u16(payload, p)
    if payload[5] >= 15 and p + 8 <= len(payload):
        alignment_probe_delta, p = be_i16(payload, p)
        alignment_direction_proved = bool(payload[p]); p += 1
        position_session_zero_ticks, p = be_i32(payload, p)
        observer_valid = bool(payload[p]); p += 1

    if current_cal:
        current_cal.update({
            "sampling_mode": current_cal_sampling_mode,
            "sampling_mode_name": "ACTIVE_LOW_FET_ZERO_VECTOR" if current_cal_sampling_mode == 1 else str(current_cal_sampling_mode),
            "bridge_active_mask": current_cal_bridge_active_mask,
            "bridge_warmup_left": bridge_warmup_left,
            "bridge_warmup_right": bridge_warmup_right,
            "bridge_transition_left": bridge_transition_left,
            "bridge_transition_right": bridge_transition_right,
            "final_active_raw": current_cal_final_active_raw or {},
            "final_active_residual": current_cal_final_active_residual or {},
        })
    amp_scale = units_per_amp if units_per_amp else 1
    current_flat: dict[str, Any] = {}
    if current_cal:
        current_flat = {
            "current_cal_state": current_cal.get("state"),
            "current_cal_state_name": current_cal.get("state_name"),
            "current_cal_valid": current_cal.get("valid"),
            "current_cal_adc1_hw_ok": current_cal.get("adc1_hw_cal_ok"),
            "current_cal_adc2_hw_ok": current_cal.get("adc2_hw_cal_ok"),
            "current_cal_samples": current_cal.get("collected_samples"),
            "current_cal_target_samples": current_cal.get("target_samples"),
            "current_cal_failure_mask": current_cal.get("failure_mask"),
            "current_cal_failure_reasons": "|".join(current_cal.get("failure_reasons", [])),
            "current_cal_generation": current_cal.get("generation"),
            "current_cal_sampling_mode": current_cal.get("sampling_mode"),
            "current_cal_sampling_mode_name": current_cal.get("sampling_mode_name"),
            "current_cal_bridge_active_mask": current_cal.get("bridge_active_mask"),
            "bridge_warmup_left": current_cal.get("bridge_warmup_left"),
            "bridge_warmup_right": current_cal.get("bridge_warmup_right"),
            "bridge_transition_left": current_cal.get("bridge_transition_left"),
            "bridge_transition_right": current_cal.get("bridge_transition_right"),
        }
        for prefix in ("raw", "candidate_mean", "offset", "raw_span", "block_span", "residual",
                       "final_active_raw", "final_active_residual"):
            for name, value in current_cal.get(prefix, {}).items():
                current_flat[f"current_{prefix}_{name}"] = value

    return {
        "node_id": node, "side": "RIGHT" if side else "LEFT",
        "flags_raw": flags,
        "armed": bool(flags & (1 << 0)), "output_enabled": bool(flags & (1 << 1)),
        "link_active": bool(flags & (1 << 2)), "feedback_valid": bool(flags & (1 << 3)),
        "calibrated": bool(flags & (1 << 4)), "encoder_aligned": bool(flags & (1 << 5)),
        "eeprom_verified": bool(flags & (1 << 6)), "detect_pending": bool(flags & (1 << 7)),
        "calibration_running": bool(flags & (1 << 8)), "homing_active": bool(flags & (1 << 9)),
        "encoder_alignment_runtime": bool(flags & (1 << 10)),
        "current_offsets_valid": bool(flags & (1 << 11)),
        "current_measurement_valid": bool(flags & (1 << 12)),
        "bridge_moe": bool(flags & (1 << 13)),
        "detect_current_control_active": bool(flags & (1 << 14)),
        "sensor_type": sensor_type, "sensor_name": SENSOR_NAMES.get(sensor_type, str(sensor_type)),
        "arm_reject": arm_reject, "arm_reject_name": ARM_REJECT_NAMES.get(arm_reject, str(arm_reject)),
        "internal_error": internal_error, "internal_error_name": INTERNAL_ERROR_NAMES.get(internal_error, hex(internal_error)),
        "vesc_fault": vesc_fault, "vesc_fault_name": FAULT_NAMES.get(vesc_fault, str(vesc_fault)),
        "control_mode": control_mode, "hall_raw": hall_raw, "encoder_ab": encoder_ab,
        "pole_pairs": pole_pairs, "encoder_cpr": encoder_cpr,
        "fault_stop_remaining_ms": fault_remaining, "link_age_ms": link_age,
        "position_ticks": position_ticks, "mechanical_angle_deg": mech_q4 / 16.0,
        "electrical_phase_q16": elec_phase, "mechanical_speed_rpm": speed_q4 / 16.0,
        "speed_rpm": speed_rpm, "id_raw": id_raw, "iq_raw": iq_raw,
        "id_A": id_raw / amp_scale, "iq_A": iq_raw / amp_scale,
        "duty_q15": duty_q15, "duty_abs": duty_q15 / 32767.0,
        "dc_current_A": dc_ca / 100.0, "battery_V": bat_cv / 100.0,
        "current_units_per_amp": units_per_amp,
        "current_internal_shift": 4,
        "current_adc_counts_per_amp": (units_per_amp / 16.0) if units_per_amp else None,
        "current_scale_matches_stock_board": units_per_amp == 800,
        "runtime_setpoint": runtime_setpoint, "runtime_command": runtime_command,
        "isr_overrun_count": isr_overruns,
        "isr_last_cycles": isr_last_cycles, "isr_max_cycles": isr_max_cycles,
        "isr_deadline_cycles": isr_deadline_cycles,
        "isr_load_pct": (100.0 * isr_last_cycles / isr_deadline_cycles) if isr_deadline_cycles else None,
        "isr_max_load_pct": (100.0 * isr_max_cycles / isr_deadline_cycles) if isr_deadline_cycles else None,
        "isr_overrun_fault_mask": overrun_mask,
        "overcurrent_fault_mask": overcurrent_mask,
        "encoder_valid_edges": enc_valid, "encoder_invalid_transitions": enc_invalid,
        "uart_rx_packets": rx_packets, "uart_rx_errors": rx_errors, "uart_tx_drops": tx_drops,
        "eeprom_generation": eeprom_generation, "eeprom_verify_failures": eeprom_verify_fail,
        "pa2_raw": pa2_raw, "pa3_raw": pa3_raw,
        "adc1_decoded": adc1_decoded / 1_000_000.0, "adc2_decoded": adc2_decoded / 1_000_000.0,
        "cal_state": cal_state, "cal_state_name": CAL_NAMES.get(cal_state, str(cal_state)),
        "cal_motor": cal_motor, "cal_type": cal_type, "homing_state": homing_state,
        "app_mode": app_mode, "app_name": APP_NAMES.get(app_mode, str(app_mode)) if app_mode is not None else None,
        "adc_ctrl_type": adc_ctrl, "multi_esc": multi_esc, "timeout_ms": timeout_ms,
        "current_cal": current_cal, "battery_current_total_A": battery_total_A,
        "imotor_diag_A": imotor_diag_A, "detect_result_code": detect_result_code,
        "dc_input_validated_A": dc_input_validated_A, "dc_raw_A": dc_raw_A,
        "dc_telemetry_reject_count": dc_reject_count,
        "detect_target_internal": detect_target_internal,
        "detect_target_A": (detect_target_internal / amp_scale) if detect_target_internal is not None else None,
        "detect_measured_internal": detect_measured_internal,
        "detect_measured_A": (detect_measured_internal / amp_scale) if detect_measured_internal is not None else None,
        "detect_phase_q16": detect_phase_q16,
        "foc_id_target_internal": foc_id_target_internal,
        "foc_id_target_A": (foc_id_target_internal / amp_scale) if foc_id_target_internal is not None else None,
        "foc_iq_target_internal": foc_iq_target_internal,
        "foc_iq_target_A": (foc_iq_target_internal / amp_scale) if foc_iq_target_internal is not None else None,
        "foc_vd_internal": foc_vd_internal,
        "foc_vq_internal": foc_vq_internal,
        "foc_mod_d": foc_mod_d,
        "foc_mod_q": foc_mod_q,
        "passive_current_rejects": passive_current_rejects,
        "commissioning_fast_fault_mask": commissioning_fast_fault_mask,
        "foc_current_kp_q16": foc_current_kp_q16,
        "foc_current_kp": (foc_current_kp_q16 / 65536.0) if foc_current_kp_q16 is not None else None,
        "foc_current_ki_dt_q16": foc_current_ki_dt_q16,
        "foc_current_ki_dt": (foc_current_ki_dt_q16 / 65536.0) if foc_current_ki_dt_q16 is not None else None,
        "buzzer_reason": buzzer_reason,
        "buzzer_reason_name": BUZZER_REASON_NAMES.get(buzzer_reason, str(buzzer_reason)) if buzzer_reason is not None else None,
        "buzzer_last_reason": buzzer_last_reason,
        "buzzer_last_reason_name": BUZZER_REASON_NAMES.get(buzzer_last_reason, str(buzzer_last_reason)) if buzzer_last_reason is not None else None,
        "buzzer_event_count": buzzer_event_count,
        "standard_bridge_active": standard_bridge_active,
        "standard_current_valid": standard_current_valid,
        "standard_current_generation": standard_current_generation,
        "standard_current_age_ms": standard_current_age_ms,
        "standard_last_id_internal": standard_last_id_internal,
        "standard_last_iq_internal": standard_last_iq_internal,
        "standard_last_dc_centi_A": standard_last_dc_centi_A,
        "standard_last_id_A": (standard_last_id_internal / amp_scale) if standard_last_id_internal is not None else None,
        "standard_last_iq_A": (standard_last_iq_internal / amp_scale) if standard_last_iq_internal is not None else None,
        "standard_last_dc_A": (standard_last_dc_centi_A / 100.0) if standard_last_dc_centi_A is not None else None,
        "commissioning_fault_snapshot": commissioning_fault_snapshot,
        "route_packets_this": route_packets_this,
        "route_packets_other": route_packets_other,
        "set_packets_this": set_packets_this,
        "last_set_command": last_set_command,
        "last_set_host_raw": last_set_host_raw,
        "last_set_normalized": last_set_normalized,
        "last_set_run": last_set_run,
        "bridge_warmup_this": bridge_warmup_right if side else bridge_warmup_left,
        "bridge_transition_this": bridge_transition_right if side else bridge_transition_left,
        "cal_sweep_direction": cal_sweep_direction,
        "cal_forward_cycles": cal_forward_cycles,
        "cal_reverse_cycles": cal_reverse_cycles,
        "cal_completed_cycles": cal_completed_cycles,
        "cal_motion_detected": cal_motion_detected,
        "cal_motion_counter": cal_motion_counter,
        "cal_motion_age_ms": cal_motion_age_ms,
        "cal_observed_hall_mask": cal_observed_hall_mask,
        "cal_observed_encoder_mask": cal_observed_encoder_mask,
        "cal_encoder_delta": cal_encoder_delta,
        "standard_pending_samples": standard_pending_samples,
        "standard_pending_dc_samples": standard_pending_dc_samples,
        "commissioning_voltage_limit_internal": commissioning_voltage_limit_internal,
        "commissioning_vd_saturated": commissioning_vd_saturated,
        "commissioning_vq_saturated": commissioning_vq_saturated,
        "wire_values_reply_count": wire_values_reply_count,
        "wire_fault_last": wire_fault_last,
        "wire_fault_last_name": FAULT_NAMES.get(wire_fault_last, str(wire_fault_last)) if wire_fault_last not in (None,255) else None,
        "wire_values_command": wire_values_command,
        "wire_values_payload_len": wire_values_payload_len,
        "tx_queue_used": tx_queue_used,
        "pending_detect_owner": pending_detect_owner,
        "pending_detect_reply_retries": pending_detect_reply_retries,
        "terminal_detect_valid": terminal_detect_valid,
        "terminal_detect_state": terminal_detect_state,
        "terminal_detect_state_name": CAL_NAMES.get(terminal_detect_state, str(terminal_detect_state)) if terminal_detect_state is not None else None,
        "terminal_detect_result_code": terminal_detect_result_code,
        "terminal_detect_sensor_type": terminal_detect_sensor_type,
        "terminal_encoder_ratio_fallback": terminal_encoder_ratio_fallback,
        "terminal_detect_pole_pairs": terminal_detect_pole_pairs,
        "terminal_detect_encoder_inverted": terminal_detect_encoder_inverted,
        "terminal_detect_encoder_cpr": terminal_detect_encoder_cpr,
        "cal_encoder_forward_delta": cal_encoder_forward_delta,
        "cal_encoder_direction_normal_score": cal_encoder_direction_normal_score,
        "cal_encoder_direction_inverted_score": cal_encoder_direction_inverted_score,
        "cal_encoder_direction_proved": cal_encoder_direction_proved,
        "encoder_ratio": encoder_ratio,
        "encoder_electrical_ready": encoder_electrical_ready,
        "steering_calibrated": steering_calibrated,
        "steering_homed": steering_homed,
        "steering_zero_ticks": steering_zero_ticks,
        "steering_span_ticks": steering_span_ticks,
        "logical_position_deg": logical_position_deg,
        "position_target_ticks": position_target_ticks,
        "duty_target_q15": duty_target_q15,
        "homing_on_boot": homing_on_boot,
        "homing_active_this": homing_active_this,
        "rotor_display_mode": rotor_display_mode,
        "auto_detect_active": auto_detect_active,
        "auto_detect_stage": auto_detect_stage,
        "auto_detect_result": auto_detect_result,
        "auto_detect_reply_retries": auto_detect_reply_retries,
        "alignment_probe_delta": alignment_probe_delta,
        "alignment_direction_proved": alignment_direction_proved,
        "position_session_zero_ticks": position_session_zero_ticks,
        "observer_valid": observer_valid,
        **current_flat,
        "payload_len": len(payload),
    }


def decode_hall_detect(payload: bytes) -> dict[str, Any]:
    if len(payload) != 10 or payload[0] != COMM_DETECT_HALL_FOC:
        raise ValueError(f"invalid Hall detect reply len={len(payload)}")
    table = list(payload[1:9]); status = payload[9]
    valid = [v for v in table if v != 255]
    return {"table": table, "status": status, "valid_count": len(valid), "success": status == 0 and len(valid) == 6}


def decode_encoder_detect(payload: bytes) -> dict[str, Any]:
    if len(payload) < 10 or payload[0] != COMM_DETECT_ENCODER:
        raise ValueError("invalid encoder detect reply")
    p = 1
    off, p = be_i32(payload, p); ratio, p = be_i32(payload, p)
    inverted = payload[p]
    return {
        "offset_deg": off / 1_000_000.0,
        "pole_pairs": ratio / 1_000_000.0,
        "inverted": bool(inverted),
        "success": abs(off / 1_000_000.0 - 1001.0) > 0.1 and ratio > 0,
    }


@dataclasses.dataclass
class TestResult:
    name: str
    status: str
    started: str
    duration_s: float
    details: Any = None
    error: Optional[str] = None


class BundleLogger:
    def __init__(self, root: Path) -> None:
        self.root = root
        root.mkdir(parents=True, exist_ok=True)
        self.session = (root / "session.log").open("w", encoding="utf-8", buffering=1)
        self.raw = (root / "raw_packets.log").open("w", encoding="utf-8", buffering=1)
        self.telemetry_file = (root / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self.diag_file = (root / "diagnostics.csv").open("w", newline="", encoding="utf-8")
        self.command_file = (root / "set_command_trace.csv").open("w", newline="", encoding="utf-8")
        self.detect_file = (root / "detect_transaction_trace.csv").open("w", newline="", encoding="utf-8")
        self.rotor_file = (root / "rotor_position_stream.csv").open("w", newline="", encoding="utf-8")
        self.homing_file = (root / "homing_state_trace.csv").open("w", newline="", encoding="utf-8")
        self.standard_wire_file = (root / "vesc_standard_wire.jsonl").open("w", encoding="utf-8", buffering=1)
        self.telemetry_writer: Optional[csv.DictWriter] = None
        self.diag_writer: Optional[csv.DictWriter] = None
        self.command_writer: Optional[csv.DictWriter] = csv.DictWriter(
            self.command_file, fieldnames=COMMAND_TRACE_FIELDS, extrasaction="ignore")
        self.command_writer.writeheader(); self.command_file.flush()
        self.detect_writer: csv.DictWriter = csv.DictWriter(
            self.detect_file, fieldnames=DETECT_TRACE_FIELDS, extrasaction="ignore")
        self.detect_writer.writeheader(); self.detect_file.flush()
        self.rotor_writer = csv.DictWriter(self.rotor_file,
            fieldnames=["timestamp","node","mode","label","position_deg"])
        self.rotor_writer.writeheader(); self.rotor_file.flush()
        self.homing_writer = csv.DictWriter(self.homing_file,
            fieldnames=["timestamp","node","homing_state","homing_state_name","active",
                        "encoder_electrical_ready","steering_calibrated","steering_homed",
                        "logical_position_deg","steering_zero_ticks","steering_span_ticks",
                        "iq_A","erpm","internal_error","vesc_fault"])
        self.homing_writer.writeheader(); self.homing_file.flush()
        self.results: list[TestResult] = []
        self.fault_events: list[dict[str, Any]] = []

    def log(self, level: str, msg: str) -> None:
        line = f"{now_iso()} [{level:<5}] {msg}"
        print(line)
        self.session.write(line + "\n")

    def packet(self, direction: str, node: str, payload: bytes, frame: bytes = b"") -> None:
        self.raw.write(f"{now_iso()} {direction:<2} node={node:<6} payload_len={len(payload):4d} payload={payload.hex()} frame={frame.hex()}\n")

    def telemetry(self, node: str, label: str, values: dict[str, Any]) -> None:
        row = {"timestamp": now_iso(), "node": node, "label": label, **values}
        # Flatten list fields to JSON strings.
        row = {k: (json.dumps(v) if isinstance(v, (list, dict)) else v) for k, v in row.items()}
        if self.telemetry_writer is None:
            self.telemetry_writer = csv.DictWriter(self.telemetry_file, fieldnames=list(row.keys()), extrasaction="ignore")
            self.telemetry_writer.writeheader()
        self.telemetry_writer.writerow(row); self.telemetry_file.flush()

    def diag(self, node: str, label: str, values: dict[str, Any]) -> None:
        row = {"timestamp": now_iso(), "node": node, "label": label, **values}
        if self.diag_writer is None:
            self.diag_writer = csv.DictWriter(self.diag_file, fieldnames=list(row.keys()), extrasaction="ignore")
            self.diag_writer.writeheader()
        self.diag_writer.writerow(row); self.diag_file.flush()

    def command_trace(self, row: dict[str, Any]) -> None:
        flat = {k: (json.dumps(v, default=safe_json) if isinstance(v, (list, dict)) else v) for k, v in row.items()}
        assert self.command_writer is not None
        self.command_writer.writerow(flat); self.command_file.flush()

    def detect_trace(self, node: str, label: str, values: dict[str, Any]) -> None:
        row = {"timestamp": now_iso(), "node": node, "label": label, **values}
        self.detect_writer.writerow(row); self.detect_file.flush()

    def standard_wire(self, node: str, label: str, family: str, payload: bytes, decoded: dict[str, Any]) -> None:
        row = {"timestamp": now_iso(), "node": node, "label": label, "family": family,
               "payload_hex": payload.hex(), "decoded": decoded}
        self.standard_wire_file.write(json.dumps(row, default=safe_json, sort_keys=True) + "\n")

    def rotor_position_sample(self, node: str, mode: int, label: str, position_deg: float) -> None:
        self.rotor_writer.writerow({"timestamp": now_iso(), "node": node, "mode": mode,
                                    "label": label, "position_deg": position_deg})
        self.rotor_file.flush()

    def homing_trace(self, node: str, d: dict[str, Any]) -> None:
        state = int(d.get("homing_state") or 0)
        self.homing_writer.writerow({
            "timestamp": now_iso(), "node": node, "homing_state": state,
            "homing_state_name": HOMING_STATE_NAMES.get(state, str(state)),
            "active": d.get("homing_active_this"),
            "encoder_electrical_ready": d.get("encoder_electrical_ready"),
            "steering_calibrated": d.get("steering_calibrated"),
            "steering_homed": d.get("steering_homed"),
            "logical_position_deg": d.get("logical_position_deg"),
            "steering_zero_ticks": d.get("steering_zero_ticks"),
            "steering_span_ticks": d.get("steering_span_ticks"),
            "iq_A": d.get("iq_A"), "erpm": d.get("erpm"),
            "internal_error": d.get("internal_error_name"), "vesc_fault": d.get("vesc_fault_name"),
        })
        self.homing_file.flush()

    def add_result(self, r: TestResult) -> None:
        self.results.append(r)
        self.log(r.status, f"{r.name}: {r.error or r.details or ''}")

    def close(self) -> None:
        self.session.close(); self.raw.close(); self.telemetry_file.close(); self.diag_file.close(); self.command_file.close(); self.detect_file.close(); self.rotor_file.close(); self.homing_file.close(); self.standard_wire_file.close()


class SerialVesc:
    def __init__(self, port: str, baud: int, timeout: float, log: BundleLogger) -> None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError("pyserial belum terpasang. Jalankan: python -m pip install pyserial") from exc
        self.serial_mod = serial
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.02, write_timeout=1.0)
        self.timeout = timeout
        self.log = log
        self.parser = StreamParser()
        self.local_id: Optional[int] = None
        self.right_id: Optional[int] = None

    def close(self) -> None:
        self.ser.close()

    def _node_payload(self, payload: bytes, node: str) -> tuple[bytes, str]:
        if node == "local":
            return payload, "local"
        if node == "right":
            if self.right_id is None:
                raise RuntimeError("right virtual CAN ID unknown")
            return bytes((COMM_FORWARD_CAN, self.right_id)) + payload, f"can{self.right_id}"
        raise ValueError(f"unknown node {node}")

    def send_only(self, payload: bytes, node: str = "local") -> None:
        wire_payload, n = self._node_payload(payload, node)
        f = encode_frame(wire_payload)
        self.log.packet("TX", n, wire_payload, f)
        self.ser.write(f); self.ser.flush()

    def read_payload(self, timeout: Optional[float] = None, node: str = "?") -> bytes:
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            b = self.ser.read(1)
            if not b:
                continue
            p = self.parser.feed(b[0])
            if p is not None:
                self.log.packet("RX", node, p)
                return p
        raise TimeoutError(f"VESC response timeout after {timeout or self.timeout:.1f}s")

    def transact(self, payload: bytes, node: str = "local", timeout: Optional[float] = None, reset_rx: bool = True) -> bytes:
        if reset_rx:
            self.ser.reset_input_buffer(); self.parser.reset()
        self.send_only(payload, node=node)
        return self.read_payload(timeout=timeout, node=node)

    def fw(self, node: str) -> dict[str, Any]:
        return decode_fw(self.transact(bytes((COMM_FW_VERSION,)), node=node))

    def values(self, node: str, label: str = "sample") -> dict[str, Any]:
        payload = self.transact(bytes((COMM_GET_VALUES,)), node=node)
        v = decode_values(payload)
        self.log.telemetry(node, label, v)
        self.log.standard_wire(node, label, "GET_VALUES", payload, v)
        return v

    def values_selective(self, node: str, mask: int, label: str = "values_selective") -> dict[str, Any]:
        req = bytes((COMM_GET_VALUES_SELECTIVE,)) + struct.pack(">I", mask & 0xFFFFFFFF)
        payload = self.transact(req, node=node)
        v = decode_values_selective(payload, mask)
        self.log.telemetry(node, label, v)
        self.log.standard_wire(node, label, "GET_VALUES_SELECTIVE", payload, v)
        return v

    def setup_values(self, node: str, *, mask: Optional[int] = None, label: str = "setup") -> dict[str, Any]:
        if mask is None:
            req = bytes((COMM_GET_VALUES_SETUP,))
            payload = self.transact(req, node=node)
            v = decode_setup_values(payload)
            family = "GET_VALUES_SETUP"
        else:
            req = bytes((COMM_GET_VALUES_SETUP_SELECTIVE,)) + struct.pack(">I", mask & 0xFFFFFFFF)
            payload = self.transact(req, node=node)
            v = decode_setup_values(payload, mask)
            family = "GET_VALUES_SETUP_SELECTIVE"
        self.log.standard_wire(node, label, family, payload, v)
        return v

    def rotor_position(self, node: str) -> float:
        return decode_rotor_position(self.transact(bytes((COMM_ROTOR_POSITION,)), node=node))

    def set_rotor_display(self, node: str, mode: int) -> None:
        if mode < 0 or mode > 7:
            raise ValueError(f"invalid rotor display mode {mode}")
        # COMM_SET_DETECT is a selector, not a request/reply transaction. The
        # firmware then streams COMM_ROTOR_POSITION at ~100 Hz until mode 0.
        self.send_only(bytes((COMM_SET_DETECT, mode & 0xFF)), node=node)

    def collect_rotor_positions(self, node: str, mode: int, seconds: float = 0.18) -> list[float]:
        self.ser.reset_input_buffer(); self.parser.reset()
        values: list[float] = []
        try:
            self.set_rotor_display(node, mode)
            deadline = time.monotonic() + max(0.05, seconds)
            while time.monotonic() < deadline:
                try:
                    payload = self.read_payload(timeout=min(0.05, max(0.01, deadline-time.monotonic())), node=node)
                except TimeoutError:
                    continue
                if payload and payload[0] == COMM_ROTOR_POSITION:
                    pos = decode_rotor_position(payload)
                    if math.isfinite(pos):
                        values.append(pos)
                        self.log.rotor_position_sample(node, mode, f"mode_{mode}", pos)
        finally:
            # Stop the periodic stream without resetting RX; a few queued frames are
            # harmless and are discarded by the next normal transaction.
            try:
                self.set_rotor_display(node, 0)
            except Exception:
                pass
            time.sleep(0.03)
        return values

    def _expected_node_id(self, node: str) -> int:
        if node == "local":
            if self.local_id is None:
                raise RuntimeError("local controller ID unknown")
            return int(self.local_id)
        if node == "right":
            if self.right_id is None:
                raise RuntimeError("right virtual CAN ID unknown")
            return int(self.right_id)
        raise ValueError(f"unknown node {node}")

    def diag(self, node: str, label: str = "diag") -> dict[str, Any]:
        """Read one HBTS diagnostic frame from the requested motor only.

        Virtual-CAN forwarding and detect-progress polling can leave a delayed HBTS
        response in the UART queue. V7 accepted the first frame blindly; that made a
        stale LEFT frame look like RIGHT Hall=000. V8 validates the embedded node_id
        and discards every stale/mismatched diagnostic until the requested node replies.
        """
        req = bytes((COMM_CUSTOM_APP_DATA,)) + HBTS_MAGIC + bytes((HBTS_VERSION, HBTS_GET_DIAG))
        expected = self._expected_node_id(node)
        self.ser.reset_input_buffer()
        self.parser.reset()
        self.send_only(req, node=node)
        deadline = time.monotonic() + self.timeout
        stale = 0
        last_error: Optional[str] = None
        while time.monotonic() < deadline:
            remaining = max(0.01, min(0.10, deadline - time.monotonic()))
            try:
                payload = self.read_payload(timeout=remaining, node=node)
            except TimeoutError:
                continue
            try:
                d = decode_diag(payload)
            except Exception as exc:
                stale += 1
                last_error = str(exc)
                self.log.log("WARN", f"{node} {label}: discard non-HBTS/stale frame: {exc}")
                continue
            got = int(d.get("node_id", -1))
            if got != expected:
                stale += 1
                self.log.log("WARN", f"{node} {label}: discard HBTS for node_id={got}, expected={expected}")
                continue
            if stale:
                d["stale_frames_discarded"] = stale
            self.log.diag(node, label, d)
            return d
        extra = f"; last decode error={last_error}" if last_error else ""
        raise TimeoutError(f"HBTS {node} node_id={expected} timeout; stale_discarded={stale}{extra}")

    def current_recalibrate(self) -> dict[str, Any]:
        req = bytes((COMM_CUSTOM_APP_DATA,)) + HBTS_MAGIC + bytes((HBTS_VERSION, HBTS_CURRENT_RECAL))
        p = self.transact(req, node="local", timeout=3.0)
        if len(p) < 9 or p[0] != COMM_CUSTOM_APP_DATA or p[1:5] != HBTS_MAGIC or p[5] != HBTS_VERSION or p[6] != HBTS_CURRENT_RECAL_REPLY:
            raise ValueError(f"bad current recal reply: {p.hex()}")
        return {"accepted": bool(p[7]), "state": p[8], "state_name": CURRENT_CAL_NAMES.get(p[8], str(p[8]))}

    def adc(self) -> dict[str, Any]:
        return decode_adc(self.transact(bytes((COMM_GET_DECODED_ADC,)), node="local"))

    def ping_can(self) -> list[int]:
        p = self.transact(bytes((COMM_PING_CAN,)), node="local")
        if not p or p[0] != COMM_PING_CAN:
            raise ValueError("invalid PING_CAN reply")
        return list(p[1:])

    def terminal(self, command: str) -> str:
        p = self.transact(bytes((COMM_TERMINAL_CMD,)) + command.encode("ascii"), node="local", timeout=3.0)
        if not p or p[0] != COMM_PRINT:
            raise ValueError(f"bad TERMINAL response: {p[:8].hex() if p else 'empty'}")
        return p[1:].decode("utf-8", errors="replace")

    def set_duty(self, node: str, duty: float) -> None:
        self.send_only(bytes((COMM_SET_DUTY,)) + pack_i32(round(duty * 100000)), node=node)

    def set_current(self, node: str, amps: float) -> None:
        self.send_only(bytes((COMM_SET_CURRENT,)) + pack_i32(round(amps * 1000)), node=node)

    def set_current_brake(self, node: str, amps: float) -> None:
        self.send_only(bytes((COMM_SET_CURRENT_BRAKE,)) + pack_i32(round(amps * 1000)), node=node)

    def set_handbrake(self, node: str, amps: float) -> None:
        self.send_only(bytes((COMM_SET_HANDBRAKE,)) + pack_i32(round(amps * 1000)), node=node)

    def set_current_rel(self, node: str, rel: float) -> None:
        self.send_only(bytes((COMM_SET_CURRENT_REL,)) + pack_i32(round(rel * 100000)), node=node)

    def set_rpm(self, node: str, erpm: int) -> None:
        self.send_only(bytes((COMM_SET_RPM,)) + pack_i32(erpm), node=node)

    def set_pos(self, node: str, deg: float) -> None:
        self.send_only(bytes((COMM_SET_POS,)) + pack_i32(round(deg * 1_000_000)), node=node)

    def stop(self, node: str) -> None:
        # Current=0 explicitly disarms in this port.
        self.set_current(node, 0.0)


class TestSuite:
    def __init__(self, args: argparse.Namespace, log: BundleLogger) -> None:
        self.args = args
        self.log = log
        self.dev: Optional[SerialVesc] = None
        self.first_fault_time: dict[str, float] = {}
        self.original_appconf: Optional[bytes] = None

    def result(self, name: str, fn: Callable[[], Any], *, skip: bool = False, skip_reason: str = "") -> Any:
        start = time.monotonic(); started = now_iso()
        if skip:
            r = TestResult(name, "SKIP", started, 0.0, details=skip_reason)
            self.log.add_result(r); return None
        try:
            value = fn()
            r = TestResult(name, "PASS", started, time.monotonic() - start, details=value)
            self.log.add_result(r); return value
        except SkipTest as exc:
            r = TestResult(name, "SKIP", started, time.monotonic() - start, details=str(exc))
            self.log.add_result(r); return None
        except AssertionError as exc:
            r = TestResult(name, "FAIL", started, time.monotonic() - start, error=str(exc))
            self.log.add_result(r); return None
        except Exception as exc:
            tb = traceback.format_exc()
            (self.log.root / "exceptions.log").open("a", encoding="utf-8").write(f"\n=== {name} {now_iso()} ===\n{tb}\n")
            r = TestResult(name, "FAIL", started, time.monotonic() - start, error=f"{type(exc).__name__}: {exc}")
            self.log.add_result(r); return None

    def warn(self, name: str, details: Any) -> None:
        self.log.add_result(TestResult(name, "WARN", now_iso(), 0.0, details=details))

    def self_test(self) -> dict[str, Any]:
        payloads = [bytes((COMM_FW_VERSION,)), bytes(range(1, 100)), bytes((i * 37) & 0xFF for i in range(300))]
        for p in payloads:
            fr = encode_frame(p); parser = StreamParser(); got = None
            for b in fr:
                x = parser.feed(b)
                if x is not None: got = x
            assert got == p, f"parser roundtrip failed len={len(p)}"
        # CRC rejection followed by valid-frame recovery.
        good = encode_frame(b"abc")
        bad = bytearray(good); bad[-3] ^= 0x01
        parser = StreamParser(); out = []
        for b in bytes(bad) + good:
            x = parser.feed(b)
            if x is not None: out.append(x)
        assert out == [b"abc"], f"CRC recovery failed: {out}"
        assert crc16(b"123456789") == 0x31C3, "CRC16 reference mismatch"

        # V9 regression for the V8 inventory circular-dependency bug. Inventory
        # resolution must work from standard VESC data before any HBTS request.
        assert resolve_inventory_ids(10, [11]) == (10, 11, "COMM_GET_VALUES")
        assert resolve_inventory_ids(None, [11]) == (10, 11, "derived_from_single_virtual_CAN")
        assert resolve_inventory_ids(253, [254]) == (253, 254, "COMM_GET_VALUES")
        assert resolve_inventory_ids(254, [0]) == (254, 0, "COMM_GET_VALUES")
        try:
            resolve_inventory_ids(None, [0])
        except RuntimeError:
            pass
        else:
            raise AssertionError("ambiguous virtual CAN ID 0 fallback must fail")

        # Synthetic HBTS diagnostic layout regression. This detects accidental
        # Python/C field-order drift before the script is used on hardware.
        d = bytearray((COMM_CUSTOM_APP_DATA,)) + bytearray(HBTS_MAGIC) + bytearray((HBTS_VERSION, HBTS_DIAG_REPLY, 10, 0))
        def pu16(v: int) -> None: d.extend(struct.pack(">H", v & 0xFFFF))
        def pi16(v: int) -> None: d.extend(struct.pack(">h", v))
        def pu32(v: int) -> None: d.extend(struct.pack(">I", v & 0xFFFFFFFF))
        def pi32(v: int) -> None: d.extend(struct.pack(">i", v))
        pu16((1 << 2) | (1 << 4) | (1 << 6))  # link, calibrated, EEPROM verified
        d.extend((0, 0, 0, 0, 0, 0b010, 0, 15))  # sensor..pole_pairs
        pu16(800); pu16(0); pu16(12)
        pi32(1234); pi16(1440); pu16(0x4567); pi16(160); pi32(10)
        pi16(12); pi16(-4); pu16(5); pi16(123); pi16(4800); pu16(800)
        pi32(250); pi16(40); pu16(3); pu32(2100); pu32(2500); pu32(4000); d.extend((0, 0))
        pu32(100); pu32(2); pu32(9); pu32(0); pu32(0)
        pu16(5); pu16(0); pu16(2048); pu16(1024)
        pi32(0); pi32(0)
        d.extend((2, 1, 0, 0, 3, 0, 0))
        pu32(1000)
        # Current-cal V7 tail: calibration evidence, validated/raw DC-link data,
        # and forced-phase current-detect target/measured values.
        d.extend((3, 1, 1, 1)); pu16(2048); pu16(2048); pu16(0); pu16(4)
        for v in (2010,2020,2030,2040,2050,2060): pu16(v)
        for v in (2011,2021,2031,2041,2051,2061): pu16(v)
        for v in (2011,2021,2031,2041,2051,2061): pu16(v)
        for v in (80,322,53,55,33,24): pu16(v)
        for v in (3,4,2,3,2,2): pu16(v)
        for v in (1,1,1,1,1,1): pi16(v)
        pi16(4); pi16(12); d.append(0)
        # V7 validated DC=0.02 A, raw DC=0.06 A, 3 rejects; detect target=1 A.
        pi16(2); pi16(6); pu16(3); pi16(800); pi16(760); pu16(0x2222); pi16(800); pi16(0)
        # V8 current-loop evidence (HBTS v6): Vd/Vq/mod, reject count, guard mask, PI gains.
        pi16(120); pi16(-40); pi16(220); pi16(-80); pu16(7); d.append(0); pi32(3277); pi32(82)
        # V8 buzzer evidence (HBTS v7): current reason, last reason, event count.
        d.extend((0, 1)); pu16(2)
        # V10 HBTS v8: per-node standard-current provenance + preserved first
        # commissioning guard trip.
        d.extend((0, 0)); pu16(9); pu16(25); pi16(0); pi16(0); pi16(0)
        d.extend((1, 1, 8, 1)); pu16(3)
        pi16(400); pi16(2600); pi16(-1800); pi16(165); pi16(-142); pi16(4)
        pu16(2628); pu16(2863); pu16(1930); pu16(2793); pu16(2721); pu16(1934)
        pu16(0x3456); pi16(100); pi16(200); pi16(300); pu16(600)
        pu32(123); pu32(77); pu32(9); d.append(COMM_SET_CURRENT)
        # V12 HBTS v10: active-zero current-cal domain, bridge warm-up counters,
        # and exact SET wire/runtime provenance.
        d.extend((1, 0, 0, 0)); pu16(12); pu16(14); pi32(500); pi32(25); d.append(1)
        for v in (2012,2022,2032,2042,2052,2062): pu16(v)
        for v in (-1,-1,-1,-1,-1,-1): pi16(v)
        # V12 HBTS v11 append-only tail: bidirectional commissioning progress,
        # live accumulator validity and exact standard VESC wire provenance.
        d.append(1)          # sweep direction +
        d.append(3)          # forward cycles
        d.append(2)          # reverse cycles
        pu16(5)              # completed cycles
        d.append(1)          # motion detected
        pu32(321)            # raw motion counter
        pu16(17)             # age of last motion
        d.append(0b01111110) # Hall states 1..6 observed
        d.append(0b00001111) # encoder AB states 0..3 observed
        pi32(456)            # encoder delta
        pu16(13)             # pending Id/Iq average samples
        pu16(11)             # pending DC samples
        pu16(2400)           # commissioning voltage cap
        d.append(1)          # Vd at cap
        d.append(0)          # Vq not at cap
        pu32(77)             # standard-values replies emitted
        d.append(0)          # exact last standard fault byte = NONE
        d.append(COMM_GET_VALUES)
        pu16(73)
        # V14 HBTS v12: UART transaction + per-motor terminal detect snapshot.
        d.append(2)          # TX ring entries currently used
        d.append(2)          # pending detect owner RIGHT
        pu16(4)              # terminal reply queue retries
        d.append(1)          # terminal snapshot valid
        d.append(2)          # ESC_SENSOR_CAL_SUCCESS
        d.append(0)          # result code
        d.append(1)          # encoder sensor type
        d.append(1)          # configured-ratio fallback was used
        d.append(15)         # pole pairs
        d.append(1)          # encoder inverted
        pu16(2048)           # encoder CPR
        # V17 HBTS v13: encoder direction proof from commanded 3 forward + 3 reverse sweeps.
        pi32(-7)             # forward displacement before configured inversion
        pu32(12)             # normal-direction transition score
        pu32(37)             # inverted-direction transition score
        d.append(1)          # direction proof decisive
        # V20 HBTS v15: encoder ratio + electrical sync + mechanical 0..360 +
        # duty target + rotor stream + integrated auto-detect transaction.
        d.append(15)         # encoder ratio independent of physical poles
        d.append(1)          # electrical ready
        d.append(1)          # steering calibrated
        d.append(1)          # steering homed this boot
        pi32(1234)           # right/zero tick
        pi32(4096)           # signed 0..360 span
        pi32(180000)         # logical position 180 deg in milli-deg
        pi32(3282)           # current position target ticks
        pi16(16384)          # 50% duty target q15
        d.append(1)          # homing-on-boot
        d.append(0)          # homing not active
        d.append(3)          # rotor display = Encoder
        d.append(1)          # integrated auto-detect active
        d.append(4)          # AUTO_DETECT_LEFT_SYNC
        pi16(2)              # result
        pu16(5)              # reply retries
        pi16(-23)            # V20 alignment raw TIM4 probe delta
        d.append(1)          # alignment direction proved
        pi32(777)            # session zero ticks
        d.append(0)          # no real flux observer in this port
        parsed = decode_diag(bytes(d))
        assert parsed["node_id"] == 10 and parsed["sensor_name"] == "HALL_UVW"
        assert parsed["calibrated"] and parsed["eeprom_verified"] and not parsed["armed"]
        assert parsed["pole_pairs"] == 15 and parsed["encoder_cpr"] == 800
        assert parsed["isr_last_cycles"] == 2100 and parsed["isr_deadline_cycles"] == 4000
        assert parsed["current_cal"]["valid"] and parsed["battery_current_total_A"] == 0.04
        assert parsed["current_cal"]["raw_span"]["rlB"] == 322
        assert parsed["current_cal"]["block_span"]["rlB"] == 4
        assert parsed["dc_input_validated_A"] == 0.02 and parsed["dc_raw_A"] == 0.06
        assert parsed["detect_target_internal"] == 800 and parsed["detect_measured_internal"] == 760
        assert parsed["foc_vd_internal"] == 120 and parsed["foc_current_kp_q16"] == 3277
        assert parsed["buzzer_reason_name"] == "NONE" and parsed["buzzer_last_reason_name"] == "HARD_RUNTIME_FAULT"
        assert parsed["buzzer_event_count"] == 2
        assert parsed["standard_bridge_active"] is False and parsed["standard_current_valid"] is False
        assert parsed["standard_current_generation"] == 9 and parsed["standard_current_age_ms"] == 25
        fs = parsed["commissioning_fault_snapshot"]
        assert fs["valid"] and fs["left"] and fs["target_internal"] == 400
        assert fs["adc_phase_1_raw"] == 2628 and fs["offset_phase_1"] == 2793
        assert parsed["route_packets_this"] == 123 and parsed["route_packets_other"] == 77
        assert parsed["set_packets_this"] == 9 and parsed["last_set_command"] == COMM_SET_CURRENT
        assert parsed["current_cal_sampling_mode_name"] == "ACTIVE_LOW_FET_ZERO_VECTOR"
        assert parsed["bridge_transition_left"] == 12 and parsed["bridge_transition_right"] == 14
        assert parsed["last_set_host_raw"] == 500 and parsed["last_set_normalized"] == 25 and parsed["last_set_run"]
        assert parsed["current_cal"]["final_active_raw"]["rlA"] == 2012
        assert parsed["current_cal"]["final_active_residual"]["dcr"] == -1
        assert parsed["cal_sweep_direction"] == 1 and parsed["cal_forward_cycles"] == 3
        assert parsed["cal_reverse_cycles"] == 2 and parsed["cal_completed_cycles"] == 5
        assert parsed["cal_motion_detected"] and parsed["cal_motion_counter"] == 321
        assert parsed["cal_observed_hall_mask"] == 0x7e and parsed["cal_observed_encoder_mask"] == 0x0f
        assert parsed["cal_encoder_delta"] == 456
        assert parsed["standard_pending_samples"] == 13 and parsed["standard_pending_dc_samples"] == 11
        assert parsed["commissioning_voltage_limit_internal"] == 2400
        assert parsed["commissioning_vd_saturated"] and not parsed["commissioning_vq_saturated"]
        assert parsed["wire_values_reply_count"] == 77 and parsed["wire_fault_last"] == 0
        assert parsed["wire_values_command"] == COMM_GET_VALUES and parsed["wire_values_payload_len"] == 73
        assert parsed["tx_queue_used"] == 2 and parsed["pending_detect_owner"] == 2
        assert parsed["pending_detect_reply_retries"] == 4
        assert parsed["terminal_detect_valid"] and parsed["terminal_detect_state"] == 2
        assert parsed["terminal_encoder_ratio_fallback"] and parsed["terminal_detect_pole_pairs"] == 15
        assert parsed["terminal_detect_encoder_inverted"] and parsed["terminal_detect_encoder_cpr"] == 2048
        assert parsed["cal_encoder_forward_delta"] == -7
        assert parsed["cal_encoder_direction_normal_score"] == 12
        assert parsed["cal_encoder_direction_inverted_score"] == 37
        assert parsed["cal_encoder_direction_proved"] is True
        assert parsed["encoder_ratio"] == 15 and parsed["encoder_electrical_ready"]
        assert parsed["steering_calibrated"] and parsed["steering_homed"]
        assert parsed["steering_zero_ticks"] == 1234 and parsed["steering_span_ticks"] == 4096
        assert abs(parsed["logical_position_deg"] - 180.0) < 1e-6
        assert parsed["position_target_ticks"] == 3282 and parsed["duty_target_q15"] == 16384
        assert parsed["homing_on_boot"] and not parsed["homing_active_this"]
        assert parsed["rotor_display_mode"] == 3 and parsed["auto_detect_active"]
        assert parsed["auto_detect_stage"] == 4 and parsed["auto_detect_result"] == 2
        assert parsed["auto_detect_reply_retries"] == 5
        assert parsed["alignment_probe_delta"] == -23 and parsed["alignment_direction_proved"]
        assert parsed["position_session_zero_ticks"] == 777 and not parsed["observer_valid"]
        return {"roundtrip_lengths": [len(p) for p in payloads], "crc_123456789": "0x31C3",
                "hbts_decode_len": parsed["payload_len"]}

    def open_device(self) -> dict[str, Any]:
        self.dev = SerialVesc(self.args.port, self.args.baud, self.args.timeout, self.log)
        return {"port": self.args.port, "baud": self.args.baud}

    def connection_inventory(self) -> dict[str, Any]:
        assert self.dev

        # Bootstrap inventory MUST NOT depend on HBTS. V8 accidentally called
        # diag("local") before local_id was known, while diag() correctly
        # requires an expected node_id to reject stale virtual-CAN frames. That
        # circular dependency made the tester fail even though VESC Tool and the
        # standard VESC protocol were healthy.
        # VESC Tool commonly retries its initial inventory packets. Do the same
        # here so one boot-time UART scheduling miss is logged rather than being
        # confused with a permanent protocol failure. V19 also had a real
        # COMM_FW_VERSION stack overflow, so each retry resets parser/RX state.
        inv_exc: Optional[Exception] = None
        lf = None
        lv = None
        for attempt in range(1, 4):
            try:
                lf = self.dev.fw("local")
                lv = self.dev.values("local", "inventory_values" if attempt == 1 else f"inventory_values_retry{attempt}")
                break
            except TimeoutError as exc:
                inv_exc = exc
                self.log.log("WARN", f"inventory bootstrap attempt {attempt}/3 timed out")
                self.dev.ser.reset_input_buffer()
                self.dev.parser.reset()
                time.sleep(0.12)
        if lf is None or lv is None:
            if inv_exc is not None:
                raise inv_exc
            raise TimeoutError("inventory bootstrap failed without response")
        local_id = lv.get("controller_id")

        # Normal/non-selective COMM_GET_VALUES from this firmware carries the
        # VESC controller ID. Keep a HBTS-free fallback based on the single
        # virtual CAN node for compatibility with older captures/configs where
        # controller_id might be absent from a shortened values payload.
        ids = self.dev.ping_can()
        resolved_local, resolved_right, id_source = resolve_inventory_ids(local_id, ids)
        self.dev.local_id = resolved_local
        self.dev.right_id = resolved_right

        # Only after IDs are established do we allow forwarded requests and HBTS
        # node validation. This is intentionally standard-VESC-only bootstrap.
        rf = self.dev.fw("right")
        assert lf["version"] == "6.00", f"unexpected local VESC wire version {lf['version']}"
        assert rf["version"] == "6.00", f"unexpected right VESC wire version {rf['version']}"
        assert lf["uuid"] != rf["uuid"], "local/right UUID must differ"

        # Verify HBTS only after standard inventory succeeded. If this fails it
        # is now reported as a diagnostic-version/path problem, not a fake
        # 'controller ID unknown' connection failure.
        ld = self.dev.diag("local", "inventory_diag_local")
        rd = self.dev.diag("right", "inventory_diag_right")
        assert int(ld["node_id"]) == self.dev.local_id
        assert int(rd["node_id"]) == self.dev.right_id

        return {
            "local_fw": lf, "right_fw": rf, "can_ids": ids,
            "local_id": self.dev.local_id, "right_id": self.dev.right_id,
            "local_id_source": id_source,
            "local_values_controller_id": lv.get("controller_id"),
            "local_diag_node_id": ld.get("node_id"),
            "right_diag_node_id": rd.get("node_id"),
        }

    def connect_state(self) -> dict[str, Any]:
        assert self.dev
        l = self.dev.diag("local", "after_connect")
        r = self.dev.diag("right", "after_connect")
        # Connecting itself must not be the reason to arm. APP_ADC can independently
        # request output, and that is reported explicitly as a failure for bench safety.
        assert not l["armed"], f"LEFT already armed after connect (APP={l['app_name']} adc_ctrl={l['adc_ctrl_type']})"
        assert not r["armed"], f"RIGHT already armed after connect (APP={r['app_name']} multi_esc={r['multi_esc']})"
        assert not l["output_enabled"] and not r["output_enabled"], "PWM gate enabled immediately after connect"
        return {"left": l, "right": r}

    def prepare_safe_uart_test_mode(self) -> dict[str, Any]:
        """Disarm both sides and temporarily force APP_UART + ADC_NONE.

        This is executed immediately after the untouched connect-state snapshot in
        a --full run. It prevents an existing APP_ADC throttle configuration from
        competing with deterministic bench commands. The original APPCONF is kept
        in RAM and restored from finally.
        """
        assert self.dev
        original = self.dev.transact(bytes((COMM_GET_APPCONF,)), node="local", timeout=3.0)
        assert original[0] == COMM_GET_APPCONF and len(original) == 494
        if self.original_appconf is None:
            self.original_appconf = bytes(original[1:])
            (self.log.root / "appconf_original.bin").write_bytes(self.original_appconf)
        # UART zero command temporarily owns the control source while APPCONF is changed.
        for node in ("local", "right"):
            for _ in range(2):
                self.dev.stop(node); time.sleep(0.03)
        blob = bytearray(self.original_appconf)
        blob[33] = 3  # APP_UART
        blob[90] = 0  # ADC_NONE
        ack = self.dev.transact(bytes((COMM_SET_APPCONF,)) + bytes(blob), node="local", timeout=4.0)
        assert ack == bytes((COMM_SET_APPCONF,)), "could not enter safe APP_UART test mode"
        time.sleep(0.12)
        d = self.dev.diag("local", "safe_uart_test_mode")
        assert d["app_mode"] == 3 and d["adc_ctrl_type"] == 0
        assert not d["armed"] and not d["output_enabled"]
        return d

    def current_offset_calibration(self) -> dict[str, Any]:
        assert self.dev
        before = self.dev.diag("local", "current_cal_before")
        attempts: list[dict[str, Any]] = []
        max_attempts = max(1, int(getattr(self.args, "current_cal_retries", 2)) + 1) if self.args.full else 1
        final = before

        for attempt in range(1, max_attempts + 1):
            if self.args.full:
                reply = self.dev.current_recalibrate()
                assert reply["accepted"], f"current offset recalibration rejected attempt={attempt}: {reply}"

            deadline = time.monotonic() + 3.0
            history = []
            while time.monotonic() < deadline:
                d = self.dev.diag("local", f"current_cal_wait_a{attempt}")
                history.append(d)
                cc = d.get("current_cal", {})
                if cc.get("valid") or cc.get("state") == 4:
                    final = d
                    break
                time.sleep(0.08)

            if history:
                final = history[-1]
            cc = final.get("current_cal", {})
            attempts.append({"attempt": attempt, "result": cc})
            if cc.get("valid"):
                break

            # Retry only transient block-mean instability. Structural failures
            # (mean outside ADC range / ADC self-cal / no blocks) stay hard FAIL.
            mask = int(cc.get("failure_mask") or 0)
            structural = mask & ((0x003F) | (1 << 12) | (1 << 13) | (1 << 14) | (1 << 15))
            if structural or not self.args.full or attempt >= max_attempts:
                break
            self.warn(f"04_current_offset_retry_{attempt}", {
                "failure_mask": mask,
                "failure_reasons": cc.get("failure_reasons", []),
                "raw_span": cc.get("raw_span", {}),
                "block_span": cc.get("block_span", {}),
            })
            time.sleep(0.20)

        cc = final.get("current_cal", {})
        assert final.get("current_units_per_amp") == 800, \
            f"current scale changed from stock-board 800 internal/A: {final.get('current_units_per_amp')}"
        assert abs(float(final.get("current_adc_counts_per_amp") or 0.0) - 50.0) < 0.001, \
            f"ADC current scale is not stock 50 count/A: {final.get('current_adc_counts_per_amp')}"
        assert cc.get("valid"), f"current offsets not valid after {len(attempts)} attempt(s): {cc}"
        assert cc.get("adc1_hw_cal_ok") and cc.get("adc2_hw_cal_ok"), f"STM32 ADC hardware calibration failed: {cc}"
        assert cc.get("sampling_mode") == 1, f"V12 current calibration not in active zero-vector domain: {cc}"
        final_active_residual = cc.get("final_active_residual", {})
        assert final_active_residual, f"missing final active-domain current evidence: {cc}"
        for name, val in final_active_residual.items():
            assert abs(val) <= 96, f"active-domain current residual {name}={val} ADC counts"
        # After calibration MOE is released, phase-shunt common-mode may move by
        # hundreds of counts. Only DCL/DCR instantaneous residual stays meaningful.
        for name in ("dcl", "dcr"):
            val = (cc.get("residual", {}) or {}).get(name)
            if val is not None:
                assert abs(val) <= 96, f"released DC-shunt residual {name}={val} ADC counts"
        for name, val in cc.get("block_span", {}).items():
            assert val <= 64, f"current ADC block mean unstable {name} block_span={val}"
        noisy = {name: val for name, val in cc.get("raw_span", {}).items() if val > 160}
        if noisy:
            self.warn("04_current_offset_raw_spikes", noisy)
        return {
            "before": before.get("current_cal", {}),
            "attempts": attempts,
            "final": cc,
            "battery_current_total_A": final.get("battery_current_total_A"),
            "imotor_diag_A": final.get("imotor_diag_A"),
        }

    def idle_telemetry(self) -> dict[str, Any]:
        assert self.dev
        samples: dict[str, list[dict[str, Any]]] = {"local": [], "right": []}
        for k in range(self.args.idle_samples):
            for node in ("local", "right"):
                v = self.dev.values(node, f"idle_{k}")
                # Presence/finite test for fields that previously disappeared while disarmed.
                for key in ("motor_current_A", "id_A", "iq_A", "erpm", "vin_V"):
                    assert key in v and isinstance(v[key], (int, float)) and math.isfinite(float(v[key])), f"{node} missing/nonfinite {key}"
                samples[node].append(v)
            time.sleep(self.args.sample_period)
        # V14 distinguishes a valid VESC zero from "telemetry unavailable".
        # With MOE/bridge OFF, upstream-style actuation current is physically 0 A;
        # current validity then comes from finite standard-wire fields + a valid
        # current-offset domain, not from forcing a fake non-zero value.
        idle_diag: dict[str, dict[str, Any]] = {}
        for node in ("local", "right"):
            d=self.dev.diag(node, "idle_current_validation")
            idle_diag[node]=d
            assert d.get("current_offsets_valid"), f"{node} current offsets invalid"
            assert not d.get("bridge_moe"), f"{node} unexpectedly active during idle telemetry proof"
            vals=samples[node]
            assert max(abs(x["id_A"]) for x in vals) < 2.0, f"{node} idle Id too large"
            assert max(abs(x["iq_A"]) for x in vals) < 2.0, f"{node} idle Iq too large"
            assert max(abs(x["motor_current_A"]) for x in vals) < 2.5, f"{node} idle Imotor too large"

        return {
            node: {
                "wire_status": "STANDARD_ZERO_VALID_BRIDGE_OFF",
                "bridge_moe": bool(idle_diag[node].get("bridge_moe")),
                "current_offsets_valid": bool(idle_diag[node].get("current_offsets_valid")),
                "id_range_A": [min(x["id_A"] for x in arr), max(x["id_A"] for x in arr)],
                "iq_range_A": [min(x["iq_A"] for x in arr), max(x["iq_A"] for x in arr)],
                "imotor_range_A": [min(x["motor_current_A"] for x in arr), max(x["motor_current_A"] for x in arr)],
                "input_current_range_A": [min(x["input_current_A"] for x in arr), max(x["input_current_A"] for x in arr)],
                "erpm_range": [min(x["erpm"] for x in arr), max(x["erpm"] for x in arr)],
                "faults": sorted(set(x["fault"] for x in arr)),
            } for node, arr in samples.items()
        }

    def passive_spin_current_observation(self) -> dict[str, Any]:
        """Released-bridge cross-side isolation test.

        The user rotates LEFT only, then RIGHT only. Standard VESC current must stay
        zero on BOTH nodes while both bridges are OFF. Raw HBTS current is retained
        to quantify shunt/common-mode coupling and prove which physical side moved.
        """
        assert self.dev
        duration=max(1.0,float(getattr(self.args,"passive_spin_seconds",3.0)))
        report: dict[str, Any]={}
        csv_rows: list[dict[str, Any]]=[]
        for moving in ("local","right"):
            other="right" if moving=="local" else "local"
            for node in ("local","right"):
                for _ in range(3): self.dev.stop(node); time.sleep(0.03)
            time.sleep(0.12)
            self.log.log("INFO",f"PASSIVE ISOLATION {duration:.1f}s: rotate {'LEFT' if moving=='local' else 'RIGHT'} ONLY now; do not move the other wheel.")
            end_t=time.monotonic()+duration
            arr: dict[str,list[dict[str,Any]]]={"local":[],"right":[]}
            while time.monotonic()<end_t:
                for node in ("local","right"):
                    v=self.dev.values(node,f"passive_{moving}")
                    d=self.dev.diag(node,f"passive_{moving}")
                    assert not d["armed"] and not d["bridge_moe"], f"{node} bridge active during passive isolation"
                    arr[node].append({"v":v,"d":d})
                    csv_rows.append({
                        "timestamp":now_iso(),"moving_side":moving,"observed_node":node,
                        "std_imotor_A":v["motor_current_A"],"std_id_A":v["id_A"],"std_iq_A":v["iq_A"],
                        "std_iin_A":v["input_current_A"],"raw_imotor_A":d.get("imotor_diag_A"),
                        "raw_id_A":d.get("id_A"),"raw_iq_A":d.get("iq_A"),"dc_raw_A":d.get("dc_raw_A"),
                        "dc_valid_A":d.get("dc_input_validated_A"),"position_ticks":d.get("position_ticks"),
                        "encoder_edges":d.get("encoder_valid_edges"),"hall_raw":d.get("hall_raw"),
                        "bridge_moe":d.get("bridge_moe"),"standard_current_valid":d.get("standard_current_valid"),
                    })
                time.sleep(0.06)
            stage={}
            for node,xs in arr.items():
                def peak(key,src): return max((abs(float(x[src].get(key) or 0.0)) for x in xs),default=0.0)
                stage[node]={
                    "samples":len(xs),
                    "std_peak_imotor_A":peak("motor_current_A","v"),
                    "std_peak_id_A":peak("id_A","v"),"std_peak_iq_A":peak("iq_A","v"),
                    "std_peak_input_A":peak("input_current_A","v"),
                    "raw_peak_imotor_A":peak("imotor_diag_A","d"),"raw_peak_id_A":peak("id_A","d"),
                    "raw_peak_iq_A":peak("iq_A","d"),"raw_peak_dc_A":peak("dc_raw_A","d"),
                    "position_delta": (xs[-1]["d"].get("position_ticks",0)-xs[0]["d"].get("position_ticks",0)) if len(xs)>1 else 0,
                    "encoder_edge_delta": (xs[-1]["d"].get("encoder_valid_edges",0)-xs[0]["d"].get("encoder_valid_edges",0)) if len(xs)>1 else 0,
                }
                # V10 standard telemetry is actuation current. With MOE OFF there
                # is no valid low-side phase-current observation for VESC Tool.
                assert stage[node]["std_peak_imotor_A"] <= 0.05, f"{node} passive Imotor leaked into VESC standard telemetry: {stage[node]}"
                assert stage[node]["std_peak_id_A"] <= 0.05 and stage[node]["std_peak_iq_A"] <= 0.05, \
                    f"{node} passive Id/Iq leaked into standard telemetry: {stage[node]}"
                assert stage[node]["std_peak_input_A"] <= 0.05, f"{node} passive input current nonzero: {stage[node]}"
            # Cross-side raw coupling is diagnostic, not a standard-current failure.
            if stage[other]["raw_peak_imotor_A"] > 1.0:
                self.warn(f"06b_raw_cross_coupling_{moving}_to_{other}",stage)
            report[f"rotate_{moving}"]=stage
        if csv_rows:
            with (self.log.root/"cross_side_isolation.csv").open("w",newline="",encoding="utf-8") as f:
                w=csv.DictWriter(f,fieldnames=list(csv_rows[0].keys())); w.writeheader(); w.writerows(csv_rows)
        return report

    def protocol_get_matrix(self) -> dict[str, Any]:
        """Exercise every GET implemented by the VESC facade on both motor contexts."""
        assert self.dev
        out: dict[str,Any]={}
        values_mask=(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7)|(1<<8)|(1<<15)|(1<<16)|(1<<17)|(1<<19)|(1<<20)|(1<<21)
        setup_mask=(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<7)|(1<<15)|(1<<16)|(1<<17)|(1<<18)|(1<<21)
        for node in ("local","right"):
            fw=self.dev.fw(node)
            vals=self.dev.values(node,"get_matrix_values")
            sel=self.dev.values_selective(node,values_mask,"get_matrix_values_selective")
            setup=self.dev.setup_values(node, label="get_matrix_setup")
            setup_sel=self.dev.setup_values(node, mask=setup_mask, label="get_matrix_setup_selective")
            rotor_samples=self.dev.collect_rotor_positions(node,4,seconds=0.12)
            assert rotor_samples, f"{node} PID-position rotor stream produced no COMM_ROTOR_POSITION frames"
            rotor=rotor_samples[-1]
            mc=self.dev.transact(bytes((COMM_GET_MCCONF,)),node=node,timeout=3.0)
            mcd=self.dev.transact(bytes((COMM_GET_MCCONF_DEFAULT,)),node=node,timeout=3.0)
            app=self.dev.transact(bytes((COMM_GET_APPCONF,)),node=node,timeout=3.0)
            appd=self.dev.transact(bytes((COMM_GET_APPCONF_DEFAULT,)),node=node,timeout=3.0)
            diag=self.dev.diag(node,"get_matrix_diag")
            expected=self.dev._expected_node_id(node)
            assert vals.get("controller_id")==expected, f"{node} GET_VALUES controller_id={vals.get('controller_id')} expected={expected}"
            assert sel.get("controller_id")==expected, f"{node} selective controller_id wrong: {sel}"
            assert setup.get("controller_id")==expected and setup_sel.get("controller_id")==expected
            assert int.from_bytes(mc[1:5],"big")==VESC6_MCCONF_SIGNATURE
            assert int.from_bytes(mcd[1:5],"big")==VESC6_MCCONF_SIGNATURE
            assert int.from_bytes(app[1:5],"big")==VESC6_APPCONF_SIGNATURE
            assert int.from_bytes(appd[1:5],"big")==VESC6_APPCONF_SIGNATURE
            assert diag["node_id"]==expected
            out[node]={"fw":fw,"values":vals,"values_selective":sel,"setup":setup,"setup_selective":setup_sel,
                       "rotor_position_deg":rotor,"mc_len":len(mc),"mc_default_len":len(mcd),
                       "app_len":len(app),"app_default_len":len(appd),"diag_node":diag["node_id"]}
        adc=self.dev.adc(); can_ids=self.dev.ping_can()
        assert self.dev.right_id in can_ids
        alive_before_l=self.dev.diag("local","alive_before_local")
        alive_before_r=self.dev.diag("right","alive_before_right")
        self.dev.send_only(bytes((COMM_ALIVE,)),node="local")
        self.dev.send_only(bytes((COMM_ALIVE,)),node="right")
        time.sleep(0.03)
        alive_after_l=self.dev.diag("local","alive_after_local")
        alive_after_r=self.dev.diag("right","alive_after_right")
        assert (alive_after_l.get("route_packets_this") or 0) > (alive_before_l.get("route_packets_this") or 0)
        assert (alive_after_r.get("route_packets_this") or 0) > (alive_before_r.get("route_packets_this") or 0)
        terminal_status=self.dev.terminal("hb_current_status")
        assert "HB current calibration:" in terminal_status
        out["local_only"]={"decoded_adc":adc,"ping_can":can_ids,"terminal_status":terminal_status.strip(),
                           "alive_local_route_delta":(alive_after_l.get("route_packets_this") or 0)-(alive_before_l.get("route_packets_this") or 0),
                           "alive_right_route_delta":(alive_after_r.get("route_packets_this") or 0)-(alive_before_r.get("route_packets_this") or 0)}
        (self.log.root/"get_command_matrix.json").write_text(json.dumps(out,indent=2,default=safe_json),encoding="utf-8")
        return {"local_controller_id":out["local"]["values"]["controller_id"],
                "right_controller_id":out["right"]["values"]["controller_id"],"can_ids":can_ids,
                "get_commands_tested":["FW_VERSION","GET_VALUES","GET_VALUES_SELECTIVE","GET_VALUES_SETUP","GET_VALUES_SETUP_SELECTIVE",
                                       "GET_MCCONF","GET_MCCONF_DEFAULT","GET_APPCONF","GET_APPCONF_DEFAULT","GET_DECODED_ADC","ROTOR_POSITION","PING_CAN","CUSTOM_APP_DATA","ALIVE","TERMINAL_STATUS"]}

    def zero_set_routing_matrix(self) -> dict[str, Any]:
        """Exercise safe zero-valued SETs on both contexts without permitting motion."""
        assert self.dev
        # Always emit a four-command scaling contract, even when later sensor detect
        # prevents non-zero actuation. This distinguishes wire/parser coverage from
        # hardware-ready motion coverage in the returned log ZIP.
        contract_rows=[]
        for kind,val in (("duty",+self.args.duty),("duty",-self.args.duty),
                         ("current",+self.args.current),("current",-self.args.current),
                         ("rpm",+self.args.erpm),("rpm",-self.args.erpm),
                         ("pos",+self.args.pos_step_deg),("pos",-self.args.pos_step_deg)):
            cmd,raw=self._set_wire_expectation(kind,float(val))
            contract_rows.append({"kind":kind,"requested":val,"command":cmd,"wire_int32":raw})
        with (self.log.root/"set_command_contract.csv").open("w",newline="",encoding="utf-8") as f:
            w=csv.DictWriter(f,fieldnames=list(contract_rows[0].keys())); w.writeheader(); w.writerows(contract_rows)
        rows=[]
        for target in ("local","right"):
            peer="right" if target=="local" else "local"
            for node in ("local","right"):
                for _ in range(2): self.dev.stop(node); time.sleep(0.02)
            before_t=self.dev.diag(target,"zero_set_before")
            before_p=self.dev.diag(peer,"zero_set_peer_before")
            commands=[
                ("SET_DUTY_0",lambda:self.dev.set_duty(target,0.0),COMM_SET_DUTY),
                ("SET_CURRENT_0",lambda:self.dev.set_current(target,0.0),COMM_SET_CURRENT),
                ("SET_BRAKE_0",lambda:self.dev.set_current_brake(target,0.0),COMM_SET_CURRENT_BRAKE),
                ("SET_HANDBRAKE_0",lambda:self.dev.set_handbrake(target,0.0),COMM_SET_HANDBRAKE),
                ("SET_RPM_0",lambda:self.dev.set_rpm(target,0),COMM_SET_RPM),
                ("SET_CURRENT_REL_0",lambda:self.dev.set_current_rel(target,0.0),COMM_SET_CURRENT_REL),
            ]
            for name,fn,cmd in commands:
                fn(); time.sleep(0.035)
                dt=self.dev.diag(target,f"zero_{name}")
                dp=self.dev.diag(peer,f"zero_{name}_peer")
                vt=self.dev.values(target,f"zero_{name}"); vp=self.dev.values(peer,f"zero_{name}_peer")
                if name == "SET_DUTY_0":
                    assert dt["bridge_moe"] and dt["armed"], f"{target} SET_DUTY(0) did not enter Full Brake"
                    assert not dp["bridge_moe"] and not dp["armed"], f"{target} Full Brake armed peer"
                    # Return to released state before exercising the remaining zero SETs.
                    self.dev.stop(target); time.sleep(0.05)
                else:
                    assert not dt["bridge_moe"] and not dp["bridge_moe"], f"zero SET unexpectedly enabled bridge: {target}/{name}"
                assert abs(vt["motor_current_A"])<=0.25 and abs(vp["motor_current_A"])<=0.05
                assert dt.get("last_set_command")==cmd, f"{target} last_set={dt.get('last_set_command')} expected={cmd}"
                # Peer set counter must not move because forwarding selected target only.
                rows.append({"target":target,"command":name,"target_set_count":dt.get("set_packets_this"),
                             "peer_set_count":dp.get("set_packets_this"),"target_bridge":dt["bridge_moe"],
                             "peer_bridge":dp["bridge_moe"],"target_controller_id":vt.get("controller_id"),
                             "peer_controller_id":vp.get("controller_id")})
                self.log.command_trace({
                    "timestamp":now_iso(),"scope":"zero_safe_routing","node":target,"peer":peer,
                    "kind":name.replace("SET_","").replace("_0","").lower(),"requested":0,
                    "expected_command":cmd,"expected_wire_raw":0,"last_set_command":dt.get("last_set_command"),
                    "last_set_host_raw":dt.get("last_set_host_raw"),"last_set_normalized":dt.get("last_set_normalized"),
                    "runtime_setpoint":dt.get("runtime_setpoint"),"runtime_command":dt.get("runtime_command"),
                    "last_set_run":dt.get("last_set_run"),"armed":dt.get("armed"),"arm_reject":dt.get("arm_reject_name"),
                    "bridge_moe":dt.get("bridge_moe"),"bridge_warmup":dt.get("bridge_warmup_this"),
                    "current_measurement_valid":dt.get("current_measurement_valid"),"standard_current_valid":dt.get("standard_current_valid"),
                    "internal_error":dt.get("internal_error_name"),"vesc_fault":dt.get("vesc_fault_name"),
                    "vesc_imotor_A":vt.get("motor_current_A"),"vesc_ibattery_A":vt.get("input_current_A"),
                    "vesc_id_A":vt.get("id_A"),"vesc_iq_A":vt.get("iq_A"),"duty":vt.get("duty"),
                    "erpm":vt.get("erpm"),"position_deg":vt.get("position_deg"),"peer_armed":dp.get("armed"),
                    "peer_bridge_moe":dp.get("bridge_moe"),"peer_imotor_A":vp.get("motor_current_A"),
                    "wire_fault_last":dt.get("wire_fault_last"),"wire_values_reply_count":dt.get("wire_values_reply_count"),
                    "wire_values_command":dt.get("wire_values_command"),"wire_values_payload_len":dt.get("wire_values_payload_len")})
            after_t=self.dev.diag(target,"zero_set_after")
            after_p=self.dev.diag(peer,"zero_set_peer_after")
            assert (after_t.get("set_packets_this") or 0)-(before_t.get("set_packets_this") or 0) >= len(commands)
            assert (after_p.get("set_packets_this") or 0)==(before_p.get("set_packets_this") or 0), \
                f"SET cross-talk: target={target} changed peer set counter"
        if rows:
            with (self.log.root/"zero_set_routing.csv").open("w",newline="",encoding="utf-8") as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0].keys()));w.writeheader();w.writerows(rows)
        return {"rows":len(rows),"commands":["SET_DUTY","SET_CURRENT","SET_CURRENT_BRAKE","SET_HANDBRAKE","SET_RPM","SET_CURRENT_REL"]}

    def adc_test(self) -> dict[str, Any]:
        assert self.dev
        arr = []
        for _ in range(5):
            a = self.dev.adc(); arr.append(a); time.sleep(0.05)
        for a in arr:
            assert -0.2 <= a["voltage1_V"] <= 3.6, f"PA2 voltage out of range {a['voltage1_V']}"
            assert -0.2 <= a["voltage2_V"] <= 3.6, f"PA3 voltage out of range {a['voltage2_V']}"
        (self.log.root / "adc_samples.json").write_text(json.dumps(arr, indent=2), encoding="utf-8")
        return arr[-1]

    def config_read_and_roundtrip(self) -> dict[str, Any]:
        assert self.dev
        info: dict[str, Any] = {}
        for node in ("local", "right"):
            mc = self.dev.transact(bytes((COMM_GET_MCCONF,)), node=node, timeout=3.0)
            assert mc[0] == COMM_GET_MCCONF and len(mc) > 5
            sig = int.from_bytes(mc[1:5], "big")
            assert sig == VESC6_MCCONF_SIGNATURE, f"{node} mc signature {sig}"
            (self.log.root / f"{node}_mcconf.bin").write_bytes(mc[1:])
            info[f"{node}_mc_len"] = len(mc) - 1
            info[f"{node}_mc_signature"] = sig
            if self.args.full:
                ack = self.dev.transact(bytes((COMM_SET_MCCONF,)) + mc[1:], node=node, timeout=4.0)
                assert ack == bytes((COMM_SET_MCCONF,)), f"{node} SET_MCCONF no ACK: {ack.hex()}"
                mc2 = self.dev.transact(bytes((COMM_GET_MCCONF,)), node=node, timeout=3.0)
                (self.log.root / f"{node}_mcconf_after.bin").write_bytes(mc2[1:])
                diffs=[{"offset":i,"before":a,"after":b} for i,(a,b) in enumerate(zip(mc[1:],mc2[1:])) if a!=b]
                (self.log.root / f"{node}_mcconf_diff.json").write_text(json.dumps(diffs,indent=2),encoding="utf-8")
                assert not diffs and len(mc2)==len(mc), f"{node} MCCONF changed after same-value roundtrip first_diff={diffs[:1]}"
        app = self.dev.transact(bytes((COMM_GET_APPCONF,)), node="local", timeout=3.0)
        assert app[0] == COMM_GET_APPCONF and len(app) > 5
        app_sig = int.from_bytes(app[1:5], "big")
        assert app_sig == VESC6_APPCONF_SIGNATURE, f"app signature {app_sig}"
        (self.log.root / "appconf_test_mode.bin").write_bytes(app[1:])
        info["app_len"] = len(app) - 1; info["app_signature"] = app_sig
        if self.args.full:
            ack = self.dev.transact(bytes((COMM_SET_APPCONF,)) + app[1:], node="local", timeout=4.0)
            assert ack == bytes((COMM_SET_APPCONF,)), f"SET_APPCONF no ACK: {ack.hex()}"
            app2 = self.dev.transact(bytes((COMM_GET_APPCONF,)), node="local", timeout=3.0)
            assert app2[1:] == app[1:], "APPCONF changed after same-value roundtrip"
        return info

    def app_mode_test(self) -> dict[str, Any]:
        """Exercise APP_UART, APP_ADC and APP_ADC_UART without commanding torque.

        The VESC6 wire offsets are locked by tests/test_vesc_config_wire.c:
        app_to_use=33 and adc_ctrl_type=90. ADC modes are tested with
        VESC_ADC_NONE so PA2/PA3 sampling and dispatch are verified without
        turning throttle voltage into a motor command. Original APPCONF is
        restored afterwards and also from the process-finally path.
        """
        assert self.dev
        if self.original_appconf is None:
            original = self.dev.transact(bytes((COMM_GET_APPCONF,)), node="local", timeout=3.0)
            assert original[0] == COMM_GET_APPCONF and len(original) == 494, f"unexpected APPCONF length {len(original)}"
            self.original_appconf = bytes(original[1:])
        out: dict[str, Any] = {}
        for mode, name in ((3, "UART"), (2, "ADC"), (5, "ADC_UART")):
            blob = bytearray(self.original_appconf)
            blob[33] = mode
            if mode in (2, 5):
                blob[90] = 0  # VESC_ADC_NONE: exercise ADC pipeline without actuation.
            ack = self.dev.transact(bytes((COMM_SET_APPCONF,)) + bytes(blob), node="local", timeout=4.0)
            assert ack == bytes((COMM_SET_APPCONF,)), f"SET_APPCONF {name} no ACK"
            time.sleep(0.15)
            d = self.dev.diag("local", f"app_{name.lower()}")
            assert d["app_mode"] == mode, f"APP {name} did not apply, diag={d['app_mode']}"
            # UART must stay usable in every mode on this port.
            fw = self.dev.fw("local")
            a = self.dev.adc()
            out[name] = {"diag": d, "fw": fw["version"], "adc": a}
            if mode in (2, 5):
                assert d["adc_ctrl_type"] == 0, f"safe ADC ctrl NONE not applied in {name}"
                assert not d["armed"], f"{name} with ADC ctrl NONE unexpectedly armed motor"
        # Keep a safe UART-only app active for the remaining motor command tests.
        # The original blob is restored from the process-finally path. This prevents
        # an original APP_ADC throttle from competing with deterministic UART tests.
        safe_uart = bytearray(self.original_appconf)
        safe_uart[33] = 3  # VESC_APP_UART
        safe_uart[90] = 0  # VESC_ADC_NONE
        ack = self.dev.transact(bytes((COMM_SET_APPCONF,)) + bytes(safe_uart), node="local", timeout=4.0)
        assert ack == bytes((COMM_SET_APPCONF,)), "failed to leave safe APP_UART mode active"
        time.sleep(0.15)
        out["test_mode"] = self.dev.diag("local", "app_uart_test_mode")
        assert out["test_mode"]["app_mode"] == 3 and not out["test_mode"]["armed"]
        return out

    def restore_appconf(self) -> None:
        if self.dev is None or self.original_appconf is None:
            return
        try:
            # Ensure the configuration write safety gate is open.
            for node in ("local", "right"):
                try:
                    self.dev.stop(node)
                except Exception:
                    pass
            time.sleep(0.15)
            ack = self.dev.transact(bytes((COMM_SET_APPCONF,)) + self.original_appconf, node="local", timeout=4.0)
            if ack != bytes((COMM_SET_APPCONF,)):
                self.log.log("WARN", f"APPCONF restore returned {ack.hex()}")
            else:
                self.log.log("INFO", "Original APPCONF restored")
        finally:
            self.original_appconf = None

    def crc_recovery_hardware(self) -> dict[str, Any]:
        assert self.dev
        # Send one bad frame with a harmless FW_VERSION payload, then verify a clean
        # request immediately works. No command is executed from the corrupt frame.
        p = bytes((COMM_FW_VERSION,)); bad = bytearray(encode_frame(p)); bad[-3] ^= 0x80
        self.log.packet("TX", "local", p, bytes(bad))
        self.dev.ser.write(bytes(bad)); self.dev.ser.flush(); time.sleep(0.05)
        fw = self.dev.fw("local")
        assert fw["version"] == "6.00"
        return fw

    def integrated_auto_detect(self) -> dict[str, Any]:
        """Board-specific commissioning: current-cal -> LEFT encoder -> RIGHT Hall -> LEFT sync -> EEPROM.

        This intentionally is not presented as the full upstream R/L/flux-linkage FOC
        motor-model detection. It is the dual-hoverboard sensor commissioning path.
        """
        assert self.dev
        before_left = self.dev.diag("local", "auto_before_left")
        before_right = self.dev.diag("right", "auto_before_right")
        self.log.detect_trace("local", "auto_before_left", before_left)
        self.log.detect_trace("right", "auto_before_right", before_right)

        self.dev.ser.reset_input_buffer(); self.dev.parser.reset()
        self.dev.send_only(bytes((COMM_DETECT_APPLY_ALL_FOC,)), node="local")
        deadline = time.monotonic() + max(45.0, self.args.detect_timeout * 2.2)
        next_local = time.monotonic() + 0.25
        next_right = time.monotonic() + 0.65
        diag_req = bytes((COMM_CUSTOM_APP_DATA,)) + HBTS_MAGIC + bytes((HBTS_VERSION, HBTS_GET_DIAG))
        reply: Optional[bytes] = None
        progress: list[dict[str, Any]] = []

        while time.monotonic() < deadline:
            try:
                payload = self.dev.read_payload(timeout=0.04, node="auto")
            except TimeoutError:
                payload = b""
            if payload:
                if payload[0] == COMM_DETECT_APPLY_ALL_FOC:
                    reply = payload
                    break
                if payload[0] == COMM_CUSTOM_APP_DATA:
                    try:
                        d = decode_diag(payload)
                        got_id = int(d.get("node_id", -1))
                        n = "right" if got_id == int(self.dev.right_id or -999) else "local"
                        stage = int(d.get("auto_detect_stage") or 0)
                        d["auto_detect_stage_name"] = AUTO_DETECT_STAGE_NAMES.get(stage, str(stage))
                        self.log.diag(n, "integrated_auto_progress", d)
                        self.log.detect_trace(n, "integrated_auto_progress", d)
                        progress.append({"node": n, "stage": stage, "cal_state": d.get("cal_state"),
                                         "encoder_ready": d.get("encoder_electrical_ready"),
                                         "alignment_probe_delta": d.get("alignment_probe_delta"),
                                         "alignment_direction_proved": d.get("alignment_direction_proved")})
                    except Exception as exc:
                        self.log.log("WARN", f"integrated auto-detect progress decode ignored: {exc}")
                    continue

            now = time.monotonic()
            if now >= next_local:
                self.dev.send_only(diag_req, node="local")
                next_local = now + 0.45
            if now >= next_right:
                self.dev.send_only(diag_req, node="right")
                next_right = now + 0.90

        if reply is None:
            raise TimeoutError("integrated COMM_DETECT_APPLY_ALL_FOC terminal reply timeout")
        if len(reply) < 3:
            raise AssertionError(f"short integrated detect reply: {reply.hex()}")
        result = struct.unpack(">h", reply[1:3])[0]
        assert result == 2, f"integrated sensor commissioning failed result={result}"

        # The terminal reply is only sent after the EEPROM save stage. Verify the
        # persistent/runtime state that matters before any motion command is tested.
        time.sleep(0.10)
        after_left = self.dev.diag("local", "auto_after_left")
        after_right = self.dev.diag("right", "auto_after_right")
        self.log.detect_trace("local", "auto_after_left", after_left)
        self.log.detect_trace("right", "auto_after_right", after_right)
        assert after_left.get("calibrated"), "integrated detect: LEFT calibrated flag false"
        assert int(after_left.get("sensor_type", -1)) == 1, "integrated detect: LEFT is not encoder AB"
        assert after_left.get("encoder_electrical_ready"), "integrated detect: LEFT encoder not electrically synchronized"
        assert after_left.get("alignment_direction_proved"), (
            "integrated detect: LEFT electrical sync did not prove encoder direction; "
            f"raw TIM4 probe delta={after_left.get('alignment_probe_delta')}")
        assert abs(int(after_left.get("alignment_probe_delta") or 0)) >= 2, (
            "integrated detect: LEFT encoder direction probe moved less than two raw counts; "
            "do not trust electrical phase for torque control")
        assert after_right.get("calibrated"), "integrated detect: RIGHT calibrated flag false"
        assert int(after_right.get("sensor_type", -1)) == 0, "integrated detect: RIGHT is not Hall"
        assert int(after_left.get("auto_detect_result") or 0) == 2 or result == 2
        return {"result": result, "progress_samples": len(progress),
                "left": after_left, "right": after_right,
                # This port currently commissions sensors/offsets only. The R/L/flux
                # fields shown by VESC Tool are compatibility configuration values,
                # not measurements from the upstream full motor-model detect path.
                "motor_model_measured": False}

    def rotor_position_stream_test(self) -> dict[str, Any]:
        assert self.dev
        # Exercise the same selector/stream path used by the Rotor Position buttons
        # in VESC Tool. One motor is selected at a time by firmware to keep replies
        # unambiguous on the single UART transport.
        # Modes 2/6/7 are genuine flux-observer views upstream. This port does
        # not have a model observer yet, so V20 deliberately sends no fake packet
        # instead of aliasing active sensor phase (the V19 bug).
        for node, mode, label in (("local", 2, "observer"), ("local", 6, "obs_vs_enc"),
                                  ("right", 2, "observer"), ("right", 7, "obs_vs_hall")):
            d = self.dev.diag(node, f"rotor_{label}_diag")
            assert not d.get("observer_valid"), f"{node} unexpectedly advertises a valid flux observer"
            vals = self.dev.collect_rotor_positions(node, mode, seconds=0.12)
            assert len(vals) == 0, f"{node} mode {mode} must not stream fake observer data: {vals[:6]}"
            out[f"{node}:{mode}:{label}"] = {"samples": 0, "observer_valid": False}

        out: dict[str, Any] = {}
        cases = [
            ("local", 3, "encoder_raw_mechanical"),
            ("local", 4, "pid_position"),
            ("local", 5, "pid_error"),
            ("right", 4, "pid_position"),
            ("right", 5, "pid_error"),
        ]
        for node, mode, label in cases:
            vals = self.dev.collect_rotor_positions(node, mode, seconds=0.15)
            assert len(vals) >= 3, f"{node} rotor-position mode {mode} streamed only {len(vals)} samples"
            assert all(math.isfinite(v) for v in vals)
            if mode in (3, 4):
                assert all(-0.01 <= v <= 360.01 for v in vals), f"{node} position outside 0..360: {vals[:6]}"
            out[f"{node}:{mode}:{label}"] = {"samples": len(vals), "first": vals[0], "last": vals[-1]}
        return out

    def homing_calibration_test(self, node: str = "local") -> dict[str, Any]:
        assert self.dev
        cmd = "hb_home_cal" if node == "local" else "hb_home_cal_right"
        response = self.dev.terminal(cmd)
        assert "started" in response.lower(), f"{node} homing calibration not started: {response.strip()}"
        deadline = time.monotonic() + 40.0
        samples: list[dict[str, Any]] = []
        active_seen = False
        terminal: Optional[dict[str, Any]] = None
        while time.monotonic() < deadline:
            d = self.dev.diag(node, "homing_cal_progress")
            self.log.homing_trace(node, d)
            samples.append(d)
            active_seen = active_seen or bool(d.get("homing_active_this"))
            state = int(d.get("homing_state") or 0)
            if state in (3, 4, 5):
                raise AssertionError(f"{node} homing failed state={HOMING_STATE_NAMES.get(state,state)}")
            if active_seen and not d.get("homing_active_this") and state in (10, 2):
                terminal = d
                break
            time.sleep(0.12)
        if terminal is None:
            raise TimeoutError(f"{node} full hard-stop homing did not reach READY")
        assert terminal.get("steering_calibrated"), f"{node} homing finished without calibrated span"
        assert terminal.get("steering_homed"), f"{node} homing finished without absolute home"
        span = int(terminal.get("steering_span_ticks") or 0)
        assert span != 0, f"{node} homing span is zero"
        pos = float(terminal.get("logical_position_deg") or 0.0)
        assert 0.0 <= pos <= 360.0, f"{node} logical position out of range after homing: {pos}"
        return {"terminal": terminal, "samples": len(samples), "span_ticks": span, "logical_position_deg": pos}

    def set_homing_on_boot_test(self, enabled: bool = True) -> dict[str, Any]:
        assert self.dev
        response = self.dev.terminal(f"hb_home_on {1 if enabled else 0}")
        expected = "ENABLED" if enabled else "DISABLED"
        assert expected.lower() in response.lower(), f"homing-on EEPROM write failed: {response.strip()}"
        d = self.dev.diag("local", "homing_on_boot_after_write")
        assert bool(d.get("homing_on_boot")) == enabled, f"homing-on flag did not roundtrip: {d.get('homing_on_boot')}"
        return {"response": response.strip(), "diag": d}

    def detect_node(self, node: str) -> dict[str, Any]:
        assert self.dev
        before = self.dev.diag(node, "before_detect")
        self.log.detect_trace(node, "before_detect", before)
        if node == "right" or before["sensor_type"] == 0:
            cmd = COMM_DETECT_HALL_FOC
            name = "hall"
            if before.get("hall_raw") in (0, 7):
                raw = int(before.get("hall_raw", 0))
                raise AssertionError(
                    f"{node} Hall preflight invalid raw={raw:03b}; valid Hall state must be 001..110. "
                    "Check Hall 5V/GND/U/V/W, connector continuity and sensor power before detect."
                )
        else:
            cmd = COMM_DETECT_ENCODER
            name = "encoder"
            assert before["encoder_cpr"] >= 4, f"LEFT encoder CPR invalid: {before['encoder_cpr']}"

        request = bytes((cmd,)) + pack_float32_scaled(self.args.detect_current, 1000.0)
        self.dev.ser.reset_input_buffer(); self.dev.parser.reset()
        self.dev.send_only(request, node=node)

        # Unlike the previous tester, do not wait blind for up to 30 s. Poll HBTS
        # concurrently and record target-vs-measured Id current, MOE and forced phase.
        deadline = time.monotonic() + self.args.detect_timeout
        next_diag = time.monotonic() + 0.20
        next_values = time.monotonic() + 0.35
        next_setup = time.monotonic() + 1.00
        progress: list[dict[str, Any]] = []
        progress_values: list[dict[str, Any]] = []
        progress_setup: list[dict[str, Any]] = []
        reply: Optional[bytes] = None
        diag_req = bytes((COMM_CUSTOM_APP_DATA,)) + HBTS_MAGIC + bytes((HBTS_VERSION, HBTS_GET_DIAG))
        values_req = bytes((COMM_GET_VALUES,))
        setup_req = bytes((COMM_GET_VALUES_SETUP,))
        while time.monotonic() < deadline:
            try:
                p = self.dev.read_payload(timeout=0.05, node=node)
                # Detect terminal reply. Other frames (HBTS poll replies) are decoded below.
                if p and p[0] == cmd:
                    reply = p
                    break
                if p and p[0] == COMM_GET_VALUES:
                    try:
                        v=decode_values(p)
                        self.log.telemetry(node,"detect_standard_values",v)
                        self.log.standard_wire(node,"detect_progress","GET_VALUES",p,v)
                        progress_values.append(v)
                        if int(v.get("fault") or 0) == 3:
                            raise AssertionError(f"{node} standard GET_VALUES emitted FAULT_CODE_DRV=3 during detect")
                    except AssertionError:
                        raise
                    except Exception as exc:
                        self.log.log("WARN",f"{node} detect GET_VALUES decode: {exc}")
                    continue
                if p and p[0] == COMM_GET_VALUES_SETUP:
                    try:
                        sv=decode_setup_values(p)
                        self.log.standard_wire(node,"detect_progress","GET_VALUES_SETUP",p,sv)
                        progress_setup.append(sv)
                        if int(sv.get("fault") or 0) == 3:
                            raise AssertionError(f"{node} standard GET_VALUES_SETUP emitted FAULT_CODE_DRV=3 during detect")
                    except AssertionError:
                        raise
                    except Exception as exc:
                        self.log.log("WARN",f"{node} detect GET_VALUES_SETUP decode: {exc}")
                    continue
                if len(p) >= 7 and p[0] == COMM_CUSTOM_APP_DATA and p[1:5] == HBTS_MAGIC:
                    try:
                        d = decode_diag(p)
                        expected_node = self.dev._expected_node_id(node)
                        if int(d.get("node_id", -1)) != expected_node:
                            self.log.log(
                                "WARN",
                                f"{node} detect: discard stale HBTS node_id={d.get('node_id')} expected={expected_node}")
                            continue
                        self.log.diag(node, "detect_progress", d)
                        self.log.detect_trace(node, "detect_progress", d)
                        progress.append(d)
                    except Exception as exc:
                        self.log.log("WARN", f"{node} detect diagnostic decode: {exc}")
            except TimeoutError:
                pass
            now_poll=time.monotonic()
            # V14 intentionally spaces commissioning polling. V12 could enqueue
            # HBTS + GET_VALUES back-to-back faster than UART DMA drained them,
            # starving the blocking COMM_DETECT_* terminal reply. We still sample
            # every path, but never burst all three requests at once.
            if now_poll >= next_diag:
                self.dev.send_only(diag_req, node=node)
                next_diag = now_poll + 0.30
            if now_poll >= next_values:
                self.dev.send_only(values_req, node=node)
                next_values = now_poll + 0.50
            if now_poll >= next_setup:
                self.dev.send_only(setup_req,node=node)
                next_setup = now_poll + 2.00

        assert reply is not None, f"{node} {name} detect response timeout after {self.args.detect_timeout}s"
        result = decode_hall_detect(reply) if cmd == COMM_DETECT_HALL_FOC else decode_encoder_detect(reply)

        # Current-regulated commissioning proof: after the 1.2-s alignment ramp,
        # there should be samples where bridge is active and measured current tracks
        # the user-selected target. Do not require perfect equality during sweep.
        active = [d for d in progress if d.get("detect_current_control_active") and d.get("bridge_moe")
                  and (d.get("detect_target_A") or 0.0) >= 0.20]
        if active:
            target_peak = max(abs(float(d.get("detect_target_A") or 0.0)) for d in active)
            measured_peak = max(abs(float(d.get("detect_measured_A") or 0.0)) for d in active)
            id_target_peak = max(abs(float(d.get("foc_id_target_A") or 0.0)) for d in active)
            assert id_target_peak >= min(0.20, target_peak * 0.5), \
                f"{node} detect current target never reached FOC: target={target_peak:.2f}A id_target={id_target_peak:.2f}A"
            if measured_peak < min(0.15, target_peak * 0.25):
                self.warn(f"{node}_detect_current_not_measured", {
                    "target_peak_A": target_peak, "measured_peak_A": measured_peak,
                    "hint": "check phase-current polarity/mapping, MOSFET gate enable and motor wiring"})

            # Raw ADC proof for the exact failure seen on V6 hardware. During a
            # <=5-A detect request, >1000 count deviation (~20 A at 50 count/A)
            # is not silently accepted as a plausible 1-A current. It is logged
            # as a sampling/common-mode/polarity warning so the next ZIP shows
            # whether the current PI is being fooled by the shunt front-end.
            if node == "local":
                phase_pairs = (("rlA", "current_raw_rlA", "current_offset_rlA"),
                               ("rlB", "current_raw_rlB", "current_offset_rlB"))
            else:
                phase_pairs = (("rrB", "current_raw_rrB", "current_offset_rrB"),
                               ("rrC", "current_raw_rrC", "current_offset_rrC"))
            worst = {"channel": None, "delta_counts": 0, "raw": None, "offset": None}
            rail_hits = 0
            for d in active:
                for ch, raw_key, off_key in phase_pairs:
                    raw = int(d.get(raw_key) or 0)
                    off = int(d.get(off_key) or 0)
                    delta = abs(raw - off)
                    if delta > worst["delta_counts"]:
                        worst = {"channel": ch, "delta_counts": delta, "raw": raw, "offset": off}
                    if raw <= 64 or raw >= 4031:
                        rail_hits += 1
            if worst["delta_counts"] > 1000 or rail_hits:
                self.warn(f"{node}_detect_phase_current_sampling_suspect", {
                    "target_peak_A": target_peak,
                    "measured_peak_A": measured_peak,
                    "worst_phase_adc": worst,
                    "equivalent_abs_A_at_50count_per_A": worst["delta_counts"] / 50.0,
                    "adc_rail_hits": rail_hits,
                    "hint": "phase-shunt sample is inconsistent with requested detect current; inspect ADC timing, current-sense common-mode, phase mapping/polarity"
                })
        else:
            self.warn(f"{node}_detect_no_active_current_samples", {"progress_samples": len(progress)})

        if progress_values:
            faults=sorted(set(int(v.get("fault") or 0) for v in progress_values))
            assert 3 not in faults, f"{node} standard VESC telemetry returned false DRV fault: {faults}"
            std_peak=max(abs(float(v.get("motor_current_A") or 0.0)) for v in progress_values)
            std_id_peak=max(abs(float(v.get("id_A") or 0.0)) for v in progress_values)
            if active and max(std_peak, std_id_peak) < 0.05:
                self.warn(f"{node}_detect_standard_current_missing",{
                    "GET_VALUES_samples":len(progress_values),"Imotor_peak_A":std_peak,"Id_peak_A":std_id_peak,
                    "hint":"bridge/current-domain is active but both standard VESC Imotor and Id stayed zero; inspect accumulator/wire provenance fields"})
        else:
            self.warn(f"{node}_detect_no_standard_values",{"progress_samples":len(progress)})

        if not result["success"]:
            failure_diag = self.dev.diag(node, "detect_failure")
            self.log.detect_trace(node, "detect_failure", failure_diag)
            bit = 0x02 if node == "right" else 0x01
            if int(failure_diag.get("commissioning_fast_fault_mask") or 0) & bit:
                snap=failure_diag.get("commissioning_fault_snapshot") or {}
                # Persist the exact FIRST guard-trip sample separately. Live FOC
                # variables are intentionally cleaned to zero after abort and are
                # therefore not sufficient to diagnose the original current event.
                (self.log.root/f"{node}_detect_first_fault_snapshot.json").write_text(
                    json.dumps(snap,indent=2,default=safe_json),encoding="utf-8")
                if snap.get("valid"):
                    cps=float(failure_diag.get("current_adc_counts_per_amp") or 50.0)
                    p1=float(snap.get("phase_current_1_delta") or 0)/cps
                    p2=float(snap.get("phase_current_2_delta") or 0)/cps
                    raise AssertionError(
                        f"{node} {name} detect aborted by FAST CURRENT GUARD; result={result}; "
                        f"FIRST_TRIP target={float(snap.get('target_internal') or 0)/800.0:.3f}A "
                        f"Id={float(snap.get('id_internal') or 0)/800.0:.3f}A "
                        f"Iq={float(snap.get('iq_internal') or 0)/800.0:.3f}A "
                        f"phase_delta=[{snap.get('phase_current_1_delta')},{snap.get('phase_current_2_delta')}] "
                        f"(~[{p1:.2f},{p2:.2f}]A @ {cps:g} count/A), "
                        f"raw=[{snap.get('adc_phase_1_raw')},{snap.get('adc_phase_2_raw')}] "
                        f"offset=[{snap.get('offset_phase_1')},{snap.get('offset_phase_2')}], "
                        f"duty_abs={snap.get('duty_abs_q15')}; inspect current-sample timing/polarity/mapping before increasing detect current")
                raise AssertionError(
                    f"{node} {name} detect aborted by FAST CURRENT GUARD without preserved snapshot: result={result}; "
                    f"live Id={failure_diag.get('id_A')}A Iq={failure_diag.get('iq_A')}A")
            raise AssertionError(f"{node} {name} detect failed: {result}; final_diag={failure_diag}")
        after = self.dev.diag(node, "after_detect")
        self.log.detect_trace(node, "after_detect", after)
        assert after["calibrated"], f"{node} detect replied success but calibrated flag false"
        assert after.get("terminal_detect_valid"), f"{node} detect reply arrived without terminal snapshot: {after}"
        assert int(after.get("terminal_detect_state") or -1) == 2, \
            f"{node} detect reply success but terminal state is not SUCCESS: {after}"
        assert int(after.get("pending_detect_owner") or 0) == 0, \
            f"{node} detect terminal reply arrived but transaction remains pending: {after}"
        if cmd == COMM_DETECT_HALL_FOC:
            assert after["sensor_type"] == 0
            hall_terminal_sensor = after.get("terminal_detect_sensor_type")
            assert hall_terminal_sensor is not None and int(hall_terminal_sensor) == 0, \
                f"{node} Hall terminal sensor type invalid: {hall_terminal_sensor}"
            assert int(after.get("cal_observed_hall_mask") or 0) in (0, 0x7E), \
                f"{node} Hall detect success without six-state evidence: mask={after.get('cal_observed_hall_mask')}"
        else:
            assert after["sensor_type"] == 1
            encoder_terminal_sensor = after.get("terminal_detect_sensor_type")
            assert encoder_terminal_sensor is not None and int(encoder_terminal_sensor) == 1, \
                f"{node} encoder terminal sensor type invalid: {encoder_terminal_sensor}"
            assert result["pole_pairs"] >= 1
            if after.get("terminal_encoder_ratio_fallback"):
                self.log.log("WARN", f"{node} encoder ratio used guarded configured-pole-pair fallback; TIM4/quadrature evidence was valid but loaded wheel did not follow enough for displacement ratio")
        return {"type": name, "result": result, "after": after,
                "detect_progress_samples": len(progress),
                "standard_values_samples":len(progress_values),"setup_values_samples":len(progress_setup)}

    def _record_fault(self, node: str, d: dict[str, Any]) -> None:
        real_runtime_fault = (d.get("fault_stop_remaining_ms",0) > 0 or
                              d.get("isr_overrun_fault_mask",0) != 0 or
                              d.get("overcurrent_fault_mask",0) != 0 or
                              d.get("vesc_fault",0) in (1,2,4))
        if real_runtime_fault and node not in self.first_fault_time:
            self.first_fault_time[node] = time.monotonic()
            self.log.fault_events.append({"timestamp": now_iso(), "node": node, "diag": d})

    @staticmethod
    def _set_wire_expectation(kind: str, value: float) -> tuple[int, int]:
        if kind == "duty": return COMM_SET_DUTY, int(round(value * 100000.0))
        if kind == "current": return COMM_SET_CURRENT, int(round(value * 1000.0))
        if kind == "brake": return COMM_SET_CURRENT_BRAKE, int(round(value * 1000.0))
        if kind == "handbrake": return COMM_SET_HANDBRAKE, int(round(value * 1000.0))
        if kind == "current_rel": return COMM_SET_CURRENT_REL, int(round(value * 100000.0))
        if kind == "rpm": return COMM_SET_RPM, int(value)
        if kind == "pos": return COMM_SET_POS, int(round(value * 1000000.0))
        raise ValueError(kind)

    def _stream_command(self, node: str, kind: str, value: float, duration: float, label: str) -> list[dict[str, Any]]:
        """Stream one command while auditing BOTH motor contexts.

        Commands are paced at <=50 Hz instead of the unbounded loop used by old
        testers. At every sample we read target + peer VESC values and HBTS state,
        which makes virtual-CAN cross-talk visible in the same row.
        """
        assert self.dev
        peer="right" if node=="local" else "local"
        end_t=time.monotonic()+duration
        rows:list[dict[str,Any]]=[]
        next_cmd=0.0; next_sample=0.0
        cmd_period=0.020
        position_guard_start_deg: Optional[float] = None
        position_guard_start_t = time.monotonic()
        before_target=self.dev.diag(node,label+"_route_before")
        before_peer=self.dev.diag(peer,label+"_peer_route_before")
        while time.monotonic()<end_t:
            now=time.monotonic()
            if now>=next_cmd:
                if kind=="duty": self.dev.set_duty(node,value)
                elif kind=="current": self.dev.set_current(node,value)
                elif kind=="brake": self.dev.set_current_brake(node,value)
                elif kind=="handbrake": self.dev.set_handbrake(node,value)
                elif kind=="current_rel": self.dev.set_current_rel(node,value)
                elif kind=="rpm": self.dev.set_rpm(node,int(value))
                elif kind=="pos": self.dev.set_pos(node,value)
                else: raise ValueError(kind)
                next_cmd=now+cmd_period
            if now>=next_sample:
                # Refresh command before and between diagnostic transactions so a
                # telemetry burst cannot starve the VESC watchdog/setpoint stream.
                if kind=="duty": self.dev.set_duty(node,value)
                elif kind=="current": self.dev.set_current(node,value)
                elif kind=="brake": self.dev.set_current_brake(node,value)
                elif kind=="handbrake": self.dev.set_handbrake(node,value)
                elif kind=="current_rel": self.dev.set_current_rel(node,value)
                elif kind=="rpm": self.dev.set_rpm(node,int(value))
                elif kind=="pos": self.dev.set_pos(node,value)
                vt=self.dev.values(node,label)
                dt=self.dev.diag(node,label)

                # V20 position fail-fast: a badly referenced electrical phase can
                # make the position PI build several amps while mechanical feedback
                # does not move at all. Preserve the evidence and release instead of
                # holding a stalled steering motor for the rest of the test window.
                if kind == "pos":
                    pnow = float(vt.get("position_deg") or 0.0)
                    if position_guard_start_deg is None:
                        position_guard_start_deg = pnow
                        position_guard_start_t = now
                    elif now - position_guard_start_t >= 0.60:
                        moved = abs(pnow - position_guard_start_deg)
                        iq_abs = abs(float(vt.get("iq_A") or 0.0))
                        if moved < 1.0 and iq_abs >= 3.0:
                            expected_cmd, expected_raw = self._set_wire_expectation(kind, value)
                            self.log.command_trace({
                                "timestamp": now_iso(), "scope": "position_stall_abort",
                                "node": node, "peer": peer, "kind": kind, "requested": value,
                                "expected_command": expected_cmd, "expected_wire_raw": expected_raw,
                                "last_set_command": dt.get("last_set_command"),
                                "last_set_host_raw": dt.get("last_set_host_raw"),
                                "runtime_setpoint": dt.get("runtime_setpoint"),
                                "armed": dt.get("armed"), "bridge_moe": dt.get("bridge_moe"),
                                "vesc_iq_A": vt.get("iq_A"), "vesc_vd_V": vt.get("vd"),
                                "vesc_vq_V": vt.get("vq"), "position_deg": pnow,
                                "logical_position_deg": dt.get("logical_position_deg"),
                                "position_target_ticks": dt.get("position_target_ticks"),
                                "encoder_electrical_ready": dt.get("encoder_electrical_ready"),
                                "alignment_probe_delta": dt.get("alignment_probe_delta"),
                                "alignment_direction_proved": dt.get("alignment_direction_proved"),
                                "position_session_zero_ticks": dt.get("position_session_zero_ticks"),
                                "observer_valid": dt.get("observer_valid"),
                            })
                            for _ in range(3):
                                self.dev.stop(node); time.sleep(0.02)
                            raise AssertionError(
                                f"{node} POSITION_STALL_OR_PHASE_REFERENCE_FAILURE: Iq={iq_abs:.2f}A "
                                f"but position moved only {moved:.2f}deg in >=0.6s; released immediately")

                # V17 fail-fast speed-direction guard. COMM_SET_RPM is electrical
                # RPM. If a clearly moving motor reports the opposite sign from
                # the requested eRPM, continuing the speed PI only increases Iq
                # while the error grows. Stop immediately and preserve evidence
                # instead of stressing the power stage/USB supply until timeout.
                if kind == "rpm" and value != 0:
                    measured_erpm = int(vt.get("erpm") or 0)
                    sign_threshold = max(30, int(abs(value) * 0.05))
                    if abs(measured_erpm) >= sign_threshold and ((measured_erpm > 0) != (value > 0)):
                        expected_cmd, expected_raw = self._set_wire_expectation(kind, value)
                        self.log.command_trace({
                            "timestamp": now_iso(), "scope": "speed_sign_abort", "node": node, "peer": peer,
                            "kind": kind, "requested": value, "expected_command": expected_cmd,
                            "expected_wire_raw": expected_raw, "last_set_command": dt.get("last_set_command"),
                            "last_set_host_raw": dt.get("last_set_host_raw"),
                            "last_set_normalized": dt.get("last_set_normalized"),
                            "runtime_setpoint": dt.get("runtime_setpoint"),
                            "runtime_command": dt.get("runtime_command"), "last_set_run": dt.get("last_set_run"),
                            "armed": dt.get("armed"), "arm_reject": dt.get("arm_reject_name"),
                            "bridge_moe": dt.get("bridge_moe"),
                            "current_measurement_valid": dt.get("current_measurement_valid"),
                            "standard_current_valid": dt.get("standard_current_valid"),
                            "vesc_imotor_A": vt.get("motor_current_A"), "vesc_ibattery_A": vt.get("input_current_A"),
                            "vesc_id_A": vt.get("id_A"), "vesc_iq_A": vt.get("iq_A"),
                            "vesc_vd_V": vt.get("vd"), "vesc_vq_V": vt.get("vq"),
                            "diag_id_A": dt.get("id_A"), "diag_iq_A": dt.get("iq_A"),
                            "duty": vt.get("duty"), "erpm": measured_erpm, "position_deg": vt.get("position_deg"),
                            "logical_position_deg": dt.get("logical_position_deg"),
                            "encoder_electrical_ready": dt.get("encoder_electrical_ready"),
                            "alignment_probe_delta": dt.get("alignment_probe_delta"),
                            "alignment_direction_proved": dt.get("alignment_direction_proved"),
                            "position_session_zero_ticks": dt.get("position_session_zero_ticks"),
                            "observer_valid": dt.get("observer_valid"),
                            "steering_calibrated": dt.get("steering_calibrated"),
                            "steering_homed": dt.get("steering_homed"),
                            "duty_target_q15": dt.get("duty_target_q15"),
                            "auto_detect_stage": dt.get("auto_detect_stage"),
                            "auto_detect_result": dt.get("auto_detect_result"),
                            "internal_error": dt.get("internal_error_name"), "vesc_fault": dt.get("vesc_fault_name"),
                        })
                        try:
                            for _ in range(3):
                                self.dev.stop(node); time.sleep(0.02)
                        finally:
                            raise AssertionError(
                                f"{node} SPEED_FEEDBACK_SIGN_MISMATCH: requested {int(value)} eRPM "
                                f"but measured {measured_erpm} eRPM; command was released immediately. "
                                f"Re-check encoder/Hall direction calibration before PID tuning")

                if kind=="duty": self.dev.set_duty(node,value)
                elif kind=="current": self.dev.set_current(node,value)
                elif kind=="brake": self.dev.set_current_brake(node,value)
                elif kind=="handbrake": self.dev.set_handbrake(node,value)
                elif kind=="current_rel": self.dev.set_current_rel(node,value)
                elif kind=="rpm": self.dev.set_rpm(node,int(value))
                elif kind=="pos": self.dev.set_pos(node,value)
                vp=self.dev.values(peer,label+"_peer")
                dp=self.dev.diag(peer,label+"_peer")
                self._record_fault(node,dt); self._record_fault(peer,dp)
                assert vt.get("controller_id")==self.dev._expected_node_id(node)
                assert vp.get("controller_id")==self.dev._expected_node_id(peer)
                # A command to one virtual motor must not arm the other motor.
                assert not dp["bridge_moe"] and not dp["armed"], \
                    f"virtual-CAN cross-talk: {node} {kind} armed peer {peer}"
                assert abs(float(vp.get("motor_current_A") or 0.0))<=0.05, \
                    f"virtual-CAN cross-talk: peer {peer} standard Imotor={vp.get('motor_current_A')}A"
                expected_cmd, expected_raw = self._set_wire_expectation(kind, value)
                assert dt.get("last_set_command") == expected_cmd, \
                    f"{node} {kind} decoder command mismatch: {dt.get('last_set_command')} != {expected_cmd}"
                assert dt.get("last_set_host_raw") == expected_raw, \
                    f"{node} {kind} wire scaling mismatch: {dt.get('last_set_host_raw')} != {expected_raw}"
                assert dt.get("last_set_run") is True, f"{node} {kind} decoded as release instead of run"
                assert dt.get("last_set_normalized") == dt.get("runtime_setpoint"), \
                    f"{node} {kind} normalized SET != RuntimeControl setpoint: {dt.get('last_set_normalized')} vs {dt.get('runtime_setpoint')}"
                assert dt.get("vesc_fault") != 3, \
                    f"{node} {kind}: false FAULT_CODE_DRV returned for internal error {dt.get('internal_error_name')}"
                self.log.command_trace({
                    "timestamp": now_iso(), "scope":"actuated_end_to_end", "node": node, "peer": peer, "kind": kind, "requested": value,
                    "expected_command": expected_cmd, "expected_wire_raw": expected_raw,
                    "last_set_command": dt.get("last_set_command"), "last_set_host_raw": dt.get("last_set_host_raw"),
                    "last_set_normalized": dt.get("last_set_normalized"), "runtime_setpoint": dt.get("runtime_setpoint"),
                    "runtime_command": dt.get("runtime_command"), "last_set_run": dt.get("last_set_run"),
                    "armed": dt.get("armed"), "arm_reject": dt.get("arm_reject_name"),
                    "bridge_moe": dt.get("bridge_moe"), "bridge_warmup": dt.get("bridge_warmup_this"),
                    "current_measurement_valid": dt.get("current_measurement_valid"),
                    "standard_current_valid": dt.get("standard_current_valid"),
                    "internal_error": dt.get("internal_error_name"), "vesc_fault": dt.get("vesc_fault_name"),
                    "vesc_imotor_A": vt.get("motor_current_A"), "vesc_ibattery_A": vt.get("input_current_A"),
                    "vesc_id_A": vt.get("id_A"), "vesc_iq_A": vt.get("iq_A"),
                    "vesc_vd_V": vt.get("vd"), "vesc_vq_V": vt.get("vq"),
                    "diag_id_A": dt.get("id_A"), "diag_iq_A": dt.get("iq_A"),
                    "dc_validated_A": dt.get("dc_input_validated_A"), "dc_raw_A": dt.get("dc_raw_A"),
                    "duty": vt.get("duty"), "erpm": vt.get("erpm"), "position_deg": vt.get("position_deg"),
                    "logical_position_deg": dt.get("logical_position_deg"),
                    "position_target_ticks": dt.get("position_target_ticks"),
                    "encoder_electrical_ready": dt.get("encoder_electrical_ready"),
                    "alignment_probe_delta": dt.get("alignment_probe_delta"),
                    "alignment_direction_proved": dt.get("alignment_direction_proved"),
                    "position_session_zero_ticks": dt.get("position_session_zero_ticks"),
                    "observer_valid": dt.get("observer_valid"),
                    "steering_calibrated": dt.get("steering_calibrated"),
                    "steering_homed": dt.get("steering_homed"),
                    "steering_span_ticks": dt.get("steering_span_ticks"),
                    "duty_target_q15": dt.get("duty_target_q15"),
                    "homing_on_boot": dt.get("homing_on_boot"),
                    "homing_active_this": dt.get("homing_active_this"),
                    "auto_detect_stage": dt.get("auto_detect_stage"),
                    "auto_detect_result": dt.get("auto_detect_result"),
                    "peer_armed": dp.get("armed"), "peer_bridge_moe": dp.get("bridge_moe"),
                    "peer_imotor_A": vp.get("motor_current_A"),
                    "wire_fault_last":dt.get("wire_fault_last"),"wire_values_reply_count":dt.get("wire_values_reply_count"),
                    "wire_values_command":dt.get("wire_values_command"),"wire_values_payload_len":dt.get("wire_values_payload_len"),
                })
                rows.append({"values":vt,"diag":dt,"peer_values":vp,"peer_diag":dp})
                next_sample=time.monotonic()+max(0.10,self.args.sample_period)
            time.sleep(0.003)
        after_target=self.dev.diag(node,label+"_route_after")
        after_peer=self.dev.diag(peer,label+"_peer_route_after")
        target_delta=(after_target.get("set_packets_this") or 0)-(before_target.get("set_packets_this") or 0)
        peer_delta=(after_peer.get("set_packets_this") or 0)-(before_peer.get("set_packets_this") or 0)
        assert target_delta>0, f"{node} {kind} SET packet not routed to target context"
        assert peer_delta==0, f"{node} {kind} leaked {peer_delta} SET packets into peer {peer}"
        # Persist compact command-route evidence even when a later numeric check fails.
        with (self.log.root/"command_route_evidence.log").open("a",encoding="utf-8") as f:
            f.write(json.dumps({"time":now_iso(),"node":node,"peer":peer,"kind":kind,"value":value,
                                "target_set_delta":target_delta,"peer_set_delta":peer_delta,
                                "target_last_set":after_target.get("last_set_command"),
                                "target_wire_raw":after_target.get("last_set_host_raw"),
                                "target_normalized":after_target.get("last_set_normalized"),
                                "target_run":after_target.get("last_set_run"),
                                "peer_last_set":after_peer.get("last_set_command")})+"\n")
        return rows

    @staticmethod
    def _ang_diff(target: float, actual: float) -> float:
        return (target - actual + 180.0) % 360.0 - 180.0

    def motion_test(self, node: str, kind: str, value: float, label: str) -> dict[str, Any]:
        assert self.dev
        pre = self.dev.diag(node, label + "_pre")
        assert pre["calibrated"], f"{node} sensor not calibrated"
        assert pre["internal_error"] == 0, f"{node} blocking error before test: {pre['internal_error_name']}"
        if kind == "brake":
            # Upstream CURRENT_BRAKE is direction-dependent. Establish measured
            # speed first; testing it at standstill incorrectly expects static Iq.
            spin_erpm = self.args.erpm if self.args.erpm > 0 else 900
            self._stream_command(node, "rpm", spin_erpm, max(0.55, self.args.motion_duration * 0.65), label + "_prespin")
            time.sleep(0.05)
        rows = self._stream_command(node, kind, value, self.args.motion_duration, label)
        assert rows, "no samples during command"

        # LEFT ABI may need its one-time D-axis electrical alignment on the first
        # non-zero command. A 0.9 s bench window can end exactly as alignment
        # completes, which made V16 report a false failure even though the bridge
        # and alignment current were active. Retry the same command only when the
        # observed reason is exclusively/almost exclusively ALIGNMENT_BUSY.
        if not any(x["diag"]["armed"] for x in rows):
            rejects = [str(x["diag"].get("arm_reject_name") or "") for x in rows]
            if rejects and any(r == "ALIGNMENT_BUSY" for r in rejects) and \
                    all(r in ("ALIGNMENT_BUSY", "NONE") for r in rejects):
                self.log.log("INFO", f"{node} {kind}: first-command encoder alignment still busy; extending same SET by 1.2 s")
                rows.extend(self._stream_command(node, kind, value, 1.2, label + "_post_alignment"))

        armed_any = any(x["diag"]["armed"] for x in rows)
        faulted = [x for x in rows if x["diag"]["internal_error"] or x["diag"]["vesc_fault"]]
        assert armed_any or faulted, f"{node} never armed; last reject={rows[-1]['diag']['arm_reject_name']}"
        if faulted:
            raise AssertionError(f"{node} fault during {kind}: {faulted[-1]['diag']['internal_error_name']} / {faulted[-1]['diag']['vesc_fault_name']}")
        assert any(x["diag"].get("bridge_moe") for x in rows), f"{node} {kind}: bridge never became active"
        assert any(x["diag"].get("current_measurement_valid") for x in rows), \
            f"{node} {kind}: current domain never became valid after bridge warm-up"
        assert any(x["diag"].get("standard_current_valid") for x in rows), \
            f"{node} {kind}: standard VESC Id/Iq/Imotor/Ibattery telemetry never became valid"
        assert all(x["diag"].get("vesc_fault") != 3 for x in rows), \
            f"{node} {kind}: FAULT_CODE_DRV must not be synthesized from internal readiness faults"
        vals = [x["values"] for x in rows]
        diags = [x["diag"] for x in rows]
        numeric_proof: dict[str, Any] = {}
        if kind == "duty":
            max_d = max(float(x["duty_abs"]) for x in diags)
            requested_abs = abs(float(value))
            assert max_d > max(0.002, requested_abs * 0.15), f"duty did not modulate, max={max_d:.4f}"
            # For low-duty tests the actual modulation should remain close to the
            # requested ceiling. This catches the old path where a small VESC duty
            # request accidentally reopened the FOC voltage vector to near-full scale.
            # Do not compare RPM here: DUTY is voltage/modulation control, not speed.
            if 0.0 < requested_abs <= 0.20:
                assert max_d >= max(0.002, requested_abs * 0.45), \
                    f"{node} duty under-modulates requested={requested_abs:.4f} actual_peak={max_d:.4f}"
                assert max_d <= requested_abs * 1.35 + 0.005, \
                    f"{node} duty exceeds requested ceiling requested={requested_abs:.4f} actual_peak={max_d:.4f}"
            numeric_proof = {
                "requested_duty": round(float(value), 6),
                "peak_abs_duty": round(max_d, 6),
                "wire_raw_duty_x100000": int(round(float(value) * 100000.0)),
                "semantics": "DUTY controls modulation/voltage, not RPM",
            }
        elif kind in ("current", "current_rel", "brake", "handbrake"):
            iq = [float(x["iq_A"]) for x in vals]
            if kind in ("brake", "handbrake"):
                assert max(abs(x) for x in iq) > 0.03, f"brake current did not produce Iq response: {iq}"
                expected_core_mode = 4 if kind == "brake" else 5
                modes = [int(x.get("control_mode") or -1) for x in diags if x.get("armed")]
                assert modes and all(m == expected_core_mode for m in modes), \
                    f"{node} {kind} collapsed into wrong FOC mode: {modes}, expected {expected_core_mode}"
                if kind == "brake":
                    # VESC CURRENT_BRAKE chooses Iq sign in the fast loop from
                    # instantaneous speed. Whenever speed is clearly non-zero,
                    # measured Iq must oppose it.
                    moving = [(int(x["values"].get("erpm") or 0), float(x["values"].get("iq_A") or 0.0))
                              for x in rows if abs(int(x["values"].get("erpm") or 0)) >= 30 and
                              abs(float(x["values"].get("iq_A") or 0.0)) >= 0.03]
                    assert all((e > 0 and q < 0) or (e < 0 and q > 0) for e, q in moving), \
                        f"{node} brake Iq did not oppose speed: {moving}"
            elif kind == "current":
                # Upstream VESC wire is milliamps for SET_CURRENT and centi-amps
                # for GET_VALUES Id/Iq. Verify the real board end-to-end instead
                # of merely checking that current is non-zero. Ignore the first
                # accumulator sample and use settled, standard-current-valid rows.
                settled = [x for x in rows if x["diag"].get("standard_current_valid") and
                           x["diag"].get("current_measurement_valid") and x["diag"].get("armed")]
                if len(settled) > 3:
                    settled = settled[-3:]
                assert settled, f"{node} no settled current telemetry samples"
                wire_iq = statistics.median(float(x["values"]["iq_A"]) for x in settled)
                diag_iq = statistics.median(float(x["diag"].get("iq_A") or 0.0) for x in settled)
                tolerance = max(0.15, abs(value) * 0.35)
                assert (wire_iq > 0 if value > 0 else wire_iq < 0), \
                    f"{node} current sign mismatch requested={value}A GET_VALUES_Iq={wire_iq:.3f}A"
                assert abs(wire_iq - value) <= tolerance, \
                    f"{node} current scaling mismatch requested={value:.3f}A GET_VALUES_Iq={wire_iq:.3f}A tolerance={tolerance:.3f}A"
                assert abs(wire_iq - diag_iq) <= max(0.20, abs(value) * 0.40), \
                    f"{node} GET_VALUES/diagnostic Iq disagree wire={wire_iq:.3f}A diag={diag_iq:.3f}A"
                numeric_proof = {
                    "requested_current_A": value,
                    "median_get_values_iq_A": round(wire_iq, 4),
                    "median_diag_iq_A": round(diag_iq, 4),
                    "tolerance_A": round(tolerance, 4),
                    "wire_raw_mA": int(round(value * 1000.0)),
                }
            else:
                peak = max(iq) if value > 0 else min(iq)
                assert (peak > 0.03 if value > 0 else peak < -0.03), f"Iq did not follow sign: {iq}"
        elif kind == "rpm":
            rpms = [int(x["erpm"]) for x in vals]
            peak = max(rpms) if value > 0 else min(rpms)
            assert (peak > 5 if value > 0 else peak < -5), f"ERPM no signed motion: {rpms}"
            numeric_proof = {
                "requested_erpm": int(value),
                "wire_raw_erpm": int(value),
                "peak_signed_erpm": int(peak),
                "pole_pairs_semantics": "15 pole-pairs = 30 motor poles; SET_RPM is electrical RPM",
            }
        # Position is verified by target tracking in dedicated routine below.
        return {"samples": len(rows), "armed": armed_any, "numeric_proof": numeric_proof,
                "last_values": vals[-1], "last_diag": diags[-1]}

    def position_test(self, node: str) -> dict[str, Any]:
        assert self.dev
        start = self.dev.values(node, "pos_pre")
        assert start["position_deg"] is not None
        start_pos = min(360.0, max(0.0, float(start["position_deg"])))
        # V19 position is a bounded mechanical coordinate. 0 and 360 are distinct
        # endpoints of one logical revolution/calibrated steering travel; do not
        # modulo-wrap a 360-degree command back to zero.
        step = abs(float(self.args.pos_step_deg))
        if start_pos + step <= 360.0:
            target = start_pos + step
        else:
            target = max(0.0, start_pos - step)
        rows = self._stream_command(node, "pos", target, self.args.position_duration, "position")
        assert rows
        final = rows[-1]["values"]
        final_pos = min(360.0, max(0.0, float(final["position_deg"])))
        start_err = abs(target - start_pos)
        final_err = abs(target - final_pos)
        assert any(x["diag"]["armed"] for x in rows), f"{node} position never armed"
        assert any(x["diag"].get("current_measurement_valid") for x in rows), f"{node} position current domain never valid"
        assert all(x["diag"].get("vesc_fault") != 3 for x in rows), f"{node} position false DRV fault"
        assert final_err < start_err or abs(final_pos - start_pos) >= 2.0, \
            f"position did not move toward target start={start_pos} target={target} final={final_pos}"
        return {"start_deg": start_pos, "target_deg": target, "final_deg": final_pos,
                "start_error": start_err, "final_error": final_err,
                "position_semantics": "bounded 0..360; endpoints distinct"}

    def full_brake_and_stop_test(self, node: str) -> dict[str, Any]:
        """Verify VESC Tool Full Brake (SET_DUTY 0) versus Stop semantics.

        Full Brake sends SET_DUTY(0); Stop sends SET_CURRENT(0). Upstream FOC may
        optionally convert equal zero-voltage PWM to a low-side short when
        foc_short_ls_on_zero_duty is enabled, so this test only requires safe zero
        modulation with the selected sensored controller active before Stop
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
        assert d_brake.get("armed") and d_brake.get("bridge_moe"),             f"{node} Full Brake did not keep target bridge active: {d_brake}"
        assert int(d_brake.get("last_set_command") or -1) == COMM_SET_DUTY
        assert int(d_brake.get("last_set_host_raw") or 1) == 0
        assert int(d_brake.get("duty_target_q15") or 0) == 0
        assert abs(float(v_brake.get("duty") or 0.0)) <= 0.002
        assert not d_peer.get("armed") and not d_peer.get("bridge_moe"),             f"{node} Full Brake leaked to peer {peer}: {d_peer}"

        for _ in range(4): self.dev.stop(node); time.sleep(0.05)
        d_stop = self.dev.diag(node, "full_brake_then_stop")
        assert not d_stop.get("armed") and not d_stop.get("bridge_moe"),             f"{node} Stop did not release bridge after Full Brake: {d_stop}"
        assert int(d_stop.get("last_set_command") or -1) == COMM_SET_CURRENT
        assert int(d_stop.get("last_set_host_raw") or 1) == 0
        return {"full_brake": d_brake, "peer": d_peer, "stop": d_stop}

    def stop_and_verify(self, node: str) -> dict[str, Any]:
        assert self.dev
        for _ in range(4):
            self.dev.stop(node); time.sleep(0.08)
        time.sleep(0.2)
        d = self.dev.diag(node, "stopped")
        assert not d["armed"], f"{node} still armed after zero-current release"
        assert not d["output_enabled"], f"{node} output gate still enabled"
        return d

    def false_drv_mapping_test(self) -> dict[str, Any]:
        assert self.dev
        out={}
        for node in ("local", "right"):
            # Test the exact standard packets VESC Tool parses, not only HBTS.
            values=self.dev.values(node, "false_drv_values")
            setup=self.dev.setup_values(node, label="false_drv_setup")
            d=self.dev.diag(node, "false_drv_mapping")
            assert int(values.get("fault") or 0) != 3, \
                f"{node}: COMM_GET_VALUES emitted FAULT_CODE_DRV=3"
            assert int(setup.get("fault") or 0) != 3, \
                f"{node}: COMM_GET_VALUES_SETUP emitted FAULT_CODE_DRV=3"
            assert d["vesc_fault"] != 3, \
                f"{node}: HBTS VESC fault mapping shows DRV for internal={d['internal_error_name']}"
            assert d.get("wire_fault_last") in (0,4), \
                f"{node}: last standard wire fault byte unexpected: {d.get('wire_fault_last')}"
            assert d.get("wire_values_command") == COMM_GET_VALUES_SETUP, \
                f"{node}: wire provenance did not observe GET_VALUES_SETUP"
            out[node]={"internal_error":d["internal_error_name"],"vesc_fault":d["vesc_fault_name"],
                       "get_values_fault":values.get("fault"),"get_values_setup_fault":setup.get("fault"),
                       "wire_fault_last":d.get("wire_fault_last"),
                       "wire_values_reply_count":d.get("wire_values_reply_count"),
                       "current_cal_sampling_mode":d.get("current_cal_sampling_mode_name"),
                       "bridge_warmup":d.get("bridge_warmup_this")}
        return out

    def monitor_fault_recovery(self, node: str) -> dict[str, Any]:
        assert self.dev
        if node not in self.first_fault_time:
            raise SkipTest("no runtime fault occurred; deliberate overcurrent injection is intentionally not performed")
        start = self.first_fault_time[node]
        rows = []
        deadline = start + 5.0
        while time.monotonic() < deadline:
            d = self.dev.diag(node, "fault_recovery"); rows.append(d)
            self.dev.stop(node)
            time.sleep(0.1)
        clear = [d for d in rows if d["fault_stop_remaining_ms"] == 0 and d["vesc_fault"] == 0]
        assert clear, "fault did not clear within 5 s"
        first_clear = next(d for d in rows if d in clear)
        elapsed = time.monotonic() - start
        return {"samples": len(rows), "clear_seen": True, "elapsed_observation_s": elapsed, "first_clear": first_clear}

    def run(self) -> None:
        self.result("00_protocol_self_test", self.self_test)
        if not self.args.port:
            self.result("01_hardware_tests", lambda: None, skip=True, skip_reason="--port not supplied")
            return
        if self.result("01_open_serial", self.open_device) is None:
            return
        self.result("02_connection_inventory", self.connection_inventory)
        if self.dev is None or self.dev.right_id is None:
            return
        self.result("03_connect_is_disarmed", self.connect_state)

        current_ok = self.result("04_current_offset_calibration", self.current_offset_calibration) is not None
        self.result("04a_no_false_drv_mapping", self.false_drv_mapping_test)
        self.result("05_prepare_safe_uart_test_mode", self.prepare_safe_uart_test_mode, skip=not self.args.full,
                    skip_reason="read-only run does not modify APPCONF")
        self.result("06_idle_telemetry_id_iq_imotor", self.idle_telemetry,
                    skip=not current_ok, skip_reason="prerequisite current-zero calibration failed")
        self.result("06a_protocol_get_matrix", self.protocol_get_matrix)
        self.result("06b_passive_spin_cross_side_isolation", self.passive_spin_current_observation,
                    skip=not current_ok, skip_reason="prerequisite current-zero calibration failed")
        self.result("06c_zero_set_virtual_can_routing", self.zero_set_routing_matrix,
                    skip=not self.args.full, skip_reason="read-only run does not send SET commands")
        self.result("07_adc_pa2_pa3", self.adc_test)
        self.result("08_config_read_roundtrip", self.config_read_and_roundtrip)
        self.result("10_crc_parser_recovery", self.crc_recovery_hardware)

        if self.args.full:
            # V19 default: one integrated board commissioning transaction. It is
            # deliberately the board sensor path (current-cal + LEFT encoder +
            # RIGHT Hall + LEFT electrical sync + EEPROM), not a claim of full
            # upstream R/L/flux motor-model detection. The old individual detect
            # commands remain available for regression/diagnosis behind a flag.
            if self.args.individual_detect:
                left_detect = self.result(
                    "11_detect_left_sensor_individual", lambda: self.detect_node("local"),
                    skip=not current_ok, skip_reason="prerequisite current-zero calibration failed")
                right_detect = self.result(
                    "12_detect_right_hall_individual", lambda: self.detect_node("right"),
                    skip=not current_ok, skip_reason="prerequisite current-zero calibration failed")
                integrated_detect = None
            else:
                integrated_detect = self.result(
                    "11_integrated_auto_detect_left_encoder_right_hall", self.integrated_auto_detect,
                    skip=not current_ok, skip_reason="prerequisite current-zero calibration failed")
                left_detect = integrated_detect
                right_detect = integrated_detect

            # Re-read state after terminal reply. The LEFT ready criterion includes
            # the new AB absolute electrical synchronization proof; a mere encoder
            # detect/fallback result is not enough to run FOC closed-loop.
            post_left = self.result("13_post_detect_diag_left", lambda: self.dev.diag("local", "post_detect"))
            post_right = self.result("14_post_detect_diag_right", lambda: self.dev.diag("right", "post_detect"))
            # Board commissioning is fault-contained per physical motor. A
            # partial integrated result must not hide a successfully calibrated
            # opposite side; post-detect hardware state is authoritative here.
            left_ready = bool(post_left and post_left.get("calibrated"))
            right_ready = bool(post_right and post_right.get("calibrated"))
            if not self.args.individual_detect and left_ready:
                left_ready = bool(post_left.get("encoder_electrical_ready"))

            self.result("14a_rotor_position_stream_modes", self.rotor_position_stream_test,
                        skip=not (left_ready or right_ready),
                        skip_reason="no commissioned motor available for Rotor Position stream test")

            # Hard-stop homing is intentionally opt-in. It physically drives the
            # steering mechanism to both stops, measures the signed span, maps it
            # to 0..360 and returns to 180. Never do this silently in --full.
            homing_result = self.result(
                "14b_left_full_0_360_homing_calibration",
                lambda: self.homing_calibration_test("local"),
                skip=not self.args.homing_calibrate or not left_ready,
                skip_reason="add --homing-calibrate (LEFT must be commissioned); this intentionally touches hard stops")
            homing_on_ready = left_ready and (not self.args.homing_calibrate or homing_result is not None)
            self.result(
                "14c_left_homing_on_boot_eeprom",
                lambda: self.set_homing_on_boot_test(True),
                skip=not self.args.homing_on or not homing_on_ready,
                skip_reason="add --homing-on after a valid saved 0..360 span; EEPROM write is opt-in")

            self.result("09_app_uart_adc_adc_uart", self.app_mode_test,
                        skip=not current_ok,
                        skip_reason="prerequisite current-zero calibration failed")

            # VESC Tool Full Brake is SET_DUTY(0), but the FOC low-side short is
            # a configuration option upstream. Test zero-modulation vs Stop only
            # on a commissioned feedback path; never force an ISR hardware short.
            self.result("14d_left_full_brake_then_stop", lambda: self.full_brake_and_stop_test("local"),
                        skip=not left_ready, skip_reason="LEFT feedback not commissioned")
            self.result("14e_right_full_brake_then_stop", lambda: self.full_brake_and_stop_test("right"),
                        skip=not right_ready, skip_reason="RIGHT feedback not commissioned")

            tests = [
                ("15_left_duty_pos", "local", left_ready, lambda: self.motion_test("local", "duty", +self.args.duty, "duty_pos")),
                ("16_left_duty_neg", "local", left_ready, lambda: self.motion_test("local", "duty", -self.args.duty, "duty_neg")),
                ("17_left_current_pos", "local", left_ready, lambda: self.motion_test("local", "current", +self.args.current, "current_pos")),
                ("18_left_current_neg", "local", left_ready, lambda: self.motion_test("local", "current", -self.args.current, "current_neg")),
                ("19_left_rpm_pos", "local", left_ready, lambda: self.motion_test("local", "rpm", +self.args.erpm, "rpm_pos")),
                ("20_left_rpm_neg", "local", left_ready, lambda: self.motion_test("local", "rpm", -self.args.erpm, "rpm_neg")),
                ("20a_left_brake", "local", left_ready, lambda: self.motion_test("local", "brake", +min(self.args.current,0.35), "brake")),
                ("20aa_left_handbrake", "local", left_ready, lambda: self.motion_test("local", "handbrake", +min(self.args.current,0.35), "handbrake")),
                ("20b_left_current_rel", "local", left_ready, lambda: self.motion_test("local", "current_rel", +0.02, "current_rel")),
                ("21_left_position_0_360", "local", left_ready, lambda: self.position_test("local")),
                ("22_right_duty_pos", "right", right_ready, lambda: self.motion_test("right", "duty", +self.args.duty, "duty_pos")),
                ("23_right_duty_neg", "right", right_ready, lambda: self.motion_test("right", "duty", -self.args.duty, "duty_neg")),
                ("24_right_current_pos", "right", right_ready, lambda: self.motion_test("right", "current", +self.args.current, "current_pos")),
                ("25_right_current_neg", "right", right_ready, lambda: self.motion_test("right", "current", -self.args.current, "current_neg")),
                ("26_right_rpm_pos", "right", right_ready, lambda: self.motion_test("right", "rpm", +self.args.erpm, "rpm_pos")),
                ("27_right_rpm_neg", "right", right_ready, lambda: self.motion_test("right", "rpm", -self.args.erpm, "rpm_neg")),
                ("27a_right_brake", "right", right_ready, lambda: self.motion_test("right", "brake", +min(self.args.current,0.35), "brake")),
                ("27aa_right_handbrake", "right", right_ready, lambda: self.motion_test("right", "handbrake", +min(self.args.current,0.35), "handbrake")),
                ("27b_right_current_rel", "right", right_ready, lambda: self.motion_test("right", "current_rel", +0.02, "current_rel")),
                ("28_right_position_0_360", "right", right_ready, lambda: self.position_test("right")),
            ]
            for name, node, ready, fn in tests:
                self.result(name, fn, skip=not ready, skip_reason=f"{node} sensor/sync prerequisite failed")
                if ready:
                    self.result(name + "_release", lambda n=node: self.stop_and_verify(n))

            for idx, node in enumerate(("local", "right"), start=29):
                ready = left_ready if node == "local" else right_ready
                self.result(
                    f"{idx:02d}_{node}_fault_stop_3s",
                    lambda n=node: self.monitor_fault_recovery(n),
                    skip=not ready, skip_reason=f"{node} sensor/sync prerequisite failed")
        else:
            self.result("11_full_commission_motion", lambda: None, skip=True,
                        skip_reason="read-only run; add --full --yes to test detect + Duty/Current/RPM/POS")

        self.result("90_final_left_diag", lambda: self.dev.diag("local", "final"))
        self.result("91_final_right_diag", lambda: self.dev.diag("right", "final"))


class SkipTest(Exception):
    pass


def safe_json(obj: Any) -> Any:
    if dataclasses.is_dataclass(obj):
        return dataclasses.asdict(obj)
    if isinstance(obj, Path):
        return str(obj)
    return obj


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--port", help="UART serial port, e.g. COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.5, help="normal transaction timeout")
    ap.add_argument("--full", action="store_true", help="run integrated commissioning and motor movement tests")
    ap.add_argument("--yes", action="store_true", help="non-interactive confirmation for motion/homing writes")
    ap.add_argument("--individual-detect", action="store_true",
                    help="use legacy separate LEFT encoder and RIGHT Hall detect instead of V21 integrated board commissioning")
    ap.add_argument("--homing-calibrate", action="store_true",
                    help="opt-in LEFT full hard-stop calibration: right stop=0, left stop=360, save span, return 180")
    ap.add_argument("--homing-on", action="store_true",
                    help="enable and persist LEFT one-stop homing on future power-on (requires a previously calibrated span)")
    ap.add_argument("--detect-current", type=float, default=1.0, help="sensor detect current [A] for --individual-detect; V21 integrated board commissioning uses conservative fixed 1.00 A")
    ap.add_argument("--current-cal-retries", type=int, default=2, help="retry transient block-mean current-zero failures")
    ap.add_argument("--detect-timeout", type=float, default=65.0)
    ap.add_argument("--duty", type=float, default=0.03, help="absolute duty used for +/- duty test")
    ap.add_argument("--current", type=float, default=0.50, help="absolute motor current used for +/- current test [A]")
    ap.add_argument("--erpm", type=int, default=900, help="absolute electrical RPM used for +/- speed test")
    ap.add_argument("--pos-step-deg", type=float, default=15.0)
    ap.add_argument("--motion-duration", type=float, default=0.9)
    ap.add_argument("--position-duration", type=float, default=1.8)
    ap.add_argument("--idle-samples", type=int, default=8)
    ap.add_argument("--sample-period", type=float, default=0.12)
    ap.add_argument("--passive-spin-seconds", type=float, default=3.0,
                    help="released-motor current observation window; rotate wheels by hand during this stage")
    ap.add_argument("--log-dir", default="vesc_test_logs", help="parent directory for timestamped log bundle")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if (args.homing_calibrate or args.homing_on) and not args.full:
        print("--homing-calibrate/--homing-on hanya boleh dipakai bersama --full.")
        return 2
    if args.full and not args.yes:
        print("FULL TEST akan menjalankan sensor commissioning dan menggerakkan KEDUA motor.")
        if args.homing_calibrate:
            print("PERINGATAN: --homing-calibrate akan menyentuh hard stop steering kanan dan kiri, lalu kembali ke 180°.")
        if args.homing_on:
            print("PERINGATAN: --homing-on akan menulis flag homing power-on ke EEPROM.")
        print("Pastikan roda terangkat, steering bebas/aman, power supply/current limit aman, emergency stop siap.")
        ans = input("Ketik RUN untuk melanjutkan: ").strip()
        if ans != "RUN":
            print("Cancelled.")
            return 3
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    root = Path(args.log_dir).expanduser().resolve() / f"vesc_full_test_{stamp}"
    log = BundleLogger(root)
    suite = TestSuite(args, log)
    log.log("INFO", f"Log bundle: {root}")
    log.log("INFO", f"tester_release={TESTER_RELEASE} Python={sys.version.split()[0]} OS={platform.platform()} args={vars(args)}")
    try:
        suite.run()
    finally:
        # Always request release on both sides if a serial connection was opened.
        if suite.dev is not None:
            for node in ("local", "right"):
                try:
                    for _ in range(3):
                        suite.dev.stop(node); time.sleep(0.05)
                except Exception as exc:
                    log.log("WARN", f"final emergency release {node} failed: {exc}")
            try:
                suite.restore_appconf()
            except Exception as exc:
                log.log("WARN", f"final APPCONF restore failed: {exc}")
            try:
                suite.dev.close()
            except Exception:
                pass

        counts = {s: sum(1 for r in log.results if r.status == s) for s in ("PASS", "FAIL", "WARN", "SKIP")}
        result_doc = {
            "generated_at": now_iso(),
            "tester_release": TESTER_RELEASE,
            "args": vars(args),
            "environment": {"python": sys.version, "platform": platform.platform()},
            "counts": counts,
            "results": [dataclasses.asdict(r) for r in log.results],
            "fault_events": log.fault_events,
            "log_files": ["session.log", "raw_packets.log", "telemetry.csv", "diagnostics.csv", "detect_transaction_trace.csv", "rotor_position_stream.csv", "homing_state_trace.csv", "set_command_trace.csv", "set_command_contract.csv", "vesc_standard_wire.jsonl", "results.json", "summary.txt"],
        }
        (root / "results.json").write_text(json.dumps(result_doc, indent=2, default=safe_json), encoding="utf-8")
        summary = [
            "HOVERBOARD VESC FULL TEST SUMMARY",
            f"Tester release: {TESTER_RELEASE}",
            f"Generated: {result_doc['generated_at']}",
            f"PASS={counts['PASS']} FAIL={counts['FAIL']} WARN={counts['WARN']} SKIP={counts['SKIP']}",
            "",
        ]
        for r in log.results:
            summary.append(f"[{r.status:4}] {r.name} ({r.duration_s:.3f}s) {r.error or ''}")
        summary += ["", "Kirim SELURUH folder log ini saat troubleshooting, terutama:",
                    "session.log, results.json, diagnostics.csv, detect_transaction_trace.csv, telemetry.csv, raw_packets.log,",
                    "set_command_trace.csv, set_command_contract.csv, vesc_standard_wire.jsonl"]
        (root / "summary.txt").write_text("\n".join(summary) + "\n", encoding="utf-8")
        log.log("INFO", f"FINAL COUNTS {counts}")
        log.log("INFO", f"Kirim folder ini untuk troubleshooting: {root}")
        log.close()
        try:
            zip_path = shutil.make_archive(str(root), "zip", root_dir=root)
            print(f"{now_iso()} [INFO ] LOG ZIP READY: {zip_path}")
        except Exception as exc:
            print(f"{now_iso()} [WARN ] gagal membuat ZIP log: {exc}")
    return 1 if any(r.status == "FAIL" for r in suite.log.results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
