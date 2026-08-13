#!/usr/bin/env python3
"""Minimal VESC 6.00 UART probe for this hoverboard port.

Usage:
    python3 tools/vesc_probe.py /dev/ttyUSB0 --baud 115200

Requires pyserial only on the host PC. It does not modify controller config.
It checks local FW_VERSION, PING_CAN, local GET_VALUES and forwarded-right
FW_VERSION/GET_VALUES.
"""
from __future__ import annotations
import argparse
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial belum terpasang: pip install pyserial") from exc

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_FORWARD_CAN = 34
COMM_PING_CAN = 62


def crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame(payload: bytes) -> bytes:
    if not payload:
        raise ValueError("empty payload")
    if len(payload) <= 255:
        head = bytes((2, len(payload)))
    elif len(payload) <= 65535:
        head = bytes((3, (len(payload) >> 8) & 0xFF, len(payload) & 0xFF))
    else:
        raise ValueError("payload too long")
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


def read_frame(port: serial.Serial, timeout: float = 1.0) -> bytes:
    deadline = time.monotonic() + timeout
    state = "wait"
    need = 0
    payload = bytearray()
    rx_crc = 0
    while time.monotonic() < deadline:
        b = port.read(1)
        if not b:
            continue
        x = b[0]
        if state == "wait":
            if x == 2:
                state = "len8"
            elif x == 3:
                state = "len16h"
        elif state == "len8":
            need = x
            payload.clear()
            state = "data"
        elif state == "len16h":
            need = x << 8
            state = "len16l"
        elif state == "len16l":
            need |= x
            payload.clear()
            state = "data"
        elif state == "data":
            payload.append(x)
            if len(payload) == need:
                state = "crch"
        elif state == "crch":
            rx_crc = x << 8
            state = "crcl"
        elif state == "crcl":
            rx_crc |= x
            state = "stop"
        elif state == "stop":
            if x == 3 and crc16(payload) == rx_crc:
                return bytes(payload)
            state = "wait"
    raise TimeoutError("VESC response timeout")


def transact(port: serial.Serial, payload: bytes, timeout: float = 1.0) -> bytes:
    port.reset_input_buffer()
    port.write(frame(payload))
    port.flush()
    return read_frame(port, timeout)


def cstring(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\0", offset)
    if end < 0:
        raise ValueError("unterminated string")
    return data[offset:end].decode("ascii", "replace"), end + 1


def decode_fw(payload: bytes) -> dict:
    if len(payload) < 4 or payload[0] != COMM_FW_VERSION:
        raise ValueError("not FW_VERSION")
    major, minor = payload[1], payload[2]
    hw, i = cstring(payload, 3)
    if i + 12 > len(payload):
        raise ValueError("short FW_VERSION UUID")
    uuid = payload[i:i + 12]
    i += 12
    # pairing/test/hw_type/custom_cfg/phase_filters/qml_hw/qml_app/nrf_flags
    extra = payload[i:i + 8]
    i += min(8, len(payload) - i)
    fw_name = ""
    if i < len(payload):
        try:
            fw_name, _ = cstring(payload, i)
        except ValueError:
            pass
    return {
        "version": f"{major}.{minor:02d}",
        "hardware": hw,
        "uuid": uuid.hex(),
        "fw_name": fw_name,
        "extra": extra.hex(),
    }


def i16(data: bytes, pos: int) -> tuple[int, int]:
    return struct.unpack_from(">h", data, pos)[0], pos + 2


def i32(data: bytes, pos: int) -> tuple[int, int]:
    return struct.unpack_from(">i", data, pos)[0], pos + 4


def decode_values(payload: bytes) -> dict:
    if not payload or payload[0] != COMM_GET_VALUES:
        raise ValueError("not GET_VALUES")
    p = 1
    temp_fet, p = i16(payload, p)
    temp_motor, p = i16(payload, p)
    current_motor, p = i32(payload, p)
    current_in, p = i32(payload, p)
    id_a, p = i32(payload, p)
    iq_a, p = i32(payload, p)
    duty, p = i16(payload, p)
    erpm, p = i32(payload, p)
    vin, p = i16(payload, p)
    # Skip Ah, Ah charged, Wh, Wh charged.
    p += 16
    tach, p = i32(payload, p)
    tach_abs, p = i32(payload, p)
    fault = payload[p] if p < len(payload) else 0
    p += 1
    pid_pos = None
    controller_id = None
    if p + 4 <= len(payload):
        raw, p = i32(payload, p)
        pid_pos = raw / 1_000_000.0
    if p < len(payload):
        controller_id = payload[p]
    return {
        "temp_fet_C": temp_fet / 10.0,
        "temp_motor_C": temp_motor / 10.0,
        "motor_current_A": current_motor / 100.0,
        "input_current_A": current_in / 100.0,
        "id_A": id_a / 100.0,
        "iq_A": iq_a / 100.0,
        "duty": duty / 1000.0,
        "erpm": erpm,
        "vin_V": vin / 10.0,
        "tach": tach,
        "tach_abs": tach_abs,
        "fault": fault,
        "position_deg": pid_pos,
        "controller_id": controller_id,
    }


def print_dict(title: str, d: dict) -> None:
    print(f"\n{title}")
    for k, v in d.items():
        print(f"  {k}: {v}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("port", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--timeout", type=float, default=1.0)
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1.0) as ser:
        fw = transact(ser, bytes((COMM_FW_VERSION,)), args.timeout)
        print_dict("LOCAL FW", decode_fw(fw))

        ping = transact(ser, bytes((COMM_PING_CAN,)), args.timeout)
        if not ping or ping[0] != COMM_PING_CAN:
            raise RuntimeError("invalid PING_CAN reply")
        ids = list(ping[1:])
        print(f"\nVirtual CAN IDs: {ids}")

        vals = transact(ser, bytes((COMM_GET_VALUES,)), args.timeout)
        print_dict("LEFT / LOCAL VALUES", decode_values(vals))

        if ids:
            rid = ids[0]
            rfw = transact(ser, bytes((COMM_FORWARD_CAN, rid, COMM_FW_VERSION)), args.timeout)
            print_dict(f"RIGHT / VIRTUAL CAN {rid} FW", decode_fw(rfw))
            rvals = transact(ser, bytes((COMM_FORWARD_CAN, rid, COMM_GET_VALUES)), args.timeout)
            print_dict(f"RIGHT / VIRTUAL CAN {rid} VALUES", decode_values(rvals))

    print("\nVESC UART facade probe: PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (TimeoutError, OSError, ValueError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
