#!/usr/bin/env python3
"""Deep validation/debug runner for the main-owned non-RTOS VESC scheduler.

Default actions:
  1. deep static architecture contract
  2. py_compile all Python tests/tools
  3. run every test_v22_deep_*.py
  4. run host C test script when available
  5. run `pio run` compile+link when PlatformIO is installed

--all-repo-tests additionally executes every tests/test_*.py one by one.
--port adds a READ-ONLY VESC serial probe and boot-liveness watch. It never sends
motor SET commands or starts detect/calibration.
"""
from __future__ import annotations

import argparse
import pathlib
import shutil
import subprocess
import sys
import time

COMM_FW_VERSION = 0
COMM_GET_VALUES = 4
COMM_TERMINAL_CMD = 20
COMM_PRINT = 21
COMM_GET_MCCONF = 14
COMM_GET_APPCONF = 17
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
    n = len(payload)
    if n <= 255:
        head = bytes((2, n))
    else:
        head = bytes((3, (n >> 8) & 0xFF, n & 0xFF))
    c = crc16(payload)
    return head + payload + bytes((c >> 8, c & 0xFF, 3))


class Parser:
    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        self.buf.extend(data)
        out: list[bytes] = []
        while True:
            while self.buf and self.buf[0] not in (2, 3):
                del self.buf[0]
            if not self.buf:
                break
            if self.buf[0] == 2:
                if len(self.buf) < 2:
                    break
                n, hdr = self.buf[1], 2
            else:
                if len(self.buf) < 3:
                    break
                n, hdr = (self.buf[1] << 8) | self.buf[2], 3
            total = hdr + n + 3
            if len(self.buf) < total:
                break
            pkt = bytes(self.buf[:total])
            del self.buf[:total]
            payload = pkt[hdr:hdr + n]
            got = (pkt[hdr + n] << 8) | pkt[hdr + n + 1]
            if pkt[-1] == 3 and got == crc16(payload):
                out.append(payload)
        return out


def run(cmd: list[str], cwd: pathlib.Path, required: bool = True) -> int:
    print("$", " ".join(map(str, cmd)), flush=True)
    p = subprocess.run(cmd, cwd=cwd)
    if required and p.returncode:
        raise SystemExit(p.returncode)
    return p.returncode


def source_contract(root: pathlib.Path) -> None:
    test = root / "tests" / "test_v22_deep_main_scheduler.py"
    if not test.exists():
        raise SystemExit(f"missing deep contract: {test}")
    run([sys.executable, str(test)], root)


def py_compile_all(root: pathlib.Path) -> None:
    files = sorted((root / "tests").glob("*.py")) + sorted((root / "tools").glob("*.py"))
    for p in files:
        run([sys.executable, "-m", "py_compile", str(p)], root)
    print(f"[PASS] py_compile {len(files)} Python files")


def run_deep_tests(root: pathlib.Path) -> None:
    tests = sorted((root / "tests").glob("test_v22_deep_*.py"))
    for p in tests:
        print(f"\n=== DEEP TEST: {p.name} ===", flush=True)
        run([sys.executable, str(p)], root)
    print(f"[PASS] deep tests {len(tests)}/{len(tests)}")


def run_all_repo_tests(root: pathlib.Path) -> None:
    tests = sorted((root / "tests").glob("test_*.py"))
    failed: list[str] = []
    for p in tests:
        print(f"\n=== REPO TEST: {p.name} ===", flush=True)
        rc = run([sys.executable, str(p)], root, required=False)
        if rc:
            failed.append(p.name)
    if failed:
        raise SystemExit("repo regression failures: " + ", ".join(failed))
    print(f"[PASS] all repository Python tests {len(tests)}/{len(tests)}")


def run_host_tests(root: pathlib.Path) -> None:
    script = root / "tests" / "run_host_tests.sh"
    if script.exists() and shutil.which("bash") and shutil.which("gcc"):
        run(["bash", str(script)], root)
        print("[PASS] host C tests")
    else:
        print("[SKIP] host C tests: tests/run_host_tests.sh, bash or gcc unavailable")


def run_pio(root: pathlib.Path) -> None:
    pio = shutil.which("pio")
    if not pio:
        print("[SKIP] PlatformIO not installed in PATH")
        return
    run([pio, "run"], root)
    print("[PASS] PlatformIO compile + link")
    nm = shutil.which("arm-none-eabi-nm")
    elf = root / ".pio" / "build" / "esc" / "firmware.elf"
    if nm and elf.exists():
        p = subprocess.run([nm, str(elf)], cwd=root, text=True, capture_output=True, check=False)
        symbols = p.stdout
        required = [
            "VescProtocol_ServiceRxBudget",
            "VescProtocol_ServiceBlocking",
            "VescProtocol_ServiceSampleSend",
            "VescProtocol_ServicePeriodic",
            "VescProtocol_RxBudgetYields",
        ]
        missing = [s for s in required if s not in symbols]
        if missing:
            raise SystemExit("ELF symbol check failed: " + ", ".join(missing))
        print("[PASS] ELF protocol scheduler symbols")


def serial_probe(port: str, baud: int, right_id: int, watch_seconds: float) -> None:
    try:
        import serial  # type: ignore
    except ImportError:
        raise SystemExit("pyserial required: python3 -m pip install pyserial")

    ser = serial.Serial(port, baudrate=baud, timeout=0.04, write_timeout=1)
    parser = Parser()

    def request(payload: bytes, expected: int, timeout: float = 3.0) -> bytes:
        ser.write(frame(payload))
        ser.flush()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for p in parser.feed(ser.read(512)):
                if p and p[0] == expected:
                    return p
        raise TimeoutError(f"timeout waiting COMM {expected}")

    def terminal(command: str, timeout: float = 2.0) -> str:
        ser.write(frame(bytes((COMM_TERMINAL_CMD,)) + command.encode("ascii")))
        ser.flush()
        end = time.monotonic() + timeout
        lines: list[str] = []
        while time.monotonic() < end:
            for p in parser.feed(ser.read(512)):
                if p and p[0] == COMM_PRINT:
                    lines.append(p[1:].decode("utf-8", errors="replace"))
                    joined = "".join(lines)
                    if "HB BOOT" in joined and "HB ISR" in joined:
                        return joined
        return "".join(lines)

    ser.reset_input_buffer()
    fw = request(bytes((COMM_FW_VERSION,)), COMM_FW_VERSION)
    print("[PASS] FW_VERSION", fw[:64])

    status = terminal("hb_boot_status")
    print("[BOOT]", status.strip() or "no terminal status reply")

    # Watch exactly the time region where the old firmware appeared to hang:
    # startup melody -> delayed current calibration -> full main/FOC runtime.
    start = time.monotonic()
    n = 0
    while time.monotonic() - start < watch_seconds:
        try:
            fw = request(bytes((COMM_FW_VERSION,)), COMM_FW_VERSION, timeout=1.0)
            n += 1
            if n % 2 == 0:
                s = terminal("hb_boot_status", timeout=0.8)
                print(f"[LIVE {time.monotonic()-start:4.1f}s]", s.strip() or fw[:32])
        except TimeoutError as exc:
            print(f"[FAIL] boot liveness lost at {time.monotonic()-start:.2f}s: {exc}")
            print("If the board resets, reconnect and run hb_boot_status; watchdog/CPU fault boots enter degraded mode.")
            raise
        time.sleep(0.25)
    print(f"[PASS] boot liveness watch: {n} FW_VERSION replies over {watch_seconds:.1f}s")

    vals = request(bytes((COMM_GET_VALUES,)), COMM_GET_VALUES)
    print("[PASS] GET_VALUES len", len(vals))
    ping = request(bytes((COMM_PING_CAN,)), COMM_PING_CAN)
    print("[PASS] PING_CAN", list(ping))
    rfw = request(bytes((COMM_FORWARD_CAN, right_id, COMM_FW_VERSION)), COMM_FW_VERSION)
    print("[PASS] RIGHT virtual-CAN FW_VERSION", rfw[:48])
    mc = request(bytes((COMM_GET_MCCONF,)), COMM_GET_MCCONF, timeout=4.0)
    print("[PASS] MCCONF bytes", len(mc) - 1)
    app = request(bytes((COMM_GET_APPCONF,)), COMM_GET_APPCONF, timeout=4.0)
    print("[PASS] APPCONF bytes", len(app) - 1)

    # Parser recovery: intentionally bad CRC, then a valid read. No motor command.
    bad = bytearray(frame(bytes((COMM_FW_VERSION,))))
    bad[-3] ^= 0x55
    ser.write(bad)
    ser.flush()
    time.sleep(0.05)
    request(bytes((COMM_FW_VERSION,)), COMM_FW_VERSION)
    print("[PASS] bad-CRC parser recovery")
    ser.close()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", nargs="?", default=".")
    ap.add_argument("--all-repo-tests", action="store_true")
    ap.add_argument("--skip-host", action="store_true")
    ap.add_argument("--skip-pio", action="store_true")
    ap.add_argument("--port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--right-id", type=int, default=2)
    ap.add_argument("--watch-seconds", type=float, default=6.0)
    a = ap.parse_args()
    root = pathlib.Path(a.root).resolve()

    source_contract(root)
    py_compile_all(root)
    run_deep_tests(root)
    if a.all_repo_tests:
        run_all_repo_tests(root)
    if not a.skip_host:
        run_host_tests(root)
    if not a.skip_pio:
        run_pio(root)
    if a.port:
        serial_probe(a.port, a.baud, a.right_id, a.watch_seconds)

    print("[PASS] V22 deep debug complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
