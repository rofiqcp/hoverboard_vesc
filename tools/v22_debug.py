#!/usr/bin/env python3
"""
V22 complete debug runner.

Default: source/link checks + Python tests + host C tests + PlatformIO build.
Optional --port performs read-only VESC protocol probing. No motor SET commands.
"""
from __future__ import annotations
import argparse, pathlib, subprocess, sys, shutil, time

COMM_FW_VERSION=0
COMM_GET_VALUES=4
COMM_GET_MCCONF=14
COMM_GET_APPCONF=17
COMM_FORWARD_CAN=34
COMM_PING_CAN=62

def crc16(data: bytes) -> int:
    crc=0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc=((crc<<1)^0x1021)&0xFFFF if (crc&0x8000) else (crc<<1)&0xFFFF
    return crc

def make_frame(payload: bytes) -> bytes:
    n=len(payload)
    head=bytes([2,n]) if n<=255 else bytes([3,(n>>8)&255,n&255])
    c=crc16(payload)
    return head+payload+bytes([c>>8,c&255,3])

class Parser:
    def __init__(self): self.buf=bytearray()
    def feed(self,data: bytes):
        self.buf.extend(data); result=[]
        while True:
            while self.buf and self.buf[0] not in (2,3): del self.buf[0]
            if not self.buf: break
            if self.buf[0]==2:
                if len(self.buf)<2: break
                n=self.buf[1]; hdr=2
            else:
                if len(self.buf)<3: break
                n=(self.buf[1]<<8)|self.buf[2]; hdr=3
            total=hdr+n+3
            if len(self.buf)<total: break
            pkt=bytes(self.buf[:total]); del self.buf[:total]
            p=pkt[hdr:hdr+n]; got=(pkt[hdr+n]<<8)|pkt[hdr+n+1]
            if pkt[-1]==3 and got==crc16(p): result.append(p)
        return result

def run(cmd,cwd,required=True):
    print("$"," ".join(map(str,cmd)),flush=True)
    p=subprocess.run(cmd,cwd=cwd)
    if required and p.returncode: raise SystemExit(p.returncode)
    return p.returncode

def source_contract(root):
    p=(root/"src/vesc_protocol.c").read_text(errors="replace")
    h=(root/"src/vesc_protocol.h").read_text(errors="replace")
    s=(root/"src/vesc_services.c").read_text(errors="replace")
    app=(root/"src/vesc_app.c").read_text(errors="replace")
    cfg=(root/"src/vesc_config_compat.c").read_text(errors="replace")
    compact="".join(p.split())
    checks={
      "ServiceBudget definition": p.count("void VescProtocol_ServiceBudget(uint16_t max_rx_bytes)")==1,
      "RxBudgetYields definition": p.count("uint32_t VescProtocol_RxBudgetYields(void)")==1,
      "ServiceBudget declaration": "VescProtocol_ServiceBudget(uint16_t max_rx_bytes)" in h,
      "RxBudgetYields declaration": "VescProtocol_RxBudgetYields(void)" in h,
      "scheduler bounded parser call": "VescProtocol_ServiceBudget(VESC_RX_BUDGET_PER_PASS)" in s,
      "statistics budget-yield call": "VescProtocol_RxBudgetYields()" in s,
      "parser has execution budget": "while(rx_old!=pos&&consumed<max_rx_bytes)" in compact,
      "critical reply backpressure": "send_payload_critical" in p and "critical_reply_valid" in p,
      "firmware identity V22": "hoverboard-vesc6-v22" in p,
      "LEFT default CAN ID 1": "controller_id = 1U" in app or "controller_id=1U" in app,
      "RIGHT virtual CAN": "C_FORWARD_CAN" in p and "second_id()" in p,
      "GET_VALUES": "C_GET_VALUES" in p and "append_values" in p,
      "MCCONF GET/SET": "C_GET_MCCONF" in p and "C_SET_MCCONF" in p,
      "APPCONF GET/SET": "C_GET_APPCONF" in p and "C_SET_APPCONF" in p,
      "Hall detect": "C_DETECT_HALL_FOC" in p,
      "Encoder detect": "C_DETECT_ENCODER" in p,
      "Apply-all detect": "C_DETECT_APPLY_ALL_FOC" in p,
      "MCCONF 481 wire": "481" in cfg,
      "APPCONF 493 wire": "493" in cfg,
    }
    failed=[]
    for name,ok in checks.items():
        print(f"[{'PASS' if ok else 'FAIL'}] {name}")
        if not ok: failed.append(name)
    if failed: raise SystemExit("source contract failed: "+", ".join(failed))

def compile_python(root):
    files=sorted((root/"tests").glob("test_*.py"))+sorted((root/"tools").glob("*.py"))
    for f in files: run([sys.executable,"-m","py_compile",str(f)],root)
    print(f"[PASS] py_compile {len(files)} files")

def run_python_tests(root):
    tests=sorted((root/"tests").glob("test_*.py"))
    if not tests: return
    passed=0
    for f in tests:
        print(f"\\n=== PYTHON TEST: {f.name} ===",flush=True)
        run([sys.executable,str(f)],root)
        passed+=1
    print(f"[PASS] Python tests: {passed}/{len(tests)} scripts")

def run_host(root):
    sh=root/"tests"/"run_host_tests.sh"
    if sh.exists() and shutil.which("bash") and shutil.which("gcc"):
        run(["bash",str(sh)],root)
        print("[PASS] host C tests")
    else:
        print("[SKIP] host C tests (bash/gcc/script unavailable)")

def run_pio(root):
    pio=shutil.which("pio")
    if pio:
        run([pio,"run"],root)
        print("[PASS] PlatformIO compile + link")
    else:
        print("[SKIP] PlatformIO not installed")

def serial_probe(port,baud,right_id):
    try: import serial
    except ImportError: raise SystemExit("pyserial required: python3 -m pip install pyserial")
    ser=serial.Serial(port,baudrate=baud,timeout=.05,write_timeout=1)
    parser=Parser()
    def req(payload,expect,timeout=4.0):
        ser.reset_input_buffer(); ser.write(make_frame(payload)); ser.flush()
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            for p in parser.feed(ser.read(512)):
                if p and p[0]==expect: return p
        raise TimeoutError(f"VESC timeout command {expect}")
    fw=req(bytes([COMM_FW_VERSION]),COMM_FW_VERSION)
    print("[PASS] FW_VERSION",fw[:48])
    vals=req(bytes([COMM_GET_VALUES]),COMM_GET_VALUES)
    print("[PASS] GET_VALUES len",len(vals))
    ping=req(bytes([COMM_PING_CAN]),COMM_PING_CAN)
    print("[PASS] PING_CAN",list(ping))
    rfw=req(bytes([COMM_FORWARD_CAN,right_id,COMM_FW_VERSION]),COMM_FW_VERSION)
    print("[PASS] RIGHT FW_VERSION",rfw[:48])
    mc=req(bytes([COMM_GET_MCCONF]),COMM_GET_MCCONF)
    print("[PASS] MCCONF wire bytes",len(mc)-1)
    app=req(bytes([COMM_GET_APPCONF]),COMM_GET_APPCONF)
    print("[PASS] APPCONF wire bytes",len(app)-1)
    bad=bytearray(make_frame(bytes([COMM_FW_VERSION]))); bad[-3]^=0x55
    ser.write(bad); ser.flush(); time.sleep(.05)
    req(bytes([COMM_FW_VERSION]),COMM_FW_VERSION)
    print("[PASS] CRC recovery")
    ser.close()

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("root",nargs="?",default=".")
    ap.add_argument("--port")
    ap.add_argument("--baud",type=int,default=115200)
    ap.add_argument("--right-id",type=int,default=2)
    ap.add_argument("--skip-python-tests",action="store_true")
    ap.add_argument("--skip-host",action="store_true")
    ap.add_argument("--skip-pio",action="store_true")
    a=ap.parse_args(); root=pathlib.Path(a.root).resolve()
    source_contract(root); compile_python(root)
    if not a.skip_python_tests: run_python_tests(root)
    if not a.skip_host: run_host(root)
    if not a.skip_pio: run_pio(root)
    if a.port: serial_probe(a.port,a.baud,a.right_id)
    print("[PASS] V22 debug complete")
if __name__=="__main__": main()
