#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
CC="${CC:-gcc}"
CFLAGS=(-std=c11 -Wall -Wextra -Werror)

echo "[1/29] strict syntax: FOC + sensor + ISR + runtime integration"
"$CC" "${CFLAGS[@]}" -I tests/host_stub -I Src -fsyntax-only \
  Src/foc_motor.c Src/foc_motor_data.c Src/motor_sensor.c \
  Src/motor.c Src/runtime_control.c Src/util.c
"$CC" "${CFLAGS[@]}" -I tests/encoder_stub -I Src -fsyntax-only Src/left_encoder.c

echo "[2/29] sensor/FOC behavioral tests"
"$CC" "${CFLAGS[@]}" -I Src tests/test_foc_sensor.c \
  Src/foc_motor.c Src/foc_motor_data.c Src/motor_sensor.c \
  -o /tmp/test_foc_sensor
/tmp/test_foc_sensor

echo "[3/29] VESC outer PID + MTPA + FW numeric/behavior tests"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_vesc_advanced.c \
  Src/foc_motor.c Src/foc_motor_data.c -lm -o /tmp/test_vesc_advanced
/tmp/test_vesc_advanced

echo "[4/29] SVM numeric comparison against VESC foc_svm equations"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_svm_vs_vesc.c \
  Src/foc_motor_data.c -o /tmp/test_svm_vs_vesc
/tmp/test_svm_vs_vesc

echo "[5/29] Q14 trig LUT accuracy"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_trig_accuracy.c \
  Src/foc_motor_data.c -lm -o /tmp/test_trig_accuracy
/tmp/test_trig_accuracy

echo "[6/29] VESC packet framing + CRC + parser"
"$CC" "${CFLAGS[@]}" -I Src tests/test_vesc_packet.c \
  Src/vesc_packet.c Src/vesc_buffer.c -lm -o /tmp/test_vesc_packet
/tmp/test_vesc_packet

echo "[7/29] VESC 6.00 configuration wire footprint + safety policy"
"$CC" "${CFLAGS[@]}" -I tests/host_stub -I Src tests/test_vesc_config_wire.c \
  Src/vesc_buffer.c Src/vesc_config_compat.c -lm -o /tmp/test_vesc_config_wire
/tmp/test_vesc_config_wire

echo "[8/29] VESC APP ADC fixed-point behavior"
"$CC" "${CFLAGS[@]}" -I tests/host_stub -I Src tests/test_vesc_app.c \
  Src/vesc_app.c -o /tmp/test_vesc_app
/tmp/test_vesc_app

echo "[9/29] VESC compatibility modules strict syntax"
"$CC" "${CFLAGS[@]}" -I tests/host_stub -I Src -fsyntax-only \
  Src/vesc_buffer.c Src/vesc_packet.c Src/vesc_app.c Src/vesc_config_compat.c \
  Src/vesc_protocol.c Src/esc_protocol_compat.c

echo "[10/29] static resource/pin/ADC contract"
python3 tests/test_resource_contract.py

echo "[11/29] robust current-zero policy incl. single-spike tolerance"
python3 tests/test_current_cal_robust.py

echo "[12/29] UBSan deterministic stress incl. MTPA/FW"
"$CC" -O1 -g "${CFLAGS[@]}" -fsanitize=undefined \
  -fno-sanitize-recover=undefined -I Src tests/test_foc_ubsan.c \
  Src/foc_motor.c Src/foc_motor_data.c Src/motor_sensor.c \
  -o /tmp/test_foc_ubsan
/tmp/test_foc_ubsan

echo "[13/29] Python tester inventory bootstrap (no HBTS circular dependency)"
python3 tests/test_tester_inventory.py

echo "[14/29] VESC facade command/tester coverage contract"
python3 tests/test_tester_command_coverage.py

echo "[15/29] V12 commissioning + standard-wire regression contract"
python3 tests/test_v12_commissioning_contract.py

echo "[16/29] V13 hardware-log regression contract"
python3 tests/test_v13_hardware_regressions.py

echo "[17/29] V14 latest hardware-log regression contract"
python3 tests/test_v14_latest_hardware_regressions.py

echo "[18/29] V14 post-detect ARM/SET contract"
python3 tests/test_v14_post_detect_arm_contract.py

echo "[19/29] V15 VESC SET/GET + V1 ISR contract"
python3 tests/test_v15_vescflow_v1isr.py

echo "[20/29] V16 serial/main-loop liveness contract"
python3 tests/test_v16_serial_liveness.py

echo "[21/29] V17 units/config/speed-direction contract"
python3 tests/test_v17_units_config_speed_contract.py

echo "[22/29] V18 current/brake/handbrake fast-loop behavior"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_v18_current_modes.c \
  Src/foc_motor.c Src/foc_motor_data.c -o /tmp/test_v18_current_modes
/tmp/test_v18_current_modes

echo "[23/29] V18 RPM/current/brake architecture contract"
python3 tests/test_v18_rpm_current_brake_contract.py

echo "[24/29] V18 exact encoder/Hall RPM scaling at 16-kHz sensor cadence"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_v18_rpm_scaling.c \
  Src/motor_sensor.c -o /tmp/test_v18_rpm_scaling
/tmp/test_v18_rpm_scaling

echo "[25/29] V19 encoder electrical-sync numeric invariant"
"$CC" -O2 "${CFLAGS[@]}" -I Src tests/test_v19_encoder_sync.c \
  Src/motor_sensor.c -o /tmp/test_v19_encoder_sync
/tmp/test_v19_encoder_sync

echo "[26/29] V19 integrated position/sync/homing/EEPROM/protocol contract"
python3 tests/test_v19_integrated_position_sync_home.py

echo "[27/29] V19 DUTY modulation behavior at 3% and 90%"
"$CC" -O2 "${CFLAGS[@]}" -I tests/host_stub -I Src tests/test_v19_duty_behavior.c \
  Src/foc_motor.c Src/foc_motor_data.c -lm -o /tmp/test_v19_duty_behavior
/tmp/test_v19_duty_behavior

echo "[28/29] V20 LEFT/position/rotor/VdVq/protocol contract"
python3 tests/test_v20_left_position_rotor_vdq_protocol.py

echo "[29/29] V21 19:23 hardware-log regression contract"
python3 tests/test_v21_hwlog_192318.py

echo "ALL_HOST_TESTS_PASS"
