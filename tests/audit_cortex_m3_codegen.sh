#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"
if ! command -v clang >/dev/null 2>&1; then
    echo "CORTEX_M3_CODEGEN_SKIP clang unavailable"
    exit 0
fi
if ! command -v nm >/dev/null 2>&1 || ! command -v objdump >/dev/null 2>&1; then
    echo "CORTEX_M3_CODEGEN_SKIP nm/objdump unavailable"
    exit 0
fi

check_reloc_range() {
    local obj="$1" start_sym="$2" end_sym="$3" label="$4"
    local reloc="${obj}.reloc"
    objdump -r "$obj" > "$reloc" || true
    local start_hex end_hex
    start_hex="$(nm -n "$obj" | awk -v s="$start_sym" '$3==s {print $1}')"
    end_hex="$(nm -n "$obj" | awk -v s="$end_sym" '$3==s {print $1}')"
    if [[ -z "$start_hex" || -z "$end_hex" ]]; then
        echo "CORTEX_M3_CODEGEN_FAIL cannot locate $label symbols" >&2
        exit 1
    fi
    local start=$((16#$start_hex)) end=$((16#$end_hex)) hot_divs=0
    while read -r addr rest; do
        [[ "$addr" =~ ^[0-9a-fA-F]+$ ]] || continue
        if grep -q '__aeabi_.*div' <<<"$addr $rest"; then
            local a=$((16#$addr))
            if (( a >= start && a < end )); then
                echo "HOTPATH_DIV_HELPER $label $addr $rest"
                hot_divs=$((hot_divs + 1))
            fi
        fi
    done < "$reloc"
    if (( hot_divs != 0 )); then
        echo "CORTEX_M3_CODEGEN_FAIL $label has $hot_divs division helper relocation(s)" >&2
        exit 1
    fi
    echo "${label}_RANGE=0x${start_hex}..0x${end_hex}"
    echo "${label}_DIV_HELPERS=0"
}

FOC_OBJ="${TMPDIR:-/tmp}/foc_motor_cortex_m3.o"
clang --target=arm-none-eabi -mcpu=cortex-m3 -mthumb -std=c11 -Os -ffreestanding \
    -I tests/cross_stub -I Src -c Src/foc_motor.c -o "$FOC_OBJ"
check_reloc_range "$FOC_OBJ" mc_foc_run_current_control current_pi_axis FAST_CURRENT_LOOP

echo "FOC_OBJECT_SIZE: $(size "$FOC_OBJ" | tail -n 1 || true)"

SENSOR_OBJ="${TMPDIR:-/tmp}/motor_sensor_cortex_m3.o"
clang --target=arm-none-eabi -mcpu=cortex-m3 -mthumb -std=c11 -Os -ffreestanding \
    -I tests/cross_stub -I Src -c Src/motor_sensor.c -o "$SENSOR_OBJ"
# Hardware-encoder hot path ends at the next public update function. Any 64-bit
# division in preparation/configuration code is allowed; it must not appear here.
check_reloc_range "$SENSOR_OBJ" MotorSensor_UpdateHardwareEncoder MotorSensor_Update FAST_ENCODER_UPDATE

echo "SENSOR_OBJECT_SIZE: $(size "$SENSOR_OBJ" | tail -n 1 || true)"

MOTOR_OBJ="${TMPDIR:-/tmp}/motor_cortex_m3.o"
clang --target=arm-none-eabi -mcpu=cortex-m3 -mthumb -std=c11 -Os -ffreestanding \
    -I tests/host_stub -I tests/cross_stub -I Src -c Src/motor.c -o "$MOTOR_OBJ"
check_reloc_range "$MOTOR_OBJ" DMA1_Channel1_IRQHandler configure_phase_override FAST_DMA_CURRENT_ISR
echo "MOTOR_ISR_OBJECT_SIZE: $(size "$MOTOR_OBJ" | tail -n 1 || true)"

echo "CORTEX_M3_CORE_OBJECT_PASS"
