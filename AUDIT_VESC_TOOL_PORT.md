# Audit — Hoverboard Dual Motor VESC Tool Port

## Scope

Final port target:

- STM32F103 hoverboard dual inverter.
- LEFT local VESC node, Hall/Encoder selectable.
- RIGHT Hall-only virtual-CAN VESC node.
- USART3 PB10/PB11 permanent VESC UART transport.
- PA2/PA3 VESC APP ADC1/ADC2.
- APP_UART, APP_ADC, APP_ADC_UART.
- no RTOS.
- fixed-point fast FOC and sensor hot path.

## Compatibility baseline

The compatibility facade is frozen to VESC **6.00** packet/config layout. The port implements VESC framing, CRC16, selected COMM commands, exact tested MCCONF/APPCONF wire footprint and virtual-CAN forwarding semantics.

Configuration wire audit:

```text
MCCONF size      481
MCCONF signature 776184161
APPCONF size      493
APPCONF signature 486554156
```

## Resource audit

### PWM / current loop

```text
LEFT PWM  TIM8
RIGHT PWM TIM1
ADC1 regular conversions = 3
ADC2 regular conversions = 3
ADC1+ADC2 dual regular simultaneous
DMA1 CH1 current ISR ~16 kHz
```

PA2/PA3 are **not** appended to the regular current scan.

### Slow/injected ADC

```text
ADC2 JDR1 = PA2 / APP ADC1
ADC2 JDR2 = PA3 / APP ADC2
trigger divider = 32 current samples ~= 500 Hz

ADC1 injected = battery + board temperature
slow housekeeping rate ~= 10 Hz
```

The injected conversion is launched after current DMA completion and read from background service when JEOC is ready. This preserves the regular current sequence length. Interaction with the exact physical board timing still requires scope/bench verification.

### UART

```text
USART3 PB10/PB11
RX DMA1 CH3 circular
TX DMA1 CH2 queued
115200 8N1 fixed
```

USART2 is not initialized. PA2/PA3 therefore remain ADC resources.

### Sensors

```text
LEFT Hall    PB5/PB6/PB7
LEFT Encoder PB6 TIM4_CH1 A
             PB7 TIM4_CH2 B
             PB5 EXTI5 Z
RIGHT Hall   PC10/PC11/PC12
```

No right hardware-encoder path exists in `motor.c`. VESC writes and EEPROM migration both force RIGHT Hall mode.

## Virtual CAN audit

```text
local_id = N, allowed 0..253
right_id = N + 1
```

`COMM_PING_CAN` returns the RIGHT ID. `COMM_FORWARD_CAN` addressed to that ID calls the same command dispatcher with RIGHT context. No physical CAN peripheral is referenced by the protocol layer.

RIGHT gets a distinct firmware UUID view by incrementing the final UUID byte.

## VESC protocol audit

Implemented:

```text
FW_VERSION
GET_VALUES / SELECTIVE
GET_VALUES_SETUP / SELECTIVE
SET_DUTY
SET_CURRENT
SET_CURRENT_BRAKE
SET_HANDBRAKE
SET_RPM
SET_POS
SET_CURRENT_REL
ALIVE
ROTOR_POSITION
GET_DECODED_ADC
GET/SET_MCCONF
GET/SET_APPCONF
PING_CAN
FORWARD_CAN
```

Packet parser tests include:

- short frame;
- >255-byte long frame;
- CRC rejection;
- noise resynchronization;
- the special VESC `0x03` case where long-frame start and normal frame stop share the same byte value.

## APP ADC audit

APP ADC uses fixed-point input normalization/filtering and command ramp.

Safety decisions:

- safe-start implemented;
- invalid/degenerate center ranges do not divide by zero;
- button-based control modes rejected;
- unsupported ADC throttle expo forced zero;
- unsupported ADC traction control forced zero;
- update rate capped at 500 Hz;
- UART control command overrides ADC for 300 ms;
- `multi_esc` can command both physical motors.

## Fixed-point/codegen audit

`tests/audit_cortex_m3_codegen.sh` cross-compiles the critical objects for Cortex-M3 and scans relocations.

Final result:

```text
FAST_CURRENT_LOOP_DIV_HELPERS=0
FAST_ENCODER_UPDATE_DIV_HELPERS=0
CORTEX_M3_CORE_OBJECT_PASS
```

The LEFT hardware-encoder batch phase update is O(1) with precomputed reciprocal/remainder logic. Runtime division helpers may exist in preparation/configuration code, but not in the audited fast encoder update region.

## Host test audit

Final test suite:

```text
[1/11] strict FOC/sensor/ISR/runtime syntax
[2/11] sensor/FOC behavioral tests
[3/11] outer PID + MTPA/FW numeric/behavior
[4/11] SVPWM numeric comparison
[5/11] Q14 trig full phase accuracy
[6/11] VESC packet framing/CRC/parser
[7/11] VESC 6.00 config wire footprint/safety
[8/11] VESC APP ADC fixed-point behavior
[9/11] VESC compatibility strict syntax
[10/11] resource/pin/ADC contract
[11/11] UBSan stress
```

Result:

```text
ALL_HOST_TESTS_PASS
```


## Runtime-control audit round 2 (Id/Iq, commissioning, commands, fault-stop)

### Observation is independent from actuation

The ADC/current transform no longer returns early when PWM/MOE is disabled. The fast loop always performs current reconstruction, Clarke/Park and publishes measured `Id`/`Iq`; only PI actuation/SVPWM is gated. Therefore VESC `motor current`, `Id`, `Iq`, RPM and position remain observable while the bridge is DISARMED. A dedicated host test verifies non-zero current observation with PWM equal to zero.

VESC `motor current` is reported as signed vector magnitude `sqrt(Id^2 + Iq^2)` with the sign of Iq. DC/input current remains the physical DC-current channel.

### VESC connect/run semantics

A UART/VESC Tool connection does not energize either inverter. `FW_VERSION`, `GET_VALUES`, config reads and telemetry are observation only. `SET_DUTY`, `SET_CURRENT`, `SET_RPM` or `SET_POS` create a run request; the hardware gate opens only when the target motor passes sensor proof, alignment, setpoint and fault checks. Near-zero duty/current/RPM releases the run request.

### Hall detection

`COMM_DETECT_HALL_FOC` (28) is implemented for LEFT/local and RIGHT/virtual-CAN. The commissioning path:

1. disables normal actuation;
2. performs a soft D-axis alignment;
3. sweeps positive electrical phase;
4. records raw 3-bit Hall state, sector votes and directed transitions;
5. rejects missing/ambiguous/reversed/noisy sequences;
6. commits only a FOC-positive six-state mapping;
7. stores the result to EEPROM and verifies the saved image;
8. returns the 8-byte VESC Hall table using the upstream 0..200 electrical-angle convention (255 invalid).

### LEFT encoder detection

`COMM_DETECT_ENCODER` (27) is implemented only for LEFT. RIGHT deliberately returns detection failure because the hardware contract is Hall-only.

For VESC detection the physical encoder CPR/counts must already be configured. The detector then performs a known electrical sweep and measures encoder motion to infer:

- electrical/mechanical ratio = motor pole-pairs;
- raw A/B direction/inversion;
- valid four-state quadrature sequence.

The inferred pole-pair ratio and inversion are committed and persisted. The two formerly-reserved v16 EEPROM words now store LEFT/RIGHT pole-pair counts without changing `NB_OF_VAR` or breaking older v16 images; zero in an old image falls back to compiled safe defaults.

For incremental A/B, persistent absolute electrical angle cannot be guaranteed without an absolute reference. The VESC detect response therefore returns offset zero and the firmware performs a 450 ms electrical alignment on the first closed-loop run after each power cycle. Mechanical position is not zeroed by that alignment. Z/index remains available on PB5 but is not required for AB operation.

### Detect watchdog

VESC blocking detect commands may take many seconds. The asynchronous bare-metal bridge now refreshes the VESC watchdog for the duration of the pending detect transaction, so a normal APP timeout cannot abort commissioning midway.

### Set-command paths

- `SET_DUTY` -> dedicated DUTY mode -> direct q-axis modulation request, not the old voltage-mode alias.
- `SET_CURRENT` -> torque/current outer target -> Iq request.
- `SET_RPM` -> VESC ERPM converted to mechanical normalized speed target -> speed PI -> Iq.
- `SET_POS` -> nearest mechanical target on the current revolution basis -> position controller -> speed/current cascade.

All four commands are still intentionally refused if closed-loop sensor proof is absent. LEFT Encoder additionally performs first-run electrical alignment after reboot.

### 3-second fault-stop / re-run

Persistent DC overcurrent is qualified in the 16 kHz ISR (8 consecutive samples, about 0.5 ms) before being promoted to a controller fault. PWM/MOE is removed immediately. The slow loop latches a 3000 ms fault-stop window. ISR overrun and persistent overcurrent are now latched even if the output was already OFF or commissioning was active, preventing a permanent uncleared ISR mask. Commissioning aborts on such a fault.

After 3000 ms the explicit ISR latch is cleared. The motor does **not** autonomously restart with no command. A new or still-streaming VESC control command may request run again, and the motor re-enables only if the underlying sensor/current condition is healthy. This mirrors the safe VESC command-after-fault-stop behavior rather than implementing a separate manual ARM-release handshake.

## Deliberate limitations

The following are deliberately not presented as working functionality:

- no physical CAN bus;
- no VESC bootloader/update command;
- no RIGHT encoder mode;
- no sensorless/HFI/observer compatibility layer;
- no ADC digital-button modes;
- no ADC throttle expo;
- no ADC traction-control algorithm;
- no calibrated motor-temperature sensor unless hardware is added;
- no accumulated Ah/Wh/odometer implementation;
- Vd/Vq engineering-volts telemetry is returned zero until scaling is validated;
- timeout brake-current setting is stored for VESC schema compatibility, but timeout behavior remains the firmware's safe release/disarm rather than active braking;
- GET_*_DEFAULT returns safe current hardware representation rather than full upstream factory default semantics.

## Verification boundary

The environment used to create this package does not contain `arm-none-eabi-gcc`, PlatformIO, STM32 hardware, VESC Tool GUI, or a connected power stage. Internet access from the container was unavailable, so target toolchains could not be installed.

Therefore the following are **not claimed**:

- final STM32Cube/PlatformIO link success;
- successful ST-Link flash;
- physical ADC injected timing margin;
- physical motor rotation/current calibration;
- a real VESC Tool GUI connection.

What is verified in this package is source-level architecture, host compilation/tests, VESC wire/config internal tests, static resource contract, and Cortex-M3 hot-path code generation. Hardware acceptance must follow `README_VESC_TOOL.md`.
