# V24 — Upstream VESC Path Map, Call Chain, Safety & Compatibility Audit

Date: 2026-08-14
Baseline: hoverboard_vescv23.zip
Reference: https://github.com/vedderb/bldc (master inspected 2026-08-14)

## 1. Upstream VESC paths relevant to this port

### UART/protocol and command dispatch
- `comm/packet.c` — VESC frame length/CRC parser and packet sender.
- `comm/commands.c` — `COMM_*` command dispatcher, GET_VALUES, SET_DUTY/CURRENT/BRAKE/RPM/POS/HANDBRAKE, config and detect commands.
- `datatypes.h` — `COMM_PACKET_ID`, `mc_configuration`, sensor/motor enums and protocol datatypes.

### Motor command interface
- `motor/mc_interface.c` — public motor API used by `commands.c`.
- `motor/mcpwm_foc.c` — FOC implementation called from `mc_interface.c` for FOC motors.

### Sensor / detection / configuration
- `encoder/encoder.c` — ABI/encoder configuration and reading.
- `conf_general.c` / `conf_general.h` — config persistence plus motor/sensor detect/autodetect entry points.
- generated configuration serializer/deserializer in upstream build is the model for exact VESC Tool wire layout.

## 2. Upstream command call chains

These are semantic call maps, not copied source.

- `COMM_SET_DUTY` -> `mc_interface_set_duty()` -> FOC motor path in `mcpwm_foc`.
- `COMM_SET_CURRENT` -> `mc_interface_set_current()` -> `mcpwm_foc_set_current()`.
- `COMM_SET_CURRENT_BRAKE` -> `mc_interface_set_brake_current()` -> `mcpwm_foc_set_brake_current()`.
- `COMM_SET_RPM` -> `mc_interface_set_pid_speed()` -> `mcpwm_foc_set_pid_speed()`.
- `COMM_SET_POS` -> `mc_interface_set_pid_pos()` -> direction/encoder normalization -> `mcpwm_foc_set_pid_pos()`.
- `COMM_SET_HANDBRAKE` -> `mc_interface_set_handbrake()` -> `mcpwm_foc_set_handbrake()`.
- `COMM_GET_VALUES` -> `mc_interface_read_reset_avg_motor_current/input_current/id/iq/vd/vq()` plus duty/RPM/voltage/tach/position/fault.
- `COMM_SET_MCCONF` -> deserialize full mcconf -> enforce hardware limits -> store config -> apply config -> reply ACK.

## 3. Local hoverboard V24 paths and call chains

### 3.1 Serial receive

`src/vesc_protocol.c`

`USART3 RX DMA Channel3`
-> `VescProtocol_Service()`
-> `VescPacket_Feed()` (`src/vesc_packet.c`)
-> `rx_cb()`
-> `process_ctx(... LEFT ...)`
-> optional `COMM_FORWARD_CAN`
-> `process_ctx(... RIGHT ...)`
-> command-specific handler.

DMA RX transfer error or disabled DMA:

`VescProtocol_Service()`
-> `RuntimeControl_VescReleaseAll()`
-> cancel sensor calibration/alignment/homing
-> disable both motor outputs
-> `VescProtocol_Init()`
-> restart USART3 DMA parser.

### 3.2 Duty

`COMM_SET_DUTY`
-> `set_duty()`
-> normalize [-1..1] to [-1000..1000]
-> `RuntimeControl_VescSetOne(... ESC_MODE_DUTY ...)`
-> `RuntimeControl_OnCommand()`
-> `try_arm_motor()`
-> `mc_foc_set_control_mode(CONTROL_MODE_DUTY)`
-> ISR/core `foc_motor.c`
-> inverse Park/SVPWM
-> TIM PWM.

Special V24: `SET_DUTY(0)` is kept as active zero-duty/full-brake state. If a target encoder alignment owns the bridge, it is cancelled before entering full brake.

### 3.3 Current / STOP

Non-zero:

`COMM_SET_CURRENT`
-> `set_current()`
-> `RuntimeControl_VescSetOne(... ESC_MODE_TRQ ...)`
-> current-Iq target
-> current PI
-> Vd/Vq
-> SVPWM.

Zero / STOP:

`COMM_SET_CURRENT(0)`
-> `set_current()`
-> **`RuntimeControl_VescStopOne()`**
-> abort target calibration/alignment
-> clear arm request/setpoint
-> `disarm_motor()`
-> MOE/output off.

This bypasses the normal CONTROL path so STOP cannot be ignored by `ALIGNMENT_BUSY`.

### 3.4 Current brake

`COMM_SET_CURRENT_BRAKE`
-> `set_brake()`
-> `RuntimeControl_VescSetOne(... ESC_MODE_BRAKE ...)`
-> `CONTROL_MODE_CURRENT_BRAKE`
-> Iq sign selected to oppose measured speed
-> current PI -> Vd/Vq -> SVPWM.

### 3.5 Handbrake

`COMM_SET_HANDBRAKE`
-> `set_handbrake()`
-> `RuntimeControl_VescSetOne(... ESC_MODE_HANDBRAKE ...)`
-> `CONTROL_MODE_HANDBRAKE`
-> FOC handbrake behavior.

### 3.6 RPM

`COMM_SET_RPM`
-> `set_rpm()`
-> eRPM conversion using physical pole-pairs
-> `RuntimeControl_VescSetOne(... ESC_MODE_SPD ...)`
-> `CONTROL_MODE_SPEED`
-> speed PI -> Iq target
-> current PI -> Vd/Vq -> SVPWM.

### 3.7 Position

`COMM_SET_POS`
-> `set_pos()`
-> `RuntimeControl_PositionTargetTicks()`
-> `RuntimeControl_VescSetOne(... ESC_MODE_POS ...)`
-> `CONTROL_MODE_POS`
-> position PID -> speed/Iq target
-> current loop -> Vd/Vq -> SVPWM.

Tester V24 position fail-fast: if >=0.6 A Iq is built while position moves <0.5 degree for >=0.15 s, send repeated STOP immediately and end that test.

## 4. Telemetry call chain

`COMM_GET_VALUES / COMM_GET_VALUES_SELECTIVE`
-> `append_values()`
-> `current_avg_take()`
-> standard VESC fields:
  - FET temperature
  - motor current
  - input/battery current
  - Id
  - Iq
  - duty
  - eRPM
  - input voltage
  - tachometer / absolute tachometer
  - fault
  - rotor/mechanical position
  - controller ID
  - Vd
  - Vq
  - timeout status.

When bridge is OFF, Id/Iq/Imotor/Ibattery/Vd/Vq remain present and finite but are reported as valid zero rather than stale active-control values. Rotor position remains readable from the sensor observer.

`COMM_ROTOR_POSITION`
-> `rotor_position()` / periodic stream mode
-> `RuntimeControl` sensor position
-> VESC float position reply.

## 5. Config without wire truncation

`src/vesc_config_compat.c`

- exact VESC 6.00 MCCONF wire footprint: 481 bytes.
- exact VESC 6.00 APPCONF wire footprint: 493 bytes.
- implemented fields are parsed, range-checked and converted to STM32 fixed-point config.
- V24 keeps exact full-size shadows for LEFT MCCONF, RIGHT MCCONF and local APPCONF so unsupported UI fields are not silently collapsed after write/readback.
- one dedicated 2-KiB flash page at `0x0803E800..0x0803EFFF` persists those shadows across MCU reset/power-cycle; the application linker region is reduced from 252 KiB to 250 KiB, while the existing 4-KiB emulated EEPROM remains at `0x0803F000..0x0803FFFF`.
- the flash shadow carries magic/version/flags/CRC32 and is verified after programming.
- firmware-side detect invalidates the corresponding MCCONF shadow before publishing detected live config.
- identical SET_MCCONF no longer performs a flash erase/program; it is an idempotent ACK-only transaction.

Unsupported VESC-only fields are preserved for VESC Tool round-trip/persistence but are still not executed by the fixed-point control core unless this port has an explicit implementation for them. Implemented tuning fields continue to persist through `RuntimeSettings_Save()`.

## 6. Hall detect

`COMM_DETECT_HALL_FOC`
-> `start_detect()`
-> `RuntimeControl_VescStartSensorDetect(... HALL ...)`
-> `sensor_cal_*` state machine
-> observe Hall raw states and electrical sweep
-> validate legal sequence/LUT
-> apply candidate
-> persist
-> `detect_service()` replies.

RIGHT is physically constrained to Hall.

## 7. Encoder detect

`COMM_DETECT_ENCODER`
-> `start_detect()`
-> `RuntimeControl_VescStartSensorDetect(... ENCODER_AB ...)`
-> encoder probe state machine
-> count valid/invalid quadrature transitions
-> direction score normal vs inverted
-> estimate electrical ratio
-> compare probe ratio with authoritative physical motor pole-pairs
-> commit encoder inversion/ratio
-> electrical phase alignment
-> persist/reply.

V24 change for the captured board:
- strong quadrature evidence (409 valid / 0 invalid in supplied log) and decisive direction score (0 vs 364) can prove encoder direction even if sequence-start bookkeeping does not produce a complete four-state cycle;
- physical motor pole-pairs, not an old detected `encoder_ratio`, are the reference for +/-1 ratio tolerance.

## 8. Full integrated autodetect

`COMM_DETECT_APPLY_ALL_FOC`
-> `auto_detect_start()`
-> verify/request current-offset calibration
-> RIGHT Hall detect
-> LEFT ABI encoder detect
-> LEFT electrical sync/alignment/homing readiness
-> `RuntimeSettings_Save()`
-> terminal reply from `auto_detect_service()`.

This port's integrated detect is board/sensor commissioning. It is not a claim that every upstream VESC motor-model measurement (R/L/flux etc.) has been ported to this fixed-point STM32F103 core.

## 9. Serial/reset fail-safe

Firmware:
- startup PWM timers have MOE disabled in `src/setup.c`;
- runtime motor enable mask starts at zero;
- RX DMA transport failure immediately invokes `RuntimeControl_VescReleaseAll()`;
- global release now also aborts active sensor calibration, encoder alignment and homing before outputs are disabled.

Host tester:
- `SerialVesc.emergency_recover()` tries to reopen the port only to transmit STOP repeatedly;
- V24 latches `motion_abort` on SerialException/OSError/EIO/device-disconnected;
- after this latch, all later motion tests are SKIP, even if the port reconnects;
- final diagnostics may still run if communication recovers.

A true MCU brownout/reset cannot be prevented in software. If USB/UART disappears because the power rail collapses, also inspect 5-V/3.3-V rails, ground bounce, gate-driver supply, bootstrap, MOSFET switching noise and USB isolation. Firmware ensures reset starts with MOE off; hardware must ensure the MCU/driver rails remain valid.
