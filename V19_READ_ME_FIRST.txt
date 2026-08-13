HOVERBOARD VESC V19 - READ ME FIRST
==================================

Release focus
-------------
V19 integrates the remaining hardware findings from the 2026-08-13 10:37:38
run without changing the current-offset method or ISR cadence that already work.

1. LEFT encoder electrical sync is corrected.
   The FOC reader uses:
       theta_e = raw_encoder_e - electrical_zero + configured_offset
   Therefore after locking the rotor at desired phase 0:
       electrical_zero = raw_encoder_e + configured_offset
   V18 stored raw_encoder_e alone, so a non-zero configured offset was applied
   again after sync. Current could regulate while torque/position were wrong.

2. Physical motor pole-pairs and encoder ratio are independent.
   Stock hoverboard geometry remains:
       15 pole-pairs = 30 total magnetic poles
   VESC Tool:
       Motor Poles        = 30
       FOC Encoder Ratio  = 15
   Do not enter Motor Poles=15; that is an odd total-pole count, not 15 pairs.
   Terminal helper: hb_set_pole_pairs 15

3. Position is bounded 0..360 degrees; 0 and 360 are distinct endpoints.
   Before mechanical homing: one sensor mechanical revolution maps to 0..360.
   After hard-stop calibration + homing: the measured right-to-left mechanical
   span maps to logical 0..360.

4. Integrated board sensor commissioning:
       current offset calibration
       -> LEFT incremental encoder detect
       -> RIGHT Hall detect
       -> LEFT encoder electrical sync
       -> EEPROM save
   This is board-specific sensor commissioning, not the full upstream VESC
   R/L/flux-linkage motor-model auto-detect.

5. VESC Tool Rotor Position stream is implemented using COMM_SET_DETECT and
   COMM_ROTOR_POSITION at about 100 Hz.

6. Homing states are non-blocking and separate from FOC encoder calibration.
   Full calibration: right hard stop=0 -> left hard stop=360 -> save span -> 180.
   Power-on homing (optional): right hard stop only -> reuse saved span -> 180.
   Full hard-stop calibration is NEVER enabled automatically at boot.

Important terminal commands
---------------------------
hb_auto_detect
hb_encoder_sync
hb_home_cal
hb_home
hb_home_on 0|1
hb_home_status
hb_set_pole_pairs 15
hb_help

Recommended first hardware test
-------------------------------
Keep the steering/wheels unloaded and safe, use a current-limited supply, and
close VESC Tool while Python owns the UART.

python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes

Hard-stop calibration is intentionally opt-in because it physically drives into
both mechanical stops:

python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes --homing-calibrate

To enable one-stop homing on later boots after a successful full calibration:

python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes --homing-calibrate --homing-on

V19 has host/static/numeric verification, but physical V19 behavior is not
claimed PASS until the new firmware is flashed and the hardware log proves it.
