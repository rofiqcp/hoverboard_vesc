HOVERBOARD DUAL FOC V20 - READ FIRST
====================================

V20 fixes two real memory/protocol faults from V19 and tightens LEFT encoder
phase synchronization without modifying the ADC/DMA current-control ISR.

FIRST TEST - CONNECTION ONLY
----------------------------
1. Wheels/steering unloaded and safe.
2. Power-cycle the controller after flashing V20.
3. Run:

   python3 tools/vesc_full_test.py --port /dev/ttyUSB0

Expected first three results:
   00_protocol_self_test PASS
   01_open_serial        PASS
   02_connection_inventory PASS

Do not continue to motion if inventory still fails. V20 firmware name on wire is:
   hoverboard-vesc6-v20

SECOND TEST - FULL SENSOR + MOTION
---------------------------------
Only after connection inventory passes:

   python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes

Integrated commissioning must prove:
   LEFT sensor                 = Encoder AB
   LEFT calibrated             = true
   LEFT alignment probe delta  = abs >= 2 raw TIM4 counts
   LEFT direction proved       = true
   LEFT electrical ready       = true
   RIGHT sensor                = Hall
   RIGHT calibrated            = true

If LEFT does not produce a signed TIM4 probe movement, V20 deliberately refuses
READY instead of guessing encoder direction.

DO NOT ENABLE HARD-STOP HOMING YET
---------------------------------
First prove LEFT and RIGHT Duty/Current/RPM and both 0..360 Position tests.
Only after normal motion and position tracking pass should hard-stop homing be run.

VESC TOOL ROTOR POSITION
------------------------
- Encoder: raw encoder mechanical angle (LEFT encoder only)
- PID Pos: logical 0..360 position
- PID Error: position error
- Detect/Inductance: detect phase only while commissioning is actually active
- Observer / Obs-vs-sensor: intentionally no fake output. This firmware does not
  yet contain a VESC-style flux observer.

Vd/Vq
-----
V20 exposes standard GET_VALUES Vd/Vq in volts. They are derived from the FOC
controller modulation and DC-link voltage; a physical BEMF sensor is not required.
This calculation runs in slow telemetry, not the DMA ISR.

AUTO DETECT SCREEN
------------------
The integrated V20 board auto-detect commissions current offsets and sensors. It
currently does NOT measure motor R/L/flux like full upstream VESC FOC Auto Detect.
Values such as 50 mOhm / 20 uH in the Detection Result are configuration
compatibility values, not new physical measurements from V20.
