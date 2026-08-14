HOVERBOARD DUAL VESC FACADE - V17 READ ME FIRST
================================================
Release focus: VESC 6.00 unit/config correctness + speed-direction proof.

IMPORTANT MOTOR UNITS
---------------------
Stock hoverboard FOC model uses 15 POLE-PAIRS.
That means:
  VESC Tool Motor Poles       = 30 total magnetic poles
  VESC FOC Encoder Ratio      = 15 pole-pairs
  Stock mechanical max        = 1000 rpm
  Equivalent electrical max   = 15000 eRPM

Do NOT change Motor Poles to 15 to represent the 15 pole-pairs. In VESC Tool,
Motor Poles is the total pole count. V17 deliberately reports 30 there while
FOC Encoder Ratio remains 15.

900 in COMM_SET_RPM means 900 eRPM, not 900 wheel rpm.
With 15 pole-pairs:
  mechanical rpm = eRPM / 15
  900 eRPM       = 60 mechanical rpm
  15000 eRPM     = 1000 mechanical rpm

WHY V16 SHOWED "PARAMETERS TRUNCATED"
--------------------------------------
VESC 6.00 MCCONF contains two different Hall tables:
  1) legacy BLDC hall_table[8]       : int8 commutation states (-1 or 1..6)
  2) FOC foc_hall_table[8]           : electrical angle 0..200, 255 invalid

V1-V16 accidentally placed the FOC-angle values into BOTH fields. Values such
as 50, 183, 16, 116, 83, 150 are valid FOC Hall angles, but invalid values for
the legacy BLDC table. VESC Tool therefore warned "Parameters truncated".

V17 separates the two wire fields. The FOC Hall table is preserved. The legacy
BLDC table now contains only legal legacy values. This fix does not change the
working RIGHT Hall FOC sequence.

CURRENT WARNINGS IN VESC TOOL
-----------------------------
Stock board limits remain:
  regulated motor current       = 15 A
  independent DC-link hard chop = 17 A

For VESC Tool compatibility, V17 reports l_abs_current_max = 22.5 A (1.5 x the
15-A normal motor-current limit) and l_slow_abs_current = OFF. This removes the
generic VESC Tool warning that ABS current is set too close to normal motor
current and removes the warning about Slow ABS Current Limit.

This does NOT raise the hoverboard DC-link hard chop to 22.5 A. The real board
DC-link protection remains 17 A.

CURRENT SCALING PROVEN ON THE V16 HARDWARE LOG
----------------------------------------------
The 2026-08-13 02:20:14 hardware log shows:
  COMM_SET_CURRENT +0.50 A -> raw wire +500
  settled GET_VALUES Iq   -> about +0.47, +0.48, +0.51 A
  diagnostic Iq           -> about +0.49..+0.54 A

  COMM_SET_CURRENT -0.50 A -> raw wire -500
  settled GET_VALUES Iq   -> about -0.46..-0.49 A

Therefore the current command and standard VESC Iq scaling are already correct.
Input Current/Ibattery can be much smaller than phase/Iq at low duty and is not
expected to equal Iq.

SPEED ISSUE FOUND IN THE SAME LOG
---------------------------------
During COMM_SET_RPM +900 eRPM, V16 reported negative measured eRPM (-60 then
-15) while the speed loop increased Iq. This is a feedback-direction mismatch,
not an RPM scaling conversion error.

V17 encoder Detect now uses 3 commanded forward + 3 commanded reverse electrical
sweeps and scores the raw quadrature transition direction to prove encoder
inversion. If a loaded wheel cannot prove direction strongly, configured
fallback remains explicit in diagnostics.

The V17 Python tester also aborts and sends STOP immediately if RPM feedback is
clearly opposite the requested RPM sign. It will not let a wrong-direction speed
PID keep increasing current during a bench test.

ZERO SET / RELEASE FIX
----------------------
Exact zero VESC commands now release for:
  SET_DUTY 0
  SET_CURRENT 0
  SET_RPM 0

SET_POS 0 degrees remains an active position target, which is correct for a
position controller. This fixes V16 cases where zero-current release stayed
armed and configuration writes then timed out while the controller was still
considered energized.

WHAT IS NOT CHANGED
-------------------
- Active LOW-FET current calibration remains unchanged.
- V16 one-shot ISR liveness relief remains unchanged.
- LEFT A/B remain PB6/PB7; PB5 remains Z/index.
- RIGHT Hall Detect remains the working 3-forward + 3-reverse FOC method.
- DC-link hard current chopping remains active.
- No new sticky PWM safety gate was added.

FIRST TEST
----------
Both wheels must be lifted. Use a current-limited supply. Close VESC Tool while
the Python tester owns /dev/ttyUSB0.

  cd /media/sirobo/Data/BLDC/hoverboard_vescv17
  ./RUN_BUILD_UPLOAD_TEST_LINUX.sh /dev/ttyUSB0

Baseline Detect Current remains 0.50 A.

Expected configuration after flashing V17:
  Motor Poles        30
  Encoder Ratio      15
  Max ERPM           15000
  Motor Current Max  15 A
  ABS Current Max    22.5 A (VESC compatibility field)
  Slow ABS Current   OFF

Do not manually change Motor Poles to 15. If VESC Tool is reopened/read-back,
V17 should report the canonical value 30.

The first hardware proof still required from V17 is encoder direction: a +RPM
command must produce positive eRPM. Host regression cannot prove physical A/B
polarity on the board.


## VESC-like eRPM current taper

V17 serializes `l_erpm_start = 0.8` and applies the same operating concept in
the slow control path: same-direction accelerating Iq is untouched below 80%
of the configured electrical-speed limit, then is reduced linearly to zero at
the limit. With 15 pole-pairs and the stock 1000-rpm mechanical ceiling this
means 12000 eRPM -> 15000 eRPM. Opposite-direction braking current remains
available. This is a normal operating-limit calculation, not a sticky PWM
fault latch.
