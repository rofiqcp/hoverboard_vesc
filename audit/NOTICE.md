# Source and License Notice

This firmware rewrite is distributed under **GPL-3.0-or-later**, consistent with the two source families used for the work:

- **VESC firmware** — portions of the FOC architecture, naming, Park/inverse-Park convention, VESC-style speed/position PID structure, MTPA equation, VESC 7 fast-loop field-weakening behavior, current-vector limiting, and six-sector `foc_svm()` equations were adapted from the user-supplied `motor.zip` and checked against the official `vedderb/bldc` master on 2026-08-12. The referenced VESC files carry copyright **2016–2022 Benjamin Vedder** and GPL-3.0-or-later notices.
- **STM32 hoverboard FOC firmware** — the board/HAL integration, current ADC paths, timer wiring, safety gate structure, protocol/EEPROM/calibration framework, and other project-specific integration derive from the user-supplied STM32 firmware, which carries copyright **2019–2020 Emanuel FERU** and GPL-3.0-or-later notices.

The rewrite is not a verbatim copy of the full VESC motor controller. It is a fixed-point STM32F103 adaptation for this board and deliberately excludes VESC observer/HFI/sensorless-estimator paths, dynamic VESC input-current/thermal override maps, and model-decoupling paths that are not validated/configured on the selected hardware.

See `COPYING` for GPL v3 terms.
