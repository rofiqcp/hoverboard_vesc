HOVERBOARD VESC V18
===================

Primary changes:
- sensor/RPM observation fixed at 16 kHz;
- LEFT/RIGHT FOC interleaved at 8 kHz each for STM32F103 CPU/UART liveness;
- SET_CURRENT remains VESC torque/Iq mode (RPM can rise on unloaded wheel);
- SET_CURRENT_BRAKE is now a dedicated dynamic braking mode;
- SET_HANDBRAKE is now a dedicated static phase-0 current mode;
- unused commissioning_current_guard warning removed.

Stock motor geometry:
- 15 pole-pairs
- 30 Motor Poles in VESC Tool
- SET/GET RPM is eRPM

First test:
  python3 tools/vesc_full_test.py --port /dev/ttyUSB0 --full --yes

Keep wheels lifted and use a current-limited supply.
