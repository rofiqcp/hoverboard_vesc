#!/usr/bin/env bash
set -u
cd "$(dirname "$0")"
PORT="${1:-/dev/ttyUSB0}"
echo "=========================================================="
echo "Hoverboard VESC Dual V17 - Full One-Shot Hardware Test"
echo "PORT: $PORT"
echo "WARNING: full test will run sensor commissioning and move both motors."
echo "Lift both wheels and use a current-limited supply for first commissioning."
echo "=========================================================="
read -r -p "Type RUN to continue: " CONFIRM
[ "$CONFIRM" = "RUN" ] || { echo "Cancelled."; exit 1; }
python3 -c 'import serial' >/dev/null 2>&1 || python3 -m pip install -r requirements-test.txt || exit 2
python3 tools/vesc_full_test.py --port "$PORT" --full --yes
RC=$?
echo "Exit code: $RC"
echo "Send the newest vesc_test_logs/vesc_full_test_*.zip for troubleshooting."
exit "$RC"
