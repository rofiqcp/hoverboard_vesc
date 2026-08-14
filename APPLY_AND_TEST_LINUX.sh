#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /path/to/hoverboard_vesc [--all-repo-tests]"
  exit 2
fi
ROOT="$(realpath "$1")"
shift || true
HERE="$(cd "$(dirname "$0")" && pwd)"
python3 "$HERE/tools/apply_v22_deep_main_scheduler.py" "$ROOT"
python3 "$ROOT/tools/v22_deep_debug.py" "$ROOT" "$@"
echo
printf 'PASS. Flash with:\n  cd %q && pio run -t upload\n' "$ROOT"
