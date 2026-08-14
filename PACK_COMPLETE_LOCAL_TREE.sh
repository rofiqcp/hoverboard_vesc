#!/usr/bin/env bash
set -euo pipefail
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 /path/to/patched/hoverboard_vesc [output.zip]"
  exit 2
fi
ROOT="$(realpath "$1")"
OUT="${2:-hoverboard_vesc_v22_deep_main_full.zip}"
if [[ "$OUT" = /* ]]; then
  OUT_ABS="$OUT"
else
  OUT_ABS="$(pwd)/$OUT"
fi
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/v22"
rsync -a --exclude '.git' --exclude '.pio' --exclude '.v22_deep_backup' "$ROOT/" "$TMP/v22/"
(cd "$TMP" && zip -qr "$OUT_ABS" v22)
echo "Created: $OUT_ABS"
