#!/usr/bin/env bash
set -u
set -o pipefail

PORT="${1:-/dev/ttyUSB0}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
BUILD_DIR="$ROOT/build_logs"
TEST_PARENT="$ROOT/vesc_test_logs"
BUILD_LOG="$BUILD_DIR/platformio_build_upload_${STAMP}.log"
MANIFEST="$BUILD_DIR/source_sha256_${STAMP}.txt"
mkdir -p "$BUILD_DIR" "$TEST_PARENT"

cd "$ROOT"
if ! command -v pio >/dev/null 2>&1; then
  echo "ERROR: pio/PlatformIO tidak ditemukan di PATH." | tee "$BUILD_LOG"
  echo "Install/aktifkan PlatformIO lalu ulangi." | tee -a "$BUILD_LOG"
  exit 127
fi

{
  echo "=== HOVERBOARD V17 BUILD/UPLOAD ==="
  echo "timestamp=$(date --iso-8601=seconds)"
  echo "root=$ROOT"
  echo "port=$PORT"
  echo "pio=$(pio --version 2>&1)"
  echo
} | tee "$BUILD_LOG"

find Src -maxdepth 1 -type f \( -name '*.c' -o -name '*.h' \) -print0 \
  | sort -z | xargs -0 sha256sum > "$MANIFEST"
sha256sum platformio.ini startup_stm32f103xe.s STM32F103RCTx_FLASH.ld >> "$MANIFEST"

set +e
pio run -t upload 2>&1 | tee -a "$BUILD_LOG"
BUILD_RC=${PIPESTATUS[0]}
set -e
if [ "$BUILD_RC" -ne 0 ]; then
  echo "BUILD_UPLOAD_RESULT=FAIL rc=$BUILD_RC" | tee -a "$BUILD_LOG"
  exit "$BUILD_RC"
fi
echo "BUILD_UPLOAD_RESULT=PASS" | tee -a "$BUILD_LOG"

sleep 2
set +e
python3 tools/vesc_full_test.py --port "$PORT" --full --yes --log-dir "$TEST_PARENT"
TEST_RC=$?
set -e

LATEST_DIR="$(find "$TEST_PARENT" -maxdepth 1 -mindepth 1 -type d -name 'vesc_full_test_*' -printf '%T@ %p\n' | sort -nr | head -1 | cut -d' ' -f2-)"
if [ -n "$LATEST_DIR" ] && [ -d "$LATEST_DIR" ]; then
  cp "$BUILD_LOG" "$LATEST_DIR/platformio_build_upload.log"
  cp "$MANIFEST" "$LATEST_DIR/source_sha256.txt"
  python3 - "$LATEST_DIR" <<'PY'
import shutil, sys, pathlib
root = pathlib.Path(sys.argv[1])
zip_path = shutil.make_archive(str(root), 'zip', root_dir=root)
print(f"FINAL TROUBLESHOOT ZIP: {zip_path}")
PY
fi

exit "$TEST_RC"
