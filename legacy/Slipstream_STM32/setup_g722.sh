#!/usr/bin/env bash
# Build standalone G.722 wideband codec for Cortex-M4F, no Speex or Opus.
set -euo pipefail
cd "$(dirname "$0")"
for cmd in git arm-none-eabi-gcc arm-none-eabi-ar; do
  command -v "$cmd" >/dev/null || { echo "Missing $cmd" >&2; exit 1; }
done
SRC="$PWD/third_party/g722tools"
OUT="$PWD/build/g722-m4f"
mkdir -p third_party "$OUT"
if [[ ! -f "$SRC/src/g722.h" ]]; then
  git clone --depth 1 https://github.com/dgoncharov/g722tools.git "$SRC"
fi
for name in g722_encode g722_decode; do
  arm-none-eabi-gcc -std=gnu11 -O3 -DNDEBUG -mcpu=cortex-m4 -mthumb \
    -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -ffunction-sections -fdata-sections \
    -include stdint.h -I"$SRC/src" -c "$SRC/src/${name}.c" -o "$OUT/${name}.o"
done
arm-none-eabi-ar rcs "$OUT/libg722.a" "$OUT/g722_encode.o" "$OUT/g722_decode.o"
echo "[S15] G.722 archive ready: $OUT/libg722.a"
git -C "$SRC" rev-parse HEAD
