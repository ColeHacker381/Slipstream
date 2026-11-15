#!/usr/bin/env bash
# Cross-build just the Codec 2 library (3200 bit/s only), with upstream CMake.
# Native CMake/codebook generator is invoked by upstream while cross-compiling.
set -euo pipefail
cd "$(dirname "$0")"
for c in git cmake arm-none-eabi-gcc arm-none-eabi-ar; do
  command -v "$c" >/dev/null || { echo "Missing $c" >&2; exit 1; }
done
SRC="$PWD/third_party/codec2"
OUT="$PWD/build/codec2-m4f"
mkdir -p third_party "$OUT"
if [[ ! -d "$SRC/.git" ]]; then
  git clone --depth 1 https://github.com/drowe67/codec2.git "$SRC"
fi
echo "Codec 2 upstream revision: $(git -C "$SRC" rev-parse HEAD)"
# Intentionally no Cortex-M7 macro: this target is STM32F412 Cortex-M4F.
# upstream src/CMakeLists cross-compiles a native generator for codebook.c.
cmake -S "$SRC" -B "$OUT" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY \
  -DCMAKE_C_FLAGS="-O3 -D__EMBEDDED__ -DCODEC2_MODE_EN_DEFAULT=0 -DCODEC2_MODE_3200_EN=1 -ffunction-sections -fdata-sections" \
  -DBUILD_SHARED_LIBS=OFF -DMICROCONTROLLER_BUILD=ON \
  -DUNITTEST=OFF -DLPCNET=OFF
cmake --build "$OUT" --target codec2 -j 4
test -f "$OUT/src/libcodec2.a" || { echo "Codec2 archive missing" >&2; exit 1; }
echo "[S16] Built $OUT/src/libcodec2.a"
