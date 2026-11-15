#!/usr/bin/env bash
# Download reproducible upstream libopus and cross-compile for STM32F412 (ARM M4F).
# Run: bash setup_opus.sh [fixed|float]
set -euo pipefail
cd "$(dirname "$0")"
FLAVOR="${1:-fixed}"
if [[ "$FLAVOR" != fixed && "$FLAVOR" != float ]]; then
  echo "Usage: bash setup_opus.sh [fixed|float]" >&2
  exit 2
fi
for cmd in git cmake ninja arm-none-eabi-gcc arm-none-eabi-ar; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "Missing $cmd" >&2; exit 1; }
done
OPUS_SRC="$PWD/third_party/opus"
if [[ ! -d "$OPUS_SRC/.git" ]]; then
  mkdir -p "$PWD/third_party"
  git clone --depth 1 --branch v1.5.2 https://github.com/xiph/opus.git "$OPUS_SRC"
fi
# Cortex-M4F supports DSP/FPU instructions but NOT ARM NEON SIMD.
# Upstream cross-compile probes can incorrectly enable NEON source files;
# explicitly disable Opus architecture intrinsics for correct portable C build.
# Keep third_party/opus untracked and isolated from the STM32 repository.
if [[ "$FLAVOR" == fixed ]]; then FIXED=ON; else FIXED=OFF; fi
OPUS_BUILD="$PWD/build/opus-${FLAVOR}-m4f"
cmake -S "$OPUS_SRC" -B "$OPUS_BUILD" -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF \
  -DOPUS_FIXED_POINT="$FIXED" \
  -DOPUS_DISABLE_INTRINSICS=ON \
  -DOPUS_BUILD_TESTING=OFF \
  -DBUILD_TESTING=OFF \
  -DOPUS_BUILD_PROGRAMS=OFF \
  -DOPUS_INSTALL_CMAKE_CONFIG_MODULE=OFF \
  -DOPUS_INSTALL_PKG_CONFIG_MODULE=OFF
cmake --build "$OPUS_BUILD" --target opus -j 4
LIB="$OPUS_BUILD/libopus.a"
if [[ ! -f "$LIB" ]]; then
  echo "Could not locate $LIB; inspect Opus CMake build output." >&2
  exit 1
fi
arm-none-eabi-size "$LIB" | tail -n 4 || true
echo "Built upstream Opus v1.5.2 ($FLAVOR) for Cortex-M4F:"
echo "  Include: $OPUS_SRC/include"
echo "  Library: $LIB"
echo "Next: bash opus100.sh '$OPUS_SRC/include' '$LIB'"
