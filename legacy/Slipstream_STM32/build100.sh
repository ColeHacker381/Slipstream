#!/usr/bin/env bash
# Run with: bash build100.sh [extra CMake configure arguments]
set -euo pipefail
cd "$(dirname "$0")"
if ! command -v cube-cmake >/dev/null 2>&1; then
  echo "cube-cmake not found. Use 'cmake' if you have STM32 tools installed normally." >&2
  CMAKE=cmake
else
  CMAKE=cube-cmake
fi
"$CMAKE" --preset Performance100 "$@"
"$CMAKE" --build --preset Performance100
ELF="build/Performance100/Slipstream_STM32.elf"
if [[ -f "$ELF" ]]; then
  arm-none-eabi-size "$ELF"
  echo "Build successful: $ELF"
fi
