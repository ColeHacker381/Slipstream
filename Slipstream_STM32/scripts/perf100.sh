#!/usr/bin/env bash
# Slipstream Performance100 helper. Usage:
#   bash scripts/perf100.sh build
#   bash scripts/perf100.sh flash
#   bash scripts/perf100.sh all
#   bash scripts/perf100.sh test
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mode="${1:-build}"
case "$mode" in
  build|flash|all|test) ;;
  *) echo "Usage: bash scripts/perf100.sh {build|flash|all|test}" >&2; exit 2 ;;
esac

if [[ "$mode" == "test" ]]; then
  command -v cc >/dev/null || { echo "Missing native C compiler (cc)" >&2; exit 1; }
  cc -std=c11 -Wall -Wextra -Werror -ICore/Inc \
    tests/s13_frames_host_test.c Core/Src/slipstream_s13_frames.c \
    -o /tmp/slipstream_s13_frames_test
  /tmp/slipstream_s13_frames_test
  echo "S13 frame assembler host test: PASS"
  exit 0
fi

if [[ "$mode" == "build" || "$mode" == "all" ]]; then
  command -v cube-cmake >/dev/null || { echo "Missing cube-cmake; activate the STM32Cube toolchain" >&2; exit 1; }
  cube-cmake --preset Performance100
  cube-cmake --build --preset Performance100
fi

if [[ "$mode" == "flash" || "$mode" == "all" ]]; then
  elf="build/Performance100/Slipstream_STM32.elf"
  [[ -f "$elf" ]] || { echo "Missing $elf; run build first" >&2; exit 1; }
  if command -v STM32_Programmer_CLI >/dev/null; then
    STM32_Programmer_CLI -c port=SWD -w "$elf" -v -rst
  elif command -v st-flash >/dev/null; then
    bin="build/Performance100/Slipstream_STM32.bin"
    if [[ ! -f "$bin" ]]; then
      command -v arm-none-eabi-objcopy >/dev/null || { echo "Missing arm-none-eabi-objcopy" >&2; exit 1; }
      arm-none-eabi-objcopy -O binary "$elf" "$bin"
    fi
    st-flash --reset write "$bin" 0x08000000
  else
    echo "No flashing tool found. Install STM32_Programmer_CLI or st-flash." >&2
    exit 1
  fi
fi
