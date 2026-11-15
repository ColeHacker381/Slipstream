#!/usr/bin/env bash
# Run with: bash flash100.sh (from any directory)
set -euo pipefail
cd "$(dirname "$0")"
ELF="build/Performance100/Slipstream_STM32.elf"
BIN="build/Performance100/Slipstream_STM32.bin"
if [[ ! -f "$ELF" ]]; then
  echo "Missing $ELF. Build with: bash build100.sh" >&2
  exit 1
fi
PROGRAMMER="$(command -v STM32_Programmer_CLI || true)"
if [[ -z "$PROGRAMMER" && -d "$HOME/.local/share/stm32cube/bundles/programmer" ]]; then
  PROGRAMMER="$(find "$HOME/.local/share/stm32cube/bundles/programmer" -type f -name STM32_Programmer_CLI | sort -V | tail -1)"
fi
if [[ -n "$PROGRAMMER" ]]; then
  echo "Using $PROGRAMMER"
  "$PROGRAMMER" -c port=SWD -w "$ELF" -v -rst
elif command -v st-flash >/dev/null 2>&1; then
  command -v arm-none-eabi-objcopy >/dev/null 2>&1 || { echo "Install arm-none-eabi-objcopy" >&2; exit 1; }
  arm-none-eabi-objcopy -O binary "$ELF" "$BIN"
  st-flash --reset write "$BIN" 0x08000000
else
  echo "Neither STM32_Programmer_CLI nor st-flash is installed." >&2
  echo "Install stlink tools (Fedora: sudo dnf install stlink) or STM32CubeProgrammer." >&2
  exit 1
fi
