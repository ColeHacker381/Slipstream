#!/usr/bin/env bash
# Run once you've built an ARM-compatible libopus archive.
# Usage: bash opus100.sh /absolute/path/to/opus/include /absolute/path/to/libopus.a
set -euo pipefail
cd "$(dirname "$0")"
if [[ $# -ne 2 ]]; then
  echo "Usage: bash opus100.sh /path/to/opus/include /path/to/libopus.a" >&2
  exit 2
fi
INCDIR="$(realpath "$1")"
LIB="$(realpath "$2")"
if [[ ! -f "$INCDIR/opus.h" || ! -f "$LIB" ]]; then
  echo "Need opus.h and a real ARM libopus.a; check paths" >&2
  exit 1
fi
if ! arm-none-eabi-ar t "$LIB" >/dev/null 2>&1; then
  echo "Archive could not be read by ARM binutils." >&2
  exit 1
fi
bash build100.sh -DS13_OPUS_BENCHMARK=ON -DS13_OPUS_INCLUDE_DIR="$INCDIR" -DS13_OPUS_LIBRARY="$LIB"
