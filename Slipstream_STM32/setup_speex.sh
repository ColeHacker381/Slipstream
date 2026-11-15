#!/usr/bin/env bash
# Build upstream Speex 1.2.1 codec for STM32F412 Cortex-M4F (NOT SpeexDSP).
# Run from Slipstream_STM32: bash setup_speex.sh
set -euo pipefail
cd "$(dirname "$0")"
for cmd in git arm-none-eabi-gcc arm-none-eabi-ar arm-none-eabi-size; do
  command -v "$cmd" >/dev/null || { echo "Missing $cmd" >&2; exit 1; }
done
SRC="$PWD/third_party/speex"
OUT="$PWD/build/speex-fixed-m4f"
mkdir -p "$PWD/third_party" "$OUT/obj" "$OUT/include/speex"
if [[ ! -d "$SRC/.git" ]]; then
  git clone --depth 1 --branch Speex-1.2.1 https://github.com/xiph/speex.git "$SRC"
fi
# Speex's autoconf normally generates this file. Cortex-M4 uses GCC
# 16-bit short and 32-bit int; keep it in ignored build/ not upstream.
cat > "$OUT/include/speex/speex_config_types.h" <<'EOF'
#ifndef __SPEEX_TYPES_H__
#define __SPEEX_TYPES_H__
#include <stdint.h>
typedef int16_t spx_int16_t;
typedef uint16_t spx_uint16_t;
typedef int32_t spx_int32_t;
typedef uint32_t spx_uint32_t;
#endif
EOF
# Codec sources from upstream libspeex/Makefile.am, no SpeexDSP/ogg/examples.
sources=(
 cb_search exc_10_32_table exc_8_128_table filters gain_table
 hexc_table high_lsp_tables lsp ltp speex stereo vbr vq bits
 exc_10_16_table exc_20_32_table exc_5_256_table exc_5_64_table
 gain_table_lbr hexc_10_32_table lpc lsp_tables_nb modes modes_wb
 nb_celp quant_lsp sb_celp speex_callbacks speex_header window
)
objects=()
for f in "${sources[@]}"; do
  obj="$OUT/obj/$f.o"
  arm-none-eabi-gcc -std=gnu99 -O3 -DNDEBUG -DFIXED_POINT -DVAR_ARRAYS \
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -ffunction-sections -fdata-sections \
    -I"$SRC/include" -I"$OUT/include" -I"$SRC/libspeex" \
    -c "$SRC/libspeex/$f.c" -o "$obj"
  objects+=("$obj")
done
arm-none-eabi-ar rcs "$OUT/libspeex.a" "${objects[@]}"
arm-none-eabi-size "$OUT/libspeex.a" | tail -n 4 || true
echo "Speex-1.2.1 fixed-point 16kHz wideband archive: $OUT/libspeex.a"
echo "Includes: $SRC/include and $OUT/include"
