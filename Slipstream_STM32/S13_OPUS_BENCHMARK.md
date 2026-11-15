# S13 Milestone 1: Real Opus benchmark — STM32F412ZG

This is a **startup-only** codec benchmark, not yet a live audio loopback. It uses real libopus `opus_encode`/`opus_decode`, not an approximation or dummy codec, and measures on the actual 100 MHz STM32. It runs before the I2S DMA begins, then exits and allows the existing S7–S11 pipeline to run normally. It benchmarks 50 frames after five warmup frames, each 320 samples / 20ms, synthetic non-silent input. This is not a realistic worst-case speech workload.

## Prerequisites

On Fedora or another machine with `arm-none-eabi-gcc`, `cmake` and `ninja`, obtain upstream Opus **source**; the ESP-IDF `78/esp-opus` binary from the original project is not suitable for this board. The upstream project is at https://github.com/xiph/opus . This source has its own license; retain all notices and verify license/compliance before distribution.

From the `Slipstream_STM32` directory, example cross-build workflow:

```sh
git clone --depth 1 https://github.com/xiph/opus.git ../opus-upstream
cmake -S ../opus-upstream -B ../opus-arm-build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
  -DOPUS_BUILD_TESTING=OFF -DOPUS_BUILD_PROGRAMS=OFF
cmake --build ../opus-arm-build -j
find ../opus-arm-build -name 'libopus.a'
```

This is a suggested build approach, **not a build run or verified archive**. Upstream CMake options can vary by revision. If the source/build does not accept one of the options, inspect `../opus-upstream/CMakeLists.txt` and adapt; do not guess that Opus was successfully cross-compiled. Use `arm-none-eabi-objdump -f <archive-or-object>` or `arm-none-eabi-ar t` and your compiler settings to confirm ARM target. Check hard-float ABI and other flags are compatible with the STM32 firmware. A libopus build may exceed flash/RAM even if it compiles.

Configure the board build with the Opus headers and **the path returned by find**:

```sh
cmake -S . -B build/s13-bench -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Performance100 \
  -DS13_OPUS_BENCHMARK=ON \
  -DS13_OPUS_INCLUDE_DIR="$PWD/../opus-upstream/include" \
  -DS13_OPUS_LIBRARY="$PWD/../opus-arm-build/libopus.a"
cmake --build build/s13-bench -j
arm-none-eabi-size build/s13-bench/Slipstream_STM32.elf
```

The final static archive path above is illustrative: use whatever `find` actually returns. We require a real `opus.h` and `libopus.a`. If the configure fails, address the toolchain/library first rather than adding a fake codec.

For the **baseline without Opus**, existing `cube-cmake --preset Performance100` and its build path remain unchanged; `S13_OPUS_BENCHMARK` defaults to OFF.

## Target UART record

At startup, before the normal `[S7]` DMA message, look for:

```text
[S13] OpusBench ok=1 frames=50 err=0 Enc=<avg>/<max>us Dec=<avg>/<max>us Total=<avg>/<max>us AvgBytes=<bytes> RAMenc=<bytes> RAMdec=<bytes>
```

The values are **measurements to collect**, not estimates. `ok=0` means actual init, allocation, encoding or decoding failed. Source uses `malloc` and `free` only in startup test; the eventual live codec must have fixed lifetime and no allocation on the audio path. This benchmark's RAM state sizes exclude stack, work buffers, and the rest of the system. Heap may be insufficient even when link succeeds. Do not increase stack/heap arbitrarily without reviewing the ELF/map and linker script.

## Acceptance criteria

1. A successful real codec build and boot: `ok=1 frames=50 err=0`. No VAD gating, 512-byte maximum Opus output; complexity 5, 16k/mono, 20ms, 20kbps VBR, no FEC and no DTX.
2. Record encode and decode average/max separately; examine the **combined maximum** relative to 20 ms and remaining DSP budget. On a single-core STM32, total CPU availability matters even if codec work happens outside the 10 ms DMA critical path. A benchmark running with DMA stopped does not measure live contention; regard it as the initial viability gate only.
3. Confirm current audio baseline after the benchmark still runs, with zero `Miss`, `Late`, `DMA` errors. Compare original build if uncertain.
4. Record flash (`arm-none-eabi-size`), RAM and map, static codec state sizes; measure stack high-water in the eventual live worker.
5. Repeat with realistic recorded voiced PCM and pathological speech/noise before choosing complexity. **Only then** consider lower complexity as a measured tradeoff.
6. Host frame-assembler test (no Opus dependency) from this directory:
```sh
cc -std=c11 -Wall -Wextra -Werror -ICore/Inc \
  tests/s13_frames_host_test.c Core/Src/slipstream_s13_frames.c \
  -o /tmp/s13frames && /tmp/s13frames
```

## Current integration boundary

Frame assembler in `slipstream_s13_frames.h/.c` is unit-tested by source tests, compiled by STM32 CMake, but deliberately **not yet inserted into the live audio pipeline**. The test makes no claim to have been run on this repository by the assistant.

**Next after benchmark:** select a safe non-blocking scheduling approach for 20ms Opus jobs and a bounded decoded-frame queue, then integrate audio behind a bypass switch. Do not encode in `s7_process_half` before DSP output is committed; doing so could miss 10ms hardware deadlines. The first live loopback must capture continuous Opus frames independently of VAD.
