# S13 Opus — Fedora STM32F412ZG build, flash and benchmark

The branch is `s13-opus-stm32-loopback` off `main`. S12 wind experiments remain in the separate draft PR. This branch contains the original S7–S11 audio chain, a standalone 10ms-to-20ms frame assembler and **real Opus startup encode/decode benchmarking**. **No live Opus speaker loopback yet.**

## Build and flash shortcuts (from Slipstream_STM32/)

Run scripts with `bash` (GitHub-created files need not have executable mode):

```sh
bash build100.sh -DS13_OPUS_BENCHMARK=OFF
bash flash100.sh
```

`build100.sh` uses `cube-cmake` when available, otherwise `cmake`. `flash100.sh` uses STM32_Programmer_CLI if installed, otherwise st-flash; for st-flash it derives a BIN from ELF using arm-none-eabi-objcopy and flashes at 0x08000000.

Required commands: `git`, `cmake`, `ninja`, `arm-none-eabi-gcc`, `arm-none-eabi-ar`, `arm-none-eabi-size`, `arm-none-eabi-objcopy`, plus either `STM32_Programmer_CLI` or `st-flash`. Fedora installs may include `arm-none-eabi-gcc-cs`, `arm-none-eabi-newlib`, `cmake`, `ninja-build`, `git`, `stlink`; check package names on your configured distro.

**CMake cache pitfall:** enabling `S13_OPUS_BENCHMARK` persists in `build/Performance100` until explicitly disabled. Reverting to plain DSP is `bash build100.sh -DS13_OPUS_BENCHMARK=OFF`.

## Build genuine ARM libopus (v1.5.2)

This project did **not** copy the ESP-IDF `78/esp-opus` prebuilt artifact. A reproducible helper downloads the official upstream C source and compiles an ARM M4F static library:

```sh
bash setup_opus.sh fixed
```

It clones `https://github.com/xiph/opus.git` at stable tag `v1.5.2` into ignored `third_party/opus`, runs cross-CMake/Ninja with the project's cortex-m4 hard-float toolchain, and creates `build/opus-fixed-m4f/libopus.a`. This is a fixed-point DSP core, while the int16 public Opus API remains available. For comparison a floating-point library can be compiled using `bash setup_opus.sh float` (separate output directory). Neither mode automatically guarantees M4F speed, RAM or stack suitability. Do NOT run the host OS libopus archive on STM32.

If the helper fails: paste the first build error, not only the final exit message. Upstream library builds can require toolchain-specific adjustment to Opus allocation or C library availability; don't add simulated codec functions.

## Real Opus benchmark without modifying active microphone/speaker PCM

```sh
bash opus100.sh "$PWD/third_party/opus/include" "$PWD/build/opus-fixed-m4f/libopus.a"
bash flash100.sh
```

The `opus100.sh` command builds `Performance100` with `S13_OPUS_BENCHMARK=ON`; it executes a real Opus **once at startup before audio DMA**. It allocates encoder and decoder using `opus_encoder_get_size(1)` / `opus_decoder_get_size(1)` and calls `opus_encoder_init` / `opus_decoder_init`. Settings: 16kHz mono, signed 16-bit PCM, 320 samples/20ms, OPUS_APPLICATION_VOIP, 20kbps, VBR on, FEC off, DTX off, max 512 bytes, complexity 5 by default. 5 warmup and 50 measured frames of varying synthetic input; prints average/max encode and decode times, total latency, average encoded bytes, and encoder/decoder allocation sizes to ST-LINK's UART3.

The UART should show a single `[S13] OpusBench C=5 ...` line before existing `[S7]`/audio telemetry. Check `ok=1 frames=50 err=0`. Example field names: `Enc=AVG/MAXus`, `Dec=AVG/MAXus`, `Total=AVG/MAXus`, `RAMenc`, `RAMdec`. Numbers are not known until the board runs.

To try complexity 3 without changing source:

```sh
bash build100.sh -DS13_OPUS_COMPLEXITY=3
bash flash100.sh
```

Return to 5 with `bash build100.sh -DS13_OPUS_COMPLEXITY=5`. The benchmark mode must be ON and a library configured to compile it; `build100.sh` reuses those cached paths. Avoid changing codec representation and complexity simultaneously when comparing.

## Safety gates / limitations

- If linking fails due to `_sbrk`, insufficient heap, `malloc`, or stack, preserve the exact error. The benchmark uses temporary **startup-only** malloc; the real-time path must not allocate. Inspect SRAM/heap/stack in map file and actual watermark before declaring viability.
- The STM32 application already uses ~5ms per 10ms DSP block in previous Performance100 logging. A single-core 100MHz MCU cannot blindly run an expensive encode+decode operation inside the 10ms DMA servicing function. Even if the pre-DMA startup benchmark succeeds, continuous scheduling must be designed and measured separately.
- **Opus cannot be considered locally looped back** until a complete 320-sample decoded frame actually reaches WM8960 speakers. This first hardware gate only validates API, ARM linking, accuracy, RAM and isolated codec speed.
- Preserve audio-only baseline, no S12 edits, no ESP bridge or Bluetooth work in this phase.
