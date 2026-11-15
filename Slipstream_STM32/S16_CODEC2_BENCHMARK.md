# S16 — Codec 2 3200bps Cortex-M4 startup benchmark

**Implementation committed, hardware build and flash NOT verified.** S7–S11 working DSP, Opus, Speex and G.722 benchmarks untouched unless opt-in S16 is configured.

**Purpose:** evaluate upstream Codec 2 mode 3200 (3.2 kbps) versus S13 Opus, S14 Speex and S15 G.722 under the same STM32F412ZG 100MHz clock. The native codec is **8kHz, 160 samples/20ms, 8 bytes/frame**. The benchmark starts with 320 16kHz samples, uses a 31-tap, 3.5kHz windowed-sinc low-pass FIR decimator (factor 2), invokes genuine Codec 2 encode and decode, and reconstructs 16kHz samples via zero-stuffing and 2x FIR interpolation. Those conversion times are included in the total.

The test uses 5 warmup + 50 measured 20ms frames of the same deterministic synthetic source as Speex and G.722. The one Codec 2 encoder/decoder state is initialized once before measuring and destroyed afterward. This is a startup test *before* audio DMA; it does not assess sustained microphone speech, latency, audio quality, loss recovery or heap/stack high-watermarks. Codec 2 uses floating point and may rely on hardware FPU; this benchmark must be measured rather than assumed fast.

**Use in VS Code** from `Slipstream_STM32`:

1. Pull latest branch `s13-opus-stm32-loopback`.
2. **Terminal -> Run Task -> STM32: Build and Flash S16 Codec2 3200 Benchmark**.
3. The first task clones upstream drowe67/codec2 into ignored `third_party/codec2`, prints its commit SHA, builds upstream static library *only* in `build/codec2-m4f` using cross-compiled CMake and a host-generated codebook. It disables all Codec 2 speech modes except 3200; Codec 2's original CMake source files and license stay in the third_party checkout.
4. The firmware's Performance100 build automatically disables S13/S14/S15 benchmarks, runs S16, and starts normal S7–S11 audio DSP after the isolated test.

Expected UART: `[S16] Codec2Bench 3200 ok=1 frames=50 err=0 Samples8k=160 Bytes=8 Down=... Enc=... Dec=... Up=... Total=...us`

**Compare with prior startup totals:** Opus C0 8.48ms, Speex C0 11.77ms, G.722 5.87ms. Note Codec2 has an additional 16k/8k conversion stage included; old startup test stimulus for Opus differed from S14–S16, so these aren't fully controlled benchmarks.

Equivalent shell:

```bash
bash setup_codec2.sh
bash build100.sh \
  -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF \
  -DS14_SPEEX_BENCHMARK=OFF -DS15_G722_BENCHMARK=OFF \
  -DS16_CODEC2_BENCHMARK=ON \
  -DS16_CODEC2_ROOT="$PWD/third_party/codec2" \
  -DS16_CODEC2_BUILD="$PWD/build/codec2-m4f"
bash flash100.sh
```

Restore DSP-only firmware:

```bash
bash build100.sh -DS16_CODEC2_BENCHMARK=OFF -DS15_G722_BENCHMARK=OFF -DS14_SPEEX_BENCHMARK=OFF -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF
bash flash100.sh
```

**Known unverified dependencies:** cross-compiling Codec 2's native codebook generator with modern CMake, ARM toolchain, and STM32 100MHz memory limits could need follow-up fixes. If task fails, capture the first compiler/CMake error rather than applying random flags. Source uses LGPL-2.1; comply with its terms if distributing Slipstream firmware.
