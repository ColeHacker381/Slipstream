# S15 — G.722 16kHz wideband codec benchmark (startup-only)

**Status:** committed but not compiled or flashed on the STM32 hardware. The current S13 Opus and S14 Speex baseline remain unchanged when S15 is disabled.

The G.722 implementation is the standalone three-file SpanDSP-derived codec from github.com/dgoncharov/g722tools (source commit recorded by setup script). The third-party source is cloned to ignored `third_party/g722tools` and cross-compiled for the Cortex-M4F at `-O3`. Source attribution/licensing remains with upstream; review attribution before distributing firmware.

**Run:**
```bash
git pull --ff-only origin s13-opus-stm32-loopback
```
In the `Slipstream_STM32` VS Code workspace, use **Terminal -> Run Task -> STM32: Build and Flash S15 G722 Benchmark**.

This task builds `build/g722-m4f/libg722.a`, configures `Performance100` with `S13_OPUS_BENCHMARK=OFF`, `S14_SPEEX_BENCHMARK=OFF`, and `S15_G722_BENCHMARK=ON`, flashes, and preserves the normal S7-S11 DSP playback after the benchmark.

**UART output:**
`[S15] G722Bench ok=1 frames=50 err=0 Rate=64000 Enc=.../...us Dec=.../...us Total=.../...us AvgBytes=160 MaxBytes=160 Frame=320`

Fixed 16kHz mono frames: 320 samples/20ms, 160 bytes per packet (64kbps), 5 warmup + 50 timed encode/decode iterations. Verify `ok=1` before interpreting times. This is not a live DMA load test or listening-quality comparison.

**Restore bare DSP:**
```sh
bash build100.sh -DS15_G722_BENCHMARK=OFF -DS14_SPEEX_BENCHMARK=OFF -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF
bash flash100.sh
```

**Codec 2 next:** Unlike G.722, Codec 2 is a separate floating-point, FPU-dependent codec whose upstream STM32 build requires careful toolchain/configuration and whose native modes generally operate at 8kHz. We should benchmark its 3200-bit/s mode including the 16kHz->8kHz downsampling and 8kHz->16kHz interpolation cost; otherwise it is not an equivalent end-to-end comparison. Native Codec 2 3200 bitrate is around 8 bytes per 20ms, but frame sizes and resampling must be validated from the actual API.

**Quality/fairness caveat:** This synthetic benchmark input matches Speex's current test generator, not Opus's startup generator. CPU comparisons are preliminary until all codecs share the same recorded mic input. G.722 packet overhead per RF node is far larger than Opus or Codec 2 and must be weighed against CPU savings.
