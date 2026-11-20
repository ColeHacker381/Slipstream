# S14 — Speex vs Opus: isolated Cortex-M4 benchmark

**Status:** implementation committed, **not compiled or measured on board yet**. Does not replace Opus or S7-S11. Targets upstream Xiph Speex **1.2.1 codec** (not the separate SpeexDSP processing package).

## Purpose
Test Speex fixed-point **16 kHz wideband, mono, 320-sample/20 ms frame** with real encoder+decoder on STM32F412ZG, 100 MHz, Performance100, against existing Opus C0 startup measurements (approx. encode 7.2ms, decode 1.25ms, total 8.48ms for a synthetic signal). Quality **6**, complexity **1**, target bitrate **20 kbps** (Speex chooses supported mode; `Rate` field reports actual setting), CBR and no DTX.

## Prepare and run
Open `Slipstream_STM32` VS Code workspace, pull `s13-opus-stm32-loopback`, then run:

**Terminal -> Run Task -> STM32: Build and Flash S14 Speex Q6 C1**.

The task first runs `setup_speex.sh`, which fetches pinned Speex-1.2.1 upstream under ignored `third_party/speex`, compiles upstream codec sources (fixed-point, Cortex-M4F, `-O3`), and archives `build/speex-fixed-m4f/libspeex.a`. It then configures and builds Performance100 with **S13 Opus disabled**, S14 startup bench enabled, and flashes the MCU. Nothing is committed into third_party.

Equivalent from `Slipstream_STM32`:

```bash
bash setup_speex.sh
bash build100.sh \
    -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF \
    -DS14_SPEEX_BENCHMARK=ON \
    -DS14_SPEEX_COMPLEXITY=1 -DS14_SPEEX_QUALITY=6 \
    -DS14_SPEEX_ROOT="$PWD/third_party/speex" \
    -DS14_SPEEX_BUILD="$PWD/build/speex-fixed-m4f"
bash flash100.sh
```

**Expected startup format:**
`[S14] SpeexBench Q=6 C=1 ok=1 frames=50 err=0 Rate=... Enc=.../...us Dec=.../...us Total=.../...us AvgBytes=... MaxBytes=... Frame=320`

Following this log, the normal unchanged S7-S11 DSP playback starts, and `Miss/Late/DMA` should remain zero. This is *only* the **startup** encoder/decoder timing benchmark, **not yet live input or an Opus-versus-Speex sound-quality listening test**. It runs 5 warmup frames and measures 50 deterministic synthetic frames. The sample generator differs from the Opus baseline generator, so packet sizes and workloads **are not directly apples-to-apples** until matched-signal and live tests are added.

After collecting C1, run C0 with:
```bash
bash build100.sh -DS14_SPEEX_COMPLEXITY=0
bash flash100.sh
```
The bitrate field is the codec-reported rate, not guaranteed to equal the requested 20k. Speex state/temporary allocation is not yet measured; Speex may use heap allocations. Guard with stack/heap measurements before enabling live speech. Ensure the new startup line prints `ok=1` and `Frame=320` before interpreting timings.

To restore the original DSP-only firmware:
```bash
bash build100.sh -DS14_SPEEX_BENCHMARK=OFF -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF
bash flash100.sh
```

If build fails, copy the first full compiler/linker error (not just the final make failure); do not bypass errors or assume that an archive compiled for a different CPU is usable.
