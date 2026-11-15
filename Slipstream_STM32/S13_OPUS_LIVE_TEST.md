# S13 — opt-in live DSP + Opus sustained stress test

This is a **bench-only failure-detection experiment**, not a real-time production scheduler or a headphone Opus loopback. It uses live microphone PCM after the existing 16kHz DSP/NS/VAD/AGC/limiter, but the existing 48kHz WM8960 speaker playback remains the original DSP signal. No VAD gating. It preserves the 10ms DMA servicing loop.

The existing Opus startup benchmark remains: it runs 50 synthetic frames before DMA. Then optional S13LIVE initializes its own persistent encoder/decoder states once before DMA, starts the normal DMA audio loop and assembles every two 160-sample DSP blocks into one 320-sample Opus frame. The frame assembler has **two pending 20ms frames**, plus one 20ms staging frame (up to ~60ms buffered material, not a latency guarantee), drops oldest when full, and counts drops. Whenever both DMA halves have been serviced, the foreground main loop attempts **one** Opus encode+decode; this blocks foreground for an entire codec invocation and may miss a DMA deadline even though interrupts continue. The experiment automatically disables live Opus on *any* reported DMA miss, rejected DSP deadline, DMA error, codec error, or after 15 seconds. Baseline DSP playback continues after live Opus stops. No CRC, RF, Bluetooth or jitter buffer added.

**Build and flash (VS Code):** `Terminal > Run Task > STM32: Build and Flash S13 Live Stress C0`. Assumes `bash setup_opus.sh fixed` has already built `build/opus-fixed-m4f/libopus.a`. Uses Performance100, 100MHz, C0, real libopus 16k mono 20ms 20kbps VBR, no FEC or DTX.

**Terminal equivalent** from `Slipstream_STM32`:

```sh
bash build100.sh -DS13_OPUS_BENCHMARK=ON -DS13_OPUS_LIVE_TEST=ON \
    -DS13_OPUS_COMPLEXITY=0 \
    -DS13_OPUS_INCLUDE_DIR="$PWD/third_party/opus/include" \
    -DS13_OPUS_LIBRARY="$PWD/build/opus-fixed-m4f/libopus.a"
bash flash100.sh
```

Watch `[S13LIVE] ON` at startup and each subsequent S11 telemetry line with appended `[S13LIVE] ...`:

- `P`: full 20ms frames produced from actual post-DSP microphone.
- `E`, `D`: frames successfully encoded and decoded; `E` and `D` should grow at **50 per second** for steady real time.
- `Qdrop`: dropped oldest when queue filled. This must remain zero.
- `Qhi`: high-water pending 20ms frames (maximum 2).
- `Enc`, `Dec`: encode/decode average and maximum, microseconds.
- `MaxRun`: maximum encode+decode time measured.
- `Bmax`: maximum observed Opus packet length, bytes.
- `Err`: codec errors.
- `Stop=1 Reason=1`: first detected DSP miss/deadline/DMA error; `Reason=2/3`: Opus encode/decode error; `Reason=4`: automatic 15s timeout.
- Existing `[S11] Miss/Late/DMA`: report whether real-time audio survived. If this test stops on the first disruption, that is a useful **negative** result: not enough contiguous CPU/scheduling headroom for this naive single-core implementation.

**Pass criteria:** `P` and `E` roughly 50/s, `Qdrop=0`, existing `Miss=0 Late=0 DMA=0` throughout, and the run reaches timeout 15s (`Reason=4`). The first warmup seconds can shift actual counters but sustained rates must hold. This does **not** prove RF full-duplex viability, realistic codec worst-case, or 30–60ms end-to-end latency. Absent any drops, actual microphone speech and packet length distribution still require extended tests.

**If audio glitches or the live test cannot keep up:** this was predicted as possible by startup CPU budget (~10ms DSP + ~8.5ms Opus per 20ms); restore DSP immediately with `bash build100.sh -DS13_OPUS_LIVE_TEST=OFF -DS13_OPUS_BENCHMARK=OFF && bash flash100.sh` (or **STM32: Restore DSP-only Performance100** then **STM32: Flash Performance100**). The bench state and test warnings stay isolated on this branch. Do not increase the allowed DSP 10ms deadline: it cannot fix utilization.

**Limitations:** initialization uses one-time `malloc` for Opus states; stack and heap watermarks are not yet verified. The 50-frame startup synthetic benchmark may favor silence-like compression (15-byte output); this live test records max encoded packet size and exercises the live mic, but must also be tested during sustained speech. The main-loop worker is not preemptible in foreground and is **deliberately not production-ready**.
