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

## Follow-up: isolate encode-only versus encoder+decoder load

A prior C0 full encode+decode live run processed 236 frames with no queue drops, then stopped on a *DMA boundary during DSP processing* (`Late=1 B/S/I=0/0/1`, `Miss=0 DMA=0`), when maximum codec time reached 12.489 ms on live speech. The remaining CPU headroom is small and it is not yet proven that an RTOS would fix total utilization.

Two VS Code tasks now explicitly select the decoder setting:

1. **STM32: Build and Flash S13 Live TX-Only C0**: real post-DSP mic -> 20ms Opus **encode**, but **no decode**, and normal DSP speaker audio untouched. Boot reports `[S13LIVE] ON: TX ENCODE ONLY`. `D=0` is expected, `E` should track `P`. This identifies whether TX-only workload can sustain 15 seconds without a DSP deadline conflict. It is *not* a full-duplex pass.
2. **STM32: Build and Flash S13 Live Stress C0**: encode **and** decode on every live frame. Boot reports `[S13LIVE] ON: 20ms live PCM -> Opus -> decode`; `E=D` should track `P`.

The tasks share the Performance100 firmware build directory; always run the full **Build and Flash** task when switching variants, and verify the boot-mode banner. Both preserve `S13_OPUS_COMPLEXITY=0` and use fixed-point libopus. Terminal equivalent for TX-only: add `-DS13_OPUS_LIVE_DECODE=OFF` to the C0 live build command above; return to full workload with `-DS13_OPUS_LIVE_DECODE=ON`.

Record the first ~15 seconds of logs under continuous normal speech. Compare `P,E,D,Qdrop,MaxRun,Enc,Dec,Miss,Late,B/S/I,Stop,Reason`. The difference between modes is CPU spent decoding and its uninterrupted scheduling span—not a pure increase in CPU clock or a production-grade scheduler.

If the TX-only test is clean but combined fails, the next experiment is a high-priority DSP worker with lower-priority preemptible codec work, subject to careful buffer ownership and stack capacity audits. If TX-only also fails, optimize codec or DSP workload before adding an RTOS.

## S13 DMA timing diagnostic (live stress mode)

A new timestamp diagnostic is automatically active in both S13 live stress modes, without changing the current strict DSP reject policy. It uses DWT cycle timestamps in each audio DMA half/full callback and records age of an RX half at entry to and exit from the DSP process call.

Appended `[S13LIVE]` fields:
- `DMAAGE=start_max/end_max us`: largest observed elapsed time from that half's DMA completion callback to entering/leaving the DSP processing routine.
- `Over=start_count/end_count`: number of calls where the respective measured age was at least 10ms (the estimated time until the *other* DMA half callback, which begins DMA reuse of the previous half).
- `IRQcross=opposite/same`: observed DSP processing spans during which a callback completed for a different half (the next reuse boundary) or the same half (a full buffer cycle late). These are diagnostics; the callback timing is distinct from a hardware DMA transfer's precise first memory write.
- `RejectAge=start/end us` and `IRQhalf`: ages at the most recent strict DSP rejection and most recently signaled half number (0/1).

For the next test: run **STM32: Build and Flash S13 Live TX-Only C0** as before, speak during the test, and send the first log with `Stop=1`. If `Late=1 B/S/I=0/0/1` accompanies `RejectAge end >= 10000us` and `IRQcross opposite`, the RX half was at or beyond its nominal DMA reuse boundary. `Miss=0` only means the main loop did not skip a ready-counter generation; it does not prove a safe RX snapshot. Do NOT turn off the interrupt-boundary rejection merely to make a benchmark pass.

This is instrumentation of callback-to-DSP service latency, **not** an end-to-end audio latency guarantee or proof of exact write ownership; DMA completion callback jitter and actual DMA write timing can differ. A full scheduler redesign should keep the initial RX copy short and prioritize DSP before codec work.
