# S11 — STM32 port of original ESP32 H11 lookahead limiter

## Source fidelity

This port follows ESP32 `main/dsp/limiter.cpp` and `main/dsp/limiter.hpp`. The code is C to fit the STM32 firmware, with a static 16-sample delay ring. The **gain algorithm, thresholds and ordering are unchanged**.

- Signed 16-bit mono PCM at nominal 16 kHz; 160-sample blocks.
- 1.0 ms lookahead / 16-sample delay, preserves state across blocks.
- -3.0 dBFS ceiling, exact linear value `0.707945784f`.
- Attack coefficient: `powf(0.001f, 1.0f / 8.0f)` (0.5 ms at 16k).
- Release coefficient: `expf(-1.0f/(16000*0.100f))` (100 ms).
- Each input sample: look ahead through all 16 ring samples; compute target gain; smooth toward it; cap gain for the **actual delayed output sample**; apply output gain; numerical-safety clamp; `lrintf` quantization.
- `threshold_hits` counts sample-level occasions where lookahead sees more than -3 dBFS; `safety_clamps` should remain zero.
- Insert after H10 AGC and before 16->48 kHz FIR interpolation, as in original ESP32 audio processor.
- Compile-time `S11_ENABLED=1` enables; `=0` bypasses and restores S10 baseline. S7–S10 are unchanged.

**Important:** Limiting at 16 kHz to -3 dBFS cannot undo distortion from upstream AGC (`AClip`) or guarantee that later 48 kHz interpolated peaks (`ClipD/F/NS/O`, particularly final `O`) never exceed full scale. Track these separately.

## Build and flash

```bash
git fetch origin
git switch s11-lookahead-limiter-stm32-port
cd Slipstream_STM32
cube-cmake --preset Performance100
cube-cmake --build --preset Performance100
```

Flash `build/Performance100/Slipstream_STM32.elf`; ensure `CPU=100000000 Hz`.

## Hardware test

1. Start quiet for 0.5-1 second while S8/S9 noise estimators initialize. Use low headphone volume.
2. Normal speech: `Lout10` should remain <= -30 (that is, -3 dBFS ceiling). `LPeak` should be no more than approximately 23198 (allow 1 PCM count for rounding).
3. Quiet speech: expect minimal limiter attenuation, `GR10` near 0; AGC still adjusts gain in S10.
4. Loud speech / sustained tone: `Hits` increments when samples would exceed ceiling, `GR10` is negative during attenuation, `Clamp=0`, `Lout10<=-30`.
5. Listen for unnatural pumping, attacks and pops. Because this algorithm has 1 ms inherent delay, expect marginal latency increase.
6. Compare final output `ClipD/F/NS/O` to S10 tests; if output still clips, distinguish whether it is upstream AGC saturation, upsampling overshoot, or output gain.
7. Confirm stable `Proc`, `Max`, `LIM` microseconds and `Late=0 Miss=0 DMA=0 Uskip=0` on a several-minute run. Watch for edge cases with wind directly on mic; don't change VAD or NS coefficients as part of this milestone.

## New UART telemetry

S11 extends the existing nonblocking USART3 line:
- `LIM=a/b`: latest limiter processing time / maximum since last summary (microseconds).
- `Lin10`: input peak level dBFS x10; `Lout10`: output peak dBFS x10.
- `GR10`: gain reduction in dB x10 (0 or negative).
- `Hits`: threshold hits in last reporting window (not cumulative).
- `Clamp`: numerical safety clamps in last reporting window; target zero.
- `LPeak`: largest absolute limited 16 kHz PCM sample in last reporting window.
- Existing `AClip` and `ClipD/F/NS/O` remain to diagnose pre- and post-limiter saturation.

Do not merge until Performance100 builds on the Nucleo and the limiter output ceiling, reliability, and listening quality have been checked. The source has not been hardware tested through this GitHub connector.
