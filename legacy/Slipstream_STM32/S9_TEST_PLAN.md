# Slipstream S9 — STM32 adaptive-energy VAD

## Source fidelity

This is a C port of ESP32 `main/dsp/vad.cpp` and `main/dsp/vad.hpp`, inserted in the same location as `main/audio/audio_processor.cpp`: on cleaned S8 16 kHz mono 160-sample blocks, **before H10 AGC**. The PCM samples are **not gated/muted** by VAD. S7 FIR, biquads and S8 FFT/noise suppression remain unchanged.

Exact ESP32 VAD implementation parameters:
- Initial noise learning: 50 blocks (nominal 500 ms) with quiet input.
- Start threshold: max(noise power * 6.309573, -50 dBFS absolute power).
- Continue threshold: min(noise power * 2.511886, start threshold).
- Noise adaptation: `0.995f` old + `0.005f` new, only when inactive and below start threshold.
- Attack: `ATTACK_FRAMES=4`, i.e. **40 ms** for four 10 ms blocks. The ESP32 header/source *comments* say 20 ms, but the executable constant is 4; we deliberately preserve executable behavior.
- Hangover: `HANGOVER_FRAMES=25`. Actual end transition occurs after the countdown reaches 0 and another below-threshold frame arrives; this mirrors ESP32 source.
- Mean square is accumulated as `double` and converted to `float`; do not change this until measurements.
- `#define S9_ENABLED 1` toggles VAD analysis. Bypass leaves audio unchanged.

## Build and first hardware test

```bash
git fetch origin
git switch s9-vad-stm32-port
cd Slipstream_STM32
cube-cmake --preset Performance100
cube-cmake --build --preset Performance100
```

Flash the `build/Performance100/Slipstream_STM32.elf` artifact (not Debug), confirm `CPU=100000000 Hz` on ST-LINK USART3.

- Stay quiet for 1 second after reset so S8 and S9 noise models can learn the environment.
- Once `VReady=1`, remain quiet and confirm `Speech=0`.
- Speak for several seconds; expect `Speech=1`, `Start` to increment.
- Stop speaking: `Speech` should eventually return to 0 and `End` increment, subject to 25-frame countdown and hysteresis.
- Repeat with quiet speech and background fan noise. VAD uses only cleaned audio; no waveform gating yet.
- Ensure `Late=0 Miss=0 DMA=0 B/S/I=0/0/0` and track `Proc` and `VAD` CPU cost.

## UART summary diagnostics

Original S8.1 metrics persist, with S9 appended:
- `VAD=x/y`: last VAD computation time / maximum for previous report interval in microseconds.
- `VReady`: VAD initial noise model ready (0/1).
- `Speech`: current voice activity state (0/1).
- `Lvl10`, `Noise10`: dBFS * 10 (signed integer; e.g. -350 means -35.0 dBFS). These are the **last processed block**, not averages.
- `Start` and `End`: cumulative VAD speech transitions since boot.

Periodic UART is still interrupt-driven; no printing from the DSP real-time path. Serial report length is constrained to the existing 400-byte buffer.

## Validation gate

This branch has not yet been compiled/flashed through GitHub. A passed build and actual Nucleo results are required before merging into main. Observe whether VAD begins in the intended quiet state; loud startup sound can contaminate initial noise-power learning. Also verify memory/stack usage and watch for cost from `double` accumulation on Cortex-M4F.
