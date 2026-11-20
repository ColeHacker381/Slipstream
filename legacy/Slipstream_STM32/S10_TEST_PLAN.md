# S10: ESP32-compatible VAD-aware AGC — STM32 first hardware test

## Source and DSP behavior

S10 ports ESP32 `main/dsp/agc.cpp` and `main/dsp/agc.hpp` into `Core/Src/slipstream_s10.c`. It processes the 160-sample, 16 kHz signed PCM **after S8 noise suppression and S9 VAD** and **before S11 (future) limiter and FIR interpolation**. S7–S9 algorithm parameters are unchanged.

Exact ESP32 AGC configuration:
- Desired speech RMS: **-24 dBFS**
- Minimum gain: **-12 dB**; maximum gain: **+18 dB**
- Attack / gain reduction: **30 ms**
- Release / gain increase: **400 ms**
- No speech: slowly return to **0 dB** over a **2-second** time constant; do not chase the background noise floor
- One-pole update per 10 ms block: `alpha=1-exp(-0.010/tau)`
- Converts dB to linear using `powf(10, dB/20)`; rounds to PCM with `lrintf` and counts saturation
- RMS computation uses double-precision accumulation/sqrt/log10, mirroring original ESP32. This may cost more cycles on Cortex-M4F than float.

Do not add an early limiter, output gain adjustment, or change the FIR/biquad/NS/VAD. **S11** is still responsible for speech peak limiting; future final-output clipping after 16→48 kHz interpolation must be evaluated separately.

The compile-time switch `#define S10_ENABLED 1` in `Core/Src/main.c` enables AGC. Setting `0` bypasses AGC while keeping S9 active. If disabling S9, AGC gets an inactive speech flag, just like the original VAD-disabled default behavior.

## Build and first Nucleo test

```bash
git fetch origin
git switch s10-agc-stm32-port
cd Slipstream_STM32
cube-cmake --preset Performance100
cube-cmake --build --preset Performance100
```

Flash **build/Performance100/Slipstream_STM32.elf**, not the Debug ELF. Start with low headphone volume and keep quiet for the initial 0.5–1 second while S8/S9 learn the ambient noise.

1. During silence `Speech=0`, AGC gain should move gradually toward `Gain10=0`, not increase to +180.
2. Speak quietly: `Speech=1`, gain should rise gradually toward the -24 dBFS speech RMS target; test consonant intelligibility.
3. Speak loudly: gain should fall relatively quickly. No severe volume pumping or distortion.
4. Apply fan/helmet wind noise without speaking: record whether VAD falsely triggers and causes AGC to increase gain; this is a known S9 limitation, not evidence AGC alone can reject wind.
5. Record `Proc`, `Max`, `AGC`, `Miss`, `Late`, `B/S/I`, `DMA`, `AClip` and `ClipD/F/NS/O`. Occasional PCM clip at output may remain until S11.
6. Compare `S10_ENABLED=0` vs `1` on Performance100 if speech sounds distorted.

## Diagnostic fields

The original S8.1 + S9 UART line remains; new fields are:
- `AGC=x/y` = last AGC processing time / maximum time since previous 1-second report, microseconds.
- `Gain10` = current applied AGC gain in tenths of a dB; e.g. `Gain10=65` means **+6.5 dB**, and `Gain10=-25` means **-2.5 dB**.
- `Ain10` and `Aout10` = last-block input/output RMS in tenths of a dBFS (signed, e.g. -350 -> -35.0 dBFS).
- `AClip` = AGC-stage clipped samples within the most recent report interval; should ideally be 0 under ordinary speech.

## Validation gate

**Uncompiled and unflashed via GitHub connector**. Keep PR draft until Performance100 builds, actual UART logs confirm zero Late/Miss/DMA errors, gain responds appropriately in quiet/loud speech, and no unacceptable noise amplification or clipping. Original `s10_process_block` calculates two RMS values and updates the gain only on completed blocks; software timing depends on CMSIS DSP + libm costs.
