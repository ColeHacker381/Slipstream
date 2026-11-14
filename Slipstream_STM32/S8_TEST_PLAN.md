# S8: ESP32-compatible noise suppression — initial STM32 test

## Implementation
- Existing **Performance100** (100 MHz, -O3) from S7.
- Existing FIR decimation, HPF, LPF, interpolation, and PCM conversion are unchanged.
- New `Core/Src/slipstream_s8.c` ports the algorithm in `main/dsp/noise_suppressor.cpp`.
- 16 kHz mono, 160 samples per block; 320 sample sqrt-Hann frame with 160-sample hop; 512 complex-point FFT.
- 50 blocks (~0.5 seconds at 16 kHz) of quiet noise estimation, then spectral subtraction.
- Original exact parameters: minimum gain 0.12, over-subtraction 1.35, update alpha 0.995, update threshold 2.5, smoothing 0.80/0.20 and 0.35/0.65.
- Switch via `#define S8_ENABLED 1` in `Core/Src/main.c`. Set to 0 to restore S7 **without removing code**.
- Added `NS`, `NSMax`, `Ready` diagnostics, while retaining `Proc`, `Max`, `Missed`, `Late`, `DMAErr`, `IRQ/s`, `Voice/s`.
- The FFT uses `arm_cfft_f32` and a static CMSIS 512-point plan. CMSIS inverse already divides by 512. No extra normalization.
- This code does **not** change TX DMA ownership or HAL UART blocking; remain vigilant for missed/late blocks.

## Build and test (hardware required)
```bash
cd Slipstream_STM32
cube-cmake --preset Performance100
cube-cmake --build --preset Performance100
```

Flash `build/Performance100/Slipstream_STM32.elf` via the verified ST-LINK task.

1. Start with headphones off your ears and low volume. Confirm clock 100000000.
2. Stay quiet for the first 0.5–1.0 second after boot; `Ready` should become 1 after the first 50 blocks.
3. Compare with `S8_ENABLED=0` and `S8_ENABLED=1` on *the same* 100 MHz optimized build.
4. Compare silence, steady fan noise, soft speech, consonants, and abrupt speech onset.
5. Log `Proc`, `Max`, `NS`, `NSMax`, `Missed`, `Late`, `DMAErr`, `IRQ/s`, `Voice/s` over minutes. Stop if clipping/crackling or counters increment.
6. If processing exceeds 10 ms or counters increase, revert `S8_ENABLED=0` and provide log output; do **not** loosen deadlines to hide a performance regression.

## Constraints and caveats
This PR has not been built or validated on hardware through GitHub. Inspect build/link errors first if CMSIS table sources are excluded by generated build settings. Full `arm_common_tables.c` is a large translation unit and may increase compilation time. We must measure actual runtime and flash/RAM before merging S8. S8 adds one 160-sample algorithmic lookback (~10 ms) and may exhibit a startup ramp as expected for overlap-add.
