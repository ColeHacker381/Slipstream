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

## S8.1 diagnostic validation (no filter changes)

S8.1 adds only diagnostics and interrupt-driven USART3 performance reporting. The 100 MHz `Performance100` S7/S8 algorithm, FIR coefficients, HPF/LPF parameters, spectral suppressor and playback gain are unchanged.

Fields in the `[S8.1]` 1-second UART report:
- `Mic`: largest absolute signed PCM at I2S microphone input, over the report window.
- `Voice`: peak after FIR decimation + frozen HPF/LPF, before noise suppression.
- `NSpeak`: peak immediately after S8 noise suppression (or same as Voice when bypassed).
- `Out`: peak after interpolation, stereo PCM output.
- `ClipD/F/O`: count of PCM saturations before decimator output quantization / after HPF+LPF / after interpolation, **per report window**.
- `ClipNS`: cumulative count of saturation in the suppressor's inverse-FFT-to-PCM conversion (the `NS` component in the `ClipD/F/NS/O` field).
- `B/S/I`: cumulative deadline rejection reasons: processing over 95% budget / RX sequence changed / other DMA half-completion during processing.
- `C`: cumulative occurrences of a new DMA boundary *during* TX output `memcpy`, not a guaranteed underrun detector.
- `Late`: cumulative number of rejected output commits; reasons can overlap for an individual rejection.
- `Uskip`: cumulative number of times a diagnostic report was skipped due to an occupied UART (or UART TX startup failure).
- `Proc` now includes the output buffer `memcpy` for successful blocks; `NS` remains suppressor-only. Both are in microseconds.

Periodic UART uses `HAL_UART_Transmit_IT` instead of blocking `HAL_UART_Transmit`. `USART3_IRQn` has priority 5, below audio DMA at priority 0. Serial boot/error messages before DMA are still blocking by design.

**Test:** compare S8 enabled/disabled at identical microphone gain and sound conditions; observe peaks and clipping while speaking normally and loudly, and when generating steady fan noise. Keep headphone volume conservative. Check that output peak increases arise before or after NS. Do not assume 1-second peaks correspond to the same sample, especially since S8 has 10ms lookback. Confirm `B/S/I` stays zero after removing blocking logging and no new `Late` blocks accumulate. If `Late` still rises, the separate rejection reasons will show why.

This revision remains unbenchmarked and is not ready to merge until built and tested on the Nucleo.
