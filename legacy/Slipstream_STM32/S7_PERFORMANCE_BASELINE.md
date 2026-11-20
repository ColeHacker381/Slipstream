# S7 performance baseline (Phase A)

This benchmark deliberately **does not alter** `Core/Src/main.c`'s FIR coefficients, voice biquads, PCM conversions, DMA-buffer dimensions, or DSP operations. It only provides optimized compiler configurations and an optional 100 MHz core clock.

## Build variants

- `Debug`: existing 72 MHz clock; `-O0 -g3`.
- `Performance72`: existing 72 MHz clock; `-O3 -g3`.
- `Performance100`: 100 MHz core clock; `-O3 -g3`; target MCU USB OTG FS initialization skipped.
- `Release`: existing `-Os -g0` unchanged.

The 100 MHz variant uses HSE bypass 8 MHz, PLLM=4, PLLN=100, PLLP=2, PLLQ=4. SYSCLK/HCLK=100 MHz, APB1=50 MHz, APB2=100 MHz. The PLLQ output is **50 MHz**, NOT valid for USB FS. This is why the target MCU's OTG FS initialization is skipped. The board's onboard ST-LINK SWD and USART3 virtual COM are separate and remain in use. PLLI2S settings for the WM8960 path are unchanged. STM32F412 voltage scale 1 and **FLASH_LATENCY_3** for the assumed 3.3 V target supply are required for 100 MHz (RM0402). Verify the target supply and actual output audio clock before validating audio quality.

## Commands (run on the development host with CMake/Ninja and ARM GNU installed)

```bash
cd Slipstream_STM32
cmake --preset Performance72
cmake --build --preset Performance72
cmake --preset Performance100
cmake --build --preset Performance100
```

Flash each corresponding `build/Performance*/Slipstream_STM32.elf` using the usual ST-LINK/CubeIDE workflow. To maintain a fair comparison, **first benchmark Performance72**, and only then Performance100.

After each flash, confirm `CPU=72000000 Hz` or `CPU=100000000 Hz` on USART3 / ST-LINK VCP (115200 baud).

Record multiple steady-state 1-second reports at `S7_MODE=2`:

| Build | CPU Hz | Proc us | Max us | IRQ/s | Voice/s | Missed | Late | DMAErr | Sound |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| Debug (known) | 72000000 | ~8100 | | ~99 | ~15360 | 0 | rises | 0 | sounds good |
| Performance72 | | | | | | | | | |
| Performance100 | | | | | | | | | |

**Important:** current ISR timing is nominally 10 ms/half, but the saved CubeMX metadata reports an I2S actual rate of approximately 47.348 kHz rather than an exact 48 kHz; ~99 DMA callbacks/real second is therefore plausible. Use measured actual I2S sample rate for accurate end-to-end bandwidth/latency assertions. The current `Late` metric includes intentionally rejected blocks and is not a complete TX-DMA underrun detector. The once-per-second UART write can still perturb scheduling.

## Phase B gate

Do **not** replace `arm_fir_decimate_f32`, `arm_fir_interpolate_f32`, or the PCM conversion order until Performance72 and Performance100 have been measured on hardware. If still slow, profile five stages individually (input copy, decimation, HPF/LPF, interpolation, output copy), then compare a Cortex-M4F-optimized explicit three-phase implementation with the unchanged CMSIS interpolator. Any subsequent modification should go in a separate PR and preserve the original ESP32 FIR values and 16 kHz voice-domain biquads.

## CubeMX regeneration warning

The 100 MHz clock and USB initialization selection are guarded by `SLIPSTREAM_100MHZ` in generated `main.c` code. The `.ioc` remains at the original 72 MHz/USB configuration to preserve the known-good Debug variant. **Regenerating CubeMX code can overwrite the conditional changes.** Re-apply/review the clock variation before using Performance100 after regeneration. The performance compiler options are in the custom toolchain/CMake presets, not in CubeMX.
