# S18 STM32 hardware transport — G.722 data via I2S3, control via UART2

**Status:** STM32-only implementation committed. Has NOT been compiled, flashed, or verified on physical I2S wires. ESP32 receive code is **not yet changed**.

## Architecture

WM8960 mic/headphone -- I2S2/SPI2 DMA --> S7-S11 STM32 DSP (16k mono) --> G.722 320-sample/20ms encoder --> 160 bytes compressed --> **I2S3/SPI3 master TX DMA** --> ESP32 I2S RX slave --> ESP-NOW RF (future).

ESP32 controller/actions -- **UART2** --> STM32: `START\n`, `STOP\n`, `PING\n`, `STATUS\n`. STM32 emits ASCII `S18 active=... made=... enc=... queued=... sent=... drop=... idle=... dma=... enc_us=... qhi=... cmds=... errors=...\n` on UART2, 115200 8N1, around once per second and after received command. USART3 PD8/PD9 remains connected to ST-LINK VCP debug output and is NOT the ESP control UART.

## Wiring — NUCLEO-F412ZG (3.3V GPIO only)

| STM32 board MCU pin | Direction | ESP32 pin/function | Signal |
| --- | --- | --- | --- |
| PC10, SPI3/I2S3 CK AF6 | STM -> ESP | ESP I2S BCLK input | I2S bit clock |
| PA4, SPI3/I2S3 WS AF6 | STM -> ESP | ESP I2S WS/LRCLK input | I2S word select |
| PC12, SPI3/I2S3 SD AF6 | STM -> ESP | ESP I2S DATA IN | encoded G.722 data |
| PA2, USART2 TX AF7 | STM -> ESP | ESP UART RX | control/status UART |
| PA3, USART2 RX AF7 | ESP -> STM | ESP UART TX | UART commands/actions |
| GND | Both | ESP GND | **mandatory shared ground** |

No MCLK is used on the ESP I2S receiver; ESP must be **I2S slave** because STM32 is master. Do **not** connect ESP32 GPIO directly to 5V, or join power rails unless voltage/current regulation is established. If powering independently, share GND and keep both logic sides 3.3V. Confirm all connector header positions against the exact NUCLEO-F412ZG silkscreen and STM32CubeMX pin view; the table specifies MCU GPIO names, **not CN header numbers**. Keep leads short for BCLK.

**Existing WM8960:** PB12 I2S2_WS, PB13 I2S2_CK, PB15 I2S2_SD, PC2 I2S2_ext_SD, PB8/PB9 I2C1; these remain connected and unchanged.

**Debug ST-LINK:** USART3 PD8/PD9, not UART2.

## Binary I2S framing contract for upcoming ESP implementation

I2S link is clocked nominally at 16kHz 16-bit **stereo I2S Philips**. Every 20ms, 640 sequential uint16 halfwords (1280 bytes) are sent. Every DMA half is one complete **20ms transport slot**. Each slot either contains 88 header/payload words followed by 552 zero words or is entirely zero if idle/no packet.

Within the first 88 words:

| Word index | Value |
| --- | --- |
| 0 | `0xA55A` frame sync |
| 1 | `0x1801` S18 format/version |
| 2 | Sequence (16-bit wrapping) |
| 3 | `160` length, bytes |
| 4 | CRC16-CCITT-FALSE (`init=FFFF`, `poly=1021`), computed on raw 160 G.722 bytes |
| 5–7 | 0, reserved for flags/timestamps |
| 8–87 | 80 words containing 160 compressed bytes, each word `(byte[2*i]<<8)|byte[2*i+1]` |
| 88–639 | Zero padding |

The ESP32 I2S driver may store 16-bit samples in memory with byte order or channel alignment different from STM32's raw sequence. Before decoding packets, use logic analyzer or known sync pattern to validate the order, swap if necessary, and resynchronize by searching for `0xA55A,0x1801,160`. The sync and CRC are only for STM-to-ESP capture, not for ESP-NOW frames. There is **no PCM voice playback** on this I2S wire: it transports **compressed** bytes in 16-bit I2S words.

Payload over RF, if each 20ms frame is sent separately, remains **160 bytes × 50/s = 64 kbps**. The physical I2S wire continuously carries stereo 16bit slots at nominal 16kHz: **512 kbps**, including zero padding. This does not represent wireless bandwidth. **Current firmware measures neither actual RF traffic nor RF packet loss.**

## DMA, timing, buffering

- Existing SPI2/I2S2 WM8960 RX+TX DMA remains operational.
- SPI3 TX uses **DMA1 Stream5, Channel0**, circular, IRQ priority 4. Existing mic DMA1 Streams3/4 retain higher priority 0.
- 1280-halfword circular TX DMA split into two 640-halfword halves. DMA callbacks refill only the half that has finished transmitting, so **additional scheduling latency of at least one DMA revolution** may occur. Audio throughput is deterministic, not minimal-latency optimized.
- 20ms post-DSP audio assembler uses existing bounded `S13_FrameAssembler` and queues up to three encoded packets; drop counters are reported. Link starts in **idle mode**: I2S3 clocks run and transmit zeros until ESP sends `START\n` over UART2. This prevents transmitting voice to an unconfigured receiver.
- Codec runs in main foreground only after both mic DMA halves have been serviced, not in IRQ.
- UART commands are collected with receive interrupts and processed in foreground. Periodic UART2 output uses nonblocking HAL UART interrupts.
- **Not yet implemented:** ESP I2S parser, ESP-NOW networking, flow control, received audio path back to STM, playback, multi-stream decode/mix, retry, or clock drift compensation.

## Build/flash

```bash
git pull --ff-only origin s13-opus-stm32-loopback
```

In `Slipstream_STM32` VS Code workspace: **STM32: Build and Flash S18 G722 I2S3 + UART2**. The task builds the existing G.722 archive first and selects S18 while disabling all S13-S17 benchmarks. On the ST-LINK debug UART expect `[S18] I2S3 framed G722 TX ready; UART2 115200 START/STOP/STATUS/PING`. UART2 outputs the status line. Use a 3.3V USB-to-UART adapter temporarily to test `START\n`, `STATUS\n`, `STOP\n` before installing ESP code.

For rollback: run **STM32: Restore DSP-only after S18**, then **STM32: Flash Performance100**.

## Success criteria

1. Build/link and flash without changing WM8960 loopback functionality.
2. Verify SPI3 WS frequency is nominally 16kHz and BCLK nominally 512kHz (actual PLL division may differ).
3. Before START, UART2 reports active=0, sent=0; I2S wire sends idle zero words.
4. After START with speaking, `encoded` and `sent` increase around 50/s, with `drop=0,dma=0` and existing `[S11] Miss=0 Late=0 DMA=0`.
5. STOP leaves I2S clocks running but clears queued packet payload.
6. ESP must recover sync, correct sequence progression and valid CRC. **Not yet tested.**

Source: STM32F412 RM0402 DMA1 mapping: SPI3_TX DMA1 Stream5 Channel0; ST STM32F412 datasheet pin alternate functions for SPI3/I2S3. See upstream ST reference documentation.
