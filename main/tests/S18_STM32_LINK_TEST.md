# S18 ESP32-WROOM-32C DevKit: STM32 G.722 I2S receive / UART control smoke test

This is a **wired-link diagnostic**, not ESP-NOW yet. Existing H19 Bluetooth and H18 radio test sources are preserved; `main/main.cpp` temporarily selects `s18::run()`.

## Wiring

STM32 NUCLEO-F412ZG and ESP32-WROOM-32C DevKit use 3.3V signals, **share GND**, and can be powered separately via USB.

| STM32 MCU GPIO | ESP32 DevKit GPIO label | Signal |
|---|---|---|
| PC10 SPI3/I2S3 CK | GPIO26 | I2S BCLK, STM master -> ESP slave |
| PA4 SPI3/I2S3 WS | GPIO25 | I2S WS/LRCLK, STM -> ESP |
| PC12 SPI3/I2S3 SD | GPIO22 | I2S data, STM -> ESP |
| PA2 USART2 TX | GPIO16 (RX2) | UART STM -> ESP telemetry |
| PA3 USART2 RX | GPIO17 (TX2) | UART ESP -> STM command |
| GND | GND | common electrical reference |

Do not connect 5V rails. This is **not** the STM debug USART3: keep the ST-LINK USB serial monitor on USART3. Keep I2S wires short.

## Run

The ESP32 project root is **the repository root** (`main/`, `CMakeLists.txt`, `partitions.csv`), NOT `Slipstream_STM32/`.

```bash
git pull --ff-only origin s13-opus-stm32-loopback
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Use the ESP board's actual serial port (`/dev/ttyUSB0` is an example). Ensure the STM S18 firmware is already running and connected. This diagnostic configures UART2 at 115200 8N1 and I2S0 16bit stereo Philips **slave RX** at 16kHz. The ESP sends `START\n` automatically about 0.5 seconds after initialization. If the STM boots later, reset ESP after STM is running; future versions will implement a proper handshake/retry. The ESP also sends `STATUS\n` every 3 seconds and displays replies via USB monitor (UART0).

Expected ESP monitor:
```text
S18 STM32 G722 I2S slave + UART2 test
UART -> STM: START
STM UART: S18 active=1 made=... enc=... queued=... sent=... drop=... idle=... dma=...
RX good=... (+~50.0/s) seq=... gaps=0 repeat=0 crc=0 badhdr=0 I2Sbytes/s=~64000 timeout=0 ioerr=0
```

At 16kHz stereo, 16 bits/slot, the physical I2S transport continuously carries **64000 bytes per second** (512kbit/s) including zero padding. A packet of 160 bytes G722 payload arrives ~50 times per second (64kbps meaningful compressed-audio bitrate). The receiver scans for 16bit `0xA55A` followed by `0x1801`, then checks 160-byte length, CRC16/CCITT-FALSE and 16bit sequence continuity. It supports byte-swapped 16bit words; if the ESP/STM hardware instead swaps entire left/right slots or shifts bits, the parser must be adapted after observing real logs.

**What constitutes a pass:** consecutive logs showing ~50 valid packets/sec, no increasing `crc`, `gaps`, `repeat`, `badhdr`, `timeout`, or `ioerr`, STM UART `drop=0 dma=0`, and STM `[S11] Miss=0 Late=0 DMA=0`. Observe at least 30 seconds of intermittent and sustained speech. The per-second counters from `RX` are cumulative except the reported packet rate and bytes/sec.

If `I2Sbytes/s` is nonzero but `good=0`, check WS/BCLK/data pins and clock/word order with logic analyzer; UART responses and `active=1` show the control plane separately. If `I2Sbytes/s=0` and timeouts climb, check STM clocks and all three I2S wires. If UART status never arrives, check crossed PA2/PA3 and GPIO16/17 and GND. GPIO16/17 may be unavailable on WROOM modules with PSRAM (this particular WROOM-32C should not require them for PSRAM).

**Limitations:** only STM -> ESP audio payload; no ESP->STM audio, G722 decode, packet storage/playback, RF transmission, security, latency measurement or over-the-air packet-loss analysis yet. ESP-NOW will be added after the wired stream passes. Actual ESP RF payload will not carry idle I2S padding.

Source references: STM `Slipstream_STM32/S18_STM32_I2S_UART.md`, ESP-IDF `driver/i2s_std.h` std mode and `driver/uart.h` UART2 API.
