# Slipstream STM32 S13 — integration plan (Milestone 1 underway)

## Baseline and source review

- Base branch: `main`, the frozen STM32 S7–S11 audio pipeline; **do not merge S12 wind suppression**. Draft S12 PR #6 / `s12-wind-detection-diagnostics` remains independent.
- STM32 `Core/Src/main.c`: SPI2/I2S2 full-duplex via DMA1 streams 4(TX)/3(RX) to WM8960, I2C1 PB8/PB9, USART3 PD8/PD9 for ST-LINK logging, 10ms 480-frame stereo halves at 48kHz, 160 mono 16k voice samples per half. S7 CMSIS FIR and S8–S11 are working and untouched.
- Old `main/codec/opus_codec.cpp/.hpp`, `tests/legacy/h12_opus.cpp`, `tests/legacy/h13_packet.cpp`, `tests/legacy/h14_jitter.cpp`, `tests/h19_bluetooth_mix.cpp` were reviewed. H12 uses a dedicated codec worker/task; H19 has large task stack demands and reduced complexity to zero for simultaneous BT/DSP/Opus. STM32 must benchmark complexity 5, not assume it is viable.
- The old ESP-IDF project gets Opus from **`78/esp-opus` version 1.0.5** (`main/idf_component.yml`, `dependencies.lock`). This is an ESP-IDF package, **not** an already-integrated STM32 ARM library. Port a compatible, licensed upstream C libopus source with an ARM/Cortex-M4F build and required fixed/floating compile configuration. Do not link ESP-IDF-specific prebuilt binaries or pretend `opus.h` currently exists for STM32. Confirm encoder and decoder memory, stack, and actual timing before touching PCM playback.
- Old H13 packet module `main/transport/slipstream_packet.cpp/.hpp`: 16-byte big-endian SL v1 header and IEEE CRC32 over header+payload; reuse after Opus loopback, not in M1.
- Old H14 jitter: 8-packet bound; 3 x 20ms preferred jitter delay; PLC via Opus API. Future phases only.
- `smbeane/slipstream-comm/main/bluetooth/{a2dp,hfp,bluetooth}.cpp` and `main/controls/controls.cpp`: retain Bluetooth Classic A2DP sink/AVRCP and HFP lifecycle/call-codec state patterns. Replace DAC output, repeated-sample resampling, shared booleans, callback blocking, and HFP microphone **zero-filled** outgoing buffer. None of this prototype is an STM32 audio pipeline.

## Hardware link plan — candidates, not yet configured

Existing **occupied** pins/peripherals from STM32 `.ioc` and MSP:
- I2S2 SPI2 / WM8960: PB12 WS, PB13 CK, PB15 SD, PC2 I2S2ext_SD.
- I2C1 / WM8960: PB8 SCL, PB9 SDA.
- USART3 / ST-LINK trace: PD8 TX, PD9 RX — keep it for UART debugging.
- PA9 USB VBUS is already reserved; avoid proposing PA9 USART1.
- Existing DMA1 stream 3+4 are owned by I2S2; preserve these reservations.

**Proposed ESP packet/control UART:** USART2, PD5 TX, PD6 RX, initially 921600 8N1 DMA where DMA mapping permits (validate against existing I2S2 DMA streams and board solder bridges). Separate GND and optional STM-to-ESP reset/ready pins. PD5/PD6 are supported USART2 pins; verify accessible Nucleo header wiring before soldering.

**Proposed ESP Bluetooth PCM:** SPI3/I2S3 full-duplex, PC10 CK, PC12 SD (STM->ESP), PC11 I2S3ext_SD (ESP->STM), PA4 I2S3_WS. These are candidate alternate-function signals and are not currently enabled in CubeMX. STM32 would own the synchronous **48 kHz** I2S clock with ESP as slave; inspect clock-tree sharing and DMA channel/stream collisions in CubeMX before hardware wiring. Bluetooth A2DP native 44.1k and HFP 8/16k must be resampled/buffered on the STM32 or at an explicitly chosen boundary, not sample duplicated; I2S does not carry variable sample rates on this single fixed clock without a planned conversion.

## Proposed UART link (Milestone 3, specification only)

Binary byte stream frame: `magic[2]='S','B' | version:u8=1 | type:u8 | link_sequence:u16 BE | payload_length:u16 BE | payload[N] | crc32:u32 BE`. Header length 8, CRC over full 8-byte header + payload; reflected IEEE CRC32 `init=FFFFFFFF xorout=FFFFFFFF poly=EDB88320`. Reject version/type/length/CRC failures and resynchronize by scanning magic with bounded parser memory. Link sequence is independent of Slipstream RF packet sequence.

Initial types: `RF_TX_PACKET`, `RF_RX_PACKET`, `BT_CONNECTED`, `BT_DISCONNECTED`, `A2DP_STARTED`, `A2DP_STOPPED`, `A2DP_SAMPLE_RATE`, `HFP_CONNECTED`, `HFP_DISCONNECTED`, `HFP_AUDIO_STARTED`, `HFP_AUDIO_STOPPED`, `HFP_CODEC`, `CALL_STATE`, `AVRCP_COMMAND`, `PAIRING_REQUEST`, `PAIRING_STATE`, `TRANSPORT_STATUS`, `TRANSPORT_ERROR`. Assign numeric IDs and payload schema before implementing bridge; don't invent guessed wire IDs now.

Max RF payload in UART: 532 bytes (16+512+4), bounded, but **ESP-NOW radio MTU must be independently validated**; classic ESP-NOW v1 packets have a 250-byte limit, so typical ~50-byte Opus payloads fit but 532-byte worst cases do not. On RF transport enforce negotiated MTU or explicit fragmentation with sequence/timeout; do not silently truncate.

No newline binary framing; support split/coalesced reads, errors, restarts, bounded TX queue, version handshake, counters and stale-frame-drop policy. Protocol is between STM32 and radio transport, not tied to ESP-NOW semantics.

## Milestones and freeze gates

1. **S13 Opus local loopback (current):** assemble every two 10ms DSP blocks, including silence regardless of VAD, into 320 samples. Port real libopus to STM32 using C bindings and compile tests. Preset 16k mono S16, 20ms, VOIP, 20kbps, VBR on, FEC off, DTX off, packet <=512, complexity 5 initially. Encode/decode in a bounded worker context, **never block the 10ms DMA servicing loop**. Playback decoded samples through existing 16->48 FIR path. Show average/max encode/decode time and audio pipeline headroom, stack + RAM, no Miss/Late/DMA. Freeze after headset listening.
2. **Packet loopback:** byte-exact port of H13 packet, CRC corruption tests, seq+1 and timestamp+320, VAD FLAG_SPEECH metadata only. Freeze.
3. **UART transport loopback:** framing, 921600 measurement, overflow/reset/recovery tests with arbitrary byte payloads. Freeze.
4. **ESP-NOW:** transparent ESP UART bridge, counters, MTU handling, two-node transport. Freeze.
5. **Two-node intercom:** RX 3 frame (~60ms) jitter, continuous decode, Opus PLC, resync, latency/reconnect. Freeze.
6. **A2DP PCM:** ESP A2DP sink/AVRCP->I2S->STM, 48k then proper 44.1k. Freeze.
7. **RF + A2DP:** STM priority mixer BT -3dB normal / -20dB absolute duck, RF 0dB; 50ms attack, 200ms hold, 500ms release, RF timeout 1000ms. Freeze.
8. **HFP downlink:** call PCM over I2S into STM RX mixer, CVSD 8k/mSBC 16k conversion. Freeze.
9. **HFP uplink:** STM *local mic only* -> ESP -> phone; never RF RX into phone uplink. Freeze.
10. **RF during HFP:** validate intercom priority and isolation from outgoing phone mic. Freeze.

## M1 implementation status and next gate

**Committed on branch:** `slipstream_s13_frames.h/.c` with a 2-frame bound (drop oldest, never gate on VAD), CMake source registration, and native host assembler unit test. These change no active STM32 output. The assembler is a reusable M1 building block, **not** a working Opus loopback yet.

**Blocked/pending:** real ARM libopus source integration; allocate encoder/decoder state outside the real-time stack, benchmark complexity 5 without running Opus within 10ms critical path, add independent Opus encode/decode job execution, bounded decoded queue with jitter-safe phase, and change output only behind explicit M1 bypass. Existing `s7_process_half` presently owns both processing and output staging; inserting blocking Opus into it would threaten real-time reliability. A separate worker/scheduling strategy must be measured before hook-up.

**Host test:** from `Slipstream_STM32`, run:
```sh
cc -std=c11 -Wall -Wextra -Werror -ICore/Inc tests/s13_frames_host_test.c Core/Src/slipstream_s13_frames.c -o /tmp/s13frames && /tmp/s13frames
```

Do not merge until codec loopback is demonstrably clean on board.

## Practical commands — Performance100 and Opus test gates

Run from `Slipstream_STM32/` on Fedora:
```bash
bash scripts/perf100.sh test       # host 10ms->20ms frame test, no target
bash scripts/perf100.sh opus-host  # host libopus smoke (requires opus-devel)
bash scripts/perf100.sh build      # configure and build STM32 Performance100
bash scripts/perf100.sh flash      # flash existing Performance100 .elf
bash scripts/perf100.sh all        # build then flash
```
The helper selects `STM32_Programmer_CLI` if available, otherwise `st-flash`; existing ST-LINK USART3 logging still works. These are explicit repository scripts, not user shell aliases. Does not install libraries automatically or modify local shell configuration.

The native Opus smoke test at `tests/s13_opus_host_smoke.c` validates encoder and decoder **on the Fedora host**: 16kHz, 320-sample 20ms frames, VOIP, 20kbps, complexity 5, VBR=1, FEC=0, DTX=0, 512-byte maximum packet, with continuous encoding and decoding even for silence. Install native development files with `sudo dnf install opus-devel pkgconf-pkg-config` when missing. Native host results **must not be interpreted as Cortex-M4 timing or RAM validation**.

Target benchmark, **not implemented yet**: after a real ARM Opus library is linked, record 20ms-frame encode/decode average + maximum cycles using DWT CYCCNT, convert to microseconds at SystemCoreClock; report codec state memory and stack peak; monitor the existing 10ms `Proc/Max/Miss/Late/DMA` continuously. At 100MHz 20ms has ~2,000,000 CPU cycles total, but Opus work must be scheduled without blocking or racing the 10ms DMA deadline. Complexity 5 is first, then deliberately benchmark complexity 3, 1, 0 only if necessary. It is not safe to run an unbounded blocking Opus call in `s7_process_half`. Full Opus audio loopback remains pending.
