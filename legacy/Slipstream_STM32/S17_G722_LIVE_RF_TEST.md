# S17 — G.722 real-time STM32 microphone + RF bandwidth model

**Status:** GitHub code committed; no board build/flash or live test has been performed by the assistant. This is NOT an ESP32 radio test. It models payload and provisional framing only. STM32 I2S/DMA headphone bypass stays unchanged, and G.722 decode is NOT played through the headset.

## Live DSP timing test

Pull `s13-opus-stm32-loopback` in `Slipstream_STM32` then run these tasks, in order:

1. **STM32: Build and Flash S17 G722 Live TX-Only** — isolates microphone DSP + G.722 encoder.
2. **STM32: Build and Flash S17 G722 Live RX+TX** — microphone DSP + G.722 encoder+decoder in foreground.
3. **STM32: Restore DSP-only after S17**, then flash with **STM32: Flash Performance100** when done.

Each S17 task fetches/builds the existing Cortex-M4 static G.722 codec, configures Performance100 with other codec tests disabled, and flashes. The S17 test auto-stops at first DMA error, missed or rejected DSP half (Reason=1), codec error (Reason=2 or 3), frame-queue overflow (Reason=5) or after 30 s (Reason=4). Audio DMA and DSP continue after the codec test stops so the UART reports the final counters.

UART includes the existing `[S11]` DSP fields and appended `[S17LIVE]`:
`P` (produced 20ms frames), `E` encoded, `D` decoded,
`Qdrop` (oldest frames dropped), `Qhi` (max queue occupancy),
`Enc` and `Dec` avg/max microseconds,
`MaxRun` maximum codec call pair duration in microseconds,
`Pkt` G.722 payload bytes/frame,
`Payload` nominal continuous payload bits/second (64000),
`WireEst` nominal bitrate INCLUDING ONLY a **hypothetical** 12-byte per-packet header (68800 bps),
`Err,Stop,Reason`.

**Success** requires `Reason=4`, `Miss=0 Late=0 DMA=0 Qdrop=0 Err=0` after sustained microphone speech, both TX-only and RX+TX. Additional speaker, RF and packet-loss testing remains necessary. A startup benchmark alone cannot establish real-time scheduling stability.

## Bandwidth and airtime arithmetic

Per participant, at 20 ms frames: 50 packets/s × 160 bytes × 8 = **64,000 payload bits/s**; 12-byte theoretical app/RF header makes 172 bytes/packet × 50 × 8 = **68,800 modeled bits/s per stream**.

| Simultaneous audio streams | Payload bit/s | +12-byte assumed header bit/s | Ideal 1 Mbit/s airtime fraction | Ideal 2 Mbit/s airtime fraction |
| --- | ---: | ---: | ---: | ---: |
| 1 | 64,000 | 68,800 | 6.88% | 3.44% |
| 3 | 192,000 | 206,400 | 20.64% | 10.32% |
| 6 | 384,000 | 412,800 | 41.28% | 20.64% |
| 8 | 512,000 | 550,400 | 55.04% | 27.52% |

**These fractions are lower bounds, not measured ESP-NOW channel utilization**. Wi-Fi/ESP-NOW MAC framing, contention, security, preamble, acknowledgments, interframe spacing, retries, other traffic, and mesh relays are OMITTED. Each hop of forwarding consumes additional radio airtime. A 160-byte G.722 payload plus your actual transport header must fit the chosen ESP-NOW maximum frame payload and MTU; verify against your ESP32/IDF and protocol configuration.

`WireEst` in the UART is **not measured radio traffic**. There is no ESP32 I/O, channel transmission, received-packet loss, range, latency or PHY data-rate measurement in this S17 firmware. Those require a paired ESP32 transport implementation with timestamps and seq numbers; e.g. transmit/receive rate, packet loss, effective airtime/PHY rate, burst drop, RTT, jitter and range.

## Baselines from observed startup tests

- Opus C0: 8.48ms encode+decode per 20ms frame (synthetic).
- Speex C0: 11.77ms per 20ms.
- G.722: 5.866ms per 20ms, 160 bytes per frame.
- Codec 2 3200: 23.272ms including 16k<>8k FIR conversion.

For S17, **real microphone signal** and concurrent DSP may alter timing. Compare `[S17LIVE]` results with the original S13 live Opus test (`Late=1` at roughly 5-6 seconds). The S17 test runs the same non-preemptive foreground scheduling scheme to make this comparison meaningful.

## Manual build (after bash setup_g722.sh)
```bash
bash build100.sh -DS13_OPUS_BENCHMARK=OFF -DS13_OPUS_LIVE_TEST=OFF \
 -DS14_SPEEX_BENCHMARK=OFF -DS15_G722_BENCHMARK=OFF \
 -DS16_CODEC2_BENCHMARK=OFF -DS17_G722_LIVE_TEST=ON \
 -DS17_G722_LIVE_DECODE=ON \
 -DS15_G722_ROOT="$PWD/third_party/g722tools" \
 -DS15_G722_BUILD="$PWD/build/g722-m4f"
bash flash100.sh
```
