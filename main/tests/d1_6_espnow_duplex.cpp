#include "d1_6_espnow_duplex.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"
#include "transport/espnow_transport.hpp"
#include "transport/slipstream_packet.hpp"
#include "transport/jitter_buffer.hpp"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {
namespace tr = slipstream::transport;
constexpr char TAG[] = "D1_7";

constexpr size_t HW_FRAMES = audio_processor::HARDWARE_FRAMES_PER_BLOCK;
constexpr size_t BLOCK_SAMPLES = audio_processor::VOICE_SAMPLES_PER_BLOCK;
constexpr size_t FRAME_SAMPLES = slipstream::OpusCodec::FRAME_SAMPLES;
static_assert(HW_FRAMES == 480 && BLOCK_SAMPLES == 160 && FRAME_SAMPLES == 320);

constexpr int64_t FRAME_PERIOD_US = 20000;
constexpr uint8_t STREAM_ID = 1;
constexpr uint8_t CHANNEL = 0;
constexpr int64_t JITTER_START_US = 60000;
constexpr UBaseType_t PLAYBACK_PREFILL_FRAMES = 2;
constexpr int16_t MAX_SEQUENCE_LEAD = 6;
constexpr uint32_t MAX_CONSECUTIVE_PLC = 4;
constexpr int64_t MAX_SCHEDULER_LAG_US = 60000;
constexpr unsigned MAX_CATCHUP_FRAMES = 2;
constexpr int64_t PEER_TIMEOUT_US = 2000000;
constexpr unsigned MAX_RX_DRAIN = 8;
constexpr size_t MAX_OPUS_BYTES = tr::EspNowTransport::MAX_PACKET_SIZE -
                                  tr::HEADER_SIZE - tr::CRC_SIZE;
static_assert(MAX_OPUS_BYTES <= tr::MAX_PAYLOAD_SIZE);

// ESP-IDF task stack arguments are BYTES. Leave the original proven
// allocations unchanged for D1.7; reduce only after measuring high-water marks.
constexpr uint32_t AUDIO_STACK = 12 * 1024;
constexpr uint32_t ENCODER_STACK = 30 * 1024;
constexpr uint32_t DECODER_STACK = 14 * 1024;
constexpr uint32_t LINK_STACK = 6 * 1024;
constexpr uint32_t MONITOR_STACK = 4096;

struct PcmFrame {
    int16_t pcm[FRAME_SAMPLES] = {};
    uint16_t sequence = 0;
    uint32_t epoch = 0;
    int64_t created_us = 0;
};
struct WireFrame {
    uint8_t data[tr::EspNowTransport::MAX_PACKET_SIZE] = {};
    uint16_t length = 0;
    int64_t encoded_us = 0;
};
struct DecodeRequest {
    bool missing = true;
    uint16_t sequence = 0;
    uint16_t length = 0;
    uint32_t epoch = 0;
    int64_t received_us = 0;
    uint8_t payload[MAX_OPUS_BYTES] = {};
};
struct CodecMetrics {
    uint32_t frames = 0, errors = 0, drops = 0, over20 = 0, plc = 0;
    uint64_t elapsed_sum = 0, queue_wait_sum = 0;
    uint32_t elapsed_max = 0, queue_wait_max = 0;
    uint32_t pcm_peak = 0, pcm_samples = 0;
    uint64_t pcm_sq = 0;
    uint32_t stack_free = 0;
};
struct LinkMetrics {
    uint32_t tx = 0, tx_fail = 0, rx = 0, delivered = 0, missing = 0;
    uint32_t bad_crc = 0, invalid = 0, ignored = 0;
    uint32_t late = 0, duplicate = 0, full = 0, too_large = 0;
    uint32_t decode_drops = 0, resync = 0, resync_lead = 0;
    uint32_t resync_clock = 0, resync_plc = 0, reconnects = 0;
    uint32_t jitter_max = 0;
    int16_t lead_max = 0;
    uint64_t rx_jitter_sum = 0, tx_queue_sum = 0;
    uint32_t rx_jitter_n = 0, tx_queue_n = 0;
    uint32_t rx_jitter_max = 0, tx_queue_max = 0;
    uint32_t stack_free = 0;
};
struct AudioMetrics {
    uint32_t blocks = 0, max_us = 0, over10 = 0;
    uint32_t underruns = 0, tx_replacements = 0;
    uint32_t rx_errors = 0, tx_errors = 0, short_reads = 0;
    uint32_t starts = 0, rebuffers = 0, seq_gaps = 0;
    uint32_t mic_peak = 0, mic_samples = 0;
    uint64_t mic_sq = 0;
    uint64_t receive_to_dac_sum = 0;
    uint32_t receive_to_dac_max = 0, receive_to_dac_n = 0;
    uint32_t stack_free = 0;
};

QueueHandle_t pcm_tx_q = nullptr, wire_tx_q = nullptr;
QueueHandle_t decode_q = nullptr, pcm_rx_q = nullptr;
QueueHandle_t enc_report_q = nullptr, dec_report_q = nullptr;
QueueHandle_t link_report_q = nullptr, audio_report_q = nullptr;

slipstream::OpusCodec encoder;
slipstream::OpusCodec decoder;
tr::EspNowTransport radio;
tr::PacketJitterBuffer jitter; // Only link_task touches jitter.

int16_t hw_in[HW_FRAMES * 2] = {};
int16_t hw_out[HW_FRAMES * 2] = {};
int16_t voice_block[BLOCK_SAMPLES] = {};
int16_t silence_block[BLOCK_SAMPLES] = {};

uint32_t abs_sample(int16_t sample) {
    const int32_t n = sample;
    return static_cast<uint32_t>(n < 0 ? -n : n);
}
void sample_meter(const int16_t *samples, size_t n, uint32_t &peak,
                  uint64_t &squares, uint32_t &count) {
    for (size_t i = 0; i < n; ++i) {
        const int32_t x = samples[i];
        peak = std::max(peak, abs_sample(samples[i]));
        squares += static_cast<uint64_t>(x * x);
    }
    count += static_cast<uint32_t>(n);
}
uint32_t rms(uint64_t squares, uint32_t count) {
    return count ? static_cast<uint32_t>(std::sqrt(static_cast<double>(squares) / count)) : 0;
}
int16_t seq_diff(uint16_t seq, uint16_t reference) {
    return static_cast<int16_t>(static_cast<uint16_t>(seq - reference));
}
template<class T> bool enqueue_latest(QueueHandle_t q, const T &value) {
    if (xQueueSend(q, &value, 0) == pdTRUE) return false;
    T discarded = {};
    xQueueReceive(q, &discarded, 0);
    // One producer per queue: no competing producer can fill it between calls.
    xQueueSend(q, &value, 0);
    return true;
}
void observe_duration(CodecMetrics &m, uint32_t us) {
    ++m.frames;
    m.elapsed_sum += us;
    m.elapsed_max = std::max(m.elapsed_max, us);
    if (us > FRAME_PERIOD_US) ++m.over20;
}
void observe_wait(CodecMetrics &m, uint32_t us) {
    m.queue_wait_sum += us;
    m.queue_wait_max = std::max(m.queue_wait_max, us);
}
void report_codec(QueueHandle_t q, CodecMetrics &m, int64_t &last) {
    const int64_t now = esp_timer_get_time();
    if (now - last < 1000000) return;
    m.stack_free = uxTaskGetStackHighWaterMark(nullptr);
    xQueueOverwrite(q, &m);
    m = {};
    last = now;
}

bool initialize_audio() {
    if (wm8960::init_i2c() != ESP_OK || !wm8960::probe() ||
        wm8960::configure_full_duplex() != ESP_OK ||
        audio::init_full_duplex() != ESP_OK) {
        ESP_LOGE(TAG, "WM8960/I2S initialization failed");
        return false;
    }
    audio_processor::reset();
    audio_processor::set_noise_suppression_enabled(true);
    audio_processor::set_vad_enabled(true);
    audio_processor::set_agc_enabled(true);
    audio_processor::set_limiter_enabled(true);
    ESP_LOGI(TAG, "WM8960/I2S/DSP ready");
    return true;
}

// Core 0. No decoder allocation in this task.
void encoder_task(void *) {
    if (!encoder.init_encoder()) {
        ESP_LOGE(TAG, "Opus encoder init failed");
        vTaskDelete(nullptr); return;
    }
    encoder.reset();
    PcmFrame input = {};
    WireFrame wire = {};
    uint8_t compressed[MAX_OPUS_BYTES] = {};
    CodecMetrics m = {};
    int64_t last = esp_timer_get_time();
    while (true) {
        if (xQueueReceive(pcm_tx_q, &input, portMAX_DELAY) != pdTRUE) continue;
        const int64_t start = esp_timer_get_time();
        if (input.created_us > 0 && start >= input.created_us)
            observe_wait(m, static_cast<uint32_t>(start - input.created_us));
        uint32_t encode_us = 0;
        const int encoded = encoder.encode_frame(input.pcm, compressed,
                                                 sizeof(compressed), encode_us);
        observe_duration(m, encode_us);
        if (encoded <= 0) { ++m.errors; report_codec(enc_report_q, m, last); continue; }
        tr::PacketHeader header = {};
        header.stream_id = STREAM_ID;
        header.channel = CHANNEL;
        header.sequence = input.sequence;
        header.timestamp = static_cast<uint32_t>(input.sequence) * FRAME_SAMPLES;
        size_t packet_len = 0;
        if (!tr::build_packet(header, compressed, static_cast<size_t>(encoded),
                              wire.data, sizeof(wire.data), packet_len)) {
            ++m.errors; report_codec(enc_report_q, m, last); continue;
        }
        wire.length = static_cast<uint16_t>(packet_len);
        wire.encoded_us = esp_timer_get_time();
        if (enqueue_latest(wire_tx_q, wire)) ++m.drops;
        report_codec(enc_report_q, m, last);
    }
}

// Core 0. Keep packet handling ahead of encoder without blocking Wi-Fi.
void link_task(void *) {
    ESP_LOGI(TAG, "ESP-NOW link on core %d", xPortGetCoreID());
    uint8_t peer_mac[6] = {};
    bool have_peer = false, playout_active = false, waiting_for_fresh = false;
    uint16_t expected = 0, newest = 0;
    bool newest_valid = false;
    uint32_t stream_epoch = 0, consecutive_missing = 0;
    int64_t last_peer = 0, next_playout = 0;
    WireFrame outgoing = {};
    tr::EspNowTransport::ReceivedPacket received = {};
    DecodeRequest request = {};
    uint8_t packet[tr::MAX_PACKET_SIZE] = {};
    LinkMetrics m = {};
    int64_t last = esp_timer_get_time();

    // Arrival timestamps are local to this receiver; unlike remote timestamps,
    // they can be used to measure receiver-side latency without clock sync.
    struct Arrival { uint16_t sequence = 0; int64_t us = 0; };
    Arrival arrival[tr::PacketJitterBuffer::CAPACITY] = {};
    auto restart = [&](uint16_t anchor, int64_t now, bool count) {
        jitter.reset();
        for (auto &a : arrival) a = {};
        expected = anchor;
        newest = anchor;
        newest_valid = true;
        consecutive_missing = 0;
        playout_active = false;
        waiting_for_fresh = false;
        next_playout = now + JITTER_START_US;
        ++stream_epoch;
        if (count) ++m.resync;
    };

    while (true) {
        if (xQueueReceive(wire_tx_q, &outgoing, 0) == pdTRUE) {
            const int64_t now = esp_timer_get_time();
            if (outgoing.encoded_us && now >= outgoing.encoded_us) {
                const uint32_t age = static_cast<uint32_t>(now - outgoing.encoded_us);
                m.tx_queue_sum += age;
                m.tx_queue_max = std::max(m.tx_queue_max, age);
                ++m.tx_queue_n;
            }
            if (radio.send(outgoing.data, outgoing.length)) ++m.tx;
            else ++m.tx_fail;
        }

        for (unsigned i = 0; i < MAX_RX_DRAIN; ++i) {
            if (!radio.receive(received, 0)) break;
            tr::ParsedPacket parsed = {};
            const auto status = tr::parse_packet(received.data, received.length, parsed);
            if (status == tr::ParseResult::BAD_CRC) { ++m.bad_crc; continue; }
            if (status != tr::ParseResult::OK ||
                parsed.header.stream_id != STREAM_ID ||
                parsed.header.channel != CHANNEL ||
                parsed.payload_length == 0 || parsed.payload_length > MAX_OPUS_BYTES) {
                ++m.invalid; continue;
            }
            const int64_t now = esp_timer_get_time();
            const uint16_t sequence = parsed.header.sequence;
            if (!have_peer) {
                std::memcpy(peer_mac, received.source_mac, sizeof(peer_mac));
                have_peer = true;
                restart(sequence, now, false);
                ESP_LOGI(TAG, "PEER CONNECTED %02X:%02X:%02X:%02X:%02X:%02X RSSI=%d",
                         peer_mac[0], peer_mac[1], peer_mac[2], peer_mac[3],
                         peer_mac[4], peer_mac[5], static_cast<int>(received.rssi));
            }
            if (std::memcmp(peer_mac, received.source_mac, sizeof(peer_mac)) != 0) {
                ++m.ignored; continue;
            }
            last_peer = now;
            ++m.rx;
            if (waiting_for_fresh) restart(sequence, now, true);
            const int16_t lead = seq_diff(sequence, expected);
            m.lead_max = std::max(m.lead_max, lead);
            if (lead >= MAX_SEQUENCE_LEAD) {
                ++m.resync_lead;
                restart(sequence, now, true);
            }
            if (!newest_valid || seq_diff(sequence, newest) > 0) {
                newest = sequence;
                newest_valid = true;
            }
            const auto insert = jitter.insert(received.data, received.length, sequence);
            switch (insert) {
                case tr::JitterInsertResult::OK:
                    arrival[sequence % tr::PacketJitterBuffer::CAPACITY] = {sequence, now};
                    break;
                case tr::JitterInsertResult::LATE: ++m.late; break;
                case tr::JitterInsertResult::DUPLICATE: ++m.duplicate; break;
                case tr::JitterInsertResult::FULL: ++m.full; break;
                case tr::JitterInsertResult::TOO_LARGE: ++m.too_large; break;
            }
            m.jitter_max = std::max(m.jitter_max,
                                    static_cast<uint32_t>(jitter.size()));
        }

        int64_t now = esp_timer_get_time();
        if (have_peer && now - last_peer > PEER_TIMEOUT_US) {
            ESP_LOGW(TAG, "Peer disconnected");
            have_peer = false;
            playout_active = false;
            waiting_for_fresh = false;
            newest_valid = false;
            jitter.reset();
            for (auto &a : arrival) a = {};
            ++stream_epoch;
            ++m.reconnects;
        }
        if (have_peer && !playout_active && !waiting_for_fresh && now >= next_playout)
            playout_active = true;

        if (playout_active && now - next_playout > MAX_SCHEDULER_LAG_US) {
            ++m.resync_clock;
            if (newest_valid) restart(newest, now, true);
            else { playout_active = false; waiting_for_fresh = true; jitter.reset(); }
        }

        unsigned processed = 0;
        while (playout_active && now >= next_playout && processed < MAX_CATCHUP_FRAMES) {
            request = {};
            request.sequence = expected;
            request.epoch = stream_epoch;
            request.missing = true;
            jitter.set_expected_sequence(expected);
            size_t packet_length = 0;
            if (jitter.take(expected, packet, sizeof(packet), packet_length)) {
                tr::ParsedPacket parsed = {};
                if (tr::parse_packet(packet, packet_length, parsed) == tr::ParseResult::OK &&
                    parsed.payload_length > 0 &&
                    parsed.payload_length <= sizeof(request.payload)) {
                    request.length = static_cast<uint16_t>(parsed.payload_length);
                    std::memcpy(request.payload, parsed.payload, parsed.payload_length);
                    request.missing = false;
                    Arrival &a = arrival[expected % tr::PacketJitterBuffer::CAPACITY];
                    if (a.us > 0 && a.sequence == expected) {
                        request.received_us = a.us;
                        const uint32_t age = static_cast<uint32_t>(now - a.us);
                        m.rx_jitter_sum += age;
                        m.rx_jitter_max = std::max(m.rx_jitter_max, age);
                        ++m.rx_jitter_n;
                        a.us = 0;
                    }
                }
            }
            if (request.missing) { ++m.missing; ++consecutive_missing; }
            else { ++m.delivered; consecutive_missing = 0; }
            if (consecutive_missing > MAX_CONSECUTIVE_PLC) {
                ++m.resync_plc; ++m.resync;
                playout_active = false;
                waiting_for_fresh = true;
                jitter.reset();
                for (auto &a : arrival) a = {};
                ++stream_epoch;
                break;
            }
            if (enqueue_latest(decode_q, request)) ++m.decode_drops;
            ++expected;
            next_playout += FRAME_PERIOD_US;
            ++processed;
            now = esp_timer_get_time();
        }
        now = esp_timer_get_time();
        if (now - last >= 1000000) {
            m.stack_free = uxTaskGetStackHighWaterMark(nullptr);
            xQueueOverwrite(link_report_q, &m);
            m = {};
            last = now;
        }
        vTaskDelay(1);
    }
}

// Core 1. No encoder allocation in this task.
void decoder_task(void *) {
    if (!decoder.init_decoder()) {
        ESP_LOGE(TAG, "Opus decoder init failed");
        vTaskDelete(nullptr); return;
    }
    decoder.reset();
    DecodeRequest request = {};
    PcmFrame output = {};
    CodecMetrics m = {};
    int64_t last = esp_timer_get_time();
    while (true) {
        if (xQueueReceive(decode_q, &request, portMAX_DELAY) != pdTRUE) continue;
        uint32_t decode_us = 0;
        const int samples = request.missing
            ? decoder.decode_plc(output.pcm, decode_us)
            : decoder.decode_frame(request.payload, request.length, output.pcm, decode_us);
        if (request.missing) ++m.plc;
        observe_duration(m, decode_us);
        if (samples != static_cast<int>(FRAME_SAMPLES)) {
            ++m.errors;
            std::memset(output.pcm, 0, sizeof(output.pcm));
        } else {
            sample_meter(output.pcm, FRAME_SAMPLES, m.pcm_peak, m.pcm_sq, m.pcm_samples);
        }
        output.sequence = request.sequence;
        output.epoch = request.epoch;
        output.created_us = request.received_us;
        if (enqueue_latest(pcm_rx_q, output)) ++m.drops;
        report_codec(dec_report_q, m, last);
    }
}

// Core 1. I2S drives cadence and keeps DSP on a separate core from RF/Opus TX.
void audio_task(void *) {
    if (!initialize_audio()) {
        ESP_LOGE(TAG, "Audio startup failed");
        vTaskDelete(nullptr); return;
    }
    ESP_LOGI(TAG, "Audio ready on Core %d", xPortGetCoreID());
    PcmFrame outgoing = {}, incoming = {};
    uint16_t tx_sequence = 0;
    unsigned tx_half = 0, rx_half = 0;
    bool playback_started = false, have_rx_frame = false;
    bool epoch_valid = false, previous_valid = false;
    uint32_t playback_epoch = 0;
    uint16_t previous_sequence = 0;
    AudioMetrics m = {};
    int64_t last = esp_timer_get_time();
    while (true) {
        size_t bytes_read = 0;
        const esp_err_t read_result = audio::read(hw_in, sizeof(hw_in), &bytes_read);
        if (read_result != ESP_OK) {
            ++m.rx_errors;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (bytes_read != sizeof(hw_in)) { ++m.short_reads; continue; }
        const int64_t start = esp_timer_get_time();
        audio_processor::Stats stats = {};
        audio_processor::process_input_block(hw_in, voice_block, stats);
        sample_meter(voice_block, BLOCK_SAMPLES, m.mic_peak, m.mic_sq, m.mic_samples);
        std::memcpy(&outgoing.pcm[tx_half * BLOCK_SAMPLES], voice_block, sizeof(voice_block));
        if (++tx_half == 2) {
            outgoing.sequence = tx_sequence++;
            outgoing.created_us = esp_timer_get_time();
            if (enqueue_latest(pcm_tx_q, outgoing)) ++m.tx_replacements;
            tx_half = 0;
        }
        if (!playback_started && uxQueueMessagesWaiting(pcm_rx_q) >= PLAYBACK_PREFILL_FRAMES) {
            playback_started = true;
            ++m.starts;
        }
        if (playback_started && (!have_rx_frame || rx_half == 2)) {
            PcmFrame next = {};
            if (xQueueReceive(pcm_rx_q, &next, 0) == pdTRUE) {
                if (!epoch_valid || next.epoch != playback_epoch) {
                    playback_epoch = next.epoch;
                    epoch_valid = true;
                    previous_valid = false;
                }
                if (previous_valid && next.sequence != static_cast<uint16_t>(previous_sequence + 1))
                    ++m.seq_gaps;
                previous_sequence = next.sequence;
                previous_valid = true;
                incoming = next;
                have_rx_frame = true;
                rx_half = 0;
                // This is local receipt-to-playback software latency, not
                // full microphone-to-ear latency (remote clocks are unsynced).
                if (incoming.created_us > 0) {
                    const int64_t age = esp_timer_get_time() - incoming.created_us;
                    if (age >= 0 && age < 10000000) {
                        const uint32_t us = static_cast<uint32_t>(age);
                        m.receive_to_dac_sum += us;
                        m.receive_to_dac_max = std::max(m.receive_to_dac_max, us);
                        ++m.receive_to_dac_n;
                    }
                }
            } else {
                ++m.underruns;
                ++m.rebuffers;
                playback_started = false;
                have_rx_frame = false;
                rx_half = 0;
                previous_valid = false;
            }
        }
        const int16_t *playback = silence_block;
        if (playback_started && have_rx_frame) {
            playback = &incoming.pcm[rx_half * BLOCK_SAMPLES];
            ++rx_half;
        }
        audio_processor::render_voice_block(playback, hw_out, stats);
        const uint32_t processing_us = static_cast<uint32_t>(esp_timer_get_time() - start);
        ++m.blocks;
        m.max_us = std::max(m.max_us, processing_us);
        if (processing_us > 10000) ++m.over10;
        if (audio::write(hw_out, sizeof(hw_out)) != ESP_OK) ++m.tx_errors;
        const int64_t now = esp_timer_get_time();
        if (now - last >= 1000000) {
            m.stack_free = uxTaskGetStackHighWaterMark(nullptr);
            xQueueOverwrite(audio_report_q, &m);
            m = {};
            last = now;
        }
    }
}

void monitor_task(void *) {
    CodecMetrics enc = {}, dec = {};
    LinkMetrics link = {};
    AudioMetrics a = {};
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (xQueueReceive(enc_report_q, &enc, 0) == pdTRUE) {
            ESP_LOGI(TAG, "ENC | frames=%lu avg=%llu max=%lu over20=%lu err=%lu drops=%lu queuewait_avg=%llu queuewait_max=%lu stack=%lu",
                     (unsigned long)enc.frames,
                     (unsigned long long)(enc.frames ? enc.elapsed_sum / enc.frames : 0),
                     (unsigned long)enc.elapsed_max, (unsigned long)enc.over20,
                     (unsigned long)enc.errors, (unsigned long)enc.drops,
                     (unsigned long long)(enc.frames ? enc.queue_wait_sum / enc.frames : 0),
                     (unsigned long)enc.queue_wait_max, (unsigned long)enc.stack_free);
        }
        if (xQueueReceive(dec_report_q, &dec, 0) == pdTRUE) {
            ESP_LOGI(TAG, "DEC | frames=%lu avg=%llu max=%lu over20=%lu err=%lu drops=%lu plc=%lu real=%lu stack=%lu",
                     (unsigned long)dec.frames,
                     (unsigned long long)(dec.frames ? dec.elapsed_sum / dec.frames : 0),
                     (unsigned long)dec.elapsed_max, (unsigned long)dec.over20,
                     (unsigned long)dec.errors, (unsigned long)dec.drops,
                     (unsigned long)dec.plc,
                     (unsigned long)(dec.frames >= dec.plc ? dec.frames - dec.plc : 0),
                     (unsigned long)dec.stack_free);
            ESP_LOGI(TAG, "RX PCM | peak=%lu rms=%lu samples=%lu",
                     (unsigned long)dec.pcm_peak, (unsigned long)rms(dec.pcm_sq, dec.pcm_samples),
                     (unsigned long)dec.pcm_samples);
        }
        if (xQueueReceive(link_report_q, &link, 0) == pdTRUE) {
            ESP_LOGI(TAG, "RF | tx=%lu fail=%lu rx=%lu delivered=%lu missing=%lu crc=%lu invalid=%lu ignored=%lu",
                     (unsigned long)link.tx, (unsigned long)link.tx_fail,
                     (unsigned long)link.rx, (unsigned long)link.delivered,
                     (unsigned long)link.missing, (unsigned long)link.bad_crc,
                     (unsigned long)link.invalid, (unsigned long)link.ignored);
            ESP_LOGI(TAG, "JITTER | late=%lu dup=%lu full=%lu large=%lu occupancy_max=%lu lead_max=%d resync=%lu (lead=%lu clock=%lu plc=%lu)",
                     (unsigned long)link.late, (unsigned long)link.duplicate,
                     (unsigned long)link.full, (unsigned long)link.too_large,
                     (unsigned long)link.jitter_max, (int)link.lead_max,
                     (unsigned long)link.resync, (unsigned long)link.resync_lead,
                     (unsigned long)link.resync_clock, (unsigned long)link.resync_plc);
            ESP_LOGI(TAG, "LATENCY | encoder_to_radio_avg=%llu max=%lu us, rx_to_jitter_playout_avg=%llu max=%lu us",
                     (unsigned long long)(link.tx_queue_n ? link.tx_queue_sum / link.tx_queue_n : 0),
                     (unsigned long)link.tx_queue_max,
                     (unsigned long long)(link.rx_jitter_n ? link.rx_jitter_sum / link.rx_jitter_n : 0),
                     (unsigned long)link.rx_jitter_max);
            ESP_LOGI(TAG, "RF QUEUE | decode_drop=%lu reconnect=%lu stack=%lu",
                     (unsigned long)link.decode_drops, (unsigned long)link.reconnects,
                     (unsigned long)link.stack_free);
            tr::EspNowTransport::Stats s = {};
            radio.get_stats(s);
            ESP_LOGI(TAG, "ESPNOW | requested=%lu submitfail=%lu macok=%lu macfail=%lu rxqdrop=%lu",
                     (unsigned long)s.tx_requests, (unsigned long)s.tx_submit_failures,
                     (unsigned long)s.tx_mac_success, (unsigned long)s.tx_mac_fail,
                     (unsigned long)s.rx_queue_drops);
        }
        if (xQueueReceive(audio_report_q, &a, 0) == pdTRUE) {
            ESP_LOGI(TAG, "MIC | peak=%lu rms=%lu samples=%lu",
                     (unsigned long)a.mic_peak, (unsigned long)rms(a.mic_sq, a.mic_samples),
                     (unsigned long)a.mic_samples);
            ESP_LOGI(TAG, "AUDIO | blocks=%lu max=%lu over10=%lu under=%lu txreplace=%lu starts=%lu rebuffers=%lu seqgap=%lu",
                     (unsigned long)a.blocks, (unsigned long)a.max_us,
                     (unsigned long)a.over10, (unsigned long)a.underruns,
                     (unsigned long)a.tx_replacements, (unsigned long)a.starts,
                     (unsigned long)a.rebuffers, (unsigned long)a.seq_gaps);
            ESP_LOGI(TAG, "RX LATENCY | radio_receipt_to_dac_queue_avg=%llu max=%lu us samples=%lu",
                     (unsigned long long)(a.receive_to_dac_n ? a.receive_to_dac_sum / a.receive_to_dac_n : 0),
                     (unsigned long)a.receive_to_dac_max, (unsigned long)a.receive_to_dac_n);
            ESP_LOGI(TAG, "AUDIO ERR | rx=%lu tx=%lu short=%lu stack=%lu",
                     (unsigned long)a.rx_errors, (unsigned long)a.tx_errors,
                     (unsigned long)a.short_reads, (unsigned long)a.stack_free);
        }
        ESP_LOGI(TAG, "MEM | free=%u largest=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
}
} // anonymous namespace

namespace d1_6 {
void run() {
    ESP_LOGI(TAG, "Slipstream D1.7 — full duplex, Opus complexity=%d, separate Bluetooth ESP32",
             slipstream::OpusCodec::COMPLEXITY);
    pcm_tx_q = xQueueCreate(1, sizeof(PcmFrame));
    wire_tx_q = xQueueCreate(2, sizeof(WireFrame));
    decode_q = xQueueCreate(4, sizeof(DecodeRequest));
    pcm_rx_q = xQueueCreate(4, sizeof(PcmFrame));
    enc_report_q = xQueueCreate(1, sizeof(CodecMetrics));
    dec_report_q = xQueueCreate(1, sizeof(CodecMetrics));
    link_report_q = xQueueCreate(1, sizeof(LinkMetrics));
    audio_report_q = xQueueCreate(1, sizeof(AudioMetrics));
    if (!pcm_tx_q || !wire_tx_q || !decode_q || !pcm_rx_q ||
        !enc_report_q || !dec_report_q || !link_report_q || !audio_report_q) {
        ESP_LOGE(TAG, "Queue allocation failed"); return;
    }
    if (!radio.init()) { ESP_LOGE(TAG, "ESP-NOW initialization failed"); return; }
    struct Task { TaskFunction_t fn; const char *name; uint32_t stack; UBaseType_t priority; BaseType_t core; };
    // Keep the proven core affinity; RF has priority over encoder on Core 0.
    const Task tasks[] = {
        {encoder_task, "d17_encoder", ENCODER_STACK, 5, 0},
        {decoder_task, "d17_decoder", DECODER_STACK, 4, 1},
        {link_task, "d17_link", LINK_STACK, 6, 0},
        {audio_task, "d17_audio", AUDIO_STACK, 6, 1},
        {monitor_task, "d17_monitor", MONITOR_STACK, 1, 0}
    };
    for (const auto &t : tasks) {
        if (xTaskCreatePinnedToCore(t.fn, t.name, t.stack, nullptr,
                                    t.priority, nullptr, t.core) != pdPASS) {
            ESP_LOGE(TAG, "Task creation failed: %s", t.name);
            return;
        }
    }
    ESP_LOGI(TAG, "D1.7 tasks started (same image on both boards)");
}
}
