
#include "d1_5_rf_pipeline.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"
#include "transport/slipstream_packet.hpp"
#include "transport/simulated_rf.hpp"
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

constexpr char TAG[] = "D1_5";

// =====================================================
// CONFIGURATION
// =====================================================

constexpr BaseType_t AUDIO_CORE = 1;
constexpr BaseType_t ENCODER_CORE = 0;
constexpr BaseType_t DECODER_CORE = 1;
constexpr BaseType_t LINK_CORE = 0;
constexpr BaseType_t MONITOR_CORE = 0;

constexpr UBaseType_t AUDIO_PRIORITY = 6;
constexpr UBaseType_t ENCODER_PRIORITY = 5;
constexpr UBaseType_t DECODER_PRIORITY = 4;
constexpr UBaseType_t LINK_PRIORITY = 4;
constexpr UBaseType_t MONITOR_PRIORITY = 1;

constexpr uint32_t AUDIO_STACK = 16 * 1024;
constexpr uint32_t ENCODER_STACK = 40 * 1024;
constexpr uint32_t DECODER_STACK = 24 * 1024;
constexpr uint32_t LINK_STACK = 8192;
constexpr uint32_t MONITOR_STACK = 4096;

constexpr size_t HW_FRAMES =
    audio_processor::HARDWARE_FRAMES_PER_BLOCK;

constexpr size_t VOICE_SAMPLES =
    audio_processor::VOICE_SAMPLES_PER_BLOCK;

constexpr size_t FRAME_SAMPLES =
    slipstream::OpusCodec::FRAME_SAMPLES;

static_assert(HW_FRAMES == 480);
static_assert(VOICE_SAMPLES == 160);
static_assert(FRAME_SAMPLES == 320);

constexpr uint32_t AUDIO_DEADLINE_US = 10000;
constexpr uint32_t FRAME_DEADLINE_US = 20000;

// 80 ms simulated RF jitter prefill.
// This is separate from decoded playback prebuffering.
constexpr uint32_t JITTER_PREFILL_TICKS = 2;

// Start playback only after two 20 ms decoded frames
// are available.  Adds roughly 20 ms of startup hold
// relative to starting with the first frame, subject
// to task scheduling.
constexpr UBaseType_t PLAYBACK_PREFILL_FRAMES = 2;

constexpr uint8_t STREAM_ID = 1;
constexpr uint8_t CHANNEL = 0;

// Temporary single-board loopback timestamp mapping.
// The simulator does not carry capture timestamps
// over the actual radio wire protocol.
constexpr size_t TIMESTAMP_SLOTS = 128;

// =====================================================
// FRAMES AND STATISTICS
// =====================================================

struct PcmFrame {
    int16_t samples[FRAME_SAMPLES] = {};
    uint32_t sequence = 0;
    int64_t capture_us = 0;
};

struct WirePacket {
    uint8_t bytes[tr::MAX_PACKET_SIZE] = {};
    size_t length = 0;
    uint16_t sequence = 0;
    int64_t capture_us = 0;
};

struct DecodeRequest {
    bool missing = false;
    uint16_t sequence = 0;
    uint16_t length = 0;
    int64_t capture_us = 0;

    uint8_t payload[
        slipstream::OpusCodec::MAX_PACKET_BYTES
    ] = {};
};

struct TimingReport {
    uint32_t frames = 0;
    uint32_t errors = 0;
    uint32_t drops = 0;
    uint32_t overruns = 0;
    uint32_t plc = 0;

    uint64_t processing_sum = 0;
    uint32_t processing_max = 0;

    uint64_t wait_sum = 0;
    uint32_t wait_max = 0;

    uint32_t stack_free = 0;
};

struct LinkReport {
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t lost = 0;
    uint32_t corrupted = 0;
    uint32_t crc_failures = 0;
    uint32_t late = 0;
    uint32_t full = 0;
    uint32_t missing = 0;
    uint32_t drops = 0;
    uint32_t stack_free = 0;
};

struct AudioReport {
    uint32_t blocks = 0;
    uint32_t overruns = 0;
    uint32_t underruns = 0;
    uint32_t tx_replacements = 0;
    uint32_t short_reads = 0;
    uint32_t read_errors = 0;
    uint32_t write_errors = 0;

    uint32_t playback_start_events = 0;
    uint32_t playback_sequence_gaps = 0;
    uint32_t rx_queue_max = 0;

    uint64_t processing_sum = 0;
    uint32_t processing_max = 0;

    uint64_t latency_sum = 0;
    uint32_t latency_max = 0;
    uint32_t latency_samples = 0;

    uint32_t stack_free = 0;
};

struct CaptureStamp {
    uint16_t sequence = 0;
    int64_t capture_us = 0;
    bool valid = false;
};

// =====================================================
// QUEUES AND TASK-OWNED STATE
// =====================================================

QueueHandle_t pcm_tx_queue = nullptr;
QueueHandle_t wire_tx_queue = nullptr;
QueueHandle_t decode_queue = nullptr;
QueueHandle_t pcm_rx_queue = nullptr;

QueueHandle_t encoder_reports = nullptr;
QueueHandle_t decoder_reports = nullptr;
QueueHandle_t link_reports = nullptr;
QueueHandle_t audio_reports = nullptr;

// Independent Opus state. Never share an instance
// between two tasks.
slipstream::OpusCodec tx_codec;
slipstream::OpusCodec rx_codec;

// Owned exclusively by link_task.
tr::SimulatedRf simulated_rf;
tr::PacketJitterBuffer jitter_buffer;

// Link-task-only timestamp table for local loopback.
CaptureStamp capture_stamps[TIMESTAMP_SLOTS] = {};

// Audio-task-only hardware working buffers.
int16_t hw_input[HW_FRAMES * 2] = {};
int16_t hw_output[HW_FRAMES * 2] = {};
int16_t voice_block[VOICE_SAMPLES] = {};
int16_t silence_block[VOICE_SAMPLES] = {};

// =====================================================
// HELPERS
// =====================================================

uint32_t free_heap()
{
    return static_cast<uint32_t>(
        heap_caps_get_free_size(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        )
    );
}

uint32_t largest_heap()
{
    return static_cast<uint32_t>(
        heap_caps_get_largest_free_block(
            MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
        )
    );
}

template<typename T>
bool send_latest(
    QueueHandle_t queue,
    const T &value
)
{
    if (xQueueSend(queue, &value, 0) == pdTRUE) {
        return false;
    }

    T discarded = {};
    xQueueReceive(queue, &discarded, 0);

    // An overflow is counted even if the consumer
    // removed the oldest element concurrently.
    xQueueSend(queue, &value, 0);
    return true;
}

void add_processing(
    TimingReport &r,
    uint32_t processing_us
)
{
    r.frames++;
    r.processing_sum += processing_us;
    r.processing_max = std::max(
        r.processing_max,
        processing_us
    );

    if (processing_us > FRAME_DEADLINE_US) {
        r.overruns++;
    }
}

void publish_timing(
    QueueHandle_t queue,
    TimingReport &report,
    int64_t &last_report
)
{
    const int64_t now = esp_timer_get_time();

    if (now - last_report >= 1000000) {
        report.stack_free =
            uxTaskGetStackHighWaterMark(nullptr);

        xQueueOverwrite(queue, &report);
        report = {};
        last_report = now;
    }
}

bool initialize_audio()
{
    esp_err_t err = wm8960::init_i2c();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WM8960 I2C init failed");
        return false;
    }

    if (!wm8960::probe()) {
        ESP_LOGE(TAG, "WM8960 not detected");
        return false;
    }

    err = wm8960::configure_full_duplex();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WM8960 config failed");
        return false;
    }

    err = audio::init_full_duplex();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed");
        return false;
    }

    audio_processor::reset();
    audio_processor::set_noise_suppression_enabled(true);
    audio_processor::set_vad_enabled(true);
    audio_processor::set_agc_enabled(true);
    audio_processor::set_limiter_enabled(true);

    ESP_LOGI(TAG, "WM8960 / I2S / DSP ready");
    return true;
}

// =====================================================
// CORE 0: INDEPENDENT TX ENCODER
// =====================================================

void encoder_task(void *)
{
    ESP_LOGI(TAG, "ENCODER on Core %d",
             xPortGetCoreID());

    if (!tx_codec.init()) {
        ESP_LOGE(TAG, "TX Opus init failed");
        vTaskDelete(nullptr);
        return;
    }

    tx_codec.reset();

    PcmFrame pcm = {};
    WirePacket wire = {};
    uint8_t encoded[
        slipstream::OpusCodec::MAX_PACKET_BYTES
    ] = {};

    TimingReport report = {};
    int64_t last_report = esp_timer_get_time();

    while (true) {
        if (xQueueReceive(
                pcm_tx_queue,
                &pcm,
                portMAX_DELAY
            ) != pdTRUE) {
            continue;
        }

        uint32_t encode_us = 0;

        const int bytes = tx_codec.encode_frame(
            pcm.samples,
            encoded,
            sizeof(encoded),
            encode_us
        );

        add_processing(report, encode_us);

        if (bytes <= 0) {
            report.errors++;
        } else {
            tr::PacketHeader header = {};
            header.stream_id = STREAM_ID;
            header.channel = CHANNEL;
            header.sequence =
                static_cast<uint16_t>(pcm.sequence);
            header.timestamp =
                pcm.sequence * FRAME_SAMPLES;

            size_t packet_length = 0;

            if (tr::build_packet(
                    header,
                    encoded,
                    static_cast<size_t>(bytes),
                    wire.bytes,
                    sizeof(wire.bytes),
                    packet_length
                )) {

                wire.length = packet_length;
                wire.sequence = header.sequence;
                wire.capture_us = pcm.capture_us;

                if (send_latest(wire_tx_queue, wire)) {
                    report.drops++;
                }
            } else {
                report.errors++;
            }
        }

        publish_timing(
            encoder_reports,
            report,
            last_report
        );
    }
}

// =====================================================
// CORE 0: RF SIMULATOR AND JITTER BUFFER
// =====================================================

void link_task(void *)
{
    ESP_LOGI(TAG, "RF LINK on Core %d",
             xPortGetCoreID());

    simulated_rf.reset();
    jitter_buffer.reset();

    uint32_t tick = 0;
    uint32_t first_packet_tick = 0;
    uint16_t expected_sequence = 0;

    bool started = false;

    WirePacket outgoing = {};
    DecodeRequest request = {};

    uint8_t arrived[tr::MAX_PACKET_SIZE] = {};
    uint8_t playout[tr::MAX_PACKET_SIZE] = {};

    LinkReport report = {};

    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_report = esp_timer_get_time();

    while (true) {
        // TX side: consume encoded packets.
        while (xQueueReceive(
                wire_tx_queue,
                &outgoing,
                0
            ) == pdTRUE) {

            if (!started) {
                started = true;
                first_packet_tick = tick;
                expected_sequence = outgoing.sequence;
            }

            // Store capture timestamp separately
            // from the actual RF packet bytes.
            CaptureStamp &stamp =
                capture_stamps[
                    outgoing.sequence % TIMESTAMP_SLOTS
                ];

            stamp.sequence = outgoing.sequence;
            stamp.capture_us = outgoing.capture_us;
            stamp.valid = true;

            const tr::SimulatedRfEvent event =
                simulated_rf.send(
                    outgoing.bytes,
                    outgoing.length,
                    outgoing.sequence,
                    tick
                );

            report.sent++;

            if (event == tr::SimulatedRfEvent::DROPPED) {
                report.lost++;
            } else if (
                event == tr::SimulatedRfEvent::CORRUPTED
            ) {
                report.corrupted++;
            } else if (
                event == tr::SimulatedRfEvent::FULL
            ) {
                report.drops++;
            }
        }

        // RX side: collect arrived packets.
        size_t packet_length = 0;

        while (simulated_rf.receive(
                tick,
                arrived,
                sizeof(arrived),
                packet_length
            )) {
            tr::ParsedPacket parsed = {};

            const tr::ParseResult result =
                tr::parse_packet(
                    arrived,
                    packet_length,
                    parsed
                );

            if (result != tr::ParseResult::OK) {
                if (result == tr::ParseResult::BAD_CRC) {
                    report.crc_failures++;
                }
                continue;
            }

            if (parsed.header.stream_id != STREAM_ID ||
                parsed.header.channel != CHANNEL ||
                parsed.payload_length >
                    slipstream::OpusCodec::MAX_PACKET_BYTES) {
                report.drops++;
                continue;
            }

            report.received++;

            const tr::JitterInsertResult result_insert =
                jitter_buffer.insert(
                    arrived,
                    packet_length,
                    parsed.header.sequence
                );

            if (result_insert == tr::JitterInsertResult::LATE) {
                report.late++;
            } else if (
                result_insert != tr::JitterInsertResult::OK
            ) {
                report.full++;
            }
        }

        // One playout deadline every 20 ms.
        // Missing frames become Opus PLC requests.
        if (started &&
            tick - first_packet_tick >=
                JITTER_PREFILL_TICKS) {

            request = {};
            request.sequence = expected_sequence;
            request.missing = true;

            const CaptureStamp &stamp =
                capture_stamps[
                    expected_sequence % TIMESTAMP_SLOTS
                ];

            if (stamp.valid &&
                stamp.sequence == expected_sequence) {
                request.capture_us = stamp.capture_us;
            }

            jitter_buffer.set_expected_sequence(
                expected_sequence
            );

            size_t playout_length = 0;

            if (jitter_buffer.take(
                    expected_sequence,
                    playout,
                    sizeof(playout),
                    playout_length
                )) {

                tr::ParsedPacket parsed = {};

                if (tr::parse_packet(
                        playout,
                        playout_length,
                        parsed
                    ) == tr::ParseResult::OK &&
                    parsed.payload_length <=
                        sizeof(request.payload)) {

                    request.length =
                        static_cast<uint16_t>(
                            parsed.payload_length
                        );

                    std::memcpy(
                        request.payload,
                        parsed.payload,
                        parsed.payload_length
                    );

                    request.missing = false;
                }
            }

            if (request.missing) {
                report.missing++;
            }

            if (send_latest(decode_queue, request)) {
                report.drops++;
            }

            expected_sequence++;
        }

        tick++;

        const int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            report.stack_free =
                uxTaskGetStackHighWaterMark(nullptr);

            xQueueOverwrite(
                link_reports,
                &report
            );

            report = {};
            last_report = now;
        }

        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(20)
        );
    }
}

// =====================================================
// CORE 1: INDEPENDENT RX DECODER
// =====================================================

void decoder_task(void *)
{
    ESP_LOGI(TAG, "DECODER on Core %d",
             xPortGetCoreID());

    if (!rx_codec.init()) {
        ESP_LOGE(TAG, "RX Opus init failed");
        vTaskDelete(nullptr);
        return;
    }

    rx_codec.reset();

    DecodeRequest request = {};
    PcmFrame decoded = {};

    TimingReport report = {};
    int64_t last_report = esp_timer_get_time();

    while (true) {
        if (xQueueReceive(
                decode_queue,
                &request,
                portMAX_DELAY
            ) != pdTRUE) {
            continue;
        }

        const int64_t begin_us =
            esp_timer_get_time();

        uint32_t decode_us = 0;
        int samples = 0;

        if (request.missing) {
            samples = rx_codec.decode_plc(
                decoded.samples,
                decode_us
            );
            report.plc++;
        } else {
            samples = rx_codec.decode_frame(
                request.payload,
                request.length,
                decoded.samples,
                decode_us
            );
        }

        const uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() - begin_us
            );

        add_processing(report, processing_us);

        // Record the actual decoder call duration too.
        // The difference reflects task-side overhead.
        (void)decode_us;

        if (samples != static_cast<int>(FRAME_SAMPLES)) {
            report.errors++;
            std::memset(
                decoded.samples,
                0,
                sizeof(decoded.samples)
            );
        }

        decoded.sequence = request.sequence;
        decoded.capture_us = request.capture_us;

        if (send_latest(pcm_rx_queue, decoded)) {
            report.drops++;
        }

        publish_timing(
            decoder_reports,
            report,
            last_report
        );
    }
}

// =====================================================
// CORE 1: REAL-TIME I2S AND DSP
// =====================================================

void audio_task(void *)
{
    ESP_LOGI(TAG, "AUDIO on Core %d",
             xPortGetCoreID());

    if (!initialize_audio()) {
        ESP_LOGE(TAG, "Audio initialization failed");
        vTaskDelete(nullptr);
        return;
    }

    PcmFrame outgoing = {};
    PcmFrame incoming = {};

    uint32_t next_sequence = 0;

    size_t outgoing_half = 0;
    size_t incoming_half = 0;

    bool playback_started = false;
    bool have_incoming = false;

    bool have_previous_sequence = false;
    uint32_t previous_sequence = 0;

    AudioReport report = {};
    int64_t last_report = esp_timer_get_time();

    ESP_LOGI(
        TAG,
        "RX prefill: %u decoded frames",
        static_cast<unsigned>(PLAYBACK_PREFILL_FRAMES)
    );

    while (true) {
        size_t bytes_read = 0;

        const esp_err_t read_err = audio::read(
            hw_input,
            sizeof(hw_input),
            &bytes_read
        );

        if (read_err != ESP_OK) {
            report.read_errors++;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read != sizeof(hw_input)) {
            report.short_reads++;
            continue;
        }

        const int64_t processing_start =
            esp_timer_get_time();

        audio_processor::Stats stats = {};

        audio_processor::process_input_block(
            hw_input,
            voice_block,
            stats
        );

        // TX PCM frame: two 10 ms voice blocks.
        std::memcpy(
            outgoing.samples +
                outgoing_half * VOICE_SAMPLES,
            voice_block,
            sizeof(voice_block)
        );

        outgoing_half++;

        if (outgoing_half == 2) {
            outgoing.sequence = next_sequence++;

            // Timestamp AFTER the complete frame
            // has been processed, not the first sample
            // capture time.
            outgoing.capture_us =
                esp_timer_get_time();

            if (send_latest(pcm_tx_queue, outgoing)) {
                report.tx_replacements++;
            }

            outgoing_half = 0;
        }

        // Monitor maximum decoded queue occupancy.
        report.rx_queue_max = std::max(
            report.rx_queue_max,
            static_cast<uint32_t>(
                uxQueueMessagesWaiting(pcm_rx_queue)
            )
        );

        // Prevent startup underflow by holding output
        // at silence until two frames are available.
        if (!playback_started &&
            uxQueueMessagesWaiting(pcm_rx_queue) >=
                PLAYBACK_PREFILL_FRAMES) {

            playback_started = true;
            report.playback_start_events++;

            ESP_LOGI(
                TAG,
                "Decoded playback prebuffer ready"
            );
        }

        // Consume exactly one 20 ms decoded frame
        // in two successive 10 ms audio blocks.
        if (playback_started &&
            (!have_incoming || incoming_half == 2)) {

            if (xQueueReceive(
                    pcm_rx_queue,
                    &incoming,
                    0
                ) == pdTRUE) {

                have_incoming = true;
                incoming_half = 0;

                if (have_previous_sequence) {
                    const uint32_t expected =
                        previous_sequence + 1;

                    if (incoming.sequence != expected) {
                        report.playback_sequence_gaps++;
                    }
                }

                previous_sequence = incoming.sequence;
                have_previous_sequence = true;

                // This measures POST-DSP assembled PCM
                // -> beginning of decoded playback.
                // It is not microphone-to-ear latency.
                if (incoming.capture_us > 0) {
                    const int64_t latency_us =
                        esp_timer_get_time() -
                        incoming.capture_us;

                    if (latency_us >= 0) {
                        report.latency_samples++;
                        report.latency_sum +=
                            static_cast<uint64_t>(
                                latency_us
                            );
                        report.latency_max = std::max(
                            report.latency_max,
                            static_cast<uint32_t>(
                                latency_us
                            )
                        );
                    }
                }

            } else {
                have_incoming = false;
                incoming_half = 0;
                report.underruns++;
            }
        }

        const int16_t *playback = silence_block;

        if (playback_started && have_incoming) {
            playback = incoming.samples +
                incoming_half * VOICE_SAMPLES;

            incoming_half++;
        }

        audio_processor::render_voice_block(
            playback,
            hw_output,
            stats
        );

        const uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                processing_start
            );

        report.blocks++;
        report.processing_sum += processing_us;
        report.processing_max = std::max(
            report.processing_max,
            processing_us
        );

        if (processing_us > AUDIO_DEADLINE_US) {
            report.overruns++;
        }

        const esp_err_t write_err = audio::write(
            hw_output,
            sizeof(hw_output)
        );

        if (write_err != ESP_OK) {
            report.write_errors++;
        }

        const int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            report.stack_free =
                uxTaskGetStackHighWaterMark(nullptr);

            xQueueOverwrite(
                audio_reports,
                &report
            );

            report = {};
            last_report = now;
        }
    }
}

// =====================================================
// DIAGNOSTIC MONITOR
// =====================================================

void print_timing(
    const char *name,
    const TimingReport &r
)
{
    ESP_LOGI(
        TAG,
        "%s | frames=%lu | avg=%llu us | "
        "max=%lu us | over20ms=%lu | "
        "errors=%lu drops=%lu plc=%lu | "
        "stack=%lu",
        name,
        static_cast<unsigned long>(r.frames),
        static_cast<unsigned long long>(
            r.frames
                ? r.processing_sum / r.frames
                : 0
        ),
        static_cast<unsigned long>(r.processing_max),
        static_cast<unsigned long>(r.overruns),
        static_cast<unsigned long>(r.errors),
        static_cast<unsigned long>(r.drops),
        static_cast<unsigned long>(r.plc),
        static_cast<unsigned long>(r.stack_free)
    );
}

void monitor_task(void *)
{
    TimingReport enc = {};
    TimingReport dec = {};
    LinkReport link = {};
    AudioReport audio = {};

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (xQueueReceive(
                encoder_reports, &enc, 0
            ) == pdTRUE) {
            print_timing("ENC", enc);
        }

        if (xQueueReceive(
                decoder_reports, &dec, 0
            ) == pdTRUE) {
            print_timing("DEC", dec);
        }

        if (xQueueReceive(
                link_reports, &link, 0
            ) == pdTRUE) {
            ESP_LOGI(
                TAG,
                "RF | sent=%lu recv=%lu | "
                "lost=%lu corrupt=%lu crc=%lu | "
                "late=%lu full=%lu missing=%lu | "
                "drops=%lu stack=%lu",
                static_cast<unsigned long>(link.sent),
                static_cast<unsigned long>(link.received),
                static_cast<unsigned long>(link.lost),
                static_cast<unsigned long>(link.corrupted),
                static_cast<unsigned long>(link.crc_failures),
                static_cast<unsigned long>(link.late),
                static_cast<unsigned long>(link.full),
                static_cast<unsigned long>(link.missing),
                static_cast<unsigned long>(link.drops),
                static_cast<unsigned long>(link.stack_free)
            );
        }

        if (xQueueReceive(
                audio_reports, &audio, 0
            ) == pdTRUE) {

            const uint64_t dsp_average =
                audio.blocks
                    ? audio.processing_sum / audio.blocks
                    : 0;

            const uint64_t latency_average_ms =
                audio.latency_samples
                    ? audio.latency_sum /
                        audio.latency_samples / 1000
                    : 0;

            ESP_LOGI(
                TAG,
                "AUDIO | blocks=%lu | DSP=%llu/%lu us | "
                "over10ms=%lu | under=%lu | "
                "TXreplace=%lu | seqgap=%lu",
                static_cast<unsigned long>(audio.blocks),
                static_cast<unsigned long long>(dsp_average),
                static_cast<unsigned long>(audio.processing_max),
                static_cast<unsigned long>(audio.overruns),
                static_cast<unsigned long>(audio.underruns),
                static_cast<unsigned long>(audio.tx_replacements),
                static_cast<unsigned long>(
                    audio.playback_sequence_gaps
                )
            );

            ESP_LOGI(
                TAG,
                "LATENCY | avg=%llu ms | max=%lu ms | "
                "samples=%lu | RXq_max=%lu | starts=%lu",
                static_cast<unsigned long long>(
                    latency_average_ms
                ),
                static_cast<unsigned long>(
                    audio.latency_max / 1000
                ),
                static_cast<unsigned long>(
                    audio.latency_samples
                ),
                static_cast<unsigned long>(
                    audio.rx_queue_max
                ),
                static_cast<unsigned long>(
                    audio.playback_start_events
                )
            );

            ESP_LOGI(
                TAG,
                "AUDIO ERR | short=%lu rxerr=%lu "
                "txerr=%lu | stack=%lu",
                static_cast<unsigned long>(
                    audio.short_reads
                ),
                static_cast<unsigned long>(
                    audio.read_errors
                ),
                static_cast<unsigned long>(
                    audio.write_errors
                ),
                static_cast<unsigned long>(
                    audio.stack_free
                )
            );
        }

        ESP_LOGI(
            TAG,
            "HEAP | free=%lu | largest=%lu",
            static_cast<unsigned long>(free_heap()),
            static_cast<unsigned long>(largest_heap())
        );
    }
}

} // namespace

// =====================================================
// STARTUP
// =====================================================

namespace d1_5 {

void run()
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Slipstream D1.5 Revised");
    ESP_LOGI(TAG, "================================");

    ESP_LOGI(
        TAG,
        "Opus complexity=%d",
        slipstream::OpusCodec::COMPLEXITY
    );

    ESP_LOGI(TAG, "ESP-NOW disabled");
    ESP_LOGI(TAG, "Simulated RF + jitter + PLC enabled");

    ESP_LOGI(TAG, "Core 0: Encoder + RF + Monitor");
    ESP_LOGI(TAG, "Core 1: Audio DSP + Decoder");

    pcm_tx_queue = xQueueCreate(
        1, sizeof(PcmFrame)
    );

    wire_tx_queue = xQueueCreate(
        4, sizeof(WirePacket)
    );

    decode_queue = xQueueCreate(
        4, sizeof(DecodeRequest)
    );

    pcm_rx_queue = xQueueCreate(
        4, sizeof(PcmFrame)
    );

    encoder_reports = xQueueCreate(
        1, sizeof(TimingReport)
    );

    decoder_reports = xQueueCreate(
        1, sizeof(TimingReport)
    );

    link_reports = xQueueCreate(
        1, sizeof(LinkReport)
    );

    audio_reports = xQueueCreate(
        1, sizeof(AudioReport)
    );

    if (!pcm_tx_queue || !wire_tx_queue ||
        !decode_queue || !pcm_rx_queue ||
        !encoder_reports || !decoder_reports ||
        !link_reports || !audio_reports) {

        ESP_LOGE(TAG, "Queue allocation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(
            encoder_task,
            "d15_encoder",
            ENCODER_STACK,
            nullptr,
            ENCODER_PRIORITY,
            nullptr,
            ENCODER_CORE
        ) != pdPASS) {

        ESP_LOGE(TAG, "Encoder task creation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(
            decoder_task,
            "d15_decoder",
            DECODER_STACK,
            nullptr,
            DECODER_PRIORITY,
            nullptr,
            DECODER_CORE
        ) != pdPASS) {

        ESP_LOGE(TAG, "Decoder task creation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(
            link_task,
            "d15_link",
            LINK_STACK,
            nullptr,
            LINK_PRIORITY,
            nullptr,
            LINK_CORE
        ) != pdPASS) {

        ESP_LOGE(TAG, "Link task creation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(
            audio_task,
            "d15_audio",
            AUDIO_STACK,
            nullptr,
            AUDIO_PRIORITY,
            nullptr,
            AUDIO_CORE
        ) != pdPASS) {

        ESP_LOGE(TAG, "Audio task creation failed");
        return;
    }

    if (xTaskCreatePinnedToCore(
            monitor_task,
            "d15_monitor",
            MONITOR_STACK,
            nullptr,
            MONITOR_PRIORITY,
            nullptr,
            MONITOR_CORE
        ) != pdPASS) {

        ESP_LOGE(TAG, "Monitor task creation failed");
        return;
    }

    ESP_LOGI(TAG, "D1.5 revised tasks started");
    ESP_LOGI(
        TAG,
        "Initial heap=%lu bytes",
        static_cast<unsigned long>(free_heap())
    );
}

} // namespace d1_5
