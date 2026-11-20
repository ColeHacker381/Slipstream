
#include "d1_4_optimization.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/opus_codec.hpp"
#include "codec/wm8960.hpp"
#include "tests/h18_espnow.hpp"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "D1_4B";

// =====================================================
// TEST B CONFIGURATION
// =====================================================

// True for D1.4 Test B.
// Set false for a repeat of the Wi-Fi-free baseline.
constexpr bool ENABLE_ESPNOW_TEST = true;

constexpr BaseType_t AUDIO_CORE = 1;
constexpr BaseType_t OPUS_CORE = 0;
constexpr BaseType_t MONITOR_CORE = 0;

constexpr UBaseType_t AUDIO_PRIORITY = 6;
constexpr UBaseType_t OPUS_PRIORITY = 5;
constexpr UBaseType_t MONITOR_PRIORITY = 1;

constexpr uint32_t AUDIO_STACK = 16 * 1024;
constexpr uint32_t OPUS_STACK = 40 * 1024;
constexpr uint32_t MONITOR_STACK = 4096;

constexpr uint32_t AUDIO_PERIOD_US = 10000;
constexpr uint32_t OPUS_PERIOD_US = 20000;

constexpr size_t HW_FRAMES =
    audio_processor::HARDWARE_FRAMES_PER_BLOCK;

constexpr size_t VOICE_SAMPLES =
    audio_processor::VOICE_SAMPLES_PER_BLOCK;

constexpr size_t OPUS_SAMPLES =
    slipstream::OpusCodec::FRAME_SAMPLES;

static_assert(HW_FRAMES == 480);
static_assert(VOICE_SAMPLES == 160);
static_assert(OPUS_SAMPLES == 320);

// TX: at most one waiting frame, plus the frame
// currently being processed by the Opus task.
constexpr UBaseType_t TX_QUEUE_DEPTH = 1;

// RX: enough buffering to tolerate normal timing
// variations without introducing unbounded latency.
constexpr UBaseType_t RX_QUEUE_DEPTH = 2;

struct VoiceFrame {
    int16_t samples[OPUS_SAMPLES] = {};
    int64_t queued_at_us = 0;
    uint32_t sequence = 0;
};

struct OpusReport {
    uint32_t frames = 0;
    uint32_t failures = 0;
    uint32_t process_over = 0;
    uint32_t response_over = 0;
    uint32_t rx_drops = 0;

    uint64_t encode_total = 0;
    uint64_t decode_total = 0;
    uint64_t process_total = 0;
    uint64_t queue_total = 0;
    uint64_t response_total = 0;

    uint32_t encode_max = 0;
    uint32_t decode_max = 0;
    uint32_t process_max = 0;
    uint32_t queue_max = 0;
    uint32_t response_max = 0;

    uint32_t max_sequence_gap = 0;
    uint32_t last_sequence = 0;
    uint64_t encoded_bytes = 0;

    uint32_t elapsed_us = 0;
    uint32_t stack_free = 0;
};

struct AudioReport {
    uint32_t blocks = 0;
    uint32_t dsp_over = 0;
    uint32_t underruns = 0;
    uint32_t short_reads = 0;
    uint32_t rx_errors = 0;
    uint32_t tx_errors = 0;
    uint32_t tx_replacements = 0;
    uint32_t speech_blocks = 0;

    uint64_t dsp_total = 0;
    uint32_t dsp_max = 0;

    uint32_t elapsed_us = 0;
    uint32_t stack_free = 0;
};

QueueHandle_t tx_queue = nullptr;
QueueHandle_t rx_queue = nullptr;

QueueHandle_t opus_report_queue = nullptr;
QueueHandle_t audio_report_queue = nullptr;

slipstream::OpusCodec codec;

int16_t hw_input[HW_FRAMES * 2] = {};
int16_t hw_output[HW_FRAMES * 2] = {};
int16_t voice_block[VOICE_SAMPLES] = {};
int16_t silence_block[VOICE_SAMPLES] = {};

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

void log_heap(const char *stage)
{
    ESP_LOGI(
        TAG,
        "%s | free=%lu | largest=%lu",
        stage,
        static_cast<unsigned long>(free_heap()),
        static_cast<unsigned long>(largest_heap())
    );
}

// =====================================================
// NONBLOCKING QUEUES
// =====================================================

// Single producer: Core 1 audio task.
// Single consumer: Core 0 Opus task.
//
// Never block the audio task. If the one pending frame
// has not been consumed, replace it with the new one.
//
// Returns true when a frame was replaced.
bool enqueue_tx_latest(const VoiceFrame &frame)
{
    if (xQueueSend(tx_queue, &frame, 0) == pdTRUE) {
        return false;
    }

    // Safe only because TX_QUEUE_DEPTH is exactly 1.
    // xQueueOverwrite preserves the newest frame.
    xQueueOverwrite(tx_queue, &frame);
    return true;
}

// Single producer: Core 0 Opus task.
// Single consumer: Core 1 audio task.
//
// Keep up to two decoded frames.
// On overflow, discard the oldest waiting frame.
bool enqueue_rx_latest(const VoiceFrame &frame)
{
    if (xQueueSend(rx_queue, &frame, 0) == pdTRUE) {
        return false;
    }

    VoiceFrame discarded = {};

    xQueueReceive(rx_queue, &discarded, 0);
    xQueueSend(rx_queue, &frame, 0);

    return true;
}

// =====================================================
// AUDIO INITIALIZATION
// =====================================================

bool initialize_audio()
{
    esp_err_t err = wm8960::init_i2c();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init: %s",
                 esp_err_to_name(err));
        return false;
    }

    if (!wm8960::probe()) {
        ESP_LOGE(TAG, "WM8960 not detected");
        return false;
    }

    err = wm8960::configure_full_duplex();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WM8960 config: %s",
                 esp_err_to_name(err));
        return false;
    }

    err = audio::init_full_duplex();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S init: %s",
                 esp_err_to_name(err));
        return false;
    }

    audio_processor::reset();

    audio_processor::set_noise_suppression_enabled(true);
    audio_processor::set_vad_enabled(true);
    audio_processor::set_agc_enabled(true);
    audio_processor::set_limiter_enabled(true);

    ESP_LOGI(TAG, "WM8960 + I2S + DSP initialized");

    return true;
}

// =====================================================
// CORE 0: OPUS TASK
// =====================================================

void opus_task(void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG, "Opus task started on Core %d",
        xPortGetCoreID()
    );

    if (!codec.init()) {
        ESP_LOGE(TAG, "Opus initialization failed");
        vTaskDelete(nullptr);
        return;
    }

    codec.reset();
    log_heap("Opus ready");

    VoiceFrame input = {};
    VoiceFrame decoded = {};

    OpusReport metrics = {};

    bool sequence_initialized = false;
    uint32_t expected_sequence = 0;

    int64_t last_report = esp_timer_get_time();

    while (true) {

        // Wait efficiently for the next 20 ms frame.
        // No polling or busy-wait.
        if (xQueueReceive(
                tx_queue,
                &input,
                portMAX_DELAY
            ) != pdTRUE) {
            continue;
        }

        const int64_t start_us = esp_timer_get_time();

        slipstream::OpusCodecStats stats = {};

        const bool success = codec.process_frame(
            input.samples,
            decoded.samples,
            stats
        );

        const int64_t end_us = esp_timer_get_time();

        if (!success) {
            metrics.failures++;
        } else {
            const uint32_t process_us =
                static_cast<uint32_t>(
                    end_us - start_us
                );

            const uint32_t queue_us =
                static_cast<uint32_t>(
                    start_us - input.queued_at_us
                );

            const uint32_t response_us =
                static_cast<uint32_t>(
                    end_us - input.queued_at_us
                );

            metrics.frames++;

            metrics.encode_total += stats.encode_us;
            metrics.decode_total += stats.decode_us;
            metrics.process_total += process_us;
            metrics.queue_total += queue_us;
            metrics.response_total += response_us;

            metrics.encoded_bytes +=
                static_cast<uint32_t>(
                    stats.encoded_bytes
                );

            metrics.encode_max = std::max(
                metrics.encode_max,
                stats.encode_us
            );

            metrics.decode_max = std::max(
                metrics.decode_max,
                stats.decode_us
            );

            metrics.process_max = std::max(
                metrics.process_max,
                process_us
            );

            metrics.queue_max = std::max(
                metrics.queue_max,
                queue_us
            );

            metrics.response_max = std::max(
                metrics.response_max,
                response_us
            );

            if (process_us > OPUS_PERIOD_US) {
                metrics.process_over++;
            }

            if (response_us > OPUS_PERIOD_US) {
                metrics.response_over++;
            }

            // Detect skipped microphone frames.
            if (sequence_initialized) {
                const uint32_t gap =
                    input.sequence - expected_sequence;

                // A positive forward gap means skipped
                // frames, normally due to TX replacement.
                if (gap > 0 && gap < 0x80000000u) {
                    metrics.max_sequence_gap = std::max(
                        metrics.max_sequence_gap,
                        gap
                    );
                }
            }

            expected_sequence = input.sequence + 1;
            sequence_initialized = true;
            metrics.last_sequence = input.sequence;

            // Timestamp when decoded PCM is ready.
            decoded.queued_at_us = end_us;
            decoded.sequence = input.sequence;

            if (enqueue_rx_latest(decoded)) {
                metrics.rx_drops++;
            }
        }

        // Low-overhead report publication.
        // Printing happens in the monitor task.
        const int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            metrics.elapsed_us = static_cast<uint32_t>(
                now - last_report
            );

            metrics.stack_free =
                uxTaskGetStackHighWaterMark(nullptr);

            xQueueOverwrite(
                opus_report_queue,
                &metrics
            );

            metrics = {};
            last_report = now;
        }
    }
}

// =====================================================
// CORE 1: REAL-TIME AUDIO TASK
// =====================================================

void audio_task(void *arg)
{
    (void)arg;

    ESP_LOGI(
        TAG, "Audio task started on Core %d",
        xPortGetCoreID()
    );

    if (!initialize_audio()) {
        ESP_LOGE(TAG, "Audio startup failed");
        vTaskDelete(nullptr);
        return;
    }

    log_heap("Audio ready");

    ESP_LOGI(TAG, "Complexity-5 loopback started");

    VoiceFrame outgoing = {};
    VoiceFrame incoming = {};

    size_t outgoing_half = 0;
    size_t incoming_half = 0;

    uint32_t next_sequence = 0;

    bool have_incoming = false;
    bool playback_started = false;

    AudioReport metrics = {};

    int64_t last_report = esp_timer_get_time();

    while (true) {
        size_t bytes_read = 0;

        esp_err_t err = audio::read(
            hw_input,
            sizeof(hw_input),
            &bytes_read
        );

        if (err != ESP_OK) {
            metrics.rx_errors++;
            ESP_LOGE(TAG, "I2S read: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read != sizeof(hw_input)) {
            metrics.short_reads++;
            continue;
        }

        const int64_t start_us = esp_timer_get_time();

        audio_processor::Stats stats = {};

        audio_processor::process_input_block(
            hw_input,
            voice_block,
            stats
        );

        if (stats.speech_active) {
            metrics.speech_blocks++;
        }

        // Combine two 10 ms blocks into one 20 ms
        // Opus frame without heap allocation.
        std::memcpy(
            outgoing.samples +
                outgoing_half * VOICE_SAMPLES,
            voice_block,
            sizeof(voice_block)
        );

        outgoing_half++;

        if (outgoing_half == 2) {
            outgoing.sequence = next_sequence++;
            outgoing.queued_at_us =
                esp_timer_get_time();

            if (enqueue_tx_latest(outgoing)) {
                metrics.tx_replacements++;
            }

            outgoing_half = 0;
        }

        // Playback consumes two 10 ms halves of
        // each decoded Opus frame.
        if (!have_incoming || incoming_half >= 2) {
            if (xQueueReceive(
                    rx_queue,
                    &incoming,
                    0
                ) == pdTRUE) {
                have_incoming = true;
                playback_started = true;
                incoming_half = 0;
            } else {
                have_incoming = false;
                incoming_half = 0;

                if (playback_started) {
                    metrics.underruns++;
                }
            }
        }

        const int16_t *playback = silence_block;

        if (have_incoming) {
            playback = incoming.samples +
                incoming_half * VOICE_SAMPLES;

            incoming_half++;
        }

        audio_processor::render_voice_block(
            playback,
            hw_output,
            stats
        );

        const uint32_t dsp_us =
            static_cast<uint32_t>(
                esp_timer_get_time() - start_us
            );

        metrics.blocks++;
        metrics.dsp_total += dsp_us;

        metrics.dsp_max = std::max(
            metrics.dsp_max, dsp_us
        );

        if (dsp_us > AUDIO_PERIOD_US) {
            metrics.dsp_over++;
        }

        err = audio::write(
            hw_output,
            sizeof(hw_output)
        );

        if (err != ESP_OK) {
            metrics.tx_errors++;
            ESP_LOGE(TAG, "I2S write: %s",
                     esp_err_to_name(err));
        }

        // Publish once per second without logging.
        const int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            metrics.elapsed_us = static_cast<uint32_t>(
                now - last_report
            );

            metrics.stack_free =
                uxTaskGetStackHighWaterMark(nullptr);

            xQueueOverwrite(
                audio_report_queue,
                &metrics
            );

            metrics = {};
            last_report = now;
        }
    }
}

// =====================================================
// LOW-PRIORITY MONITOR TASK — CORE 0
// =====================================================

void monitor_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Monitor task on Core %d",
             xPortGetCoreID());

    OpusReport opus = {};
    AudioReport audio = {};

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Only the monitor task performs periodic
        // diagnostic printing.
        if (xQueueReceive(
                opus_report_queue,
                &opus,
                0
            ) == pdTRUE) {

            const uint32_t n = opus.frames;

            const double rate =
                opus.elapsed_us
                    ? static_cast<double>(n) *
                        1000000.0 / opus.elapsed_us
                    : 0.0;

            ESP_LOGI(TAG,
                "OPUS | n=%lu rate=%.1f/s | "
                "ENC=%llu/%lu us | DEC=%llu/%lu us",
                static_cast<unsigned long>(n),
                rate,
                static_cast<unsigned long long>(
                    n ? opus.encode_total / n : 0
                ),
                static_cast<unsigned long>(
                    opus.encode_max
                ),
                static_cast<unsigned long long>(
                    n ? opus.decode_total / n : 0
                ),
                static_cast<unsigned long>(
                    opus.decode_max
                )
            );

            ESP_LOGI(TAG,
                "TIMING | proc=%llu/%lu | "
                "qwait=%llu/%lu | response=%llu/%lu us",
                static_cast<unsigned long long>(
                    n ? opus.process_total / n : 0
                ),
                static_cast<unsigned long>(
                    opus.process_max
                ),
                static_cast<unsigned long long>(
                    n ? opus.queue_total / n : 0
                ),
                static_cast<unsigned long>(
                    opus.queue_max
                ),
                static_cast<unsigned long long>(
                    n ? opus.response_total / n : 0
                ),
                static_cast<unsigned long>(
                    opus.response_max
                )
            );

            ESP_LOGI(TAG,
                "OPUS ERR | proc>20ms=%lu | "
                "resp>20ms=%lu | fail=%lu | "
                "rxdrop=%lu | seqgap_max=%lu | pkt=%llu B",
                static_cast<unsigned long>(
                    opus.process_over
                ),
                static_cast<unsigned long>(
                    opus.response_over
                ),
                static_cast<unsigned long>(
                    opus.failures
                ),
                static_cast<unsigned long>(
                    opus.rx_drops
                ),
                static_cast<unsigned long>(
                    opus.max_sequence_gap
                ),
                static_cast<unsigned long long>(
                    n ? opus.encoded_bytes / n : 0
                )
            );

            ESP_LOGI(TAG,
                "OPUS STACK | free=%lu | TXq=%u RXq=%u",
                static_cast<unsigned long>(
                    opus.stack_free
                ),
                static_cast<unsigned>(
                    uxQueueMessagesWaiting(tx_queue)
                ),
                static_cast<unsigned>(
                    uxQueueMessagesWaiting(rx_queue)
                )
            );
        }

        if (xQueueReceive(
                audio_report_queue,
                &audio,
                0
            ) == pdTRUE) {

            const uint32_t n = audio.blocks;

            const double rate =
                audio.elapsed_us
                    ? static_cast<double>(n) *
                        1000000.0 / audio.elapsed_us
                    : 0.0;

            ESP_LOGI(TAG,
                "AUDIO | n=%lu rate=%.1f/s | "
                "DSP=%llu/%lu us | over10ms=%lu",
                static_cast<unsigned long>(n),
                rate,
                static_cast<unsigned long long>(
                    n ? audio.dsp_total / n : 0
                ),
                static_cast<unsigned long>(
                    audio.dsp_max
                ),
                static_cast<unsigned long>(
                    audio.dsp_over
                )
            );

            ESP_LOGI(TAG,
                "AUDIO ERR | under=%lu | TXreplace=%lu | "
                "short=%lu | rxerr=%lu txerr=%lu | "
                "speech=%lu%%",
                static_cast<unsigned long>(
                    audio.underruns
                ),
                static_cast<unsigned long>(
                    audio.tx_replacements
                ),
                static_cast<unsigned long>(
                    audio.short_reads
                ),
                static_cast<unsigned long>(
                    audio.rx_errors
                ),
                static_cast<unsigned long>(
                    audio.tx_errors
                ),
                static_cast<unsigned long>(
                    n ? audio.speech_blocks * 100 / n : 0
                )
            );

            ESP_LOGI(TAG,
                "MEM | audio_stack=%lu | free=%lu | "
                "largest=%lu",
                static_cast<unsigned long>(
                    audio.stack_free
                ),
                static_cast<unsigned long>(
                    free_heap()
                ),
                static_cast<unsigned long>(
                    largest_heap()
                )
            );
        }
    }
}

} // namespace

// =====================================================
// APPLICATION ENTRY POINT
// =====================================================

namespace d1_4 {

void run()
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Slipstream D1.4 Test B");
    ESP_LOGI(TAG, "================================");

    ESP_LOGI(TAG,
        "ESP-NOW: %s",
        ENABLE_ESPNOW_TEST ? "ON" : "OFF"
    );

    ESP_LOGI(TAG,
        "Opus complexity: %d",
        slipstream::OpusCodec::COMPLEXITY
    );

    ESP_LOGI(TAG,
        "Core 0: Opus + H18 + low-priority monitor"
    );

    ESP_LOGI(TAG,
        "Core 1: 48k I2S + DSP + playback"
    );

    tx_queue = xQueueCreate(
        TX_QUEUE_DEPTH,
        sizeof(VoiceFrame)
    );

    rx_queue = xQueueCreate(
        RX_QUEUE_DEPTH,
        sizeof(VoiceFrame)
    );

    opus_report_queue = xQueueCreate(
        1,
        sizeof(OpusReport)
    );

    audio_report_queue = xQueueCreate(
        1,
        sizeof(AudioReport)
    );

    if (!tx_queue || !rx_queue ||
        !opus_report_queue || !audio_report_queue) {
        ESP_LOGE(TAG, "Queue allocation failure");
        return;
    }

    // The codec task is started first so its
    // initialization has a chance to finish before
    // audio capture starts.
    if (xTaskCreatePinnedToCore(
            opus_task,
            "d1_4_opus",
            OPUS_STACK,
            nullptr,
            OPUS_PRIORITY,
            nullptr,
            OPUS_CORE
        ) != pdPASS) {
        ESP_LOGE(TAG, "Opus task creation failed");
        return;
    }

    // Temporary RF simulation, not yet voice transport.
    if (ENABLE_ESPNOW_TEST) {
        h18::run();
    }

    if (xTaskCreatePinnedToCore(
            audio_task,
            "d1_4_audio",
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
            "d1_4_monitor",
            MONITOR_STACK,
            nullptr,
            MONITOR_PRIORITY,
            nullptr,
            MONITOR_CORE
        ) != pdPASS) {
        ESP_LOGE(TAG, "Monitor task creation failed");
        return;
    }

    log_heap("D1.4 Test B tasks created");
}

} // namespace d1_4
