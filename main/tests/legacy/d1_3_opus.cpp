
#include "d1_3_opus.hpp"

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

constexpr const char *TAG = "D1_3";

constexpr size_t HW_FRAMES =
    audio_processor::HARDWARE_FRAMES_PER_BLOCK;
constexpr size_t VOICE_SAMPLES =
    audio_processor::VOICE_SAMPLES_PER_BLOCK;
constexpr size_t OPUS_SAMPLES =
    slipstream::OpusCodec::FRAME_SAMPLES;

static_assert(HW_FRAMES == 480);
static_assert(VOICE_SAMPLES == 160);
static_assert(OPUS_SAMPLES == VOICE_SAMPLES * 2);

// Preserve the working DSP/RF core arrangement.
constexpr BaseType_t AUDIO_CORE = 1;
constexpr BaseType_t OPUS_CORE = 0;

constexpr uint32_t AUDIO_STACK = 16 * 1024;
constexpr uint32_t OPUS_STACK = 40 * 1024;

constexpr UBaseType_t AUDIO_PRIORITY = 6;
constexpr UBaseType_t OPUS_PRIORITY = 5;

constexpr uint32_t AUDIO_DEADLINE_US = 10000;
constexpr uint32_t OPUS_DEADLINE_US = 20000;

// Two Opus frames = up to 40 ms queued in each direction.
constexpr UBaseType_t QUEUE_DEPTH = 2;

struct VoiceFrame {
    int16_t samples[OPUS_SAMPLES] = {};
};

QueueHandle_t tx_queue = nullptr;
QueueHandle_t rx_queue = nullptr;

slipstream::OpusCodec opus_codec;

// Shared counters are updated atomically.
uint32_t tx_drops = 0;
uint32_t rx_drops = 0;

// Audio buffers remain static.
int16_t hardware_input[HW_FRAMES * 2] = {};
int16_t hardware_output[HW_FRAMES * 2] = {};
int16_t voice_block[VOICE_SAMPLES] = {};
int16_t silent_block[VOICE_SAMPLES] = {};

void log_memory(const char *stage)
{
    ESP_LOGI(TAG,
        "%s | heap=%u | largest=%u",
        stage,
        static_cast<unsigned>(
            heap_caps_get_free_size(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            )
        ),
        static_cast<unsigned>(
            heap_caps_get_largest_free_block(
                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
            )
        )
    );
}

// Never block the audio thread on an occupied queue.
// Discard the oldest frame, preserving the newest audio.
bool enqueue_latest(
    QueueHandle_t queue,
    const VoiceFrame &frame
)
{
    if (xQueueSend(queue, &frame, 0) == pdTRUE) {
        return false;
    }

    VoiceFrame discarded = {};
    xQueueReceive(queue, &discarded, 0);

    return xQueueSend(queue, &frame, 0) == pdTRUE;
}

bool initialize_audio()
{
    esp_err_t err = wm8960::init_i2c();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    if (!wm8960::probe()) {
        ESP_LOGE(TAG, "WM8960 not detected");
        return false;
    }

    err = wm8960::configure_full_duplex();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WM8960 config failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    err = audio::init_full_duplex();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    audio_processor::reset();
    audio_processor::set_noise_suppression_enabled(true);
    audio_processor::set_vad_enabled(true);
    audio_processor::set_agc_enabled(true);
    audio_processor::set_limiter_enabled(true);

    ESP_LOGI(TAG, "WM8960 + 48k I2S + DSP ready");
    return true;
}

void opus_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Opus task on Core %d",
             xPortGetCoreID());

    if (!opus_codec.init()) {
        ESP_LOGE(TAG, "Opus initialization FAILED");
        vTaskDelete(nullptr);
        return;
    }

    opus_codec.reset();
    log_memory("After Opus initialization");

    VoiceFrame input = {};
    VoiceFrame decoded = {};

    uint32_t frames = 0;
    uint64_t total_enc_us = 0;
    uint64_t total_dec_us = 0;
    uint64_t total_bytes = 0;
    uint32_t max_enc_us = 0;
    uint32_t max_dec_us = 0;
    uint32_t deadline_misses = 0;
    uint32_t codec_failures = 0;
    uint32_t output_drops = 0;

    int64_t last_report = esp_timer_get_time();

    while (true) {
        // Block until a complete 20 ms PCM frame arrives.
        if (xQueueReceive(
                tx_queue,
                &input,
                portMAX_DELAY
            ) != pdTRUE) {
            continue;
        }

        slipstream::OpusCodecStats stats = {};

        bool success = opus_codec.process_frame(
            input.samples,
            decoded.samples,
            stats
        );

        if (!success) {
            codec_failures++;
        } else {
            frames++;
            total_enc_us += stats.encode_us;
            total_dec_us += stats.decode_us;
            total_bytes +=
                static_cast<uint32_t>(stats.encoded_bytes);

            max_enc_us = std::max(
                max_enc_us, stats.encode_us
            );
            max_dec_us = std::max(
                max_dec_us, stats.decode_us
            );

            if (stats.encode_us + stats.decode_us >
                OPUS_DEADLINE_US) {
                deadline_misses++;
            }

            if (enqueue_latest(rx_queue, decoded)) {
                output_drops++;
            }
        }

        int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            ESP_LOGI(TAG,
                "OPUS | frames=%u | ENC=%llu/%u us | "
                "DEC=%llu/%u us | avg_bytes=%llu | "
                "over20ms=%u | fail=%u | rxdrop=%u",
                static_cast<unsigned>(frames),
                static_cast<unsigned long long>(
                    frames ? total_enc_us / frames : 0
                ),
                static_cast<unsigned>(max_enc_us),
                static_cast<unsigned long long>(
                    frames ? total_dec_us / frames : 0
                ),
                static_cast<unsigned>(max_dec_us),
                static_cast<unsigned long long>(
                    frames ? total_bytes / frames : 0
                ),
                static_cast<unsigned>(deadline_misses),
                static_cast<unsigned>(codec_failures),
                static_cast<unsigned>(output_drops)
            );

            ESP_LOGI(TAG,
                "OPUS QUEUES | TX=%u RX=%u | "
                "TXdrop=%u | stack_free=%u | heap=%u",
                static_cast<unsigned>(
                    uxQueueMessagesWaiting(tx_queue)
                ),
                static_cast<unsigned>(
                    uxQueueMessagesWaiting(rx_queue)
                ),
                static_cast<unsigned>(
                    __atomic_load_n(&tx_drops, __ATOMIC_RELAXED)
                ),
                static_cast<unsigned>(
                    uxTaskGetStackHighWaterMark(nullptr)
                ),
                static_cast<unsigned>(
                    heap_caps_get_free_size(
                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
                    )
                )
            );

            frames = 0;
            total_enc_us = 0;
            total_dec_us = 0;
            total_bytes = 0;
            max_enc_us = 0;
            max_dec_us = 0;
            deadline_misses = 0;
            codec_failures = 0;
            output_drops = 0;
            last_report = now;
        }
    }
}

void audio_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Audio task on Core %d",
             xPortGetCoreID());

    if (!initialize_audio()) {
        ESP_LOGE(TAG, "Audio startup FAILED");
        vTaskDelete(nullptr);
        return;
    }

    ESP_LOGI(TAG,
        "D1.3 encoded Opus loopback started"
    );
    ESP_LOGI(TAG,
        "Keep quiet for first 500 ms"
    );

    VoiceFrame outgoing = {};
    VoiceFrame incoming = {};

    size_t outgoing_half = 0;
    size_t incoming_half = 0;
    bool have_incoming = false;

    uint32_t blocks = 0;
    uint64_t processing_total_us = 0;
    uint32_t processing_max_us = 0;
    uint32_t processing_misses = 0;
    uint32_t playback_underruns = 0;
    uint32_t read_errors = 0;
    uint32_t write_errors = 0;
    uint32_t short_reads = 0;
    uint32_t speech_blocks = 0;

    int64_t last_report = esp_timer_get_time();

    while (true) {
        size_t bytes_read = 0;

        esp_err_t err = audio::read(
            hardware_input,
            sizeof(hardware_input),
            &bytes_read
        );

        if (err != ESP_OK) {
            read_errors++;
            ESP_LOGE(TAG, "I2S RX: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read != sizeof(hardware_input)) {
            short_reads++;
            continue;
        }

        const int64_t start = esp_timer_get_time();

        audio_processor::Stats dsp_stats = {};

        audio_processor::process_input_block(
            hardware_input,
            voice_block,
            dsp_stats
        );

        if (dsp_stats.speech_active) {
            speech_blocks++;
        }

        // Assemble 20 ms frame from two 10 ms blocks.
        std::memcpy(
            outgoing.samples +
                outgoing_half * VOICE_SAMPLES,
            voice_block,
            sizeof(voice_block)
        );

        outgoing_half++;

        if (outgoing_half == 2) {
            if (enqueue_latest(tx_queue, outgoing)) {
                __atomic_add_fetch(
                    &tx_drops, 1, __ATOMIC_RELAXED
                );
            }
            outgoing_half = 0;
        }

        // Fetch one 20 ms decoded frame, if available.
        if (!have_incoming || incoming_half == 2) {
            if (xQueueReceive(
                    rx_queue,
                    &incoming,
                    0
                ) == pdTRUE) {
                have_incoming = true;
                incoming_half = 0;
            } else {
                have_incoming = false;
                incoming_half = 0;
                playback_underruns++;
            }
        }

        const int16_t *playback = silent_block;

        if (have_incoming) {
            playback = incoming.samples +
                incoming_half * VOICE_SAMPLES;
            incoming_half++;
        }

        audio_processor::render_voice_block(
            playback,
            hardware_output,
            dsp_stats
        );

        const uint32_t processing_us =
            static_cast<uint32_t>(
                esp_timer_get_time() - start
            );

        if (processing_us > AUDIO_DEADLINE_US) {
            processing_misses++;
        }

        processing_max_us = std::max(
            processing_max_us, processing_us
        );
        processing_total_us += processing_us;
        blocks++;

        // DMA-paced playback; never waits for Opus.
        err = audio::write(
            hardware_output,
            sizeof(hardware_output)
        );

        if (err != ESP_OK) {
            write_errors++;
            ESP_LOGE(TAG, "I2S TX: %s",
                     esp_err_to_name(err));
        }

        const int64_t now = esp_timer_get_time();

        if (now - last_report >= 1000000) {
            ESP_LOGI(TAG,
                "AUDIO | blocks=%u | DSP=%llu/%u us | "
                "over10ms=%u | under=%u | "
                "speech=%u%% | short=%u | rxerr=%u txerr=%u",
                static_cast<unsigned>(blocks),
                static_cast<unsigned long long>(
                    blocks ? processing_total_us / blocks : 0
                ),
                static_cast<unsigned>(processing_max_us),
                static_cast<unsigned>(processing_misses),
                static_cast<unsigned>(playback_underruns),
                static_cast<unsigned>(
                    blocks ? speech_blocks * 100 / blocks : 0
                ),
                static_cast<unsigned>(short_reads),
                static_cast<unsigned>(read_errors),
                static_cast<unsigned>(write_errors)
            );

            ESP_LOGI(TAG,
                "AUDIO MEM | stack_free=%u | heap=%u",
                static_cast<unsigned>(
                    uxTaskGetStackHighWaterMark(nullptr)
                ),
                static_cast<unsigned>(
                    heap_caps_get_free_size(
                        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT
                    )
                )
            );

            blocks = 0;
            processing_total_us = 0;
            processing_max_us = 0;
            processing_misses = 0;
            playback_underruns = 0;
            read_errors = 0;
            write_errors = 0;
            short_reads = 0;
            speech_blocks = 0;
            last_report = now;
        }
    }
}

} // namespace

namespace d1_3 {

void run()
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Slipstream D1.3 - Opus Loopback");
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Audio: Core 1, 10 ms blocks");
    ESP_LOGI(TAG, "Opus: Core 0, 20 ms frames");
    ESP_LOGI(TAG, "ESP-NOW: H18 packet test");
    ESP_LOGI(TAG, "Bluetooth disabled");

    tx_queue = xQueueCreate(
        QUEUE_DEPTH, sizeof(VoiceFrame)
    );
    rx_queue = xQueueCreate(
        QUEUE_DEPTH, sizeof(VoiceFrame)
    );

    if (!tx_queue || !rx_queue) {
        ESP_LOGE(TAG, "Queue allocation failed");
        return;
    }

    // Start Opus first. It blocks waiting for PCM.
    if (xTaskCreatePinnedToCore(
            opus_task,
            "d1_3_opus",
            OPUS_STACK,
            nullptr,
            OPUS_PRIORITY,
            nullptr,
            OPUS_CORE
        ) != pdPASS) {
        ESP_LOGE(TAG, "Opus task creation failed");
        return;
    }

    // Keep H18 running as a separate radio test.
    h18::run();

    if (xTaskCreatePinnedToCore(
            audio_task,
            "d1_3_audio",
            AUDIO_STACK,
            nullptr,
            AUDIO_PRIORITY,
            nullptr,
            AUDIO_CORE
        ) != pdPASS) {
        ESP_LOGE(TAG, "Audio task creation failed");
        return;
    }

    log_memory("After task creation");
}

} // namespace d1_3
