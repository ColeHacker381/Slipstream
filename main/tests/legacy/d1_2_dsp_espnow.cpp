
#include "d1_2_dsp_espnow.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"
#include "tests/h18_espnow.hpp"

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr const char *TAG = "D1_2";

// Hardware: 48 kHz stereo, 10 ms per block.
constexpr size_t FRAMES_PER_BLOCK =
    audio_processor::HARDWARE_FRAMES_PER_BLOCK;

constexpr size_t VOICE_SAMPLES =
    audio_processor::VOICE_SAMPLES_PER_BLOCK;

constexpr size_t CHANNELS = 2;
constexpr uint32_t BLOCK_DEADLINE_US = 10000;

// Keep this on Core 1, away from Wi-Fi on Core 0.
constexpr BaseType_t AUDIO_CORE = 1;
constexpr UBaseType_t AUDIO_PRIORITY = 6;

// Conservative initial stack; measure actual headroom.
constexpr uint32_t AUDIO_STACK_BYTES = 16 * 1024;

// Change to false to test DSP without audible loopback.
constexpr bool ENABLE_LOOPBACK = true;

static_assert(FRAMES_PER_BLOCK == 480);
static_assert(VOICE_SAMPLES == 160);

// Static buffers avoid repeated heap allocation.
int16_t hardware_input[
    FRAMES_PER_BLOCK * CHANNELS
] = {};

int16_t hardware_output[
    FRAMES_PER_BLOCK * CHANNELS
] = {};

int16_t voice_output[
    VOICE_SAMPLES
] = {};

int16_t silent_voice[
    VOICE_SAMPLES
] = {};

struct TestStats {
    uint64_t blocks = 0;
    uint64_t total_process_us = 0;

    uint32_t max_process_us = 0;
    uint32_t deadline_misses = 0;
    uint32_t short_reads = 0;
    uint32_t read_errors = 0;
    uint32_t write_errors = 0;

    int32_t max_input_peak = 0;
    int32_t max_output_peak = 0;

    uint64_t vad_active_blocks = 0;
};

int32_t magnitude(int16_t value)
{
    int32_t sample = static_cast<int32_t>(value);
    return sample < 0 ? -sample : sample;
}

int32_t peak_of(const int16_t *buffer, size_t count)
{
    int32_t peak = 0;

    for (size_t i = 0; i < count; ++i) {
        int32_t value = magnitude(buffer[i]);

        if (value > peak) {
            peak = value;
        }
    }

    return peak;
}

void log_heap(const char *stage)
{
    ESP_LOGI(
        TAG,
        "%s | internal_free=%u | largest=%u | dma_free=%u",
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
        ),
        static_cast<unsigned>(
            heap_caps_get_free_size(MALLOC_CAP_DMA)
        )
    );
}

bool initialize_audio()
{
    ESP_LOGI(TAG, "Initializing WM8960 I2C");

    esp_err_t err = wm8960::init_i2c();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    if (!wm8960::probe()) {
        ESP_LOGE(TAG, "WM8960 not detected");
        return false;
    }

    ESP_LOGI(TAG, "WM8960 detected");

    err = wm8960::configure_full_duplex();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Codec configuration failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    err = audio::init_full_duplex();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed: %s",
                 esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "I2S active: 48 kHz / 16-bit stereo");

    return true;
}

void configure_dsp()
{
    audio_processor::reset();

    audio_processor::set_noise_suppression_enabled(true);
    audio_processor::set_vad_enabled(true);
    audio_processor::set_agc_enabled(true);
    audio_processor::set_limiter_enabled(true);

    ESP_LOGI(TAG, "Noise suppression enabled");
    ESP_LOGI(TAG, "VAD enabled");
    ESP_LOGI(TAG, "AGC enabled");
    ESP_LOGI(TAG, "Limiter enabled");
}

void print_report(
    const TestStats &stats,
    int64_t elapsed_us
)
{
    if (stats.blocks == 0) {
        return;
    }

    const uint64_t average_us =
        stats.total_process_us / stats.blocks;

    const double seconds =
        static_cast<double>(elapsed_us) / 1000000.0;

    const double blocks_per_second =
        seconds > 0.0
            ? static_cast<double>(stats.blocks) / seconds
            : 0.0;

    const double average_cpu_percent =
        static_cast<double>(average_us) / 100.0;

    const double vad_percent =
        100.0 * static_cast<double>(
            stats.vad_active_blocks
        ) / static_cast<double>(stats.blocks);

    ESP_LOGI(TAG,
        "AUDIO | blocks=%llu | rate=%.1f/s | "
        "DSP avg=%llu us | max=%u us | "
        "DSP load=%.1f%%",
        static_cast<unsigned long long>(stats.blocks),
        blocks_per_second,
        static_cast<unsigned long long>(average_us),
        static_cast<unsigned>(stats.max_process_us),
        average_cpu_percent
    );

    ESP_LOGI(TAG,
        "SIGNAL | input_peak=%ld | output_peak=%ld | "
        "VAD=%.1f%%",
        static_cast<long>(stats.max_input_peak),
        static_cast<long>(stats.max_output_peak),
        vad_percent
    );

    ESP_LOGI(TAG,
        "ERRORS | DSP_over_10ms=%u | "
        "short_rx=%u | rx_errors=%u | tx_errors=%u",
        static_cast<unsigned>(stats.deadline_misses),
        static_cast<unsigned>(stats.short_reads),
        static_cast<unsigned>(stats.read_errors),
        static_cast<unsigned>(stats.write_errors)
    );

    ESP_LOGI(TAG,
        "MEMORY | stack_free=%u bytes | "
        "internal_free=%u | largest=%u",
        static_cast<unsigned>(
            uxTaskGetStackHighWaterMark(nullptr)
        ),
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

void audio_task(void *argument)
{
    (void)argument;

    ESP_LOGI(TAG, "Audio task started on Core %d",
             xPortGetCoreID());

    if (!initialize_audio()) {
        ESP_LOGE(TAG, "Audio initialization failed");
        vTaskDelete(nullptr);
        return;
    }

    configure_dsp();

    log_heap("Audio initialized");

    ESP_LOGI(TAG,
        "D1.2 local DSP loopback %s",
        ENABLE_LOOPBACK ? "ENABLED" : "DISABLED"
    );

    ESP_LOGI(TAG,
        "Keep microphone quiet for first 500 ms "
        "while noise estimation initializes"
    );

    TestStats stats = {};
    int64_t last_report_us = esp_timer_get_time();

    bool noise_ready_reported = false;
    bool vad_ready_reported = false;

    while (true) {
        size_t bytes_read = 0;

        // I2S DMA-paced receive, nominally every 10 ms.
        esp_err_t err = audio::read(
            hardware_input,
            sizeof(hardware_input),
            &bytes_read
        );

        if (err != ESP_OK) {
            stats.read_errors++;
            ESP_LOGE(TAG, "I2S RX error: %s",
                     esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (bytes_read != sizeof(hardware_input)) {
            stats.short_reads++;
            ESP_LOGW(TAG, "Short I2S block: %u/%u",
                static_cast<unsigned>(bytes_read),
                static_cast<unsigned>(sizeof(hardware_input))
            );
            continue;
        }

        const int64_t start_us = esp_timer_get_time();

        audio_processor::Stats block_stats = {};

        // 48 kHz stereo -> 16 kHz mono with full DSP.
        audio_processor::process_input_block(
            hardware_input,
            voice_output,
            block_stats
        );

        // Local loopback for D1.2.
        // D1.3 will replace this with Opus-decoded PCM.
        const int16_t *playback =
            ENABLE_LOOPBACK ? voice_output : silent_voice;

        // 16 kHz mono -> 48 kHz stereo.
        audio_processor::render_voice_block(
            playback,
            hardware_output,
            block_stats
        );

        const uint32_t process_us =
            static_cast<uint32_t>(
                esp_timer_get_time() - start_us
            );

        // Record processing time, excluding DMA waits.
        stats.blocks++;
        stats.total_process_us += process_us;

        if (process_us > stats.max_process_us) {
            stats.max_process_us = process_us;
        }

        if (process_us > BLOCK_DEADLINE_US) {
            stats.deadline_misses++;
        }

        const int32_t input_peak = peak_of(
            hardware_input,
            FRAMES_PER_BLOCK * CHANNELS
        );

        const int32_t output_peak = peak_of(
            hardware_output,
            FRAMES_PER_BLOCK * CHANNELS
        );

        if (input_peak > stats.max_input_peak) {
            stats.max_input_peak = input_peak;
        }

        if (output_peak > stats.max_output_peak) {
            stats.max_output_peak = output_peak;
        }

        if (block_stats.speech_active) {
            stats.vad_active_blocks++;
        }

        if (!noise_ready_reported &&
            audio_processor::noise_estimate_ready()) {
            ESP_LOGI(TAG, "Noise estimate READY");
            noise_ready_reported = true;
        }

        if (!vad_ready_reported &&
            block_stats.vad_noise_ready) {
            ESP_LOGI(TAG, "VAD noise estimate READY");
            vad_ready_reported = true;
        }

        err = audio::write(
            hardware_output,
            sizeof(hardware_output)
        );

        if (err != ESP_OK) {
            stats.write_errors++;
            ESP_LOGE(TAG, "I2S TX error: %s",
                     esp_err_to_name(err));
        }

        const int64_t now_us = esp_timer_get_time();

        if (now_us - last_report_us >= 1000000) {
            print_report(stats, now_us - last_report_us);
            stats = {};
            last_report_us = now_us;
        }
    }
}

} // namespace

namespace d1_2 {

void run()
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Slipstream D1.2 - DSP + ESP-NOW");
    ESP_LOGI(TAG, "=================================");

    ESP_LOGI(TAG, "Core 0: H18 ESP-NOW transport");
    ESP_LOGI(TAG, "Core 1: WM8960 + DSP + I2S");
    ESP_LOGI(TAG, "Bluetooth disabled");
    ESP_LOGI(TAG, "Opus not yet enabled");

    // Preserve the working H18 RF test.
    // It creates its own pinned Core 0 task.
    h18::run();

    log_heap("After H18 initialization");

    const BaseType_t result = xTaskCreatePinnedToCore(
        audio_task,
        "d1_2_audio",
        AUDIO_STACK_BYTES,
        nullptr,
        AUDIO_PRIORITY,
        nullptr,
        AUDIO_CORE
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create audio task");
        log_heap("Task creation failure");
        return;
    }

    ESP_LOGI(TAG, "D1.2 audio task created");
    log_heap("After task creation");
}

} // namespace d1_2
