#include "h8_noise_suppression.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char *TAG = "H8";

int16_t input_buffer[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK * 2
] = {};

int16_t output_buffer[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK * 2
] = {};

double sample_to_dbfs(
    int16_t sample
)
{
    int32_t magnitude =
        std::abs(
            static_cast<int32_t>(sample)
        );

    if (magnitude == 0) {
        return -120.0;
    }

    return
        20.0 *
        std::log10(
            static_cast<double>(magnitude) /
            32768.0
        );
}

void update_peak(
    int16_t sample,
    int16_t &peak
)
{
    if (
        std::abs(
            static_cast<int32_t>(sample)
        )
        >
        std::abs(
            static_cast<int32_t>(peak)
        )
    ) {
        peak = sample;
    }
}

}

namespace h8 {

void run()
{
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Slipstream H8 - Noise Suppression");
    ESP_LOGI(TAG, "================================");
    ESP_LOGI(TAG, "Hardware: 48 kHz stereo");
    ESP_LOGI(TAG, "DSP: 16 kHz mono / 160 samples");
    ESP_LOGI(TAG, "33-tap FIR resampler");
    ESP_LOGI(TAG, "100 Hz HPF + 6 kHz LPF active");
    ESP_LOGI(TAG, "NS: 320 frame / 160 hop / FFT512");
    ESP_LOGI(TAG, "Keep quiet for the first ~500 ms");

    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );

    if (!wm8960::probe()) {
        ESP_LOGE(
            TAG,
            "H8 FAIL: WM8960 not detected"
        );

        return;
    }

    ESP_ERROR_CHECK(
        wm8960::configure_full_duplex()
    );

    ESP_ERROR_CHECK(
        audio::init_full_duplex()
    );

    audio_processor::reset();
    audio_processor::set_noise_suppression_enabled(true);

    uint64_t block_count = 0;
    uint64_t total_processing_us = 0;
    int64_t maximum_processing_us = 0;

    uint64_t total_downsample_us = 0;
    uint64_t total_dsp_us = 0;
    uint64_t total_upsample_us = 0;
    uint64_t total_clipped_samples = 0;

    int16_t report_voice_input_peak = 0;
    int16_t report_voice_output_peak = 0;

    bool ready_reported = false;

    while (true) {
        size_t bytes_read = 0;

        ESP_ERROR_CHECK(
            audio::read(
                input_buffer,
                sizeof(input_buffer),
                &bytes_read
            )
        );

        if (bytes_read != sizeof(input_buffer)) {
            ESP_LOGW(
                TAG,
                "Short RX block: %zu / %zu",
                bytes_read,
                sizeof(input_buffer)
            );

            continue;
        }

        audio_processor::Stats stats = {};

        int64_t start_us =
            esp_timer_get_time();

        audio_processor::process_block(
            input_buffer,
            output_buffer,
            stats
        );

        int64_t elapsed_us =
            esp_timer_get_time() -
            start_us;

        ESP_ERROR_CHECK(
            audio::write(
                output_buffer,
                sizeof(output_buffer)
            )
        );

        block_count++;
        total_processing_us += elapsed_us;
        total_downsample_us += stats.downsample_us;
        total_dsp_us += stats.dsp_us;
        total_upsample_us += stats.upsample_us;
        total_clipped_samples += stats.clipped_samples;

        if (elapsed_us > maximum_processing_us) {
            maximum_processing_us = elapsed_us;
        }

        update_peak(
            stats.voice_input_peak,
            report_voice_input_peak
        );

        update_peak(
            stats.voice_output_peak,
            report_voice_output_peak
        );

        if (
            !ready_reported &&
            audio_processor::noise_estimate_ready()
        ) {
            ESP_LOGI(
                TAG,
                "Noise estimate ready"
            );

            ready_reported = true;
        }

        if (block_count % 100 == 0) {
            ESP_LOGI(
                TAG,
                "blocks=%llu | total=%llu us | max=%lld us | down=%llu us | DSP=%llu us | up=%llu us | in=%5.1f | out=%5.1f dBFS | clipped=%llu",
                static_cast<unsigned long long>(block_count),
                static_cast<unsigned long long>(
                    total_processing_us / 100
                ),
                static_cast<long long>(
                    maximum_processing_us
                ),
                static_cast<unsigned long long>(
                    total_downsample_us / 100
                ),
                static_cast<unsigned long long>(
                    total_dsp_us / 100
                ),
                static_cast<unsigned long long>(
                    total_upsample_us / 100
                ),
                sample_to_dbfs(report_voice_input_peak),
                sample_to_dbfs(report_voice_output_peak),
                static_cast<unsigned long long>(
                    total_clipped_samples
                )
            );

            total_processing_us = 0;
            maximum_processing_us = 0;
            total_downsample_us = 0;
            total_dsp_us = 0;
            total_upsample_us = 0;
            total_clipped_samples = 0;
            report_voice_input_peak = 0;
            report_voice_output_peak = 0;
        }
    }
}

}
