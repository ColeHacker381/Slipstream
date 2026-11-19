#include "h4_block_processing.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char *TAG = "H4";

constexpr size_t FRAMES_PER_BLOCK = 441;

/*
 * 10 ms at 44.1 kHz.
 *
 * Each frame contains:
 *
 * left sample
 * right sample
 */
int16_t input_buffer[
    FRAMES_PER_BLOCK * 2
];

int16_t output_buffer[
    FRAMES_PER_BLOCK * 2
];


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

}


namespace h4 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H4 - Block Processing"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (!wm8960::probe()) {

        ESP_LOGE(
            TAG,
            "H4 FAIL: WM8960 not detected"
        );

        return;
    }


    ESP_ERROR_CHECK(
        wm8960::configure_full_duplex()
    );


    ESP_ERROR_CHECK(
        audio::init_full_duplex()
    );


    ESP_LOGI(
        TAG,
        "10 ms processing blocks enabled"
    );

    ESP_LOGI(
        TAG,
        "Temporary digital gain: +6 dB"
    );


    uint64_t block_count = 0;

    uint64_t total_process_time_us = 0;

    int64_t max_process_time_us = 0;

    uint64_t total_clipped_samples = 0;

    int16_t report_input_peak = 0;

    int16_t report_output_peak = 0;


    while (true) {

        size_t bytes_read = 0;


        esp_err_t err =
            audio::read(
                input_buffer,
                sizeof(input_buffer),
                &bytes_read
            );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "I2S RX failed: %s",
                esp_err_to_name(err)
            );

            return;
        }


        size_t frames =
            bytes_read /
            (
                sizeof(int16_t) *
                2
            );


        /*
         * Start timing only the DSP block.
         *
         * Do not include I2S read/write time.
         */
        int64_t start_us =
            esp_timer_get_time();


        audio_processor::Stats stats = {};

        audio_processor::process_block(
            input_buffer,
            output_buffer,
            frames,
            stats
        );


        int64_t end_us =
            esp_timer_get_time();


        int64_t process_time_us =
            end_us -
            start_us;


        total_process_time_us +=
            process_time_us;

        if (
            process_time_us >
            max_process_time_us
        ) {
            max_process_time_us =
                process_time_us;
        }


        total_clipped_samples +=
            stats.clipped_samples;


        if (
            std::abs(
                static_cast<int32_t>(
                    stats.input_peak
                )
            )
            >
            std::abs(
                static_cast<int32_t>(
                    report_input_peak
                )
            )
        ) {
            report_input_peak =
                stats.input_peak;
        }


        if (
            std::abs(
                static_cast<int32_t>(
                    stats.output_peak
                )
            )
            >
            std::abs(
                static_cast<int32_t>(
                    report_output_peak
                )
            )
        ) {
            report_output_peak =
                stats.output_peak;
        }


        err =
            audio::write(
                output_buffer,
                frames *
                sizeof(int16_t) *
                2
            );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "I2S TX failed: %s",
                esp_err_to_name(err)
            );

            return;
        }


        block_count++;


        /*
         * At 10 ms per block:
         *
         * 100 blocks ~= 1 second
         */
        if (
            block_count % 100 == 0
        ) {

            uint64_t average_us =
                total_process_time_us /
                100;


            ESP_LOGI(
                TAG,
                "blocks=%llu | avg=%llu us | max=%lld us | in=%5.1f dBFS | out=%5.1f dBFS | clipped=%llu",
                static_cast<unsigned long long>(
                    block_count
                ),
                static_cast<unsigned long long>(
                    average_us
                ),
                static_cast<long long>(
                    max_process_time_us
                ),
                sample_to_dbfs(
                    report_input_peak
                ),
                sample_to_dbfs(
                    report_output_peak
                ),
                static_cast<unsigned long long>(
                    total_clipped_samples
                )
            );


            /*
             * Reset one-second statistics.
             */
            total_process_time_us = 0;

            max_process_time_us = 0;

            total_clipped_samples = 0;

            report_input_peak = 0;

            report_output_peak = 0;
        }
    }
}

}