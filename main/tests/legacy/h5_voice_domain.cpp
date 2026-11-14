#include "h5_voice_domain.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char *TAG = "H5";


int16_t input_buffer[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK * 2
];

int16_t output_buffer[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK * 2
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
            static_cast<double>(
                magnitude
            ) /
            32768.0
        );
}

}


namespace h5 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H5 - Voice Domain"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Hardware domain: 48 kHz stereo"
    );

    ESP_LOGI(
        TAG,
        "Voice DSP domain: 16 kHz mono"
    );

    ESP_LOGI(
        TAG,
        "Hardware block: 480 frames / 10 ms"
    );

    ESP_LOGI(
        TAG,
        "Voice block: 160 samples / 10 ms"
    );


    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (!wm8960::probe()) {

        ESP_LOGE(
            TAG,
            "H5 FAIL: WM8960 not detected"
        );

        return;
    }


    ESP_ERROR_CHECK(
        wm8960::configure_full_duplex()
    );


    ESP_ERROR_CHECK(
        audio::init_full_duplex()
    );


    uint64_t block_count = 0;

    uint64_t total_processing_us = 0;

    int64_t max_processing_us = 0;


    int16_t hardware_input_peak = 0;
    int16_t voice_input_peak = 0;
    int16_t voice_output_peak = 0;
    int16_t hardware_output_peak = 0;


    while (true) {

        size_t bytes_read = 0;


        ESP_ERROR_CHECK(
            audio::read(
                input_buffer,
                sizeof(input_buffer),
                &bytes_read
            )
        );


        size_t expected_bytes =
            sizeof(input_buffer);


        if (bytes_read != expected_bytes) {

            ESP_LOGW(
                TAG,
                "Short RX block: %zu / %zu bytes",
                bytes_read,
                expected_bytes
            );

            continue;
        }


        int64_t start_us =
            esp_timer_get_time();


        audio_processor::Stats stats = {};

        audio_processor::process_block(
            input_buffer,
            output_buffer,
            stats
        );


        int64_t process_us =
            esp_timer_get_time() -
            start_us;


        total_processing_us +=
            process_us;


        if (
            process_us >
            max_processing_us
        ) {

            max_processing_us =
                process_us;
        }


        auto update_report_peak =
            [](
                int16_t sample,
                int16_t &peak
            )
            {
                if (
                    std::abs(
                        static_cast<int32_t>(
                            sample
                        )
                    )
                    >
                    std::abs(
                        static_cast<int32_t>(
                            peak
                        )
                    )
                ) {

                    peak = sample;
                }
            };


        update_report_peak(
            stats.hardware_input_peak,
            hardware_input_peak
        );

        update_report_peak(
            stats.voice_input_peak,
            voice_input_peak
        );

        update_report_peak(
            stats.voice_output_peak,
            voice_output_peak
        );

        update_report_peak(
            stats.hardware_output_peak,
            hardware_output_peak
        );


        ESP_ERROR_CHECK(
            audio::write(
                output_buffer,
                sizeof(output_buffer)
            )
        );


        block_count++;


        /*
         * 100 blocks = approximately 1 second.
         */
        if (
            block_count % 100 == 0
        ) {

            uint64_t average_us =
                total_processing_us /
                100;


            ESP_LOGI(
                TAG,
                "blocks=%llu | avg=%llu us | max=%lld us | HW in=%5.1f | voice in=%5.1f | voice out=%5.1f | HW out=%5.1f dBFS",
                static_cast<unsigned long long>(
                    block_count
                ),
                static_cast<unsigned long long>(
                    average_us
                ),
                static_cast<long long>(
                    max_processing_us
                ),
                sample_to_dbfs(
                    hardware_input_peak
                ),
                sample_to_dbfs(
                    voice_input_peak
                ),
                sample_to_dbfs(
                    voice_output_peak
                ),
                sample_to_dbfs(
                    hardware_output_peak
                )
            );


            total_processing_us = 0;
            max_processing_us = 0;

            hardware_input_peak = 0;
            voice_input_peak = 0;
            voice_output_peak = 0;
            hardware_output_peak = 0;
        }
    }
}

}