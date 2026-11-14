#include "h6_filters.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace {

constexpr const char *TAG = "H6";


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
            static_cast<int32_t>(
                sample
            )
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


void update_peak(
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
}

}


namespace h6 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H6 - Voice Filters"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Hardware: 48 kHz stereo"
    );

    ESP_LOGI(
        TAG,
        "DSP: 16 kHz mono / 160 samples"
    );

    ESP_LOGI(
        TAG,
        "HPF: 100 Hz, 2nd-order Butterworth"
    );

    ESP_LOGI(
        TAG,
        "LPF: 6 kHz, 2nd-order Butterworth"
    );


    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (!wm8960::probe()) {

        ESP_LOGE(
            TAG,
            "H6 FAIL: WM8960 not detected"
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

    int64_t maximum_processing_us = 0;


    int16_t report_voice_input_peak = 0;
    int16_t report_voice_output_peak = 0;


    while (true) {

        size_t bytes_read = 0;


        ESP_ERROR_CHECK(
            audio::read(
                input_buffer,
                sizeof(input_buffer),
                &bytes_read
            )
        );


        if (
            bytes_read !=
            sizeof(input_buffer)
        ) {

            ESP_LOGW(
                TAG,
                "Short RX block: %zu / %zu",
                bytes_read,
                sizeof(input_buffer)
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


        int64_t elapsed_us =
            esp_timer_get_time()
            -
            start_us;


        total_processing_us +=
            elapsed_us;


        if (
            elapsed_us >
            maximum_processing_us
        ) {

            maximum_processing_us =
                elapsed_us;
        }


        update_peak(
            stats.voice_input_peak,
            report_voice_input_peak
        );


        update_peak(
            stats.voice_output_peak,
            report_voice_output_peak
        );


        ESP_ERROR_CHECK(
            audio::write(
                output_buffer,
                sizeof(output_buffer)
            )
        );


        block_count++;


        if (
            block_count % 100 == 0
        ) {

            uint64_t average_us =
                total_processing_us /
                100;


            ESP_LOGI(
                TAG,
                "blocks=%llu | avg=%llu us | max=%lld us | voice in=%5.1f dBFS | filtered=%5.1f dBFS",
                static_cast<unsigned long long>(
                    block_count
                ),
                static_cast<unsigned long long>(
                    average_us
                ),
                static_cast<long long>(
                    maximum_processing_us
                ),
                sample_to_dbfs(
                    report_voice_input_peak
                ),
                sample_to_dbfs(
                    report_voice_output_peak
                )
            );


            total_processing_us = 0;
            maximum_processing_us = 0;

            report_voice_input_peak = 0;
            report_voice_output_peak = 0;
        }
    }
}

}