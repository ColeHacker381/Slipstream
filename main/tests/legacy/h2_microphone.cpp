#include "h2_microphone.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "audio/audio_io.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"

namespace {

constexpr const char *TAG = "H2";

constexpr size_t FRAMES_PER_BLOCK = 441;

/*
 * Stereo:
 *
 * L R L R L R...
 */
int16_t samples[
    FRAMES_PER_BLOCK * 2
];

}


namespace h2 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H2 - Microphone Test"
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
            "H2 FAIL: WM8960 not detected"
        );

        return;
    }


    ESP_ERROR_CHECK(
        wm8960::configure_microphone_capture()
    );


    ESP_ERROR_CHECK(
        audio::init_input()
    );


    ESP_LOGI(
        TAG,
        "Microphone capture started"
    );

    ESP_LOGI(
        TAG,
        "Speak into the headset microphone..."
    );


    /*
     * Accumulate about 100 ms before printing.
     */
    int blocks = 0;

    int64_t sum = 0;
    uint64_t sum_squared = 0;

    int32_t peak = 0;

    size_t sample_count = 0;


    while (true) {

        size_t bytes_read = 0;

        esp_err_t err = audio::read(
            samples,
            sizeof(samples),
            &bytes_read
        );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "I2S read failed: %s",
                esp_err_to_name(err)
            );

            return;
        }


        size_t stereo_samples =
            bytes_read /
            sizeof(int16_t);

        size_t frames =
            stereo_samples / 2;


        /*
         * Only inspect LEFT ADC for H2.
         */
        for (
            size_t frame = 0;
            frame < frames;
            ++frame
        ) {

            int32_t sample =
                samples[frame * 2];

            sum += sample;

            int64_t sample64 =
                static_cast<int64_t>(sample);

            sum_squared +=
                static_cast<uint64_t>(
                    sample64 * sample64
                );


            int32_t magnitude =
                std::abs(sample);

            if (magnitude > peak) {
                peak = magnitude;
            }


            sample_count++;
        }


        blocks++;


        /*
         * 10 x 10 ms ~= 100 ms.
         */
        if (blocks >= 10) {

            if (sample_count == 0) {
                continue;
            }


            double mean =
                static_cast<double>(sum) /
                static_cast<double>(sample_count);


            double mean_square =
                static_cast<double>(sum_squared) /
                static_cast<double>(sample_count);


            double rms =
                std::sqrt(mean_square);


            double rms_dbfs = -120.0;

            if (rms > 0.0) {

                rms_dbfs =
                    20.0 *
                    std::log10(
                        rms / 32768.0
                    );
            }


            double peak_dbfs = -120.0;

            if (peak > 0) {

                peak_dbfs =
                    20.0 *
                    std::log10(
                        static_cast<double>(peak) /
                        32768.0
                    );
            }


            ESP_LOGI(
                TAG,
                "mean=%7.1f  RMS=%7.1f (%6.1f dBFS)  peak=%5ld (%6.1f dBFS)",
                mean,
                rms,
                rms_dbfs,
                static_cast<long>(peak),
                peak_dbfs
            );


            /*
             * Reset statistics.
             */
            blocks = 0;

            sum = 0;
            sum_squared = 0;

            peak = 0;
            sample_count = 0;
        }
    }
}

}