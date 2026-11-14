#include "h3_loopback.hpp"

#include <cstdint>

#include "audio/audio_io.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"

namespace {

constexpr const char *TAG = "H3";

/*
 * 10 ms at 44.1 kHz.
 */
constexpr size_t FRAMES_PER_BLOCK = 441;

/*
 * I2S ADC gives us stereo frames:
 *
 * L R L R ...
 */
int16_t input_buffer[
    FRAMES_PER_BLOCK * 2
];

/*
 * Output is also stereo.
 */
int16_t output_buffer[
    FRAMES_PER_BLOCK * 2
];

}


namespace h3 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H3 - Digital Loopback"
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
            "H3 FAIL: WM8960 not detected"
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
        "Digital mic-to-headphone loopback started"
    );


    while (true) {

        size_t bytes_read = 0;


        esp_err_t err = audio::read(
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


        /*
         * Number of complete stereo frames captured.
         */
        size_t frames =
            bytes_read /
            (sizeof(int16_t) * 2);


        /*
         * Our microphone is on the LEFT ADC.
         *
         * Copy that mono microphone signal into both
         * headphone channels.
         */
        for (
            size_t frame = 0;
            frame < frames;
            ++frame
        ) {

            int16_t mic_sample =
                input_buffer[frame * 2];

            output_buffer[frame * 2] =
                mic_sample;

            output_buffer[frame * 2 + 1] =
                mic_sample;
        }


        err = audio::write(
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
    }
}

}