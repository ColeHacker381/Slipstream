#include "h1_playback.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"

namespace {

constexpr const char *TAG = "H1";

constexpr int SAMPLE_RATE = 44100;

constexpr float TONE_FREQUENCY = 440.0f;

/*
 * Roughly 7.6% of int16 full scale.
 *
 * We are intentionally starting quiet.
 */
constexpr float AMPLITUDE = 2500.0f;

/*
 * 10 ms of audio at 44.1 kHz.
 */
constexpr size_t FRAMES_PER_BLOCK = 441;

constexpr float TWO_PI =
    6.28318530717958647692f;

int16_t audio_buffer[
    FRAMES_PER_BLOCK * 2
];

}


namespace h1 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );

    ESP_LOGI(
        TAG,
        "Slipstream H1 - Playback Test"
    );

    ESP_LOGI(
        TAG,
        "================================"
    );


    /*
     * H0 first.
     */
    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );

    if (!wm8960::probe()) {
        ESP_LOGE(
            TAG,
            "H1 FAIL: WM8960 not detected"
        );

        return;
    }


    /*
     * Configure codec DAC/headphone path.
     */
    ESP_ERROR_CHECK(
        wm8960::configure_headphone_playback()
    );


    /*
     * Configure ESP32 I2S transmitter.
     */
    ESP_ERROR_CHECK(
        audio::init_output()
    );


    ESP_LOGI(
        TAG,
        "Starting 440 Hz tone"
    );

    ESP_LOGI(
        TAG,
        "Initial headphone gain: -20 dB"
    );


    float phase = 0.0f;

    const float phase_increment =
        TWO_PI *
        TONE_FREQUENCY /
        static_cast<float>(SAMPLE_RATE);


    while (true) {

        for (
            size_t frame = 0;
            frame < FRAMES_PER_BLOCK;
            ++frame
        ) {

            int16_t sample =
                static_cast<int16_t>(
                    std::sin(phase) *
                    AMPLITUDE
                );

            /*
             * Interleaved stereo PCM:
             *
             * L R L R L R ...
             */
            audio_buffer[frame * 2] =
                sample;

            audio_buffer[frame * 2 + 1] =
                sample;


            phase += phase_increment;

            if (phase >= TWO_PI) {
                phase -= TWO_PI;
            }
        }


        esp_err_t err = audio::write(
            audio_buffer,
            sizeof(audio_buffer)
        );

        if (err != ESP_OK) {
            ESP_LOGE(
                TAG,
                "H1 FAIL: I2S write failed"
            );

            return;
        }
    }
}

}