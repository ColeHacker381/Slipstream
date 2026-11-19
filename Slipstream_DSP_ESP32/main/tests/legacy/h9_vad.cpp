#include "h9_vad.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "esp_log.h"
#include "esp_timer.h"


namespace {

constexpr const char *TAG = "H9";


int16_t input_buffer[
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK *
    2
] = {};


int16_t output_buffer[
    audio_processor::
        HARDWARE_FRAMES_PER_BLOCK *
    2
] = {};


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


    if (
        magnitude == 0
    ) {

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


namespace h9 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H9 - Voice Activity Detection"
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
        "33-tap FIR resampler"
    );


    ESP_LOGI(
        TAG,
        "100 Hz HPF + 6 kHz LPF active"
    );


    ESP_LOGI(
        TAG,
        "H8 noise suppression active"
    );


    ESP_LOGI(
        TAG,
        "H9 adaptive energy VAD active"
    );


    ESP_LOGI(
        TAG,
        "VAD attack: 20 ms"
    );


    ESP_LOGI(
        TAG,
        "VAD hangover: 250 ms"
    );


    ESP_LOGI(
        TAG,
        "Keep quiet for the first ~500 ms"
    );


    /*
     * -----------------------------------------
     * Codec + I2S setup.
     * -----------------------------------------
     */

    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (
        !wm8960::probe()
    ) {

        ESP_LOGE(
            TAG,
            "H9 FAIL: WM8960 not detected"
        );


        return;
    }


    ESP_ERROR_CHECK(
        wm8960::
            configure_full_duplex()
    );


    ESP_ERROR_CHECK(
        audio::
            init_full_duplex()
    );


    /*
     * -----------------------------------------
     * Enable H8 + H9.
     * -----------------------------------------
     */

    audio_processor::reset();


    audio_processor::
        set_noise_suppression_enabled(
            true
        );


    audio_processor::
        set_vad_enabled(
            true
        );


    /*
     * -----------------------------------------
     * Timing/report state.
     * -----------------------------------------
     */

    uint64_t block_count = 0;


    uint64_t total_processing_us = 0;

    int64_t maximum_processing_us = 0;


    uint64_t total_downsample_us = 0;

    uint64_t total_dsp_us = 0;

    uint64_t total_vad_us = 0;

    uint64_t total_upsample_us = 0;


    uint64_t total_clipped_samples = 0;


    int16_t report_voice_input_peak = 0;

    int16_t report_voice_output_peak = 0;


    float latest_vad_level_dbfs =
        -120.0f;


    float latest_vad_noise_dbfs =
        -120.0f;


    bool latest_speech_state = false;


    bool ns_ready_reported = false;

    bool vad_ready_reported = false;


    /*
     * -----------------------------------------
     * Main 10 ms loop.
     * -----------------------------------------
     */

    while (true) {

        size_t bytes_read = 0;


        ESP_ERROR_CHECK(
            audio::read(
                input_buffer,
                sizeof(
                    input_buffer
                ),
                &bytes_read
            )
        );


        if (
            bytes_read !=
            sizeof(
                input_buffer
            )
        ) {

            ESP_LOGW(
                TAG,
                "Short RX block: %zu / %zu",
                bytes_read,
                sizeof(
                    input_buffer
                )
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
                sizeof(
                    output_buffer
                )
            )
        );


        /*
         * -----------------------------------------
         * Accumulate timing.
         * -----------------------------------------
         */

        block_count++;


        total_processing_us +=
            elapsed_us;


        total_downsample_us +=
            stats.downsample_us;


        total_dsp_us +=
            stats.dsp_us;


        total_vad_us +=
            stats.vad_us;


        total_upsample_us +=
            stats.upsample_us;


        total_clipped_samples +=
            stats.clipped_samples;


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


        latest_vad_level_dbfs =
            stats.vad_level_dbfs;


        latest_vad_noise_dbfs =
            stats.vad_noise_dbfs;


        latest_speech_state =
            stats.speech_active;


        /*
         * -----------------------------------------
         * Startup-ready notifications.
         * -----------------------------------------
         */

        if (
            !ns_ready_reported &&
            audio_processor::
                noise_estimate_ready()
        ) {

            ESP_LOGI(
                TAG,
                "H8 noise estimate ready"
            );


            ns_ready_reported = true;
        }


        if (
            !vad_ready_reported &&
            stats.vad_noise_ready
        ) {

            ESP_LOGI(
                TAG,
                "H9 VAD noise estimate ready"
            );


            vad_ready_reported = true;
        }


        /*
         * -----------------------------------------
         * Speech state transitions.
         * -----------------------------------------
         */

        if (
            stats.speech_started
        ) {

            ESP_LOGI(
                TAG,
                "SPEECH START | level=%5.1f dBFS | noise=%5.1f dBFS",
                stats.vad_level_dbfs,
                stats.vad_noise_dbfs
            );
        }


        if (
            stats.speech_ended
        ) {

            ESP_LOGI(
                TAG,
                "SPEECH END   | level=%5.1f dBFS | noise=%5.1f dBFS",
                stats.vad_level_dbfs,
                stats.vad_noise_dbfs
            );
        }


        /*
         * -----------------------------------------
         * One-second summary.
         * -----------------------------------------
         */

        if (
            block_count %
            100 ==
            0
        ) {

            ESP_LOGI(
                TAG,

                "blocks=%llu | "
                "total=%llu us | "
                "max=%lld us | "
                "down=%llu us | "
                "DSP=%llu us | "
                "VAD=%llu us | "
                "up=%llu us | "
                "speech=%s | "
                "level=%5.1f | "
                "noise=%5.1f dBFS | "
                "in=%5.1f | "
                "out=%5.1f dBFS | "
                "clipped=%llu",

                static_cast<
                    unsigned long long
                >(
                    block_count
                ),

                static_cast<
                    unsigned long long
                >(
                    total_processing_us /
                    100
                ),

                static_cast<
                    long long
                >(
                    maximum_processing_us
                ),

                static_cast<
                    unsigned long long
                >(
                    total_downsample_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_dsp_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_vad_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_upsample_us /
                    100
                ),

                latest_speech_state
                    ? "YES"
                    : "NO",

                latest_vad_level_dbfs,

                latest_vad_noise_dbfs,

                sample_to_dbfs(
                    report_voice_input_peak
                ),

                sample_to_dbfs(
                    report_voice_output_peak
                ),

                static_cast<
                    unsigned long long
                >(
                    total_clipped_samples
                )
            );


            total_processing_us = 0;

            maximum_processing_us = 0;


            total_downsample_us = 0;

            total_dsp_us = 0;

            total_vad_us = 0;

            total_upsample_us = 0;


            total_clipped_samples = 0;


            report_voice_input_peak = 0;

            report_voice_output_peak = 0;
        }
    }
}

}