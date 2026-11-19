#include "h11_limiter.hpp"

#include <cmath>
#include <cstdint>

#include "audio/audio_io.hpp"
#include "audio/audio_processor.hpp"
#include "codec/wm8960.hpp"

#include "dsp/limiter.hpp"

#include "esp_log.h"
#include "esp_timer.h"


namespace {

constexpr const char *TAG =
    "H11";


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


/*
 * ------------------------------------------------
 * H11 synthetic limiter test buffers.
 * ------------------------------------------------
 */

int16_t synthetic_input[
    dsp::LookaheadLimiter::BLOCK_SIZE
] = {};


int16_t synthetic_output[
    dsp::LookaheadLimiter::BLOCK_SIZE
] = {};


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


/*
 * ------------------------------------------------
 * Synthetic limiter verification.
 *
 * Generates:
 *
 *      1 kHz sine burst
 *      16 kHz sample rate
 *      approximately -0.5 dBFS peak
 *
 * Expected:
 *
 *      input  ~= -0.5 dBFS
 *      output <= -3.0 dBFS
 *
 * The test intentionally bypasses:
 *
 *      microphone
 *      filters
 *      noise suppression
 *      VAD
 *      AGC
 *
 * so H11 is tested in isolation.
 * ------------------------------------------------
 */

bool run_synthetic_limiter_test()
{
    ESP_LOGI(
        TAG,
        ""
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "H11 SYNTHETIC LIMITER TEST"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    /*
     * Clear both buffers.
     */
    for (
        size_t i = 0;
        i <
        dsp::LookaheadLimiter::BLOCK_SIZE;
        ++i
    ) {

        synthetic_input[i] = 0;

        synthetic_output[i] = 0;
    }


    /*
     * ------------------------------------------------
     * -0.5 dBFS amplitude.
     *
     * linear =
     *
     * 10 ^ (-0.5 / 20)
     *
     * ~= 0.94406
     * ------------------------------------------------
     */

    constexpr float TEST_PEAK_DBFS =
        -0.5f;


    const float test_amplitude =
        32767.0f *
        std::pow(
            10.0f,
            TEST_PEAK_DBFS /
            20.0f
        );


    /*
     * ------------------------------------------------
     * Generate a 1 kHz burst.
     *
     * At 16 kHz:
     *
     * 16 samples = one 1 kHz cycle.
     *
     * Start sufficiently far into the block so
     * startup zeros do not confuse the result.
     *
     * 64 samples = 4 ms silence
     *
     * 64 samples = 4 ms burst
     *
     * remaining samples = silence
     * ------------------------------------------------
     */

    constexpr size_t BURST_START =
        64;


    constexpr size_t BURST_LENGTH =
        64;


    constexpr float TWO_PI =
        6.28318530717958647692f;


    for (
        size_t i = 0;
        i < BURST_LENGTH;
        ++i
    ) {

        float phase =
            TWO_PI *
            1000.0f *
            static_cast<float>(
                i
            )
            /
            16000.0f;


        float sample =
            test_amplitude *
            std::sin(
                phase
            );


        synthetic_input[
            BURST_START + i
        ] =
            static_cast<int16_t>(
                std::lrintf(
                    sample
                )
            );
    }


    /*
     * Fresh limiter instance so this test has
     * absolutely no state from live processing.
     */

    dsp::LookaheadLimiter limiter;


    limiter.reset();


    int64_t start_us =
        esp_timer_get_time();


    dsp::LimiterResult result =
        limiter.process_block(
            synthetic_input,
            synthetic_output,
            dsp::LookaheadLimiter::
                BLOCK_SIZE
        );


    uint32_t elapsed_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            start_us
        );


    ESP_LOGI(
        TAG,
        "Synthetic source: 1 kHz sine burst"
    );


    ESP_LOGI(
        TAG,
        "Requested input peak: %.1f dBFS",
        static_cast<double>(
            TEST_PEAK_DBFS
        )
    );


    ESP_LOGI(
        TAG,
        "Measured input peak:  %.2f dBFS",
        static_cast<double>(
            result.input_peak_dbfs
        )
    );


    ESP_LOGI(
        TAG,
        "Measured output peak: %.2f dBFS",
        static_cast<double>(
            result.output_peak_dbfs
        )
    );


    ESP_LOGI(
        TAG,
        "Maximum gain reduction: %.2f dB",
        static_cast<double>(
            result.gain_reduction_db
        )
    );


    ESP_LOGI(
        TAG,
        "Threshold hits: %lu",
        static_cast<unsigned long>(
            result.threshold_hits
        )
    );


    ESP_LOGI(
        TAG,
        "Safety clamps: %lu",
        static_cast<unsigned long>(
            result.safety_clamps
        )
    );


    ESP_LOGI(
        TAG,
        "Limiter processing time: %lu us",
        static_cast<unsigned long>(
            elapsed_us
        )
    );


    /*
     * Allow a tiny amount of integer rounding
     * tolerance around the -3 dBFS ceiling.
     */
    constexpr float MAX_ALLOWED_OUTPUT_DBFS =
        -2.90f;


    bool ceiling_pass =
        result.output_peak_dbfs <=
        MAX_ALLOWED_OUTPUT_DBFS;


    bool limiter_engaged =
        result.threshold_hits > 0;


    if (
        ceiling_pass &&
        limiter_engaged
    ) {

        ESP_LOGI(
            TAG,
            "H11 synthetic ceiling test: PASS"
        );
    }
    else {

        ESP_LOGE(
            TAG,
            "H11 synthetic ceiling test: FAIL"
        );
    }


    /*
     * This is deliberately a separate diagnostic.
     *
     * If clamps occur:
     *
     * the ceiling is protected,
     * but the lookahead envelope itself did not
     * reduce the gain quickly enough.
     *
     * We would want to fix that before freezing
     * H11.
     */

    if (
        result.safety_clamps == 0
    ) {

        ESP_LOGI(
            TAG,
            "Lookahead envelope test: PASS "
            "(no safety clamps)"
        );
    }
    else {

        ESP_LOGW(
            TAG,
            "Lookahead envelope test: "
            "NEEDS REVIEW (%lu safety clamps)",
            static_cast<unsigned long>(
                result.safety_clamps
            )
        );
    }


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        ""
    );


    return
        ceiling_pass &&
        limiter_engaged;
}

}


namespace h11 {

void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H11 - Lookahead Limiter"
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
        "H8 noise suppression active"
    );


    ESP_LOGI(
        TAG,
        "H9 VAD active"
    );


    ESP_LOGI(
        TAG,
        "H10 AGC target: -24 dBFS"
    );


    ESP_LOGI(
        TAG,
        "H11 limiter ceiling: -3.0 dBFS"
    );


    ESP_LOGI(
        TAG,
        "H11 lookahead: 1.0 ms / 16 samples"
    );


    ESP_LOGI(
        TAG,
        "H11 attack: 0.5 ms"
    );


    ESP_LOGI(
        TAG,
        "H11 release: 100 ms"
    );


    /*
     * ------------------------------------------------
     * First verify H11 independently.
     * ------------------------------------------------
     */

    bool synthetic_pass =
        run_synthetic_limiter_test();


    if (!synthetic_pass) {

        ESP_LOGW(
            TAG,
            "Synthetic test failed; "
            "continuing live test for diagnostics"
        );
    }


    /*
     * ------------------------------------------------
     * Codec + hardware initialization.
     * ------------------------------------------------
     */

    ESP_ERROR_CHECK(
        wm8960::init_i2c()
    );


    if (
        !wm8960::probe()
    ) {

        ESP_LOGE(
            TAG,
            "H11 FAIL: WM8960 not detected"
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
     * ------------------------------------------------
     * Reset the complete real-time pipeline AFTER
     * the synthetic limiter test.
     *
     * That guarantees the synthetic test has no
     * influence on live limiter state.
     * ------------------------------------------------
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


    audio_processor::
        set_agc_enabled(
            true
        );


    audio_processor::
        set_limiter_enabled(
            true
        );


    ESP_LOGI(
        TAG,
        "Live H11 pipeline starting"
    );


    ESP_LOGI(
        TAG,
        "Keep quiet for first ~500 ms"
    );


    /*
     * ------------------------------------------------
     * Timing state.
     * ------------------------------------------------
     */

    uint64_t block_count = 0;


    uint64_t total_processing_us = 0;

    uint64_t total_downsample_us = 0;

    uint64_t total_dsp_us = 0;

    uint64_t total_vad_us = 0;

    uint64_t total_agc_us = 0;

    uint64_t total_limiter_us = 0;

    uint64_t total_upsample_us = 0;


    int64_t maximum_processing_us = 0;


    uint64_t total_threshold_hits = 0;

    uint64_t total_safety_clamps = 0;


    float latest_agc_gain_db =
        0.0f;


    float latest_agc_output_dbfs =
        -120.0f;


    float latest_limiter_input_dbfs =
        -120.0f;


    float latest_limiter_output_dbfs =
        -120.0f;


    float latest_gain_reduction_db =
        0.0f;


    bool latest_speech_state = false;


    bool noise_ready_reported = false;

    bool vad_ready_reported = false;


    /*
     * ------------------------------------------------
     * Live 10 ms processing loop.
     * ------------------------------------------------
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


        block_count++;


        total_processing_us +=
            elapsed_us;


        total_downsample_us +=
            stats.downsample_us;


        total_dsp_us +=
            stats.dsp_us;


        total_vad_us +=
            stats.vad_us;


        total_agc_us +=
            stats.agc_us;


        total_limiter_us +=
            stats.limiter_us;


        total_upsample_us +=
            stats.upsample_us;


        total_threshold_hits +=
            stats.limiter_threshold_hits;


        total_safety_clamps +=
            stats.limiter_safety_clamps;


        if (
            elapsed_us >
            maximum_processing_us
        ) {

            maximum_processing_us =
                elapsed_us;
        }


        latest_speech_state =
            stats.speech_active;


        latest_agc_gain_db =
            stats.agc_gain_db;


        latest_agc_output_dbfs =
            stats.agc_output_dbfs;


        latest_limiter_input_dbfs =
            stats.limiter_input_peak_dbfs;


        latest_limiter_output_dbfs =
            stats.limiter_output_peak_dbfs;


        latest_gain_reduction_db =
            stats.limiter_gain_reduction_db;


        /*
         * ------------------------------------------------
         * Ready messages.
         * ------------------------------------------------
         */

        if (
            !noise_ready_reported &&
            audio_processor::
                noise_estimate_ready()
        ) {

            ESP_LOGI(
                TAG,
                "H8 noise estimate ready"
            );


            noise_ready_reported = true;
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
         * ------------------------------------------------
         * One-second status report.
         * ------------------------------------------------
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
                "down=%llu | "
                "DSP=%llu | "
                "VAD=%llu | "
                "AGC=%llu | "
                "LIM=%llu | "
                "up=%llu us | "
                "speech=%s | "
                "AGCgain=%+5.1f dB | "
                "AGCout=%5.1f dBFS | "
                "LIMin=%5.1f | "
                "LIMout=%5.1f dBFS | "
                "GR=%5.1f dB | "
                "hits=%llu | "
                "clamps=%llu",

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
                    total_agc_us /
                    100
                ),

                static_cast<
                    unsigned long long
                >(
                    total_limiter_us /
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

                latest_agc_gain_db,

                latest_agc_output_dbfs,

                latest_limiter_input_dbfs,

                latest_limiter_output_dbfs,

                latest_gain_reduction_db,

                static_cast<
                    unsigned long long
                >(
                    total_threshold_hits
                ),

                static_cast<
                    unsigned long long
                >(
                    total_safety_clamps
                )
            );


            total_processing_us = 0;

            maximum_processing_us = 0;


            total_downsample_us = 0;

            total_dsp_us = 0;

            total_vad_us = 0;

            total_agc_us = 0;

            total_limiter_us = 0;

            total_upsample_us = 0;


            total_threshold_hits = 0;

            total_safety_clamps = 0;
        }
    }
}

}