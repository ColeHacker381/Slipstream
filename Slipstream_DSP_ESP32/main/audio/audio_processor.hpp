#pragma once

#include <cstddef>
#include <cstdint>

namespace audio_processor {

constexpr size_t HARDWARE_FRAMES_PER_BLOCK = 480;

constexpr size_t VOICE_SAMPLES_PER_BLOCK = 160;


struct Stats {

    uint32_t clipped_samples = 0;


    int16_t hardware_input_peak = 0;

    int16_t voice_input_peak = 0;

    int16_t voice_output_peak = 0;

    int16_t hardware_output_peak = 0;


    uint32_t downsample_us = 0;

    uint32_t dsp_us = 0;

    uint32_t upsample_us = 0;


    /*
     * H9 VAD.
     */
    uint32_t vad_us = 0;

    bool speech_active = false;

    bool speech_started = false;

    bool speech_ended = false;

    bool vad_noise_ready = false;

    float vad_level_dbfs = -120.0f;

    float vad_noise_dbfs = -120.0f;


    /*
     * H10 AGC.
     */
    uint32_t agc_us = 0;

    float agc_input_dbfs = -120.0f;

    float agc_output_dbfs = -120.0f;

    float agc_gain_db = 0.0f;

    uint32_t agc_clipped_samples = 0;


    /*
     * H11 limiter.
     */
    uint32_t limiter_us = 0;

    float limiter_input_peak_dbfs =
        -120.0f;

    float limiter_output_peak_dbfs =
        -120.0f;

    float limiter_gain_reduction_db =
        0.0f;

    uint32_t limiter_threshold_hits = 0;

    uint32_t limiter_safety_clamps = 0;


    /*
     * Historical H4 field names.
     */
    int16_t input_peak = 0;

    int16_t output_peak = 0;
};


void reset();


/*
 * H8.
 */
void set_noise_suppression_enabled(
    bool enabled
);

bool noise_estimate_ready();


/*
 * H9.
 */
void set_vad_enabled(
    bool enabled
);

bool vad_enabled();

bool speech_active();


/*
 * H10.
 */
void set_agc_enabled(
    bool enabled
);

bool agc_enabled();

float agc_gain_db();


/*
 * H11.
 */
void set_limiter_enabled(
    bool enabled
);

bool limiter_enabled();

float limiter_gain_db();


/*
 * H12 split pipeline.
 *
 * Input:
 *     480 frames of 48 kHz stereo hardware PCM
 *
 * Output:
 *     160 samples of final 16 kHz mono voice PCM
 *
 * This output is AFTER:
 *
 *     H6 filters
 *     H8 NS
 *     H9 VAD observation
 *     H10 AGC
 *     H11 limiter
 *
 * This is the PCM that should eventually be sent
 * to Opus and then RF.
 */
void process_input_block(
    const int16_t *hardware_input,
    int16_t *voice_output_16k,
    Stats &stats
);


/*
 * Convert one final 10 ms / 160-sample / 16 kHz
 * voice block back into the 48 kHz stereo hardware
 * format.
 *
 * Used by H12 to play decoded Opus audio.
 */
void render_voice_block(
    const int16_t *voice_input_16k,
    int16_t *hardware_output,
    Stats &stats
);


/*
 * H5-H11 compatibility path.
 *
 * Performs both:
 *
 *     process_input_block()
 *     render_voice_block()
 *
 * Existing tests therefore behave exactly as before.
 */
void process_block(
    const int16_t *hardware_input,
    int16_t *hardware_output,
    Stats &stats
);


/*
 * Historical H4 interface.
 */
void process_block(
    const int16_t *input,
    int16_t *output,
    size_t frames,
    Stats &stats
);

}