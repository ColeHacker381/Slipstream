#include "audio_processor.hpp"

#include <cstdlib>

#include "dsp/agc.hpp"
#include "dsp/limiter.hpp"
#include "dsp/noise_suppressor.hpp"
#include "dsp/resampler.hpp"
#include "dsp/vad.hpp"
#include "dsp/voice_filters.hpp"

#include "esp_timer.h"


namespace {

dsp::VoiceResampler voice_resampler;

dsp::NoiseSuppressor noise_suppressor;

dsp::VoiceActivityDetector voice_activity_detector;

dsp::AutomaticGainControl automatic_gain_control;

dsp::LookaheadLimiter lookahead_limiter;


bool noise_suppression_enabled = false;

bool voice_activity_detection_enabled = false;

bool automatic_gain_control_enabled = false;

bool limiter_enabled_flag = false;


int16_t microphone_48k[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK
] = {};


int16_t voice_input[
    audio_processor::VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t filtered_voice[
    audio_processor::VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t internal_voice_output[
    audio_processor::VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t agc_output[
    audio_processor::VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t limiter_output[
    audio_processor::VOICE_SAMPLES_PER_BLOCK
] = {};


int16_t playback_48k[
    audio_processor::HARDWARE_FRAMES_PER_BLOCK
] = {};


void update_peak(
    int16_t sample,
    int16_t &peak
)
{
    int32_t magnitude =
        std::abs(
            static_cast<int32_t>(
                sample
            )
        );


    int32_t peak_magnitude =
        std::abs(
            static_cast<int32_t>(
                peak
            )
        );


    if (
        magnitude >
        peak_magnitude
    ) {
        peak = sample;
    }
}


int16_t gain_by_two(
    int16_t sample,
    uint32_t &clipped_samples
)
{
    int32_t amplified =
        static_cast<int32_t>(
            sample
        ) *
        2;


    if (
        amplified >
        32767
    ) {

        clipped_samples++;

        return 32767;
    }


    if (
        amplified <
        -32768
    ) {

        clipped_samples++;

        return -32768;
    }


    return
        static_cast<int16_t>(
            amplified
        );
}


void process_voice_dsp(
    const int16_t *input,
    int16_t *output
)
{
    /*
     * H6 filters.
     */
    dsp::process_voice_filters(
        input,
        filtered_voice,
        audio_processor::
            VOICE_SAMPLES_PER_BLOCK
    );


    /*
     * H8 noise suppression.
     */
    if (
        noise_suppression_enabled
    ) {

        noise_suppressor.process_block(
            filtered_voice,
            output
        );


        return;
    }


    /*
     * H7 fallback.
     */
    for (
        size_t i = 0;
        i <
        audio_processor::
            VOICE_SAMPLES_PER_BLOCK;
        ++i
    ) {

        output[i] =
            filtered_voice[i];
    }
}

}


namespace audio_processor {

void reset()
{
    voice_resampler.reset();

    dsp::reset_voice_filters();

    noise_suppressor.reset();

    voice_activity_detector.reset();

    automatic_gain_control.reset();

    lookahead_limiter.reset();


    for (
        size_t i = 0;
        i <
        HARDWARE_FRAMES_PER_BLOCK;
        ++i
    ) {

        microphone_48k[i] = 0;

        playback_48k[i] = 0;
    }


    for (
        size_t i = 0;
        i <
        VOICE_SAMPLES_PER_BLOCK;
        ++i
    ) {

        voice_input[i] = 0;

        filtered_voice[i] = 0;

        internal_voice_output[i] = 0;

        agc_output[i] = 0;

        limiter_output[i] = 0;
    }
}


void set_noise_suppression_enabled(
    bool enabled
)
{
    noise_suppression_enabled =
        enabled;
}


bool noise_estimate_ready()
{
    return
        noise_suppression_enabled &&
        noise_suppressor.
            noise_estimate_ready();
}


void set_vad_enabled(
    bool enabled
)
{
    voice_activity_detection_enabled =
        enabled;


    voice_activity_detector.reset();
}


bool vad_enabled()
{
    return
        voice_activity_detection_enabled;
}


bool speech_active()
{
    if (
        !voice_activity_detection_enabled
    ) {
        return false;
    }


    return
        voice_activity_detector.
            speech_active();
}


void set_agc_enabled(
    bool enabled
)
{
    automatic_gain_control_enabled =
        enabled;


    automatic_gain_control.reset();
}


bool agc_enabled()
{
    return
        automatic_gain_control_enabled;
}


float agc_gain_db()
{
    return
        automatic_gain_control.
            gain_db();
}


void set_limiter_enabled(
    bool enabled
)
{
    limiter_enabled_flag =
        enabled;


    lookahead_limiter.reset();
}


bool limiter_enabled()
{
    return
        limiter_enabled_flag;
}


float limiter_gain_db()
{
    return
        lookahead_limiter.
            current_gain_db();
}


void process_input_block(
    const int16_t *hardware_input,
    int16_t *voice_output_16k,
    Stats &stats
)
{
    stats = {};


    if (
        hardware_input == nullptr ||
        voice_output_16k == nullptr
    ) {
        return;
    }


    /*
     * -----------------------------------------
     * 1. Extract microphone channel.
     * -----------------------------------------
     */

    for (
        size_t frame = 0;
        frame <
        HARDWARE_FRAMES_PER_BLOCK;
        ++frame
    ) {

        int16_t sample =
            hardware_input[
                frame * 2
            ];


        microphone_48k[
            frame
        ] = sample;


        update_peak(
            sample,
            stats.hardware_input_peak
        );
    }


    /*
     * -----------------------------------------
     * 2. 48 kHz -> 16 kHz.
     * -----------------------------------------
     */

    int64_t downsample_start =
        esp_timer_get_time();


    voice_resampler.
        downsample_48k_to_16k(
            microphone_48k,
            voice_input
        );


    stats.downsample_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            downsample_start
        );


    for (
        size_t i = 0;
        i <
        VOICE_SAMPLES_PER_BLOCK;
        ++i
    ) {

        update_peak(
            voice_input[i],
            stats.voice_input_peak
        );
    }


    /*
     * -----------------------------------------
     * 3. DSP.
     * -----------------------------------------
     */

    int64_t dsp_start =
        esp_timer_get_time();


    /*
     * H6 + H8.
     */
    process_voice_dsp(
        voice_input,
        internal_voice_output
    );


    /*
     * -----------------------------------------
     * H9 VAD.
     *
     * VAD observes cleaned pre-AGC audio.
     * -----------------------------------------
     */

    dsp::VadResult vad = {};


    if (
        voice_activity_detection_enabled
    ) {

        int64_t vad_start =
            esp_timer_get_time();


        vad =
            voice_activity_detector.
                process_block(
                    internal_voice_output,
                    VOICE_SAMPLES_PER_BLOCK
                );


        stats.vad_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                vad_start
            );


        stats.speech_active =
            vad.speech_active;


        stats.speech_started =
            vad.speech_started;


        stats.speech_ended =
            vad.speech_ended;


        stats.vad_noise_ready =
            vad.noise_estimate_ready;


        stats.vad_level_dbfs =
            vad.level_dbfs;


        stats.vad_noise_dbfs =
            vad.noise_dbfs;
    }


    /*
     * -----------------------------------------
     * H10 AGC.
     * -----------------------------------------
     */

    if (
        automatic_gain_control_enabled
    ) {

        int64_t agc_start =
            esp_timer_get_time();


        dsp::AgcResult agc =
            automatic_gain_control.
                process_block(
                    internal_voice_output,
                    agc_output,
                    VOICE_SAMPLES_PER_BLOCK,
                    vad.speech_active
                );


        stats.agc_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                agc_start
            );


        stats.agc_input_dbfs =
            agc.input_dbfs;


        stats.agc_output_dbfs =
            agc.output_dbfs;


        stats.agc_gain_db =
            agc.applied_gain_db;


        stats.agc_clipped_samples =
            agc.clipped_samples;


        for (
            size_t i = 0;
            i <
            VOICE_SAMPLES_PER_BLOCK;
            ++i
        ) {

            internal_voice_output[i] =
                agc_output[i];
        }
    }


    /*
     * -----------------------------------------
     * H11 limiter.
     * -----------------------------------------
     */

    if (
        limiter_enabled_flag
    ) {

        int64_t limiter_start =
            esp_timer_get_time();


        dsp::LimiterResult limiter =
            lookahead_limiter.
                process_block(
                    internal_voice_output,
                    limiter_output,
                    VOICE_SAMPLES_PER_BLOCK
                );


        stats.limiter_us =
            static_cast<uint32_t>(
                esp_timer_get_time() -
                limiter_start
            );


        stats.limiter_input_peak_dbfs =
            limiter.input_peak_dbfs;


        stats.limiter_output_peak_dbfs =
            limiter.output_peak_dbfs;


        stats.limiter_gain_reduction_db =
            limiter.gain_reduction_db;


        stats.limiter_threshold_hits =
            limiter.threshold_hits;


        stats.limiter_safety_clamps =
            limiter.safety_clamps;


        for (
            size_t i = 0;
            i <
            VOICE_SAMPLES_PER_BLOCK;
            ++i
        ) {

            internal_voice_output[i] =
                limiter_output[i];
        }
    }


    stats.dsp_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            dsp_start
        );


    /*
     * -----------------------------------------
     * 4. Export final H11 PCM.
     * -----------------------------------------
     */

    for (
        size_t i = 0;
        i <
        VOICE_SAMPLES_PER_BLOCK;
        ++i
    ) {

        voice_output_16k[i] =
            internal_voice_output[i];


        update_peak(
            voice_output_16k[i],
            stats.voice_output_peak
        );
    }
}


void render_voice_block(
    const int16_t *voice_input_16k,
    int16_t *hardware_output,
    Stats &stats
)
{
    if (
        voice_input_16k == nullptr ||
        hardware_output == nullptr
    ) {
        return;
    }


    /*
     * -----------------------------------------
     * 16 kHz -> 48 kHz.
     * -----------------------------------------
     */

    int64_t upsample_start =
        esp_timer_get_time();


    voice_resampler.
        upsample_16k_to_48k(
            voice_input_16k,
            playback_48k
        );


    stats.upsample_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            upsample_start
        );


    /*
     * -----------------------------------------
     * Mono -> stereo hardware output.
     * -----------------------------------------
     */

    for (
        size_t frame = 0;
        frame <
        HARDWARE_FRAMES_PER_BLOCK;
        ++frame
    ) {

        int16_t sample =
            playback_48k[
                frame
            ];


        hardware_output[
            frame * 2
        ] = sample;


        hardware_output[
            frame * 2 + 1
        ] = sample;


        update_peak(
            sample,
            stats.hardware_output_peak
        );


        if (
            sample == 32767 ||
            sample == -32768
        ) {

            stats.clipped_samples++;
        }
    }
}


void process_block(
    const int16_t *hardware_input,
    int16_t *hardware_output,
    Stats &stats
)
{
    int16_t final_voice_block[
        VOICE_SAMPLES_PER_BLOCK
    ] = {};


    process_input_block(
        hardware_input,
        final_voice_block,
        stats
    );


    render_voice_block(
        final_voice_block,
        hardware_output,
        stats
    );
}


void process_block(
    const int16_t *input,
    int16_t *output,
    size_t frames,
    Stats &stats
)
{
    stats = {};


    size_t samples =
        frames *
        2;


    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        update_peak(
            input[i],
            stats.input_peak
        );


        output[i] =
            gain_by_two(
                input[i],
                stats.clipped_samples
            );


        update_peak(
            output[i],
            stats.output_peak
        );
    }
}

}