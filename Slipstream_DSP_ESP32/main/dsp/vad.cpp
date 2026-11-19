#include "vad.hpp"

#include <cmath>

namespace {

constexpr float INT16_FULL_SCALE =
    32768.0f;

constexpr float INT16_FULL_SCALE_POWER =
    INT16_FULL_SCALE *
    INT16_FULL_SCALE;

}


namespace dsp {

VoiceActivityDetector::VoiceActivityDetector()
{
    reset();
}


void VoiceActivityDetector::reset()
{
    speech_active_ = false;

    noise_frame_count_ = 0;

    attack_counter_ = 0;

    hangover_counter_ = 0;

    noise_power_ = 1.0f;

    initial_noise_sum_ = 0.0f;
}


float VoiceActivityDetector::power_to_dbfs(
    float power
)
{
    if (power <= 1.0e-12f) {
        return -120.0f;
    }


    return
        10.0f *
        std::log10(
            power /
            INT16_FULL_SCALE_POWER
        );
}


float VoiceActivityDetector::dbfs_to_power(
    float dbfs
)
{
    return
        INT16_FULL_SCALE_POWER *
        std::pow(
            10.0f,
            dbfs / 10.0f
        );
}


VadResult VoiceActivityDetector::process_block(
    const int16_t *input,
    size_t samples
)
{
    VadResult result = {};


    if (
        input == nullptr ||
        samples == 0
    ) {
        result.speech_active =
            speech_active_;

        result.noise_estimate_ready =
            noise_estimate_ready();

        return result;
    }


    /*
     * -----------------------------------------
     * 1. Calculate mean-square energy.
     * -----------------------------------------
     */

    double energy_sum = 0.0;


    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        double sample =
            static_cast<double>(
                input[i]
            );


        energy_sum +=
            sample *
            sample;
    }


    float power =
        static_cast<float>(
            energy_sum /
            static_cast<double>(
                samples
            )
        );


    /*
     * Avoid zero / log problems.
     */
    if (power < 1.0f) {
        power = 1.0f;
    }


    result.level_dbfs =
        power_to_dbfs(
            power
        );


    /*
     * -----------------------------------------
     * 2. Initial noise-floor learning.
     *
     * H9 starts with 500 ms of silence,
     * matching the H8 startup procedure.
     * -----------------------------------------
     */

    if (
        noise_frame_count_ <
        NOISE_INIT_FRAMES
    ) {

        initial_noise_sum_ +=
            power;


        noise_frame_count_++;


        noise_power_ =
            initial_noise_sum_ /
            static_cast<float>(
                noise_frame_count_
            );


        result.speech_active = false;

        result.noise_estimate_ready =
            noise_estimate_ready();

        result.noise_dbfs =
            power_to_dbfs(
                noise_power_
            );


        return result;
    }


    result.noise_estimate_ready = true;


    /*
     * -----------------------------------------
     * 3. Build thresholds.
     * -----------------------------------------
     */

    float start_threshold =
        noise_power_ *
        START_POWER_RATIO;


    /*
     * Also enforce a minimum absolute speech
     * threshold.
     */
    float absolute_minimum =
        dbfs_to_power(
            MIN_SPEECH_DBFS
        );


    if (
        start_threshold <
        absolute_minimum
    ) {
        start_threshold =
            absolute_minimum;
    }


    float continue_threshold =
        noise_power_ *
        CONTINUE_POWER_RATIO;


    if (
        continue_threshold >
        start_threshold
    ) {
        continue_threshold =
            start_threshold;
    }


    /*
     * -----------------------------------------
     * 4. SILENCE state.
     * -----------------------------------------
     */

    if (!speech_active_) {

        if (
            power >=
            start_threshold
        ) {

            /*
             * Do not update the noise model while
             * we may be seeing the start of speech.
             */
            attack_counter_++;


            if (
                attack_counter_ >=
                ATTACK_FRAMES
            ) {

                speech_active_ = true;

                result.speech_started = true;

                hangover_counter_ =
                    HANGOVER_FRAMES;

                attack_counter_ = 0;
            }
        }
        else {

            attack_counter_ = 0;


            /*
             * Slowly track the ambient noise floor.
             */
            noise_power_ =
                NOISE_ALPHA *
                noise_power_
                +
                (
                    1.0f -
                    NOISE_ALPHA
                ) *
                power;
        }
    }


    /*
     * -----------------------------------------
     * 5. SPEECH state.
     * -----------------------------------------
     */

    else {

        if (
            power >=
            continue_threshold
        ) {

            /*
             * Speech is still clearly present.
             */
            hangover_counter_ =
                HANGOVER_FRAMES;
        }
        else {

            /*
             * Energy dropped below threshold.
             *
             * Keep VAD active for a short time
             * so pauses between words/syllables
             * do not repeatedly toggle TX.
             */
            if (
                hangover_counter_ > 0
            ) {

                hangover_counter_--;
            }
            else {

                speech_active_ = false;

                result.speech_ended = true;

                attack_counter_ = 0;
            }
        }
    }


    result.speech_active =
        speech_active_;


    result.noise_dbfs =
        power_to_dbfs(
            noise_power_
        );


    return result;
}

}