#include "agc.hpp"

#include <cmath>

namespace {

constexpr float FULL_SCALE =
    32768.0f;

}


namespace dsp {

AutomaticGainControl::AutomaticGainControl()
{
    reset();
}


void AutomaticGainControl::reset()
{
    current_gain_db_ = 0.0f;
}


float AutomaticGainControl::clamp(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum) {
        return minimum;
    }

    if (value > maximum) {
        return maximum;
    }

    return value;
}


float AutomaticGainControl::calculate_rms_dbfs(
    const int16_t *input,
    size_t samples
)
{
    if (
        input == nullptr ||
        samples == 0
    ) {
        return -120.0f;
    }


    double sum = 0.0;


    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        double sample =
            static_cast<double>(
                input[i]
            );


        sum +=
            sample *
            sample;
    }


    double mean_square =
        sum /
        static_cast<double>(
            samples
        );


    if (
        mean_square <
        1.0e-12
    ) {
        return -120.0f;
    }


    double rms =
        std::sqrt(
            mean_square
        );


    return
        static_cast<float>(
            20.0 *
            std::log10(
                rms /
                FULL_SCALE
            )
        );
}


float AutomaticGainControl::db_to_linear(
    float gain_db
)
{
    return
        std::pow(
            10.0f,
            gain_db /
            20.0f
        );
}


int16_t AutomaticGainControl::saturate_to_int16(
    float sample,
    uint32_t &clipped_samples
)
{
    if (
        sample >
        32767.0f
    ) {

        clipped_samples++;

        return 32767;
    }


    if (
        sample <
        -32768.0f
    ) {

        clipped_samples++;

        return -32768;
    }


    return
        static_cast<int16_t>(
            std::lrintf(
                sample
            )
        );
}


AgcResult AutomaticGainControl::process_block(
    const int16_t *input,
    int16_t *output,
    size_t samples,
    bool speech_active
)
{
    AgcResult result = {};


    if (
        input == nullptr ||
        output == nullptr ||
        samples == 0
    ) {

        result.applied_gain_db =
            current_gain_db_;

        return result;
    }


    /*
     * -----------------------------------------
     * 1. Measure input speech level.
     * -----------------------------------------
     */

    result.input_dbfs =
        calculate_rms_dbfs(
            input,
            samples
        );


    /*
     * -----------------------------------------
     * 2. Determine desired gain.
     * -----------------------------------------
     */

    float target_gain_db =
        current_gain_db_;


    if (speech_active) {

        /*
         * Example:
         *
         * input = -34 dBFS
         * target = -24 dBFS
         *
         * required gain = +10 dB
         */

        target_gain_db =
            TARGET_DBFS -
            result.input_dbfs;


        target_gain_db =
            clamp(
                target_gain_db,
                MIN_GAIN_DB,
                MAX_GAIN_DB
            );
    }
    else {

        /*
         * During silence:
         *
         * DO NOT chase the noise floor.
         *
         * Slowly return toward unity gain.
         */
        target_gain_db =
            0.0f;
    }


    result.target_gain_db =
        target_gain_db;


    /*
     * -----------------------------------------
     * 3. Select gain smoothing rate.
     * -----------------------------------------
     */

    float time_constant;


    if (!speech_active) {

        time_constant =
            IDLE_RETURN_SECONDS;
    }

    else if (
        target_gain_db <
        current_gain_db_
    ) {

        /*
         * Need less gain because speaker got
         * louder.
         *
         * Respond quickly.
         */
        time_constant =
            ATTACK_SECONDS;
    }

    else {

        /*
         * Need more gain because speaker got
         * quieter.
         *
         * Respond gradually.
         */
        time_constant =
            RELEASE_SECONDS;
    }


    /*
     * One-pole smoothing coefficient:
     *
     * alpha =
     *
     * 1 - exp(-dt / tau)
     */
    float alpha =
        1.0f -
        std::exp(
            -BLOCK_SECONDS /
            time_constant
        );


    current_gain_db_ +=
        alpha *
        (
            target_gain_db -
            current_gain_db_
        );


    current_gain_db_ =
        clamp(
            current_gain_db_,
            MIN_GAIN_DB,
            MAX_GAIN_DB
        );


    result.applied_gain_db =
        current_gain_db_;


    /*
     * -----------------------------------------
     * 4. Convert dB gain into multiplier.
     * -----------------------------------------
     */

    float linear_gain =
        db_to_linear(
            current_gain_db_
        );


    /*
     * -----------------------------------------
     * 5. Apply gain.
     * -----------------------------------------
     */

    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        float amplified =
            static_cast<float>(
                input[i]
            ) *
            linear_gain;


        output[i] =
            saturate_to_int16(
                amplified,
                result.clipped_samples
            );
    }


    /*
     * -----------------------------------------
     * 6. Measure actual output RMS.
     * -----------------------------------------
     */

    result.output_dbfs =
        calculate_rms_dbfs(
            output,
            samples
        );


    return result;
}

}