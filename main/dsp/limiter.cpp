#include "limiter.hpp"

#include <cmath>
#include <cstdint>

namespace {

constexpr float FULL_SCALE =
    32768.0f;


/*
 * A 0.5 ms attack at 16 kHz is 8 samples.
 *
 * Treat the attack time as the time by which the
 * envelope should be essentially settled.
 */
constexpr float ATTACK_SETTLING_ERROR =
    0.001f;

}


namespace dsp {

LookaheadLimiter::LookaheadLimiter()
{
    reset();
}


void LookaheadLimiter::reset()
{
    for (
        size_t i = 0;
        i < LOOKAHEAD_SAMPLES;
        ++i
    ) {
        delay_buffer_[i] = 0;
    }


    write_index_ = 0;

    current_gain_ = 1.0f;


    /*
     * ------------------------------------------------
     * ATTACK
     * ------------------------------------------------
     *
     * Tested value:
     *
     *      0.5 ms
     *
     * At 16 kHz:
     *
     *      0.5 ms = 8 samples
     *
     * Choose the coefficient so that only 0.1%
     * of the original gain error remains after
     * those eight samples.
     * ------------------------------------------------
     */

    const float attack_samples =
        static_cast<float>(
            SAMPLE_RATE
        ) *
        (
            ATTACK_MS /
            1000.0f
        );


    attack_coeff_ =
        std::pow(
            ATTACK_SETTLING_ERROR,
            1.0f /
            attack_samples
        );


    /*
     * ------------------------------------------------
     * RELEASE
     * ------------------------------------------------
     *
     * Tested value:
     *
     *      100 ms
     * ------------------------------------------------
     */

    const float release_seconds =
        RELEASE_MS /
        1000.0f;


    release_coeff_ =
        std::exp(
            -1.0f /
            (
                static_cast<float>(
                    SAMPLE_RATE
                ) *
                release_seconds
            )
        );
}


float LookaheadLimiter::db_to_linear(
    float db
)
{
    return
        std::pow(
            10.0f,
            db /
            20.0f
        );
}


float LookaheadLimiter::linear_to_db(
    float linear
)
{
    if (
        linear <=
        1.0e-12f
    ) {
        return -120.0f;
    }


    return
        20.0f *
        std::log10(
            linear
        );
}


float LookaheadLimiter::sample_peak_dbfs(
    const int16_t *samples,
    size_t count
)
{
    int32_t peak = 0;


    for (
        size_t i = 0;
        i < count;
        ++i
    ) {

        int32_t magnitude =
            static_cast<int32_t>(
                samples[i]
            );


        if (
            magnitude <
            0
        ) {
            magnitude =
                -magnitude;
        }


        if (
            magnitude >
            peak
        ) {
            peak =
                magnitude;
        }
    }


    if (
        peak == 0
    ) {
        return -120.0f;
    }


    return
        20.0f *
        std::log10(
            static_cast<float>(
                peak
            ) /
            FULL_SCALE
        );
}


LimiterResult LookaheadLimiter::process_block(
    const int16_t *input,
    int16_t *output,
    size_t samples
)
{
    LimiterResult result = {};


    if (
        input == nullptr ||
        output == nullptr ||
        samples == 0
    ) {
        return result;
    }


    /*
     * ------------------------------------------------
     * Input diagnostic.
     * ------------------------------------------------
     */

    result.input_peak_dbfs =
        sample_peak_dbfs(
            input,
            samples
        );


    float minimum_gain_seen =
        1.0f;


    /*
     * ------------------------------------------------
     * Main sample loop.
     * ------------------------------------------------
     */

    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        /*
         * ------------------------------------------------
         * 1. Read the sample leaving the 1 ms delay.
         * ------------------------------------------------
         */

        int16_t delayed_sample =
            delay_buffer_[
                write_index_
            ];


        /*
         * ------------------------------------------------
         * 2. Insert newest future sample.
         * ------------------------------------------------
         */

        delay_buffer_[
            write_index_
        ] = input[i];


        write_index_++;


        if (
            write_index_ >=
            LOOKAHEAD_SAMPLES
        ) {
            write_index_ = 0;
        }


        /*
         * ------------------------------------------------
         * 3. Find maximum future peak over the
         *    1 ms lookahead window.
         *
         * Only 16 comparisons per output sample.
         * ------------------------------------------------
         */

        int32_t future_peak = 0;


        for (
            size_t j = 0;
            j < LOOKAHEAD_SAMPLES;
            ++j
        ) {

            int32_t magnitude =
                static_cast<int32_t>(
                    delay_buffer_[j]
                );


            if (
                magnitude <
                0
            ) {
                magnitude =
                    -magnitude;
            }


            if (
                magnitude >
                future_peak
            ) {
                future_peak =
                    magnitude;
            }
        }


        /*
         * ------------------------------------------------
         * 4. Determine lookahead envelope target.
         * ------------------------------------------------
         */

        float target_gain =
            1.0f;


        if (
            future_peak > 0
        ) {

            float normalized_peak =
                static_cast<float>(
                    future_peak
                ) /
                FULL_SCALE;


            if (
                normalized_peak >
                CEILING_LINEAR
            ) {

                target_gain =
                    CEILING_LINEAR /
                    normalized_peak;


                result.threshold_hits++;
            }
        }


        /*
         * ------------------------------------------------
         * 5. Smooth envelope.
         *
         * Gain reduction:
         *      0.5 ms attack
         *
         * Gain recovery:
         *      100 ms release
         * ------------------------------------------------
         */

        if (
            target_gain <
            current_gain_
        ) {

            current_gain_ =
                target_gain
                +
                attack_coeff_ *
                (
                    current_gain_
                    -
                    target_gain
                );
        }
        else {

            current_gain_ =
                target_gain
                +
                release_coeff_ *
                (
                    current_gain_
                    -
                    target_gain
                );
        }


        if (
            current_gain_ >
            1.0f
        ) {
            current_gain_ =
                1.0f;
        }


        if (
            current_gain_ <
            0.0f
        ) {
            current_gain_ =
                0.0f;
        }


        /*
         * ------------------------------------------------
         * 6. Exact brick-wall gain protection.
         * ------------------------------------------------
         *
         * The lookahead envelope above provides the
         * smooth gain trajectory.
         *
         * But for the sample that is actually leaving
         * the delay buffer right now, we know the exact
         * maximum legal gain.
         *
         * Instead of hard-clipping the sample after
         * amplification, cap the GAIN itself.
         *
         * This guarantees the -3 dBFS ceiling while
         * preserving the sample waveform.
         * ------------------------------------------------
         */

        float output_gain =
            current_gain_;


        int32_t delayed_magnitude =
            static_cast<int32_t>(
                delayed_sample
            );


        if (
            delayed_magnitude <
            0
        ) {
            delayed_magnitude =
                -delayed_magnitude;
        }


        if (
            delayed_magnitude > 0
        ) {

            float normalized_delayed =
                static_cast<float>(
                    delayed_magnitude
                ) /
                FULL_SCALE;


            if (
                normalized_delayed >
                CEILING_LINEAR
            ) {

                float exact_required_gain =
                    CEILING_LINEAR /
                    normalized_delayed;


                /*
                 * If the smoothed envelope is still
                 * slightly too high, use the exact
                 * gain necessary for this sample.
                 *
                 * This is still gain reduction, not
                 * waveform clipping.
                 */
                if (
                    exact_required_gain <
                    output_gain
                ) {

                    output_gain =
                        exact_required_gain;
                }
            }
        }


        if (
            output_gain <
            minimum_gain_seen
        ) {
            minimum_gain_seen =
                output_gain;
        }


        /*
         * ------------------------------------------------
         * 7. Apply final gain.
         * ------------------------------------------------
         */

        float limited =
            static_cast<float>(
                delayed_sample
            ) *
            output_gain;


        /*
         * ------------------------------------------------
         * 8. Absolute numerical safety guard.
         *
         * This should now effectively never fire.
         *
         * Keep it only as protection against floating
         * point rounding / unexpected state.
         * ------------------------------------------------
         */

        constexpr float POSITIVE_CEILING =
            CEILING_LINEAR *
            32767.0f;


        constexpr float NEGATIVE_CEILING =
            -CEILING_LINEAR *
            32768.0f;


        if (
            limited >
            POSITIVE_CEILING +
            1.0f
        ) {

            limited =
                POSITIVE_CEILING;

            result.safety_clamps++;
        }


        if (
            limited <
            NEGATIVE_CEILING -
            1.0f
        ) {

            limited =
                NEGATIVE_CEILING;

            result.safety_clamps++;
        }


        output[i] =
            static_cast<int16_t>(
                std::lrintf(
                    limited
                )
            );
    }


    /*
     * ------------------------------------------------
     * Output diagnostics.
     * ------------------------------------------------
     */

    result.output_peak_dbfs =
        sample_peak_dbfs(
            output,
            samples
        );


    result.gain_reduction_db =
        linear_to_db(
            minimum_gain_seen
        );


    return result;
}


float LookaheadLimiter::current_gain_db() const
{
    return
        linear_to_db(
            current_gain_
        );
}

}