#include "rx_output_stage.hpp"

#include <algorithm>
#include <cmath>


namespace {

constexpr float FULL_SCALE =
    32768.0f;


constexpr float MIN_DB =
    -120.0f;


constexpr float EPSILON =
    0.000001f;

}


namespace audio {


RxOutputStage::RxOutputStage()
{
    ceiling_linear_ =
        db_to_linear(
            LIMITER_CEILING_DBFS
        );


    set_master_gain_db(
        MASTER_GAIN_DB
    );


    /*
     * Exponential release coefficient per stereo
     * sample frame.
     */
    const float release_seconds =
        static_cast<float>(
            LIMITER_RELEASE_MS
        )
        /
        1000.0f;


    release_coeff_ =
        std::exp(
            -1.0f /
            (
                static_cast<float>(
                    SAMPLE_RATE
                )
                *
                release_seconds
            )
        );


    reset();
}


void RxOutputStage::reset()
{
    limiter_gain_ =
        1.0f;
}


float RxOutputStage::db_to_linear(
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


float RxOutputStage::linear_to_db(
    float linear
)
{
    if (
        linear <=
        EPSILON
    ) {

        return
            MIN_DB;
    }


    return
        20.0f *
        std::log10(
            linear
        );
}


float RxOutputStage::peak_dbfs(
    float peak
)
{
    if (
        peak <=
        EPSILON
    ) {

        return
            MIN_DB;
    }


    return
        20.0f *
        std::log10(
            peak /
            FULL_SCALE
        );
}


int16_t RxOutputStage::clamp_int16(
    float sample,
    uint32_t &safety_clamps
)
{
    if (
        sample >
        32767.0f
    ) {

        safety_clamps++;


        return
            32767;
    }


    if (
        sample <
        -32768.0f
    ) {

        safety_clamps++;


        return
            -32768;
    }


    return
        static_cast<int16_t>(
            std::lrint(
                sample
            )
        );
}


void RxOutputStage::set_eq_enabled(
    bool enabled
)
{
    eq_enabled_ =
        enabled;
}


bool RxOutputStage::eq_enabled() const
{
    return
        eq_enabled_;
}


void RxOutputStage::set_master_gain_db(
    float gain_db
)
{
    /*
     * Keep the software volume within a reasonable
     * range.
     */
    if (
        gain_db >
        6.0f
    ) {

        gain_db =
            6.0f;
    }


    if (
        gain_db <
        -60.0f
    ) {

        gain_db =
            -60.0f;
    }


    master_gain_db_ =
        gain_db;


    master_gain_linear_ =
        db_to_linear(
            master_gain_db_
        );
}


float RxOutputStage::master_gain_db() const
{
    return
        master_gain_db_;
}


float RxOutputStage::process_eq_sample(
    float sample,
    size_t channel
)
{
    (void) channel;


    /*
     * H17:
     *
     * EQ architecture exists, but no helmet-specific
     * curve is applied yet.
     *
     * When actual helmet speakers are characterized,
     * this function can become the EQ chain.
     */
    if (
        !eq_enabled_
    ) {

        return
            sample;
    }


    /*
     * Intentional unity EQ placeholder.
     */
    return
        sample;
}


void RxOutputStage::process_block(
    const float *input,
    int16_t *output,
    size_t stereo_frames,
    RxOutputStats &stats
)
{
    stats =
        {};


    if (
        input ==
            nullptr ||
        output ==
            nullptr
    ) {

        return;
    }


    float input_peak =
        0.0f;


    float output_peak =
        0.0f;


    float minimum_gain =
        1.0f;


    for (
        size_t frame = 0;
        frame < stereo_frames;
        ++frame
    ) {

        const size_t left_index =
            frame *
            2;


        const size_t right_index =
            left_index +
            1;


        float left =
            input[
                left_index
            ];


        float right =
            input[
                right_index
            ];


        input_peak =
            std::max(
                input_peak,
                std::max(
                    std::fabs(
                        left
                    ),
                    std::fabs(
                        right
                    )
                )
            );


        /*
         * -----------------------------------------
         * Optional EQ hook.
         * -----------------------------------------
         */

        left =
            process_eq_sample(
                left,
                0
            );


        right =
            process_eq_sample(
                right,
                1
            );


        /*
         * -----------------------------------------
         * Master software volume.
         * -----------------------------------------
         */

        left *=
            master_gain_linear_;


        right *=
            master_gain_linear_;


        /*
         * -----------------------------------------
         * Stereo-linked limiter.
         * -----------------------------------------
         */

        const float frame_peak =
            std::max(
                std::fabs(
                    left
                ),
                std::fabs(
                    right
                )
            )
            /
            FULL_SCALE;


        float required_gain =
            1.0f;


        if (
            frame_peak >
            ceiling_linear_
        ) {

            required_gain =
                ceiling_linear_
                /
                frame_peak;
        }


        /*
         * Attack:
         *
         * immediate.
         *
         * This guarantees the sample cannot exceed
         * the ceiling.
         */
        if (
            required_gain <
            limiter_gain_
        ) {

            limiter_gain_ =
                required_gain;


            stats.limited_frames++;
        }
        else {

            /*
             * Release smoothly toward unity.
             */
            limiter_gain_ =
                1.0f
                -
                (
                    1.0f -
                    limiter_gain_
                )
                *
                release_coeff_;
        }


        /*
         * Exact ceiling guard.
         *
         * If release recovery would exceed the
         * ceiling on the current frame, constrain it.
         */
        if (
            required_gain <
            limiter_gain_
        ) {

            limiter_gain_ =
                required_gain;
        }


        minimum_gain =
            std::min(
                minimum_gain,
                limiter_gain_
            );


        left *=
            limiter_gain_;


        right *=
            limiter_gain_;


        output_peak =
            std::max(
                output_peak,
                std::max(
                    std::fabs(
                        left
                    ),
                    std::fabs(
                        right
                    )
                )
            );


        output[
            left_index
        ] =
            clamp_int16(
                left,
                stats.safety_clamps
            );


        output[
            right_index
        ] =
            clamp_int16(
                right,
                stats.safety_clamps
            );
    }


    stats.input_peak_dbfs =
        peak_dbfs(
            input_peak
        );


    stats.output_peak_dbfs =
        peak_dbfs(
            output_peak
        );


    stats.gain_reduction_db =
        -linear_to_db(
            minimum_gain
        );
}


float RxOutputStage::
current_limiter_gain_db() const
{
    return
        linear_to_db(
            limiter_gain_
        );
}


}