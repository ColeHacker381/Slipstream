#include "priority_mixer.hpp"

#include <cmath>


namespace {

constexpr uint32_t BLOCK_MS =
    10;


constexpr float MIN_DB =
    -120.0f;

}


namespace audio {


PriorityMixer::PriorityMixer()
{
    normal_gain_ =
        db_to_linear(
            BT_NORMAL_DB
        );


    ducked_gain_ =
        db_to_linear(
            BT_DUCKED_DB
        );


    rf_gain_ =
        db_to_linear(
            RF_GAIN_DB
        );


    reset();
}


void PriorityMixer::reset()
{
    current_bt_gain_ =
        normal_gain_;


    rf_speech_active_ =
        false;


    ducking_ =
        false;


    silence_ms_ =
        0;


    blocks_without_rf_update_ms_ =
        0;
}


float PriorityMixer::db_to_linear(
    float db
)
{
    if (
        db <=
        MIN_DB
    ) {

        return
            0.0f;
    }


    return
        std::pow(
            10.0f,
            db /
            20.0f
        );
}


int16_t PriorityMixer::saturate_int16(
    float value
)
{
    if (
        value >
        32767.0f
    ) {

        return
            32767;
    }


    if (
        value <
        -32768.0f
    ) {

        return
            -32768;
    }


    return
        static_cast<int16_t>(
            value
        );
}


void PriorityMixer::set_rf_speech_active(
    bool active
)
{
    rf_speech_active_ =
        active;


    blocks_without_rf_update_ms_ =
        0;


    if (
        active
    ) {

        ducking_ =
            true;


        silence_ms_ =
            0;
    }
}


void PriorityMixer::update_gain()
{
    blocks_without_rf_update_ms_ +=
        BLOCK_MS;


    if (
        blocks_without_rf_update_ms_ >=
        RF_TIMEOUT_MS
    ) {

        rf_speech_active_ =
            false;
    }


    if (
        rf_speech_active_
    ) {

        ducking_ =
            true;


        silence_ms_ =
            0;
    }
    else if (
        ducking_
    ) {

        silence_ms_ +=
            BLOCK_MS;
    }


    float target_gain =
        normal_gain_;


    uint32_t transition_ms =
        RELEASE_MS;


    if (
        rf_speech_active_ ||
        (
            ducking_ &&
            silence_ms_ <
                HOLD_MS
        )
    ) {

        target_gain =
            ducked_gain_;


        transition_ms =
            ATTACK_MS;
    }
    else {

        target_gain =
            normal_gain_;


        transition_ms =
            RELEASE_MS;
    }


    float alpha =
        static_cast<float>(
            BLOCK_MS
        )
        /
        static_cast<float>(
            transition_ms
        );


    if (
        alpha >
        1.0f
    ) {

        alpha =
            1.0f;
    }


    current_bt_gain_ +=
        (
            target_gain -
            current_bt_gain_
        )
        *
        alpha;


    if (
        !rf_speech_active_ &&
        silence_ms_ >=
            HOLD_MS &&
        std::fabs(
            current_bt_gain_ -
            normal_gain_
        )
        <
        0.001f
    ) {

        current_bt_gain_ =
            normal_gain_;


        ducking_ =
            false;


        silence_ms_ =
            0;
    }
}


void PriorityMixer::process_block_float(
    const int16_t *bt_input,
    const int16_t *rf_input,
    float *output,
    size_t stereo_samples
)
{
    if (
        output ==
        nullptr
    ) {

        return;
    }


    update_gain();


    for (
        size_t i = 0;
        i < stereo_samples;
        ++i
    ) {

        float bt =
            0.0f;


        float rf =
            0.0f;


        if (
            bt_input !=
            nullptr
        ) {

            bt =
                static_cast<float>(
                    bt_input[i]
                )
                *
                current_bt_gain_;
        }


        if (
            rf_input !=
            nullptr
        ) {

            rf =
                static_cast<float>(
                    rf_input[i]
                )
                *
                rf_gain_;
        }


        /*
         * Deliberately NO saturation here.
         *
         * H17 receives the complete mixed waveform,
         * even if it exceeds int16 full scale.
         */
        output[i] =
            bt +
            rf;
    }
}


void PriorityMixer::process_block(
    const int16_t *bt_input,
    const int16_t *rf_input,
    int16_t *output,
    size_t stereo_samples
)
{
    if (
        output ==
        nullptr
    ) {

        return;
    }


    /*
     * Preserve the original H16 behavior without
     * allocating another block buffer.
     */

    update_gain();


    for (
        size_t i = 0;
        i < stereo_samples;
        ++i
    ) {

        float bt =
            0.0f;


        float rf =
            0.0f;


        if (
            bt_input !=
            nullptr
        ) {

            bt =
                static_cast<float>(
                    bt_input[i]
                )
                *
                current_bt_gain_;
        }


        if (
            rf_input !=
            nullptr
        ) {

            rf =
                static_cast<float>(
                    rf_input[i]
                )
                *
                rf_gain_;
        }


        output[i] =
            saturate_int16(
                bt +
                rf
            );
    }
}


float PriorityMixer::current_bt_gain_db() const
{
    if (
        current_bt_gain_ <=
        0.000001f
    ) {

        return
            MIN_DB;
    }


    return
        20.0f *
        std::log10(
            current_bt_gain_
        );
}


bool PriorityMixer::ducking() const
{
    return
        ducking_;
}


}