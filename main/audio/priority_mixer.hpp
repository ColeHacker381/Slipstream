#pragma once

#include <cstddef>
#include <cstdint>


namespace audio {


class PriorityMixer {
public:

    static constexpr uint32_t SAMPLE_RATE =
        48000;


    static constexpr float BT_NORMAL_DB =
        -3.0f;


    static constexpr float BT_DUCKED_DB =
        -20.0f;


    static constexpr float RF_GAIN_DB =
        0.0f;


    static constexpr uint32_t ATTACK_MS =
        50;


    static constexpr uint32_t HOLD_MS =
        200;


    static constexpr uint32_t RELEASE_MS =
        500;


    static constexpr uint32_t RF_TIMEOUT_MS =
        1000;


    PriorityMixer();


    void reset();


    void set_rf_speech_active(
        bool active
    );


    /*
     * H16 compatibility path.
     *
     * Mixes and immediately converts to int16.
     */
    void process_block(
        const int16_t *bt_input,
        const int16_t *rf_input,
        int16_t *output,
        size_t stereo_samples
    );


    /*
     * H17 path.
     *
     * Produces UNCLIPPED floating-point PCM in
     * int16 amplitude units.
     *
     * Example:
     *
     *     +32767.0f = approximately 0 dBFS
     *
     * Values greater than +/-32767 are allowed here
     * so the RX limiter can properly control them
     * before final int16 conversion.
     */
    void process_block_float(
        const int16_t *bt_input,
        const int16_t *rf_input,
        float *output,
        size_t stereo_samples
    );


    float current_bt_gain_db() const;


    bool ducking() const;


private:

    static float db_to_linear(
        float db
    );


    static int16_t saturate_int16(
        float value
    );


    void update_gain();


    float normal_gain_ =
        1.0f;


    float ducked_gain_ =
        1.0f;


    float rf_gain_ =
        1.0f;


    float current_bt_gain_ =
        1.0f;


    bool rf_speech_active_ =
        false;


    bool ducking_ =
        false;


    uint32_t silence_ms_ =
        0;


    uint32_t blocks_without_rf_update_ms_ =
        0;
};


}