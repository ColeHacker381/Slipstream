#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

struct AgcResult {
    float input_dbfs = -120.0f;
    float output_dbfs = -120.0f;

    float target_gain_db = 0.0f;
    float applied_gain_db = 0.0f;

    uint32_t clipped_samples = 0;
};


class AutomaticGainControl {
public:

    static constexpr size_t BLOCK_SIZE = 160;

    AutomaticGainControl();

    void reset();

    AgcResult process_block(
        const int16_t *input,
        int16_t *output,
        size_t samples,
        bool speech_active
    );

    float gain_db() const
    {
        return current_gain_db_;
    }


private:

    /*
     * Desired speech RMS.
     */
    static constexpr float
        TARGET_DBFS = -24.0f;


    /*
     * Gain limits.
     */
    static constexpr float
        MAX_GAIN_DB = 18.0f;

    static constexpr float
        MIN_GAIN_DB = -12.0f;


    /*
     * 10 ms block duration.
     */
    static constexpr float
        BLOCK_SECONDS = 0.010f;


    /*
     * Gain reduction should happen quickly.
     *
     * Example:
     *
     * loud speaker suddenly gets close to mic.
     */
    static constexpr float
        ATTACK_SECONDS = 0.030f;


    /*
     * Gain increases much more slowly.
     *
     * This prevents pumping between syllables
     * and short pauses.
     */
    static constexpr float
        RELEASE_SECONDS = 0.400f;


    /*
     * When speech is not active, slowly move
     * gain back toward unity rather than
     * increasing gain into background noise.
     */
    static constexpr float
        IDLE_RETURN_SECONDS = 2.0f;


    static float calculate_rms_dbfs(
        const int16_t *input,
        size_t samples
    );


    static float db_to_linear(
        float gain_db
    );


    static float clamp(
        float value,
        float minimum,
        float maximum
    );


    static int16_t saturate_to_int16(
        float sample,
        uint32_t &clipped_samples
    );


    float current_gain_db_ = 0.0f;
};

}