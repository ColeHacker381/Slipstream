#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

struct LimiterResult {
    float input_peak_dbfs = -120.0f;
    float output_peak_dbfs = -120.0f;

    float gain_reduction_db = 0.0f;

    uint32_t threshold_hits = 0;
    uint32_t safety_clamps = 0;
};


class LookaheadLimiter {
public:

    static constexpr size_t BLOCK_SIZE = 160;

    /*
     * Earlier desktop-tested values.
     */
    static constexpr int SAMPLE_RATE = 16000;

    static constexpr float CEILING_DBFS = -3.0f;

    static constexpr float LOOKAHEAD_MS = 1.0f;

    static constexpr float ATTACK_MS = 0.5f;

    static constexpr float RELEASE_MS = 100.0f;

    /*
     * 1 ms at 16 kHz = 16 samples.
     */
    static constexpr size_t LOOKAHEAD_SAMPLES = 16;


    LookaheadLimiter();

    void reset();


    LimiterResult process_block(
        const int16_t *input,
        int16_t *output,
        size_t samples
    );


    float current_gain_db() const;


private:

    static constexpr float
        CEILING_LINEAR = 0.707945784f;


    static float db_to_linear(
        float db
    );


    static float linear_to_db(
        float linear
    );


    static float sample_peak_dbfs(
        const int16_t *samples,
        size_t count
    );


    int16_t delay_buffer_[
        LOOKAHEAD_SAMPLES
    ] = {};


    size_t write_index_ = 0;

    float current_gain_ = 1.0f;

    float attack_coeff_ = 0.0f;

    float release_coeff_ = 0.0f;
};

}