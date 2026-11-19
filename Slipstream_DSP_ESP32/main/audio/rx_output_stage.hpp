#pragma once

#include <cstddef>
#include <cstdint>


namespace audio {


struct RxOutputStats {

    float input_peak_dbfs =
        -120.0f;


    float output_peak_dbfs =
        -120.0f;


    float gain_reduction_db =
        0.0f;


    uint32_t limited_frames =
        0;


    uint32_t safety_clamps =
        0;
};


class RxOutputStage {
public:

    static constexpr uint32_t SAMPLE_RATE =
        48000;


    static constexpr float LIMITER_CEILING_DBFS =
        -3.0f;


    static constexpr float MASTER_GAIN_DB =
        0.0f;


    static constexpr uint32_t LIMITER_RELEASE_MS =
        100;


    RxOutputStage();


    void reset();


    /*
     * EQ is intentionally bypassed for H17.
     *
     * The hook exists so helmet-specific tuning can
     * be inserted here later without changing the
     * RX architecture.
     */
    void set_eq_enabled(
        bool enabled
    );


    bool eq_enabled() const;


    void set_master_gain_db(
        float gain_db
    );


    float master_gain_db() const;


    /*
     * Input:
     *
     *     interleaved stereo float PCM
     *     in int16 amplitude units
     *
     * Output:
     *
     *     interleaved stereo int16 PCM
     */
    void process_block(
        const float *input,
        int16_t *output,
        size_t stereo_frames,
        RxOutputStats &stats
    );


    float current_limiter_gain_db() const;


private:

    static float db_to_linear(
        float db
    );


    static float linear_to_db(
        float linear
    );


    static float peak_dbfs(
        float peak
    );


    static int16_t clamp_int16(
        float sample,
        uint32_t &safety_clamps
    );


    float process_eq_sample(
        float sample,
        size_t channel
    );


    float ceiling_linear_ =
        1.0f;


    float master_gain_linear_ =
        1.0f;


    float master_gain_db_ =
        MASTER_GAIN_DB;


    float limiter_gain_ =
        1.0f;


    float release_coeff_ =
        0.0f;


    bool eq_enabled_ =
        false;
};


}