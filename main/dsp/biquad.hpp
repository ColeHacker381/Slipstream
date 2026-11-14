#pragma once

#include <cstddef>

namespace dsp {

class Biquad {
public:

    Biquad(
        float b0,
        float b1,
        float b2,
        float a1,
        float a2
    );

    float process(float input);

    void process_block(
        const float *input,
        float *output,
        size_t samples
    );

    void reset();

private:

    float b0_;
    float b1_;
    float b2_;

    float a1_;
    float a2_;

    /*
     * Direct Form II Transposed state.
     */
    float z1_ = 0.0f;
    float z2_ = 0.0f;
};

}