#include "biquad.hpp"

namespace dsp {

Biquad::Biquad(
    float b0,
    float b1,
    float b2,
    float a1,
    float a2
)
    :
    b0_(b0),
    b1_(b1),
    b2_(b2),
    a1_(a1),
    a2_(a2)
{
}


float Biquad::process(
    float input
)
{
    /*
     * Direct Form II Transposed:
     *
     * y[n] =
     *     b0*x[n] + z1
     *
     * z1 =
     *     b1*x[n]
     *     - a1*y[n]
     *     + z2
     *
     * z2 =
     *     b2*x[n]
     *     - a2*y[n]
     *
     * IMPORTANT:
     *
     * The a1/a2 values are stored using the
     * standard denominator convention:
     *
     * 1 + a1*z^-1 + a2*z^-2
     */
    float output =
        b0_ * input +
        z1_;

    z1_ =
        b1_ * input
        - a1_ * output
        + z2_;

    z2_ =
        b2_ * input
        - a2_ * output;

    return output;
}


void Biquad::process_block(
    const float *input,
    float *output,
    size_t samples
)
{
    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        output[i] =
            process(
                input[i]
            );
    }
}


void Biquad::reset()
{
    z1_ = 0.0f;
    z2_ = 0.0f;
}

}