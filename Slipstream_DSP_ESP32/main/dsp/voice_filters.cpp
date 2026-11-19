#include "voice_filters.hpp"

#include "biquad.hpp"

namespace {

/*
 * Slipstream voice domain:
 *
 * 16 kHz mono
 *
 *
 * 100 Hz Butterworth HPF
 */
dsp::Biquad high_pass(
     0.9726138985f,
    -1.9452277970f,
     0.9726138985f,

    -1.9444776578f,
     0.9459779362f
);


/*
 * 6 kHz Butterworth LPF
 */
dsp::Biquad low_pass(
    0.5690355937f,
    1.1380711875f,
    0.5690355937f,

    0.9428090416f,
    0.3333333333f
);


int16_t float_to_int16(
    float value
)
{
    if (value > 32767.0f) {
        return 32767;
    }

    if (value < -32768.0f) {
        return -32768;
    }

    return static_cast<int16_t>(
        value
    );
}

}


namespace dsp {

void process_voice_filters(
    const int16_t *input,
    int16_t *output,
    size_t samples
)
{
    for (
        size_t i = 0;
        i < samples;
        ++i
    ) {

        /*
         * Keep the signal in PCM-like float units.
         *
         * Example:
         *
         * int16 5000
         *
         * becomes:
         *
         * float 5000.0
         */
        float sample =
            static_cast<float>(
                input[i]
            );


        /*
         * First remove low-frequency rumble/DC.
         */
        sample =
            high_pass.process(
                sample
            );


        /*
         * Then band-limit the voice signal.
         */
        sample =
            low_pass.process(
                sample
            );


        output[i] =
            float_to_int16(
                sample
            );
    }
}


void reset_voice_filters()
{
    high_pass.reset();
    low_pass.reset();
}

}