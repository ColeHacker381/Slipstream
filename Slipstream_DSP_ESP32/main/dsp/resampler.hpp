#pragma once

#include <cstddef>
#include <cstdint>

#include "dsps_fir.h"

namespace dsp {

class VoiceResampler {
public:

    static constexpr size_t INPUT_48K_SAMPLES = 480;
    static constexpr size_t OUTPUT_16K_SAMPLES = 160;

    VoiceResampler();

    void reset();

    void downsample_48k_to_16k(
        const int16_t *input,
        int16_t *output
    );

    void upsample_16k_to_48k(
        const int16_t *input,
        int16_t *output
    );

private:

    static constexpr size_t FIR_TAPS = 33;
    static constexpr size_t PHASE_TAPS = 11;

    /*
     * ESP-DSP 48 kHz -> 16 kHz decimator.
     */
    fir_f32_t decimator_ = {};

    float decim_delay_[FIR_TAPS] = {};

    float input_48k_float_[
        INPUT_48K_SAMPLES
    ] = {};

    float output_16k_float_[
        OUTPUT_16K_SAMPLES
    ] = {};


    /*
     * Interpolator history.
     *
     * Unlike the previous circular buffer,
     * this is kept newest-first:
     *
     * history[0] = newest sample
     * history[1] = previous sample
     * ...
     *
     * This lets ESP-DSP process each phase as
     * one contiguous dot product.
     */
    float interp_history_[
        PHASE_TAPS
    ] = {};
};

}