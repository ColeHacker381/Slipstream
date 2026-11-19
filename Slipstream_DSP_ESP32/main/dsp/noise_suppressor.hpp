#pragma once

#include <cstddef>
#include <cstdint>

namespace dsp {

class NoiseSuppressor {
public:

    static constexpr size_t BLOCK_SIZE = 160;
    static constexpr size_t FRAME_SIZE = 320;
    static constexpr size_t FFT_SIZE = 512;
    static constexpr size_t FFT_BINS = FFT_SIZE / 2 + 1;

    NoiseSuppressor();

    /*
     * Clear all streaming state and restart
     * noise-floor estimation.
     */
    void reset();

    /*
     * Process exactly one 10 ms / 160-sample
     * block of 16 kHz mono PCM.
     *
     * input and output may point to different buffers.
     */
    void process_block(
        const int16_t *input,
        int16_t *output
    );

    /*
     * Useful for logs/debugging later.
     */
    bool noise_estimate_ready() const
    {
        return noise_frame_count_ >= NOISE_INIT_FRAMES;
    }

private:

    static constexpr uint32_t NOISE_INIT_FRAMES = 50;

    /*
     * Never completely remove a frequency bin.
     *
     * A hard zero tends to create unnatural
     * "musical noise."
     */
    static constexpr float MIN_GAIN = 0.12f;

    /*
     * How aggressively estimated noise is removed.
     *
     * 1.0 = estimated noise
     * >1.0 = increasingly aggressive.
     */
    static constexpr float OVER_SUBTRACTION = 1.35f;

    /*
     * Slow noise adaptation after startup.
     */
    static constexpr float NOISE_UPDATE_ALPHA = 0.995f;

    /*
     * Only update the noise model when a bin looks
     * reasonably close to the current noise floor.
     *
     * This reduces the chance of learning speech
     * as "noise."
     */
    static constexpr float NOISE_UPDATE_THRESHOLD = 2.5f;


    void initialize_fft();

    void build_window();

    void forward_fft();

    void inverse_fft();

    static int16_t float_to_int16(
        float value
    );


    bool initialized_ = false;

    uint32_t noise_frame_count_ = 0;


    /*
     * Previous 10 ms block.
     *
     * Previous + current creates the 320-sample
     * analysis frame.
     */
    float previous_input_[BLOCK_SIZE] = {};


    /*
     * 320-point sqrt-Hann window.
     *
     * sqrt-Hann analysis + sqrt-Hann synthesis
     * gives Hann overlap/add behavior.
     */
    float window_[FRAME_SIZE] = {};


    /*
     * Interleaved complex FFT buffer:
     *
     * Re[0], Im[0],
     * Re[1], Im[1],
     * ...
     *
     * 512 complex samples = 1024 floats.
     */
    float fft_buffer_[FFT_SIZE * 2] = {};


    /*
     * Estimated noise power spectrum.
     */
    float noise_power_[FFT_BINS] = {};


    /*
     * Smoothed gain for every positive-frequency bin.
     */
    float gain_[FFT_BINS] = {};


    /*
     * Overlap from the previous synthesized frame.
     */
    float overlap_[BLOCK_SIZE] = {};
};

}