#include "noise_suppressor.hpp"

#include <cmath>

#include "dsps_fft2r.h"

#include "esp_err.h"
#include "esp_log.h"

namespace {

constexpr const char *TAG = "noise_suppressor";

constexpr float PI =
    3.14159265358979323846f;

}


namespace dsp {

NoiseSuppressor::NoiseSuppressor()
{
    reset();
}


void NoiseSuppressor::initialize_fft()
{
    /*
     * ESP-DSP uses a shared FFT coefficient table.
     *
     * Only initialize it if another DSP component
     * has not already done so.
     */
    if (!dsps_fft2r_initialized) {

        esp_err_t err =
            dsps_fft2r_init_fc32(
                nullptr,
                FFT_SIZE
            );


        if (err != ESP_OK) {

            ESP_LOGE(
                TAG,
                "FFT initialization failed: %s",
                esp_err_to_name(err)
            );

            initialized_ = false;

            return;
        }
    }


    build_window();

    initialized_ = true;


    ESP_LOGI(
        TAG,
        "Noise suppressor initialized: "
        "16 kHz / 320 frame / 160 hop / FFT512"
    );
}


void NoiseSuppressor::build_window()
{
    /*
     * Periodic sqrt-Hann.
     *
     * With a 50% overlap:
     *
     * hann[n] + hann[n + N/2] = 1
     *
     * Because we use sqrt-Hann on analysis
     * AND synthesis:
     *
     * sqrt(hann) * sqrt(hann) = hann
     *
     * giving proper overlap/add reconstruction.
     */
    for (
        size_t n = 0;
        n < FRAME_SIZE;
        ++n
    ) {

        float hann =
            0.5f -
            0.5f *
            std::cos(
                2.0f *
                PI *
                static_cast<float>(n) /
                static_cast<float>(FRAME_SIZE)
            );


        /*
         * Protect against tiny negative floating
         * point values at the ends.
         */
        if (hann < 0.0f) {
            hann = 0.0f;
        }


        window_[n] =
            std::sqrt(hann);
    }
}


void NoiseSuppressor::reset()
{
    initialized_ = false;

    noise_frame_count_ = 0;


    for (
        size_t i = 0;
        i < BLOCK_SIZE;
        ++i
    ) {

        previous_input_[i] = 0.0f;
        overlap_[i] = 0.0f;
    }


    for (
        size_t i = 0;
        i < FFT_BINS;
        ++i
    ) {

        noise_power_[i] = 0.0f;

        /*
         * Start with unity gain.
         */
        gain_[i] = 1.0f;
    }


    for (
        size_t i = 0;
        i < FFT_SIZE * 2;
        ++i
    ) {

        fft_buffer_[i] = 0.0f;
    }


    initialize_fft();
}


int16_t NoiseSuppressor::float_to_int16(
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


void NoiseSuppressor::forward_fft()
{
    /*
     * Radix-2 FFT followed by bit reversal
     * into normal frequency-bin order.
     */
    dsps_fft2r_fc32(
        fft_buffer_,
        FFT_SIZE
    );


    dsps_bit_rev_fc32(
        fft_buffer_,
        FFT_SIZE
    );
}


void NoiseSuppressor::inverse_fft()
{
    /*
     * ESP-DSP gives us the forward FFT primitive.
     *
     * Use:
     *
     * IFFT(X) = conj(
     *              FFT(conj(X))
     *           ) / N
     *
     * Since we only need the real time-domain result,
     * the final imaginary conjugation is not important.
     */


    /*
     * Conjugate frequency-domain signal.
     */
    for (
        size_t i = 0;
        i < FFT_SIZE;
        ++i
    ) {

        fft_buffer_[
            i * 2 + 1
        ] *= -1.0f;
    }


    dsps_fft2r_fc32(
        fft_buffer_,
        FFT_SIZE
    );


    dsps_bit_rev_fc32(
        fft_buffer_,
        FFT_SIZE
    );


    /*
     * Normalize inverse FFT.
     */
    constexpr float SCALE =
        1.0f /
        static_cast<float>(
            FFT_SIZE
        );


    for (
        size_t i = 0;
        i < FFT_SIZE;
        ++i
    ) {

        fft_buffer_[
            i * 2
        ] *= SCALE;


        /*
         * Complete the mathematical conjugation
         * for correctness/debugging.
         */
        fft_buffer_[
            i * 2 + 1
        ] *= -SCALE;
    }
}


void NoiseSuppressor::process_block(
    const int16_t *input,
    int16_t *output
)
{
    if (!initialized_) {

        /*
         * If FFT setup failed, bypass instead
         * of breaking the audio stream.
         */
        for (
            size_t i = 0;
            i < BLOCK_SIZE;
            ++i
        ) {

            output[i] =
                input[i];
        }

        return;
    }


    /*
     * ------------------------------------------------
     * 1. Construct the 20 ms analysis frame.
     *
     * frame:
     *
     * previous 10 ms | current 10 ms
     *
     * Then apply sqrt-Hann.
     * ------------------------------------------------
     */

    for (
        size_t i = 0;
        i < BLOCK_SIZE;
        ++i
    ) {

        fft_buffer_[
            i * 2
        ] =
            previous_input_[i] *
            window_[i];


        fft_buffer_[
            i * 2 + 1
        ] = 0.0f;


        fft_buffer_[
            (i + BLOCK_SIZE) * 2
        ] =
            static_cast<float>(
                input[i]
            ) *
            window_[
                i + BLOCK_SIZE
            ];


        fft_buffer_[
            (i + BLOCK_SIZE) * 2 + 1
        ] = 0.0f;
    }


    /*
     * Save current block for next frame.
     */
    for (
        size_t i = 0;
        i < BLOCK_SIZE;
        ++i
    ) {

        previous_input_[i] =
            static_cast<float>(
                input[i]
            );
    }


    /*
     * Zero pad 320 -> 512.
     */
    for (
        size_t i = FRAME_SIZE;
        i < FFT_SIZE;
        ++i
    ) {

        fft_buffer_[
            i * 2
        ] = 0.0f;

        fft_buffer_[
            i * 2 + 1
        ] = 0.0f;
    }


    /*
     * ------------------------------------------------
     * 2. Analysis FFT.
     * ------------------------------------------------
     */

    forward_fft();


    bool learning_noise =
        noise_frame_count_ <
        NOISE_INIT_FRAMES;


    /*
     * ------------------------------------------------
     * 3. Estimate noise and calculate suppression
     *    gain for positive-frequency bins.
     * ------------------------------------------------
     */

    for (
        size_t bin = 0;
        bin < FFT_BINS;
        ++bin
    ) {

        float real =
            fft_buffer_[
                bin * 2
            ];

        float imag =
            fft_buffer_[
                bin * 2 + 1
            ];


        float power =
            real * real +
            imag * imag;


        /*
         * Avoid divide-by-zero.
         */
        if (power < 1.0e-12f) {
            power = 1.0e-12f;
        }


        if (learning_noise) {

            /*
             * Running average during initial
             * noise-learning period.
             */
            if (noise_frame_count_ == 0) {

                noise_power_[bin] =
                    power;
            }
            else {

                float count =
                    static_cast<float>(
                        noise_frame_count_ + 1
                    );


                noise_power_[bin] +=
                    (
                        power -
                        noise_power_[bin]
                    ) /
                    count;
            }


            /*
             * Do not suppress while learning.
             */
            gain_[bin] = 1.0f;
        }
        else {

            /*
             * ------------------------------------------------
             * Slowly adapt noise estimate only if this
             * bin looks reasonably noise-like.
             *
             * Strong speech bins therefore do not quickly
             * contaminate the noise model.
             * ------------------------------------------------
             */

            if (
                power <
                noise_power_[bin] *
                NOISE_UPDATE_THRESHOLD
            ) {

                noise_power_[bin] =
                    NOISE_UPDATE_ALPHA *
                    noise_power_[bin]
                    +
                    (
                        1.0f -
                        NOISE_UPDATE_ALPHA
                    ) *
                    power;
            }


            /*
             * Spectral subtraction in the POWER domain.
             */
            float clean_power =
                power -
                OVER_SUBTRACTION *
                noise_power_[bin];


            if (clean_power < 0.0f) {
                clean_power = 0.0f;
            }


            /*
             * Convert power ratio to an amplitude gain.
             */
            float target_gain =
                std::sqrt(
                    clean_power /
                    power
                );


            if (
                target_gain <
                MIN_GAIN
            ) {

                target_gain =
                    MIN_GAIN;
            }


            if (
                target_gain >
                1.0f
            ) {

                target_gain =
                    1.0f;
            }


            /*
             * ------------------------------------------------
             * Temporal gain smoothing.
             *
             * Suppression comes in relatively slowly to
             * avoid abrupt spectral holes.
             *
             * Speech recovery is faster so consonants and
             * word attacks do not get swallowed.
             * ------------------------------------------------
             */

            if (
                target_gain <
                gain_[bin]
            ) {

                gain_[bin] =
                    0.80f *
                    gain_[bin]
                    +
                    0.20f *
                    target_gain;
            }
            else {

                gain_[bin] =
                    0.35f *
                    gain_[bin]
                    +
                    0.65f *
                    target_gain;
            }
        }
    }


    if (
        noise_frame_count_ <
        NOISE_INIT_FRAMES
    ) {

        noise_frame_count_++;
    }


    /*
     * ------------------------------------------------
     * 4. Apply the calculated gain.
     *
     * Real input produces conjugate-symmetric FFT.
     * Modify both positive and negative frequency bins.
     * ------------------------------------------------
     */


    /*
     * DC.
     */
    fft_buffer_[0] *= gain_[0];
    fft_buffer_[1] *= gain_[0];


    /*
     * Bins 1 ... 255.
     */
    for (
        size_t bin = 1;
        bin < FFT_SIZE / 2;
        ++bin
    ) {

        float g =
            gain_[bin];


        /*
         * Positive frequency.
         */
        fft_buffer_[
            bin * 2
        ] *= g;

        fft_buffer_[
            bin * 2 + 1
        ] *= g;


        /*
         * Matching negative frequency.
         */
        size_t mirror =
            FFT_SIZE -
            bin;


        fft_buffer_[
            mirror * 2
        ] *= g;

        fft_buffer_[
            mirror * 2 + 1
        ] *= g;
    }


    /*
     * Nyquist bin.
     */
    constexpr size_t NYQUIST =
        FFT_SIZE / 2;


    fft_buffer_[
        NYQUIST * 2
    ] *=
        gain_[NYQUIST];

    fft_buffer_[
        NYQUIST * 2 + 1
    ] *=
        gain_[NYQUIST];


    /*
     * ------------------------------------------------
     * 5. Synthesis IFFT.
     * ------------------------------------------------
     */

    inverse_fft();


    /*
     * ------------------------------------------------
     * 6. sqrt-Hann synthesis + overlap/add.
     *
     * We output the FIRST 160 samples of this
     * 320-sample frame.
     *
     * This adds one 10 ms block of algorithmic latency,
     * which is expected for 50% WOLA.
     * ------------------------------------------------
     */

    for (
        size_t i = 0;
        i < BLOCK_SIZE;
        ++i
    ) {

        float first_half =
            fft_buffer_[
                i * 2
            ] *
            window_[i];


        float second_half =
            fft_buffer_[
                (i + BLOCK_SIZE) * 2
            ] *
            window_[
                i + BLOCK_SIZE
            ];


        float sample =
            overlap_[i] +
            first_half;


        output[i] =
            float_to_int16(
                sample
            );


        overlap_[i] =
            second_half;
    }
}

}