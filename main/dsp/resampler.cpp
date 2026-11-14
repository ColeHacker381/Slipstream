#include "resampler.hpp"

#include "dsps_dotprod.h"

#include "esp_err.h"
#include "esp_log.h"

namespace {

constexpr const char *TAG = "resampler";


/*
 * Same 33-tap FIR from H7 v2/v3.
 *
 * IMPORTANT:
 *
 * We are intentionally NOT redesigning the filter
 * during this optimization test.
 *
 * That lets us compare CPU time while keeping the
 * audio response essentially identical.
 */
constexpr float FIR[33] = {

     0.0012146106f,
     0.0023260760f,
     0.0015351378f,
    -0.0031749238f,
    -0.0100945219f,
    -0.0101718408f,
     0.0023477392f,
     0.0218961922f,
     0.0294725790f,
     0.0076512116f,
    -0.0375374010f,
    -0.0658048924f,
    -0.0323882860f,
     0.0736134608f,
     0.2064548135f,
     0.2988090563f,
     0.2988090563f,
     0.2988090563f,
     0.2064548135f,
     0.0736134608f,
    -0.0323882860f,
    -0.0658048924f,
    -0.0375374010f,
     0.0076512116f,
     0.0294725790f,
     0.0218961922f,
     0.0023477392f,
    -0.0101718408f,
    -0.0100945219f,
    -0.0031749238f,
     0.0015351378f,
     0.0023260760f,
     0.0012146106f
};


/*
 * Pre-split polyphase coefficients.
 *
 * Original FIR:
 *
 * phase 0:
 * h[0], h[3], h[6] ... h[30]
 *
 * phase 1:
 * h[1], h[4], h[7] ... h[31]
 *
 * phase 2:
 * h[2], h[5], h[8] ... h[32]
 *
 * Each phase contains exactly 11 coefficients.
 *
 * Making these contiguous allows us to use
 * dsps_dotprod_f32().
 */
constexpr float PHASE_0[11] = {

     FIR[0],
     FIR[3],
     FIR[6],
     FIR[9],
     FIR[12],
     FIR[15],
     FIR[18],
     FIR[21],
     FIR[24],
     FIR[27],
     FIR[30]
};


constexpr float PHASE_1[11] = {

     FIR[1],
     FIR[4],
     FIR[7],
     FIR[10],
     FIR[13],
     FIR[16],
     FIR[19],
     FIR[22],
     FIR[25],
     FIR[28],
     FIR[31]
};


constexpr float PHASE_2[11] = {

     FIR[2],
     FIR[5],
     FIR[8],
     FIR[11],
     FIR[14],
     FIR[17],
     FIR[20],
     FIR[23],
     FIR[26],
     FIR[29],
     FIR[32]
};


inline int16_t float_to_int16(
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

VoiceResampler::VoiceResampler()
{
    esp_err_t err =
        dsps_fird_init_f32(
            &decimator_,
            const_cast<float *>(FIR),
            decim_delay_,
            FIR_TAPS,
            3
        );

    if (err != ESP_OK) {

        ESP_LOGE(
            TAG,
            "ESP-DSP decimator init failed: %s",
            esp_err_to_name(err)
        );
    }
}


void VoiceResampler::reset()
{
    /*
     * Reset decimator delay.
     */
    for (
        size_t i = 0;
        i < FIR_TAPS;
        ++i
    ) {

        decim_delay_[i] = 0.0f;
    }


    /*
     * Reset interpolation history.
     */
    for (
        size_t i = 0;
        i < PHASE_TAPS;
        ++i
    ) {

        interp_history_[i] = 0.0f;
    }


    /*
     * Reinitialize ESP-DSP decimator state.
     */
    ESP_ERROR_CHECK(
        dsps_fird_init_f32(
            &decimator_,
            const_cast<float *>(FIR),
            decim_delay_,
            FIR_TAPS,
            3
        )
    );
}


void VoiceResampler::downsample_48k_to_16k(
    const int16_t *input,
    int16_t *output
)
{
    /*
     * int16 PCM -> float.
     *
     * We intentionally keep ordinary PCM magnitude
     * rather than normalizing to -1.0 ... +1.0.
     */
    for (
        size_t i = 0;
        i < INPUT_48K_SAMPLES;
        ++i
    ) {

        input_48k_float_[i] =
            static_cast<float>(
                input[i]
            );
    }


    /*
     * Optimized ESP-DSP FIR decimator.
     *
     * 480 samples @ 48 kHz
     *
     * / 3
     *
     * 160 samples @ 16 kHz
     */
    int produced =
        dsps_fird_f32(
            &decimator_,
            input_48k_float_,
            output_16k_float_,
            OUTPUT_16K_SAMPLES
        );


    if (
        produced !=
        static_cast<int>(
            OUTPUT_16K_SAMPLES
        )
    ) {

        ESP_LOGW(
            TAG,
            "Unexpected decimator output: %d",
            produced
        );
    }


    /*
     * float -> int16 PCM.
     */
    for (
        size_t i = 0;
        i < OUTPUT_16K_SAMPLES;
        ++i
    ) {

        output[i] =
            float_to_int16(
                output_16k_float_[i]
            );
    }
}


void VoiceResampler::upsample_16k_to_48k(
    const int16_t *input,
    int16_t *output
)
{
    size_t output_index = 0;


    for (
        size_t input_index = 0;
        input_index <
        OUTPUT_16K_SAMPLES;
        ++input_index
    ) {

        /*
         * Shift history one sample older.
         *
         * history[0] remains newest.
         *
         * Because there are only 11 samples,
         * this is inexpensive and removes all
         * circular-buffer index arithmetic from
         * the three FIR phases.
         */
        for (
            size_t i = PHASE_TAPS - 1;
            i > 0;
            --i
        ) {

            interp_history_[i] =
                interp_history_[i - 1];
        }


        /*
         * Store newest input sample.
         */
        interp_history_[0] =
            static_cast<float>(
                input[input_index]
            );


        float phase_0_output = 0.0f;
        float phase_1_output = 0.0f;
        float phase_2_output = 0.0f;


        /*
         * Each interpolated sample is now simply
         * an optimized 11-element dot product.
         */
        esp_err_t err0 =
            dsps_dotprod_f32(
                PHASE_0,
                interp_history_,
                &phase_0_output,
                PHASE_TAPS
            );

        esp_err_t err1 =
            dsps_dotprod_f32(
                PHASE_1,
                interp_history_,
                &phase_1_output,
                PHASE_TAPS
            );

        esp_err_t err2 =
            dsps_dotprod_f32(
                PHASE_2,
                interp_history_,
                &phase_2_output,
                PHASE_TAPS
            );


        /*
         * These calls should not fail with valid,
         * fixed-size buffers.
         *
         * Don't spam logs inside the real-time path.
         * If one fails, emit silence for this group.
         */
        if (
            err0 != ESP_OK ||
            err1 != ESP_OK ||
            err2 != ESP_OK
        ) {

            output[output_index++] = 0;
            output[output_index++] = 0;
            output[output_index++] = 0;

            continue;
        }


        /*
         * Zero-stuff interpolation needs gain
         * compensation equal to the interpolation
         * factor.
         *
         * Keep this identical to H7 v2/v3.
         */
        phase_0_output *= 3.0f;
        phase_1_output *= 3.0f;
        phase_2_output *= 3.0f;


        output[output_index++] =
            float_to_int16(
                phase_0_output
            );

        output[output_index++] =
            float_to_int16(
                phase_1_output
            );

        output[output_index++] =
            float_to_int16(
                phase_2_output
            );
    }
}

}