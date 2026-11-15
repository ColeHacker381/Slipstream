/*
 * Slipstream S8 — faithful port of main/dsp/noise_suppressor.cpp.
 * ESP32 16 kHz mono 160-sample blocks; 320-sample sqrt-Hann WOLA;
 * 512-complex FFT; 50-block noise training; spectral subtraction.
 *
 * Only the FFT primitive differs: CMSIS-DSP instead of ESP-DSP.
 * No dynamic allocation or FFT/window generation in the real-time path.
 */

#include "slipstream_s8.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include <math.h>
#include <string.h>

#define S8_BINS (S8_FFT_SIZE / 2U + 1U)
#define S8_LEARN_BLOCKS 50U
#define S8_MIN_GAIN 0.12f
#define S8_OVER_SUBTRACTION 1.35f
#define S8_UPDATE_ALPHA 0.995f
#define S8_UPDATE_THRESHOLD 2.5f
#define S8_PI 3.14159265358979323846f

static float s8_previous[S8_BLOCK_SIZE];
static float s8_overlap[S8_BLOCK_SIZE];
static float s8_window[S8_FRAME_SIZE];
static float s8_fft[S8_FFT_SIZE * 2U];
static float s8_noise[S8_BINS];
static float s8_gain[S8_BINS];
static uint32_t s8_frames = 0U;
static uint32_t s8_initialized = 0U;
static uint32_t s8_pcm_clips = 0U;

static int16_t s8_saturate(float x)
{
    if (x > 32767.0f) { s8_pcm_clips++; return 32767; }
    if (x < -32768.0f) { s8_pcm_clips++; return -32768; }
    return (int16_t)x;
}

void s8_reset(void)
{
    memset(s8_previous, 0, sizeof(s8_previous));
    memset(s8_overlap, 0, sizeof(s8_overlap));
    memset(s8_noise, 0, sizeof(s8_noise));
    memset(s8_fft, 0, sizeof(s8_fft));
    for (uint32_t i = 0; i < S8_BINS; i++) s8_gain[i] = 1.0f;
    s8_frames = 0U;
    s8_pcm_clips = 0U;
}

int s8_init(void)
{
    /*
     * CMSIS preinitialized constant struct: 512 complex points.
     * Requires arm_cfft_f32, radix8, bit-reversal, and twiddle tables.
     */
    s8_initialized = 0U;
    for (uint32_t i = 0; i < S8_FRAME_SIZE; i++)
    {
        float hann = 0.5f - 0.5f * cosf(
            (2.0f * S8_PI * (float)i) / (float)S8_FRAME_SIZE
        );
        if (hann < 0.0f) hann = 0.0f;
        s8_window[i] = sqrtf(hann);
    }
    s8_reset();
    s8_initialized = 1U;
    return 1;
}

uint32_t s8_noise_ready(void)
{
    return (s8_initialized && s8_frames >= S8_LEARN_BLOCKS);
}

uint32_t s8_learning_blocks(void)
{
    return s8_frames;
}

uint32_t s8_clipped_samples(void)
{
    return s8_pcm_clips;
}

void s8_process(const int16_t input[S8_BLOCK_SIZE],
                int16_t output[S8_BLOCK_SIZE])
{
    if (!s8_initialized || input == 0 || output == 0)
    {
        if (input != 0 && output != 0)
            memcpy(output, input, S8_BLOCK_SIZE * sizeof(int16_t));
        return;
    }

    /* Previous | current 10 ms blocks; periodic sqrt-Hann analysis. */
    for (uint32_t i = 0; i < S8_BLOCK_SIZE; i++)
    {
        s8_fft[2U * i] = s8_previous[i] * s8_window[i];
        s8_fft[2U * i + 1U] = 0.0f;

        s8_fft[2U * (i + S8_BLOCK_SIZE)] =
            (float)input[i] * s8_window[i + S8_BLOCK_SIZE];
        s8_fft[2U * (i + S8_BLOCK_SIZE) + 1U] = 0.0f;
        s8_previous[i] = (float)input[i];
    }
    for (uint32_t i = S8_FRAME_SIZE; i < S8_FFT_SIZE; i++)
    {
        s8_fft[2U * i] = 0.0f;
        s8_fft[2U * i + 1U] = 0.0f;
    }

    /* Forward FFT: CMSIS bit-reverses output when bitReverseFlag=1. */
    arm_cfft_f32(&arm_cfft_sR_f32_len512, s8_fft, 0U, 1U);

    const uint32_t learning = s8_frames < S8_LEARN_BLOCKS;
    for (uint32_t bin = 0; bin < S8_BINS; bin++)
    {
        const float re = s8_fft[2U * bin];
        const float im = s8_fft[2U * bin + 1U];
        float power = re * re + im * im;
        if (power < 1.0e-12f) power = 1.0e-12f;

        if (learning)
        {
            if (s8_frames == 0U) s8_noise[bin] = power;
            else
            {
                const float n = (float)(s8_frames + 1U);
                s8_noise[bin] += (power - s8_noise[bin]) / n;
            }
            s8_gain[bin] = 1.0f;
        }
        else
        {
            if (power < s8_noise[bin] * S8_UPDATE_THRESHOLD)
            {
                s8_noise[bin] =
                    S8_UPDATE_ALPHA * s8_noise[bin] +
                    (1.0f - S8_UPDATE_ALPHA) * power;
            }

            float clean = power - S8_OVER_SUBTRACTION * s8_noise[bin];
            if (clean < 0.0f) clean = 0.0f;
            float target = sqrtf(clean / power);
            if (target < S8_MIN_GAIN) target = S8_MIN_GAIN;
            if (target > 1.0f) target = 1.0f;

            if (target < s8_gain[bin])
                s8_gain[bin] = 0.80f * s8_gain[bin] + 0.20f * target;
            else
                s8_gain[bin] = 0.35f * s8_gain[bin] + 0.65f * target;
        }
    }
    if (s8_frames < S8_LEARN_BLOCKS) s8_frames++;

    /* Positive and mirrored negative spectrum, plus DC/Nyquist. */
    s8_fft[0] *= s8_gain[0];
    s8_fft[1] *= s8_gain[0];
    for (uint32_t bin = 1; bin < S8_FFT_SIZE / 2U; bin++)
    {
        const float gain = s8_gain[bin];
        s8_fft[2U * bin] *= gain;
        s8_fft[2U * bin + 1U] *= gain;
        uint32_t mirror = S8_FFT_SIZE - bin;
        s8_fft[2U * mirror] *= gain;
        s8_fft[2U * mirror + 1U] *= gain;
    }
    s8_fft[S8_FFT_SIZE] *= s8_gain[S8_FFT_SIZE / 2U];
    s8_fft[S8_FFT_SIZE + 1U] *= s8_gain[S8_FFT_SIZE / 2U];

    /* CMSIS inverse complex FFT includes 1/N scaling. */
    arm_cfft_f32(&arm_cfft_sR_f32_len512, s8_fft, 1U, 1U);

    /* First 160 samples output, 160 samples held for next WOLA hop. */
    for (uint32_t i = 0; i < S8_BLOCK_SIZE; i++)
    {
        const float first = s8_fft[2U * i] * s8_window[i];
        const float second = s8_fft[2U * (i + S8_BLOCK_SIZE)] *
                             s8_window[i + S8_BLOCK_SIZE];
        output[i] = s8_saturate(s8_overlap[i] + first);
        s8_overlap[i] = second;
    }
}
