/*
 * Slipstream S11 — faithful C port of ESP32 main/dsp/limiter.cpp.
 * 16 kHz mono PCM; 1 ms lookahead (16 samples); -3 dBFS ceiling;
 * 0.5 ms attack, 100 ms release. Static buffers; no heap.
 * S11 consumes the H10 AGC output, prior to 16->48 kHz interpolation.
 */
#include "slipstream_s11.h"
#include <math.h>
#include <stddef.h>

#define S11_FULL_SCALE 32768.0f
#define S11_CEILING_LINEAR 0.707945784f
#define S11_ATTACK_SETTLING_ERROR 0.001f
#define S11_SAMPLE_RATE 16000.0f
#define S11_ATTACK_MS 0.5f
#define S11_RELEASE_MS 100.0f

static int16_t delay_buffer[S11_LOOKAHEAD_SAMPLES];
static uint32_t write_index = 0U;
static float current_gain = 1.0f;
static float attack_coeff = 0.0f;
static float release_coeff = 0.0f;

static float linear_to_db(float linear)
{
    if (linear <= 1.0e-12f) return -120.0f;
    return 20.0f * log10f(linear);
}

static float sample_peak_dbfs(const int16_t *samples, uint32_t count)
{
    int32_t peak = 0;
    for (uint32_t i = 0U; i < count; ++i)
    {
        int32_t magnitude = (int32_t)samples[i];
        if (magnitude < 0) magnitude = -magnitude;
        if (magnitude > peak) peak = magnitude;
    }
    if (peak == 0) return -120.0f;
    return 20.0f * log10f((float)peak / S11_FULL_SCALE);
}

void s11_reset(void)
{
    for (uint32_t i = 0U; i < S11_LOOKAHEAD_SAMPLES; ++i)
        delay_buffer[i] = 0;
    write_index = 0U;
    current_gain = 1.0f;

    const float attack_samples = S11_SAMPLE_RATE * (S11_ATTACK_MS / 1000.0f);
    attack_coeff = powf(S11_ATTACK_SETTLING_ERROR, 1.0f / attack_samples);
    const float release_seconds = S11_RELEASE_MS / 1000.0f;
    release_coeff = expf(-1.0f / (S11_SAMPLE_RATE * release_seconds));
}

float s11_current_gain_db(void)
{
    return linear_to_db(current_gain);
}

S11_LimiterResult s11_process_block(const int16_t *input,
                                    int16_t *output,
                                    uint32_t samples)
{
    S11_LimiterResult result = {0};
    result.input_peak_dbfs = -120.0f;
    result.output_peak_dbfs = -120.0f;
    if (input == NULL || output == NULL || samples == 0U)
        return result;

    result.input_peak_dbfs = sample_peak_dbfs(input, samples);
    float minimum_gain_seen = 1.0f;

    for (uint32_t i = 0U; i < samples; ++i)
    {
        /* Read delayed sample, then add the newest future sample. */
        int16_t delayed_sample = delay_buffer[write_index];
        delay_buffer[write_index] = input[i];
        write_index++;
        if (write_index >= S11_LOOKAHEAD_SAMPLES) write_index = 0U;

        /* Exact ESP32 scan across the 16-sample lookahead buffer. */
        int32_t future_peak = 0;
        for (uint32_t j = 0U; j < S11_LOOKAHEAD_SAMPLES; ++j)
        {
            int32_t magnitude = (int32_t)delay_buffer[j];
            if (magnitude < 0) magnitude = -magnitude;
            if (magnitude > future_peak) future_peak = magnitude;
        }

        float target_gain = 1.0f;
        if (future_peak > 0)
        {
            float normalized_peak = (float)future_peak / S11_FULL_SCALE;
            if (normalized_peak > S11_CEILING_LINEAR)
            {
                target_gain = S11_CEILING_LINEAR / normalized_peak;
                result.threshold_hits++;
            }
        }

        if (target_gain < current_gain)
            current_gain = target_gain +
                attack_coeff * (current_gain - target_gain);
        else
            current_gain = target_gain +
                release_coeff * (current_gain - target_gain);

        if (current_gain > 1.0f) current_gain = 1.0f;
        if (current_gain < 0.0f) current_gain = 0.0f;

        /* Cap gain itself to protect the exact delayed output sample. */
        float output_gain = current_gain;
        int32_t delayed_magnitude = (int32_t)delayed_sample;
        if (delayed_magnitude < 0) delayed_magnitude = -delayed_magnitude;
        if (delayed_magnitude > 0)
        {
            float normalized_delayed =
                (float)delayed_magnitude / S11_FULL_SCALE;
            if (normalized_delayed > S11_CEILING_LINEAR)
            {
                float exact_required_gain =
                    S11_CEILING_LINEAR / normalized_delayed;
                if (exact_required_gain < output_gain)
                    output_gain = exact_required_gain;
            }
        }

        if (output_gain < minimum_gain_seen)
            minimum_gain_seen = output_gain;

        float limited = (float)delayed_sample * output_gain;
        const float positive_ceiling = S11_CEILING_LINEAR * 32767.0f;
        const float negative_ceiling = -S11_CEILING_LINEAR * 32768.0f;
        if (limited > positive_ceiling + 1.0f)
        {
            limited = positive_ceiling;
            result.safety_clamps++;
        }
        if (limited < negative_ceiling - 1.0f)
        {
            limited = negative_ceiling;
            result.safety_clamps++;
        }
        output[i] = (int16_t)lrintf(limited);
    }

    result.output_peak_dbfs = sample_peak_dbfs(output, samples);
    result.gain_reduction_db = linear_to_db(minimum_gain_seen);
    return result;
}
