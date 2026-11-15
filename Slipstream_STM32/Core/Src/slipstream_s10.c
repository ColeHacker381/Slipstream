/*
 * Slipstream S10 AGC — C port of main/dsp/agc.cpp.
 * Constants, state and rounding follow original ESP32 implementation.
 * Applies only after S9 VAD (cleaned 16k mono) and before interpolation.
 */
#include "slipstream_s10.h"
#include <math.h>
#include <stddef.h>

#define S10_FULL_SCALE 32768.0f
#define S10_TARGET_DBFS (-24.0f)
#define S10_MAX_GAIN_DB 18.0f
#define S10_MIN_GAIN_DB (-12.0f)
#define S10_BLOCK_SECONDS 0.010f
#define S10_ATTACK_SECONDS 0.030f
#define S10_RELEASE_SECONDS 0.400f
#define S10_IDLE_RETURN_SECONDS 2.0f

static float current_gain_db = 0.0f;

static float clamp_gain(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float calculate_rms_dbfs(const int16_t *input, uint32_t samples)
{
    if (input == NULL || samples == 0U) return -120.0f;
    double sum = 0.0;
    for (uint32_t i = 0; i < samples; i++)
    {
        const double x = (double)input[i];
        sum += x * x;
    }
    const double mean_square = sum / (double)samples;
    if (mean_square < 1.0e-12) return -120.0f;
    const double rms = sqrt(mean_square);
    return (float)(20.0 * log10(rms / (double)S10_FULL_SCALE));
}

static int16_t saturate_to_int16(float sample, uint32_t *clipped)
{
    if (sample > 32767.0f)
    {
        (*clipped)++;
        return 32767;
    }
    if (sample < -32768.0f)
    {
        (*clipped)++;
        return -32768;
    }
    return (int16_t)lrintf(sample);
}

void s10_reset(void)
{
    current_gain_db = 0.0f;
}

float s10_current_gain_db(void)
{
    return current_gain_db;
}

S10_AgcResult s10_process_block(const int16_t *input,
                                int16_t *output,
                                uint32_t samples,
                                uint8_t speech_active)
{
    S10_AgcResult r = {0};
    r.input_dbfs = -120.0f;
    r.output_dbfs = -120.0f;

    if (input == NULL || output == NULL || samples == 0U)
    {
        r.applied_gain_db = current_gain_db;
        return r;
    }

    r.input_dbfs = calculate_rms_dbfs(input, samples);

    float target_gain_db = current_gain_db;
    if (speech_active)
    {
        target_gain_db = clamp_gain(
            S10_TARGET_DBFS - r.input_dbfs,
            S10_MIN_GAIN_DB,
            S10_MAX_GAIN_DB
        );
    }
    else
    {
        /* Silence: never chase noise; relax toward 0 dB. */
        target_gain_db = 0.0f;
    }
    r.target_gain_db = target_gain_db;

    float time_constant;
    if (!speech_active)
        time_constant = S10_IDLE_RETURN_SECONDS;
    else if (target_gain_db < current_gain_db)
        time_constant = S10_ATTACK_SECONDS;
    else
        time_constant = S10_RELEASE_SECONDS;

    const float alpha = 1.0f - expf(-S10_BLOCK_SECONDS / time_constant);
    current_gain_db += alpha * (target_gain_db - current_gain_db);
    current_gain_db = clamp_gain(current_gain_db, S10_MIN_GAIN_DB, S10_MAX_GAIN_DB);
    r.applied_gain_db = current_gain_db;

    const float linear_gain = powf(10.0f, current_gain_db / 20.0f);
    for (uint32_t i = 0; i < samples; i++)
        output[i] = saturate_to_int16((float)input[i] * linear_gain, &r.clipped_samples);

    r.output_dbfs = calculate_rms_dbfs(output, samples);
    return r;
}
