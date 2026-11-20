/*
 * Slipstream S9 energy VAD.
 * Direct C port of main/dsp/vad.cpp; coefficients and state-machine
 * behavior are unchanged. No audio samples are altered.
 *
 * Important source inconsistency: ATTACK_FRAMES=4 in original code,
 * meaning four 10 ms blocks (40 ms), despite its 20 ms comment.
 */
#include "slipstream_s9.h"
#include <math.h>
#include <stddef.h>

#define S9_FULL_SCALE 32768.0f
#define S9_FULL_SCALE_POWER (S9_FULL_SCALE * S9_FULL_SCALE)
#define S9_NOISE_INIT_FRAMES 50U
#define S9_ATTACK_FRAMES 4U
#define S9_HANGOVER_FRAMES 25U
#define S9_START_POWER_RATIO 6.309573f
#define S9_CONTINUE_POWER_RATIO 2.511886f
#define S9_NOISE_ALPHA 0.995f
#define S9_MIN_SPEECH_DBFS (-50.0f)

static uint8_t speech_active = 0U;
static uint32_t noise_frame_count = 0U;
static uint32_t attack_counter = 0U;
static uint32_t hangover_counter = 0U;
static float noise_power = 1.0f;
static float initial_noise_sum = 0.0f;

static float power_to_dbfs(float power)
{
    if (power <= 1.0e-12f) return -120.0f;
    return 10.0f * log10f(power / S9_FULL_SCALE_POWER);
}

static float dbfs_to_power(float dbfs)
{
    return S9_FULL_SCALE_POWER * powf(10.0f, dbfs / 10.0f);
}

void s9_reset(void)
{
    speech_active = 0U;
    noise_frame_count = 0U;
    attack_counter = 0U;
    hangover_counter = 0U;
    noise_power = 1.0f;
    initial_noise_sum = 0.0f;
}

uint8_t s9_noise_estimate_ready(void)
{
    return noise_frame_count >= S9_NOISE_INIT_FRAMES;
}

S9_VadResult s9_process_block(const int16_t *input, uint32_t samples)
{
    S9_VadResult result = {0};
    result.level_dbfs = -120.0f;
    result.noise_dbfs = -120.0f;

    if (input == NULL || samples == 0U)
    {
        result.speech_active = speech_active;
        result.noise_estimate_ready = s9_noise_estimate_ready();
        return result;
    }

    /* Identical double-precision mean-square accumulation as ESP32. */
    double energy_sum = 0.0;
    for (uint32_t i = 0; i < samples; i++)
    {
        double sample = (double)input[i];
        energy_sum += sample * sample;
    }
    float power = (float)(energy_sum / (double)samples);
    if (power < 1.0f) power = 1.0f;
    result.level_dbfs = power_to_dbfs(power);

    if (noise_frame_count < S9_NOISE_INIT_FRAMES)
    {
        initial_noise_sum += power;
        noise_frame_count++;
        noise_power = initial_noise_sum / (float)noise_frame_count;
        result.speech_active = 0U;
        result.noise_estimate_ready = s9_noise_estimate_ready();
        result.noise_dbfs = power_to_dbfs(noise_power);
        return result;
    }

    result.noise_estimate_ready = 1U;
    float start_threshold = noise_power * S9_START_POWER_RATIO;
    float absolute_minimum = dbfs_to_power(S9_MIN_SPEECH_DBFS);
    if (start_threshold < absolute_minimum)
        start_threshold = absolute_minimum;
    float continue_threshold = noise_power * S9_CONTINUE_POWER_RATIO;
    if (continue_threshold > start_threshold)
        continue_threshold = start_threshold;

    if (!speech_active)
    {
        if (power >= start_threshold)
        {
            attack_counter++;
            if (attack_counter >= S9_ATTACK_FRAMES)
            {
                speech_active = 1U;
                result.speech_started = 1U;
                hangover_counter = S9_HANGOVER_FRAMES;
                attack_counter = 0U;
            }
        }
        else
        {
            attack_counter = 0U;
            noise_power = S9_NOISE_ALPHA * noise_power +
                          (1.0f - S9_NOISE_ALPHA) * power;
        }
    }
    else
    {
        if (power >= continue_threshold)
            hangover_counter = S9_HANGOVER_FRAMES;
        else if (hangover_counter > 0U)
            hangover_counter--;
        else
        {
            speech_active = 0U;
            result.speech_ended = 1U;
            attack_counter = 0U;
        }
    }

    result.speech_active = speech_active;
    result.noise_dbfs = power_to_dbfs(noise_power);
    return result;
}
