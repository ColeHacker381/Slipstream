/*
 * S12.2 exploratory wind/handling-noise feature extraction.
 * Observation ONLY: reads the raw 48k mono input before S7's 100 Hz HPF.
 * Stateful approximate 20-200 Hz, 20-80 Hz and 200-1000 Hz
 * bands via single-pole low-pass differences;
 * no PCM modifications, no AGC/VAD gating. Constants are provisional:
 * test against actual helmet wind, fan, speech, and mic handling.
 */
#include "slipstream_s12.h"
#include <math.h>
#include <stddef.h>

#define LP20_ALPHA (0.00261439f)  /* 1-exp(-2*pi*20/48000) */
#define LP200_ALPHA (0.0258402f) /* 1-exp(-2*pi*200/48000) */
#define LP80_ALPHA (0.010417f)  /* 1-exp(-2*pi*80/48000) */
#define LP1000_ALPHA (0.1227f) /* 1-exp(-2*pi*1000/48000) */
/* Provisional thresholds, not calibrated wind classification. */
#define WIND_RMS_MIN 250.0f
#define WIND_RATIO_MIN 0.60f

static float lp20 = 0.0f;
static float lp80 = 0.0f;
static float lp200 = 0.0f;
static float lp1000 = 0.0f;

/* Conservative, experimental. Candidate must persist 250 ms.
   Wind false positives are possible, so suppression only blends
   in a low-frequency highpass; never gates or changes S9/S10. */
#define S12_WIND_ATTACK_BLOCKS 25U
#define S12_WIND_RELEASE_BLOCKS 45U
#define S12_WIND_RAW_MIN 1100U
#define S12_WIND_SUB80_MIN 170U
#define S12_MAX_BLEND 0.60f
#define S12_BLEND_ALPHA 0.035f /* ~280 ms / block */
#define S12_HP_ALPHA 0.9173f /* 1/(1+2*pi*220/16000) */
static uint32_t wind_consecutive = 0U;
static uint32_t wind_release = 0U;
static uint8_t wind_active = 0U;
static float hp_previous_x = 0.0f;
static float hp_previous_y = 0.0f;
static float blend = 0.0f;
static S12_SuppressStats suppress_stats = {0};

void s12_reset(void)
{
    lp20 = 0.0f;
    lp80 = 0.0f;
    lp200 = 0.0f;
    lp1000 = 0.0f;
    wind_consecutive = 0U;
    wind_release = 0U;
    wind_active = 0U;
    s12_suppress_reset();
}

S12_WindResult s12_analyze_raw_48k(const float *pcm, uint32_t frames)
{
    S12_WindResult r = {0};
    if (pcm == NULL || frames == 0U) return r;
    /* Cortex-M4F has hardware float but emulates double arithmetic.
       Single precision provides ample range for 480 PCM samples. */
    float raw_sq = 0.0f;
    float band_sq = 0.0f;
    float sub80_sq = 0.0f;
    float mid_sq = 0.0f;
    for (uint32_t i = 0U; i < frames; ++i)
    {
        const float x = pcm[i];
        lp20 += LP20_ALPHA * (x - lp20);
        lp80 += LP80_ALPHA * (x - lp80);
        lp200 += LP200_ALPHA * (x - lp200);
        lp1000 += LP1000_ALPHA * (x - lp1000);
        const float wind_band = lp200 - lp20;
        const float sub80 = lp80 - lp20;
        const float mid = lp1000 - lp200;
        raw_sq += x * x;
        band_sq += wind_band * wind_band;
        sub80_sq += sub80 * sub80;
        mid_sq += mid * mid;
    }
    const float raw_rms = sqrtf(raw_sq / (float)frames);
    const float band_rms = sqrtf(band_sq / (float)frames);
    /* Ratios near digital silence are dominated by the IIR filter tail:
       report zero below meaningful raw signal level, never classify it. */
    const float ratio = raw_rms >= 25.0f ? band_rms / raw_rms : 0.0f;
    r.raw_rms = (uint32_t)raw_rms;
    r.wind_band_rms = (uint32_t)band_rms;
    r.sub80_rms = (uint32_t)sqrtf(sub80_sq / (float)frames);
    r.mid_rms = (uint32_t)sqrtf(mid_sq / (float)frames);
    r.low_ratio_pct = (uint32_t)(fminf(ratio, 1.0f) * 100.0f);
    r.wind_candidate = (band_rms >= WIND_RMS_MIN && ratio >= WIND_RATIO_MIN) ? 1U : 0U;
    /* Distinct conservative wind state for S12.3. The legacy
       wind_candidate flag remains untouched for historical comparison. */
    const uint8_t likely_wind =
        (r.raw_rms >= S12_WIND_RAW_MIN &&
         r.sub80_rms >= S12_WIND_SUB80_MIN) ? 1U : 0U;
    if (likely_wind)
    {
        if (wind_consecutive < S12_WIND_ATTACK_BLOCKS)
            wind_consecutive++;
        wind_release = S12_WIND_RELEASE_BLOCKS;
        if (wind_consecutive >= S12_WIND_ATTACK_BLOCKS)
            wind_active = 1U;
    }
    else
    {
        wind_consecutive = 0U;
        if (wind_release > 0U)
            wind_release--;
        if (wind_release == 0U)
            wind_active = 0U;
    }
    r.sustained_wind = wind_active;
    return r;
}

void s12_suppress_reset(void)
{
    hp_previous_x = 0.0f;
    hp_previous_y = 0.0f;
    blend = 0.0f;
    suppress_stats = (S12_SuppressStats){0};
}

void s12_suppress_16k(const int16_t *input, int16_t *output,
                     uint32_t frames, uint8_t enable)
{
    if (input == NULL || output == NULL || frames == 0U)
        return;

    /* Always track highpass filter state, even when disabled.
       Bypass returns exact unmodified int16 PCM. */
    const float target = (enable && wind_active) ? S12_MAX_BLEND : 0.0f;
    blend += S12_BLEND_ALPHA * (target - blend);
    if (blend < 0.0f) blend = 0.0f;
    if (blend > S12_MAX_BLEND) blend = S12_MAX_BLEND;
    if (enable && blend >= 0.005f) suppress_stats.active_blocks++;

    /* This is intentionally NOT a hard 220Hz cutoff: unchanged
       speech is mixed back in, preserving the voice fundamentals. */
    for (uint32_t i = 0U; i < frames; i++)
    {
        float x = (float)input[i];
        float hp = S12_HP_ALPHA * (hp_previous_y + x - hp_previous_x);
        hp_previous_x = x;
        hp_previous_y = hp;
        float y = enable ? x + blend * (hp - x) : x;
        if (y > 32767.0f) y = 32767.0f;
        if (y < -32768.0f) y = -32768.0f;
        output[i] = (int16_t)y;
    }
    suppress_stats.filtered_blocks++;
    suppress_stats.last_mix_pct = (uint32_t)(blend * 100.0f);
}

S12_SuppressStats s12_suppress_stats(void)
{
    return suppress_stats;
}

void s12_suppress_clear_interval(void)
{
    suppress_stats.filtered_blocks = 0U;
    suppress_stats.active_blocks = 0U;
}
