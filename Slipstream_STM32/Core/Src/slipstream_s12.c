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

void s12_reset(void)
{
    lp20 = 0.0f;
    lp80 = 0.0f;
    lp200 = 0.0f;
    lp1000 = 0.0f;
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
    return r;
}
