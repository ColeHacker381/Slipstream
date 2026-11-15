/*
 * S12.0 exploratory wind/handling-noise feature extraction.
 * Observation ONLY: reads the raw 48k mono input before S7's 100 Hz HPF.
 * Stateful approximate 20-200 Hz band via two 1-pole low-pass paths;
 * no PCM modifications, no AGC/VAD gating. Constants are provisional:
 * test against actual helmet wind, fan, speech, and mic handling.
 */
#include "slipstream_s12.h"
#include <math.h>
#include <stddef.h>

#define LP20_ALPHA (0.00261439f)  /* 1-exp(-2*pi*20/48000) */
#define LP200_ALPHA (0.0258402f) /* 1-exp(-2*pi*200/48000) */
/* Provisional thresholds, not calibrated wind classification. */
#define WIND_RMS_MIN 250.0f
#define WIND_RATIO_MIN 0.60f

static float lp20 = 0.0f;
static float lp200 = 0.0f;

void s12_reset(void)
{
    lp20 = 0.0f;
    lp200 = 0.0f;
}

S12_WindResult s12_analyze_raw_48k(const float *pcm, uint32_t frames)
{
    S12_WindResult r = {0};
    if (pcm == NULL || frames == 0U) return r;
    double raw_sq = 0.0;
    double band_sq = 0.0;
    for (uint32_t i = 0U; i < frames; ++i)
    {
        const float x = pcm[i];
        lp20 += LP20_ALPHA * (x - lp20);
        lp200 += LP200_ALPHA * (x - lp200);
        const float wind_band = lp200 - lp20;
        raw_sq += (double)x * (double)x;
        band_sq += (double)wind_band * (double)wind_band;
    }
    const float raw_rms = sqrtf((float)(raw_sq / (double)frames));
    const float band_rms = sqrtf((float)(band_sq / (double)frames));
    const float ratio = raw_rms > 1.0f ? band_rms / raw_rms : 0.0f;
    r.raw_rms = (uint32_t)raw_rms;
    r.wind_band_rms = (uint32_t)band_rms;
    r.low_ratio_pct = (uint32_t)(fminf(ratio, 1.0f) * 100.0f);
    r.wind_candidate = (band_rms >= WIND_RMS_MIN && ratio >= WIND_RATIO_MIN) ? 1U : 0U;
    return r;
}
