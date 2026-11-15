#ifndef SLIPSTREAM_S12_H
#define SLIPSTREAM_S12_H
#include <stdint.h>

/* Detection only; no PCM gain, muting or filtering in S12.0. */
typedef struct {
    uint32_t raw_rms;
    uint32_t wind_band_rms;
    uint32_t low_ratio_pct;
    uint8_t wind_candidate;
} S12_WindResult;

void s12_reset(void);
S12_WindResult s12_analyze_raw_48k(const float *pcm, uint32_t frames);
#endif
