#ifndef SLIPSTREAM_S12_H
#define SLIPSTREAM_S12_H
#include <stdint.h>

/* Raw wind diagnostics and optional conservative S12.3 suppression. */
typedef struct {
    uint32_t raw_rms;
    uint32_t wind_band_rms;
    uint32_t low_ratio_pct;
    uint32_t sub80_rms;     /* approx 20-80 Hz */
    uint32_t mid_rms;       /* approx 200-1000 Hz */
    uint8_t wind_candidate;
    uint8_t sustained_wind; /* 0/1, hysteretic */
} S12_WindResult;

typedef struct {
    uint32_t filtered_blocks;
    uint32_t active_blocks;
    uint32_t last_mix_pct; /* 0..60, blended post-S8 high-pass */
} S12_SuppressStats;

void s12_reset(void);
void s12_suppress_reset(void);
/* Applies optional wind-selective 220 Hz first-order highpass blend.
   Input must be 160-sample, 16 kHz mono, after S8 and before S9. */
void s12_suppress_16k(const int16_t *input, int16_t *output,
                     uint32_t frames, uint8_t enable);
S12_SuppressStats s12_suppress_stats(void);
void s12_suppress_clear_interval(void);
S12_WindResult s12_analyze_raw_48k(const float *pcm, uint32_t frames);
#endif
