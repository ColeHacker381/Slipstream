#ifndef SLIPSTREAM_S13_LIVE_H
#define SLIPSTREAM_S13_LIVE_H
#include <stdint.h>
#include "slipstream_s13_frames.h"
typedef struct {
    uint32_t produced, consumed, encoded, decoded, errors;
    uint32_t dropped, high_water, max_encoded_bytes;
    uint32_t enc_max_us, dec_max_us;
    uint32_t enc_avg_us, dec_avg_us;
    uint32_t max_run_us;
    uint32_t stopped, stop_reason;
} S13_LiveStats;
int s13_live_init(void);
void s13_live_push(const int16_t *pcm160);
void s13_live_run_one(void);
void s13_live_stop(uint32_t reason);
void s13_live_get_stats(S13_LiveStats *out);
#endif
