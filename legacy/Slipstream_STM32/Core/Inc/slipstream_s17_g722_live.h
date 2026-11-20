#ifndef SLIPSTREAM_S17_G722_LIVE_H
#define SLIPSTREAM_S17_G722_LIVE_H
#include <stdint.h>
#include "slipstream_s13_frames.h"
/* S17 is a real-time G.722 codec load test, NOT a radio transmission test. */
typedef struct {
    uint32_t produced, consumed, encoded, decoded;
    uint32_t dropped, queue_high, errors, stopped, reason;
    uint32_t enc_avg_us, enc_max_us, dec_avg_us, dec_max_us, run_max_us;
    uint32_t bytes_total, payload_bps, modeled_wire_bps;
    uint32_t packet_bytes, modeled_overhead_bytes;
} S17_G722LiveStats;
int s17_g722_live_init(void);
void s17_g722_live_push(const int16_t *block160);
void s17_g722_live_run_one(void);
void s17_g722_live_stop(uint32_t reason);
void s17_g722_live_stats(S17_G722LiveStats *out);
#endif
