#ifndef SLIPSTREAM_S14_SPEEX_BENCH_H
#define SLIPSTREAM_S14_SPEEX_BENCH_H
#include <stdint.h>
#ifndef S14_SPEEX_COMPLEXITY
#define S14_SPEEX_COMPLEXITY 1
#endif
#ifndef S14_SPEEX_QUALITY
#define S14_SPEEX_QUALITY 6
#endif
typedef struct {
    uint32_t ok, frames, errors, frame_samples, target_bitrate;
    uint32_t encode_avg_us, encode_max_us;
    uint32_t decode_avg_us, decode_max_us;
    uint32_t total_avg_us, total_max_us;
    uint32_t packet_avg_bytes, packet_max_bytes;
} S14_SpeexBenchResult;
int s14_speex_benchmark(S14_SpeexBenchResult *result);
#endif
