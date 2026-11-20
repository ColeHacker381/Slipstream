#ifndef SLIPSTREAM_S16_CODEC2_BENCH_H
#define SLIPSTREAM_S16_CODEC2_BENCH_H
#include <stdint.h>
typedef struct {
    uint32_t frames, errors, samples8k, packet_bytes;
    uint32_t down_avg_us, down_max_us;
    uint32_t enc_avg_us, enc_max_us;
    uint32_t dec_avg_us, dec_max_us;
    uint32_t up_avg_us, up_max_us;
    uint32_t total_avg_us, total_max_us;
} S16_Codec2BenchResult;
int s16_codec2_benchmark(S16_Codec2BenchResult *out);
#endif
