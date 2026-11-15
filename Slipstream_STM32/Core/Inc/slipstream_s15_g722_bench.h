#ifndef SLIPSTREAM_S15_G722_BENCH_H
#define SLIPSTREAM_S15_G722_BENCH_H
#include <stdint.h>
typedef struct {
  uint32_t frames, errors, enc_avg_us, enc_max_us, dec_avg_us, dec_max_us;
  uint32_t total_avg_us, total_max_us, avg_bytes, max_bytes;
} S15_G722Result;
int s15_g722_benchmark(S15_G722Result *out);
#endif
