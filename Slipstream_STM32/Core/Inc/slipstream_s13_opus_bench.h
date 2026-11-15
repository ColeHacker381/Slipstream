#ifndef SLIPSTREAM_S13_OPUS_BENCH_H
#define SLIPSTREAM_S13_OPUS_BENCH_H

#include <stdint.h>

typedef struct {
    uint32_t encoder_bytes;
    uint32_t decoder_bytes;
    uint32_t encode_avg_us;
    uint32_t encode_max_us;
    uint32_t decode_avg_us;
    uint32_t decode_max_us;
    uint32_t total_avg_us;
    uint32_t total_max_us;
    uint32_t encoded_avg_bytes;
    uint32_t errors;
    uint32_t frames;
} S13_OpusBenchResult;

/* Requires valid 32-bit DWT->CYCCNT + SystemCoreClock; before live DMA.
   Returns 1 on success. Allocates and frees Opus states at startup only. */
int s13_opus_benchmark(S13_OpusBenchResult *out);

#endif
