#ifndef SLIPSTREAM_S11_H
#define SLIPSTREAM_S11_H

#include <stdint.h>

#define S11_BLOCK_SIZE 160U
#define S11_LOOKAHEAD_SAMPLES 16U

typedef struct
{
    float input_peak_dbfs;
    float output_peak_dbfs;
    float gain_reduction_db;
    uint32_t threshold_hits;
    uint32_t safety_clamps;
} S11_LimiterResult;

void s11_reset(void);
S11_LimiterResult s11_process_block(const int16_t *input,
                                    int16_t *output,
                                    uint32_t samples);
float s11_current_gain_db(void);
#endif
