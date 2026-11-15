#ifndef SLIPSTREAM_S9_H
#define SLIPSTREAM_S9_H

#include <stdint.h>

#define S9_BLOCK_SIZE 160U

typedef struct {
    uint8_t speech_active;
    uint8_t speech_started;
    uint8_t speech_ended;
    uint8_t noise_estimate_ready;
    float level_dbfs;
    float noise_dbfs;
} S9_VadResult;

void s9_reset(void);
S9_VadResult s9_process_block(const int16_t *samples, uint32_t count);
uint8_t s9_noise_estimate_ready(void);

#endif
