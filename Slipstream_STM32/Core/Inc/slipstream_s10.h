#ifndef SLIPSTREAM_S10_H
#define SLIPSTREAM_S10_H

#include <stdint.h>

#define S10_BLOCK_SIZE 160U

typedef struct
{
    float input_dbfs;
    float output_dbfs;
    float target_gain_db;
    float applied_gain_db;
    uint32_t clipped_samples;
} S10_AgcResult;

/* ESP32 H10 behavior, 16 kHz signed PCM / 160 samples per block. */
void s10_reset(void);
/* Optional S12.6 experiment: positive AGC gain returns faster after
   voice ends. Pass 0 to retain original 2.0-second behavior. */
void s10_set_fast_positive_idle(uint8_t enabled);
uint8_t s10_fast_positive_idle_enabled(void);
S10_AgcResult s10_process_block(const int16_t *input,
                                int16_t *output,
                                uint32_t samples,
                                uint8_t speech_active);
float s10_current_gain_db(void);

#endif
