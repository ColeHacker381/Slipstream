#ifndef SLIPSTREAM_S8_H
#define SLIPSTREAM_S8_H

#include <stdint.h>

#define S8_BLOCK_SIZE 160U
#define S8_FRAME_SIZE 320U
#define S8_FFT_SIZE 512U

/* Returns 1 after successful initialization, 0 otherwise. */
int s8_init(void);
void s8_reset(void);

/* Process precisely 160 signed 16-bit mono samples.
   Input/output must not overlap. */
void s8_process(const int16_t input[S8_BLOCK_SIZE],
                int16_t output[S8_BLOCK_SIZE]);

/* Noise estimator becomes ready after 50 successful input blocks. */
uint32_t s8_noise_ready(void);
uint32_t s8_learning_blocks(void);

#endif
