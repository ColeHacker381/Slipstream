#ifndef SLIPSTREAM_S13_FRAMES_H
#define SLIPSTREAM_S13_FRAMES_H

#include <stdint.h>

#define S13_BLOCK_SAMPLES 160U
#define S13_FRAME_SAMPLES 320U
#define S13_FRAME_QUEUE_CAPACITY 2U

/* 10ms DSP producer -> bounded 20ms Opus-frame consumer.
   No heap, no VAD gating, and no changes to the 10ms DMA cadence.
   Not interrupt-safe: producer and consumer run in main context. */
typedef struct {
    int16_t staging[S13_FRAME_SAMPLES];
    int16_t pending[S13_FRAME_QUEUE_CAPACITY][S13_FRAME_SAMPLES];
    uint32_t fill_blocks;
    uint32_t read_index;
    uint32_t write_index;
    uint32_t count;
    uint32_t completed_frames;
    uint32_t dropped_oldest;
    uint32_t consumed_frames;
} S13_FrameAssembler;

void s13_frames_reset(S13_FrameAssembler *a);
/* Always pushes the 10ms block regardless of speech/VAD state.
   At full capacity the OLDEST 20ms frame is dropped to bound latency.
   Returns 1 when a new complete frame was assembled. */
uint8_t s13_frames_push(S13_FrameAssembler *a, const int16_t *block160);
/* Returns 1 and copies one 320-sample frame when available. */
uint8_t s13_frames_take(S13_FrameAssembler *a, int16_t *frame320);

#endif
