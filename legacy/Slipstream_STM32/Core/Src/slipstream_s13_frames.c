#include "slipstream_s13_frames.h"
#include <stddef.h>
#include <string.h>

void s13_frames_reset(S13_FrameAssembler *a)
{
    if (a != NULL) memset(a, 0, sizeof(*a));
}

uint8_t s13_frames_push(S13_FrameAssembler *a, const int16_t *block160)
{
    if (a == NULL || block160 == NULL) return 0U;

    memcpy(&a->staging[a->fill_blocks * S13_BLOCK_SAMPLES],
           block160, S13_BLOCK_SAMPLES * sizeof(int16_t));
    a->fill_blocks++;
    if (a->fill_blocks < 2U) return 0U;
    a->fill_blocks = 0U;

    if (a->count == S13_FRAME_QUEUE_CAPACITY)
    {
        /* Fresh intercom audio is preferable to an unbounded backlog. */
        a->read_index = (a->read_index + 1U) % S13_FRAME_QUEUE_CAPACITY;
        a->count--;
        a->dropped_oldest++;
    }
    memcpy(a->pending[a->write_index], a->staging,
           S13_FRAME_SAMPLES * sizeof(int16_t));
    a->write_index = (a->write_index + 1U) % S13_FRAME_QUEUE_CAPACITY;
    a->count++;
    a->completed_frames++;
    return 1U;
}

uint8_t s13_frames_take(S13_FrameAssembler *a, int16_t *frame320)
{
    if (a == NULL || frame320 == NULL || a->count == 0U) return 0U;
    memcpy(frame320, a->pending[a->read_index],
           S13_FRAME_SAMPLES * sizeof(int16_t));
    a->read_index = (a->read_index + 1U) % S13_FRAME_QUEUE_CAPACITY;
    a->count--;
    a->consumed_frames++;
    return 1U;
}
