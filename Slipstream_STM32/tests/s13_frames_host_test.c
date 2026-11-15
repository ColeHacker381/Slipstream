/* Run from Slipstream_STM32:
   cc -std=c11 -Wall -Wextra -Werror -ICore/Inc tests/s13_frames_host_test.c Core/Src/slipstream_s13_frames.c -o /tmp/s13frames && /tmp/s13frames
*/
#include "slipstream_s13_frames.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>

int main(void)
{
    S13_FrameAssembler q;
    int16_t a[160], b[160], out[320];
    s13_frames_reset(&q);
    for (unsigned i = 0U; i < 160U; i++) { a[i] = (int16_t)i; b[i] = (int16_t)(i + 160U); }
    assert(s13_frames_push(&q, a) == 0U);
    assert(s13_frames_take(&q, out) == 0U);
    assert(s13_frames_push(&q, b) == 1U);
    assert(s13_frames_take(&q, out) == 1U);
    for (unsigned i = 0U; i < 320U; i++) assert(out[i] == (int16_t)i);
    assert(q.completed_frames == 1U && q.consumed_frames == 1U);
    /* Every frame including silence must be produced. */
    memset(a, 0, sizeof(a)); memset(b, 0, sizeof(b));
    assert(s13_frames_push(&q, a) == 0U);
    assert(s13_frames_push(&q, b) == 1U);
    assert(s13_frames_take(&q, out) == 1U);
    for (unsigned i = 0U; i < 320U; i++) assert(out[i] == 0);
    /* Full queue drops oldest, never builds unbounded latency. */
    for (unsigned frame = 0U; frame < 3U; frame++) {
        for (unsigned i = 0U; i < 160U; i++) {
            a[i] = (int16_t)(frame + 10U);
            b[i] = (int16_t)(frame + 10U);
        }
        assert(s13_frames_push(&q, a) == 0U);
        assert(s13_frames_push(&q, b) == 1U);
    }
    assert(q.dropped_oldest == 1U && q.count == 2U);
    assert(s13_frames_take(&q, out) == 1U && out[0] == 11);
    assert(s13_frames_take(&q, out) == 1U && out[0] == 12);
    assert(s13_frames_take(&q, out) == 0U);
    return 0;
}
