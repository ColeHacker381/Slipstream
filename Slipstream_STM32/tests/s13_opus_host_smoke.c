/*
 * Native libopus functional smoke test; NOT STM32 CPU benchmark.
 * Requires libopus headers and development library:
 *   cc -O2 -Wall -Wextra -Werror tests/s13_opus_host_smoke.c \
 *      $(pkg-config --cflags --libs opus) -lm -o /tmp/s13_opus_host_smoke
 *   /tmp/s13_opus_host_smoke
 *
 * Checks the frozen RF Opus format (16k mono/20ms/20kbps VOIP),
 * continuous encode incl. silence, FEC=0 and DTX=0.
 */
#include <opus/opus.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define S13_FRAME 320
#define S13_MAX_BYTES 512
#define S13_TEST_FRAMES 250

int main(void)
{
    int error = OPUS_OK;
    OpusEncoder *enc = opus_encoder_create(16000, 1, OPUS_APPLICATION_VOIP, &error);
    if (!enc || error != OPUS_OK) { fprintf(stderr, "Opus encoder create failed: %d\n", error); return 1; }
    OpusDecoder *dec = opus_decoder_create(16000, 1, &error);
    if (!dec || error != OPUS_OK) { fprintf(stderr, "Opus decoder create failed: %d\n", error); opus_encoder_destroy(enc); return 1; }

    int ok = 1;
    ok &= opus_encoder_ctl(enc, OPUS_SET_BITRATE(20000)) == OPUS_OK;
    ok &= opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(5)) == OPUS_OK;
    ok &= opus_encoder_ctl(enc, OPUS_SET_VBR(1)) == OPUS_OK;
    ok &= opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(0)) == OPUS_OK;
    ok &= opus_encoder_ctl(enc, OPUS_SET_DTX(0)) == OPUS_OK;
    if (!ok) { fprintf(stderr, "Opus config failed\n"); return 2; }

    opus_int16 in[S13_FRAME], out[S13_FRAME];
    unsigned char payload[S13_MAX_BYTES];
    long total_bytes = 0;
    int max_bytes = 0;
    for (int frame = 0; frame < S13_TEST_FRAMES; ++frame)
    {
        for (int i = 0; i < S13_FRAME; ++i)
        {
            /* 1 s speechlike sine, 1 s silence, then speech again.
               The silence second must still create valid packets. */
            const int silent = (frame >= 50 && frame < 100);
            const float t = (float)(frame * S13_FRAME + i) / 16000.0f;
            in[i] = silent ? 0 : (opus_int16)(9000.0f * sinf(2.0f * 3.14159265f * 220.0f * t));
        }
        const int bytes = opus_encode(enc, in, S13_FRAME, payload, S13_MAX_BYTES);
        if (bytes <= 0 || bytes > S13_MAX_BYTES)
        {
            fprintf(stderr, "Opus frame %d encode failed: %d\n", frame, bytes);
            return 3;
        }
        const int samples = opus_decode(dec, payload, bytes, out, S13_FRAME, 0);
        if (samples != S13_FRAME)
        {
            fprintf(stderr, "Opus frame %d decode failed: %d\n", frame, samples);
            return 4;
        }
        total_bytes += bytes;
        if (bytes > max_bytes) max_bytes = bytes;
    }
    printf("Opus host smoke PASS: %d continuous frames, average %.1f bytes, max %d bytes\n",
           S13_TEST_FRAMES, (double)total_bytes / S13_TEST_FRAMES, max_bytes);
    printf("NOTE: host timing is NOT STM32 benchmark. Target DWT cycle instrumentation remains pending.\n");
    opus_decoder_destroy(dec);
    opus_encoder_destroy(enc);
    return 0;
}
