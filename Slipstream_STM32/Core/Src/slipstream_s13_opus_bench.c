/*
 * Slipstream S13 — standalone REAL libopus benchmark on STM32F412.
 * This intentionally runs once at boot BEFORE I2S DMA starts.
 * It does not claim to implement real-time encode/decode yet.
 */
#include "slipstream_s13_opus_bench.h"
#include "main.h"
#include "opus.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define S13_SAMPLE_RATE 16000
#define S13_FRAME_SAMPLES 320
#define S13_FRAME_COUNT 50U
#define S13_WARMUP_COUNT 5U
#define S13_PACKET_MAX 512

static int16_t s13_input[S13_FRAME_SAMPLES];
static int16_t s13_decoded[S13_FRAME_SAMPLES];
static uint8_t s13_packet[S13_PACKET_MAX];

static uint32_t cycles_to_us(uint32_t cycles)
{
    if (SystemCoreClock == 0U) return 0U;
    return (uint32_t)(((uint64_t)cycles * 1000000ULL) /
                      (uint64_t)SystemCoreClock);
}

int s13_opus_benchmark(S13_OpusBenchResult *out)
{
    if (out == NULL || SystemCoreClock == 0U) return 0;
    memset(out, 0, sizeof(*out));

    int enc_size = opus_encoder_get_size(1);
    int dec_size = opus_decoder_get_size(1);
    if (enc_size <= 0 || dec_size <= 0) { out->errors++; return 0; }
    out->encoder_bytes = (uint32_t)enc_size;
    out->decoder_bytes = (uint32_t)dec_size;

    /* Startup-only allocation: no malloc in real-time DSP path. */
    OpusEncoder *encoder = (OpusEncoder *)malloc((size_t)enc_size);
    OpusDecoder *decoder = (OpusDecoder *)malloc((size_t)dec_size);
    if (encoder == NULL || decoder == NULL)
    {
        out->errors++;
        free(encoder);
        free(decoder);
        return 0;
    }
    int rc = opus_encoder_init(encoder, S13_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP);
    if (rc == OPUS_OK) rc = opus_decoder_init(decoder, S13_SAMPLE_RATE, 1);
    if (rc == OPUS_OK) rc = opus_encoder_ctl(encoder, OPUS_SET_BITRATE(20000));
    if (rc == OPUS_OK) rc = opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(5));
    if (rc == OPUS_OK) rc = opus_encoder_ctl(encoder, OPUS_SET_VBR(1));
    if (rc == OPUS_OK) rc = opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(0));
    if (rc == OPUS_OK) rc = opus_encoder_ctl(encoder, OPUS_SET_DTX(0));
    if (rc != OPUS_OK)
    {
        out->errors++;
        free(encoder);
        free(decoder);
        return 0;
    }

    /* Varying speech-band synthetic input, no zero-only fast path.
       This is a preliminary CPU measurement, not a worst-case bound. */
    uint32_t phase = 0U;
    uint64_t encode_sum = 0U, decode_sum = 0U;
    uint64_t size_sum = 0U;
    for (uint32_t frame = 0U; frame < S13_FRAME_COUNT + S13_WARMUP_COUNT; frame++)
    {
        for (uint32_t i = 0U; i < S13_FRAME_SAMPLES; i++)
        {
            phase += 271U + ((frame & 3U) * 47U);
            int32_t tri = (int32_t)((phase >> 8) & 0x7FFFU) - 16384;
            s13_input[i] = (int16_t)(tri / 3);
        }

        uint32_t start = DWT->CYCCNT;
        opus_int32 bytes = opus_encode(encoder, s13_input, S13_FRAME_SAMPLES,
                                      s13_packet, S13_PACKET_MAX);
        uint32_t enc_cycles = DWT->CYCCNT - start;
        if (bytes <= 0 || bytes > S13_PACKET_MAX)
        {
            out->errors++;
            break;
        }

        start = DWT->CYCCNT;
        int samples = opus_decode(decoder, s13_packet, bytes, s13_decoded,
                                  S13_FRAME_SAMPLES, 0);
        uint32_t dec_cycles = DWT->CYCCNT - start;
        if (samples != S13_FRAME_SAMPLES)
        {
            out->errors++;
            break;
        }

        if (frame < S13_WARMUP_COUNT) continue;
        uint32_t enc_us = cycles_to_us(enc_cycles);
        uint32_t dec_us = cycles_to_us(dec_cycles);
        uint32_t total_us = enc_us + dec_us;
        encode_sum += enc_us;
        decode_sum += dec_us;
        size_sum += (uint32_t)bytes;
        if (enc_us > out->encode_max_us) out->encode_max_us = enc_us;
        if (dec_us > out->decode_max_us) out->decode_max_us = dec_us;
        if (total_us > out->total_max_us) out->total_max_us = total_us;
        out->frames++;
    }

    if (out->frames > 0U)
    {
        out->encode_avg_us = (uint32_t)(encode_sum / out->frames);
        out->decode_avg_us = (uint32_t)(decode_sum / out->frames);
        out->total_avg_us = (uint32_t)((encode_sum + decode_sum) / out->frames);
        out->encoded_avg_bytes = (uint32_t)(size_sum / out->frames);
    }
    free(encoder);
    free(decoder);
    return out->errors == 0U && out->frames == S13_FRAME_COUNT;
}
