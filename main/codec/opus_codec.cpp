#include "opus_codec.hpp"
#include <cstring>
#include "esp_log.h"
#include "esp_timer.h"
#include "opus.h"

namespace {
constexpr const char *TAG = "opus_codec";
}

namespace slipstream {
OpusCodec::~OpusCodec() {
    if (encoder_) opus_encoder_destroy(static_cast<OpusEncoder *>(encoder_));
    if (decoder_) opus_decoder_destroy(static_cast<OpusDecoder *>(decoder_));
}

bool OpusCodec::init_encoder() {
    if (encoder_) return true;
    int error = OPUS_OK;
    OpusEncoder *enc = opus_encoder_create(SAMPLE_RATE, CHANNELS,
                                          OPUS_APPLICATION_VOIP, &error);
    if (!enc || error != OPUS_OK) {
        ESP_LOGE(TAG, "encoder allocation failed: %d", error);
        return false;
    }
    // Allocate once; destroy on *any* setup error rather than leaking memory.
    if ((error = opus_encoder_ctl(enc, OPUS_SET_BITRATE(BITRATE))) != OPUS_OK ||
        (error = opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(COMPLEXITY))) != OPUS_OK ||
        (error = opus_encoder_ctl(enc, OPUS_SET_VBR(1))) != OPUS_OK ||
        (error = opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(0))) != OPUS_OK ||
        (error = opus_encoder_ctl(enc, OPUS_SET_DTX(0))) != OPUS_OK) {
        ESP_LOGE(TAG, "encoder setup failed: %d", error);
        opus_encoder_destroy(enc);
        return false;
    }
    encoder_ = enc;
    ESP_LOGI(TAG, "Encoder: %d Hz mono, %d samples, %d bps, complexity %d",
             SAMPLE_RATE, FRAME_SAMPLES, BITRATE, COMPLEXITY);
    return true;
}

bool OpusCodec::init_decoder() {
    if (decoder_) return true;
    int error = OPUS_OK;
    OpusDecoder *dec = opus_decoder_create(SAMPLE_RATE, CHANNELS, &error);
    if (!dec || error != OPUS_OK) {
        ESP_LOGE(TAG, "decoder allocation failed: %d", error);
        return false;
    }
    decoder_ = dec;
    ESP_LOGI(TAG, "Decoder: %d Hz mono, %d samples", SAMPLE_RATE, FRAME_SAMPLES);
    return true;
}

bool OpusCodec::init() {
    // Old tests rely on both encode and decode in the same object.
    return init_encoder() && init_decoder();
}

void OpusCodec::reset() {
    if (encoder_) opus_encoder_ctl(static_cast<OpusEncoder *>(encoder_), OPUS_RESET_STATE);
    if (decoder_) opus_decoder_ctl(static_cast<OpusDecoder *>(decoder_), OPUS_RESET_STATE);
    std::memset(encoded_buffer_, 0, sizeof(encoded_buffer_));
}

int OpusCodec::encode_frame(const int16_t *input_pcm, uint8_t *output_packet,
                            size_t output_capacity, uint32_t &encode_us) {
    encode_us = 0;
    if (!encoder_ || !input_pcm || !output_packet || output_capacity == 0)
        return OPUS_BAD_ARG;
    const int64_t start = esp_timer_get_time();
    const int result = opus_encode(static_cast<OpusEncoder *>(encoder_),
                                   input_pcm, FRAME_SAMPLES, output_packet,
                                   static_cast<opus_int32>(output_capacity));
    encode_us = static_cast<uint32_t>(esp_timer_get_time() - start);
    if (result < 0) ESP_LOGE(TAG, "opus_encode failed: %d (%s)", result, opus_strerror(result));
    return result;
}

int OpusCodec::decode_frame(const uint8_t *packet, size_t packet_length,
                            int16_t *output_pcm, uint32_t &decode_us) {
    decode_us = 0;
    if (!decoder_ || !packet || packet_length == 0 || !output_pcm)
        return OPUS_BAD_ARG;
    const int64_t start = esp_timer_get_time();
    const int result = opus_decode(static_cast<OpusDecoder *>(decoder_),
                                   packet, static_cast<opus_int32>(packet_length),
                                   output_pcm, FRAME_SAMPLES, 0);
    decode_us = static_cast<uint32_t>(esp_timer_get_time() - start);
    if (result < 0) ESP_LOGE(TAG, "opus_decode failed: %d (%s)", result, opus_strerror(result));
    return result;
}

int OpusCodec::decode_plc(int16_t *output_pcm, uint32_t &decode_us) {
    decode_us = 0;
    if (!decoder_ || !output_pcm) return OPUS_BAD_ARG;
    const int64_t start = esp_timer_get_time();
    const int result = opus_decode(static_cast<OpusDecoder *>(decoder_),
                                   nullptr, 0, output_pcm, FRAME_SAMPLES, 0);
    decode_us = static_cast<uint32_t>(esp_timer_get_time() - start);
    if (result < 0) ESP_LOGE(TAG, "Opus PLC failed: %d (%s)", result, opus_strerror(result));
    return result;
}

bool OpusCodec::process_frame(const int16_t *input_pcm, int16_t *output_pcm,
                              OpusCodecStats &stats) {
    stats = {};
    if (!initialized()) return false;
    stats.encoded_bytes = encode_frame(input_pcm, encoded_buffer_,
                                        sizeof(encoded_buffer_), stats.encode_us);
    if (stats.encoded_bytes <= 0) return false;
    stats.encode_ok = true;
    stats.decoded_samples = decode_frame(encoded_buffer_,
        static_cast<size_t>(stats.encoded_bytes), output_pcm, stats.decode_us);
    stats.decode_ok = stats.decoded_samples == FRAME_SAMPLES;
    return stats.decode_ok;
}
}
