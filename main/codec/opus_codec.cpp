#include "opus_codec.hpp"

#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"

#include "opus.h"


namespace {

constexpr const char *TAG =
    "opus_codec";

}


namespace slipstream {

OpusCodec::OpusCodec()
{
}


OpusCodec::~OpusCodec()
{
    if (
        encoder_ != nullptr
    ) {

        opus_encoder_destroy(
            static_cast<OpusEncoder *>(
                encoder_
            )
        );

        encoder_ =
            nullptr;
    }


    if (
        decoder_ != nullptr
    ) {

        opus_decoder_destroy(
            static_cast<OpusDecoder *>(
                decoder_
            )
        );

        decoder_ =
            nullptr;
    }
}


bool OpusCodec::init()
{
    if (
        initialized_
    ) {
        return true;
    }


    int error =
        OPUS_OK;


    OpusEncoder *encoder =
        opus_encoder_create(
            SAMPLE_RATE,
            CHANNELS,
            OPUS_APPLICATION_VOIP,
            &error
        );


    if (
        encoder == nullptr ||
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "opus_encoder_create failed: %d",
            error
        );

        return false;
    }


    encoder_ =
        encoder;


    error =
        opus_encoder_ctl(
            encoder,
            OPUS_SET_BITRATE(
                BITRATE
            )
        );


    if (
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "OPUS_SET_BITRATE failed: %d",
            error
        );

        return false;
    }


    error =
        opus_encoder_ctl(
            encoder,
            OPUS_SET_COMPLEXITY(
                COMPLEXITY
            )
        );


    if (
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "OPUS_SET_COMPLEXITY failed: %d",
            error
        );

        return false;
    }


    error =
        opus_encoder_ctl(
            encoder,
            OPUS_SET_VBR(
                1
            )
        );


    if (
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "OPUS_SET_VBR failed: %d",
            error
        );

        return false;
    }


    error =
        opus_encoder_ctl(
            encoder,
            OPUS_SET_INBAND_FEC(
                0
            )
        );


    if (
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "OPUS_SET_INBAND_FEC failed: %d",
            error
        );

        return false;
    }


    error =
        opus_encoder_ctl(
            encoder,
            OPUS_SET_DTX(
                0
            )
        );


    if (
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "OPUS_SET_DTX failed: %d",
            error
        );

        return false;
    }


    OpusDecoder *decoder =
        opus_decoder_create(
            SAMPLE_RATE,
            CHANNELS,
            &error
        );


    if (
        decoder == nullptr ||
        error != OPUS_OK
    ) {

        ESP_LOGE(
            TAG,
            "opus_decoder_create failed: %d",
            error
        );

        return false;
    }


    decoder_ =
        decoder;


    initialized_ =
        true;


    ESP_LOGI(
        TAG,
        "Opus initialized: "
        "%d Hz mono / "
        "%d samples / "
        "%d bps / complexity %d",
        SAMPLE_RATE,
        FRAME_SAMPLES,
        BITRATE,
        COMPLEXITY
    );


    return true;
}


void OpusCodec::reset()
{
    if (
        !initialized_
    ) {
        return;
    }


    opus_encoder_ctl(
        static_cast<OpusEncoder *>(
            encoder_
        ),
        OPUS_RESET_STATE
    );


    opus_decoder_ctl(
        static_cast<OpusDecoder *>(
            decoder_
        ),
        OPUS_RESET_STATE
    );


    std::memset(
        encoded_buffer_,
        0,
        sizeof(
            encoded_buffer_
        )
    );
}


int OpusCodec::encode_frame(
    const int16_t *input_pcm,
    uint8_t *output_packet,
    size_t output_capacity,
    uint32_t &encode_us
)
{
    encode_us =
        0;


    if (
        !initialized_ ||
        input_pcm == nullptr ||
        output_packet == nullptr ||
        output_capacity == 0
    ) {

        return
            OPUS_BAD_ARG;
    }


    int64_t start =
        esp_timer_get_time();


    int encoded_bytes =
        opus_encode(
            static_cast<OpusEncoder *>(
                encoder_
            ),
            input_pcm,
            FRAME_SAMPLES,
            output_packet,
            static_cast<opus_int32>(
                output_capacity
            )
        );


    encode_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            start
        );


    if (
        encoded_bytes < 0
    ) {

        ESP_LOGE(
            TAG,
            "opus_encode failed: %d (%s)",
            encoded_bytes,
            opus_strerror(
                encoded_bytes
            )
        );
    }


    return
        encoded_bytes;
}


int OpusCodec::decode_frame(
    const uint8_t *packet,
    size_t packet_length,
    int16_t *output_pcm,
    uint32_t &decode_us
)
{
    decode_us =
        0;


    if (
        !initialized_ ||
        packet == nullptr ||
        packet_length == 0 ||
        output_pcm == nullptr
    ) {

        return
            OPUS_BAD_ARG;
    }


    int64_t start =
        esp_timer_get_time();


    int decoded_samples =
        opus_decode(
            static_cast<OpusDecoder *>(
                decoder_
            ),
            packet,
            static_cast<opus_int32>(
                packet_length
            ),
            output_pcm,
            FRAME_SAMPLES,
            0
        );


    decode_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            start
        );


    if (
        decoded_samples < 0
    ) {

        ESP_LOGE(
            TAG,
            "opus_decode failed: %d (%s)",
            decoded_samples,
            opus_strerror(
                decoded_samples
            )
        );
    }


    return
        decoded_samples;
}


int OpusCodec::decode_plc(
    int16_t *output_pcm,
    uint32_t &decode_us
)
{
    decode_us =
        0;


    if (
        !initialized_ ||
        output_pcm == nullptr
    ) {

        return
            OPUS_BAD_ARG;
    }


    int64_t start =
        esp_timer_get_time();


    /*
     * Standard Opus packet-loss concealment:
     *
     *     packet = nullptr
     *     length = 0
     */
    int decoded_samples =
        opus_decode(
            static_cast<OpusDecoder *>(
                decoder_
            ),
            nullptr,
            0,
            output_pcm,
            FRAME_SAMPLES,
            0
        );


    decode_us =
        static_cast<uint32_t>(
            esp_timer_get_time() -
            start
        );


    if (
        decoded_samples < 0
    ) {

        ESP_LOGE(
            TAG,
            "Opus PLC failed: %d (%s)",
            decoded_samples,
            opus_strerror(
                decoded_samples
            )
        );
    }


    return
        decoded_samples;
}


bool OpusCodec::process_frame(
    const int16_t *input_pcm,
    int16_t *output_pcm,
    OpusCodecStats &stats
)
{
    stats = {};


    int encoded_bytes =
        encode_frame(
            input_pcm,
            encoded_buffer_,
            sizeof(
                encoded_buffer_
            ),
            stats.encode_us
        );


    stats.encoded_bytes =
        encoded_bytes;


    if (
        encoded_bytes < 0
    ) {
        return false;
    }


    stats.encode_ok =
        true;


    int decoded_samples =
        decode_frame(
            encoded_buffer_,
            static_cast<size_t>(
                encoded_bytes
            ),
            output_pcm,
            stats.decode_us
        );


    stats.decoded_samples =
        decoded_samples;


    if (
        decoded_samples < 0
    ) {
        return false;
    }


    stats.decode_ok =
        true;


    return
        decoded_samples ==
        FRAME_SAMPLES;
}

}