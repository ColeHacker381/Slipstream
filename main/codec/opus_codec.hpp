#pragma once

#include <cstddef>
#include <cstdint>

namespace slipstream {

struct OpusCodecStats {

    int encoded_bytes = 0;

    uint32_t encode_us = 0;

    uint32_t decode_us = 0;

    int decoded_samples = 0;

    bool encode_ok = false;

    bool decode_ok = false;
};


class OpusCodec {
public:

    static constexpr int SAMPLE_RATE =
        16000;

    static constexpr int CHANNELS =
        1;

    static constexpr int FRAME_SAMPLES =
        320;

    static constexpr int BITRATE =
        20000;

    static constexpr int COMPLEXITY =
        5;

    static constexpr size_t MAX_PACKET_BYTES =
        512;


    OpusCodec();

    ~OpusCodec();


    bool init();

    void reset();


    int encode_frame(
        const int16_t *input_pcm,
        uint8_t *output_packet,
        size_t output_capacity,
        uint32_t &encode_us
    );


    int decode_frame(
        const uint8_t *packet,
        size_t packet_length,
        int16_t *output_pcm,
        uint32_t &decode_us
    );


    /*
     * H14:
     *
     * Generate one 20 ms concealment frame using
     * the decoder's internal packet-loss state.
     *
     * Opus PLC is requested by decoding:
     *
     *     data = nullptr
     *     len  = 0
     */
    int decode_plc(
        int16_t *output_pcm,
        uint32_t &decode_us
    );


    bool process_frame(
        const int16_t *input_pcm,
        int16_t *output_pcm,
        OpusCodecStats &stats
    );


    bool initialized() const
    {
        return initialized_;
    }


private:

    void *encoder_ =
        nullptr;

    void *decoder_ =
        nullptr;


    uint8_t encoded_buffer_[
        MAX_PACKET_BYTES
    ] = {};


    bool initialized_ =
        false;
};

}