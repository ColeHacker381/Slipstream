#pragma once

#include <cstddef>
#include <cstdint>

namespace slipstream {
namespace transport {

static constexpr uint8_t PACKET_VERSION =
    1;

static constexpr size_t HEADER_SIZE =
    16;

static constexpr size_t CRC_SIZE =
    4;

static constexpr size_t MAX_PAYLOAD_SIZE =
    512;

static constexpr size_t MAX_PACKET_SIZE =
    HEADER_SIZE +
    MAX_PAYLOAD_SIZE +
    CRC_SIZE;


/*
 * Packet flags.
 */
enum PacketFlags : uint8_t {

    FLAG_NONE =
        0,

    FLAG_SPEECH =
        1 << 0,

    FLAG_FEC =
        1 << 1,

    FLAG_CONTROL =
        1 << 2
};


struct PacketHeader {

    uint8_t version =
        PACKET_VERSION;

    uint8_t flags =
        FLAG_NONE;

    uint8_t stream_id =
        0;

    uint8_t channel =
        0;

    uint16_t sequence =
        0;

    uint32_t timestamp =
        0;

    uint16_t payload_length =
        0;

    uint16_t reserved =
        0;
};


struct ParsedPacket {

    PacketHeader header = {};

    const uint8_t *payload =
        nullptr;

    size_t payload_length =
        0;

    uint32_t received_crc =
        0;

    uint32_t calculated_crc =
        0;
};


enum class ParseResult {

    OK,

    NULL_ARGUMENT,

    TOO_SHORT,

    BAD_MAGIC,

    BAD_VERSION,

    BAD_LENGTH,

    BAD_CRC
};


uint32_t crc32(
    const uint8_t *data,
    size_t length
);


bool build_packet(
    const PacketHeader &header,
    const uint8_t *payload,
    size_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t &output_length
);


ParseResult parse_packet(
    const uint8_t *packet,
    size_t packet_length,
    ParsedPacket &parsed
);


const char *parse_result_string(
    ParseResult result
);

}
}