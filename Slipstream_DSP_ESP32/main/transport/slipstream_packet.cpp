#include "slipstream_packet.hpp"

namespace {

constexpr uint8_t MAGIC_0 =
    'S';

constexpr uint8_t MAGIC_1 =
    'L';


void write_u16_be(
    uint8_t *destination,
    uint16_t value
)
{
    destination[0] =
        static_cast<uint8_t>(
            (value >> 8) &
            0xFF
        );

    destination[1] =
        static_cast<uint8_t>(
            value &
            0xFF
        );
}


void write_u32_be(
    uint8_t *destination,
    uint32_t value
)
{
    destination[0] =
        static_cast<uint8_t>(
            (value >> 24) &
            0xFF
        );

    destination[1] =
        static_cast<uint8_t>(
            (value >> 16) &
            0xFF
        );

    destination[2] =
        static_cast<uint8_t>(
            (value >> 8) &
            0xFF
        );

    destination[3] =
        static_cast<uint8_t>(
            value &
            0xFF
        );
}


uint16_t read_u16_be(
    const uint8_t *source
)
{
    return
        static_cast<uint16_t>(
            (
                static_cast<uint16_t>(
                    source[0]
                )
                << 8
            )
            |
            static_cast<uint16_t>(
                source[1]
            )
        );
}


uint32_t read_u32_be(
    const uint8_t *source
)
{
    return
        (
            static_cast<uint32_t>(
                source[0]
            )
            << 24
        )
        |
        (
            static_cast<uint32_t>(
                source[1]
            )
            << 16
        )
        |
        (
            static_cast<uint32_t>(
                source[2]
            )
            << 8
        )
        |
        static_cast<uint32_t>(
            source[3]
        );
}

}


namespace slipstream {
namespace transport {

uint32_t crc32(
    const uint8_t *data,
    size_t length
)
{
    if (
        data == nullptr &&
        length != 0
    ) {
        return 0;
    }


    uint32_t crc =
        0xFFFFFFFFu;


    for (
        size_t i = 0;
        i < length;
        ++i
    ) {

        crc ^=
            static_cast<uint32_t>(
                data[i]
            );


        for (
            int bit = 0;
            bit < 8;
            ++bit
        ) {

            if (
                crc &
                1u
            ) {

                crc =
                    (
                        crc >>
                        1
                    )
                    ^
                    0xEDB88320u;
            }
            else {

                crc >>=
                    1;
            }
        }
    }


    return
        crc ^
        0xFFFFFFFFu;
}


bool build_packet(
    const PacketHeader &header,
    const uint8_t *payload,
    size_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t &output_length
)
{
    output_length =
        0;


    if (
        output == nullptr
    ) {
        return false;
    }


    if (
        payload_length >
        MAX_PAYLOAD_SIZE
    ) {
        return false;
    }


    if (
        payload_length != 0 &&
        payload == nullptr
    ) {
        return false;
    }


    const size_t required_size =
        HEADER_SIZE +
        payload_length +
        CRC_SIZE;


    if (
        output_capacity <
        required_size
    ) {
        return false;
    }


    /*
     * -----------------------------------------
     * Header.
     * -----------------------------------------
     */

    output[0] =
        MAGIC_0;

    output[1] =
        MAGIC_1;

    output[2] =
        header.version;

    output[3] =
        header.flags;

    output[4] =
        header.stream_id;

    output[5] =
        header.channel;


    write_u16_be(
        &output[6],
        header.sequence
    );


    write_u32_be(
        &output[8],
        header.timestamp
    );


    write_u16_be(
        &output[12],
        static_cast<uint16_t>(
            payload_length
        )
    );


    write_u16_be(
        &output[14],
        header.reserved
    );


    /*
     * -----------------------------------------
     * Payload.
     * -----------------------------------------
     */

    for (
        size_t i = 0;
        i < payload_length;
        ++i
    ) {

        output[
            HEADER_SIZE + i
        ] =
            payload[i];
    }


    /*
     * -----------------------------------------
     * CRC32.
     *
     * CRC covers:
     *
     * header + payload
     *
     * but not the CRC field itself.
     * -----------------------------------------
     */

    const uint32_t crc =
        crc32(
            output,
            HEADER_SIZE +
            payload_length
        );


    write_u32_be(
        &output[
            HEADER_SIZE +
            payload_length
        ],
        crc
    );


    output_length =
        required_size;


    return true;
}


ParseResult parse_packet(
    const uint8_t *packet,
    size_t packet_length,
    ParsedPacket &parsed
)
{
    parsed = {};


    if (
        packet == nullptr
    ) {
        return
            ParseResult::
                NULL_ARGUMENT;
    }


    if (
        packet_length <
        HEADER_SIZE +
        CRC_SIZE
    ) {

        return
            ParseResult::
                TOO_SHORT;
    }


    /*
     * -----------------------------------------
     * Magic.
     * -----------------------------------------
     */

    if (
        packet[0] !=
            MAGIC_0 ||
        packet[1] !=
            MAGIC_1
    ) {

        return
            ParseResult::
                BAD_MAGIC;
    }


    /*
     * -----------------------------------------
     * Version.
     * -----------------------------------------
     */

    if (
        packet[2] !=
        PACKET_VERSION
    ) {

        return
            ParseResult::
                BAD_VERSION;
    }


    /*
     * -----------------------------------------
     * Read payload length.
     * -----------------------------------------
     */

    const uint16_t payload_length =
        read_u16_be(
            &packet[12]
        );


    if (
        payload_length >
        MAX_PAYLOAD_SIZE
    ) {

        return
            ParseResult::
                BAD_LENGTH;
    }


    const size_t expected_length =
        HEADER_SIZE +
        static_cast<size_t>(
            payload_length
        )
        +
        CRC_SIZE;


    if (
        packet_length !=
        expected_length
    ) {

        return
            ParseResult::
                BAD_LENGTH;
    }


    /*
     * -----------------------------------------
     * CRC.
     * -----------------------------------------
     */

    const uint32_t received_crc =
        read_u32_be(
            &packet[
                HEADER_SIZE +
                payload_length
            ]
        );


    const uint32_t calculated_crc =
        crc32(
            packet,
            HEADER_SIZE +
            payload_length
        );


    if (
        received_crc !=
        calculated_crc
    ) {

        return
            ParseResult::
                BAD_CRC;
    }


    /*
     * -----------------------------------------
     * Populate parsed packet.
     * -----------------------------------------
     */

    parsed.header.version =
        packet[2];


    parsed.header.flags =
        packet[3];


    parsed.header.stream_id =
        packet[4];


    parsed.header.channel =
        packet[5];


    parsed.header.sequence =
        read_u16_be(
            &packet[6]
        );


    parsed.header.timestamp =
        read_u32_be(
            &packet[8]
        );


    parsed.header.payload_length =
        payload_length;


    parsed.header.reserved =
        read_u16_be(
            &packet[14]
        );


    parsed.payload =
        &packet[
            HEADER_SIZE
        ];


    parsed.payload_length =
        payload_length;


    parsed.received_crc =
        received_crc;


    parsed.calculated_crc =
        calculated_crc;


    return
        ParseResult::OK;
}


const char *parse_result_string(
    ParseResult result
)
{
    switch (
        result
    ) {

        case ParseResult::OK:
            return "OK";


        case ParseResult::
            NULL_ARGUMENT:

            return
                "NULL_ARGUMENT";


        case ParseResult::
            TOO_SHORT:

            return
                "TOO_SHORT";


        case ParseResult::
            BAD_MAGIC:

            return
                "BAD_MAGIC";


        case ParseResult::
            BAD_VERSION:

            return
                "BAD_VERSION";


        case ParseResult::
            BAD_LENGTH:

            return
                "BAD_LENGTH";


        case ParseResult::
            BAD_CRC:

            return
                "BAD_CRC";
    }


    return
        "UNKNOWN";
}

}
}