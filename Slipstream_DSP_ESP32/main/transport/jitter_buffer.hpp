#pragma once

#include <cstddef>
#include <cstdint>

#include "transport/slipstream_packet.hpp"


namespace slipstream {
namespace transport {


enum class JitterInsertResult {

    OK,

    DUPLICATE,

    LATE,

    FULL,

    TOO_LARGE
};


class PacketJitterBuffer {
public:

    static constexpr size_t CAPACITY =
        8;


    PacketJitterBuffer();


    void reset();


    JitterInsertResult insert(
        const uint8_t *packet,
        size_t packet_length,
        uint16_t sequence
    );


    /*
     * Retrieve and remove exactly the requested
     * sequence number.
     *
     * Returns false when that packet is missing.
     */
    bool take(
        uint16_t sequence,
        uint8_t *packet,
        size_t packet_capacity,
        size_t &packet_length
    );


    /*
     * Inform the buffer that this sequence has now
     * passed its playback deadline.
     *
     * Packets older than this point are "late".
     */
    void set_expected_sequence(
        uint16_t sequence
    );


    size_t size() const;


private:

    struct Slot {

        bool occupied =
            false;

        uint16_t sequence =
            0;

        size_t packet_length =
            0;

        uint8_t packet[
            MAX_PACKET_SIZE
        ] = {};
    };


    static bool sequence_before(
        uint16_t lhs,
        uint16_t rhs
    );


    Slot slots_[
        CAPACITY
    ];


    bool expected_valid_ =
        false;


    uint16_t expected_sequence_ =
        0;
};

}
}