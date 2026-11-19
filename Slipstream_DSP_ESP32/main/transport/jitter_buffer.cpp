#include "jitter_buffer.hpp"

#include <cstring>


namespace slipstream {
namespace transport {


PacketJitterBuffer::PacketJitterBuffer()
{
    reset();
}


void PacketJitterBuffer::reset()
{
    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        slots_[i] =
            {};
    }


    expected_valid_ =
        false;


    expected_sequence_ =
        0;
}


bool PacketJitterBuffer::sequence_before(
    uint16_t lhs,
    uint16_t rhs
)
{
    return
        static_cast<int16_t>(
            lhs -
            rhs
        )
        <
        0;
}


JitterInsertResult PacketJitterBuffer::insert(
    const uint8_t *packet,
    size_t packet_length,
    uint16_t sequence
)
{
    if (
        packet == nullptr ||
        packet_length >
            MAX_PACKET_SIZE
    ) {

        return
            JitterInsertResult::
                TOO_LARGE;
    }


    /*
     * Packet already missed its playback deadline.
     */
    if (
        expected_valid_ &&
        sequence_before(
            sequence,
            expected_sequence_
        )
    ) {

        return
            JitterInsertResult::LATE;
    }


    /*
     * Duplicate?
     */
    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        if (
            slots_[i].occupied &&
            slots_[i].sequence ==
                sequence
        ) {

            return
                JitterInsertResult::
                    DUPLICATE;
        }
    }


    /*
     * Find free slot.
     */
    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        if (
            !slots_[i].occupied
        ) {

            slots_[i].occupied =
                true;


            slots_[i].sequence =
                sequence;


            slots_[i].packet_length =
                packet_length;


            std::memcpy(
                slots_[i].packet,
                packet,
                packet_length
            );


            return
                JitterInsertResult::OK;
        }
    }


    return
        JitterInsertResult::FULL;
}


bool PacketJitterBuffer::take(
    uint16_t sequence,
    uint8_t *packet,
    size_t packet_capacity,
    size_t &packet_length
)
{
    packet_length =
        0;


    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        Slot &slot =
            slots_[i];


        if (
            !slot.occupied ||
            slot.sequence !=
                sequence
        ) {

            continue;
        }


        if (
            slot.packet_length >
            packet_capacity
        ) {

            slot.occupied =
                false;

            return false;
        }


        packet_length =
            slot.packet_length;


        std::memcpy(
            packet,
            slot.packet,
            packet_length
        );


        slot.occupied =
            false;


        return true;
    }


    return false;
}


void PacketJitterBuffer::
set_expected_sequence(
    uint16_t sequence
)
{
    expected_valid_ =
        true;


    expected_sequence_ =
        sequence;


    /*
     * Remove packets that have become too late.
     */
    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        if (
            slots_[i].occupied &&
            sequence_before(
                slots_[i].sequence,
                expected_sequence_
            )
        ) {

            slots_[i].occupied =
                false;
        }
    }
}


size_t PacketJitterBuffer::size() const
{
    size_t count =
        0;


    for (
        size_t i = 0;
        i < CAPACITY;
        ++i
    ) {

        if (
            slots_[i].occupied
        ) {

            count++;
        }
    }


    return count;
}

}
}