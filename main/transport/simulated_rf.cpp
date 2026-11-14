#include "simulated_rf.hpp"

#include <cstring>


namespace slipstream {
namespace transport {


SimulatedRf::SimulatedRf()
{
    reset();
}


void SimulatedRf::reset()
{
    for (
        size_t i = 0;
        i < MAX_IN_FLIGHT;
        ++i
    ) {

        slots_[i] =
            {};
    }
}


SimulatedRfEvent SimulatedRf::send(
    const uint8_t *packet,
    size_t packet_length,
    uint16_t sequence,
    uint32_t current_tick
)
{
    if (
        packet == nullptr ||
        packet_length == 0 ||
        packet_length >
            MAX_PACKET_SIZE
    ) {

        return
            SimulatedRfEvent::FULL;
    }


    /*
     * ------------------------------------------------
     * Deterministic RF impairment profile.
     * ------------------------------------------------
     *
     * Roughly:
     *
     * 2% hard packet loss
     * 2% corruption
     * 5% +40 ms delay
     * 10% +20 ms delay
     *
     * Delay naturally produces some out-of-order
     * arrival.
     * ------------------------------------------------
     */

    if (
        sequence %
            50 ==
        17
    ) {

        return
            SimulatedRfEvent::DROPPED;
    }


    bool corrupt =
        (
            sequence %
                50 ==
            31
        );


    uint32_t delay_ticks =
        0;


    if (
        sequence %
            20 ==
        7
    ) {

        delay_ticks =
            2;
    }
    else if (
        sequence %
            10 ==
        3
    ) {

        delay_ticks =
            1;
    }


    Slot *free_slot =
        nullptr;


    for (
        size_t i = 0;
        i < MAX_IN_FLIGHT;
        ++i
    ) {

        if (
            !slots_[i].occupied
        ) {

            free_slot =
                &slots_[i];

            break;
        }
    }


    if (
        free_slot == nullptr
    ) {

        return
            SimulatedRfEvent::FULL;
    }


    free_slot->occupied =
        true;


    free_slot->delivery_tick =
        current_tick +
        delay_ticks;


    free_slot->sequence =
        sequence;


    free_slot->packet_length =
        packet_length;


    std::memcpy(
        free_slot->packet,
        packet,
        packet_length
    );


    /*
     * Corrupt one byte AFTER CRC generation.
     *
     * H13 parsing should reject this packet.
     */
    if (
        corrupt &&
        packet_length >
            HEADER_SIZE +
            CRC_SIZE
    ) {

        free_slot->packet[
            HEADER_SIZE
        ] ^=
            0x01;


        return
            SimulatedRfEvent::CORRUPTED;
    }


    if (
        delay_ticks == 2
    ) {

        return
            SimulatedRfEvent::
                DELAY_40_MS;
    }


    if (
        delay_ticks == 1
    ) {

        return
            SimulatedRfEvent::
                DELAY_20_MS;
    }


    return
        SimulatedRfEvent::NORMAL;
}


bool SimulatedRf::receive(
    uint32_t current_tick,
    uint8_t *packet,
    size_t packet_capacity,
    size_t &packet_length
)
{
    packet_length =
        0;


    if (
        packet == nullptr
    ) {
        return false;
    }


    /*
     * Find the oldest packet whose delivery time
     * has arrived.
     */

    Slot *selected =
        nullptr;


    for (
        size_t i = 0;
        i < MAX_IN_FLIGHT;
        ++i
    ) {

        Slot &slot =
            slots_[i];


        if (
            !slot.occupied ||
            slot.delivery_tick >
                current_tick
        ) {

            continue;
        }


        if (
            selected == nullptr ||
            slot.delivery_tick <
                selected->delivery_tick ||
            (
                slot.delivery_tick ==
                    selected->delivery_tick &&
                slot.sequence <
                    selected->sequence
            )
        ) {

            selected =
                &slot;
        }
    }


    if (
        selected == nullptr
    ) {

        return false;
    }


    if (
        selected->packet_length >
        packet_capacity
    ) {

        selected->occupied =
            false;

        return false;
    }


    packet_length =
        selected->packet_length;


    std::memcpy(
        packet,
        selected->packet,
        packet_length
    );


    selected->occupied =
        false;


    return true;
}

}
}