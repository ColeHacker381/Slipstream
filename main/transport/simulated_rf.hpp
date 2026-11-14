#pragma once

#include <cstddef>
#include <cstdint>

#include "transport/slipstream_packet.hpp"


namespace slipstream {
namespace transport {


enum class SimulatedRfEvent {

    NORMAL,

    DELAY_20_MS,

    DELAY_40_MS,

    DROPPED,

    CORRUPTED,

    FULL
};


class SimulatedRf {
public:

    static constexpr size_t MAX_IN_FLIGHT =
        8;


    SimulatedRf();


    void reset();


    /*
     * Submit a complete H13 packet at the current
     * 20 ms RF tick.
     *
     * The simulation deterministically introduces:
     *
     *     packet loss
     *     20 ms jitter
     *     40 ms jitter
     *     corruption
     *
     * Reordering naturally results from the delays.
     */
    SimulatedRfEvent send(
        const uint8_t *packet,
        size_t packet_length,
        uint16_t sequence,
        uint32_t current_tick
    );


    /*
     * Return one packet whose simulated delivery
     * time has arrived.
     *
     * Call repeatedly until false to drain all packets
     * arriving during the current tick.
     */
    bool receive(
        uint32_t current_tick,
        uint8_t *packet,
        size_t packet_capacity,
        size_t &packet_length
    );


private:

    struct Slot {

        bool occupied =
            false;

        uint32_t delivery_tick =
            0;

        uint16_t sequence =
            0;

        size_t packet_length =
            0;

        uint8_t packet[
            MAX_PACKET_SIZE
        ] = {};
    };


    Slot slots_[
        MAX_IN_FLIGHT
    ];
};

}
}