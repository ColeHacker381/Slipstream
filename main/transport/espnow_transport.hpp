#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "esp_now.h"


namespace slipstream {
namespace transport {


class EspNowTransport {
public:

    static constexpr uint8_t WIFI_CHANNEL =
        6;


    static constexpr size_t MAX_PACKET_SIZE =
        ESP_NOW_MAX_DATA_LEN;


    struct ReceivedPacket {

        uint8_t source_mac[
            ESP_NOW_ETH_ALEN
        ] = {};


        int8_t rssi =
            0;


        size_t length =
            0;


        uint8_t data[
            MAX_PACKET_SIZE
        ] = {};
    };


    struct Stats {

        uint32_t tx_requests =
            0;


        uint32_t tx_submit_failures =
            0;


        uint32_t tx_mac_success =
            0;


        uint32_t tx_mac_fail =
            0;


        uint32_t rx_packets =
            0;


        uint32_t rx_queue_drops =
            0;


        uint32_t rx_oversize =
            0;
    };


    EspNowTransport();


    bool init();


    bool send(
        const uint8_t *data,
        size_t length
    );


    bool receive(
        ReceivedPacket &packet,
        uint32_t timeout_ms
    );


    void get_stats(
        Stats &stats
    ) const;


    void reset_stats();


    void print_local_mac() const;


private:

    static void receive_callback(
        const esp_now_recv_info_t *info,
        const uint8_t *data,
        int length
    );


    static void send_callback(
        const esp_now_send_info_t *tx_info,
        esp_now_send_status_t status
    );


    bool initialize_wifi();


    bool initialize_espnow();


    void *rx_queue_ =
        nullptr;


    bool initialized_ =
        false;


    Stats stats_ = {};


    static EspNowTransport *instance_;
};


}
}