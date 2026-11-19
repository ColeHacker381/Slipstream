#include "h18_espnow.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "transport/espnow_transport.hpp"
#include "transport/slipstream_packet.hpp"

#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


namespace {

constexpr const char *TAG =
    "H18";


/*
 * ============================================================
 * CHANGE THIS FOR EACH BOARD.
 *
 * Board A:
 *     true
 *
 * Board B:
 *     false
 * ============================================================
 */

constexpr bool IS_TRANSMITTER =
    false;


/*
 * H18A deliberately sends slower than our eventual
 * audio packet rate.
 *
 * Once this works, H18B will send every 20 ms.
 */
constexpr uint32_t TEST_SEND_PERIOD_MS =
    100;


constexpr uint8_t STREAM_ID =
    1;


constexpr uint8_t CHANNEL =
    0;


slipstream::transport::
    EspNowTransport radio;


/*
 * Payload exists only to make it easy to verify the
 * packet arrived intact through the complete H13
 * framing/CRC path.
 */
struct TestPayload {

    uint32_t marker;

    uint32_t counter;

};


void transmitter_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "H18A transmitter task started"
    );


    uint16_t sequence =
        0;


    uint32_t timestamp =
        0;


    uint32_t counter =
        0;


    uint8_t packet[
        slipstream::transport::
            MAX_PACKET_SIZE
    ] = {};


    int64_t last_report =
        esp_timer_get_time();


    while (true) {

        TestPayload payload = {};


        payload.marker =
            0x534C4950;


        payload.counter =
            counter;


        slipstream::transport::
            PacketHeader header = {};


        header.flags =
            slipstream::transport::
                FLAG_NONE;


        header.stream_id =
            STREAM_ID;


        header.channel =
            CHANNEL;


        header.sequence =
            sequence;


        header.timestamp =
            timestamp;


        size_t packet_length =
            0;


        bool built =
            slipstream::transport::
                build_packet(
                    header,
                    reinterpret_cast<
                        const uint8_t *
                    >(
                        &payload
                    ),
                    sizeof(
                        payload
                    ),
                    packet,
                    sizeof(
                        packet
                    ),
                    packet_length
                );


        if (!built) {

            ESP_LOGE(
                TAG,
                "H13 packet build failed"
            );
        }
        else {

            radio.send(
                packet,
                packet_length
            );
        }


        sequence++;


        /*
         * H18A isn't audio yet, but keep a timestamp
         * progression in the same units as our
         * 16 kHz stream.
         *
         * 100 ms = 1600 samples.
         */
        timestamp +=
            1600;


        counter++;


        int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            slipstream::transport::
                EspNowTransport::Stats
                stats = {};


            radio.get_stats(
                stats
            );


            ESP_LOGI(
                TAG,
                "TX | "
                "seq=%u | "
                "requested=%lu | "
                "submitfail=%lu | "
                "macok=%lu | "
                "macfail=%lu",
                static_cast<unsigned int>(
                    sequence
                ),
                static_cast<unsigned long>(
                    stats.tx_requests
                ),
                static_cast<unsigned long>(
                    stats.tx_submit_failures
                ),
                static_cast<unsigned long>(
                    stats.tx_mac_success
                ),
                static_cast<unsigned long>(
                    stats.tx_mac_fail
                )
            );


            radio.reset_stats();


            last_report =
                now;
        }


        vTaskDelay(
            pdMS_TO_TICKS(
                TEST_SEND_PERIOD_MS
            )
        );
    }
}


void receiver_task(
    void *argument
)
{
    (void) argument;


    ESP_LOGI(
        TAG,
        "H18A receiver task started"
    );


    bool sequence_valid =
        false;


    uint16_t expected_sequence =
        0;


    uint32_t good_packets =
        0;


    uint32_t crc_failures =
        0;


    uint32_t sequence_gaps =
        0;


    uint32_t invalid_payloads =
        0;


    uint32_t last_counter =
        0;


    int8_t last_rssi =
        0;


    uint8_t last_source_mac[6] =
        {};


    int64_t last_report =
        esp_timer_get_time();


    while (true) {

        slipstream::transport::
            EspNowTransport::
                ReceivedPacket received = {};


        bool got_packet =
            radio.receive(
                received,
                100
            );


        if (
            got_packet
        ) {

            last_rssi =
                received.rssi;


            std::memcpy(
                last_source_mac,
                received.source_mac,
                6
            );


            slipstream::transport::
                ParsedPacket parsed = {};


            auto result =
                slipstream::transport::
                    parse_packet(
                        received.data,
                        received.length,
                        parsed
                    );


            if (
                result ==
                slipstream::transport::
                    ParseResult::BAD_CRC
            ) {

                crc_failures++;
            }
            else if (
                result ==
                slipstream::transport::
                    ParseResult::OK
            ) {

                if (
                    parsed.payload_length ==
                    sizeof(
                        TestPayload
                    )
                ) {

                    TestPayload payload = {};


                    std::memcpy(
                        &payload,
                        parsed.payload,
                        sizeof(
                            payload
                        )
                    );


                    if (
                        payload.marker ==
                        0x534C4950
                    ) {

                        good_packets++;


                        last_counter =
                            payload.counter;


                        if (
                            sequence_valid
                        ) {

                            if (
                                parsed.header.sequence !=
                                expected_sequence
                            ) {

                                /*
                                 * For H18A we only need a
                                 * simple gap count.
                                 */
                                sequence_gaps++;


                                ESP_LOGW(
                                    TAG,
                                    "Sequence gap: "
                                    "expected=%u got=%u",
                                    static_cast<
                                        unsigned int
                                    >(
                                        expected_sequence
                                    ),
                                    static_cast<
                                        unsigned int
                                    >(
                                        parsed.header.sequence
                                    )
                                );
                            }
                        }


                        expected_sequence =
                            static_cast<uint16_t>(
                                parsed.header.sequence +
                                1
                            );


                        sequence_valid =
                            true;
                    }
                    else {

                        invalid_payloads++;
                    }
                }
                else {

                    invalid_payloads++;
                }
            }
        }


        int64_t now =
            esp_timer_get_time();


        if (
            now -
            last_report >=
            1000000
        ) {

            slipstream::transport::
                EspNowTransport::Stats
                stats = {};


            radio.get_stats(
                stats
            );


            ESP_LOGI(
                TAG,
                "RX | "
                "good=%lu | "
                "crc=%lu | "
                "gaps=%lu | "
                "invalid=%lu | "
                "counter=%lu | "
                "RSSI=%d dBm | "
                "src=%02X:%02X:%02X:"
                "%02X:%02X:%02X | "
                "qdrop=%lu",
                static_cast<unsigned long>(
                    good_packets
                ),
                static_cast<unsigned long>(
                    crc_failures
                ),
                static_cast<unsigned long>(
                    sequence_gaps
                ),
                static_cast<unsigned long>(
                    invalid_payloads
                ),
                static_cast<unsigned long>(
                    last_counter
                ),
                static_cast<int>(
                    last_rssi
                ),
                last_source_mac[0],
                last_source_mac[1],
                last_source_mac[2],
                last_source_mac[3],
                last_source_mac[4],
                last_source_mac[5],
                static_cast<unsigned long>(
                    stats.rx_queue_drops
                )
            );


            /*
             * Leave sequence_gaps cumulative because
             * it is especially useful to notice even
             * one failure during the whole run.
             */

            good_packets =
                0;


            crc_failures =
                0;


            invalid_payloads =
                0;


            radio.reset_stats();


            last_report =
                now;
        }
    }
}

}


namespace h18 {


void run()
{
    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Slipstream H18A - ESP-NOW Link"
    );


    ESP_LOGI(
        TAG,
        "================================"
    );


    ESP_LOGI(
        TAG,
        "Role: %s",
        IS_TRANSMITTER
            ? "TRANSMITTER"
            : "RECEIVER"
    );


    ESP_LOGI(
        TAG,
        "ESP-NOW channel: %u",
        static_cast<unsigned int>(
            slipstream::transport::
                EspNowTransport::
                    WIFI_CHANNEL
        )
    );


    ESP_LOGI(
        TAG,
        "Broadcast transport / no encryption"
    );


    if (
        !radio.init()
    ) {

        ESP_LOGE(
            TAG,
            "ESP-NOW initialization failed"
        );


        return;
    }


    if (
        IS_TRANSMITTER
    ) {

        BaseType_t result =
            xTaskCreatePinnedToCore(
                transmitter_task,
                "h18_tx",
                8192,
                nullptr,
                4,
                nullptr,
                0
            );


        if (
            result !=
            pdPASS
        ) {

            ESP_LOGE(
                TAG,
                "TX task creation failed"
            );
        }
    }
    else {

        BaseType_t result =
            xTaskCreatePinnedToCore(
                receiver_task,
                "h18_rx",
                8192,
                nullptr,
                4,
                nullptr,
                0
            );


        if (
            result !=
            pdPASS
        ) {

            ESP_LOGE(
                TAG,
                "RX task creation failed"
            );
        }
    }
}


}