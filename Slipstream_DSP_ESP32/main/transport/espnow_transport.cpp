#include "espnow_transport.hpp"

#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "nvs_flash.h"


namespace {

constexpr const char *TAG =
    "espnow_transport";


constexpr uint8_t BROADCAST_MAC[
    ESP_NOW_ETH_ALEN
] = {

    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF,
    0xFF

};

}


namespace slipstream {
namespace transport {


EspNowTransport *
EspNowTransport::instance_ =
    nullptr;


EspNowTransport::EspNowTransport()
{
}


bool EspNowTransport::initialize_wifi()
{
    esp_err_t error =
        nvs_flash_init();


    if (
        error ==
            ESP_ERR_NVS_NO_FREE_PAGES ||
        error ==
            ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {

        ESP_ERROR_CHECK(
            nvs_flash_erase()
        );


        error =
            nvs_flash_init();
    }


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "NVS init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_netif_init();


    if (
        error != ESP_OK &&
        error != ESP_ERR_INVALID_STATE
    ) {

        ESP_LOGE(
            TAG,
            "esp_netif_init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_event_loop_create_default();


    if (
        error != ESP_OK &&
        error != ESP_ERR_INVALID_STATE
    ) {

        ESP_LOGE(
            TAG,
            "event loop init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    wifi_init_config_t config =
        WIFI_INIT_CONFIG_DEFAULT();


    error =
        esp_wifi_init(
            &config
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_wifi_init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_wifi_set_storage(
            WIFI_STORAGE_RAM
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_wifi_set_storage failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_wifi_set_mode(
            WIFI_MODE_STA
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_wifi_set_mode failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_wifi_start();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_wifi_start failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_wifi_set_channel(
            WIFI_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_wifi_set_channel failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    ESP_LOGI(
        TAG,
        "Wi-Fi initialized: STA / channel %u",
        static_cast<unsigned int>(
            WIFI_CHANNEL
        )
    );


    return true;
}


bool EspNowTransport::initialize_espnow()
{
    esp_err_t error =
        esp_now_init();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "esp_now_init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_now_register_recv_cb(
            &EspNowTransport::
                receive_callback
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "register RX callback failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_now_register_send_cb(
            &EspNowTransport::
                send_callback
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "register TX callback failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    /*
     * Broadcast transmissions require the broadcast
     * address to exist in the peer table.
     */
    esp_now_peer_info_t peer = {};


    std::memcpy(
        peer.peer_addr,
        BROADCAST_MAC,
        ESP_NOW_ETH_ALEN
    );


    peer.channel =
        WIFI_CHANNEL;


    peer.ifidx =
        WIFI_IF_STA;


    peer.encrypt =
        false;


    error =
        esp_now_add_peer(
            &peer
        );


    if (
        error != ESP_OK &&
        error != ESP_ERR_ESPNOW_EXIST
    ) {

        ESP_LOGE(
            TAG,
            "add broadcast peer failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    ESP_LOGI(
        TAG,
        "ESP-NOW initialized"
    );


    return true;
}


bool EspNowTransport::init()
{
    if (
        initialized_
    ) {

        return true;
    }


    rx_queue_ =
        xQueueCreate(
            16,
            sizeof(
                ReceivedPacket
            )
        );


    if (
        rx_queue_ ==
        nullptr
    ) {

        ESP_LOGE(
            TAG,
            "RX queue creation failed"
        );


        return false;
    }


    instance_ =
        this;


    if (
        !initialize_wifi()
    ) {

        return false;
    }


    if (
        !initialize_espnow()
    ) {

        return false;
    }


    initialized_ =
        true;


    print_local_mac();


    return true;
}


void EspNowTransport::print_local_mac() const
{
    uint8_t mac[
        ESP_NOW_ETH_ALEN
    ] = {};


    esp_err_t error =
        esp_wifi_get_mac(
            WIFI_IF_STA,
            mac
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "Unable to read MAC: %s",
            esp_err_to_name(
                error
            )
        );


        return;
    }


    ESP_LOGI(
        TAG,
        "Local STA MAC: "
        "%02X:%02X:%02X:%02X:%02X:%02X",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );
}


bool EspNowTransport::send(
    const uint8_t *data,
    size_t length
)
{
    if (
        !initialized_ ||
        data ==
            nullptr ||
        length ==
            0 ||
        length >
            MAX_PACKET_SIZE
    ) {

        return false;
    }


    stats_.tx_requests++;


    esp_err_t error =
        esp_now_send(
            BROADCAST_MAC,
            data,
            length
        );


    if (
        error !=
        ESP_OK
    ) {

        stats_.
            tx_submit_failures++;


        ESP_LOGW(
            TAG,
            "esp_now_send failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    return true;
}


bool EspNowTransport::receive(
    ReceivedPacket &packet,
    uint32_t timeout_ms
)
{
    if (
        !initialized_ ||
        rx_queue_ ==
            nullptr
    ) {

        return false;
    }


    return
        xQueueReceive(
            static_cast<QueueHandle_t>(
                rx_queue_
            ),
            &packet,
            pdMS_TO_TICKS(
                timeout_ms
            )
        )
        ==
        pdTRUE;
}


void EspNowTransport::receive_callback(
    const esp_now_recv_info_t *info,
    const uint8_t *data,
    int length
)
{
    if (
        instance_ ==
            nullptr ||
        info ==
            nullptr ||
        data ==
            nullptr ||
        length <=
            0
    ) {

        return;
    }


    if (
        length >
        static_cast<int>(
            MAX_PACKET_SIZE
        )
    ) {

        instance_->
            stats_.
            rx_oversize++;


        return;
    }


    ReceivedPacket packet = {};


    std::memcpy(
        packet.source_mac,
        info->src_addr,
        ESP_NOW_ETH_ALEN
    );


    packet.length =
        static_cast<size_t>(
            length
        );


    std::memcpy(
        packet.data,
        data,
        packet.length
    );


    if (
        info->rx_ctrl !=
        nullptr
    ) {

        packet.rssi =
            info->rx_ctrl->rssi;
    }


    /*
     * ESP-NOW callback runs in the Wi-Fi task,
     * not a hardware ISR.
     *
     * Keep this non-blocking.
     */
    if (
        xQueueSend(
            static_cast<QueueHandle_t>(
                instance_->
                    rx_queue_
            ),
            &packet,
            0
        )
        !=
        pdTRUE
    ) {

        instance_->
            stats_.
            rx_queue_drops++;
    }
    else {

        instance_->
            stats_.
            rx_packets++;
    }
}


void EspNowTransport::send_callback(
    const esp_now_send_info_t *tx_info,
    esp_now_send_status_t status
)
{
    if (
        instance_ ==
        nullptr ||
        tx_info ==
            nullptr
    ) {

        return;
    }


    if (
        status ==
        ESP_NOW_SEND_SUCCESS
    ) {

        instance_->
            stats_.
            tx_mac_success++;
    }
    else {

        instance_->
            stats_.
            tx_mac_fail++;
    }
}


void EspNowTransport::get_stats(
    Stats &stats
) const
{
    stats =
        stats_;
}


void EspNowTransport::reset_stats()
{
    stats_ =
        {};
}


}
}