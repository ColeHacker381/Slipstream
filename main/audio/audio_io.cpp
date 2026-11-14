#include "audio_io.hpp"

#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_check.h"

namespace {

constexpr const char *TAG = "audio_io";

constexpr int HARDWARE_SAMPLE_RATE = 48000;

constexpr gpio_num_t I2S_BCLK =
    GPIO_NUM_16;

constexpr gpio_num_t I2S_LRCLK =
    GPIO_NUM_25;

constexpr gpio_num_t I2S_DOUT =
    GPIO_NUM_4;

constexpr gpio_num_t I2S_DIN =
    GPIO_NUM_17;


i2s_chan_handle_t tx_handle = nullptr;
i2s_chan_handle_t rx_handle = nullptr;

}


namespace audio {

esp_err_t init_output()
{
    return init_full_duplex();
}


esp_err_t init_input()
{
    return init_full_duplex();
}


esp_err_t init_full_duplex()
{
    ESP_LOGI(TAG, "Initializing full-duplex I2S");

    /*
     * Create TX and RX together on the same I2S peripheral.
     *
     * They therefore share BCLK and WS.
     */
    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_MASTER
        );

    esp_err_t err = i2s_new_channel(
        &channel_config,
        &tx_handle,
        &rx_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to create I2S channels: %s",
            esp_err_to_name(err)
        );

        return err;
    }


    /*
     * Common clock/slot configuration.
     */
    i2s_std_config_t tx_config = {};

    tx_config.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(
            HARDWARE_SAMPLE_RATE
        );

    tx_config.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        );

    tx_config.gpio_cfg.mclk =
        I2S_GPIO_UNUSED;

    tx_config.gpio_cfg.bclk =
        GPIO_NUM_16;

    tx_config.gpio_cfg.ws =
        GPIO_NUM_25;

    tx_config.gpio_cfg.dout =
        GPIO_NUM_4;

    tx_config.gpio_cfg.din =
        I2S_GPIO_UNUSED;


    /*
     * RX uses the same BCLK and WS.
     */
    i2s_std_config_t rx_config = {};

    rx_config.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(
            HARDWARE_SAMPLE_RATE
        );

    rx_config.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        );

    rx_config.gpio_cfg.mclk =
        I2S_GPIO_UNUSED;

    rx_config.gpio_cfg.bclk =
        GPIO_NUM_16;

    rx_config.gpio_cfg.ws =
        GPIO_NUM_25;

    rx_config.gpio_cfg.dout =
        I2S_GPIO_UNUSED;

    rx_config.gpio_cfg.din =
        GPIO_NUM_17;


    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            tx_handle,
            &tx_config
        ),
        TAG,
        "TX configuration failed"
    );


    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            rx_handle,
            &rx_config
        ),
        TAG,
        "RX configuration failed"
    );


    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(tx_handle),
        TAG,
        "TX enable failed"
    );


    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(rx_handle),
        TAG,
        "RX enable failed"
    );


    ESP_LOGI(
        TAG,
        "I2S full duplex active: 48 kHz / 16-bit stereo"
    );

    return ESP_OK;
}

esp_err_t read(
    void *samples,
    size_t bytes,
    size_t *bytes_read
)
{
    return i2s_channel_read(
        rx_handle,
        samples,
        bytes,
        bytes_read,
        portMAX_DELAY
    );
}

esp_err_t write(
    const void *samples,
    size_t bytes
)
{
    size_t bytes_written = 0;

    esp_err_t err = i2s_channel_write(
        tx_handle,
        samples,
        bytes,
        &bytes_written,
        portMAX_DELAY
    );


    if (err != ESP_OK) {
        return err;
    }


    if (bytes_written != bytes) {

        ESP_LOGW(
            TAG,
            "Short TX write: %zu / %zu",
            bytes_written,
            bytes
        );
    }


    return ESP_OK;
}

}
