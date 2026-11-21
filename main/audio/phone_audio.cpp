
#include "phone_audio.hpp"

#include <atomic>
#include <algorithm>
#include <cstring>

#include "driver/i2s_std.h"

#include "esp_check.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "phone_audio";

constexpr gpio_num_t PIN_BCLK = GPIO_NUM_26;
constexpr gpio_num_t PIN_WS   = GPIO_NUM_27;
constexpr gpio_num_t PIN_DIN  = GPIO_NUM_34;

constexpr size_t QUEUE_DEPTH = 3;

constexpr size_t BLOCK_BYTES =
    phone_audio::SAMPLES_PER_BLOCK *
    sizeof(int16_t);

struct AudioBlock {
    int16_t samples[
        phone_audio::SAMPLES_PER_BLOCK
    ];
};

i2s_chan_handle_t rx_channel = nullptr;

QueueHandle_t audio_queue = nullptr;

std::atomic<uint32_t> received_blocks{0};
std::atomic<uint32_t> consumed_blocks{0};
std::atomic<uint32_t> dropped_blocks{0};
std::atomic<uint32_t> missing_blocks{0};
std::atomic<uint32_t> read_errors{0};
std::atomic<uint32_t> peak_amplitude{0};

void update_peak(uint32_t peak)
{
    uint32_t previous = peak_amplitude.load();

    while (previous < peak) {
        if (peak_amplitude.compare_exchange_weak(
                previous,
                peak
            )) {
            break;
        }
    }
}

void receiver_task(void *)
{
    AudioBlock block = {};
    AudioBlock discarded = {};

    size_t filled = 0;

    ESP_LOGI(
        TAG,
        "Phone audio receiver running on CPU%d",
        xPortGetCoreID()
    );

    while (true) {

        size_t bytes_read = 0;

        uint8_t *destination =
            reinterpret_cast<uint8_t *>(
                block.samples
            );

        const esp_err_t error =
            i2s_channel_read(
                rx_channel,
                destination + filled,
                BLOCK_BYTES - filled,
                &bytes_read,
                30
            );

        if (bytes_read > BLOCK_BYTES - filled) {
            read_errors.fetch_add(1);
            filled = 0;
            continue;
        }

        filled += bytes_read;

        if (error != ESP_OK) {
            read_errors.fetch_add(1);

            if (error != ESP_ERR_TIMEOUT) {
                filled = 0;
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
        }

        // Preserve partial reads until we have
        // an entire 10 ms stereo audio block.
        if (filled < BLOCK_BYTES) {
            continue;
        }

        filled = 0;

        received_blocks.fetch_add(1);

        uint32_t peak = 0;

        for (size_t i = 0;
             i < phone_audio::SAMPLES_PER_BLOCK;
             ++i) {

            const int32_t sample =
                block.samples[i];

            const uint32_t absolute =
                static_cast<uint32_t>(
                    sample < 0 ? -sample : sample
                );

            peak = std::max(peak, absolute);
        }

        update_peak(peak);

        // Preserve recent audio if the DSP
        // consumer temporarily falls behind.
        if (xQueueSend(
                audio_queue,
                &block,
                0
            ) != pdTRUE) {

            if (xQueueReceive(
                    audio_queue,
                    &discarded,
                    0
                ) == pdTRUE) {
                dropped_blocks.fetch_add(1);
            }

            if (xQueueSend(
                    audio_queue,
                    &block,
                    0
                ) != pdTRUE) {
                dropped_blocks.fetch_add(1);
            }
        }
    }
}

} // namespace

namespace phone_audio {

esp_err_t initialize()
{
    if (audio_queue != nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    audio_queue = xQueueCreate(
        QUEUE_DEPTH,
        sizeof(AudioBlock)
    );

    if (!audio_queue) {
        return ESP_ERR_NO_MEM;
    }

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_1,
            I2S_ROLE_MASTER
        );

    channel_config.dma_desc_num = 4;
    channel_config.dma_frame_num = 240;

    ESP_RETURN_ON_ERROR(
        i2s_new_channel(
            &channel_config,
            nullptr,
            &rx_channel
        ),
        TAG,
        "I2S1 allocation failed"
    );

    i2s_std_config_t config = {};

    config.clk_cfg =
        I2S_STD_CLK_DEFAULT_CONFIG(
            SAMPLE_RATE
        );

    config.slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        );

    config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    config.gpio_cfg.bclk = PIN_BCLK;
    config.gpio_cfg.ws   = PIN_WS;
    config.gpio_cfg.din  = PIN_DIN;
    config.gpio_cfg.dout = I2S_GPIO_UNUSED;

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            rx_channel,
            &config
        ),
        TAG,
        "I2S1 configuration failed"
    );

    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(rx_channel),
        TAG,
        "I2S1 enable failed"
    );

    const BaseType_t result =
        xTaskCreatePinnedToCore(
            receiver_task,
            "phone_audio_rx",
            6144,
            nullptr,
            5,
            nullptr,
            1
        );

    if (result != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "I2S1 RX ready: 48k stereo PCM16"
    );

    ESP_LOGI(
        TAG,
        "BCLK=26 WS=27 DIN=34"
    );

    return ESP_OK;
}

bool read_block(int16_t *output)
{
    if (!output) {
        return false;
    }

    // Never block the real-time DSP audio task.
    if (!audio_queue ||
        xQueueReceive(
            audio_queue,
            output,
            0
        ) != pdTRUE) {

        std::memset(
            output,
            0,
            BLOCK_BYTES
        );

        missing_blocks.fetch_add(1);

        return false;
    }

    consumed_blocks.fetch_add(1);

    return true;
}

void read_and_reset_stats(Stats &stats)
{
    stats.received_blocks =
        received_blocks.exchange(0);

    stats.consumed_blocks =
        consumed_blocks.exchange(0);

    stats.dropped_blocks =
        dropped_blocks.exchange(0);

    stats.missing_blocks =
        missing_blocks.exchange(0);

    stats.read_errors =
        read_errors.exchange(0);

    stats.peak =
        peak_amplitude.exchange(0);
}

} // namespace phone_audio
