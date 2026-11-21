
#include "audio_io.hpp"

#include <algorithm>
#include <cstdint>

#include "phone_audio.hpp"
#include "priority_mixer.hpp"
#include "rx_output_stage.hpp"

#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

namespace {

constexpr const char *TAG = "audio_io";

constexpr uint32_t HARDWARE_SAMPLE_RATE = 48000;

constexpr size_t FRAMES_PER_BLOCK = 480;
constexpr size_t SAMPLES_PER_BLOCK =
    FRAMES_PER_BLOCK * 2;

constexpr size_t BLOCK_BYTES =
    SAMPLES_PER_BLOCK * sizeof(int16_t);

// Existing WM8960 I2S0 pin assignments.
// These remain unchanged.
constexpr gpio_num_t I2S_BCLK =
    GPIO_NUM_16;

constexpr gpio_num_t I2S_LRCLK =
    GPIO_NUM_25;

constexpr gpio_num_t I2S_DOUT =
    GPIO_NUM_4;

constexpr gpio_num_t I2S_DIN =
    GPIO_NUM_17;

// Set to false if testing the intercom
// without the dedicated Bluetooth board.
constexpr bool ENABLE_PHONE_AUDIO = true;

i2s_chan_handle_t tx_handle = nullptr;
i2s_chan_handle_t rx_handle = nullptr;

bool phone_audio_ready = false;

// Reuse the existing H16/H17 audio components.
audio::PriorityMixer priority_mixer;
audio::RxOutputStage output_stage;

// Fixed buffers: no per-block heap allocation.
int16_t phone_pcm[SAMPLES_PER_BLOCK] = {};
int16_t mixed_output[SAMPLES_PER_BLOCK] = {};
float mixed_float[SAMPLES_PER_BLOCK] = {};

// Integration diagnostics.
uint32_t mix_blocks = 0;
uint32_t mix_peak = 0;
uint32_t mix_max_us = 0;
uint32_t mix_limited_frames = 0;
uint32_t mix_safety_clamps = 0;

int64_t last_report_us = 0;

void report_phone_audio()
{
    const int64_t now = esp_timer_get_time();

    if (now - last_report_us < 10000000) {
        return;
    }

    phone_audio::Stats stats = {};

    phone_audio::read_and_reset_stats(stats);

    ESP_LOGI(
        TAG,
        "PHONE | rx=%lu consumed=%lu "
        "dropped=%lu missing=%lu "
        "errors=%lu peak=%lu",

        static_cast<unsigned long>(
            stats.received_blocks
        ),

        static_cast<unsigned long>(
            stats.consumed_blocks
        ),

        static_cast<unsigned long>(
            stats.dropped_blocks
        ),

        static_cast<unsigned long>(
            stats.missing_blocks
        ),

        static_cast<unsigned long>(
            stats.read_errors
        ),

        static_cast<unsigned long>(
            stats.peak
        )
    );

    ESP_LOGI(
        TAG,
        "MIX | blocks=%lu peak=%lu "
        "max_us=%lu limited=%lu clamps=%lu",

        static_cast<unsigned long>(
            mix_blocks
        ),

        static_cast<unsigned long>(
            mix_peak
        ),

        static_cast<unsigned long>(
            mix_max_us
        ),

        static_cast<unsigned long>(
            mix_limited_frames
        ),

        static_cast<unsigned long>(
            mix_safety_clamps
        )
    );

    mix_blocks = 0;
    mix_peak = 0;
    mix_max_us = 0;
    mix_limited_frames = 0;
    mix_safety_clamps = 0;

    last_report_us = now;
}

esp_err_t write_to_codec(
    const void *samples,
    size_t bytes
)
{
    size_t bytes_written = 0;

    const esp_err_t err =
        i2s_channel_write(
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

} // namespace

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
    ESP_LOGI(
        TAG,
        "Initializing full-duplex WM8960 I2S"
    );

    i2s_chan_config_t channel_config =
        I2S_CHANNEL_DEFAULT_CONFIG(
            I2S_NUM_0,
            I2S_ROLE_MASTER
        );

    ESP_RETURN_ON_ERROR(
        i2s_new_channel(
            &channel_config,
            &tx_handle,
            &rx_handle
        ),
        TAG,
        "WM8960 I2S allocation failed"
    );

    // Existing WM8960 transmit configuration.
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
        I2S_BCLK;

    tx_config.gpio_cfg.ws =
        I2S_LRCLK;

    tx_config.gpio_cfg.dout =
        I2S_DOUT;

    tx_config.gpio_cfg.din =
        I2S_GPIO_UNUSED;

    // Existing WM8960 receive configuration.
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
        I2S_BCLK;

    rx_config.gpio_cfg.ws =
        I2S_LRCLK;

    rx_config.gpio_cfg.dout =
        I2S_GPIO_UNUSED;

    rx_config.gpio_cfg.din =
        I2S_DIN;

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            tx_handle,
            &tx_config
        ),
        TAG,
        "WM8960 TX configuration failed"
    );

    ESP_RETURN_ON_ERROR(
        i2s_channel_init_std_mode(
            rx_handle,
            &rx_config
        ),
        TAG,
        "WM8960 RX configuration failed"
    );

    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(tx_handle),
        TAG,
        "WM8960 TX enable failed"
    );

    ESP_RETURN_ON_ERROR(
        i2s_channel_enable(rx_handle),
        TAG,
        "WM8960 RX enable failed"
    );

    ESP_LOGI(
        TAG,
        "WM8960 I2S0 ready: 48 kHz stereo"
    );

    // Initialize the existing output processing.
    priority_mixer.reset();
    output_stage.reset();

    last_report_us = esp_timer_get_time();

    // Initialize the second I2S peripheral.
    // Failure must not disable RF communication.
    if (ENABLE_PHONE_AUDIO) {

        const esp_err_t phone_error =
            phone_audio::initialize();

        if (phone_error == ESP_OK) {

            phone_audio_ready = true;

            ESP_LOGI(
                TAG,
                "D1.8.3 phone audio enabled"
            );

            ESP_LOGI(
                TAG,
                "I2S1: BCLK=26 WS=27 DIN=34"
            );

            ESP_LOGI(
                TAG,
                "Bluetooth + RF output mixing active"
            );

        } else {

            phone_audio_ready = false;

            ESP_LOGE(
                TAG,
                "Phone audio initialization failed: %s",
                esp_err_to_name(phone_error)
            );

            ESP_LOGW(
                TAG,
                "Continuing with RF intercom only"
            );
        }
    }

    return ESP_OK;
}

esp_err_t read(
    void *samples,
    size_t bytes,
    size_t *bytes_read
)
{
    // The microphone path is unchanged.
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
    if (samples == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    // Preserve the original output behavior for
    // older tests, nonstandard block sizes,
    // or failed phone audio initialization.
    if (!phone_audio_ready ||
        bytes != BLOCK_BYTES) {

        return write_to_codec(
            samples,
            bytes
        );
    }

    const int64_t start_us =
        esp_timer_get_time();

    const auto *rf_pcm =
        static_cast<const int16_t *>(samples);

    // Nonblocking: a missing phone block
    // becomes silence.
    phone_audio::read_block(phone_pcm);

    // D1.8.3 validates simultaneous playback.
    //
    // Remote speech metadata will be restored
    // in D1.8.4 for automatic music ducking.
    priority_mixer.set_rf_speech_active(false);

    // Mix both streams in floating-point PCM
    // units, without premature clipping.
    priority_mixer.process_block_float(
        phone_pcm,
        rf_pcm,
        mixed_float,
        SAMPLES_PER_BLOCK
    );

    // Apply the existing H17 output limiter
    // and convert the result back to PCM16.
    RxOutputStats output_stats = {};

    output_stage.process_block(
        mixed_float,
        mixed_output,
        FRAMES_PER_BLOCK,
        output_stats
    );

    mix_limited_frames +=
        output_stats.limited_frames;

    mix_safety_clamps +=
        output_stats.safety_clamps;

    // Measure final headphone signal peak.
    for (size_t i = 0;
         i < SAMPLES_PER_BLOCK;
         ++i) {

        const int32_t sample =
            mixed_output[i];

        const uint32_t absolute =
            static_cast<uint32_t>(
                sample < 0 ? -sample : sample
            );

        mix_peak = std::max(
            mix_peak,
            absolute
        );
    }

    const uint32_t elapsed_us =
        static_cast<uint32_t>(
            esp_timer_get_time() - start_us
        );

    mix_max_us = std::max(
        mix_max_us,
        elapsed_us
    );

    ++mix_blocks;

    const esp_err_t result =
        write_to_codec(
            mixed_output,
            sizeof(mixed_output)
        );

    report_phone_audio();

    return result;
}

} // namespace audio
