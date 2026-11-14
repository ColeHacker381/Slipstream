#include "a2dp_sink.hpp"

#include <cstring>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "nvs_flash.h"


namespace {

constexpr const char *TAG =
    "a2dp_sink";


constexpr uint32_t Q16_ONE =
    65536;


}


namespace slipstream {
namespace bluetooth {


A2dpSink *
A2dpSink::instance_ =
    nullptr;


A2dpSink::A2dpSink()
{
}


bool A2dpSink::init()
{
    if (
        initialized_
    ) {

        return true;
    }


    /*
     * --------------------------------------------------------
     * STREAM BUFFERS
     * --------------------------------------------------------
     *
     * input_stream_:
     *     bursty decoded A2DP PCM coming from Bluedroid
     *
     * output_stream_:
     *     continuous 48 kHz stereo stream consumed by the
     *     Slipstream audio task
     */

    input_stream_ =
        xStreamBufferCreate(
            INPUT_STREAM_BYTES,
            1
        );


    if (
        input_stream_ ==
        nullptr
    ) {

        ESP_LOGE(
            TAG,
            "Input stream buffer creation failed"
        );


        return false;
    }


    output_stream_ =
        xStreamBufferCreate(
            OUTPUT_STREAM_BYTES,
            1
        );


    if (
        output_stream_ ==
        nullptr
    ) {

        ESP_LOGE(
            TAG,
            "Output stream buffer creation failed"
        );


        return false;
    }


    instance_ =
        this;


    /*
     * --------------------------------------------------------
     * NVS
     * --------------------------------------------------------
     */

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


    /*
     * We are operating BR/EDR only.
     *
     * If BLE memory is still reserved, release it before the
     * controller starts.
     */

    const esp_err_t
        release_result =
            esp_bt_controller_mem_release(
                ESP_BT_MODE_BLE
            );


    if (
        release_result !=
            ESP_OK &&
        release_result !=
            ESP_ERR_INVALID_STATE
    ) {

        ESP_LOGW(
            TAG,
            "BLE memory release returned: %s",
            esp_err_to_name(
                release_result
            )
        );
    }


    /*
     * --------------------------------------------------------
     * BLUETOOTH CONTROLLER
     * --------------------------------------------------------
     */

    esp_bt_controller_config_t
        controller_config =
            BT_CONTROLLER_INIT_CONFIG_DEFAULT();


    error =
        esp_bt_controller_init(
            &controller_config
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "BT controller init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_bt_controller_enable(
            ESP_BT_MODE_CLASSIC_BT
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "BT controller enable failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    /*
     * --------------------------------------------------------
     * BLUEDROID
     * --------------------------------------------------------
     */

    esp_bluedroid_config_t
        bluedroid_config =
            BT_BLUEDROID_INIT_CONFIG_DEFAULT();


    bluedroid_config.ssp_en =
        true;


    error =
        esp_bluedroid_init_with_cfg(
            &bluedroid_config
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "Bluedroid init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_bluedroid_enable();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "Bluedroid enable failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    /*
     * --------------------------------------------------------
     * A2DP SINK
     * --------------------------------------------------------
     */

    error =
        esp_bt_gap_set_device_name(
            "Slipstream"
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "Device name failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_a2d_register_callback(
            a2dp_event_callback
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "A2DP callback registration failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_a2d_sink_register_data_callback(
            a2dp_pcm_callback
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "A2DP PCM callback registration failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_a2d_sink_init();


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "A2DP sink init failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    error =
        esp_bt_gap_set_scan_mode(
            ESP_BT_CONNECTABLE,
            ESP_BT_GENERAL_DISCOVERABLE
        );


    if (
        error !=
        ESP_OK
    ) {

        ESP_LOGE(
            TAG,
            "Scan mode failed: %s",
            esp_err_to_name(
                error
            )
        );


        return false;
    }


    BaseType_t
        task_result =
            xTaskCreatePinnedToCore(
                worker_task_entry,
                "bt_pcm_worker",
                WORKER_STACK,
                this,
                WORKER_PRIORITY,
                nullptr,
                1
            );


    if (
        task_result !=
        pdPASS
    ) {

        ESP_LOGE(
            TAG,
            "Bluetooth PCM worker creation failed"
        );


        return false;
    }


    initialized_ =
        true;


    ESP_LOGI(
        TAG,
        "Classic Bluetooth A2DP sink initialized"
    );


    ESP_LOGI(
        TAG,
        "Device name: Slipstream"
    );


    ESP_LOGI(
        TAG,
        "Output: 48 kHz / stereo / PCM16"
    );


    ESP_LOGI(
        TAG,
        "BT prebuffer: %u ms",
        30U
    );


    return true;
}


void A2dpSink::a2dp_event_callback(
    esp_a2d_cb_event_t event,
    esp_a2d_cb_param_t *param
)
{
    if (
        instance_ ==
            nullptr ||
        param ==
            nullptr
    ) {

        return;
    }


    switch (
        event
    ) {

        case
            ESP_A2D_CONNECTION_STATE_EVT:
        {

            const bool connected =
                (
                    param->
                        conn_stat.
                        state
                    ==
                    ESP_A2D_CONNECTION_STATE_CONNECTED
                );


            instance_->
                connected_.
                store(
                    connected
                );


            if (
                !connected
            ) {

                instance_->
                    streaming_.
                    store(
                        false
                    );
            }


            ESP_LOGI(
                TAG,
                "A2DP connection state: %d",
                static_cast<int>(
                    param->
                        conn_stat.
                        state
                )
            );


            break;
        }


        case
            ESP_A2D_AUDIO_STATE_EVT:
        {

            const bool streaming =
                (
                    param->
                        audio_stat.
                        state
                    ==
                    ESP_A2D_AUDIO_STATE_STARTED
                );


            instance_->
                streaming_.
                store(
                    streaming
                );


            ESP_LOGI(
                TAG,
                "A2DP audio state: %d",
                static_cast<int>(
                    param->
                        audio_stat.
                        state
                )
            );


            break;
        }


        case
            ESP_A2D_AUDIO_CFG_EVT:
        {

            uint32_t sample_rate =
                44100;


            if (
                param->
                    audio_cfg.
                    mcc.
                    type
                ==
                ESP_A2D_MCT_SBC
            ) {

                const uint8_t
                    frequency =
                        param->
                            audio_cfg.
                            mcc.
                            cie.
                            sbc_info.
                            samp_freq;


                if (
                    frequency &
                    ESP_A2D_SBC_CIE_SF_48K
                ) {

                    sample_rate =
                        48000;
                }
                else if (
                    frequency &
                    ESP_A2D_SBC_CIE_SF_44K
                ) {

                    sample_rate =
                        44100;
                }
                else if (
                    frequency &
                    ESP_A2D_SBC_CIE_SF_32K
                ) {

                    sample_rate =
                        32000;
                }
                else if (
                    frequency &
                    ESP_A2D_SBC_CIE_SF_16K
                ) {

                    sample_rate =
                        16000;
                }
            }


            instance_->
                source_sample_rate_.
                store(
                    sample_rate
                );


            ESP_LOGI(
                TAG,
                "A2DP SBC stream configured: %lu Hz",
                static_cast<unsigned long>(
                    sample_rate
                )
            );


            break;
        }


        default:
            break;
    }
}


void A2dpSink::a2dp_pcm_callback(
    const uint8_t *data,
    uint32_t length
)
{
    if (
        instance_ ==
            nullptr ||
        data ==
            nullptr ||
        length ==
            0 ||
        instance_->
            input_stream_
        ==
        nullptr
    ) {

        return;
    }


    instance_->
        callback_packets_.
        fetch_add(
            1
        );


    instance_->
        callback_bytes_.
        fetch_add(
            length
        );


    /*
     * This callback runs in the Bluetooth host task. Keep it
     * extremely short, but preserve stereo PCM framing.
     *
     * A2DP PCM is signed 16-bit stereo: four bytes per frame.
     * Never write a partial callback into the byte stream. A
     * partial write would shift L/R sample alignment and sound
     * like harsh grain/static until the stream is reset.
     *
     * There is one writer and one reader. If enough space is
     * visible here, the reader can only make MORE space before
     * xStreamBufferSend() executes.
     */

    if (
        (length & 0x3U) !=
        0U
    ) {

        instance_->
            callback_drops_.
            fetch_add(
                1
            );


        return;
    }


    const size_t free_bytes =
        xStreamBufferSpacesAvailable(
            instance_->
                input_stream_
        );


    if (
        free_bytes <
        length
    ) {

        instance_->
            callback_drops_.
            fetch_add(
                1
            );


        return;
    }


    const size_t sent =
        xStreamBufferSend(
            instance_->
                input_stream_,
            data,
            length,
            0
        );


    if (
        sent !=
        length
    ) {

        instance_->
            callback_drops_.
            fetch_add(
                1
            );
    }
}


void A2dpSink::worker_task_entry(
    void *argument
)
{
    auto *self =
        static_cast<A2dpSink *>(
            argument
        );


    self->
        worker_task();
}


void A2dpSink::worker_task()
{
    ESP_LOGI(
        TAG,
        "Bluetooth PCM worker running on CPU%d",
        xPortGetCoreID()
    );


    /*
     * Deliberately small local buffer.
     *
     * The previous implementation carried more work and
     * floating point through this task. H19 now uses a fixed
     * point streaming resampler.
     */

    uint8_t input_chunk[
        1024
    ] = {};


    while (
        true
    ) {

        const size_t received =
            xStreamBufferReceive(
                input_stream_,
                input_chunk,
                sizeof(
                    input_chunk
                ),
                portMAX_DELAY
            );


        if (
            received ==
            0
        ) {

            continue;
        }


        process_pcm(
            input_chunk,
            received
        );
    }
}


void A2dpSink::reset_resampler(
    uint32_t source_rate
)
{
    resampler_ready_ =
        false;


    resampler_source_rate_ =
        source_rate;


    /*
     * Input-frame position advanced for each output frame:
     *
     * source_rate / 48000
     *
     * represented as Q16.
     */

    source_step_q16_ =
        static_cast<uint32_t>(
            (
                static_cast<uint64_t>(
                    source_rate
                )
                <<
                16
            )
            /
            OUTPUT_SAMPLE_RATE
        );


    input_frame_index_ =
        0;


    next_output_position_q16_ =
        0;


    previous_left_ =
        0;


    previous_right_ =
        0;


    output_frames_in_block_ =
        0;


    ESP_LOGI(
        TAG,
        "Fixed-point resampler: %lu -> %lu Hz",
        static_cast<unsigned long>(
            source_rate
        ),
        static_cast<unsigned long>(
            OUTPUT_SAMPLE_RATE
        )
    );
}


int16_t A2dpSink::interpolate_sample(
    int16_t previous,
    int16_t current,
    uint32_t fraction_q16
)
{
    const int32_t delta =
        static_cast<int32_t>(
            current
        )
        -
        static_cast<int32_t>(
            previous
        );


    const int64_t scaled =
        static_cast<int64_t>(
            delta
        )
        *
        static_cast<int64_t>(
            fraction_q16
        );


    int32_t value =
        static_cast<int32_t>(
            previous
        )
        +
        static_cast<int32_t>(
            scaled >>
            16
        );


    if (
        value >
        32767
    ) {

        value =
            32767;
    }


    if (
        value <
        -32768
    ) {

        value =
            -32768;
    }


    return
        static_cast<int16_t>(
            value
        );
}


void A2dpSink::process_pcm(
    const uint8_t *data,
    size_t length
)
{
    if (
        data ==
            nullptr ||
        length <
            4
    ) {

        return;
    }


    const uint32_t
        source_rate =
            source_sample_rate_.
                load();


    if (
        source_rate ==
            0
    ) {

        return;
    }


    if (
        source_rate !=
            resampler_source_rate_
    ) {

        reset_resampler(
            source_rate
        );
    }


    const size_t frame_count =
        length /
        4;


    const auto *samples =
        reinterpret_cast<
            const int16_t *
        >(
            data
        );


    for (
        size_t frame = 0;
        frame < frame_count;
        ++frame
    ) {

        const int16_t
            current_left =
                samples[
                    frame *
                    2
                ];


        const int16_t
            current_right =
                samples[
                    frame *
                    2 +
                    1
                ];


        if (
            !resampler_ready_
        ) {

            previous_left_ =
                current_left;


            previous_right_ =
                current_right;


            input_frame_index_ =
                0;


            next_output_position_q16_ =
                0;


            push_output_sample(
                current_left,
                current_right
            );


            next_output_position_q16_ +=
                source_step_q16_;


            resampler_ready_ =
                true;


            continue;
        }


        input_frame_index_++;


        const uint64_t
            current_position_q16 =
                input_frame_index_
                <<
                16;


        const uint64_t
            previous_position_q16 =
                (
                    input_frame_index_
                    -
                    1
                )
                <<
                16;


        while (
            next_output_position_q16_
            <=
            current_position_q16
        ) {

            uint64_t
                fraction =
                    0;


            if (
                next_output_position_q16_
                >
                previous_position_q16
            ) {

                fraction =
                    next_output_position_q16_
                    -
                    previous_position_q16;
            }


            if (
                fraction >
                Q16_ONE
            ) {

                fraction =
                    Q16_ONE;
            }


            const int16_t left =
                interpolate_sample(
                    previous_left_,
                    current_left,
                    static_cast<uint32_t>(
                        fraction
                    )
                );


            const int16_t right =
                interpolate_sample(
                    previous_right_,
                    current_right,
                    static_cast<uint32_t>(
                        fraction
                    )
                );


            push_output_sample(
                left,
                right
            );


            next_output_position_q16_ +=
                source_step_q16_;
        }


        previous_left_ =
            current_left;


        previous_right_ =
            current_right;
    }
}


void A2dpSink::push_output_sample(
    int16_t left,
    int16_t right
)
{
    const size_t
        sample_index =
            output_frames_in_block_
            *
            2;


    output_block_[
        sample_index
    ] =
        left;


    output_block_[
        sample_index +
        1
    ] =
        right;


    output_frames_in_block_++;


    if (
        output_frames_in_block_
        >=
        OUTPUT_FRAMES_PER_BLOCK
    ) {

        push_completed_output_block();


        output_frames_in_block_ =
            0;
    }
}


void A2dpSink::
push_completed_output_block()
{
    if (
        output_stream_ ==
            nullptr
    ) {

        return;
    }


    /*
     * Never write a partial 10 ms block.
     *
     * If the music buffer is temporarily full, dropping one
     * newest music block is preferable to corrupting alignment
     * or allowing latency to grow without bound.
     */

    const size_t free_bytes =
        xStreamBufferSpacesAvailable(
            output_stream_
        );


    if (
        free_bytes <
        OUTPUT_BYTES_PER_BLOCK
    ) {

        block_drops_.
            fetch_add(
                1
            );


        return;
    }


    const size_t sent =
        xStreamBufferSend(
            output_stream_,
            output_block_,
            OUTPUT_BYTES_PER_BLOCK,
            0
        );


    if (
        sent ==
        OUTPUT_BYTES_PER_BLOCK
    ) {

        blocks_produced_.
            fetch_add(
                1
            );
    }
    else {

        block_drops_.
            fetch_add(
                1
            );
    }
}


void A2dpSink::drain_output_stream()
{
    if (
        output_stream_ ==
            nullptr
    ) {

        return;
    }


    int16_t discard[
        OUTPUT_SAMPLES_PER_BLOCK
    ] = {};


    while (
        xStreamBufferBytesAvailable(
            output_stream_
        )
        >
        0
    ) {

        const size_t received =
            xStreamBufferReceive(
                output_stream_,
                discard,
                sizeof(
                    discard
                ),
                0
            );


        if (
            received ==
            0
        ) {

            break;
        }
    }
}


bool A2dpSink::read_block(
    int16_t *output
)
{
    if (
        output ==
            nullptr ||
        output_stream_ ==
            nullptr
    ) {

        return false;
    }


    /*
     * If A2DP is not actively playing, discard stale music and
     * restart cleanly on the next STARTED event.
     */

    if (
        !streaming_.
            load()
    ) {

        playback_started_ =
            false;


        consecutive_underflows_ =
            0;


        drain_output_stream();


        return false;
    }


    size_t available =
        xStreamBufferBytesAvailable(
            output_stream_
        );


    /*
     * Only the initial start (or a genuine multi-block outage)
     * needs the 30 ms cushion.
     */
    if (
        !playback_started_
    ) {

        if (
            available <
            PREBUFFER_BYTES
        ) {

            return false;
        }


        playback_started_ =
            true;


        consecutive_underflows_ =
            0;


        prebuffer_events_.
            fetch_add(
                1
            );
    }


    available =
        xStreamBufferBytesAvailable(
            output_stream_
        );


    /*
     * A single late Bluetooth burst should cause at most one
     * 10 ms silent block. Do NOT immediately force another
     * 30 ms prebuffer. Rebuffer only after three consecutive
     * misses, which indicates a real stream interruption.
     */

    if (
        available <
        OUTPUT_BYTES_PER_BLOCK
    ) {

        block_underflows_.
            fetch_add(
                1
            );


        consecutive_underflows_++;


        if (
            consecutive_underflows_ >=
            MAX_CONSECUTIVE_UNDERFLOWS
        ) {

            playback_started_ =
                false;


            consecutive_underflows_ =
                0;
        }


        return false;
    }


    const size_t received =
        xStreamBufferReceive(
            output_stream_,
            output,
            OUTPUT_BYTES_PER_BLOCK,
            0
        );


    if (
        received !=
        OUTPUT_BYTES_PER_BLOCK
    ) {

        block_underflows_.
            fetch_add(
                1
            );


        consecutive_underflows_++;


        if (
            consecutive_underflows_ >=
            MAX_CONSECUTIVE_UNDERFLOWS
        ) {

            playback_started_ =
                false;


            consecutive_underflows_ =
                0;
        }


        return false;
    }


    consecutive_underflows_ =
        0;


    return true;
}


bool A2dpSink::connected() const
{
    return
        connected_.
            load();
}


bool A2dpSink::streaming() const
{
    return
        streaming_.
            load();
}


uint32_t A2dpSink::
source_sample_rate() const
{
    return
        source_sample_rate_.
            load();
}


void A2dpSink::get_stats(
    Stats &stats
) const
{
    stats.callback_packets =
        callback_packets_.
            load();


    stats.callback_bytes =
        callback_bytes_.
            load();


    stats.callback_drops =
        callback_drops_.
            load();


    stats.blocks_produced =
        blocks_produced_.
            load();


    stats.block_drops =
        block_drops_.
            load();


    stats.block_underflows =
        block_underflows_.
            load();


    stats.prebuffer_events =
        prebuffer_events_.
            load();


    stats.source_sample_rate =
        source_sample_rate_.
            load();


    stats.connected =
        connected_.
            load();


    stats.streaming =
        streaming_.
            load();


    stats.playback_started =
        playback_started_;


    if (
        output_stream_ !=
        nullptr
    ) {

        stats.output_buffer_bytes =
            xStreamBufferBytesAvailable(
                output_stream_
            );
    }
    else {

        stats.output_buffer_bytes =
            0;
    }
}


void A2dpSink::
reset_interval_stats()
{
    callback_packets_.
        store(
            0
        );


    callback_bytes_.
        store(
            0
        );


    callback_drops_.
        store(
            0
        );


    blocks_produced_.
        store(
            0
        );


    block_drops_.
        store(
            0
        );


    block_underflows_.
        store(
            0
        );


    prebuffer_events_.
        store(
            0
        );
}


}
}