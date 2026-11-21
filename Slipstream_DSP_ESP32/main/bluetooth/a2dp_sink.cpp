#include "a2dp_sink.hpp"

#include <algorithm>
#include <cstring>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"

#include "nvs_flash.h"


namespace {

constexpr const char *TAG =
    "a2dp_sink";


constexpr uint32_t Q16_ONE =
    65536;


/*
 * A decoded A2DP PCM frame is:
 *
 * int16 left
 * int16 right
 *
 * = 4 bytes.
 */
constexpr size_t INPUT_FRAME_BYTES =
    4;


}


namespace slipstream {
namespace bluetooth {


A2dpSink *
A2dpSink::instance_ =
    nullptr;


/*
 * ============================================================
 * CONSTRUCTION
 * ============================================================
 */

A2dpSink::A2dpSink()
{
}


/*
 * ============================================================
 * INITIALIZATION
 * ============================================================
 */

bool A2dpSink::init()
{
    if (
        initialized_
    ) {

        return true;
    }


    /*
     * No dynamic StreamBuffer allocations are needed anymore.
     *
     * Both PCM rings are fixed members of A2dpSink.
     */

    input_write_bytes_.store(
        0
    );

    input_read_bytes_.store(
        0
    );

    output_write_blocks_.store(
        0
    );

    output_read_blocks_.store(
        0
    );


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
     * --------------------------------------------------------
     * RELEASE BLE MEMORY
     * --------------------------------------------------------
     *
     * Slipstream H19 only needs Bluetooth Classic.
     */

    const esp_err_t release_result =
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
     * CONTROLLER
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
     * A2DP
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


    /*
     * --------------------------------------------------------
     * PCM WORKER
     * --------------------------------------------------------
     */

    const BaseType_t task_result =
        xTaskCreatePinnedToCore(
            worker_task_entry,
            "bt_pcm_worker",
            WORKER_STACK,
            this,
            WORKER_PRIORITY,
            &worker_task_handle_,
            1
        );


    if (
        task_result !=
        pdPASS
    ) {

        worker_task_handle_ =
            nullptr;


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
        "Raw BT input ring: %u bytes",
        static_cast<unsigned>(
            INPUT_RING_BYTES
        )
    );


    ESP_LOGI(
        TAG,
        "BT output ring: %u blocks / %u ms",
        static_cast<unsigned>(
            OUTPUT_RING_BLOCKS
        ),
        static_cast<unsigned>(
            OUTPUT_RING_BLOCKS * 10
        )
    );


    ESP_LOGI(
        TAG,
        "BT target cushion: %u blocks / %u ms",
        static_cast<unsigned>(
            OUTPUT_TARGET_BLOCKS
        ),
        static_cast<unsigned>(
            OUTPUT_TARGET_BLOCKS * 10
        )
    );


    return true;
}


/*
 * ============================================================
 * EVENT CALLBACK
 * ============================================================
 */

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

        /*
         * ----------------------------------------------------
         * CONNECTION
         * ----------------------------------------------------
         */

        case
            ESP_A2D_CONNECTION_STATE_EVT:
        {

            const bool is_connected =
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
                    is_connected,
                    std::memory_order_release
                );


            if (
                !is_connected
            ) {

                instance_->
                    streaming_.
                    store(
                        false,
                        std::memory_order_release
                    );


                instance_->
                    worker_reset_requested_.
                    store(
                        true,
                        std::memory_order_release
                    );


                instance_->
                    wake_worker();
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


        /*
         * ----------------------------------------------------
         * PLAY / STOP
         * ----------------------------------------------------
         */

        case
            ESP_A2D_AUDIO_STATE_EVT:
        {

            const bool is_streaming =
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
                    is_streaming,
                    std::memory_order_release
                );


            if (
                !is_streaming
            ) {

                instance_->
                    worker_reset_requested_.
                    store(
                        true,
                        std::memory_order_release
                    );
            }


            instance_->
                wake_worker();


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


        /*
         * ----------------------------------------------------
         * FORMAT
         * ----------------------------------------------------
         */

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

                const uint8_t frequency =
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
                    sample_rate,
                    std::memory_order_release
                );


            /*
             * New stream format means old interpolation state
             * must not be reused.
             */

            instance_->
                worker_reset_requested_.
                store(
                    true,
                    std::memory_order_release
                );


            instance_->
                wake_worker();


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


/*
 * ============================================================
 * PCM CALLBACK
 * ============================================================
 */

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
            0
    ) {

        return;
    }


    instance_->
        callback_packets_.
        fetch_add(
            1,
            std::memory_order_relaxed
        );


    instance_->
        callback_bytes_.
        fetch_add(
            length,
            std::memory_order_relaxed
        );


    /*
     * This callback is part of the Bluetooth host path.
     *
     * It must:
     *
     * - never block
     * - never resample
     * - never mix
     * - never wait for the audio task
     *
     * It now performs only:
     *
     * memcpy -> ring
     * task notification
     */

    if (
        !instance_->
            enqueue_input_pcm(
                data,
                static_cast<size_t>(
                    length
                )
            )
    ) {

        instance_->
            callback_drops_.
            fetch_add(
                1,
                std::memory_order_relaxed
            );


        return;
    }


    instance_->
        wake_worker();
}


/*
 * ============================================================
 * TASK NOTIFICATION
 * ============================================================
 */

void A2dpSink::wake_worker()
{
    const TaskHandle_t handle =
        worker_task_handle_;


    if (
        handle !=
        nullptr
    ) {

        xTaskNotifyGive(
            handle
        );
    }
}


/*
 * ============================================================
 * INPUT RING
 * ============================================================
 */

size_t A2dpSink::input_bytes_available() const
{
    const uint32_t write =
        input_write_bytes_.
            load(
                std::memory_order_acquire
            );


    const uint32_t read =
        input_read_bytes_.
            load(
                std::memory_order_acquire
            );


    const uint32_t used =
        write -
        read;


    if (
        used >
        INPUT_RING_BYTES
    ) {

        return
            INPUT_RING_BYTES;
    }


    return
        static_cast<size_t>(
            used
        );
}


size_t A2dpSink::input_free_bytes() const
{
    const size_t used =
        input_bytes_available();


    return
        INPUT_RING_BYTES -
        used;
}


bool A2dpSink::enqueue_input_pcm(
    const uint8_t *data,
    size_t length
)
{
    if (
        data ==
            nullptr ||
        length ==
            0
    ) {

        return false;
    }


    /*
     * Never admit data that does not contain complete stereo
     * PCM16 frames.
     */

    if (
        (
            length %
            INPUT_FRAME_BYTES
        )
        !=
        0
    ) {

        return false;
    }


    /*
     * A single callback should never be larger than our whole
     * queue. If it is, throwing it away is safer than breaking
     * PCM framing.
     */

    if (
        length >
        INPUT_RING_BYTES
    ) {

        return false;
    }


    const uint32_t write =
        input_write_bytes_.
            load(
                std::memory_order_relaxed
            );


    const uint32_t read =
        input_read_bytes_.
            load(
                std::memory_order_acquire
            );


    const uint32_t used =
        write -
        read;


    if (
        used >
        INPUT_RING_BYTES
    ) {

        return false;
    }


    const size_t free_bytes =
        INPUT_RING_BYTES -
        static_cast<size_t>(
            used
        );


    /*
     * Whole-callback policy.
     *
     * We never partially write an A2DP callback.
     *
     * A partial write could create a discontinuity inside a
     * decoder burst and was one likely contributor to the
     * earlier grain/static behavior.
     */

    if (
        free_bytes <
        length
    ) {

        return false;
    }


    const size_t write_index =
        static_cast<size_t>(
            write %
            INPUT_RING_BYTES
        );


    const size_t first_bytes =
        std::min(
            length,
            INPUT_RING_BYTES -
            write_index
        );


    std::memcpy(
        &input_ring_[
            write_index
        ],
        data,
        first_bytes
    );


    const size_t second_bytes =
        length -
        first_bytes;


    if (
        second_bytes >
        0
    ) {

        std::memcpy(
            &input_ring_[0],
            data +
                first_bytes,
            second_bytes
        );
    }


    /*
     * Publish only after all PCM bytes are safely copied.
     */

    input_write_bytes_.
        store(
            write +
            static_cast<uint32_t>(
                length
            ),
            std::memory_order_release
        );


    return true;
}


void A2dpSink::flush_input_ring()
{
    const uint32_t write =
        input_write_bytes_.
            load(
                std::memory_order_acquire
            );


    input_read_bytes_.
        store(
            write,
            std::memory_order_release
        );
}


/*
 * ============================================================
 * OUTPUT RING
 * ============================================================
 */

size_t A2dpSink::output_blocks_available() const
{
    const uint32_t write =
        output_write_blocks_.
            load(
                std::memory_order_acquire
            );


    const uint32_t read =
        output_read_blocks_.
            load(
                std::memory_order_acquire
            );


    const uint32_t available =
        write -
        read;


    if (
        available >
        OUTPUT_RING_BLOCKS
    ) {

        return
            OUTPUT_RING_BLOCKS;
    }


    return
        static_cast<size_t>(
            available
        );
}


void A2dpSink::flush_output_ring()
{
    const uint32_t write =
        output_write_blocks_.
            load(
                std::memory_order_acquire
            );


    output_read_blocks_.
        store(
            write,
            std::memory_order_release
        );
}


/*
 * ============================================================
 * WORKER
 * ============================================================
 */

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


    while (
        true
    ) {

        /*
         * Sleep until either:
         *
         * - PCM arrives,
         * - output audio consumes a block,
         * - Bluetooth state changes.
         */

        ulTaskNotifyTake(
            pdTRUE,
            portMAX_DELAY
        );


        if (
            worker_reset_requested_.
                exchange(
                    false,
                    std::memory_order_acq_rel
                )
        ) {

            reset_worker_stream_state();
        }


        /*
         * Do not waste CPU transforming stale data while A2DP
         * is not actively playing.
         */

        if (
            !streaming_.
                load(
                    std::memory_order_acquire
                )
        ) {

            flush_input_ring();


            continue;
        }


        /*
         * Drain as much input as useful in ONE worker wake.
         *
         * The old StreamBuffer implementation woke, copied
         * 1024 bytes, processed it, slept, then repeated.
         *
         * This version directly processes PCM from the ring and
         * keeps going until:
         *
         * - input is empty, or
         * - the output ring has reached its target cushion.
         */

        while (
            streaming_.
                load(
                    std::memory_order_acquire
                )
        ) {

            /*
             * Keep roughly 30 ms of completed 48 kHz output
             * ready for the audio task.
             *
             * If that target is already satisfied, leave newer
             * Bluetooth PCM in the larger raw input ring.
             */

            if (
                output_blocks_available()
                >=
                OUTPUT_TARGET_BLOCKS
            ) {

                break;
            }


            const uint32_t read =
                input_read_bytes_.
                    load(
                        std::memory_order_relaxed
                    );


            const uint32_t write =
                input_write_bytes_.
                    load(
                        std::memory_order_acquire
                    );


            uint32_t available =
                write -
                read;


            if (
                available <
                INPUT_FRAME_BYTES
            ) {

                break;
            }


            if (
                available >
                INPUT_RING_BYTES
            ) {

                /*
                 * Defensive recovery. This should not occur
                 * with the SPSC write policy.
                 */

                input_read_bytes_.
                    store(
                        write,
                        std::memory_order_release
                    );


                reset_resampler(
                    source_sample_rate_.
                        load(
                            std::memory_order_acquire
                        )
                );


                break;
            }


            const size_t read_index =
                static_cast<size_t>(
                    read %
                    INPUT_RING_BYTES
                );


            size_t contiguous =
                std::min(
                    static_cast<size_t>(
                        available
                    ),
                    INPUT_RING_BYTES -
                    read_index
                );


            /*
             * Never process less than a complete stereo frame.
             */

            contiguous &=
                ~static_cast<size_t>(
                    INPUT_FRAME_BYTES -
                    1
                );


            if (
                contiguous <
                INPUT_FRAME_BYTES
            ) {

                break;
            }


            /*
             * Limit one inner iteration to about 10 ms of
             * SOURCE audio.
             *
             * This gives process_pcm() roughly one destination
             * block at a time rather than allowing one huge
             * A2DP burst to overflow the four-block output ring.
             */

            const uint32_t source_rate =
                source_sample_rate_.
                    load(
                        std::memory_order_acquire
                    );


            size_t source_frames_per_10ms =
                static_cast<size_t>(
                    source_rate /
                    100
                );


            if (
                source_frames_per_10ms <
                1
            ) {

                source_frames_per_10ms =
                    1;
            }


            size_t preferred_bytes =
                source_frames_per_10ms *
                INPUT_FRAME_BYTES;


            preferred_bytes &=
                ~static_cast<size_t>(
                    INPUT_FRAME_BYTES -
                    1
                );


            if (
                preferred_bytes >=
                INPUT_FRAME_BYTES
            ) {

                contiguous =
                    std::min(
                        contiguous,
                        preferred_bytes
                    );
            }


            process_pcm(
                &input_ring_[
                    read_index
                ],
                contiguous
            );


            /*
             * Publish consumption only after the resampler has
             * finished reading this memory.
             */

            input_read_bytes_.
                store(
                    read +
                    static_cast<uint32_t>(
                        contiguous
                    ),
                    std::memory_order_release
                );
        }
    }
}


void A2dpSink::reset_worker_stream_state()
{
    flush_input_ring();


    resampler_ready_ =
        false;


    resampler_source_rate_ =
        0;


    source_step_q16_ =
        0;


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
}


/*
 * ============================================================
 * RESAMPLER
 * ============================================================
 */

void A2dpSink::reset_resampler(
    uint32_t source_rate
)
{
    resampler_ready_ =
        false;


    resampler_source_rate_ =
        source_rate;


    if (
        source_rate ==
        0
    ) {

        source_step_q16_ =
            0;


        return;
    }


    /*
     * Input position advanced for every 48 kHz output sample:
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
    const int32_t previous32 =
        static_cast<int32_t>(
            previous
        );


    const int32_t current32 =
        static_cast<int32_t>(
            current
        );


    const int32_t delta =
        current32 -
        previous32;


    const int64_t scaled =
        static_cast<int64_t>(
            delta
        )
        *
        static_cast<int64_t>(
            fraction_q16
        );


    int32_t value =
        previous32 +
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
    else if (
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
            INPUT_FRAME_BYTES
    ) {

        return;
    }


    length -=
        length %
        INPUT_FRAME_BYTES;


    if (
        length ==
        0
    ) {

        return;
    }


    const uint32_t source_rate =
        source_sample_rate_.
            load(
                std::memory_order_acquire
            );


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
        INPUT_FRAME_BYTES;


    /*
     * input_ring_ is 4-byte aligned and all read offsets are
     * multiples of four, so this cast remains aligned.
     */

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

        const int16_t current_left =
            samples[
                frame * 2
            ];


        const int16_t current_right =
            samples[
                frame * 2 +
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


        const uint64_t current_position_q16 =
            input_frame_index_
            <<
            16;


        const uint64_t previous_position_q16 =
            (
                input_frame_index_ -
                1
            )
            <<
            16;


        while (
            next_output_position_q16_
            <=
            current_position_q16
        ) {

            uint64_t fraction =
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


/*
 * ============================================================
 * OUTPUT PRODUCTION
 * ============================================================
 */

void A2dpSink::push_output_sample(
    int16_t left,
    int16_t right
)
{
    const size_t sample_index =
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


void A2dpSink::push_completed_output_block()
{
    const uint32_t write =
        output_write_blocks_.
            load(
                std::memory_order_relaxed
            );


    const uint32_t read =
        output_read_blocks_.
            load(
                std::memory_order_acquire
            );


    const uint32_t used =
        write -
        read;


    if (
        used >=
        OUTPUT_RING_BLOCKS
    ) {

        /*
         * This should be rare because the worker normally stops
         * at OUTPUT_TARGET_BLOCKS=3 while capacity is 4.
         */

        block_drops_.
            fetch_add(
                1,
                std::memory_order_relaxed
            );


        return;
    }


    const size_t slot =
        static_cast<size_t>(
            write %
            OUTPUT_RING_BLOCKS
        );


    std::memcpy(
        output_ring_[
            slot
        ],
        output_block_,
        OUTPUT_BYTES_PER_BLOCK
    );


    /*
     * Publish block after copy completes.
     */

    output_write_blocks_.
        store(
            write + 1,
            std::memory_order_release
        );


    blocks_produced_.
        fetch_add(
            1,
            std::memory_order_relaxed
        );
}


/*
 * ============================================================
 * UNDERFLOW CONCEALMENT
 * ============================================================
 */

void A2dpSink::create_underflow_fade(
    int16_t *output
)
{
    if (
        output ==
            nullptr
    ) {

        return;
    }


    /*
     * A hard transition from arbitrary music amplitude to an
     * all-zero 10 ms block sounds like a click / grain.
     *
     * Instead, ramp the final known stereo sample to zero over
     * the whole missing block.
     */

    if (
        !have_last_output_sample_
    ) {

        std::memset(
            output,
            0,
            OUTPUT_BYTES_PER_BLOCK
        );


        fade_in_next_block_ =
            true;


        return;
    }


    for (
        size_t frame = 0;
        frame < OUTPUT_FRAMES_PER_BLOCK;
        ++frame
    ) {

        const int32_t remaining =
            static_cast<int32_t>(
                OUTPUT_FRAMES_PER_BLOCK -
                frame
            );


        const int32_t denominator =
            static_cast<int32_t>(
                OUTPUT_FRAMES_PER_BLOCK
            );


        const int32_t left =
            (
                static_cast<int32_t>(
                    last_output_left_
                )
                *
                remaining
            )
            /
            denominator;


        const int32_t right =
            (
                static_cast<int32_t>(
                    last_output_right_
                )
                *
                remaining
            )
            /
            denominator;


        output[
            frame * 2
        ] =
            static_cast<int16_t>(
                left
            );


        output[
            frame * 2 + 1
        ] =
            static_cast<int16_t>(
                right
            );
    }


    last_output_left_ =
        0;


    last_output_right_ =
        0;


    fade_in_next_block_ =
        true;
}


void A2dpSink::apply_fade_in(
    int16_t *output
)
{
    if (
        output ==
            nullptr ||
        !fade_in_next_block_
    ) {

        return;
    }


    const size_t fade_frames =
        std::min(
            FADE_IN_FRAMES,
            OUTPUT_FRAMES_PER_BLOCK
        );


    for (
        size_t frame = 0;
        frame < fade_frames;
        ++frame
    ) {

        const int32_t numerator =
            static_cast<int32_t>(
                frame + 1
            );


        const int32_t denominator =
            static_cast<int32_t>(
                fade_frames
            );


        const size_t left_index =
            frame * 2;


        const size_t right_index =
            left_index + 1;


        output[
            left_index
        ] =
            static_cast<int16_t>(
                (
                    static_cast<int32_t>(
                        output[
                            left_index
                        ]
                    )
                    *
                    numerator
                )
                /
                denominator
            );


        output[
            right_index
        ] =
            static_cast<int16_t>(
                (
                    static_cast<int32_t>(
                        output[
                            right_index
                        ]
                    )
                    *
                    numerator
                )
                /
                denominator
            );
    }


    fade_in_next_block_ =
        false;
}


/*
 * ============================================================
 * AUDIO CONSUMER
 * ============================================================
 */

bool A2dpSink::read_block(
    int16_t *output
)
{
    if (
        output ==
            nullptr
    ) {

        return false;
    }


    /*
     * --------------------------------------------------------
     * NOT PLAYING
     * --------------------------------------------------------
     */

    if (
        !streaming_.
            load(
                std::memory_order_acquire
            )
    ) {

        playback_started_ =
            false;


        consecutive_underflows_ =
            0;


        have_last_output_sample_ =
            false;


        last_output_left_ =
            0;


        last_output_right_ =
            0;


        fade_in_next_block_ =
            false;


        flush_output_ring();


        return false;
    }


    size_t available =
        output_blocks_available();


    /*
     * --------------------------------------------------------
     * START / REBUFFER
     * --------------------------------------------------------
     */

    if (
        !playback_started_
    ) {

        if (
            available <
            PREBUFFER_BLOCKS
        ) {

            /*
             * Ask worker to continue filling in case it paused
             * waiting for output demand.
             */

            wake_worker();


            return false;
        }


        playback_started_ =
            true;


        consecutive_underflows_ =
            0;


        prebuffer_events_.
            fetch_add(
                1,
                std::memory_order_relaxed
            );
    }


    /*
     * --------------------------------------------------------
     * UNDERFLOW
     * --------------------------------------------------------
     */

    available =
        output_blocks_available();


    if (
        available <
        1
    ) {

        block_underflows_.
            fetch_add(
                1,
                std::memory_order_relaxed
            );


        consecutive_underflows_++;


        /*
         * Instead of returning false and allowing H19 to insert
         * a hard 10 ms all-zero block, provide a click-free
         * concealment block.
         */

        create_underflow_fade(
            output
        );


        /*
         * Wake worker immediately. It may have raw PCM waiting
         * in the larger input ring.
         */

        wake_worker();


        if (
            consecutive_underflows_
            >=
            MAX_CONSECUTIVE_UNDERFLOWS
        ) {

            playback_started_ =
                false;


            consecutive_underflows_ =
                0;
        }


        /*
         * Return true because output now contains a valid
         * concealment block.
         *
         * btUF telemetry still records the real starvation.
         */

        return true;
    }


    /*
     * --------------------------------------------------------
     * NORMAL READ
     * --------------------------------------------------------
     */

    const uint32_t read =
        output_read_blocks_.
            load(
                std::memory_order_relaxed
            );


    const uint32_t write =
        output_write_blocks_.
            load(
                std::memory_order_acquire
            );


    if (
        write ==
        read
    ) {

        /*
         * Extremely small race with producer publication.
         * Treat exactly like an underflow.
         */

        block_underflows_.
            fetch_add(
                1,
                std::memory_order_relaxed
            );


        consecutive_underflows_++;


        create_underflow_fade(
            output
        );


        wake_worker();


        if (
            consecutive_underflows_
            >=
            MAX_CONSECUTIVE_UNDERFLOWS
        ) {

            playback_started_ =
                false;


            consecutive_underflows_ =
                0;
        }


        return true;
    }


    const size_t slot =
        static_cast<size_t>(
            read %
            OUTPUT_RING_BLOCKS
        );


    std::memcpy(
        output,
        output_ring_[
            slot
        ],
        OUTPUT_BYTES_PER_BLOCK
    );


    /*
     * Release the slot only after the entire block has been
     * copied.
     */

    output_read_blocks_.
        store(
            read + 1,
            std::memory_order_release
        );


    /*
     * If the previous 10 ms period had to be concealed,
     * gently fade this real block back in.
     */

    apply_fade_in(
        output
    );


    last_output_left_ =
        output[
            OUTPUT_SAMPLES_PER_BLOCK -
            2
        ];


    last_output_right_ =
        output[
            OUTPUT_SAMPLES_PER_BLOCK -
            1
        ];


    have_last_output_sample_ =
        true;


    consecutive_underflows_ =
        0;


    /*
     * The consumer just freed one output slot. Wake the worker
     * so it can promptly refill the target 30 ms cushion.
     */

    wake_worker();


    return true;
}


/*
 * ============================================================
 * PUBLIC STATE
 * ============================================================
 */

bool A2dpSink::connected() const
{
    return
        connected_.
            load(
                std::memory_order_acquire
            );
}


bool A2dpSink::streaming() const
{
    return
        streaming_.
            load(
                std::memory_order_acquire
            );
}


uint32_t A2dpSink::source_sample_rate() const
{
    return
        source_sample_rate_.
            load(
                std::memory_order_acquire
            );
}


/*
 * ============================================================
 * TELEMETRY
 * ============================================================
 */

void A2dpSink::get_stats(
    Stats &stats
) const
{
    stats.callback_packets =
        callback_packets_.
            load(
                std::memory_order_relaxed
            );


    stats.callback_bytes =
        callback_bytes_.
            load(
                std::memory_order_relaxed
            );


    stats.callback_drops =
        callback_drops_.
            load(
                std::memory_order_relaxed
            );


    stats.blocks_produced =
        blocks_produced_.
            load(
                std::memory_order_relaxed
            );


    stats.block_drops =
        block_drops_.
            load(
                std::memory_order_relaxed
            );


    stats.block_underflows =
        block_underflows_.
            load(
                std::memory_order_relaxed
            );


    stats.prebuffer_events =
        prebuffer_events_.
            load(
                std::memory_order_relaxed
            );


    stats.source_sample_rate =
        source_sample_rate_.
            load(
                std::memory_order_acquire
            );


    stats.connected =
        connected_.
            load(
                std::memory_order_acquire
            );


    stats.streaming =
        streaming_.
            load(
                std::memory_order_acquire
            );


    stats.playback_started =
        playback_started_;


    stats.output_buffer_bytes =
        output_blocks_available()
        *
        OUTPUT_BYTES_PER_BLOCK;
}


void A2dpSink::reset_interval_stats()
{
    callback_packets_.
        store(
            0,
            std::memory_order_relaxed
        );


    callback_bytes_.
        store(
            0,
            std::memory_order_relaxed
        );


    callback_drops_.
        store(
            0,
            std::memory_order_relaxed
        );


    blocks_produced_.
        store(
            0,
            std::memory_order_relaxed
        );


    block_drops_.
        store(
            0,
            std::memory_order_relaxed
        );


    block_underflows_.
        store(
            0,
            std::memory_order_relaxed
        );


    prebuffer_events_.
        store(
            0,
            std::memory_order_relaxed
        );
}


}
}